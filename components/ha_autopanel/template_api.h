// template_api.h: thin wrapper around Home Assistant's
// /api/template REST endpoint and the render_template
// WebSocket subscription. The point of this class is to
// centralize the "give me a string from HA, formatted on
// HA's side" pattern so the rest of the component doesn't
// have to hand-build {"template": "..."} JSON bodies, manage
// Bearer auth, or hand-roll render_template WebSocket
// subscriptions.
//
// Two usage patterns:
//
// 1. ONE-SHOT: render() and render_with_vars() POST to
//    <base>/api/template and return the rendered string.
//    Used for initial fetches (weather, home name, per-area
//    aggregate).
//
// 2. PUSH: subscribe() sends a render_template WebSocket
//    message and registers a callback that fires whenever
//    HA re-evaluates the template. HA's template engine
//    re-evaluates time-aware templates like {{ now() }}
//    once per minute, so the clock subscription is the
//    canonical example. The callback runs on the WS
//    parse_task - thread-safe callback functions only.
//
// Template bodies are written as R"DELIM(... )DELIM" raw
// strings in the C++ call site, so the user can paste
// Jinja2 with no JSON escaping. Example:
//
//   std::string t = R"DELIM({{ now().strftime('%-H:%M') }})DELIM";
//   api.render(t);                          // one-shot
//   api.subscribe(t, [](std::string s){...});  // push
//
// Header-only inline, matching the http_client.h style.
// The build system only compiles the component's main .cpp,
// so inline methods don't bloat other translation units.

#pragma once

#include <atomic>
#include <functional>
#include <map>
#include <mutex>
#include <optional>
#include <shared_mutex>
#include <string>
#include <vector>

#include "esp_heap_caps.h"
#include "esphome/components/http_request/http_request.h"
#include "esphome/core/application.h"
#include "esphome/core/log.h"

namespace esphome {
namespace ha_autopanel {

#ifndef HA_TPL_TAG
#define HA_TPL_TAG "ha_tpl"
#endif

class TemplateApi {
 public:
  TemplateApi(http_request::HttpRequestComponent* http,
              const std::string& base_url,
              const std::string& token)
      : http_(http), base_url_(base_url), token_(token) {}

  // Mutexes are not copyable / not movable, so explicitly
  // delete the copy operations. The class is meant to be
  // constructed in place and live for the component lifetime.
  TemplateApi(const TemplateApi&) = delete;
  TemplateApi& operator=(const TemplateApi&) = delete;
  TemplateApi(TemplateApi&&) = delete;
  TemplateApi& operator=(TemplateApi&&) = delete;

  // Update URL/token at runtime (after /autopanel/setup saves
  // a new configuration). Render and subscribe calls from
  // this point on use the new values.
  void update(const std::string& base_url, const std::string& token) {
    std::lock_guard<std::mutex> lk(write_mu_);
    this->base_url_ = base_url;
    this->token_ = token;
  }

  // Set the WS-send callback. The callback is invoked from
  // subscribe()/unsubscribe() to send raw JSON text over
  // the WebSocket connection. The callback returns true on
  // success, false on failure. Typically wired to
  // HaWsClient::send_text_external in start_discovery_().
  // Using a std::function (instead of a HaWsClient* pointer)
  // keeps template_api.h free of ha_ws_client.h's include
  // requirements - the latter requires HaAutoPanel to be
  // complete.
  void set_ws_sender(std::function<bool(const std::string&)> sender) {
    std::lock_guard<std::mutex> lk(write_mu_);
    this->ws_sender_ = std::move(sender);
  }

  // v1.27: set the HTTP component used for the one-shot
  // /api/template REST calls. Called from setup() AFTER
  // set_http_request() has wired http_request_ (the
  // constructor's http_ pointer is captured too early
  // for it to be useful - the in-class initializer for
  // template_api_ runs before YAML's set_http_request
  // call).
  void set_http(http_request::HttpRequestComponent* http) {
    std::lock_guard<std::mutex> lk(write_mu_);
    this->http_ = http;
  }

  // One-shot render. POST {base}/api/template with
  // {"template": "<tmpl>"} + Bearer auth. Returns the bare
  // rendered string (HA returns text/plain for string
  // templates). Empty optional on HTTP error / connect fail.
  //
  // The returned body is a string - for structured responses
  // (e.g. the per-area aggregate), the caller parses with
  // ArduinoJson.
  std::optional<std::string> render(const std::string& tmpl) {
    return this->do_post_(tmpl, "");
  }

  // One-shot render with variables. POST body is
  // {"template": "<tmpl>", "variables": <json_object>}.
  // The variables_json argument is already a valid JSON
  // object string (e.g. `{"included_areas": ["kitchen"]}`).
  std::optional<std::string> render_with_vars(const std::string& tmpl,
                                              const std::string& variables_json) {
    return this->do_post_(tmpl, variables_json);
  }

  // Push subscription via WebSocket render_template. Sends
  // {"id":N,"type":"render_template","template":"<tmpl>"} and
  // registers cb to fire on every event for that id. Returns
  // the sub id (>0) or 0 on failure.
  //
  // The callback runs on the WS parse_task. If the callback
  // needs to touch LVGL or any loopTask-only data, the
  // caller is responsible for queueing the event to a
  // loopTask-drained queue (same pattern as StateEvent).
  uint32_t subscribe(const std::string& tmpl,
                     std::function<void(const std::string&)> cb) {
    return this->subscribe_impl_(tmpl, "", std::move(cb));
  }

  // v1.27: subscribe with render-time variables. The variables
  // argument is a pre-serialized JSON object string (e.g.
  // `{"included_areas": ["kitchen"]}`). The subscribe body
  // becomes:
  //   {"id":N,"type":"render_template","template":"...",
  //    "variables":<vars>}
  // HA snapshots the variables at subscribe time. State
  // changes to any entity referenced in the template trigger
  // re-evaluation. Useful for the per-area aggregate: pass
  // the panel's area list as `included_areas` so the
  // template only iterates over our areas.
  uint32_t subscribe_with_vars(const std::string& tmpl,
                               const std::string& variables_json,
                               std::function<void(const std::string&)> cb) {
    return this->subscribe_impl_(tmpl, variables_json, std::move(cb));
  }

  // Cancel a subscription. Sends
  // {"id":M,"type":"unsubscribe_events","subscription":N}
  // and removes the local callback.
  bool unsubscribe(uint32_t id) {
    if (id == 0) return false;
    {
      std::unique_lock<std::shared_mutex> lk(subs_mu_);
      this->subs_.erase(id);
    }
    std::function<bool(const std::string&)> sender;
    {
      std::lock_guard<std::mutex> lk(write_mu_);
      sender = this->ws_sender_;
    }
    if (!sender) return false;
    uint32_t req_id = this->next_sub_id_.fetch_add(1, std::memory_order_relaxed);
    if (req_id == 0) req_id = this->next_sub_id_.fetch_add(1, std::memory_order_relaxed);
    char body[96];
    snprintf(body, sizeof(body),
             "{\"id\":%u,\"type\":\"unsubscribe_events\",\"subscription\":%u}",
             (unsigned) req_id, (unsigned) id);
    return sender(body);
  }

  // Hook: called by HaWsClient::parse_event_message_ when an
  // event arrives with a tracked id. Dispatches to the
  // matching callback. Thread: WS parse_task. The callback
  // must be safe to invoke there.
  void on_ws_event_(uint32_t id, const char* result) {
    if (id == 0 || result == nullptr) return;
    std::function<void(const std::string&)> cb;
    {
      std::shared_lock<std::shared_mutex> lk(subs_mu_);
      auto it = this->subs_.find(id);
      if (it == this->subs_.end()) return;
      cb = it->second;  // copy under lock
    }
    cb(std::string(result));
  }

  // Clear all subscriptions (called on WS disconnect so we
  // don't fire stale callbacks into a re-initialized world).
  void clear_subs_() {
    std::unique_lock<std::shared_mutex> lk(subs_mu_);
    this->subs_.clear();
  }

 private:
  // Shared implementation for subscribe() and
  // subscribe_with_vars(). variables_json is "" for plain
  // subscribe; otherwise it's a JSON object string to
  // include in the body.
  uint32_t subscribe_impl_(const std::string& tmpl,
                           const std::string& variables_json,
                           std::function<void(const std::string&)> cb) {
    if (cb == nullptr) {
      ESP_LOGW(HA_TPL_TAG, "subscribe: cb is null");
      return 0;
    }
    std::function<bool(const std::string&)> sender;
    {
      std::lock_guard<std::mutex> lk(write_mu_);
      sender = this->ws_sender_;
    }
    if (!sender) {
      ESP_LOGW(HA_TPL_TAG, "subscribe: ws_sender not set (WS not ready)");
      return 0;
    }
    uint32_t id = this->next_sub_id_.fetch_add(1, std::memory_order_relaxed);
    if (id == 0) {
      id = this->next_sub_id_.fetch_add(1, std::memory_order_relaxed);
    }
    // Register the callback BEFORE sending so the parse_task
    // can dispatch immediately on the first event.
    {
      std::unique_lock<std::shared_mutex> lk(subs_mu_);
      this->subs_[id] = std::move(cb);
    }
    // Build the JSON body:
    //   {"id":N,"type":"render_template","template":"<tmpl>"[,"variables":<vars>]}
    // Escape \ and " in the template body for valid JSON.
    std::string esc;
    esc.reserve(tmpl.size() + 32);
    for (char c : tmpl) {
      if (c == '\\' || c == '"') esc.push_back('\\');
      esc.push_back(c);
    }
    char header[64];
    snprintf(header, sizeof(header),
             "{\"id\":%u,\"type\":\"render_template\",\"template\":\"", (unsigned) id);
    std::string body = header + esc + "\"";
    if (!variables_json.empty()) {
      body += ",\"variables\":";
      body += variables_json;
    }
    body += "}";
    if (!sender(body)) {
      ESP_LOGW(HA_TPL_TAG, "subscribe id=%u: send failed", (unsigned) id);
      std::unique_lock<std::shared_mutex> lk(subs_mu_);
      this->subs_.erase(id);
      return 0;
    }
    ESP_LOGI(HA_TPL_TAG, "subscribed id=%u (%zu bytes)",
             (unsigned) id, body.size());
    return id;
  }

  // One-shot POST to {base}/api/template. The variables_json
  // One-shot POST to {base}/api/template. The variables_json
  // is "" for a plain render, or a JSON object for render_with_vars.
  std::optional<std::string> do_post_(const std::string& tmpl,
                                      const std::string& variables_json) {
    static const char* TAG = HA_TPL_TAG;
    if (this->http_ == nullptr) return std::nullopt;
    std::string base_url, token;
    {
      std::lock_guard<std::mutex> lk(write_mu_);
      base_url = this->base_url_;
      token = this->token_;
    }
    if (base_url.empty() || token.empty()) {
      ESP_LOGW(TAG, "render: base_url or token empty");
      return std::nullopt;
    }

    // Build {"template":"<escaped>"[,"variables":<vars>]}
    std::string esc;
    esc.reserve(tmpl.size() + 32);
    for (char c : tmpl) {
      if (c == '\\' || c == '"') esc.push_back('\\');
      esc.push_back(c);
    }
    std::string body = std::string("{\"template\":\"") + esc + "\"";
    if (!variables_json.empty()) {
      body += ",\"variables\":";
      body += variables_json;
    }
    body += "}";

    std::vector<http_request::Header> headers;
    headers.push_back({"Content-Type", "application/json"});
    headers.push_back({"Authorization", "Bearer " + token});

    auto container = this->http_->post(base_url + "/api/template", body, headers);
    if (container == nullptr) {
      ESP_LOGW(TAG, "render: null container (http_request internal error)");
      return std::nullopt;
    }
    int code = container->status_code;
    if (code == 0) {
      ESP_LOGW(TAG, "render: connection failed");
      container->end();
      return std::nullopt;
    }
    if (code != 200) {
      ESP_LOGW(TAG, "render: HTTP %d", code);
      container->end();
      return std::nullopt;
    }
    std::string out;
    out.reserve(512);
    uint8_t buf[512];
    while (container->get_bytes_read() < container->content_length) {
      esphome::App.feed_wdt();
      int n = container->read(buf, sizeof(buf));
      if (n > 0) {
        out.append((char*) buf, n);
      } else if (n == http_request::HTTP_ERROR_CONNECTION_CLOSED) {
        break;
      } else {
        yield();
      }
    }
    container->end();
    return out;
  }

  http_request::HttpRequestComponent* http_;
  // WS send callback. Invoked from subscribe()/unsubscribe()
  // to push JSON over the WebSocket. Set in
  // HaAutoPanel::start_discovery_() once the WS client exists.
  // Captured by copy (under write_mu_) at send time so that a
  // re-set of the callback doesn't race with an in-flight send.
  std::function<bool(const std::string&)> ws_sender_;
  std::string base_url_;
  std::string token_;

  // Subscription registry. Read-mostly (the hot path is
  // on_ws_event_ on parse_task), so a shared_mutex.
  std::map<uint32_t, std::function<void(const std::string&)>> subs_;
  mutable std::shared_mutex subs_mu_;

  // Monotonic id allocator. Starts at 100 to avoid colliding
  // with HaWsClient's hardcoded ids (get_states=1,
  // subscribe_events state_changed=2). 0 is the failure
  // sentinel. 32-bit unsigned wraps at 4G - in practice we
  // have a few subscriptions, so this is fine.
  std::atomic<uint32_t> next_sub_id_{100};

  // write_mu_ guards the URL/token update. Read-mostly path
  // (render) snapshots under this lock; a shared_mutex would
  // be more parallel but render is called at most a few
  // times per second.
  std::mutex write_mu_;
};

}  // namespace ha_autopanel
}  // namespace esphome
