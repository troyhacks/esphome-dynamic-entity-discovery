// v1.24: ha_ws_client.cpp — implementation. Class declaration in
// ha_ws_client.h. This file is compiled by esphome alongside
// ha_autopanel.cpp (the component dir is the build unit).
//
// All method bodies here are out-of-line because they reference
// the full HaAutoPanel class definition (members like
// entities_by_area_, on_entity_state_changed_,
// on_entity_attribute_changed_). The header has only the
// declaration; the .h is includable without dragging in the
// full HaAutoPanel.

#include "ha_ws_client.h"
#include "ha_autopanel.h"

#include <ArduinoJson.h>
#include <cstring>
#include <cstdio>

namespace esphome::ha_autopanel {

// Local TwdtGuard (parallels the one in ha_autopanel.cpp:99-124).
// Parsing the 200 KB get_states response can take >5s; we
// unsubscribe the parse_task from the IDF TWDT for the duration
// of the parse and re-subscribe on scope exit.
namespace {

class HaWsTwdtGuard {
 public:
  HaWsTwdtGuard() {
    this->task_ = xTaskGetCurrentTaskHandle();
    if (esp_task_wdt_delete(this->task_) != ESP_OK) {
      this->task_ = nullptr;
    }
  }
  ~HaWsTwdtGuard() {
    if (this->task_ != nullptr) {
      esp_task_wdt_add(this->task_);
    }
  }
  HaWsTwdtGuard(const HaWsTwdtGuard&) = delete;
  HaWsTwdtGuard& operator=(const HaWsTwdtGuard&) = delete;
 private:
  TaskHandle_t task_{nullptr};
};

// PSRAM-preferring JSON document allocator (used for both the
// large get_states parse and small event messages).
class HaWsPsramAllocator : public ArduinoJson::Allocator {
 public:
  void* allocate(size_t n) override {
    return heap_caps_malloc_prefer(n, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT,
                                  MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  }
  void deallocate(void* p) override { heap_caps_free(p); }
  void* reallocate(void* p, size_t n) override {
    return heap_caps_realloc_prefer(p, n, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT,
                                    MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  }
};

}  // namespace

// ----- Lifecycle -----

void HaWsClient::start() {
  if (this->client_ != nullptr) {
    return;  // already started
  }
  if (this->http_url_.empty() || this->token_.empty()) {
    ESP_LOGW(HA_WS_TAG, "start: missing url or token, skipping");
    return;
  }
  if (!this->derive_ws_uri_()) {
    ESP_LOGE(HA_WS_TAG, "start: could not derive ws uri from %s", this->http_url_.c_str());
    return;
  }

  // Snapshot the entity_id set from parent's entities_by_area_.
  // Synchronous on loopTask before parse_task is created; no race.
  this->our_entity_ids_.clear();
  for (const auto& kv : this->parent_->entities_by_area_) {
    for (const auto& e : kv.second) {
      this->our_entity_ids_.insert(std::string(e.entity_id));
    }
  }
  ESP_LOGI(HA_WS_TAG, "start: %zu entity_ids snapshotted, connecting to %s",
           this->our_entity_ids_.size(), this->ws_uri_.c_str());

  // esp_websocket_client config
  esp_websocket_client_config_t cfg = {};
  cfg.uri               = this->ws_uri_.c_str();
  cfg.task_prio         = WS_TASK_PRIO;
  cfg.task_stack        = WS_TASK_STACK;
  cfg.buffer_size       = WS_RX_BUFFER_BYTES;
  cfg.user_context      = this;
  cfg.disable_auto_reconnect = false;
  cfg.network_timeout_ms = 10000;
  cfg.ping_interval_sec  = PING_INTERVAL_SEC;
  cfg.pingpong_timeout_sec = PING_TIMEOUT_SEC;
  cfg.transport = WEBSOCKET_TRANSPORT_OVER_TCP;

  this->client_ = esp_websocket_client_init(&cfg);
  if (this->client_ == nullptr) {
    ESP_LOGE(HA_WS_TAG, "esp_websocket_client_init failed");
    return;
  }
  esp_websocket_register_events(this->client_, WEBSOCKET_EVENT_ANY,
                                HaWsClient::ws_event_handler_, this);

  // Create parse_task. It subscribes itself to the TWDT at the
  // top of parse_task_loop_().
  BaseType_t r = xTaskCreatePinnedToCore(
      HaWsClient::parse_task_trampoline_, "ha_ws_parse",
      PARSE_TASK_STACK, this, PARSE_TASK_PRIO,
      &this->parse_task_handle_, PARSE_TASK_CORE);
  if (r != pdPASS) {
    ESP_LOGE(HA_WS_TAG, "xTaskCreatePinnedToCore(parse_task) failed: %d", r);
    esp_websocket_client_destroy(this->client_);
    this->client_ = nullptr;
    return;
  }
  this->raw_queue_.set_task_to_notify(this->parse_task_handle_);

  // Start the WS client
  this->ws_state_.store(HaWsState::CONNECTING, std::memory_order_release);
  esp_websocket_client_start(this->client_);
  ESP_LOGI(HA_WS_TAG, "start: client started, parse task created on core %d",
           PARSE_TASK_CORE);
}

void HaWsClient::stop_() {
  if (this->stopping_.exchange(true)) return;
  if (this->client_ != nullptr) {
    esp_websocket_client_stop(this->client_);
    esp_websocket_client_destroy(this->client_);
    this->client_ = nullptr;
  }
  if (this->inflight_buf_ != nullptr) {
    heap_caps_free(this->inflight_buf_);
    this->inflight_buf_ = nullptr;
    this->inflight_cap_ = 0;
    this->inflight_len_ = 0;
  }
  // Free any queued raw messages
  RawWsMessage* m;
  while ((m = this->raw_queue_.pop()) != nullptr) {
    m->release();
    this->raw_pool_.release(m);
  }
  // Free any queued state events
  StateEvent* e;
  while ((e = this->state_queue_.pop()) != nullptr) {
    e->release();
    this->state_pool_.release(e);
  }
  this->ws_state_.store(HaWsState::DISCONNECTED, std::memory_order_release);
  this->stopping_.store(false);
}

void HaWsClient::drain_state_events() {
  size_t n = 0;
  StateEvent* ev;
  while (n < STATE_DRAIN_PER_TICK && (ev = this->state_queue_.pop()) != nullptr) {
    if (ev->has_state) {
      this->parent_->on_entity_state_changed_(ev->entity_id, ev->state.c_str());
    }
    if (ev->has_brightness) {
      char buf[16];
      snprintf(buf, sizeof(buf), "%u", (unsigned) ev->brightness);
      this->parent_->on_entity_attribute_changed_(ev->entity_id, buf);
    }
    this->states_applied_.fetch_add(1, std::memory_order_relaxed);
    ev->release();
    this->state_pool_.release(ev);
    n++;
  }
}

// ----- esp_websocket_client event handler (runs on the IDF
// event loop task). Keep it fast: memcpy + queue push only. -----

void HaWsClient::ws_event_handler_(void* arg, esp_event_base_t base,
                                   int32_t event_id, void* event_data) {
  (void) base;
  HaWsClient* self = static_cast<HaWsClient*>(arg);
  if (self == nullptr || self->stopping_.load(std::memory_order_acquire)) {
    return;
  }
  switch (event_id) {
    case WEBSOCKET_EVENT_CONNECTED:
      self->on_ws_connected_();
      break;
    case WEBSOCKET_EVENT_DISCONNECTED:
      self->on_ws_disconnected_();
      break;
    case WEBSOCKET_EVENT_DATA: {
      auto* ev = static_cast<esp_websocket_event_data_t*>(event_data);
      self->on_ws_data_chunk_(ev);
      break;
    }
    case WEBSOCKET_EVENT_CLOSED:
      self->on_ws_closed_();
      break;
    case WEBSOCKET_EVENT_ERROR:
      self->on_ws_error_();
      break;
    default:
      break;
  }
}

void HaWsClient::on_ws_connected_() {
  ESP_LOGI(HA_WS_TAG, "WEBSOCKET_EVENT_CONNECTED to %s", this->ws_uri_.c_str());
  this->ws_state_.store(HaWsState::AUTH_REQUIRED, std::memory_order_release);
  // Auth will be sent when we receive auth_required from the server.
}

void HaWsClient::on_ws_disconnected_() {
  ESP_LOGW(HA_WS_TAG, "WEBSOCKET_EVENT_DISCONNECTED");
  this->ws_state_.store(HaWsState::DISCONNECTED, std::memory_order_release);
  if (this->inflight_buf_ != nullptr) {
    heap_caps_free(this->inflight_buf_);
    this->inflight_buf_ = nullptr;
    this->inflight_cap_ = 0;
    this->inflight_len_ = 0;
  }
}

void HaWsClient::on_ws_data_chunk_(const esp_websocket_event_data_t* ev) {
  if (ev == nullptr || ev->data_ptr == nullptr || ev->data_len == 0) return;
  if (ev->op_code == 0x8 || ev->op_code == 0x9 || ev->op_code == 0xA) return;
  if (ev->op_code != 0x1 && ev->op_code != 0x0 && ev->op_code != 0x2) return;

  // First chunk of a (possibly multi-chunk) message?
  if (ev->payload_offset == 0) {
    if (this->inflight_buf_ != nullptr) {
      heap_caps_free(this->inflight_buf_);
      this->inflight_buf_ = nullptr;
    }
    size_t need = ev->payload_len + 1;
    this->inflight_buf_ = psram_strdup_(need);
    if (this->inflight_buf_ == nullptr) {
      ESP_LOGE(HA_WS_TAG, "psram alloc fail for %zu bytes", need);
      this->inflight_cap_ = 0;
      this->inflight_len_ = 0;
      this->events_dropped_.fetch_add(1, std::memory_order_relaxed);
      return;
    }
    this->inflight_cap_ = need;
    this->inflight_len_ = 0;
  }
  // Append
  if (this->inflight_buf_ != nullptr &&
      this->inflight_len_ + ev->data_len <= this->inflight_cap_) {
    memcpy(this->inflight_buf_ + this->inflight_len_, ev->data_ptr, ev->data_len);
    this->inflight_len_ += ev->data_len;
  }
  // Final chunk?
  if (ev->payload_offset + ev->data_len >= ev->payload_len) {
    if (this->inflight_buf_ == nullptr) {
      this->events_dropped_.fetch_add(1, std::memory_order_relaxed);
      return;
    }
    this->inflight_buf_[this->inflight_len_] = '\0';
    RawWsMessage* msg = this->raw_pool_.allocate();
    if (msg == nullptr) {
      this->events_dropped_.fetch_add(1, std::memory_order_relaxed);
      heap_caps_free(this->inflight_buf_);
    } else {
      msg->data = this->inflight_buf_;
      msg->len = this->inflight_len_;
      msg->kind = this->classify_kind_(this->inflight_buf_, this->inflight_len_);
      msg->id = 0;
      if (!this->raw_queue_.push(msg)) {
        this->events_dropped_.fetch_add(1, std::memory_order_relaxed);
        heap_caps_free(this->inflight_buf_);
        this->raw_pool_.release(msg);
      }
    }
    this->inflight_buf_ = nullptr;
    this->inflight_cap_ = 0;
    this->inflight_len_ = 0;
  }
}

void HaWsClient::on_ws_closed_() {
  ESP_LOGI(HA_WS_TAG, "WEBSOCKET_EVENT_CLOSED");
  this->ws_state_.store(HaWsState::DISCONNECTED, std::memory_order_release);
}

void HaWsClient::on_ws_error_() {
  ESP_LOGE(HA_WS_TAG, "WEBSOCKET_EVENT_ERROR");
  this->ws_state_.store(HaWsState::DISCONNECTED, std::memory_order_release);
}

// ----- Parse task -----

void HaWsClient::parse_task_trampoline_(void* arg) {
  HaWsClient* self = static_cast<HaWsClient*>(arg);
  if (self != nullptr) self->parse_task_loop_();
  vTaskDelete(nullptr);
}

void HaWsClient::parse_task_loop_() {
  esp_err_t wdt_r = esp_task_wdt_add(NULL);
  if (wdt_r != ESP_OK) {
    ESP_LOGW(HA_WS_TAG, "parse_task: esp_task_wdt_add returned %d, TwdtGuard is a no-op", wdt_r);
  }
  ESP_LOGI(HA_WS_TAG, "parse_task: started on core %d", PARSE_TASK_CORE);

  for (;;) {
    if (this->stopping_.load(std::memory_order_acquire)) return;

    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000));
    esp_task_wdt_reset();

    RawWsMessage* msg;
    while ((msg = this->raw_queue_.pop()) != nullptr) {
      if (this->stopping_.load(std::memory_order_acquire)) {
        msg->release();
        this->raw_pool_.release(msg);
        return;
      }
      switch (msg->kind) {
        case 1:  // auth_required
          this->parse_auth_required_();
          break;
        case 2:  // auth_ok
          this->parse_auth_ok_();
          break;
        case 3:  // auth_invalid
          this->parse_auth_invalid_(msg->data, msg->len);
          break;
        case 4:  // result
          if (msg->data != nullptr && msg->data[0] == '{') {
            const char* idp = strstr(msg->data, "\"id\":");
            if (idp != nullptr) {
              int idv = atoi(idp + 5);
              msg->id = (uint32_t) idv;
            }
          }
          if (msg->id == GET_STATES_ID) {
            this->parse_get_states_result_(msg->data, msg->len);
          } else {
            ESP_LOGI(HA_WS_TAG, "result msg id=%u (%zu bytes), ignoring",
                     (unsigned) msg->id, msg->len);
          }
          break;
        case 5:  // event
          this->parse_event_message_(msg->data, msg->len);
          break;
        default:
          if (msg->data != nullptr) {
            ESP_LOGW(HA_WS_TAG, "unknown msg kind=%u (%zu bytes): %.40s...",
                     (unsigned) msg->kind, msg->len, msg->data);
          }
          break;
      }
      msg->release();
      this->raw_pool_.release(msg);
    }

    // Timeout checks
    uint32_t now_ms = (uint32_t) (esp_timer_get_time() / 1000);
    if (this->ws_state_.load(std::memory_order_acquire) == HaWsState::AUTHENTICATING &&
        this->auth_sent_ms_.load(std::memory_order_acquire) != 0 &&
        now_ms - this->auth_sent_ms_.load(std::memory_order_acquire) > AUTH_TIMEOUT_MS) {
      ESP_LOGE(HA_WS_TAG, "auth_ok timeout after %ums, forcing reconnect",
               (unsigned) (now_ms - this->auth_sent_ms_.load(std::memory_order_acquire)));
      if (this->client_ != nullptr) {
        esp_websocket_client_close(this->client_, portMAX_DELAY);
      }
    }
    if (this->ws_state_.load(std::memory_order_acquire) == HaWsState::GETTING_STATES &&
        this->subscribe_sent_ms_.load(std::memory_order_acquire) != 0 &&
        now_ms - this->subscribe_sent_ms_.load(std::memory_order_acquire) > SUBSCRIBE_TIMEOUT_MS) {
      ESP_LOGE(HA_WS_TAG, "subscribe_events ack timeout after %ums, forcing reconnect",
               (unsigned) (now_ms - this->subscribe_sent_ms_.load(std::memory_order_acquire)));
      if (this->client_ != nullptr) {
        esp_websocket_client_close(this->client_, portMAX_DELAY);
      }
    }
  }
}

void HaWsClient::parse_auth_required_() {
  ESP_LOGI(HA_WS_TAG, "auth_required received, sending auth");
  this->ws_state_.store(HaWsState::AUTHENTICATING, std::memory_order_release);
  this->auth_sent_ms_.store((uint32_t) (esp_timer_get_time() / 1000),
                            std::memory_order_release);
  this->send_auth_();
}

void HaWsClient::parse_auth_ok_() {
  ESP_LOGI(HA_WS_TAG, "auth_ok received");
  this->ws_state_.store(HaWsState::AUTHENTICATED, std::memory_order_release);
  this->auth_sent_ms_.store(0, std::memory_order_release);
  this->subscribe_sent_ms_.store((uint32_t) (esp_timer_get_time() / 1000),
                                std::memory_order_release);
  this->send_get_states_();
  this->ws_state_.store(HaWsState::GETTING_STATES, std::memory_order_release);
}

void HaWsClient::parse_auth_invalid_(const char* json, size_t len) {
  ESP_LOGE(HA_WS_TAG, "auth_invalid: %.*s", (int) len, json ? json : "(null)");
  this->ws_state_.store(HaWsState::DISCONNECTED, std::memory_order_release);
  if (this->client_ != nullptr) {
    esp_websocket_client_stop(this->client_);
  }
}

void HaWsClient::parse_get_states_result_(const char* json, size_t len) {
  if (json == nullptr || len == 0) {
    ESP_LOGW(HA_WS_TAG, "get_states result empty");
    return;
  }
  ESP_LOGI(HA_WS_TAG, "get_states result: %u bytes, parsing...", (unsigned) len);
  uint32_t t0 = (uint32_t) (esp_timer_get_time() / 1000);

  ArduinoJson::JsonDocument doc(new HaWsPsramAllocator());

  {
    HaWsTwdtGuard g;  // unsubscribe from TWDT for the long parse
    DeserializationError err = deserializeJson(doc, json, len);
    if (err) {
      ESP_LOGE(HA_WS_TAG, "get_states parse failed: %s", err.c_str());
      return;
    }
  }
  esp_task_wdt_reset();

  JsonArray results = doc["result"].as<JsonArray>();
  if (results.isNull()) {
    ESP_LOGE(HA_WS_TAG, "get_states result: no 'result' field");
    return;
  }
  uint32_t matched = 0;
  for (JsonObject entry : results) {
    const char* eid = entry["entity_id"];
    if (eid == nullptr) continue;
    if (this->our_entity_ids_.find(eid) == this->our_entity_ids_.end()) continue;
    const char* st = entry["state"] | "";
    int brightness = -1;
    bool has_brightness = false;
    JsonObject attrs = entry["attributes"].as<JsonObject>();
    if (!attrs.isNull()) {
      JsonVariant bv = attrs["brightness"];
      if (!bv.isNull()) {
        brightness = bv.as<int>();
        if (brightness < 0) brightness = 0;
        if (brightness > 255) brightness = 255;
        has_brightness = true;
      }
    }
    this->push_state_event_(eid, st, brightness, has_brightness);
    matched++;
  }
  uint32_t dt = (uint32_t) (esp_timer_get_time() / 1000) - t0;
  ESP_LOGI(HA_WS_TAG, "get_states: matched %u of %zu entities, %u ms",
           (unsigned) matched, this->our_entity_ids_.size(), (unsigned) dt);

  this->initial_states_received_.store(true, std::memory_order_release);
  this->send_subscribe_events_();
  this->ws_state_.store(HaWsState::READY, std::memory_order_release);
  this->subscribe_sent_ms_.store((uint32_t) (esp_timer_get_time() / 1000),
                                std::memory_order_release);
}

void HaWsClient::parse_event_message_(const char* json, size_t len) {
  if (json == nullptr || len == 0) return;
  this->events_received_.fetch_add(1, std::memory_order_relaxed);

  ArduinoJson::JsonDocument doc(new HaWsPsramAllocator());

  {
    HaWsTwdtGuard g;
    DeserializationError err = deserializeJson(doc, json, len);
    if (err) return;
  }

  const char* type = doc["type"] | "";
  if (strcmp(type, "event") != 0) return;
  JsonObject ev = doc["event"].as<JsonObject>();
  if (ev.isNull()) return;
  const char* event_type = ev["event_type"] | "";
  if (strcmp(event_type, "state_changed") != 0) return;
  JsonObject data = ev["data"].as<JsonObject>();
  if (data.isNull()) return;
  const char* eid = data["entity_id"];
  if (eid == nullptr) return;
  if (this->our_entity_ids_.find(eid) == this->our_entity_ids_.end()) return;
  JsonObject new_state = data["new_state"].as<JsonObject>();
  if (new_state.isNull()) return;
  const char* st = new_state["state"] | "";
  int brightness = -1;
  bool has_brightness = false;
  JsonObject attrs = new_state["attributes"].as<JsonObject>();
  if (!attrs.isNull()) {
    JsonVariant bv = attrs["brightness"];
    if (!bv.isNull()) {
      brightness = bv.as<int>();
      if (brightness < 0) brightness = 0;
      if (brightness > 255) brightness = 255;
      has_brightness = true;
    }
  }
  this->push_state_event_(eid, st, brightness, has_brightness);
  esphome::App.wake_loop_threadsafe();
}

// ----- Senders -----

bool HaWsClient::send_text_(const char* body, size_t body_len) {
  if (this->client_ == nullptr) return false;
  int r = esp_websocket_client_send_text(this->client_, body, body_len, portMAX_DELAY);
  if (r < 0) {
    ESP_LOGW(HA_WS_TAG, "esp_websocket_client_send_text failed: %d", r);
    return false;
  }
  return true;
}

bool HaWsClient::send_auth_() {
  // {"type":"auth","access_token":"<token>"}
  // The token may contain JSON special chars; we need to escape it.
  std::string esc;
  esc.reserve(this->token_.size() + 16);
  for (char c : this->token_) {
    if (c == '\\' || c == '"') esc.push_back('\\');
    esc.push_back(c);
  }
  std::string body = std::string("{\"type\":\"auth\",\"access_token\":\"") + esc + "\"}";
  return this->send_text_(body.data(), body.size());
}

bool HaWsClient::send_get_states_() {
  const char* body = "{\"id\":1,\"type\":\"get_states\"}";
  return this->send_text_(body, strlen(body));
}

bool HaWsClient::send_subscribe_events_() {
  const char* body = "{\"id\":2,\"type\":\"subscribe_events\",\"event_type\":\"state_changed\"}";
  return this->send_text_(body, strlen(body));
}

// ----- Helpers -----

bool HaWsClient::derive_ws_uri_() {
  // http://x:8123 -> ws://x:8123/api/websocket
  // https://x:8123 -> wss://x:8123/api/websocket
  const std::string& u = this->http_url_;
  if (u.rfind("https://", 0) == 0) {
    this->ws_uri_ = "wss://" + u.substr(8) + "/api/websocket";
  } else if (u.rfind("http://", 0) == 0) {
    this->ws_uri_ = "ws://" + u.substr(7) + "/api/websocket";
  } else {
    return false;
  }
  return true;
}

uint8_t HaWsClient::classify_kind_(const char* buf, size_t len) {
  if (buf == nullptr || len == 0) return 0;
  size_t n = len < 64 ? len : 64;
  if (memmem(buf, n, "\"type\":\"auth_required\"", 22) != nullptr) return 1;
  if (memmem(buf, n, "\"type\":\"auth_ok\"",        16) != nullptr) return 2;
  if (memmem(buf, n, "\"type\":\"auth_invalid\"",  20) != nullptr) return 3;
  if (memmem(buf, n, "\"type\":\"result\"",        14) != nullptr) return 4;
  if (memmem(buf, n, "\"type\":\"event\"",         13) != nullptr) return 5;
  return 0;
}

bool HaWsClient::push_state_event_(const std::string& eid, const char* state,
                                   int brightness, bool has_brightness) {
  StateEvent* ev = this->state_pool_.allocate();
  if (ev == nullptr) {
    this->events_dropped_.fetch_add(1, std::memory_order_relaxed);
    return false;
  }
  ev->entity_id = eid;
  ev->state = state ? state : "";
  ev->brightness = (uint8_t) (has_brightness ? brightness : 0);
  ev->has_brightness = has_brightness;
  ev->has_state = (state != nullptr);
  if (!this->state_queue_.push(ev)) {
    this->events_dropped_.fetch_add(1, std::memory_order_relaxed);
    ev->release();
    this->state_pool_.release(ev);
    return false;
  }
  return true;
}

}  // namespace esphome::ha_autopanel
