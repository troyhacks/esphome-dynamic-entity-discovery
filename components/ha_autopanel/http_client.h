// HttpClient: thin wrapper around esphome::http_request that
// encapsulates the boilerplate of doing an HTTP request and
// reading the full body into a NUL-terminated buffer. The
// pattern was duplicated in 8+ fetch functions in
// ha_autopanel.cpp (each ~30 lines of status-code checks,
// null-container check, PSRAM-preferring allocation, chunked
// read with timeout, free). This wrapper collapses it to a
// single HttpResult{status, http_code, body} return value.
//
// The wrapper does NOT do JSON parsing — callers parse with
// ArduinoJson directly on result.body. JSON parsing is
// context-specific (per-endpoint schema) and putting it in
// the wrapper would force a one-size-fits-all parser that
// would actually be MORE code than the current duplicates.
//
// v1.23: extracted from ha_autopanel.cpp during the
// readability refactor. See memory
// `ha-autopanel-v123-refactor` for the full rationale.
//
// NOTE on inline implementation: ESPHome's build system
// only compiles the component's main .cpp file (the one
// matching the component name). To keep this header
// self-contained without forcing a CMakeLists.txt change,
// all methods are inline in the header. The class is only
// instantiated a handful of times (one per fetch), so the
// code-size cost of inline methods is negligible. If we
// later need non-inline methods, the right path is to add
// a CMakeLists.txt that lists http_client.cpp as a source
// (ESPHome supports this via the standard scons_sources
// or extra_scripts mechanism).

#pragma once

#include <string>
#include <vector>
#include "esp_heap_caps.h"
#include "esphome/components/http_request/http_request.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"

namespace esphome {
namespace ha_autopanel {

// Result of an HTTP request. status encodes the OUTCOME
// (network failure, OOM, etc.) without the caller having to
// inspect http_code / exception / etc. body is non-empty only
// when status == OK.
enum class HttpStatus {
  OK = 0,                  // 200 + body read
  CONNECTION_FAILED,        // status_code == 0 (network error)
  AUTH_FAILED,              // 401
  NOT_FOUND,                // 404
  HTTP_ERROR,               // other non-2xx
  OOM,                      // could not allocate response buffer
  TIMEOUT,                  // read timed out before body complete
  INTERNAL_ERROR,           // null container / unexpected state
};

struct HttpResult {
  HttpStatus status{HttpStatus::INTERNAL_ERROR};
  int http_code{0};
  // Body is only populated when status == OK. Caller is
  // responsible for the parse — see http_client.h comment.
  // v1.28: psram_string. The body can be 25+ KB (the
  // aggregate template response, the /api/states burst, etc.)
  // - with std::string that payload landed in the precious
  // 384 KB internal heap and was the #1 contributor to the
  // cxx_exception_stubs OOM aborts. psram_string keeps the
  // body in PSRAM. Callers that bind `std::string& x = result.body`
  // must switch to `auto& x = result.body` (the converting
  // ctor from psram_string to std::string is explicit because
  // PsramStlAllocator != std::allocator).
  psram_string body;
};

// Thin wrapper. Lifetime is tied to the http_request component
// (must outlive this). Construct at the top of each fetch
// function (cheap — no allocation beyond the pointer copy).
class HttpClient {
 public:
  explicit HttpClient(http_request::HttpRequestComponent* http) : http_(http) {}

  // GET <url>. Returns the body in result.body when OK.
  // The 8+ fetch functions in ha_autopanel.cpp duplicate this
  // exact pattern (auth check, status check, response read).
  HttpResult get(const std::string& url,
                 const std::vector<http_request::Header>& headers) {
    return this->do_request_(/*is_post=*/false, url, /*body=*/"", headers);
  }

  // POST <url> with the given body. The Content-Type header
  // (if needed) is the caller's responsibility — pass it in
  // `headers`.
  HttpResult post(const std::string& url,
                  const std::string& body,
                  const std::vector<http_request::Header>& headers) {
    return this->do_request_(/*is_post=*/true, url, body, headers);
  }

  // Common header: Authorization: Bearer <token>. Callers
  // add Content-Type (and any others) on top.
  std::vector<http_request::Header> bearer_auth(const std::string& token) const {
    return {{"Authorization", "Bearer " + token}};
  }

 private:
  // Single private method that does the work for both get
  // and post. Kept inline so the whole class is in the
  // header (no separate .cpp file needed; see note at top).
  HttpResult do_request_(bool is_post, const std::string& url,
                          const std::string& body,
                          const std::vector<http_request::Header>& headers) {
    static const char* TAG = "ha_http";
    auto container = is_post
        ? this->http_->post(url, body, headers)
        : this->http_->get(url, headers);
    if (container == nullptr) {
      ESP_LOGW(TAG, "%s %s: null container (http_request internal error)",
               is_post ? "POST" : "GET", url.c_str());
      return {HttpStatus::INTERNAL_ERROR, 0, ""};
    }
    int code = container->status_code;
    if (code == 0) {
      ESP_LOGW(TAG, "%s %s: connection failed", is_post ? "POST" : "GET", url.c_str());
      container->end();
      return {HttpStatus::CONNECTION_FAILED, 0, ""};
    }
    if (code == 401) {
      ESP_LOGW(TAG, "%s %s: 401 unauthorized (token rejected)",
               is_post ? "POST" : "GET", url.c_str());
      container->end();
      return {HttpStatus::AUTH_FAILED, 401, ""};
    }
    if (code == 404) {
      ESP_LOGD(TAG, "%s %s: 404 not found", is_post ? "POST" : "GET", url.c_str());
      container->end();
      return {HttpStatus::NOT_FOUND, 404, ""};
    }
    if (code != 200) {
      ESP_LOGW(TAG, "%s %s: HTTP %d", is_post ? "POST" : "GET", url.c_str(), code);
      container->end();
      return {HttpStatus::HTTP_ERROR, code, ""};
    }
    HttpResult read_result = this->read_body_(container.get());
    container->end();
    return read_result;
  }

  // Read full body of `container` into a PSRAM-preferring
  // buffer. Returns OK on success with result.body set, or
  // OOM / TIMEOUT on failure with result.body empty. The
  // container is NOT closed — caller owns it.
  HttpResult read_body_(void* container_v) {
    static const char* TAG = "ha_http";
    auto* container = static_cast<http_request::HttpContainer*>(container_v);

    size_t expected = container->content_length;
    if (expected == 0) expected = 32768;

    // v1.12: PSRAM first, internal heap fallback. The 200KB+
    // /api/states response would OOM the S3 (no PSRAM, ~384KB
    // internal heap) on a plain malloc; heap_caps_malloc_prefer
    // puts the buffer in PSRAM on the Crowpanel / P4 and falls
    // back to internal heap on the S3 if needed.
    char* response = (char*)heap_caps_malloc_prefer(
        expected + 1,                              // +1 for NUL
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT,
        MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (response == nullptr) {
      ESP_LOGE(TAG, "OOM allocating %u-byte response buffer",
               (unsigned)(expected + 1));
      return {HttpStatus::OOM, 0, ""};
    }

    size_t response_len = 0;
    uint8_t buf[512];
    uint32_t last_data_time = millis();
    const uint32_t timeout = 20000;
    int iter_count = 0;

    while (container->get_bytes_read() < container->content_length) {
      App.feed_wdt();
      yield();
      int read = container->read(buf, sizeof(buf));
      if (read > 0) {
        if (response_len + (size_t)read + 1 > expected + 1) {
          size_t new_size = (expected + read + 1) * 2;
          char* grown = (char*)heap_caps_malloc_prefer(
              new_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT,
              MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
          if (grown == nullptr) {
            ESP_LOGE(TAG, "OOM growing response buffer to %u bytes",
                     (unsigned)new_size);
            heap_caps_free(response);
            return {HttpStatus::OOM, 0, ""};
          }
          memcpy(grown, response, response_len);
          heap_caps_free(response);
          response = grown;
          expected = new_size - 1;
        }
        memcpy(response + response_len, buf, read);
        response_len += read;
        last_data_time = millis();
        iter_count = 0;
      } else if (read == http_request::HTTP_ERROR_CONNECTION_CLOSED) {
        break;
      } else {
        iter_count++;
        if (iter_count % 10 == 0) App.feed_wdt();
      }
      if (millis() - last_data_time > timeout) {
        ESP_LOGW(TAG, "Timeout reading response");
        heap_caps_free(response);
        return {HttpStatus::TIMEOUT, 0, ""};
      }
    }

    response[response_len] = '\0';
    HttpResult result;
    result.status = HttpStatus::OK;
    result.body.assign(response, response_len);
    heap_caps_free(response);
    return result;
  }

  http_request::HttpRequestComponent* http_;
};

}  // namespace ha_autopanel
}  // namespace esphome
