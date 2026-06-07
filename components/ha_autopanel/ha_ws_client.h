// v1.24: ha_ws_client.h — raw WebSocket-to-Home-Assistant client
// for state sync. Replaces the v1.22w esphome-native-API
// subscribe_home_assistant_state path (which triggered PC
// 0x480dxxxx abort ~300ms after Panel READY due to std::function
// allocation under heap pressure).
//
// HA WebSocket protocol: connect to ws://<ha>/api/websocket,
// receive auth_required, send auth with access_token, receive
// auth_ok, then get_states (one big response) + subscribe_events
// (continuous event stream).
//
// Architecture: esp_websocket_client's internal task delivers
// WEBSOCKET_EVENT_DATA chunks. Our handler accumulates one
// in-flight message's bytes into a PSRAM buffer (using
// payload_offset/payload_len to detect message boundary, since
// 200 KB can be split across many chunks). When complete, the
// owned buffer pointer is handed to our parse_task via an
// EventPool + NotifyingLockFreeQueue. parse_task (which we
// subscribe to the TWDT) does the JSON parsing, demuxes by
// "type" field, and pushes per-entity StateEvents to loopTask
// via a second EventPool + NotifyingLockFreeQueue. loopTask
// drains in HaAutoPanel::loop() (bounded to STATE_DRAIN_PER_TICK
// to stay under the 50ms loop warn threshold).
//
// Header-only inline, but the inline method bodies reference
// HaAutoPanel's full definition. The header is therefore
// included from ha_autopanel.cpp (not ha_autopanel.h) so
// HaAutoPanel is complete at the point the methods are
// processed. ha_autopanel.h forward-declares HaWsClient and
// has a manually-declared ~HaAutoPanel() whose definition in
// ha_autopanel.cpp sees the full HaWsClient (PIMPL idiom for
// the unique_ptr<HaWsClient> ws_client_ member).

#pragma once

#include <atomic>
#include <chrono>
#include <cstring>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "esphome/core/application.h"
#include "esphome/core/event_pool.h"
#include "esphome/core/lock_free_queue.h"
#include "esphome/core/log.h"

#include "esp_event.h"
#include "esp_heap_caps.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "esp_websocket_client.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <ArduinoJson.h>

namespace esphome::ha_autopanel {

// Forward declare the parent class. The full definition is in
// ha_autopanel.h, included BEFORE this header in ha_autopanel.cpp.
class HaAutoPanel;

#ifndef HA_WS_TAG
#define HA_WS_TAG "ha_ws"
#endif

// HA WebSocket protocol state machine. Surfaced to the panel
// only for logging; the panel's existing PanelState is left
// alone (the READY transition is still driven by the
// entities_by_area_ ready flag in start_discovery_()).
enum class HaWsState : uint8_t {
  DISCONNECTED,    // Initial state, no client
  CONNECTING,      // esp_websocket_client_start() called
  AUTH_REQUIRED,   // Connected, awaiting first message (auth_required)
  AUTHENTICATING,  // auth sent, awaiting auth_ok
  AUTHENTICATED,   // auth_ok received, about to send get_states
  GETTING_STATES,  // get_states sent, awaiting result (200 KB!)
  READY,           // subscribe_events ack received; live updates streaming
};

// Per-entity state change event handed from parse_task to
// loopTask. Strings are owned copies (std::string) because they
// live across tick boundaries and the WS receive buffer is
// recycled. Allocated from EventPool<StateEvent, 31>.
struct StateEvent {
  std::string entity_id;
  std::string state;
  uint8_t brightness{0};
  bool has_brightness{false};
  bool has_state{false};

  // Required by EventPool. Resets all fields without
  // freeing the underlying memory. Called by the consumer
  // (loopTask drain) after applying the event.
  void release() {
    entity_id.clear();
    entity_id.shrink_to_fit();
    state.clear();
    state.shrink_to_fit();
    brightness = 0;
    has_brightness = false;
    has_state = false;
  }
};

// Raw, owned PSRAM payload handed from the WS event handler
// (on esp_websocket_client's internal task) to parse_task.
// The PSRAM data buffer is owned by RawWsMessage and freed
// via release() (called by parse_task after the JSON parse).
// Allocated from EventPool<RawWsMessage, 7>.
struct RawWsMessage {
  char* data{nullptr};
  size_t len{0};
  uint32_t id{0};   // HA's "id" field, demux'd by parse_task
  uint8_t kind{0};  // 0=unknown, 1=auth_required, 2=auth_ok, 3=auth_invalid,
                    // 4=result, 5=event
  void release() {
    if (data) {
      heap_caps_free(data);
      data = nullptr;
    }
    len = 0;
  }
};

// PSRAM-preferring malloc. PSRAM first (8 MB pool on the
// Crowpanel P4), fall back to internal heap if PSRAM is full.
static inline char* psram_strdup_(size_t cap) {
  char* p = (char*) heap_caps_malloc_prefer(
      cap, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT,
      MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  return p;
}

// Pre-allocated PSRAM pool for the 229KB get_states JSON parse.
// Allocated once at HaWsClient::start(), reused for every
// subsequent get_states result. ArduinoJson's deserializer
// calls our allocate()/reallocate() to grow its internal pool;
// since we just bump pos_ in a 1MB PSRAM buffer, no heap
// lock is taken during the parse. The SDIO RX task on
// core 0 can drain its wifi buffer without contending for
// the heap. v1.24 fix for sdio_drv.c:1260 copy_payload assert.
//
// The user confirmed 32MB PSRAM is available, so 1MB for the
// pool is cheap.
class HaWsPreallocPool : public ArduinoJson::Allocator {
 public:
  static constexpr size_t CAPACITY = 1024 * 1024;  // 1 MB

  void init() {
    if (buf_ == nullptr) {
      buf_ = (char*) heap_caps_malloc_prefer(
          CAPACITY, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT,
          MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
      if (buf_ != nullptr) {
        ESP_LOGI(HA_WS_TAG, "init: pre-allocated 1MB PSRAM pool for JSON parse");
      } else {
        ESP_LOGE(HA_WS_TAG, "init: failed to pre-allocate 1MB PSRAM");
      }
    }
    this->reset_();
  }

  void reset_() { pos_ = 0; }

  void* allocate(size_t n) override {
    if (n == 0) return nullptr;
    if (buf_ == nullptr || pos_ + n > CAPACITY) return nullptr;  // OOM
    void* p = buf_ + pos_;
    pos_ += n;
    return p;
  }

  void* reallocate(void* p, size_t n) override {
    if (n == 0) return nullptr;
    if (buf_ == nullptr || pos_ + n > CAPACITY) return nullptr;
    void* np = buf_ + pos_;
    pos_ += n;
    if (p != nullptr) {
      memcpy(np, p, n);
    }
    return np;
  }

  void deallocate(void* p) override { /* no-op */ }

  size_t used() const { return pos_; }

 private:
  char* buf_{nullptr};
  size_t pos_{0};
};

// ChunkedJsonReader: wraps a const char* + size_t and yields
// to the FreeRTOS scheduler every CHUNK_YIELD_BYTES bytes
// during the 200 KB+ get_states parse. Without the yields,
// the parse_task hogs core 0 for >1s and the SDIO RX task
// can't drain its buffer - sdio_drv.c:1260 copy_payload
// asserts. The yields give the SDIO task (and any other
// core-0 task) a chance to run. The TWDT is fed by
// esp_task_wdt_reset() between yields.
//
// ArduinoJson 7.x's deserializeJson takes any class that
// duck-types read() + readBytes() (no inheritance required
// - the template deduces the interface). We pass a reference
// to this class to deserializeJson.
class ChunkedJsonReader {
 public:
  ChunkedJsonReader(const char* data, size_t len)
      : data_(data), len_(len), pos_(0) {}

  int read() {
    if (pos_ >= len_) return -1;
    char c = this->data_[this->pos_++];
    this->maybe_yield_(1);
    return static_cast<unsigned char>(c);
  }

  size_t readBytes(char* buffer, size_t length) {
    size_t avail = this->len_ - this->pos_;
    size_t to_read = (length < avail) ? length : avail;
    if (to_read > 0) {
      memcpy(buffer, this->data_ + this->pos_, to_read);
      this->pos_ += to_read;
      this->maybe_yield_(to_read);
    }
    return to_read;
  }

 private:
  void maybe_yield_(size_t n) {
    this->bytes_since_yield_ += n;
    if (this->bytes_since_yield_ >= CHUNK_YIELD_BYTES) {
      this->bytes_since_yield_ = 0;
      esp_task_wdt_reset();
      vTaskDelay(1);
    }
  }

  const char* data_;
  size_t len_;
  size_t pos_;
  size_t bytes_since_yield_{0};
  static constexpr size_t CHUNK_YIELD_BYTES = 4096;
};

class HaWsClient {
 public:
  explicit HaWsClient(HaAutoPanel* parent) : parent_(parent) {}

  ~HaWsClient() {
    stop_();
  }

  // Called from HaAutoPanel::start_discovery_() AFTER
  // fetch_areas_() has populated entities_by_area_ with the
  // entity_id list. We snapshot the entity_id set here (one
  // synchronous copy, no concurrency) so parse_task can filter
  // get_states responses and state_changed events without
  // needing to lock entities_by_area_. Safe to call from
  // loopTask. Idempotent.
  void start();

  // Drain up to STATE_DRAIN_PER_TICK pending StateEvents into
  // the parent's on_entity_state_changed_ /
  // on_entity_attribute_changed_ handlers. Called from
  // HaAutoPanel::loop() each tick. Bounded to keep the loop
  // iteration under the 50ms warn threshold.
  void drain_state_events();

  // True after we've successfully received and parsed the
  // get_states response. HaAutoPanel uses this to decide
  // when it's safe to render the room grid (though the
  // room grid already renders with stub state from
  // fetch_areas_(); this just gates the PanelState::READY
  // transition).
  bool initial_states_received() const { return initial_states_received_.load(std::memory_order_acquire); }

  // Convenience: is the WS state machine at READY?
  bool is_ws_ready() const { return ws_state_.load(std::memory_order_acquire) == HaWsState::READY; }

  // For dump_config and the test debug page.
  HaWsState ws_state() const { return ws_state_.load(std::memory_order_acquire); }
  uint32_t events_received() const { return events_received_.load(std::memory_order_relaxed); }
  uint32_t events_dropped() const { return events_dropped_.load(std::memory_order_relaxed); }
  uint32_t states_applied() const { return states_applied_.load(std::memory_order_relaxed); }

  // setters (called once from HaAutoPanel::start_discovery_
  // before start()).
  void set_url(const std::string& http_url) { http_url_ = http_url; }
  void set_token(const std::string& token) { token_ = token; }

 private:
  // ===== Constants =====
  static constexpr size_t WS_RX_BUFFER_BYTES = 8192;     // esp_websocket_client buffer_size
  static constexpr int    WS_TASK_PRIO       = 3;        // esp_websocket_client's internal task
  static constexpr int    WS_TASK_STACK      = 6144;     // bytes
  static constexpr int    PARSE_TASK_PRIO    = 3;        // our parse task
  static constexpr int    PARSE_TASK_STACK   = 16384;    // bytes; 200 KB JSON parse needs headroom
  // v1.24 fix: was 0. The SDIO RX task (which drains the wifi
  // C6's data into the host) runs on core 0. When our parse_task
  // was also on core 0, the 200KB get_states parse hogged the
  // core for >1s and the SDIO ring buffer overflowed
  // (sdio_drv.c:1260 copy_payload assert). Moving the parse
  // to core 1 lets the SDIO task on core 0 drain in real time
  // as the 200KB response streams in. loopTask also runs on
  // core 1 but is preempted by the parse_task (prio 3 vs 1)
  // for the duration of the parse. The chunked-parse
  // ChunkedJsonReader is kept as a safety net (helps if per-
  // entity processing is the bottleneck rather than the
  // deserializeJson itself).
  static constexpr int    PARSE_TASK_CORE    = 1;
  static constexpr size_t RAW_QUEUE_CAPACITY = 8;        // WS task -> parse_task
  static constexpr size_t STATE_QUEUE_CAPACITY = 32;     // parse_task -> loopTask
  static constexpr size_t STATE_DRAIN_PER_TICK = 16;     // hard cap per loop iteration
  static constexpr int    PING_INTERVAL_SEC   = 25;       // HA hardcodes ~55s pong timeout
  static constexpr int    PING_TIMEOUT_SEC    = 60;
  static constexpr uint32_t GET_STATES_ID     = 1;
  static constexpr uint32_t SUBSCRIBE_ID      = 2;
  // After 10s with no auth_ok after sending auth, force a reconnect
  // (esp_websocket_client is otherwise silent if HA's response stalls).
  static constexpr uint32_t AUTH_TIMEOUT_MS    = 10000;
  // After 30s with no subscribe_events ack, force a reconnect.
  static constexpr uint32_t SUBSCRIBE_TIMEOUT_MS = 30000;

  // ===== Owner / config =====
  HaAutoPanel* parent_;
  std::string  http_url_;     // e.g. http://homeassistant.local:8123
  std::string  token_;        // long-lived access token

  // ===== esp_websocket_client =====
  esp_websocket_client_handle_t client_{nullptr};
  std::string ws_uri_;                              // derived in start_(); held alive while client_ is up
  std::atomic<HaWsState> ws_state_{HaWsState::DISCONNECTED};
  std::atomic<bool> initial_states_received_{false};
  std::atomic<bool> stopping_{false};
  std::atomic<uint32_t> auth_sent_ms_{0};           // esp_timer_get_time()/1000
  std::atomic<uint32_t> subscribe_sent_ms_{0};

  // ===== Per-message in-flight buffer (used by ws_event_handler
  // to reassemble multi-chunk WebSocket frames into a single
  // owned message). Only ever written on the IDF event loop
  // task. Released either to raw_queue_ (when complete) or
  // freed (on error / disconnect). =====
  char*   inflight_buf_{nullptr};
  size_t  inflight_cap_{0};
  size_t  inflight_len_{0};

  // ===== SPSC queues =====
  // WS event handler -> parse_task: complete raw JSON messages
  EventPool<RawWsMessage, RAW_QUEUE_CAPACITY - 1> raw_pool_;
  NotifyingLockFreeQueue<RawWsMessage, RAW_QUEUE_CAPACITY> raw_queue_;

  // parse_task -> loopTask: per-entity StateEvents
  EventPool<StateEvent, STATE_QUEUE_CAPACITY - 1> state_pool_;
  NotifyingLockFreeQueue<StateEvent, STATE_QUEUE_CAPACITY> state_queue_;

  TaskHandle_t parse_task_handle_{nullptr};

  // ===== Filter =====
  // Snapshot of entity_ids we care about, captured synchronously
  // in start() (after fetch_areas_ completes, before parse_task
  // is created). parse_task reads this on its own context; the
  // HaAutoPanel never modifies it after start() returns.
  std::set<std::string> our_entity_ids_;

  // ===== Counters =====
  std::atomic<uint32_t> events_received_{0};
  std::atomic<uint32_t> events_dropped_{0};
  std::atomic<uint32_t> states_applied_{0};

  // ===== Lifecycle =====
  void stop_();

  // ===== esp_websocket_client event handler (static; arg is
  // the HaWsClient instance) =====
  static void ws_event_handler_(void* arg, esp_event_base_t base,
                                int32_t event_id, void* event_data);
  void on_ws_connected_();
  void on_ws_disconnected_();
  void on_ws_data_chunk_(const esp_websocket_event_data_t* ev);
  void on_ws_closed_();
  void on_ws_error_();

  // ===== Parse task =====
  static void parse_task_trampoline_(void* arg);
  void parse_task_loop_();
  void parse_auth_required_();
  void parse_auth_ok_();
  void parse_auth_invalid_(const char* json, size_t len);
  void parse_get_states_result_(const char* json, size_t len);
  void parse_event_message_(const char* json, size_t len);

  // ===== Outgoing messages (called on parse_task; synchronous send) =====
  bool send_auth_();
  bool send_get_states_();
  bool send_subscribe_events_();

  // ===== Helpers =====
  bool derive_ws_uri_();  // http://x:8123 -> ws://x:8123/api/websocket; https->wss
  // Peek at first ~64 bytes for "type":"..." to classify the
  // message kind. Runs on WS event task; must be fast.
  uint8_t classify_kind_(const char* buf, size_t len);
  // Push one StateEvent to loopTask. Returns false on full
  // queue (increments events_dropped_).
  bool push_state_event_(const std::string& eid, const char* state,
                         int brightness, bool has_brightness);
  bool send_text_(const char* body, size_t body_len);

  // v1.24: pre-allocated 1MB PSRAM pool for the ArduinoJson
  // deserializer. Allocated once at start() and reused for
  // every get_states response. Eliminates heap lock
  // contention with the SDIO RX task (which was the
  // sdio_drv.c:1260 copy_payload assert trigger). With 32MB
  // PSRAM on the P4, 1MB for the pool is cheap.
  HaWsPreallocPool psram_pool_;
};

// =====================================================================
// Implementation (inline). All bodies are defined here so the
// .h is self-contained - but the bodies reference HaAutoPanel's
// full definition, so this header MUST be included AFTER
// ha_autopanel.h defines the class. The integration point is
// ha_autopanel.cpp (which includes both).
// =====================================================================

namespace {

// Local TwdtGuard (parallels the one in ha_autopanel.cpp:99-124).
// Parsing the 200 KB get_states response can take >5s; we
// unsubscribe the parse_task from the IDF TWDT for the duration
// of the parse and re-subscribe on scope exit.
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

// PSRAM-preferring JSON document allocator (used for small event
// messages, where heap allocation is fine). For the large
// get_states parse we use a pre-allocated pool (see below) to
// avoid contending with the SDIO RX task for the heap lock.
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

// Pre-allocated PSRAM pool for the 229KB get_states JSON parse.
// Allocated once at HaWsClient::start(), reused for every
// subsequent get_states result. ArduinoJson's deserializer
// calls our allocate()/reallocate() to grow its internal pool;
// (Duplicates removed - HaWsPreallocPool and ChunkedJsonReader
// are now defined before the HaWsClient class above.)

}  // namespace

// ----- Lifecycle -----

inline void HaWsClient::start() {
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

  // v1.24: pre-allocate the 1MB PSRAM pool for the
  // ArduinoJson deserializer. Done here, in start(), so the
  // SDIO RX task isn't competing for the heap lock during
  // the 229KB get_states parse.
  this->psram_pool_.init();

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

inline void HaWsClient::stop_() {
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

inline void HaWsClient::drain_state_events() {
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

inline void HaWsClient::ws_event_handler_(void* arg, esp_event_base_t base,
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

inline void HaWsClient::on_ws_connected_() {
  ESP_LOGI(HA_WS_TAG, "WEBSOCKET_EVENT_CONNECTED to %s", this->ws_uri_.c_str());
  this->ws_state_.store(HaWsState::AUTH_REQUIRED, std::memory_order_release);
}

inline void HaWsClient::on_ws_disconnected_() {
  ESP_LOGW(HA_WS_TAG, "WEBSOCKET_EVENT_DISCONNECTED");
  this->ws_state_.store(HaWsState::DISCONNECTED, std::memory_order_release);
  if (this->inflight_buf_ != nullptr) {
    heap_caps_free(this->inflight_buf_);
    this->inflight_buf_ = nullptr;
    this->inflight_cap_ = 0;
    this->inflight_len_ = 0;
  }
}

inline void HaWsClient::on_ws_data_chunk_(const esp_websocket_event_data_t* ev) {
  if (ev == nullptr || ev->data_ptr == nullptr || ev->data_len == 0) return;
  if (ev->op_code == 0x8 || ev->op_code == 0x9 || ev->op_code == 0xA) return;
  if (ev->op_code != 0x1 && ev->op_code != 0x0 && ev->op_code != 0x2) return;

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
  if (this->inflight_buf_ != nullptr &&
      this->inflight_len_ + ev->data_len <= this->inflight_cap_) {
    memcpy(this->inflight_buf_ + this->inflight_len_, ev->data_ptr, ev->data_len);
    this->inflight_len_ += ev->data_len;
  }
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

inline void HaWsClient::on_ws_closed_() {
  ESP_LOGI(HA_WS_TAG, "WEBSOCKET_EVENT_CLOSED");
  this->ws_state_.store(HaWsState::DISCONNECTED, std::memory_order_release);
}

inline void HaWsClient::on_ws_error_() {
  ESP_LOGE(HA_WS_TAG, "WEBSOCKET_EVENT_ERROR");
  this->ws_state_.store(HaWsState::DISCONNECTED, std::memory_order_release);
}

// ----- Parse task -----

inline void HaWsClient::parse_task_trampoline_(void* arg) {
  HaWsClient* self = static_cast<HaWsClient*>(arg);
  if (self != nullptr) self->parse_task_loop_();
  vTaskDelete(nullptr);
}

inline void HaWsClient::parse_task_loop_() {
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

inline void HaWsClient::parse_auth_required_() {
  ESP_LOGI(HA_WS_TAG, "auth_required received, sending auth");
  this->ws_state_.store(HaWsState::AUTHENTICATING, std::memory_order_release);
  this->auth_sent_ms_.store((uint32_t) (esp_timer_get_time() / 1000),
                            std::memory_order_release);
  this->send_auth_();
}

inline void HaWsClient::parse_auth_ok_() {
  ESP_LOGI(HA_WS_TAG, "auth_ok received");
  this->ws_state_.store(HaWsState::AUTHENTICATED, std::memory_order_release);
  this->auth_sent_ms_.store(0, std::memory_order_release);
  this->subscribe_sent_ms_.store((uint32_t) (esp_timer_get_time() / 1000),
                                std::memory_order_release);
  this->send_get_states_();
  this->ws_state_.store(HaWsState::GETTING_STATES, std::memory_order_release);
}

inline void HaWsClient::parse_auth_invalid_(const char* json, size_t len) {
  ESP_LOGE(HA_WS_TAG, "auth_invalid: %.*s", (int) len, json ? json : "(null)");
  this->ws_state_.store(HaWsState::DISCONNECTED, std::memory_order_release);
  if (this->client_ != nullptr) {
    esp_websocket_client_stop(this->client_);
  }
}

inline void HaWsClient::parse_get_states_result_(const char* json, size_t len) {
  if (json == nullptr || len == 0) {
    ESP_LOGW(HA_WS_TAG, "get_states result empty");
    return;
  }
  ESP_LOGI(HA_WS_TAG, "get_states result: %u bytes, parsing...", (unsigned) len);
  uint32_t t0 = (uint32_t) (esp_timer_get_time() / 1000);

  // v1.24: use the pre-allocated 1MB PSRAM pool. No heap
  // lock contention with the SDIO RX task during the
  // parse. Reset the pool position before each parse so
  // the same buffer is reused.
  this->psram_pool_.reset_();
  ArduinoJson::JsonDocument doc(&this->psram_pool_);

  {
    HaWsTwdtGuard g;  // unsubscribe from TWDT for the long parse
    // ChunkedJsonReader yields to the scheduler every 4 KB
    // during the parse. This gives the SDIO RX task on
    // core 0 a chance to drain its buffer, preventing the
    // sdio_drv.c:1260 copy_payload assert that fires when
    // the parse_task hogs the core for >1s with a 200 KB+
    // get_states response.
    ChunkedJsonReader reader(json, len);
    DeserializationError err = deserializeJson(doc, reader);
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

inline void HaWsClient::parse_event_message_(const char* json, size_t len) {
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

inline bool HaWsClient::send_text_(const char* body, size_t body_len) {
  if (this->client_ == nullptr) return false;
  int r = esp_websocket_client_send_text(this->client_, body, body_len, portMAX_DELAY);
  if (r < 0) {
    ESP_LOGW(HA_WS_TAG, "esp_websocket_client_send_text failed: %d", r);
    return false;
  }
  return true;
}

inline bool HaWsClient::send_auth_() {
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

inline bool HaWsClient::send_get_states_() {
  const char* body = "{\"id\":1,\"type\":\"get_states\"}";
  return this->send_text_(body, strlen(body));
}

inline bool HaWsClient::send_subscribe_events_() {
  const char* body = "{\"id\":2,\"type\":\"subscribe_events\",\"event_type\":\"state_changed\"}";
  return this->send_text_(body, strlen(body));
}

// ----- Helpers -----

inline bool HaWsClient::derive_ws_uri_() {
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

inline uint8_t HaWsClient::classify_kind_(const char* buf, size_t len) {
  if (buf == nullptr || len == 0) return 0;
  size_t n = len < 64 ? len : 64;
  if (memmem(buf, n, "\"type\":\"auth_required\"", 22) != nullptr) return 1;
  if (memmem(buf, n, "\"type\":\"auth_ok\"",        16) != nullptr) return 2;
  if (memmem(buf, n, "\"type\":\"auth_invalid\"",  20) != nullptr) return 3;
  if (memmem(buf, n, "\"type\":\"result\"",        14) != nullptr) return 4;
  if (memmem(buf, n, "\"type\":\"event\"",         13) != nullptr) return 5;
  return 0;
}

inline bool HaWsClient::push_state_event_(const std::string& eid, const char* state,
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
