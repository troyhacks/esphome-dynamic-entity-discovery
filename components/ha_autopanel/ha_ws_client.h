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

// HA WebSocket protocol state machine. v1.27 (Phase 4.7):
// AUTHENTICATED -> READY directly. The get_states round
// trip is replaced by the per-area aggregate template
// (subscribed in setup_render_template_subscriptions_).
enum class HaWsState : uint8_t {
  DISCONNECTED,    // Initial state, no client
  CONNECTING,      // esp_websocket_client_start() called
  AUTH_REQUIRED,   // Connected, awaiting first message (auth_required)
  AUTHENTICATING,  // auth sent, awaiting auth_ok
  AUTHENTICATED,   // auth_ok received, subscriptions sent
  READY,           // subscriptions live; clock + aggregate updates streaming
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

// v1.27: clock event. Holds the HA-rendered clock string
// (e.g. "14:30" or "2:30 PM") that HA pushed via the
// render_template subscription. Handed from parse_task to
// loopTask via a small SPSC queue. The clock fires once per
// minute, so the pool depth (3) is plenty.
struct ClockEvent {
  std::string rendered;  // e.g. "14:30"
  void release() {
    rendered.clear();
    rendered.shrink_to_fit();
  }
};

// v1.27: aggregate event. Holds the per-area aggregate JSON
// that HA pushed when any of the panel's relevant entities
// changed. Same lifecycle as ClockEvent. Pool depth 3 is
// fine: the per-area template only re-renders ~once per
// second under heavy state changes (HA's coalescing).
// The 25KB JSON buffer is in PSRAM (via PsramStlAllocator)
// because the parse_task's internal heap is too small to
// hold two of these at once (the 4.7KB httpd-style 4KB
// stack and ~300KB internal heap can't absorb a 50KB burst).
// Pre-v1.27 this struct was std::string (default allocator,
// internal heap) and we saw std::bad_alloc panics on the
// S3/P4 parse_task whenever HA pushed >1 aggregate per
// few seconds.
struct AggregateEvent {
  std::basic_string<char, std::char_traits<char>, PsramStlAllocator<char>> json;
  void release() {
    json.clear();
    json.shrink_to_fit();
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
//
// v1.27 (Phase 4.7): HaWsPreallocPool and ChunkedJsonReader
// are GONE. The 200 KB get_states parse is replaced by the
// per-area render_template subscription (~50 KB per push),
// which fits comfortably in the HaWsPsramAllocator's heap
// path. No more pre-alloc pool, no more chunked reader.

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

  // v1.27: drain up to CLOCK_DRAIN_PER_TICK pending ClockEvents
  // into the parent's on_clock_update_() handler. Same SPSC
  // pattern as drain_state_events. Called from
  // HaAutoPanel::loop() each tick.
  void drain_clock_events();

  // v1.27: push one rendered clock string to the clock event
  // queue. Called from the TemplateApi::subscribe callback
  // (which runs on parse_task). The queue is drained on
  // loopTask, so the actual lv_label_set_text happens in
  // LVGL-safe context.
  bool push_clock_event_(const char* rendered);

  // v1.27: same, for the per-area aggregate push. Used in Phase 4.
  bool push_aggregate_event_(const char* json);
  void drain_aggregate_events();

  // v1.27 (Phase 4.7): initial_states_received() is gone
  // (the get_states round trip is gone). The room grid
  // renders with stub state from fetch_areas_() and the
  // per-area aggregate (subscribed via WS) updates the
  // state asynchronously. Use is_ws_ready() (below) to
  // check the WS connection state.

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

  // Thread-safe external WS send. Used by TemplateApi to
  // send render_template / unsubscribe_events messages from
  // the calling thread (typically loopTask). The underlying
  // esp_websocket_client_send_text is internally locked, so
  // this is safe to call from any context. Returns false if
  // the client isn't connected.
  bool send_text_external(const std::string& body) {
    if (this->client_ == nullptr) return false;
    int r = esp_websocket_client_send_text(this->client_,
                                           body.data(), body.size(),
                                           portMAX_DELAY);
    if (r < 0) {
      ESP_LOGW(HA_WS_TAG, "send_text_external: esp_websocket_client_send_text failed: %d", r);
      return false;
    }
    return true;
  }

  // Notify the WS layer that a TemplateApi event with the
  // given id was received. Called from
  // parse_event_message_() after demuxing by the event's id.
  // Safe to call from parse_task only.
  void forward_template_event_(uint32_t id, const char* result) {
    if (this->parent_ == nullptr) return;
    // parent_->template_api_ is accessible because HaAutoPanel
    // is a complete type at the point this header is included
    // (ha_ws_client.h is included from ha_autopanel.cpp AFTER
    // HaAutoPanel's full definition).
    this->parent_->template_api_.on_ws_event_(id, result);
  }

 private:
  // ===== Constants =====
  static constexpr size_t WS_RX_BUFFER_BYTES = 8192;     // esp_websocket_client buffer_size
  static constexpr int    WS_TASK_PRIO       = 3;        // esp_websocket_client's internal task
  static constexpr int    WS_TASK_STACK      = 6144;     // bytes
  static constexpr int    PARSE_TASK_PRIO    = 3;        // our parse task
  // v1.27 (Phase 4.7): reduced from 16 KB to 8 KB. The
  // 200 KB get_states parse is gone; the parse_task now
  // only handles small render_template event payloads
  // (~50 KB per push, which ArduinoJson stores on the
  // heap, not on the task stack).
  static constexpr size_t PARSE_TASK_STACK   = 8192;     // bytes
  // v1.27 (Phase 4.7): PARSE_TASK_CORE is now
  // unimportant for SDIO contention (the big parse is
  // gone). Kept on core 1 to avoid being preempted by
  // loopTask; future-tuning can move it if needed.
  static constexpr int    PARSE_TASK_CORE    = 1;
  static constexpr size_t RAW_QUEUE_CAPACITY = 8;        // WS task -> parse_task
  static constexpr size_t STATE_QUEUE_CAPACITY = 32;     // parse_task -> loopTask
  static constexpr size_t STATE_DRAIN_PER_TICK = 16;     // hard cap per loop iteration
  static constexpr int    PING_INTERVAL_SEC   = 25;       // HA hardcodes ~55s pong timeout
  static constexpr int    PING_TIMEOUT_SEC    = 60;
  // v1.27 (Phase 4.7): GET_STATES_ID and SUBSCRIBE_ID
  // removed. The render_template subscriptions use
  // TemplateApi's auto-incrementing ids (starting at 100).
  // After 10s with no auth_ok after sending auth, force a reconnect
  // (esp_websocket_client is otherwise silent if HA's response stalls).
  static constexpr uint32_t AUTH_TIMEOUT_MS    = 10000;

  // ===== Owner / config =====
  HaAutoPanel* parent_;
  std::string  http_url_;     // e.g. http://homeassistant.local:8123
  std::string  token_;        // long-lived access token

  // ===== esp_websocket_client =====
  esp_websocket_client_handle_t client_{nullptr};
  std::string ws_uri_;                              // derived in start_(); held alive while client_ is up
  std::atomic<HaWsState> ws_state_{HaWsState::DISCONNECTED};
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

  // v1.27: parse_task -> loopTask: clock update events.
  // Pool depth 3 covers the per-minute clock push plus a
  // burst of up to 2 backlogged events if loopTask is busy
  // doing a long render.
  static constexpr size_t CLOCK_QUEUE_CAPACITY = 4;
  EventPool<ClockEvent, CLOCK_QUEUE_CAPACITY - 1> clock_pool_;
  NotifyingLockFreeQueue<ClockEvent, CLOCK_QUEUE_CAPACITY> clock_queue_;
  static constexpr size_t CLOCK_DRAIN_PER_TICK = 4;

  // v1.27: parse_task -> loopTask: per-area aggregate events.
  // Pool depth 3 covers a burst of updates (HA coalesces
  // multiple state changes into one re-render, so this
  // rarely fires more than 1-2 times per second).
  static constexpr size_t AGG_QUEUE_CAPACITY = 4;
  EventPool<AggregateEvent, AGG_QUEUE_CAPACITY - 1> agg_pool_;
  NotifyingLockFreeQueue<AggregateEvent, AGG_QUEUE_CAPACITY> agg_queue_;
  static constexpr size_t AGG_DRAIN_PER_TICK = 4;

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
  // v1.27 (Phase 4.7): the 1 MB PSRAM pre-alloc pool is gone
  // (the 200 KB get_states parse is replaced by a per-area
  // render_template subscription). Free heap is ~1 MB larger
  // at idle and the SDIO RX task no longer contends with us
  // for the heap lock during a multi-second parse.
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
// PSRAM-preferring JSON document allocator (used for the
// per-area aggregate event payload, ~50 KB). Heap
// allocation is fine because:
//   1. The aggregate event is much smaller than the old
//      200 KB get_states response.
//   2. ArduinoJson's internal PSRAM allocation (via
//      heap_caps_realloc_prefer) doesn't compete with the
//      SDIO RX task for the heap lock - the SDIO RX task
//      only takes the lock briefly to copy payloads.
//   3. The parse_task is on core 1 (PARSE_TASK_CORE); the
//      SDIO RX task runs on core 0. They never share
//      heaps.
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
  // v1.27 (Phase 4.7): psram_pool_.init() is gone (the
  // 1 MB pre-alloc pool is deleted). The parse_task below
  // uses HaWsPsramAllocator (heap-backed) for the
  // render_template event payloads.

  // Create parse_task. It subscribes itself to the TWDT at the
  // top of parse_task_loop_().
  // v1.27 reconnect fix: if we're being re-entered (parse_task
  // already exists), skip task creation - the existing one
  // will pick up the new client.
  if (this->parse_task_handle_ == nullptr) {
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
  }
  this->raw_queue_.set_task_to_notify(this->parse_task_handle_);

  // Start the WS client
  this->ws_state_.store(HaWsState::CONNECTING, std::memory_order_release);
  esp_websocket_client_start(this->client_);
  ESP_LOGI(HA_WS_TAG, "start: client started, parse task on core %d%s",
           PARSE_TASK_CORE, this->parse_task_handle_ != nullptr ? " (re-used)" : " (new)");
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
  // v1.27 reconnect fix: the esp_websocket_client's
  // auto-reconnect (cfg.disable_auto_reconnect = false) isn't
  // triggering in our setup - we see a single CONNECTED,
  // a 9.5s wait for auth_required, then an immediate CLOSE
  // 4ms after auth. After CLOSE, no further CONNECTED ever
  // fires. The WS stays dead and the panel loses all
  // state_changed + render_template events (clock, light
  // state, etc.).
  //
  // Workaround: schedule a manual re-start after a short
  // backoff. We use set_timeout (which runs on loopTask) so
  // we don't spin in the IDF event handler. The start() guard
  // (if (this->client_ != nullptr) return) prevents
  // double-start, so we tear down the dead client first.
  if (this->stopping_.load(std::memory_order_acquire)) return;
  if (this->client_ == nullptr) return;
  // Capture pointers locally; set_timeout runs on loopTask
  // after we're back in the IDF event loop, so we need to be
  // careful about ordering.
  esp_websocket_client_handle_t dead = this->client_;
  this->client_ = nullptr;
  this->set_timeout("ha_ws_reconnect", 2000, [this, dead]() {
    if (this->stopping_.load(std::memory_order_acquire)) return;
    if (this->client_ != nullptr) return;  // already restarted
    if (dead != nullptr) {
      esp_websocket_client_stop(dead);
      esp_websocket_client_destroy(dead);
    }
    ESP_LOGW(HA_WS_TAG, "reconnect: re-starting websocket client");
    this->start();
  });
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
          // v1.27 (Phase 4.7): the get_states round trip is
          // gone. The only "result" message we ever receive
          // now is the ack for the clock / aggregate
          // render_template subscriptions (with
          // success=true, result=null). Just log + ignore.
          if (msg->data != nullptr && msg->data[0] == '{') {
            const char* idp = strstr(msg->data, "\"id\":");
            if (idp != nullptr) {
              int idv = atoi(idp + 5);
              msg->id = (uint32_t) idv;
            }
          }
          ESP_LOGD(HA_WS_TAG, "result msg id=%u (%zu bytes), acking",
                   (unsigned) msg->id, msg->len);
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
    // v1.27 (Phase 4.7): removed the GETTING_STATES +
    // SUBSCRIBE_TIMEOUT_MS check. The state machine no longer
    // goes through GETTING_STATES (we go AUTHENTICATED -> READY
    // directly after subscribe_clock_/subscribe_aggregate_).
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
  // v1.27 (Phase 4): the clock + aggregate render_template
  // subscriptions replace both the get_states round trip
  // AND the state_changed subscription. No more 200KB bulk
  // parse, no more PSRAM pre-alloc pool. The aggregate
  // subscription's first event arrives within a few hundred
  // ms; the panel's entities are populated by the one-shot
  // fetch_room_aggregates_() in start_discovery_() before
  // the WS even opens, so the UI is never empty.
  if (this->parent_ != nullptr) {
    this->parent_->setup_render_template_subscriptions_();
  }
  // Mark the panel READY now - the entity data is already
  // in entities_by_area_ (from fetch_room_aggregates_()),
  // and the WS push will keep it fresh.
  this->ws_state_.store(HaWsState::READY, std::memory_order_release);
}

inline void HaWsClient::parse_auth_invalid_(const char* json, size_t len) {
  ESP_LOGE(HA_WS_TAG, "auth_invalid: %.*s", (int) len, json ? json : "(null)");
  this->ws_state_.store(HaWsState::DISCONNECTED, std::memory_order_release);
  if (this->client_ != nullptr) {
    esp_websocket_client_stop(this->client_);
  }
}

// v1.27 (Phase 4.7): the get_states round trip and its
// 200KB parse are GONE. The per-area aggregate template
// (subscribed via WS render_template) replaces them. The
// function declaration is kept (so the parse_task dispatch
// can be updated cleanly in a future pass) but the body
// is empty - no WS message with id=GET_STATES_ID (1)
// is ever sent, so the dispatch never reaches here.
inline void HaWsClient::parse_get_states_result_(const char* /*json*/, size_t /*len*/) {
  // Phase 4.7: this method is unreachable. Kept as an
  // empty stub to minimize the diff; safe to delete in
  // the next pass along with the parse_task dispatch.
}


// v1.27 (Phase 4.7): the state_changed subscription is gone.
// All WS events are now render_template pushes (clock +
// aggregate). The dispatch is purely by the subscription id
// in doc["id"]. A typical event payload looks like:
//
//   {"id":N, "type":"event",
//    "event": {"result": "<rendered string>",
//              "listeners": {...}}}
//
// For errors, the payload is:
//   {"id":N, "type":"event",
//    "event": {"error": "UndefinedError: ...", "level": "ERROR"}}
inline void HaWsClient::parse_event_message_(const char* json, size_t len) {
  if (json == nullptr || len == 0) return;
  this->events_received_.fetch_add(1, std::memory_order_relaxed);

  ArduinoJson::JsonDocument doc(new HaWsPsramAllocator());

  DeserializationError err = deserializeJson(doc, json, len);
  if (err) {
    ESP_LOGW(HA_WS_TAG, "event parse failed (%zu bytes): %s",
             len, err.c_str());
    return;
  }

  const char* type = doc["type"] | "";
  if (strcmp(type, "event") != 0) return;
  uint32_t sub_id = doc["id"] | 0;
  JsonObject ev = doc["event"].as<JsonObject>();
  if (ev.isNull()) {
    ESP_LOGW(HA_WS_TAG, "event has no 'event' object");
    return;
  }
  if (ev["error"].is<const char*>()) {
    ESP_LOGW(HA_WS_TAG, "render_template id=%u error: %s",
             (unsigned) sub_id, ev["error"].as<const char*>());
    return;
  }
  // v1.27: HA's WS render_template event re-parses the
  // rendered value as JSON before sending. The aggregate
  // template uses `to_json` to build the final string, but
  // HA's re-parser converts it back to a JsonObject on the
  // wire. The clock template (plain string) arrives as a
  // const char* (the "happy path"). For the aggregate, we
  // need to serialize the JsonObject back to a JSON string
  // before dispatching to the callback.
  const char* result = ev["result"].as<const char*>();
  if (result == nullptr) {
    if (ev["result"].is<JsonObject>()) {
      // v1.27: allocate a std::string once here, pass the
      // c_str to the callback, and let the callback do its
      // own copy. The std::string temporary goes out of
      // scope when this if-block exits, but the callback
      // has already copied the data into its own queue
      // slot (ev->json = json in push_aggregate_event_).
      // The as-of-v1.27 path was meant to avoid this case
      // entirely, but the test confirmed HA re-parses
      // to_json output. Net cost: 2x 25KB allocations per
      // aggregate event (1x serializeJson here, 1x deep
      // copy in the callback) vs the pre-v1.27 3x.
      // Both buffers are PSRAM-backed (PsramStlAllocator)
      // because the parse_task's internal heap is too small
      // to hold two 25KB strings at once without
      // std::bad_alloc panics.
      std::basic_string<char, std::char_traits<char>, PsramStlAllocator<char>> result_str;
      serializeJson(ev["result"].as<JsonObject>(), result_str);
      if (this->parent_ == nullptr) return;
      this->parent_->template_api_.on_ws_event_(sub_id, result_str.c_str());
      esphome::App.wake_loop_threadsafe();
      return;
    }
    ESP_LOGW(HA_WS_TAG, "event id=%u has no result", (unsigned) sub_id);
    return;
  }
  if (this->parent_ == nullptr) return;
  // Dispatch via the TemplateApi. The callback takes a
  // const char* (not std::string&) and copies the data into
  // its own queue slot (clock_queue_/agg_queue_) in a
  // single allocation, then the parse_task is done with
  // the buffer. The on_ws_event_ callback passes the
  // pointer through - the queue push does the deep copy.
  this->parent_->template_api_.on_ws_event_(sub_id, result);
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

// v1.27 (Phase 4.7): send_get_states_ and send_subscribe_events_
// are GONE. The clock + aggregate render_template subscriptions
// (sent via TemplateApi::subscribe() in
// setup_render_template_subscriptions_) replace the get_states
// round trip AND the state_changed subscription.

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

// v1.27: push a rendered clock string to the clock queue.
// Called from the TemplateApi::subscribe callback (parse_task).
// Returns false on full queue; events are rare (1/min) so a
// drop is logged and the next push will succeed.
inline bool HaWsClient::push_clock_event_(const char* rendered) {
  if (rendered == nullptr) return false;
  ClockEvent* ev = this->clock_pool_.allocate();
  if (ev == nullptr) {
    this->events_dropped_.fetch_add(1, std::memory_order_relaxed);
    return false;
  }
  ev->rendered = rendered;
  if (!this->clock_queue_.push(ev)) {
    this->events_dropped_.fetch_add(1, std::memory_order_relaxed);
    ev->release();
    this->clock_pool_.release(ev);
    return false;
  }
  return true;
}

// v1.27: drain clock events on loopTask. Bounded to
// CLOCK_DRAIN_PER_TICK (4) per tick; the clock only fires
// once per minute so 4 is far more than needed.
inline void HaWsClient::drain_clock_events() {
  size_t n = 0;
  ClockEvent* ev;
  while (n < CLOCK_DRAIN_PER_TICK && (ev = this->clock_queue_.pop()) != nullptr) {
    if (this->parent_ != nullptr) {
      this->parent_->on_clock_update_(ev->rendered);
    }
    ev->release();
    this->clock_pool_.release(ev);
    n++;
  }
}

// v1.27: same as push_clock_event_ but for the per-area
// aggregate JSON. Used in Phase 4.
inline bool HaWsClient::push_aggregate_event_(const char* json) {
  if (json == nullptr) return false;
  AggregateEvent* ev = this->agg_pool_.allocate();
  if (ev == nullptr) {
    this->events_dropped_.fetch_add(1, std::memory_order_relaxed);
    return false;
  }
  ev->json = json;
  if (!this->agg_queue_.push(ev)) {
    this->events_dropped_.fetch_add(1, std::memory_order_relaxed);
    ev->release();
    this->agg_pool_.release(ev);
    return false;
  }
  return true;
}

inline void HaWsClient::drain_aggregate_events() {
  size_t n = 0;
  AggregateEvent* ev;
  while (n < AGG_DRAIN_PER_TICK && (ev = this->agg_queue_.pop()) != nullptr) {
    if (this->parent_ != nullptr) {
      // Pass the PSRAM-backed string_view directly. No
      // 25KB std::string copy on the internal heap - the
      // view points into ev->json which is in PSRAM and
      // stays alive until ev->release() below. The consumer
      // (apply_room_aggregates_) takes a string_view and
      // calls deserializeJson on it.
      this->parent_->on_aggregate_update_(
          std::string_view(ev->json.data(), ev->json.size()));
    }
    ev->release();
    this->agg_pool_.release(ev);
    n++;
  }
}

}  // namespace esphome::ha_autopanel
