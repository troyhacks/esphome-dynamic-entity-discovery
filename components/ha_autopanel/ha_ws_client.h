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
// Implementation lives in ha_ws_client.cpp (not inline in this
// header) because the methods access HaAutoPanel's members
// directly and need the full class definition. esphome compiles
// every .cpp in the component dir alongside the main
// ha_autopanel.cpp.

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

namespace esphome::ha_autopanel {

// Forward declare the parent class. ha_autopanel.h (which
// includes this file) will have the full definition visible
// by the time any .cpp uses HaWsClient members.
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
  static constexpr int    PARSE_TASK_CORE    = 0;        // off loopTask's core 1
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
};

}  // namespace esphome::ha_autopanel
