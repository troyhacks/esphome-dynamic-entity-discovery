#pragma once

#include <string>
#include <string_view>
#include <vector>
#include <deque>
#include <memory>
#include <map>
#include <set>
#include <functional>
#include "esphome/core/component.h"
#include "esphome/components/lvgl/lvgl_proxy.h"
#include "esphome/components/http_request/http_request.h"
#include "esphome/components/wifi/wifi_component.h"
#include "esphome/components/web_server_base/web_server_base.h"
#include "esphome/components/time/real_time_clock.h"
#include "esp_http_server.h"  // for AsyncWebHandler / AsyncWebServerRequest
#include "esphome/core/preferences.h"
#include "esp_heap_caps.h"

namespace esphome {
namespace ha_autopanel {

// ----------------------------------------------------------------------------
// Memory infrastructure
// ----------------------------------------------------------------------------

// STL allocator that uses heap_caps_malloc_prefer(PSRAM, INTERNAL).
// On boards with PSRAM (Freenove N16R8V, ESP32-P4, etc.) the storage
// lands in the 8MB+ OPI/QSPI PSRAM. On boards without PSRAM it falls
// back to internal SRAM - same call site, no branch. This means the
// same binary works on both board types without #ifdefs.
//
// Without this, std::vector<Entity> on a small-heap board (ESP32-S3
// ~384KB internal) crashes during push_back when a single room has
// many entities (Garage in the user's HA install: 138 entities).
// log2(N) reallocations fragment the heap; reserve() fixes the
// churn (see filter_and_build_room_cards_) and this allocator
// moves the storage out of the precious internal heap entirely.
//
// ESPHome builds with -fno-exceptions, so we cannot throw
// std::bad_alloc on OOM. Instead we log via ESP_LOGE and abort();
// the panic gets a backtrace pointing here, which is much more
// useful than a later null-deref inside std::vector.
template <typename T>
struct PsramStlAllocator {
  using value_type = T;

  T *allocate(size_t n) __attribute__((nothrow)) {
    if (n == 0) return nullptr;
    if (n > (SIZE_MAX / sizeof(T))) {
      ESP_LOGE("psram_alloc", "PsramStlAllocator: overflow n=%zu * sizeof=%zu",
               n, sizeof(T));
      abort();
    }
    void *p = heap_caps_malloc_prefer(
        n * sizeof(T),
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT,
        MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (p == nullptr) {
      ESP_LOGE("psram_alloc", "PsramStlAllocator: OOM n=%zu * sizeof=%zu (no PSRAM and no internal free)",
               n, sizeof(T));
      abort();
    }
    return static_cast<T *>(p);
  }

  void deallocate(T *p, size_t /*n*/) noexcept {
    if (p != nullptr) heap_caps_free(p);
  }

  // C++17 allocator-awareness: vector<Pair, Alloc> needs rebind to
  // vector<Value, Alloc> when constructed from a different element type.
  template <typename U>
  struct rebind { using other = PsramStlAllocator<U>; };

  // Stateless - all PsramStlAllocator<T> instances are interchangeable.
  bool operator==(const PsramStlAllocator &) const noexcept { return true; }
  bool operator!=(const PsramStlAllocator &) const noexcept { return false; }
};

// Stable-address string arena. Entity's string fields are
// std::string_view (16 B each) pointing into this arena instead of
// std::string (32 B each, 5x = 160 B). For 419 entities the savings
// are ~30 KB on a 384 KB heap.
//
// Stability: std::deque<std::string> is a sequence of fixed-size
// chunks. push_back only adds new chunks; existing elements are not
// moved. std::string::data() is stable after construction (SSO buffer
// is inline, heap-allocated payload is freed only with the string).
// Therefore string_views returned from intern() stay valid for the
// component's lifetime. Each std::string itself owns its own
// allocation (or uses SSO) - this arena doesn't try to share the
// payload, it just gives Entity cheap reference storage.
class StringArena {
 public:
  // Intern a C string + length. Returns a view into arena-owned
  // storage; the view remains valid until the arena is destroyed
  // (component lifetime).
  std::string_view intern(const char *s, size_t n) {
    if (s == nullptr || n == 0) return std::string_view();
    // Special case: empty string is common (e.g. entities with no
    // icon). Hand back a view of a static "" to avoid burning arena
    // slots on duplicates.
    if (n == 0) return std::string_view("", 0);
    // Construct the std::string in the deque. If its payload is
    // short enough for SSO, no separate alloc happens; the SSO
    // buffer is part of the string struct inside the deque slot
    // and is therefore stable. If longer, std::string allocates
    // its own buffer from the default heap, which is also stable
    // as long as the std::string is alive.
    strings_.emplace_back(s, n);
    return std::string_view(strings_.back().data(), n);
  }

  std::string_view intern(const std::string &s) {
    return intern(s.data(), s.size());
  }

  std::string_view intern(const char *s) {
    return intern(s == nullptr ? "" : s, s == nullptr ? 0 : strlen(s));
  }

  // Intern a string_view directly (for substr() / substr(0, dot) etc.).
  // Implementation note: substr() on a std::string returns a new
  // std::string, but on a std::string_view it returns another
  // string_view into the same backing. We need a null-terminated
  // copy, so we round-trip through std::string. This is a single
  // allocation per intern() call; the std::string is constructed
  // in-place in the deque slot and the allocation cost is
  // amortized to one heap_malloc per unique string.
  std::string_view intern(std::string_view v) {
    if (v.empty()) return std::string_view();
    strings_.emplace_back(v.data(), v.size());
    return std::string_view(strings_.back().data(), v.size());
  }

  // For diagnostics.
  size_t size() const { return strings_.size(); }
  size_t bytes() const {
    size_t total = 0;
    for (const auto &s : strings_) total += s.size();
    return total;
  }

 private:
  std::deque<std::string> strings_;
};

// One global arena, owned by the singleton instance. The Entity
// string_views are valid as long as this arena is, which is the
// lifetime of the HaAutoPanel instance (and the component lives
// for the device's uptime).
inline StringArena &entity_arena() {
  static StringArena arena;
  return arena;
}

// Area structure from HA
struct Area {
  std::string area_id;
  std::string name;
  std::vector<std::string> entity_ids;  // Entity IDs in this area
};

// Entity structure from HA.
//
// The five string fields are std::string_view (16 B each) backed by
// entity_arena(). This shrinks Entity from ~168 B to ~88 B and keeps
// all the heavy string payloads in a single arena container. The
// state field stays std::string because it changes during runtime
// (HA pushes state updates; arena_emplace would invalidate any
// existing string_view pointing at the same Entity).
struct Entity {
  std::string_view entity_id;
  std::string_view name;
  std::string_view domain;
  std::string_view area_id;
  std::string_view icon;
  std::string state;        // "on" / "off" / etc. - mutates at runtime
  uint8_t brightness{0};    // 0-255, valid only when state == "on" and has_brightness
  bool has_brightness{false};
  bool is_hue_group{false};
};

// Room card data for UI
struct RoomCard {
  Area area;
  std::vector<Entity, PsramStlAllocator<Entity>> entities;
  int grid_index{0};
  int x{0};
  int y{0};
  uint32_t color;
};

// Per-arc registry entry - replaces heap-allocated ArcCallbackData.
// Lives for the component lifetime; index is stable per render but the
// vector is cleared at the top of each create_*_ call that appends to it.
//
// string_view fields are populated directly from Entity fields
// (which are also string_views into entity_arena()) - the
// assignment is a bitwise pointer copy, no allocation. The views
// are valid as long as the arena is (component lifetime).
struct ArcRecord {
  std::string_view entity_id;   // empty for room-level arc
  std::string_view area_id;     // always populated
  lv_obj_t* pct_label{nullptr};  // the %-label widget, may be null (room arc)
  bool is_room_arc{false};  // true = room-level big 240px arc
};

// Per-button/toggle registry entry.
struct ControlRecord {
  std::string_view entity_id;       // empty for room-level controls
  std::string_view area_id;         // empty for per-entity controls
  std::string_view domain;          // "light", "switch", etc.
  lv_obj_t* btn{nullptr};
  lv_obj_t* state_label{nullptr};
};

// Persisted config struct (NVS). Char arrays, not std::string, so the
// type is trivially_copyable (required by global_preferences).
// Note: hidden_rooms / hidden_entities / room_order / entity_order
// are stored separately as a JSON blob (CustomizationConfig) on
// LittleFS, NOT in this struct, because the size would blow past
// the NVS blob limit (508 bytes).
struct StoredConfig {
  char api_url[128];
  char api_token[256];
  bool configured;
  uint8_t reserved[7];  // pad to 4-byte alignment for NVS
};
static_assert(sizeof(StoredConfig) <= 512, "StoredConfig must fit in a single NVS blob");

// User customizations persisted as JSON on LittleFS at
// /storage/customizations.cfg. Plain key-value fields, not NVS.
struct CustomizationConfig {
  std::set<std::string, std::less<>> hidden_rooms;
  std::set<std::string, std::less<>> hidden_entities;
  std::vector<std::string> room_order;
  // entity_order is per-area, keyed by area_id
  std::map<std::string, std::vector<std::string>> entity_order;
  bool loaded{false};
};

// Panel-level state machine. Drives the home screen. The room grid only
// renders when state is READY.
enum class PanelState {
  BOOTING,         // initial state, before any checks have run
  SETUP_REQUIRED,  // no API token / can't reach HA / no areas found
  AUTH_FAILED,     // 401 from HA - token is wrong
  NOT_AUTHORIZED,  // token works but allow_service_calls is off
  CONNECTING,      // authenticated, fetching areas
  READY,           // rooms visible
};

class HaAutoPanel : public Component {
 public:
  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::AFTER_WIFI; }

  // Boot counter preference. Each setup() reads the prior value,
  // increments, and saves - so the gap between consecutive saved
  // values reveals unclean restarts. See setup() for the
  // reset-reason logging that pairs with this.
  ESPPreferenceObject pref_ = global_preferences->make_preference<uint32_t>(0xA0B00700);

  // Called from YAML lambda to trigger discovery after HA connects
  void trigger_discovery();

  // Called from YAML lambda to (re-)subscribe to state changes. Safe to call
  // multiple times. No-op if the API isn't connected yet or if there's
  // nothing to subscribe to.
  void trigger_subscription();

  // Called from YAML on_client_connected to fire a single probe that
  // checks whether HA is allowing this device to call services. If the
  // probe times out, the panel transitions to NOT_AUTHORIZED. If the
  // probe gets a response, the panel stays in READY (or transitions to
  // it once rooms are rendered).
  void trigger_auth_probe();

  void set_ha_api_url(const std::string& url) { this->ha_api_url_ = url; }
  void set_ha_api_password(const std::string& password) { this->ha_api_password_ = password; }
  void set_http_request(http_request::HttpRequestComponent* http) { this->http_request_ = http; }
  // Optional SNTP time source. When set, the title bar's clock and
  // the debug panel's Date/Time section read from this. When null,
  // those widgets show "--:--" / a placeholder.
  void set_time(time::RealTimeClock* t) { this->time_ = t; }
  void set_include_all(bool include_all) { this->include_all_ = include_all; }
  void set_include_areas(const std::vector<std::string>& areas) { this->include_areas_ = areas; }
  void set_exclude_areas(const std::vector<std::string>& areas) { this->exclude_areas_ = areas; }
  void set_entity_domains(const std::vector<std::string>& domains) { this->entity_domains_ = domains; }
  void set_exclude_entities(const std::vector<std::string>& entities) { this->exclude_entities_ = entities; }
  void set_card_width(int w) { this->card_width_ = w; }
  void set_card_gap(int g) { this->card_gap_ = g; }
  void set_screen_width(int w) { this->screen_width_ = w; }
  void set_screen_height(int h) { this->screen_height_ = h; }
  void set_start_x(int x) { this->start_x_ = x; }
  void set_start_y(int y) { this->start_y_ = y; }
  void set_default_on_pct(int pct) { this->default_on_pct_ = pct; }
  // AGENT_DEBUG opt-in switch. When true, /autopanel/test/* HTTP
  // endpoints are registered (click, scroll, cmd, state). Default
  // false so a production build does not expose them. See
  // __init__.py for the full security rationale.
  void set_agent_debug(bool v) { this->agent_debug_ = v; }

 protected:
  std::string ha_api_url_;
  std::string ha_api_password_;
  http_request::HttpRequestComponent* http_request_{nullptr};
  // Optional time source - set by the YAML's `time_ref` (via the
  // set_time setter) when the user wires up a SNTP block.
  time::RealTimeClock* time_{nullptr};
  // Cached friendly name of HA's zone.home entity, shown centered in
  // the title bar on the grid page. Populated by fetch_home_name_().
  std::string home_name_;
  // Wall-clock timestamp (millis()) of the last successful home-name
  // fetch. Used to throttle periodic refresh in loop() and to skip
  // redundant work when a state change re-triggers the fetch.
  uint32_t last_home_fetch_ms_{0};
  bool include_all_{true};
  std::vector<std::string> include_areas_;
  std::vector<std::string> exclude_areas_;
  std::vector<std::string> entity_domains_{"light"};
  std::vector<std::string> exclude_entities_;
  // Auto-fit square-card layout. card_width is both the width and height
  // (square cards). cards_per_row is computed from screen_width and
  // card_gap. main_container_ height is computed from card count.
  int card_width_{250};
  int card_gap_{12};
  int screen_width_{1024};
  int screen_height_{600};
  int start_x_{10};
  int start_y_{12};
  int default_on_pct_{30};

  std::vector<Area> discovered_areas_;
  // entities_by_area_ uses the PSRAM-preferring allocator so all
  // Entity storage (the bulk of ha_autopanel's RAM) lands in PSRAM
  // on boards that have it, and falls back to internal SRAM on
  // boards that don't. The Entity structs themselves use
  // string_view fields pointing into entity_arena(), so the
  // std::string payloads share a single arena container.
  std::map<std::string, std::vector<Entity, PsramStlAllocator<Entity>>> entities_by_area_;
  std::vector<RoomCard> room_cards_;

  // UI state - stored for navigation between room grid and entity detail
  lv_obj_t* main_container_{nullptr};
  lv_obj_t* detail_container_{nullptr};
  int current_room_index_{-1};

  // Per-render registries (component-owned, no heap, no LV_EVENT_DELETE)
  std::vector<ArcRecord> arc_records_;
  std::vector<ControlRecord> control_records_;

  // Live-update widget maps (keyed by stable string id)
  std::map<std::string, lv_obj_t*> room_arc_widgets_;                  // area_id -> big 240px arc
  std::map<std::string, std::pair<lv_obj_t*, lv_obj_t*>> room_btn_widgets_;  // area_id -> (btn, label)
  std::map<std::string, lv_obj_t*> room_label_btn_widgets_;            // area_id -> transparent room-name button

  // Bounce-back storage: last non-zero brightness for each area
  std::map<std::string, uint8_t> last_brightness_pct_;

  // Track which entity_ids we've subscribed to (avoids double-subscribe)
  // std::less<> (transparent comparator) enables heterogeneous
  // lookup so we can call .count(e.entity_id) where e.entity_id is
  // a std::string_view without allocating a std::string first.
  std::set<std::string, std::less<>> subscribed_entity_ids_;

  // Panel state machine
  PanelState state_{PanelState::BOOTING};
  // Boot splash — full-screen black with red centered "HA AutoPanel v1.0".
  // Shown immediately at the start of setup() so the user never sees a
  // bare/white display, then hidden as soon as the status screen or room
  // grid takes over. Doubles as branding and as a cover for the brief
  // display-default artifacts the panel can show before LVGL takes over.
  lv_obj_t* splash_container_{nullptr};
  lv_obj_t* splash_label_{nullptr};
  // Holds the BOOTING / SETUP_REQUIRED / etc. screens
  lv_obj_t* status_container_{nullptr};
  lv_obj_t* status_title_{nullptr};
  lv_obj_t* status_message_{nullptr};
  lv_obj_t* status_retry_btn_{nullptr};

  // Top title bar (HA status + Edit button) on the main grid page.
  // Always visible, room cards scroll under it.
  lv_obj_t* title_bar_{nullptr};
  // Right-side cluster of the title bar - a flex-row sub-container
  // that holds Time, Sort, Cancel, Debug, Edit (in that left-to-right
  // order, with hidden children collapsing out of the layout).
  // Positioned at LV_ALIGN_RIGHT_MID on the title bar with a 12px
  // right margin; the flex layout auto-sizes around whatever
  // children are currently visible, so the whole bar reflows when
  // the user changes screen_width in YAML.
  lv_obj_t* title_right_cluster_{nullptr};
  lv_obj_t* title_status_dot_{nullptr};
  lv_obj_t* title_status_label_{nullptr};
  // title_edit_btn_ and title_cancel_btn_ were removed in v1.11
  // (Edit-mode redundancy cleanup). The Sort panel now handles
  // both show/hide and reorder; there is no separate inline
  // Edit mode with X badges on each room card. See the
  // v1.11 commit message for the full rationale.
  lv_obj_t* title_back_btn_{nullptr};  // shown only on the entity detail page
  lv_obj_t* title_room_label_{nullptr};  // room name in the center of the title bar (detail page only)
  // HA zone.home friendly_name, fetched at runtime and shown in the
  // center of the title bar on the main grid page. Hidden on the
  // detail page (where title_room_label_ takes that spot) and during
  // the splash / status screens. Cached in home_name_ so the title
  // bar has something to show on the next boot before the network
  // call returns.
  lv_obj_t* title_home_label_{nullptr};
  // Local clock, populated by the SNTP time component (yaml id
  // `sntp_time`) and refreshed every loop() tick (~1Hz). Sits to
  // the right of the home name, just left of the Edit button.
  // Shows "--:--" until NTP sync succeeds.
  lv_obj_t* title_time_label_{nullptr};
  // "DBG" button in the title bar. Shown when the panel is in any
  // non-READY state (the user needs to diagnose AUTH_FAILED /
  // NOT_AUTHORIZED / SETUP_REQUIRED / CONNECTING).
  // Tapping it opens debug_panel_, the bottom-anchored overlay that
  // shows WiFi / HA / Customizations / Device / Date+Time status and
  // exposes action buttons (re-probe auth, re-run discovery, reset
  // customizations, reboot).
  lv_obj_t* title_debug_btn_{nullptr};
  // Bottom-anchored overlay, ~70% screen height, full width. Created
  // lazily on the first tap of title_debug_btn_ (one-time per
  // session) and rebuilt on every show so live status is always
  // fresh. Hidden by default. Follows the same lazy-create pattern
  // as the sort panel below.
  lv_obj_t* debug_panel_{nullptr};
  // True between the first and second tap of a 2-tap confirm button
  // (reboot or reset). Set to "" / cleared after 5s of inactivity or
  // when the second tap fires. Maps to a colour change on the
  // pending button so the user has a visible cue that a confirmation
  // tap will actually execute the action.
  std::string pending_action_;
  uint32_t pending_action_started_ms_{0};
  static constexpr uint32_t PENDING_ACTION_TIMEOUT_MS = 5000;
  // Sort & hide button in the title bar. Visible only in edit mode.
  // Tapping it opens sort_panel_, a full-screen list view of every
  // room with up/down arrow buttons and a Hide/Show toggle per row.
  // The user sorts and toggles there, then taps Apply to persist to
  // /storage/customizations.cfg and re-render the grid. This is the
  // recommended way to reorder and hide rooms (replaces the
  // older "Show hidden (N)" button + hidden_panel_ + drag-to-reorder
  // gesture paths, which the user found jumpy and didn't save).
  lv_obj_t* title_sort_btn_{nullptr};
  // Full-screen sort & hide overlay. Created lazily on the first tap
  // of title_sort_btn_, rebuilt every show so live state is current.
  // Lives as a child of the screen. Same lazy-create / rebuild-on-show
  // pattern as debug_panel_.
  lv_obj_t* sort_panel_{nullptr};
  // Local copy of the room order while the sort panel is open. On
  // Apply, this is what we persist to customizations_.room_order.
  // Cleared on close.
  std::vector<std::string> sort_local_order_;
  // Local copy of which rooms are hidden while the sort panel is
  // open. On Apply, this is what we persist to customizations_.
  // hidden_rooms. Cleared on close.
  std::set<std::string> sort_local_hidden_;
  // v1.11: edit_mode_, in_edit_session_, and edit_baseline_ were
  // removed. The Sort panel is now the single customization entry
  // point, so there is no need for a separate "Edit" toggle on the
  // title bar. The X badge that used to appear on each room card
  // in edit mode is gone too - the Sort panel's per-row hide
  // checkbox is the only hide path now.
  // Number of rooms currently visible (for the status line)
  int visible_room_count_{0};

  // Active media player banner. A horizontal strip at the top of the
  // grid (just below the title bar) that surfaces any media_player
  // entity currently in 'playing' state. Tapping the banner pauses
  // the media player. Hidden when no media is playing.
  //
  // Design: a single banner row holds up to N "tile" widgets, one
  // per currently-playing media_player. Each tile shows the friendly
  // name + a small pause icon. The whole banner row is hidden
  // (LV_OBJ_FLAG_HIDDEN) when there are no active tiles.
  //
  // The banner's visibility is recomputed from on_entity_state_changed_
  // (real-time, when state-sync works) and from refresh_room_cards_()
  // (initial render, and after hide/reveal).
  //
  // When the state-sync bug is open (state_changed never fires), the
  // banner shows whatever entities have state set at fetch time. This
  // is documented in [[project_state_sync_bug]].
  lv_obj_t* media_banner_{nullptr};
  // Map: entity_id (as string) -> tile widget. Cleared on rebuild.
  std::map<std::string, lv_obj_t*> media_tiles_;

  // Authorization probe state
  bool auth_probe_pending_{false};
  uint32_t auth_probe_started_ms_{0};
  static constexpr uint32_t AUTH_PROBE_TIMEOUT_MS = 5000;
  static constexpr uint32_t AUTH_PROBE_CALL_ID = 0xA1701ACE;  // unique probe id

  // Persistent config (LittleFS at /storage/autopanel.cfg)
  bool config_loaded_{false};
  std::string config_path_{"/storage/autopanel.cfg"};

  // User customizations (LittleFS at /storage/customizations.cfg).
  // Separate from the API token config because it's larger and more
  // frequently changed.
  CustomizationConfig customizations_;
  std::string customizations_path_{"/storage/customizations.cfg"};
  // v1.12: tracks which areas fetch_entities_() has already populated
  // during a single bulk-fetch pass, so the bucket is cleared
  // exactly once per area. The state-subscription push in
  // on_entity_state_changed_() would otherwise append a duplicate
  // copy of every entity. Reset at the start of each fetch pass
  // (top of fetch_entities_()).
  std::set<std::string> seen_areas_during_bulk_fetch_;

  // Web UI handler
  bool web_handler_registered_{false};
  // AGENT_DEBUG opt-in. When true, /autopanel/test/* handlers are
  // registered and respond. Default false (set in the schema), so
  // production builds do not expose the test API.
  bool agent_debug_{false};
  void register_web_handler_();
  void handle_setup_get_(class AsyncWebServerRequest *request);
  void handle_setup_post_(class AsyncWebServerRequest *request);
  void handle_setup_reset_(class AsyncWebServerRequest *request);
  void handle_customizations_get_(class AsyncWebServerRequest *request);
  void handle_customizations_post_(class AsyncWebServerRequest *request);
  // Screenshot handler. Returns a BMP file of the current display
  // buffer. The Crowpanel has 32MB PSRAM, so a 1.2MB single-screen
  // BMP fits without issue. The handler is registered at
  // GET /autopanel/screenshot.bmp. Note this is NOT the LVGL
  // snapshot API (LV_USE_SNAPSHOT is off in this build); we
  // directly read the display's draw buffer (which is populated
  // by lv_refr_now()).
  void handle_screenshot_(class AsyncWebServerRequest *request);
  // JPEG screenshot handler. Returns a JPEG-compressed screenshot
  // suitable for direct viewing by an AI agent (no PIL conversion
  // needed). Uses the P4's hardware JPEG encoder when available
  // (SOC_JPEG_ENCODE_SUPPORTED); on chips without it (S2, S3,
  // C3, C6) the handler returns 501 Not Implemented so the test
  // harness can fall back to /autopanel/screenshot.bmp.
  // The LVGL display buffer is RGB565; the handler converts to
  // RGB888 in-place into a PSRAM scratch buffer before encoding.
  // Output quality is fixed at 80 (visually lossless for UI text
  // and arcs, ~5-10x smaller than the BMP at 1024x600).
  void handle_screenshot_jpg_(class AsyncWebServerRequest *request);
  // Simulated input device (test harness helpers). Both drive the
  // default LVGL input device (the GT911 touchscreen) by injecting
  // press/release + point events. Triggered by 'C' and 'S' serial
  // commands (see loop()) and exercised by send_cmd.py click /
  // scroll subcommands. The same functions are also called from
  // the /autopanel/test/click and /autopanel/test/scroll HTTP
  // handlers when agent_debug_ is true.
  void simulate_click_(int x, int y);
  void simulate_scroll_(int x1, int y1, int x2, int y2);
  // Process a single-character command the same way the serial
  // command parser does. Extracted so the web API can trigger the
  // same state transitions ('g' for grid, '0'..'9' for detail, 'o'
  // for sort, etc.) without needing the serial port to be free.
  // Returns true if the command was recognized, false otherwise.
  bool process_command_(char c);

  // AGENT_DEBUG: opt-in /autopanel/test/* endpoints. Each handler
  // checks agent_debug_ at the top and returns 404 if disabled, so
  // the URL space is invisible to a non-test build.
  void handle_test_click_(class AsyncWebServerRequest *request);
  void handle_test_scroll_(class AsyncWebServerRequest *request);
  void handle_test_cmd_(class AsyncWebServerRequest *request);
  void handle_test_state_(class AsyncWebServerRequest *request);

  // LittleFS helpers
  bool mount_storage_();
  bool read_config_file_(std::string &out);
  bool write_config_file_(const std::string &body);
  void apply_config_file_(const std::string &body);
  void apply_runtime_config_();
  // Refresh the title-bar clock label. Reads from the SNTP time
  // component (id `sntp_time` in the user's yaml) and formats
  // "h:mm AM/PM" (or "--:--" until sync succeeds). Cheap; safe to
  // call from loop() every tick (the time string only changes once
  // per minute, so most calls early-exit without touching LVGL).
  void update_title_time_();
  // Show / hide the debug button in the title bar. Called whenever
  // the panel state changes. No-op if the button hasn't been
  // created yet (still in setup()).
  void update_debug_btn_visibility_();
  // Show / hide the debug overlay. show_debug_panel_() creates the
  // panel lazily (one-time) and rebuilds content every time it's
  // shown so live status is current. hide_debug_panel_() just sets
  // the HIDDEN flag; the widgets are reused on the next show.
  void show_debug_panel_();
  void hide_debug_panel_();
  // Build (or rebuild) the rows inside debug_panel_ from current
  // device state. Each section is a label pair: section header
  // (bold) and key:value lines. The action buttons live at the
  // bottom. Called from show_debug_panel_() and from a timer while
  // the panel is open (so the WiFi RSSI / heap values update
  // without the user having to re-open).
  void build_debug_panel_content_();
  // Reset a 2-tap confirm (called when a non-confirm button is
  // tapped, or when PENDING_ACTION_TIMEOUT_MS elapses).
  void clear_pending_action_();
  // Show / hide the sort panel. show_sort_panel_() creates the panel
  // lazily (one-time) and rebuilds content every time it's shown
  // from the current customizations_. hide_sort_panel_() just sets
  // the HIDDEN flag; widgets are reused on the next show.
  void show_sort_panel_();
  void hide_sort_panel_();
  // Build (or rebuild) the rows inside sort_panel_ from the current
  // sort_local_order_ and sort_local_hidden_ state. Each row is a
  // 60-px-tall panel with: room name, up arrow, down arrow, and a
  // hide/show toggle. The Apply button at the bottom persists the
  // local state to customizations_ and re-renders the grid.
  void build_sort_panel_content_();
  // Apply the sort panel's local state to customizations_ (room_order
  // and hidden_rooms), persist to /storage/customizations.cfg, and
  // trigger a grid refresh. Called by the Apply button.
  void apply_sort_panel_();
  // Move a room up/down in sort_local_order_ and rebuild the panel.
  // Used by the ↑/↓ button click handlers in build_sort_panel_content_().
  void sort_move_(int index, int delta);
  // GET <ha_api>/api/states, find entity_id == "zone.home", extract
  // attributes.friendly_name, cache it in home_name_ and update
  // title_home_label_. Triggered on state transition to READY and
  // throttled to one call per HOME_FETCH_INTERVAL_MS in loop(). The
  // [home] tag in the log lets the host harness verify the result.
  void fetch_home_name_();
  static constexpr uint32_t HOME_FETCH_INTERVAL_MS = 5 * 60 * 1000;  // 5 min
  // Recompute x/y/grid_index for every entry in room_cards_ in current
  // vector order (top-to-bottom, left-to-right). Call after any
  // mutation (hide, restore, future bulk import) so the grid always
  // reflects the room_cards_ vector with no gaps.
  void repack_room_cards_();
  bool read_customizations_file_();
  bool write_customizations_file_();
  void apply_customizations_file_(const std::string &body);
  // Returns true if the named room/entity is hidden in the user config
  bool is_room_hidden_(std::string_view room_name) const;
  bool is_entity_hidden_(std::string_view entity_id) const;
  // Returns the custom display order for a room/area, or an empty
  // vector if none is set (callers should fall back to alphabetical
  // or HA's order).
  const std::vector<std::string> *get_entity_order_(const std::string &area_id) const;

  static const uint32_t ROOM_COLORS_[];
  static const int MAX_ROOM_COLORS_ = 8;

  void fetch_areas_();
  void fetch_entities_();
  void fetch_entities_for_area_(const Area& area);
  void filter_and_build_room_cards_();
  // Predicate helpers take std::string_view so callers can pass
  // either an Entity field (now string_view) or a std::string from
  // other code paths. The std::string_view -> comparison is
  // allocation-free; std::string still works via implicit conversion
  // to string_view.
  bool is_area_excluded_(std::string_view area_name) const;
  bool is_entity_excluded_(std::string_view entity_id) const;
  bool is_domain_included_(std::string_view domain) const;
  void start_discovery_();
  void create_ui_from_room_cards_();
  void create_room_card_(void* parent, const RoomCard& room);
  void show_room_grid_();
  void show_entity_detail_(int room_index);
  void create_entity_control_(void* parent, const Entity& entity, int entity_index, int y_pos, uint32_t color);
  int compute_cards_per_row_() const;
  int get_card_x_(int col) const;
  int get_card_y_(int row) const;
  int arc_size_() const { return this->card_width_ - 20; }  // square, ~10px margin
  uint32_t get_room_color_(int index) const;

  // HA service call helper - uses native ESPHome API
  // service:    e.g. "light.turn_on", "light.turn_off", "switch.toggle"
  // target_type: "entity_id" or "area_id"
  // target_id:   the entity_id or area_id value
  // brightness_pct: 0-100, or -1 to omit
  // Takes string_view for the string params so we can pass Entity
  // fields and ControlRecord/ArcRecord fields without allocating
  // std::string temporaries at every call site.
  void call_ha_service_(std::string_view service,
                        std::string_view target_type,
                        std::string_view target_id,
                        int brightness_pct);

  // Local computation - replaces the per-room HA templates
  bool is_room_any_light_on_(const std::string& area_id,
                             const std::string& exclude_entity_id = "") const;
  uint8_t compute_room_brightness_pct_(const std::string& area_id) const;

  // Native API state subscription
  void subscribe_to_all_entities_();
  // Callbacks fired by the api server when HA pushes a state change
  // or attribute change. Take string_view because the const char*
  // overload of subscribe_home_assistant_state is zero-allocation
  // and the captured entity_id (a string_view into the arena) is
  // passed straight through.
  void on_entity_state_changed_(std::string_view entity_id, const char* state);
  void on_entity_attribute_changed_(std::string_view entity_id, const char* value);
  void update_room_card_visual_state_for_entity_(std::string_view entity_id);
  void update_room_card_visual_state_for_area_(const std::string& area_id);
  std::string find_area_id_for_entity_(std::string_view entity_id) const;

  // Panel state machine
  void set_panel_state_(PanelState new_state);
  void show_status_screen_(const char* title, const char* message, bool show_retry);
  void hide_status_screen_();
  // Build the appropriate "how to reach this device" message for the
  // SETUP_REQUIRED screen. Picks the right info based on whether WiFi is
  // connected (use use_address) or in AP fallback (use AP SSID/password).
  std::string build_setup_message_();

  // Update the title bar status indicator (called on every state change
  // and on api.on_client_connected / disconnected).
  void update_title_bar_();
  // Create the top title bar (HA status + Edit button + optional back
  // button) as a child of the given parent (the screen). Called from
  // both create_ui_from_room_cards_ and show_entity_detail_ so it's
  // visible on every page.
  void create_title_bar_(lv_obj_t *parent);
  // Re-render the room grid (called when customizations change,
  // when a room is hidden/shown via the Sort panel, or after
  // discovery rebuilds room_cards_).
  void refresh_room_cards_();
  // Update the "Now Playing" banner at the top of the grid. Reads
  // the current entity state fields and shows one tile per media_player
  // in 'playing' state. Lazy-creates the banner container on first
  // call. Called from create_ui_from_room_cards_() at render time
  // and from on_entity_state_changed_() for live updates.
  void update_media_banner_();

  // Boot: mount LittleFS, read config, decide what state to start in.
  void boot_from_storage_();
  void probe_authorization_();
  void on_auth_probe_response_(bool success, const char* error);
  void loop() override;  // for auth-probe timeout
};

}  // namespace ha_autopanel
}  // namespace esphome
