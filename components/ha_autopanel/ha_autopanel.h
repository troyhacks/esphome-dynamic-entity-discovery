#pragma once

#include <string>
#include <vector>
#include <map>
#include <set>
#include "esphome/core/component.h"
#include "esphome/components/lvgl/lvgl_proxy.h"
#include "esphome/components/http_request/http_request.h"
#include "esphome/components/wifi/wifi_component.h"
#include "esphome/components/web_server_base/web_server_base.h"
#include "esp_http_server.h"  // for AsyncWebHandler / AsyncWebServerRequest
#include "esphome/core/preferences.h"

namespace esphome {
namespace ha_autopanel {

// Area structure from HA
struct Area {
  std::string area_id;
  std::string name;
  std::vector<std::string> entity_ids;  // Entity IDs in this area
};

// Entity structure from HA
struct Entity {
  std::string entity_id;
  std::string name;
  std::string domain;
  std::string area_id;
  std::string icon;
  std::string state;        // "on" / "off" / etc.
  uint8_t brightness{0};    // 0-255, valid only when state == "on" and has_brightness
  bool has_brightness{false};
  bool is_hue_group{false};
};

// Room card data for UI
struct RoomCard {
  Area area;
  std::vector<Entity> entities;
  int grid_index{0};
  int x{0};
  int y{0};
  uint32_t color;
};

// Per-arc registry entry - replaces heap-allocated ArcCallbackData.
// Lives for the component lifetime; index is stable per render but the
// vector is cleared at the top of each create_*_ call that appends to it.
struct ArcRecord {
  std::string entity_id;   // empty for room-level arc
  std::string area_id;     // always populated
  lv_obj_t* pct_label{nullptr};  // the %-label widget, may be null (room arc)
  bool is_room_arc{false};  // true = room-level big 240px arc
};

// Per-button/toggle registry entry.
struct ControlRecord {
  std::string entity_id;       // empty for room-level controls
  std::string area_id;         // empty for per-entity controls
  std::string domain;          // "light", "switch", etc.
  lv_obj_t* btn{nullptr};
  lv_obj_t* state_label{nullptr};
};

// Persisted config struct (NVS). Char arrays, not std::string, so the
// type is trivially_copyable (required by global_preferences).
struct StoredConfig {
  char api_url[128];
  char api_token[256];
  bool configured;
  uint8_t reserved[7];  // pad to 4-byte alignment for NVS
};
static_assert(sizeof(StoredConfig) <= 512, "StoredConfig must fit in a single NVS blob");

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

 protected:
  std::string ha_api_url_;
  std::string ha_api_password_;
  http_request::HttpRequestComponent* http_request_{nullptr};
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
  std::map<std::string, std::vector<Entity>> entities_by_area_;
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
  std::set<std::string> subscribed_entity_ids_;

  // Panel state machine
  PanelState state_{PanelState::BOOTING};
  lv_obj_t* status_container_{nullptr};  // Holds the BOOTING / SETUP_REQUIRED / etc. screens
  lv_obj_t* status_title_{nullptr};
  lv_obj_t* status_message_{nullptr};
  lv_obj_t* status_retry_btn_{nullptr};

  // Authorization probe state
  bool auth_probe_pending_{false};
  uint32_t auth_probe_started_ms_{0};
  static constexpr uint32_t AUTH_PROBE_TIMEOUT_MS = 5000;
  static constexpr uint32_t AUTH_PROBE_CALL_ID = 0xA1701ACE;  // unique probe id

  // Persistent config (LittleFS at /storage/autopanel.cfg)
  bool config_loaded_{false};
  std::string config_path_{"/storage/autopanel.cfg"};

  // Web UI handler
  bool web_handler_registered_{false};
  void register_web_handler_();
  void handle_setup_get_(class AsyncWebServerRequest *request);
  void handle_setup_post_(class AsyncWebServerRequest *request);

  // LittleFS helpers
  bool mount_storage_();
  bool read_config_file_(std::string &out);
  bool write_config_file_(const std::string &body);
  void apply_config_file_(const std::string &body);
  void apply_runtime_config_();

  static const uint32_t ROOM_COLORS_[];
  static const int MAX_ROOM_COLORS_ = 8;

  void fetch_areas_();
  void fetch_entities_();
  void fetch_entities_for_area_(const Area& area);
  void filter_and_build_room_cards_();
  bool is_area_excluded_(const std::string& area_name) const;
  bool is_entity_excluded_(const std::string& entity_id) const;
  bool is_domain_included_(const std::string& domain) const;
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
  void call_ha_service_(const std::string& service,
                        const std::string& target_type,
                        const std::string& target_id,
                        int brightness_pct);

  // Local computation - replaces the per-room HA templates
  bool is_room_any_light_on_(const std::string& area_id,
                             const std::string& exclude_entity_id = "") const;
  uint8_t compute_room_brightness_pct_(const std::string& area_id) const;

  // Native API state subscription
  void subscribe_to_all_entities_();
  void on_entity_state_changed_(const std::string& entity_id, const char* state);
  void on_entity_attribute_changed_(const std::string& entity_id, const char* value);
  void update_room_card_visual_state_for_entity_(const std::string& entity_id);
  void update_room_card_visual_state_for_area_(const std::string& area_id);
  std::string find_area_id_for_entity_(const std::string& entity_id) const;

  // Panel state machine
  void set_panel_state_(PanelState new_state);
  void show_status_screen_(const char* title, const char* message, bool show_retry);
  void hide_status_screen_();
  // Build the appropriate "how to reach this device" message for the
  // SETUP_REQUIRED screen. Picks the right info based on whether WiFi is
  // connected (use use_address) or in AP fallback (use AP SSID/password).
  std::string build_setup_message_();

  // Boot: mount LittleFS, read config, decide what state to start in.
  void boot_from_storage_();
  void probe_authorization_();
  void on_auth_probe_response_(bool success, const char* error);
  void loop() override;  // for auth-probe timeout
};

}  // namespace ha_autopanel
}  // namespace esphome
