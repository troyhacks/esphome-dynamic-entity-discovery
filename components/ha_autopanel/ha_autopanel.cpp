#include "ha_autopanel.h"
#include "esphome/core/log.h"
#include "esphome/core/string_ref.h"
#include "esphome/components/json/json_util.h"
#include "esphome/components/api/api_server.h"
#include "esphome/components/api/api_pb2.h"
#include "esphome/components/api/homeassistant_service.h"
#include <lvgl.h>
#include "esphome/components/lvgl/lvgl_proxy.h"
#include "esp_heap_caps.h"
#include "driver/uart.h"
#include "esp_littlefs.h"

namespace esphome {
namespace ha_autopanel {

// ArduinoJson 7.x: Custom PSRAM allocator. The default JsonDocument uses
// internal-RAM malloc, which is too small for a 220KB+ states response.
// This puts the JSON DOM in PSRAM where there's plenty of room.
struct PsramAllocator : ArduinoJson::Allocator {
  void* allocate(size_t size) override {
    if (size == 0) return nullptr;
    void* p = heap_caps_malloc(size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (p == nullptr) p = malloc(size);  // fallback to internal
    return p;
  }
  void deallocate(void* p) override { if (p) free(p); }
  void* reallocate(void* p, size_t new_size) override {
    if (new_size == 0) {
      deallocate(p);
      return nullptr;
    }
    void* np = allocate(new_size);
    if (np && p) {
      // ArduinoJson 7.x: caller may or may not have copied yet. Be safe and
      // memcpy a chunk; old buffer is freed after this returns.
      memcpy(np, p, new_size);
      deallocate(p);
    }
    return np;
  }
};

using PsramJsonDocument = ArduinoJson::JsonDocument;

// Module-level static allocator (single instance shared by all parses)
static PsramAllocator s_psram_allocator;

static const char* TAG = "ha_autopanel";

const uint32_t HaAutoPanel::ROOM_COLORS_[] = {
    0xFFFF00,  // Yellow - Living Room
    0x00FFFF,  // Cyan - Kitchen
    0xFF00FF,  // Magenta - Bedroom
    0x00FF00,  // Green - Bathroom
    0xFF8800,  // Orange - Office
    0x8800FF,  // Purple - Garage
    0xFF4444,  // Red - Dining Room
    0x44FF44,  // Lime - Hallway
};

// Static pointer to component instance for use in C callbacks
static HaAutoPanel* s_instance = nullptr;

// Heap-allocated control data for room-level widgets (the big arc and the
// ON/OFF button on each room card). Each card's widget needs a stable
// pointer-to-string that outlives the LVGL widget so the click callback
// can recover the right area_id. We allocate a small struct, store its
// pointer in lv_obj_set_user_data, and free it in LV_EVENT_DELETE.
struct RoomControlData {
  std::string area_id;
  lv_obj_t* pct_label;   // null for room arc / room button
};


void HaAutoPanel::setup() {
  ESP_LOGI(TAG, "Dynamic Entity Discovery starting...");
  s_instance = this;

  ESP_LOGI(TAG, "  HA API URL: %s", this->ha_api_url_.c_str());
  ESP_LOGI(TAG, "  Layout: screen=%dx%d, cards=%dx%d px, gap=%d -> %d cards/row",
           this->screen_width_, this->screen_height_,
           this->card_width_, this->card_width_, this->card_gap_,
           this->compute_cards_per_row_());
  ESP_LOGI(TAG, "  http_request configured: %s", this->http_request_ ? "YES" : "NO");

  // Paint the LVGL screen dark immediately to avoid a white flash at boot
  // (frame buffer is uninitialized white until something draws to it).
  lv_obj_t* screen = lv_scr_act();
  if (screen != nullptr) {
    lv_obj_set_style_bg_color(screen, lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, LV_PART_MAIN);
  }

  ESP_LOGI(TAG, "Note: Use trigger_discovery() after boot to create UI");

  // Start in BOOTING state. show_status_screen_ is deferred until trigger_discovery()
  // is called, since at this point the screen size and the layout may not be
  // fully known yet.
  this->state_ = PanelState::BOOTING;

  // Mount LittleFS and read the saved config (if any). Decides which
  // state to start in - SETUP_REQUIRED if no config exists, READY if
  // the config is good.
  this->boot_from_storage_();
}

void HaAutoPanel::trigger_discovery() {
  ESP_LOGI(TAG, "trigger_discovery() called");

  if (s_instance == nullptr) {
    ESP_LOGE(TAG, "Instance not set!");
    return;
  }

  s_instance->start_discovery_();
}

void HaAutoPanel::trigger_subscription() {
  ESP_LOGI(TAG, "trigger_subscription() called");
  if (s_instance == nullptr) {
    ESP_LOGE(TAG, "Instance not set!");
    return;
  }
  // If discovery hasn't happened yet, there's nothing to subscribe to -
  // the lambda will fire again when the API reconnects post-discovery.
  if (s_instance->entities_by_area_.empty()) {
    ESP_LOGW(TAG, "No entities discovered yet; subscription will retry on next API connect");
    return;
  }
  s_instance->subscribe_to_all_entities_();
}

void HaAutoPanel::trigger_auth_probe() {
  ESP_LOGI(TAG, "trigger_auth_probe() called");
  if (s_instance == nullptr) return;
  s_instance->probe_authorization_();
}

void HaAutoPanel::fetch_areas_() {
  if (this->http_request_ == nullptr) {
    ESP_LOGE(TAG, "http_request_ is null - cannot fetch areas");
    return;
  }

  ESP_LOGI(TAG, "Fetching areas from HA via template API...");

  std::string url = this->ha_api_url_ + "/api/template";

  // Build the Jinja template dynamically, applying exclude_areas, include_areas,
  // domains, and exclude_entities filters on the HA side. This dramatically
  // reduces the payload that the ESP32 has to parse.
  //
  // Template structure:
  //   {% set ns = namespace(rooms=[]) %}
  //   {% for a in areas() %}
  //     {% if (not include_all) and (a not in include_areas) %}{% continue %}{% endif %}
  //     {% if a in exclude_areas %}{% continue %}{% endif %}
  //     {% set ents = area_entities(a)
  //         | select("match", domain_regex)
  //         | reject("in", exclude_entities_list) | list %}
  //     {% set ns.rooms = ns.rooms + [{"area_id": a, "name": area_name(a), "entities": ents}] %}
  //   {% endfor %}
  //   {{ ns.rooms | tojson }}

  // Build a list of full entity-ID prefixes for the included domains.
  // We use a regex `match` test on entity_id. The pattern needs an explicit
  // backslash-dot to avoid matching things like "lightning.*" when "light"
  // is a domain. We carefully escape the regex for JSON: each backslash
  // in the regex becomes two backslashes in the JSON body so JSON decoding
  // yields a single backslash.
  std::string domain_regex;
  for (size_t i = 0; i < this->entity_domains_.size(); i++) {
    if (i > 0) domain_regex += "|";
    domain_regex += this->entity_domains_[i];
  }
  // Build the JSON-safe version of the regex: ^(?:light|switch|...)\.
  // In the final body string, we need \\. (4 chars in C++ source: \\\\.)
  // to produce "\\\\" in the C++ runtime, which JSON-decodes to "\\",
  // which the regex engine then sees as the escape for literal dot.
  // Wait, simpler: we want the body string to contain "\\\\." so that
  // JSON decoding gives "\\." and the regex sees "\.". So in C++ source
  // we need "\\\\\\\\." (8 backslashes + dot) -> runtime is "\\\\." ->
  // JSON decodes to "\\." -> regex sees "\.". Yes.
  std::string domain_match_json = "^(?:" + domain_regex + ")\\\\\\\\.";

  // Build include/exclude area lists (HA's "in" test works on string lists).
  // The whole body is a JSON string, so inner double quotes MUST be escaped
  // as \\\" so they survive JSON decoding intact.
  std::string include_areas_list;
  for (size_t i = 0; i < this->include_areas_.size(); i++) {
    if (i > 0) include_areas_list += ", ";
    include_areas_list += "\\\"" + this->include_areas_[i] + "\\\"";
  }
  std::string exclude_areas_list;
  for (size_t i = 0; i < this->exclude_areas_.size(); i++) {
    if (i > 0) exclude_areas_list += ", ";
    exclude_areas_list += "\\\"" + this->exclude_areas_[i] + "\\\"";
  }
  std::string exclude_entities_list;
  for (size_t i = 0; i < this->exclude_entities_.size(); i++) {
    if (i > 0) exclude_entities_list += ", ";
    exclude_entities_list += "\\\"" + this->exclude_entities_[i] + "\\\"";
  }

  // Area gating condition
  std::string area_gate;
  if (!this->include_all_) {
    if (this->include_areas_.empty()) {
      // include_all=false with no include_areas would exclude everything -
      // safety: include nothing (no areas pass)
      area_gate = "{% if false %}";
    } else {
      area_gate = "{% if a in [" + include_areas_list + "] %}";
    }
  } else if (!this->exclude_areas_.empty()) {
    area_gate = "{% if a not in [" + exclude_areas_list + "] %}";
  } else {
    area_gate = "{% if true %}";
  }

  // Entity filtering: domain select, then exclude_entities reject.
  // All inner double quotes are escaped with \\\" so the whole template
  // remains a valid JSON string. The domain_match_json already contains
  // JSON-escaped backslashes, so we use it as-is.
  std::string entity_filter = "area_entities(a) | select(\\\"match\\\", \\\"" + domain_match_json + "\\\")";
  if (!this->exclude_entities_.empty()) {
    entity_filter += " | reject(\\\"in\\\", [" + exclude_entities_list + "])";
  }
  entity_filter += " | list";

  std::string body =
      "{\"template\": \"{% set ns = namespace(rooms=[]) %}"
      "{% for a in areas() %}" + area_gate +
      "{% set ents = " + entity_filter + " %}"
      "{% set ns.rooms = ns.rooms + [{ \\\"area_id\\\": a, \\\"name\\\": area_name(a), \\\"entities\\\": ents }] %}"
      "{% endif %}{% endfor %}"
      "{{ ns.rooms | tojson }}\"}";

  ESP_LOGD(TAG, "Template body length: %d", (int)body.size());

  std::vector<http_request::Header> headers = {
      {"Authorization", "Bearer " + this->ha_api_password_},
      {"Content-Type", "application/json"},
  };

  auto container = this->http_request_->post(url, body, headers);

  if (container == nullptr) {
    ESP_LOGE(TAG, "Failed to get areas: container is null");
    return;
  }

  if (container->status_code == 0) {
    ESP_LOGE(TAG, "Failed to get areas: connection failed");
    container->end();
    return;
  }

  if (container->status_code == 401) {
    ESP_LOGE(TAG, "HA API auth failed - check your token");
    container->end();
    this->set_panel_state_(PanelState::AUTH_FAILED);
    return;
  }

  if (container->status_code == 404) {
    ESP_LOGE(TAG, "HA API endpoint not found - check HA version");
    container->end();
    return;
  }

  if (container->status_code != 200) {
    ESP_LOGW(TAG, "Failed to get areas: HTTP %d", container->status_code);
    container->end();
    return;
  }

  // Read response body - reserve to content_length to avoid reallocations
  size_t expected = container->content_length;
  std::string response;
  response.reserve(expected > 0 ? expected : 16384);
  uint8_t buf[512];
  uint32_t last_data_time = millis();
  const uint32_t timeout = 15000;
  int iter_count = 0;

  while (container->get_bytes_read() < container->content_length) {
    App.feed_wdt();
    yield();
    int read = container->read(buf, sizeof(buf));
    if (read > 0) {
      response.append(reinterpret_cast<char*>(buf), read);
      last_data_time = millis();
      iter_count = 0;
    } else if (read == http_request::HTTP_ERROR_CONNECTION_CLOSED) {
      break;
    } else {
      iter_count++;
      if (iter_count % 10 == 0) {
        App.feed_wdt();
      }
    }
    if (millis() - last_data_time > timeout) {
      ESP_LOGW(TAG, "Timeout reading areas response");
      break;
    }
  }
  container->end();

  ESP_LOGI(TAG, "Areas response: %d bytes", (int)response.size());
  // Diagnostic: log first 200 chars (and last 50) to see actual format
  {
    std::string head = response.substr(0, std::min<size_t>(200, response.size()));
    ESP_LOGD(TAG, "  head: %s", head.c_str());
    if (response.size() > 50) {
      std::string tail = response.substr(response.size() - 50);
      ESP_LOGD(TAG, "  tail: %s", tail.c_str());
    }
  }

  // ArduinoJson 7.x: JsonDocument with PSRAM allocator to keep DOM off
  // the small internal heap. No filter - the filter can drop the top-level
  // array in some configurations and was previously discarding all data.
  PsramJsonDocument doc(&s_psram_allocator);

  DeserializationError parse_err = deserializeJson(doc, response);
  if (parse_err) {
    ESP_LOGE(TAG, "Failed to parse areas JSON: %s", parse_err.c_str());
    return;
  }

  JsonArray arr = doc.as<JsonArray>();
  if (arr.isNull()) {
    JsonObject obj = doc.as<JsonObject>();
    if (!obj.isNull()) {
      ESP_LOGE(TAG, "Areas response is a JSON object, keys:");
      for (JsonPair kv : obj) {
        ESP_LOGE(TAG, "  key: %s", kv.key().c_str());
      }
    } else {
      ESP_LOGE(TAG, "Areas response is neither array nor object");
    }
    return;
  }
  for (JsonObject area : arr) {
    const char* area_id = area["area_id"];
    const char* name = area["name"];
    if (area_id && name) {
      Area a;
      a.area_id = area_id;
      a.name = name;

      // Parse entity IDs array - HA's area_entities() returns array of strings
      JsonArray entities = area["entities"].as<JsonArray>();
      for (JsonVariant entity : entities) {
        if (entity.is<const char*>()) {
          // String entry: "light.kitchen_main"
          a.entity_ids.push_back(entity.as<const char*>());
        } else if (entity.is<JsonObject>()) {
          // Object entry: {"entity_id": "light.kitchen_main"}
          const char* entity_id = entity["entity_id"];
          if (entity_id) a.entity_ids.push_back(entity_id);
        }
      }

      this->discovered_areas_.push_back(a);
      ESP_LOGI(TAG, "  Found area: %s with %d entities", name, (int)a.entity_ids.size());
    }
  }
}

void HaAutoPanel::fetch_entities_() {
  if (this->http_request_ == nullptr) {
    ESP_LOGE(TAG, "http_request_ is null - cannot fetch entities");
    return;
  }

  ESP_LOGI(TAG, "Fetching entity states from HA...");

  std::string url = this->ha_api_url_ + "/api/states";

  std::vector<http_request::Header> headers = {
      {"Authorization", "Bearer " + this->ha_api_password_},
      {"Content-Type", "application/json"},
  };

  auto container = this->http_request_->get(url, headers);

  if (container == nullptr) {
    ESP_LOGE(TAG, "Failed to get states: container is null");
    return;
  }

  if (container->status_code == 0) {
    ESP_LOGE(TAG, "Failed to get states: connection failed");
    container->end();
    return;
  }

  if (container->status_code == 401) {
    ESP_LOGE(TAG, "HA API auth failed - check your token");
    container->end();
    this->set_panel_state_(PanelState::AUTH_FAILED);
    return;
  }

  if (container->status_code == 404) {
    ESP_LOGE(TAG, "HA API endpoint not found - check HA version");
    container->end();
    return;
  }

  if (container->status_code != 200) {
    ESP_LOGW(TAG, "Failed to get states: HTTP %d", container->status_code);
    container->end();
    return;
  }

  // Read response body - reserve to content_length to avoid reallocations
  size_t expected = container->content_length;
  std::string response;
  response.reserve(expected > 0 ? expected : 32768);
  uint8_t buf[512];
  uint32_t last_data_time = millis();
  const uint32_t timeout = 20000;
  int iter_count = 0;

  while (container->get_bytes_read() < container->content_length) {
    App.feed_wdt();
    yield();
    int read = container->read(buf, sizeof(buf));
    if (read > 0) {
      response.append(reinterpret_cast<char*>(buf), read);
      last_data_time = millis();
      iter_count = 0;
    } else if (read == http_request::HTTP_ERROR_CONNECTION_CLOSED) {
      break;
    } else {
      iter_count++;
      if (iter_count % 10 == 0) {
        App.feed_wdt();
      }
    }
    if (millis() - last_data_time > timeout) {
      ESP_LOGW(TAG, "Timeout reading states response");
      break;
    }
  }
  container->end();

  ESP_LOGI(TAG, "States response: %d bytes", (int)response.size());
  ESP_LOGD(TAG, "  Heap free=%u largest=%u, PSRAM free=%u largest=%u",
           (unsigned)heap_caps_get_free_size(MALLOC_CAP_8BIT),
           (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT),
           (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
           (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM));
  // Diagnostic: log first 200 chars to see actual format
  {
    std::string head = response.substr(0, std::min<size_t>(200, response.size()));
    ESP_LOGD(TAG, "  head: %s", head.c_str());
  }

  // ArduinoJson 7.x: JsonDocument with PSRAM allocator to keep DOM off
  // the small internal heap.
  PsramJsonDocument doc(&s_psram_allocator);

  DeserializationError parse_err = deserializeJson(doc, response);
  if (parse_err) {
    ESP_LOGW(TAG, "Failed to parse states JSON: %s", parse_err.c_str());
    return;
  }

  JsonArray arr = doc.as<JsonArray>();
  if (arr.isNull()) {
    JsonObject obj = doc.as<JsonObject>();
    if (!obj.isNull()) {
      ESP_LOGW(TAG, "States response is a JSON object, keys:");
      for (JsonPair kv : obj) {
        ESP_LOGW(TAG, "  key: %s", kv.key().c_str());
      }
    } else {
      ESP_LOGW(TAG, "States response is neither array nor object");
    }
    return;
  }

  // Build a lookup: entity_id -> area_id from discovered_areas_
  // Since we already have entity_ids per area, match against that
  std::map<std::string, std::string> entity_to_area;  // entity_id -> area_id
  for (const auto& area : this->discovered_areas_) {
    for (const auto& eid : area.entity_ids) {
      entity_to_area[eid] = area.area_id;
    }
  }

  for (JsonObject state : arr) {
    const char* entity_id = state["entity_id"];
    if (!entity_id) continue;

    // Check if this entity belongs to one of our areas
    auto it = entity_to_area.find(entity_id);
    if (it == entity_to_area.end()) continue;  // Not in any discovered area

    std::string area_id = it->second;

    // Check if excluded
    if (this->is_entity_excluded_(entity_id)) continue;

    // Extract domain from entity_id (e.g., "light.living_room" -> "light")
    std::string eid(entity_id);
    size_t dot = eid.find('.');
    if (dot == std::string::npos) continue;
    std::string domain = eid.substr(0, dot);

    // Check if domain is included
    if (!this->is_domain_included_(domain)) continue;

    Entity entity;
    entity.entity_id = entity_id;
    entity.domain = domain;
    entity.area_id = area_id;

    // Get attributes
    JsonObject attributes = state["attributes"];

    // Get friendly name from attributes, or derive from entity_id
    if (!attributes["friendly_name"].isNull()) {
      entity.name = std::string(attributes["friendly_name"].as<const char*>());
    } else {
      entity.name = eid.substr(dot + 1);
    }

    // Capture state string ("on" / "off" / etc.)
    const char* state_str_c = state["state"];
    if (state_str_c) {
      entity.state = state_str_c;
    }

    // Check if entity has brightness (lights with brightness support)
    if (domain == "light" && !attributes["brightness"].isNull()) {
      entity.has_brightness = true;
      entity.brightness = static_cast<uint8_t>(attributes["brightness"].as<int>());
    }

    // Hue group flag - skip these for room-level aggregations
    if (!attributes["is_hue_group"].isNull()) {
      entity.is_hue_group = attributes["is_hue_group"].as<bool>();
    }

    this->entities_by_area_[area_id].push_back(entity);
  }
  ESP_LOGI(TAG, "  Parsed %d entities into areas", (int)this->entities_by_area_.size());
}

void HaAutoPanel::start_discovery_() {
  ESP_LOGI(TAG, "=== Starting Dynamic Entity Discovery ===");

  // If we already transitioned to AUTH_FAILED, don't bother - the auth check
  // path already short-circuited.
  if (this->state_ == PanelState::AUTH_FAILED) {
    ESP_LOGW(TAG, "Skipping discovery: already in AUTH_FAILED state");
    return;
  }

  this->set_panel_state_(PanelState::CONNECTING);

  // Clear previous discovery data to avoid heap growth on retry
  this->discovered_areas_.clear();
  this->entities_by_area_.clear();
  this->room_cards_.clear();
  // Note: subscribed_entity_ids_ is intentionally preserved across re-discoveries
  // so we don't double-subscribe. New entities get added in subscribe_to_all_entities_.

  // Fetch real areas and entities from HA
  fetch_areas_();
  // After fetch_areas_(), we may have transitioned to AUTH_FAILED. Re-check.
  if (this->state_ == PanelState::AUTH_FAILED) {
    ESP_LOGW(TAG, "Aborting discovery: auth failed during area fetch");
    return;
  }
  fetch_entities_();
  if (this->state_ == PanelState::AUTH_FAILED) {
    ESP_LOGW(TAG, "Aborting discovery: auth failed during entity fetch");
    return;
  }

  if (discovered_areas_.empty()) {
    ESP_LOGW(TAG, "No areas discovered from HA - check API token and connectivity");
    // If we got a 200 but no areas, the user might not have any areas defined,
    // OR the device is not authorized to read them. Show NOT_AUTHORIZED so
    // the user can fix the toggle.
    this->set_panel_state_(PanelState::NOT_AUTHORIZED);
    return;
  }

  filter_and_build_room_cards_();

  // Create the UI
  create_ui_from_room_cards_();

  // Note: state subscription is now triggered from api.on_client_connected
  // in the YAML, not here. At WiFi-connect time the API isn't necessarily
  // up yet (encrypted auth takes a moment), so subscribing here would fail
  // silently with "HA not connected". The on_client_connected hook fires
  // after the API is ready and is the right place to subscribe.

  ESP_LOGI(TAG, "=== Discovery Complete ===");

  // Mark panel as READY - we have rooms to show.
  this->set_panel_state_(PanelState::READY);
}

void HaAutoPanel::dump_config() {
  ESP_LOGI(TAG, "Dynamic Entity Discovery Configuration:");
  ESP_LOGI(TAG, "  HA API URL: %s", this->ha_api_url_.c_str());
  ESP_LOGI(TAG, "  Include all areas: %s", this->include_all_ ? "YES" : "NO");
  ESP_LOGI(TAG, "  Excluded areas:");
  for (const auto& a : this->exclude_areas_) {
    ESP_LOGI(TAG, "    - %s", a.c_str());
  }
  ESP_LOGI(TAG, "  Entity domains:");
  for (const auto& d : this->entity_domains_) {
    ESP_LOGI(TAG, "    - %s", d.c_str());
  }
  ESP_LOGI(TAG, "  Layout: %d cards/row (%dx%d cards, screen %dx%d)",
           this->compute_cards_per_row_(),
           this->card_width_, this->card_width_,
           this->screen_width_, this->screen_height_);
  ESP_LOGI(TAG, "  Discovered areas: %d", (int)discovered_areas_.size());
}

bool HaAutoPanel::is_area_excluded_(const std::string& area_name) const {
  for (const auto& excluded : this->exclude_areas_) {
    if (area_name == excluded) return true;
  }
  if (!this->include_all_ && !this->include_areas_.empty()) {
    bool found = false;
    for (const auto& included : this->include_areas_) {
      if (area_name == included) { found = true; break; }
    }
    if (!found) return true;
  }
  return false;
}

bool HaAutoPanel::is_entity_excluded_(const std::string& entity_id) const {
  for (const auto& excluded : this->exclude_entities_) {
    if (entity_id == excluded) return true;
  }
  return false;
}

bool HaAutoPanel::is_domain_included_(const std::string& domain) const {
  for (const auto& d : this->entity_domains_) {
    if (domain == d) return true;
  }
  return false;
}

void HaAutoPanel::filter_and_build_room_cards_() {
  room_cards_.clear();

  for (size_t i = 0; i < discovered_areas_.size(); i++) {
    const auto& area = discovered_areas_[i];

    if (is_area_excluded_(area.name)) {
      ESP_LOGI(TAG, "Skipping excluded area: %s", area.name.c_str());
      continue;
    }

    RoomCard card;
    card.area = area;
    int cards_per_row = this->compute_cards_per_row_();
    card.grid_index = (int)room_cards_.size();
    card.x = get_card_x_(card.grid_index % cards_per_row);
    card.y = get_card_y_(card.grid_index / cards_per_row);
    card.color = get_room_color_(card.grid_index);

    auto it = entities_by_area_.find(area.area_id);
    if (it != entities_by_area_.end()) {
      for (const auto& entity : it->second) {
        if (is_entity_excluded_(entity.entity_id)) continue;
        if (!is_domain_included_(entity.domain)) continue;
        card.entities.push_back(entity);
      }
    }

    room_cards_.push_back(card);
    ESP_LOGI(TAG, "Room card: %s at (%d,%d) color=0x%06X, %d entities",
             area.name.c_str(), card.x, card.y,
             (unsigned int)card.color, (int)card.entities.size());
  }
}

void HaAutoPanel::create_ui_from_room_cards_() {
  ESP_LOGI(TAG, "Creating dynamic UI for %d room cards", (int)room_cards_.size());

  // Clean up any previous main container and its widget references.
  // On a re-discovery (e.g. after a config change) the old widgets would
  // otherwise persist under the new render.
  if (this->main_container_ != nullptr) {
    lv_obj_del(this->main_container_);
    this->main_container_ = nullptr;
  }
  this->room_arc_widgets_.clear();
  this->room_btn_widgets_.clear();
  this->room_label_btn_widgets_.clear();
  // The per-render registries (arc_records_, control_records_) are cleared
  // at the top of each create_*_ call that appends to them.

  // Get LVGL screen
  lv_obj_t* screen = lv_scr_act();
  if (screen == nullptr) {
    ESP_LOGW(TAG, "No active LVGL screen yet, UI creation deferred");
    return;
  }

  ESP_LOGI(TAG, "Got LVGL screen, creating widgets...");

  // Set screen background to dark to hide boot artifacts
  lv_obj_set_style_bg_color(screen, lv_color_hex(0x111827), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, LV_PART_MAIN);

  // Create a scrollable container for the room cards. Width is the screen
  // width; height grows to fit all cards (so they can be scrolled
  // vertically when they overflow the screen).
  this->main_container_ = lv_obj_create(screen);
  lv_obj_set_pos(this->main_container_, 0, 0);
  lv_obj_set_size(this->main_container_, this->screen_width_, this->screen_height_);
  lv_obj_set_style_bg_color(this->main_container_, lv_color_hex(0x111827), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(this->main_container_, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_scrollbar_mode(this->main_container_, LV_SCROLLBAR_MODE_OFF);  // Clean, no scrollbar
  lv_obj_set_style_pad_all(this->main_container_, 0, 0);  // No padding
  lv_obj_set_style_border_width(this->main_container_, 0, 0);  // No border
  lv_obj_set_style_border_color(this->main_container_, lv_color_hex(0x111827), 0);  // Hide border
  // No scroll-snap. With snap enabled, scrolling snaps each row to the top
  // of the screen, but the user can still scroll past the last card to a
  // blank area. We size the container tightly so the last card is always
  // at the bottom of the scrollable area.

  for (const auto& room : room_cards_) {
    create_room_card_(this->main_container_, room);
  }

  // Size the main container to fit all rendered cards, with no extra
  // blank space. Cards are square (width == height == card_width_), so
  // the height per row is just card_width_. If the content fits within
  // screen_height_ we use screen_height_ so the page fills the screen;
  // otherwise the container grows and LVGL scrolls.
  int cards_per_row = this->compute_cards_per_row_();
  int num_rows = (int)((room_cards_.size() + cards_per_row - 1) / cards_per_row);
  if (num_rows < 1) num_rows = 1;
  // Size the page to exactly the bottom of the last card. Previously this
  // added a full card_gap_ + 20px padding, which left dead scrollable space
  // below the last row (especially noticeable when the page is taller than
  // the screen, since the user could scroll past the cards into emptiness).
  int actual_bottom = this->start_y_ + (num_rows - 1) * (this->card_width_ + this->card_gap_) + this->card_width_;
  // But still expand to fill the screen if content is shorter, so the dark
  // background of main_container_ covers the full display (otherwise the
  // bottom of the screen would show the default screen background).
  int total_height = actual_bottom > this->screen_height_ ? actual_bottom : this->screen_height_;
  lv_obj_set_style_height(this->main_container_, total_height, LV_PART_MAIN);

  ESP_LOGI(TAG, "UI creation complete: %d cards in %d rows, page height=%d (screen=%d, last card bottom=%d)",
           (int) room_cards_.size(), num_rows, total_height, this->screen_height_, actual_bottom);
}

void HaAutoPanel::create_room_card_(void* parent, const RoomCard& room) {
  // NOTE: do NOT clear the per-render registries here. create_room_card_() is
  // called in a loop for every room, and clearing the registry at the top of
  // each iteration would clobber indices from previous cards - all room
  // widgets would end up with user_data=0, pointing at the last room's
  // record. Instead, the room-level arc and button use heap-allocated
  // RoomControlData (freed in LV_EVENT_DELETE) to carry a stable area_id.

  lv_obj_t* card = lv_obj_create((lv_obj_t*) parent);
  lv_obj_set_pos(card, room.x, room.y);
  lv_obj_set_size(card, this->card_width_, this->card_width_);  // square
  lv_obj_set_style_bg_color(card, lv_color_hex(0x1a1a2e), 0);
  lv_obj_set_style_radius(card, 12, 0);
  lv_obj_set_style_border_width(card, 0, 0);
  lv_obj_set_scrollbar_mode(card, LV_SCROLLBAR_MODE_OFF);
  lv_obj_remove_flag(card, LV_OBJ_FLAG_SCROLLABLE);
  // Belt and suspenders: disable every scroll-related flag so the card
  // can never intercept a touch as a scroll gesture. Without these, a
  // diagonal drag that started on the arc could bubble up and be
  // interpreted as a scroll on the card itself, fighting the arc drag.
  lv_obj_remove_flag(card, LV_OBJ_FLAG_SCROLL_CHAIN_HOR);
  lv_obj_remove_flag(card, LV_OBJ_FLAG_SCROLL_CHAIN_VER);
  lv_obj_remove_flag(card, LV_OBJ_FLAG_SCROLL_ELASTIC);
  lv_obj_remove_flag(card, LV_OBJ_FLAG_SCROLL_MOMENTUM);
  lv_obj_remove_flag(card, LV_OBJ_FLAG_SCROLL_WITH_ARROW);
  lv_obj_remove_flag(card, LV_OBJ_FLAG_SEND_DRAW_TASK_EVENTS);  // Skip draw notifications

  // Compute current room state for initial visual
  bool is_on = is_room_any_light_on_(room.area.area_id);
  uint8_t initial_pct = is_on
      ? std::max<uint8_t>(compute_room_brightness_pct_(room.area.area_id), 10)
      : 0;

  // Arc for brightness - CREATE FIRST so it's below label button in z-order.
  // The arc is square and scaled to fit the card (card_width - 20).
  int arc_size = this->arc_size_();
  lv_obj_t* arc = lv_arc_create(card);
  // Hide during construction so the first paint cycle doesn't show the
  // arc with its default (white) color before our styles take effect.
  // We unhide it once all the colors/widths are set below.
  lv_obj_add_flag(arc, LV_OBJ_FLAG_HIDDEN);
  lv_obj_set_size(arc, arc_size, arc_size);  // square
  lv_obj_align(arc, LV_ALIGN_CENTER, 0, -10);  // shift up a bit to make room for button
  lv_arc_set_min_value(arc, 0);
  lv_arc_set_max_value(arc, 100);
  lv_arc_set_value(arc, initial_pct);  // From computed state, not hardcoded 50
  lv_arc_set_bg_start_angle(arc, 135);
  lv_arc_set_bg_end_angle(arc, 405);  // 135 + 270
  // Arc width scales with the card size (about 8% of card_width) so the
  // arc looks proportional on both small and large cards.
  int arc_width = std::max(8, this->card_width_ / 12);
  lv_obj_set_style_arc_color(arc, lv_color_hex(0x404040), LV_PART_MAIN);
  lv_obj_set_style_arc_width(arc, arc_width, LV_PART_MAIN);
  lv_obj_set_style_arc_color(arc, lv_color_hex(room.color), LV_PART_INDICATOR);
  lv_obj_set_style_arc_width(arc, arc_width, LV_PART_INDICATOR);
  // Force a layout pass so styles take effect before the arc is shown
  lv_obj_update_layout(arc);
  // Now safe to make visible - styles are applied
  lv_obj_remove_flag(arc, LV_OBJ_FLAG_HIDDEN);

  // Heap-allocated control data - the user_data is a raw pointer that stays
  // valid for the lifetime of the widget. The struct owns the area_id string.
  RoomControlData* arc_data = new RoomControlData{room.area.area_id, nullptr};
  lv_obj_set_user_data(arc, arc_data);

  // Stash widget pointer for live state updates
  this->room_arc_widgets_[room.area.area_id] = arc;

  // Use LV_EVENT_RELEASED (not VALUE_CHANGED) so we get ONE service call
  // per drag gesture, not one per finger-move tick. This matches the old
  // YAML's "mode: restart" script behavior.
  lv_obj_add_event_cb(arc, [](lv_event_t* event) {
    lv_obj_t* arc = (lv_obj_t*)lv_event_get_target(event);
    RoomControlData* data = (RoomControlData*)lv_obj_get_user_data(arc);
    if (s_instance == nullptr || data == nullptr) return;
    int value = lv_arc_get_value(arc);
    if (value <= 0) {
      s_instance->call_ha_service_("light.turn_off", "area_id", data->area_id, -1);
      // Optimistic: mark lights off so the button color flips immediately
      for (auto& e : s_instance->entities_by_area_[data->area_id]) {
        if (e.domain == "light" && !e.is_hue_group) {
          e.state = "off";
        }
      }
      s_instance->update_room_card_visual_state_for_area_(data->area_id);
    } else {
      s_instance->last_brightness_pct_[data->area_id] = (uint8_t) value;
      s_instance->call_ha_service_("light.turn_on", "area_id", data->area_id, value);
      // Optimistic: mark lights on at the dragged brightness
      for (auto& e : s_instance->entities_by_area_[data->area_id]) {
        if (e.domain == "light" && !e.is_hue_group) {
          e.state = "on";
          e.brightness = static_cast<uint8_t>((value * 255) / 100);
        }
      }
      s_instance->update_room_card_visual_state_for_area_(data->area_id);
    }
  }, LV_EVENT_RELEASED, nullptr);

  // Free the heap struct when the arc is destroyed (e.g. on re-discovery)
  lv_obj_add_event_cb(arc, [](lv_event_t* event) {
    lv_obj_t* target = (lv_obj_t*)lv_event_get_target(event);
    RoomControlData* data = (RoomControlData*)lv_obj_get_user_data(target);
    delete data;
  }, LV_EVENT_DELETE, nullptr);

  // ON/OFF button - small, just wide enough to fit the "ON/OFF" label
  // (~110px at the default font). Not tied to card_width.
  lv_obj_t* btn = lv_obj_create(card);
  int btn_w = 110;
  int btn_h = std::max(28, this->card_width_ / 8);
  lv_obj_set_size(btn, btn_w, btn_h);
  lv_obj_align(btn, LV_ALIGN_BOTTOM_MID, 0, -8);
  // Disable all internal scrolling on the button. By default lv_obj_create()
  // makes a scrollable object, which means dragging on the button can
  // scroll its contents (the label) around. We want the button to be a
  // fixed touch target.
  lv_obj_remove_flag(btn, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_remove_flag(btn, LV_OBJ_FLAG_SCROLL_CHAIN_HOR);
  lv_obj_remove_flag(btn, LV_OBJ_FLAG_SCROLL_CHAIN_VER);
  lv_obj_remove_flag(btn, LV_OBJ_FLAG_SCROLL_ELASTIC);
  lv_obj_remove_flag(btn, LV_OBJ_FLAG_SCROLL_MOMENTUM);
  lv_obj_remove_flag(btn, LV_OBJ_FLAG_SCROLL_WITH_ARROW);
  lv_obj_set_scrollbar_mode(btn, LV_SCROLLBAR_MODE_OFF);
  lv_obj_set_style_radius(btn, 6, 0);
  lv_obj_set_style_border_width(btn, 2, 0);
  lv_obj_set_style_border_color(btn, lv_color_hex(room.color), 0);
  // Set initial color based on current state (matches the old YAML's update_*_btn script)
  lv_obj_set_style_bg_color(btn, is_on ? lv_color_hex(room.color) : lv_color_hex(0x222222), 0);

  lv_obj_t* btn_label = lv_label_create(btn);
  lv_label_set_text(btn_label, "ON/OFF");
  lv_obj_set_style_text_color(btn_label, is_on ? lv_color_hex(0x222222) : lv_color_hex(room.color), 0);
  lv_obj_center(btn_label);
  lv_obj_set_scrollbar_mode(btn, LV_SCROLLBAR_MODE_OFF);

  // Heap-allocated control data - stable pointer, owned by us, freed in DELETE
  RoomControlData* btn_data = new RoomControlData{room.area.area_id, nullptr};
  lv_obj_set_user_data(btn, btn_data);

  // Stash for live state updates
  this->room_btn_widgets_[room.area.area_id] = std::make_pair(btn, btn_label);

  lv_obj_add_event_cb(btn, [](lv_event_t* event) {
    lv_obj_t* btn = (lv_obj_t*)lv_event_get_target(event);
    RoomControlData* data = (RoomControlData*)lv_obj_get_user_data(btn);
    if (s_instance == nullptr || data == nullptr) return;
    bool is_on = s_instance->is_room_any_light_on_(data->area_id);
    if (is_on) {
      s_instance->call_ha_service_("light.turn_off", "area_id", data->area_id, -1);
      // Optimistic UI: flip immediately. Will be re-synced when HA pushes
      // the state change back via the subscription.
      for (auto& e : s_instance->entities_by_area_[data->area_id]) {
        if (e.domain == "light" && !e.is_hue_group) {
          e.state = "off";
        }
      }
      s_instance->update_room_card_visual_state_for_area_(data->area_id);
    } else {
      // Bounce-back: use last non-zero brightness for this area, otherwise
      // the configured default_on_pct.
      uint8_t pct = (uint8_t) s_instance->default_on_pct_;
      auto it = s_instance->last_brightness_pct_.find(data->area_id);
      if (it != s_instance->last_brightness_pct_.end() && it->second > 0) {
        pct = it->second;
      }
      s_instance->call_ha_service_("light.turn_on", "area_id", data->area_id, pct);
      // Optimistic UI: mark lights on at the bounced-back brightness so the
      // visual update sees the new state. The subscription will refine the
      // brightness to whatever HA actually reports.
      for (auto& e : s_instance->entities_by_area_[data->area_id]) {
        if (e.domain == "light" && !e.is_hue_group) {
          e.state = "on";
          e.brightness = static_cast<uint8_t>((pct * 255) / 100);
        }
      }
      s_instance->update_room_card_visual_state_for_area_(data->area_id);
    }
  }, LV_EVENT_CLICKED, nullptr);

  // Free the heap struct when the button is destroyed (e.g. on re-discovery)
  lv_obj_add_event_cb(btn, [](lv_event_t* event) {
    lv_obj_t* target = (lv_obj_t*)lv_event_get_target(event);
    RoomControlData* data = (RoomControlData*)lv_obj_get_user_data(target);
    delete data;
  }, LV_EVENT_DELETE, nullptr);

  // Room name label - clickable area using transparent button
  // CREATE LAST so it's on TOP in z-order and receives touches
  lv_obj_t* label_btn = lv_obj_create(card);
  lv_obj_remove_style_all(label_btn);  // Start with clean slate to avoid default button styles
  // Match the arc's position and width so the label is centered relative
  // to the arc, not to the card. Pass the arc as the align base so the
  // label center lands on the arc center.
  lv_obj_set_size(label_btn, arc_size, 32);
  lv_obj_align_to(label_btn, arc, LV_ALIGN_CENTER, 0, 0);
  lv_obj_set_style_bg_opa(label_btn, LV_OPA_TRANSP, 0);  // Invisible background
  lv_obj_set_style_border_width(label_btn, 0, 0);  // No border
  lv_obj_set_style_radius(label_btn, 6, 0);
  lv_obj_set_style_pad_all(label_btn, 5, 0);  // Ensure touch padding
  // Disable scrolling on the label_btn (it's a touch target, not a list).
  lv_obj_remove_flag(label_btn, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_remove_flag(label_btn, LV_OBJ_FLAG_SCROLL_CHAIN_HOR);
  lv_obj_remove_flag(label_btn, LV_OBJ_FLAG_SCROLL_CHAIN_VER);
  lv_obj_remove_flag(label_btn, LV_OBJ_FLAG_SCROLL_ELASTIC);
  lv_obj_remove_flag(label_btn, LV_OBJ_FLAG_SCROLL_MOMENTUM);
  lv_obj_remove_flag(label_btn, LV_OBJ_FLAG_SCROLL_WITH_ARROW);
  lv_obj_set_scrollbar_mode(label_btn, LV_SCROLLBAR_MODE_OFF);
  lv_obj_add_flag(label_btn, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_set_user_data(label_btn, (void*)(intptr_t)room.grid_index);
  lv_obj_add_event_cb(label_btn, [](lv_event_t* event) {
    int room_index = (int)(intptr_t)lv_obj_get_user_data((lv_obj_t*)lv_event_get_target(event));
    s_instance->show_entity_detail_(room_index);
  }, LV_EVENT_CLICKED, nullptr);

  lv_obj_t* label = lv_label_create(label_btn);
  lv_label_set_text(label, room.area.name.c_str());
  lv_obj_set_style_text_color(label, lv_color_hex(0xFFFFFF), 0);
  lv_obj_center(label);

  lv_obj_move_foreground(label_btn);

  ESP_LOGI(TAG, "  Created room card: %s", room.area.name.c_str());
}

void HaAutoPanel::show_entity_detail_(int room_index) {
  ESP_LOGI(TAG, "Showing entity detail for room index %d", room_index);

  if (room_index < 0 || room_index >= (int)room_cards_.size()) {
    ESP_LOGW(TAG, "Invalid room index %d", room_index);
    return;
  }

  const RoomCard& room = room_cards_[room_index];
  this->current_room_index_ = room_index;

  // Hide main container
  if (this->main_container_) {
    lv_obj_scroll_to_y(this->main_container_, 0, LV_ANIM_OFF);  // Reset scroll to top
    lv_obj_add_flag(this->main_container_, LV_OBJ_FLAG_HIDDEN);
  }

  // Create detail container if not exists
  if (this->detail_container_) {
    lv_obj_del(this->detail_container_);
    // Per-render registries for the entity detail view start fresh.
    // The room-level registries were cleared at the top of create_room_card_()
    // and are not used in the detail view.
    this->arc_records_.clear();
    this->control_records_.clear();
  }

  lv_obj_t* screen = lv_scr_act();
  this->detail_container_ = lv_obj_create(screen);
  lv_obj_set_pos(this->detail_container_, 0, 0);
  lv_obj_set_size(this->detail_container_, 1024, 600);
  lv_obj_set_style_bg_color(this->detail_container_, lv_color_hex(0x111827), 0);
  lv_obj_set_scrollbar_mode(this->detail_container_, LV_SCROLLBAR_MODE_OFF);  // Clean, no scrollbar
  lv_obj_set_style_pad_all(this->detail_container_, 0, 0);  // No padding
  lv_obj_set_style_border_width(this->detail_container_, 0, 0);  // No border

  // Back button
  lv_obj_t* back_btn = lv_obj_create(this->detail_container_);
  lv_obj_set_pos(back_btn, 10, 10);
  lv_obj_set_size(back_btn, 100, 40);
  lv_obj_set_style_radius(back_btn, 6, 0);
  lv_obj_set_style_bg_color(back_btn, lv_color_hex(0x333333), 0);

  lv_obj_t* back_label = lv_label_create(back_btn);
  lv_label_set_text(back_label, "< Back");
  lv_obj_set_style_text_color(back_label, lv_color_hex(0xFFFFFF), 0);
  lv_obj_center(back_label);
  lv_obj_add_event_cb(back_btn, [](lv_event_t* event) {
    s_instance->show_room_grid_();
  }, LV_EVENT_CLICKED, nullptr);

  // Room title - to the right of back button
  lv_obj_t* title = lv_label_create(this->detail_container_);
  lv_label_set_text(title, room.area.name.c_str());
  lv_obj_set_style_text_color(title, lv_color_hex(room.color), 0);
  lv_obj_set_pos(title, 130, 15);

  // Entity list - start below back button and title
  int y_offset = 65;
  for (size_t i = 0; i < room.entities.size(); i++) {
    create_entity_control_(this->detail_container_, room.entities[i], (int)i, y_offset, room.color);
    y_offset += 80;
  }

  // Expand container height to fit all entities for scrolling
  int total_content_height = y_offset + 20;  // Add some padding at bottom
  if (total_content_height > 600) {
    lv_obj_set_style_height(this->detail_container_, total_content_height, LV_PART_MAIN);
    ESP_LOGI(TAG, "Expanded detail container to %d px for %d entities", total_content_height, (int)room.entities.size());
  }

  if (room.entities.empty()) {
    lv_obj_t* no_entities = lv_label_create(this->detail_container_);
    lv_label_set_text(no_entities, "No entities in this room");
    lv_obj_set_style_text_color(no_entities, lv_color_hex(0x888888), 0);
    lv_obj_set_pos(no_entities, 50, 100);
  }

  ESP_LOGI(TAG, "Entity detail view created with %d entities", (int)room.entities.size());
}

void HaAutoPanel::show_room_grid_() {
  ESP_LOGI(TAG, "Showing room grid");

  // Hide detail container
  if (this->detail_container_) {
    lv_obj_add_flag(this->detail_container_, LV_OBJ_FLAG_HIDDEN);
  }

  // Show main container
  if (this->main_container_) {
    lv_obj_scroll_to_y(this->main_container_, 0, LV_ANIM_OFF);  // Reset scroll to top
    lv_obj_remove_flag(this->main_container_, LV_OBJ_FLAG_HIDDEN);
  }

  this->current_room_index_ = -1;
}

void HaAutoPanel::create_entity_control_(void* parent, const Entity& entity, int entity_index, int y_pos, uint32_t color) {
  lv_obj_t* control = lv_obj_create((lv_obj_t*) parent);
  lv_obj_set_pos(control, 32, y_pos);  // Centered: (1024-960)/2 = 32
  lv_obj_set_size(control, 960, 70);
  lv_obj_set_style_bg_color(control, lv_color_hex(0x1a1a2e), 0);
  lv_obj_set_style_radius(control, 8, 0);
  lv_obj_set_style_border_width(control, 1, 0);
  lv_obj_set_style_border_color(control, lv_color_hex(color), 0);
  lv_obj_set_scrollbar_mode(control, LV_SCROLLBAR_MODE_OFF);
  lv_obj_remove_flag(control, LV_OBJ_FLAG_SCROLLABLE);

  // Entity name - center in control
  lv_obj_t* name_label = lv_label_create(control);
  lv_label_set_text(name_label, entity.name.c_str());
  lv_obj_set_style_text_color(name_label, lv_color_hex(0xFFFFFF), 0);
  lv_obj_align(name_label, LV_ALIGN_LEFT_MID, 15, 0);

  // Domain label - right side, before arc
  lv_obj_t* domain_label = lv_label_create(control);
  // Use shorter display names for long domain names
  if (entity.domain == "binary_sensor") {
    lv_label_set_text(domain_label, "contact");
  } else if (entity.domain == "light") {
    lv_label_set_text(domain_label, "light");
  } else {
    lv_label_set_text(domain_label, entity.domain.c_str());
  }
  lv_obj_set_style_text_color(domain_label, lv_color_hex(color), 0);
  lv_obj_align(domain_label, LV_ALIGN_RIGHT_MID, -170, 0);

  if (entity.domain == "light" && entity.has_brightness) {
    // Brightness arc for lights - initial value reflects current HA state
    uint8_t initial_pct = (entity.state == "on" && entity.brightness > 0)
        ? static_cast<uint8_t>((entity.brightness * 100) / 255)
        : 0;
    lv_obj_t* arc = lv_arc_create(control);
    lv_obj_set_size(arc, 50, 50);
    lv_obj_align(arc, LV_ALIGN_RIGHT_MID, -100, 0);
    lv_arc_set_min_value(arc, 0);
    lv_arc_set_max_value(arc, 100);
    lv_arc_set_value(arc, initial_pct);
    lv_arc_set_bg_start_angle(arc, 135);
    lv_arc_set_bg_end_angle(arc, 405);
    lv_obj_set_style_arc_color(arc, lv_color_hex(0x404040), LV_PART_MAIN);
    lv_obj_set_style_arc_width(arc, 6, LV_PART_MAIN);
    lv_obj_set_style_arc_color(arc, lv_color_hex(color), LV_PART_INDICATOR);
    lv_obj_set_style_arc_width(arc, 6, LV_PART_INDICATOR);

    // Brightness percentage label
    lv_obj_t* pct_label = lv_label_create(control);
    lv_label_set_text_fmt(pct_label, "%d%%", initial_pct);
    lv_obj_set_style_text_color(pct_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(pct_label, LV_ALIGN_RIGHT_MID, -30, 0);

    // Register in component-owned registry (no heap, no DELETE handler)
    ArcRecord arc_rec;
    arc_rec.entity_id = entity.entity_id;
    arc_rec.area_id = entity.area_id;
    arc_rec.pct_label = pct_label;
    arc_rec.is_room_arc = false;
    this->arc_records_.push_back(arc_rec);
    size_t arc_idx = this->arc_records_.size() - 1;
    lv_obj_set_user_data(arc, (void*)(intptr_t)arc_idx);

    // Two callbacks:
    //  - VALUE_CHANGED: update the % label live as the user drags
    //  - RELEASED: send ONE service call per drag gesture
    lv_obj_add_event_cb(arc, [](lv_event_t* event) {
      lv_obj_t* arc = (lv_obj_t*)lv_event_get_target(event);
      int value = lv_arc_get_value(arc);
      size_t idx = (size_t)(intptr_t)lv_obj_get_user_data(arc);
      if (s_instance != nullptr && idx < s_instance->arc_records_.size()) {
        lv_obj_t* label = s_instance->arc_records_[idx].pct_label;
        if (label != nullptr) {
          lv_label_set_text_fmt(label, "%d%%", value);
        }
      }
    }, LV_EVENT_VALUE_CHANGED, nullptr);

    lv_obj_add_event_cb(arc, [](lv_event_t* event) {
      lv_obj_t* arc = (lv_obj_t*)lv_event_get_target(event);
      size_t idx = (size_t)(intptr_t)lv_obj_get_user_data(arc);
      if (s_instance == nullptr || idx >= s_instance->arc_records_.size()) return;
      const auto& rec = s_instance->arc_records_[idx];
      int value = lv_arc_get_value(arc);
      if (value <= 0) {
        s_instance->call_ha_service_("light.turn_off", "entity_id", rec.entity_id, -1);
      } else {
        s_instance->call_ha_service_("light.turn_on", "entity_id", rec.entity_id, value);
      }
    }, LV_EVENT_RELEASED, nullptr);
  } else if (entity.domain == "switch") {
    // Switch has toggle button
    lv_obj_t* toggle_btn = lv_obj_create(control);
    lv_obj_set_size(toggle_btn, 100, 40);
    lv_obj_align(toggle_btn, LV_ALIGN_RIGHT_MID, -15, 0);
    lv_obj_set_style_radius(toggle_btn, 6, 0);
    lv_obj_set_style_bg_color(toggle_btn, lv_color_hex(0x333333), 0);

    lv_obj_t* toggle_label = lv_label_create(toggle_btn);
    lv_label_set_text(toggle_label, "Toggle");
    lv_obj_set_style_text_color(toggle_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_center(toggle_label);

    // Register in control registry
    ControlRecord rec;
    rec.entity_id = entity.entity_id;
    rec.area_id = entity.area_id;
    rec.domain = "switch";
    rec.btn = toggle_btn;
    rec.state_label = toggle_label;
    this->control_records_.push_back(rec);
    size_t btn_idx = this->control_records_.size() - 1;
    lv_obj_set_user_data(toggle_btn, (void*)(intptr_t)btn_idx);

    lv_obj_add_event_cb(toggle_btn, [](lv_event_t* event) {
      lv_obj_t* btn = (lv_obj_t*)lv_event_get_target(event);
      size_t idx = (size_t)(intptr_t)lv_obj_get_user_data(btn);
      if (s_instance == nullptr || idx >= s_instance->control_records_.size()) return;
      const auto& rec = s_instance->control_records_[idx];
      s_instance->call_ha_service_("switch.toggle", "entity_id", rec.entity_id, -1);
    }, LV_EVENT_CLICKED, nullptr);
  } else {
    // Read-only entities (sensor, binary_sensor, climate, cover, etc.) - just show domain text aligned right
    // Nothing to do - domain label already shows the type
  }

  ESP_LOGI(TAG, "  Created control for entity: %s (%s)", entity.name.c_str(), entity.domain.c_str());
}

int HaAutoPanel::compute_cards_per_row_() const {
  // How many square cards of `card_width_` fit horizontally in `screen_width_`,
  // leaving start_x_ padding on each side and card_gap_ between cards.
  int usable = this->screen_width_ - 2 * this->start_x_;
  if (usable < (int) this->card_width_) return 1;  // smallest screen, 1 col
  // (n cards + (n-1) gaps + 2*start_x) <= screen_width
  // n <= (usable + gap) / (card_width + gap)
  return (usable + this->card_gap_) / (this->card_width_ + this->card_gap_);
}

int HaAutoPanel::get_card_x_(int col) const {
  return this->start_x_ + col * (this->card_width_ + this->card_gap_);
}

int HaAutoPanel::get_card_y_(int row) const {
  return this->start_y_ + row * (this->card_width_ + this->card_gap_);
}

uint32_t HaAutoPanel::get_room_color_(int index) const {
  return ROOM_COLORS_[index % MAX_ROOM_COLORS_];
}

// --- HA service call (native API) ---

void HaAutoPanel::call_ha_service_(const std::string& service,
                                              const std::string& target_type,
                                              const std::string& target_id,
                                              int brightness_pct) {
#ifdef USE_API_HOMEASSISTANT_SERVICES
  if (api::global_api_server == nullptr) {
    ESP_LOGW(TAG, "api::global_api_server is null; cannot call %s", service.c_str());
    return;
  }
  if (!api::global_api_server->is_connected()) {
    ESP_LOGW(TAG, "HA not connected; skipping %s %s=%s", service.c_str(), target_type.c_str(), target_id.c_str());
    return;
  }

  api::HomeassistantActionRequest req;
  req.service = StringRef(service);

  // StringRef is non-owning - copy to stack-local buffers that outlive the
  // call (the protobuf serializer runs synchronously inside send_homeassistant_action).
  char target_buf[128];
  strncpy(target_buf, target_id.c_str(), sizeof(target_buf) - 1);
  target_buf[sizeof(target_buf) - 1] = '\0';

  char bright_buf[16] = {0};
  int num_keys = 1;  // always include the target
  if (brightness_pct >= 0 && brightness_pct <= 100) {
    snprintf(bright_buf, sizeof(bright_buf), "%d", brightness_pct);
    num_keys = 2;
  }

  req.data.init(num_keys);
  {
    auto &kv = req.data.emplace_back();
    kv.key   = StringRef(target_type);
    kv.value = StringRef(target_buf);
  }
  if (num_keys == 2) {
    auto &kv = req.data.emplace_back();
    kv.key   = StringRef::from_lit("brightness_pct");
    kv.value = StringRef(bright_buf);
  }

  api::global_api_server->send_homeassistant_action(req);

  if (brightness_pct >= 0) {
    ESP_LOGI(TAG, "Sent %s %s=%s brightness_pct=%d", service.c_str(), target_type.c_str(), target_id.c_str(), brightness_pct);
  } else {
    ESP_LOGI(TAG, "Sent %s %s=%s", service.c_str(), target_type.c_str(), target_id.c_str());
  }
#else
  ESP_LOGW(TAG, "API homeassistant_services disabled in YAML; cannot call %s", service.c_str());
#endif
}

// --- Local room-state computation (replaces per-room HA templates) ---

bool HaAutoPanel::is_room_any_light_on_(const std::string& area_id,
                                                   const std::string& exclude_entity_id) const {
  auto it = entities_by_area_.find(area_id);
  if (it == entities_by_area_.end()) return false;
  for (const auto& e : it->second) {
    if (e.domain != "light") continue;
    if (!exclude_entity_id.empty() && e.entity_id == exclude_entity_id) continue;
    if (e.is_hue_group) continue;
    if (e.state == "on") return true;
  }
  return false;
}

uint8_t HaAutoPanel::compute_room_brightness_pct_(const std::string& area_id) const {
  auto it = entities_by_area_.find(area_id);
  if (it == entities_by_area_.end()) return 0;
  uint8_t max_b = 0;
  for (const auto& e : it->second) {
    if (e.domain != "light" || e.state != "on") continue;
    if (e.is_hue_group) continue;
    if (e.brightness > max_b) max_b = e.brightness;
  }
  return static_cast<uint8_t>((max_b * 100) / 255);
}

// --- Native API state subscription ---

void HaAutoPanel::subscribe_to_all_entities_() {
#ifdef USE_API_HOMEASSISTANT_STATES
  if (api::global_api_server == nullptr) {
    ESP_LOGW(TAG, "api::global_api_server is null; skipping state subscriptions");
    return;
  }
  if (!api::global_api_server->is_connected()) {
    ESP_LOGW(TAG, "HA not connected; skipping state subscriptions");
    return;
  }

  int subscribed_this_run = 0;
  for (auto& kv : this->entities_by_area_) {
    for (const auto& e : kv.second) {
      if (this->subscribed_entity_ids_.count(e.entity_id)) continue;
      // Capture entity_id by value (small string) so the lambda outlives the loop.
      const std::string entity_id = e.entity_id;

      // Subscribe to entity state (new callback signature: void(StringRef))
      api::global_api_server->subscribe_home_assistant_state(
          entity_id,
          optional<std::string>(),  // empty = subscribe to state itself, not an attribute
          [this, entity_id](StringRef state) {
            this->on_entity_state_changed_(entity_id, state.c_str());
          });
      this->subscribed_entity_ids_.insert(entity_id);
      subscribed_this_run++;

      // Also subscribe to the "brightness" attribute (only meaningful for dimmable lights;
      // HA will still send updates if the attribute is missing, but it just becomes a no-op).
      api::global_api_server->subscribe_home_assistant_state(
          entity_id,
          optional<std::string>("brightness"),
          [this, entity_id](StringRef value) {
            this->on_entity_attribute_changed_(entity_id, value.c_str());
          });
    }
  }
  ESP_LOGI(TAG, "Subscribed to %d entity state streams (%d new this run)",
           (int)this->subscribed_entity_ids_.size(), subscribed_this_run);
#else
  ESP_LOGW(TAG, "API homeassistant_states disabled in YAML; live updates disabled");
#endif
}

void HaAutoPanel::on_entity_state_changed_(const std::string& entity_id, const char* state) {
  std::string new_state = state ? state : "";
  ESP_LOGD(TAG, "state_changed: %s -> %s", entity_id.c_str(), new_state.c_str());

  for (auto& kv : this->entities_by_area_) {
    for (auto& e : kv.second) {
      if (e.entity_id == entity_id) {
        e.state = new_state;
        break;
      }
    }
  }
  this->update_room_card_visual_state_for_entity_(entity_id);
}

void HaAutoPanel::on_entity_attribute_changed_(const std::string& entity_id, const char* value) {
  if (value == nullptr) return;
  int b = atoi(value);
  if (b < 0) b = 0;
  if (b > 255) b = 255;
  uint8_t new_brightness = static_cast<uint8_t>(b);

  for (auto& kv : this->entities_by_area_) {
    for (auto& e : kv.second) {
      if (e.entity_id == entity_id) {
        e.brightness = new_brightness;
        if (new_brightness > 0) e.has_brightness = true;
        break;
      }
    }
  }
  this->update_room_card_visual_state_for_entity_(entity_id);
}

std::string HaAutoPanel::find_area_id_for_entity_(const std::string& entity_id) const {
  for (const auto& kv : this->entities_by_area_) {
    for (const auto& e : kv.second) {
      if (e.entity_id == entity_id) return kv.first;
    }
  }
  return std::string();
}

void HaAutoPanel::update_room_card_visual_state_for_entity_(const std::string& entity_id) {
  std::string area_id = this->find_area_id_for_entity_(entity_id);
  if (area_id.empty()) return;
  this->update_room_card_visual_state_for_area_(area_id);
}

void HaAutoPanel::update_room_card_visual_state_for_area_(const std::string& area_id) {
  if (area_id.empty()) return;

  // Look up the room color from room_cards_
  uint32_t color = 0xFFFF00;
  for (const auto& card : this->room_cards_) {
    if (card.area.area_id == area_id) {
      color = card.color;
      break;
    }
  }

  bool is_on = this->is_room_any_light_on_(area_id);
  uint8_t pct = is_on ? this->compute_room_brightness_pct_(area_id) : 0;

  // Update the room card's big arc + button to match the current state.
  // Mirrors the old YAML's update_*_btn scripts: ON -> bg=room_color, text=dark;
  // OFF -> bg=dark, text=room_color.
  auto arc_it = this->room_arc_widgets_.find(area_id);
  if (arc_it != this->room_arc_widgets_.end() && arc_it->second != nullptr) {
    // lv_arc_set_value() does NOT fire LV_EVENT_VALUE_CHANGED (verified),
    // so this won't trigger a feedback loop.
    lv_arc_set_value(arc_it->second, pct);
    // Force a repaint of the arc. Style + value changes don't always
    // invalidate the widget in our LVGL setup; without this, the change
    // isn't visible until the parent is hidden and re-shown (e.g. by
    // navigating to a detail view and back).
    lv_obj_invalidate(arc_it->second);
  }

  auto btn_it = this->room_btn_widgets_.find(area_id);
  if (btn_it != this->room_btn_widgets_.end()) {
    lv_obj_set_style_bg_color(btn_it->second.first,
                              is_on ? lv_color_hex(color) : lv_color_hex(0x222222), 0);
    lv_obj_set_style_text_color(btn_it->second.second,
                                is_on ? lv_color_hex(0x222222) : lv_color_hex(color), 0);
    // Force a repaint of the button (see arc comment above)
    lv_obj_invalidate(btn_it->second.first);
  }
}

// --- Panel state machine ---

void HaAutoPanel::set_panel_state_(PanelState new_state) {
  if (this->state_ == new_state) return;
  this->state_ = new_state;
  ESP_LOGI(TAG, "Panel state -> %d", (int) new_state);

  // Toggle the main room-grid container's visibility based on the new state.
  // When we're showing a status screen (anything not READY), hide the room
  // grid so the status screen has the full display.
  if (new_state == PanelState::READY) {
    if (this->main_container_ != nullptr) {
      lv_obj_remove_flag(this->main_container_, LV_OBJ_FLAG_HIDDEN);
    }
  } else {
    if (this->main_container_ != nullptr) {
      lv_obj_add_flag(this->main_container_, LV_OBJ_FLAG_HIDDEN);
    }
  }

  switch (new_state) {
    case PanelState::BOOTING:
      this->show_status_screen_("Starting...", "Connecting to WiFi", false);
      break;
    case PanelState::SETUP_REQUIRED: {
      std::string msg = this->build_setup_message_();
      this->show_status_screen_("Setup required", msg.c_str(), false);
      break;
    }
    case PanelState::AUTH_FAILED: {
      std::string dev_name = std::string(App.get_friendly_name());
      std::string setup = this->build_setup_message_();
      std::string msg = std::string("Home Assistant rejected the API token for '") +
                                    dev_name + "'.\n\n"
                                    "Open the web UI on this device to re-enter the token:\n" +
                                    setup;
      this->show_status_screen_("Auth failed", msg.c_str(), false);
      break;
    }
    case PanelState::NOT_AUTHORIZED: {
      std::string dev_name = std::string(App.get_friendly_name());
      std::string msg = std::string("This device ('") + dev_name +
                                    "') is not authorized to perform HA actions.\n\n"
                                    "Go to: " + this->ha_api_url_ +
                                    "\nDevices > ESPHome > click the gear beside '" +
                                    dev_name + "'\n"
                                    "Select 'Allow this device to perform HA actions' and Submit.\n\n"
                                    "Then tap Retry.";
      this->show_status_screen_("Not authorized", msg.c_str(), true);
      break;
    }
    case PanelState::CONNECTING:
      this->show_status_screen_("Connecting...", "Fetching rooms from Home Assistant", false);
      break;
    case PanelState::READY:
      this->hide_status_screen_();
      break;
  }
}

std::string HaAutoPanel::build_setup_message_() {
  // Two cases:
  //  1. Device is connected to the user's WiFi (STA mode). The user can
  //     reach the panel by IP at http://X.X.X.X/autopanel.
  //  2. Device is in AP fallback mode (no WiFi configured or station
  //     failed). The user must connect to the device's AP, then browse
  //     to its default address.
  wifi::WiFiComponent* wifi = wifi::global_wifi_component;
  if (wifi == nullptr) {
    return std::string("No WiFi component bound; cannot determine address.");
  }
  std::string msg;
  if (wifi->is_connected()) {
    const char* use_addr = wifi->get_use_address();
    if (use_addr != nullptr && use_addr[0] != '\0') {
      msg = std::string("Open http://") + use_addr + "/autopanel in a browser to configure.";
    } else {
      // use_address not set: show the first IP
      auto ips = wifi->get_ip_addresses();
      if (!ips.empty() && ips[0].is_set()) {
        msg = std::string("Open http://") + ips[0].str() + "/autopanel in a browser to configure.";
      } else {
        msg = std::string("WiFi connected but no IP address yet. Please wait.");
      }
    }
  } else if (wifi->has_ap() && wifi->is_ap_active()) {
    // AP fallback mode: tell the user the SSID/password to connect to
    auto ap = wifi->get_ap();
    std::string ssid(ap.get_ssid().c_str(), ap.get_ssid().size());
    std::string password(ap.get_password().c_str(), ap.get_password().size());
    msg = std::string("WiFi not connected. Connect to this device's AP:\n"
                      "SSID: ") + ssid +
          "\nPassword: " + password +
          "\n\nThen open http://192.168.4.1/autopanel in a browser.";
  } else {
    msg = std::string("WiFi not connected. Please check your WiFi configuration.");
  }
  return msg;
}

void HaAutoPanel::show_status_screen_(const char* title, const char* message, bool show_retry) {
  lv_obj_t* screen = lv_scr_act();
  if (screen == nullptr) return;

  ESP_LOGI(TAG, "show_status_screen: title='%s' show_retry=%d", title, (int) show_retry);
  ESP_LOGI(TAG, "  message: %s", message);

  if (this->status_container_ == nullptr) {
    // Create the status container on the first time we show it
    this->status_container_ = lv_obj_create(screen);
    lv_obj_set_pos(this->status_container_, 0, 0);
    lv_obj_set_size(this->status_container_, this->screen_width_, this->screen_height_);
    lv_obj_set_style_bg_color(this->status_container_, lv_color_hex(0x111827), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(this->status_container_, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_scrollbar_mode(this->status_container_, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_style_pad_all(this->status_container_, 30, 0);

    // Title at the top, centered horizontally, ~120px from top
    this->status_title_ = lv_label_create(this->status_container_);
    lv_obj_set_style_text_color(this->status_title_, lv_color_hex(0xFFFFFF), 0);
    lv_label_set_long_mode(this->status_title_, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(this->status_title_, this->screen_width_ - 60);
    lv_obj_set_style_text_align(this->status_title_, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(this->status_title_, LV_ALIGN_TOP_MID, 0, 120);

    // Message below the title, centered, padded
    this->status_message_ = lv_label_create(this->status_container_);
    lv_obj_set_style_text_color(this->status_message_, lv_color_hex(0x9CA3AF), 0);
    lv_label_set_long_mode(this->status_message_, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(this->status_message_, this->screen_width_ - 60);
    lv_obj_set_style_text_align(this->status_message_, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(this->status_message_, LV_ALIGN_TOP_MID, 0, 180);

    // Retry button at the bottom, centered
    this->status_retry_btn_ = lv_obj_create(this->status_container_);
    lv_obj_set_size(this->status_retry_btn_, 200, 60);
    lv_obj_set_style_bg_color(this->status_retry_btn_, lv_color_hex(0xFFFF00), 0);
    lv_obj_set_style_radius(this->status_retry_btn_, 8, 0);
    lv_obj_add_flag(this->status_retry_btn_, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_align(this->status_retry_btn_, LV_ALIGN_BOTTOM_MID, 0, -60);
    lv_obj_t* retry_label = lv_label_create(this->status_retry_btn_);
    lv_label_set_text(retry_label, "Retry");
    lv_obj_set_style_text_color(retry_label, lv_color_hex(0x111827), 0);
    lv_obj_center(retry_label);
    lv_obj_add_event_cb(this->status_retry_btn_, [](lv_event_t* event) {
      if (s_instance == nullptr) return;
      ESP_LOGI(TAG, "Retry tapped; re-probing authorization");
      s_instance->probe_authorization_();
    }, LV_EVENT_CLICKED, nullptr);
    lv_obj_add_flag(this->status_retry_btn_, LV_OBJ_FLAG_HIDDEN);  // hidden by default
  }

  lv_label_set_text(this->status_title_, title);
  lv_label_set_text(this->status_message_, message);
  if (show_retry) {
    lv_obj_remove_flag(this->status_retry_btn_, LV_OBJ_FLAG_HIDDEN);
  } else {
    lv_obj_add_flag(this->status_retry_btn_, LV_OBJ_FLAG_HIDDEN);
  }
  lv_obj_remove_flag(this->status_container_, LV_OBJ_FLAG_HIDDEN);
}

void HaAutoPanel::hide_status_screen_() {
  if (this->status_container_ != nullptr) {
    lv_obj_add_flag(this->status_container_, LV_OBJ_FLAG_HIDDEN);
  }
}

// --- Authorization probe ---

void HaAutoPanel::probe_authorization_() {
#ifdef USE_API_HOMEASSISTANT_SERVICES
  if (api::global_api_server == nullptr) {
    ESP_LOGW(TAG, "probe_authorization: api server null");
    this->set_panel_state_(PanelState::NOT_AUTHORIZED);
    return;
  }
  if (!api::global_api_server->is_connected()) {
    ESP_LOGW(TAG, "probe_authorization: HA not connected yet, will retry on next connect");
    return;
  }
  ESP_LOGI(TAG, "Probing HA authorization (5s timeout)...");
  this->auth_probe_pending_ = true;
  this->auth_probe_started_ms_ = millis();
  this->set_panel_state_(PanelState::CONNECTING);

  // Build the probe: homeassistant.update_entity on sun.sun
  // We do this through the global api server's send_homeassistant_action.
  // The probe's response is handled via the registered action response
  // callback (we use the high call_id of 0xA1701ACE for this probe; the
  // firmware's regular action calls use a different call_id range).

  api::HomeassistantActionRequest req;
  req.service = StringRef::from_lit("homeassistant.update_entity");
  req.call_id = AUTH_PROBE_CALL_ID;
  req.wants_response = true;
  // response_template is a StringRef (Jinja template for the response);
  // empty means "no template" which still returns the response. We just
  // need wants_response=true to get the on_success/on_error callback.
  req.response_template = StringRef();

  char entity_buf[32];
  strncpy(entity_buf, "sun.sun", sizeof(entity_buf) - 1);
  entity_buf[sizeof(entity_buf) - 1] = '\0';

  req.data.init(1);
  auto& kv = req.data.emplace_back();
  kv.key = StringRef::from_lit("entity_id");
  kv.value = StringRef(entity_buf);

  // Register the response callback BEFORE sending, so we don't miss the
  // response if HA replies before our lambda is in place.
  api::global_api_server->register_action_response_callback(
      AUTH_PROBE_CALL_ID,
      [this](const api::ActionResponse &resp) {
        bool success = resp.is_success();
        const char* err = nullptr;
        std::string err_str;
        if (!resp.get_error_message().empty()) {
          err_str = std::string(resp.get_error_message());
          err = err_str.c_str();
        }
        ESP_LOGD(TAG, "Probe response: success=%d err=%s", (int) success, err ? err : "(none)");
        this->on_auth_probe_response_(success, err);
      });

  api::global_api_server->send_homeassistant_action(req);
  ESP_LOGD(TAG, "Probe sent, awaiting response");
#else
  ESP_LOGW(TAG, "Cannot probe: USE_API_HOMEASSISTANT_SERVICES not defined");
  this->set_panel_state_(PanelState::NOT_AUTHORIZED);
#endif
}

void HaAutoPanel::on_auth_probe_response_(bool success, const char* error) {
  if (!this->auth_probe_pending_) return;
  this->auth_probe_pending_ = false;
  ESP_LOGI(TAG, "Auth probe response: success=%d error=%s",
           (int) success, error ? error : "(none)");
  if (success) {
    this->set_panel_state_(PanelState::READY);
  } else {
    // HA responded with an error - means the device IS authorized to call
    // services (the gate is open), but the probe service failed. We treat
    // this as authorized.
    ESP_LOGW(TAG, "Auth probe got error response, but the device is authorized: %s",
             error ? error : "");
    this->set_panel_state_(PanelState::READY);
  }
}

void HaAutoPanel::loop() {
  // Check authorization probe timeout
  if (this->auth_probe_pending_) {
    if (millis() - this->auth_probe_started_ms_ > AUTH_PROBE_TIMEOUT_MS) {
      ESP_LOGW(TAG, "Auth probe timed out (HA may have dropped the call silently)");
      this->auth_probe_pending_ = false;
      this->set_panel_state_(PanelState::NOT_AUTHORIZED);
    }
  }

  // Lazy-register the web handler once the web server is up.
  if (!this->web_handler_registered_) {
    this->register_web_handler_();
  }

  // Test/serial command interface. Reads one byte at a time from UART0
  // (the same UART the logger uses for log output). Useful for headless
  // testing - the test harness can write a single char and the panel
  // will respond without needing a physical touch.
  //
  // Commands (single ASCII char, terminated by \n or any non-alpha):
  //   'p' or 'r' = re-probe authorization (same as Retry button)
  //   's' = set state SETUP_REQUIRED
  //   'a' = set state AUTH_FAILED
  //   'n' = set state NOT_AUTHORIZED
  //   'c' = set state CONNECTING
  //   'g' = set state READY (re-render the room grid)
  //   'd' = re-run full discovery
  //   '0'..'9' = open detail view for room N (0 = first card)
  static char cmd_buf[8];
  static size_t cmd_len = 0;
  uint8_t b;
  while (uart_read_bytes(UART_NUM_0, &b, 1, 0) > 0) {
    if (b == '\n' || b == '\r' || b == ' ' || b == '\t') {
      if (cmd_len == 0) continue;
      cmd_buf[cmd_len] = '\0';
      ESP_LOGI(TAG, "[cmd] received: '%s'", cmd_buf);
      char c = cmd_buf[0];
      cmd_len = 0;
      switch (c) {
        case 'p': case 'r':
          this->probe_authorization_();
          break;
        case 's':
          this->set_panel_state_(PanelState::SETUP_REQUIRED);
          break;
        case 'a':
          this->set_panel_state_(PanelState::AUTH_FAILED);
          break;
        case 'n':
          this->set_panel_state_(PanelState::NOT_AUTHORIZED);
          break;
        case 'c':
          this->set_panel_state_(PanelState::CONNECTING);
          break;
        case 'g':
          // Force a re-render of the room grid
          this->set_panel_state_(PanelState::READY);
          break;
        case 'd':
          this->start_discovery_();
          break;
        case '0': case '1': case '2': case '3': case '4':
        case '5': case '6': case '7': case '8': case '9': {
          int idx = c - '0';
          if (idx >= 0 && idx < (int) this->room_cards_.size()) {
            ESP_LOGI(TAG, "[cmd] opening detail for room %d (%s)", idx,
                     this->room_cards_[idx].area.name.c_str());
            this->show_entity_detail_(idx);
          }
          break;
        }
        default:
          ESP_LOGW(TAG, "[cmd] unknown command '%c'", c);
          break;
      }
    } else if (cmd_len < sizeof(cmd_buf) - 1) {
      cmd_buf[cmd_len++] = (char) b;
    }
  }
}

// --- LittleFS + persistent config + web UI ---

// Minimal JSON value extractor - we only need string and number for the
// config file. Uses ArduinoJson which is already in the project.
#include "esphome/components/json/json_util.h"

bool HaAutoPanel::mount_storage_() {
  // Mount the 'storage' partition at /storage using esp_littlefs.
  // The partition must be declared in partitions.csv as subtype=littlefs.
  // ESP-IDF's esp_littlefs driver is the IDF-native one (not joltwallet).
  esp_vfs_littlefs_conf_t conf = {};
  conf.base_path = "/storage";
  conf.partition_label = "storage";
  conf.format_if_mount_failed = true;  // first boot will format
  conf.dont_mount_by_default = false;
  esp_err_t err = esp_vfs_littlefs_register(&conf);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_vfs_littlefs_register failed: %s", esp_err_to_name(err));
    return false;
  }
  ESP_LOGI(TAG, "LittleFS mounted at /storage");
  return true;
}

bool HaAutoPanel::read_config_file_(std::string &out) {
  FILE *f = fopen(this->config_path_.c_str(), "r");
  if (f == nullptr) {
    ESP_LOGW(TAG, "No config file at %s", this->config_path_.c_str());
    return false;
  }
  out.clear();
  char buf[512];
  size_t n;
  while ((n = fread(buf, 1, sizeof(buf), f)) > 0) {
    out.append(buf, n);
  }
  fclose(f);
  return true;
}

bool HaAutoPanel::write_config_file_(const std::string &body) {
  FILE *f = fopen(this->config_path_.c_str(), "w");
  if (f == nullptr) {
    ESP_LOGE(TAG, "Failed to open %s for writing", this->config_path_.c_str());
    return false;
  }
  size_t written = fwrite(body.c_str(), 1, body.size(), f);
  fclose(f);
  if (written != body.size()) {
    ESP_LOGE(TAG, "Short write: %u of %u", (unsigned) written, (unsigned) body.size());
    return false;
  }
  ESP_LOGI(TAG, "Wrote %u bytes to %s", (unsigned) body.size(), this->config_path_.c_str());
  return true;
}

void HaAutoPanel::apply_config_file_(const std::string &body) {
  // Parse the JSON and apply the keys we know about. Unrecognized keys
  // are ignored (forward compatibility).
  PsramJsonDocument doc(&s_psram_allocator);
  DeserializationError err = deserializeJson(doc, body);
  if (err) {
    ESP_LOGW(TAG, "Config JSON parse failed: %s", err.c_str());
    return;
  }
  if (doc["api_url"].is<const char*>()) {
    this->ha_api_url_ = doc["api_url"].as<std::string>();
  }
  if (doc["api_token"].is<const char*>()) {
    this->ha_api_password_ = doc["api_token"].as<std::string>();
  }
  this->config_loaded_ = true;
  ESP_LOGI(TAG, "Config applied: api_url=%s, token_len=%u",
           this->ha_api_url_.c_str(), (unsigned) this->ha_api_password_.size());
}

void HaAutoPanel::apply_runtime_config_() {
  // No-op for now: the runtime config (api_url, api_token) is read at
  // setup() and stays in memory. A future enhancement would be to also
  // load entity filtering / display overrides here.
}

void HaAutoPanel::boot_from_storage_() {
  if (!this->mount_storage_()) {
    // Mount failed - keep the YAML config (ha_api_url_ stays as the
    // YAML default) and show SETUP_REQUIRED so the user can recover.
    ESP_LOGW(TAG, "Storage mount failed; falling back to YAML config");
    this->set_panel_state_(PanelState::SETUP_REQUIRED);
    return;
  }
  std::string body;
  if (!this->read_config_file_(body)) {
    // No file - first boot. Keep YAML defaults and prompt the user
    // to set up via the web UI.
    ESP_LOGI(TAG, "No saved config; first boot");
    this->set_panel_state_(PanelState::SETUP_REQUIRED);
    return;
  }
  this->apply_config_file_(body);
  this->apply_runtime_config_();
  // Don't change state here - the discovery will run via wifi.on_connect
  // or api.on_client_connected and SETUP_REQUIRED will be replaced by
  // READY on success. If the saved config is bad, the discovery/auth
  // probe will surface the failure.
}

void HaAutoPanel::register_web_handler_() {
  if (this->web_handler_registered_) return;
  if (web_server_base::global_web_server_base == nullptr) {
    ESP_LOGW(TAG, "global_web_server_base is null; deferring web handler registration");
    return;
  }

  // Register GET /autopanel and POST /autopanel/save
  class AutoPanelHandler : public AsyncWebHandler {
   public:
    AutoPanelHandler(HaAutoPanel *parent) : parent_(parent) {}
    bool canHandle(AsyncWebServerRequest *request) const override {
      char url_buf[AsyncWebServerRequest::URL_BUF_SIZE];
      std::string url(request->url_to(url_buf).str());
      return url == "/autopanel" || url == "/autopanel/save" || url == "/autopanel/api/areas";
    }
    void handleRequest(AsyncWebServerRequest *request) override {
      char url_buf[AsyncWebServerRequest::URL_BUF_SIZE];
      std::string url(request->url_to(url_buf).str());
      const char *method_str = (request->method() == HTTP_GET) ? "GET" : "POST";
      ESP_LOGI(TAG, "web: %s %s", method_str, url.c_str());
      if (url == "/autopanel") {
        if (request->method() == HTTP_GET) {
          parent_->handle_setup_get_(request);
        } else {
          request->send(405, "text/plain", "Method not allowed");
        }
      } else if (url == "/autopanel/save") {
        if (request->method() == HTTP_POST) {
          parent_->handle_setup_post_(request);
        } else {
          request->send(405, "text/plain", "Method not allowed");
        }
      } else {
        request->send(404, "text/plain", "Not found");
      }
    }
   private:
    HaAutoPanel *parent_;
  };

  web_server_base::global_web_server_base->add_handler(new AutoPanelHandler(this));
  this->web_handler_registered_ = true;
  ESP_LOGI(TAG, "Web handler registered for /autopanel");
}

void HaAutoPanel::handle_setup_get_(AsyncWebServerRequest *request) {
  // Build a tiny HTML form. Post values: api_url, api_token.
  std::string body;
  body += "<!doctype html><html><head><meta charset='utf-8'>"
          "<title>ha_autopanel setup</title>"
          "<style>"
          "body{font-family:sans-serif;max-width:560px;margin:24px auto;padding:0 16px;color:#eee;background:#111827}"
          "h1{color:#fff;font-size:20px}"
          "label{display:block;margin:16px 0 4px;color:#9ca3af;font-size:13px}"
          "input[type=text],input[type=password]{width:100%;padding:8px;box-sizing:border-box;"
          "  background:#1f2937;color:#fff;border:1px solid #374151;border-radius:4px;font-family:inherit;font-size:14px}"
          "button{margin-top:20px;padding:10px 18px;background:#facc15;color:#111827;border:0;"
          "  border-radius:4px;font-weight:600;font-size:14px;cursor:pointer}"
          ".note{margin-top:24px;padding:10px;background:#1f2937;border-left:3px solid #facc15;color:#9ca3af;font-size:12px}"
          "</style></head><body>";

  body += "<h1>ha_autopanel setup</h1>";
  body += "<form method='POST' action='/autopanel/save'>";
  body += "<label>Home Assistant URL</label>";
  body += "<input type='text' name='api_url' value='";
  body += this->ha_api_url_;
  body += "' placeholder='http://homeassistant.local:8123'>";
  body += "<label>Home Assistant long-lived access token</label>";
  body += "<input type='password' name='api_token' value='";
  body += this->ha_api_password_;
  body += "' placeholder='paste token from HA profile page'>";
  body += "<br><button type='submit'>Save &amp; restart</button>";
  body += "</form>";
  body += "<div class='note'>Settings are saved to /storage/autopanel.cfg on the device. "
          "The device restarts after save. The YAML-supplied values are used as a fallback. "
          "In HA, allow this device to perform Home Assistant actions under "
          "Devices &gt; ESPHome &gt; configure.</div>";
  body += "</body></html>";

  AsyncWebServerResponse *response =
      request->beginResponse(200, "text/html", body.c_str());
  request->send(response);
}

void HaAutoPanel::handle_setup_post_(AsyncWebServerRequest *request) {
  // Read the form fields by name. The ESP-IDF web server API gives us
  // a name-based lookup, not an index-based one.
  std::string new_url = this->ha_api_url_;
  std::string new_token = this->ha_api_password_;
  if (request->hasArg("api_url")) {
    new_url = request->arg("api_url");
  }
  if (request->hasArg("api_token")) {
    new_token = request->arg("api_token");
  }
  if (new_url.empty() || new_token.empty()) {
    request->send(400, "text/plain", "api_url and api_token are required");
    return;
  }
  // Build a JSON config blob and persist
  std::string body;
  body += "{\"api_url\":\"";
  for (char c : new_url) {
    if (c == '"' || c == '\\') body += '\\';
    body += c;
  }
  body += "\",\"api_token\":\"";
  for (char c : new_token) {
    if (c == '"' || c == '\\') body += '\\';
    body += c;
  }
  body += "\"}";
  if (!this->write_config_file_(body)) {
    request->send(500, "text/plain", "Failed to write config");
    return;
  }
  // Acknowledge then restart
  std::string ack = "<!doctype html><html><body style='font-family:sans-serif;background:#111827;color:#fff;padding:24px'>";
  ack += "<h1>Saved</h1><p>Config written. Device restarting in 2 seconds.</p></body></html>";
  AsyncWebServerResponse *resp = request->beginResponse(200, "text/html", ack.c_str());
  request->send(resp);
  // Restart after a short delay so the response is flushed
  ESP_LOGI(TAG, "Restarting in 2s to apply new config");
  set_timeout(2000, []() {
    App.safe_reboot();
  });
}

}  // namespace ha_autopanel
}  // namespace esphome
