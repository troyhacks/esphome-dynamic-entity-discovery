#include "dynamic_entity_discovery.h"
#include "esphome/core/log.h"
#include "esphome/components/json/json_util.h"
#include <lvgl.h>
#include "esphome/components/lvgl/lvgl_proxy.h"
#include "esp_heap_caps.h"

namespace esphome {
namespace dynamic_entity_discovery {

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

static const char* TAG = "dynamic_entity_discovery";

const uint32_t DynamicEntityDiscovery::ROOM_COLORS_[] = {
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
static DynamicEntityDiscovery* s_instance = nullptr;

// Callback data for arc value changes - allocated on heap, freed on LV_EVENT_DELETE
struct ArcCallbackData {
    int entity_idx;
    lv_obj_t* label;
};

void DynamicEntityDiscovery::setup() {
  ESP_LOGI(TAG, "Dynamic Entity Discovery starting...");
  s_instance = this;

  ESP_LOGI(TAG, "  HA API URL: %s", this->ha_api_url_.c_str());
  ESP_LOGI(TAG, "  Grid: %dx%d cards (%dx%d px each, gap %dx%d)",
           this->grid_cols_, this->grid_rows_,
           this->grid_card_width_, this->grid_card_height_,
           this->grid_gap_x_, this->grid_gap_y_);
  ESP_LOGI(TAG, "  http_request configured: %s", this->http_request_ ? "YES" : "NO");

  // Paint the LVGL screen dark immediately to avoid a white flash at boot
  // (frame buffer is uninitialized white until something draws to it).
  lv_obj_t* screen = lv_scr_act();
  if (screen != nullptr) {
    lv_obj_set_style_bg_color(screen, lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, LV_PART_MAIN);
  }

  ESP_LOGI(TAG, "Note: Use trigger_discovery() after boot to create UI");
}

void DynamicEntityDiscovery::trigger_discovery() {
  ESP_LOGI(TAG, "trigger_discovery() called");

  if (s_instance == nullptr) {
    ESP_LOGE(TAG, "Instance not set!");
    return;
  }

  s_instance->start_discovery_();
}

void DynamicEntityDiscovery::fetch_areas_() {
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

void DynamicEntityDiscovery::fetch_entities_() {
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

    // Check if entity has brightness (lights with brightness support)
    if (domain == "light" && !attributes["brightness"].isNull()) {
      entity.has_brightness = true;
    }

    this->entities_by_area_[area_id].push_back(entity);
  }
  ESP_LOGI(TAG, "  Parsed %d entities into areas", (int)this->entities_by_area_.size());
}

void DynamicEntityDiscovery::start_discovery_() {
  ESP_LOGI(TAG, "=== Starting Dynamic Entity Discovery ===");

  // Clear previous discovery data to avoid heap growth on retry
  this->discovered_areas_.clear();
  this->entities_by_area_.clear();
  this->room_cards_.clear();

  // Fetch real areas and entities from HA
  fetch_areas_();
  fetch_entities_();

  if (discovered_areas_.empty()) {
    ESP_LOGW(TAG, "No areas discovered from HA - check API token and connectivity");
    return;
  }

  filter_and_build_room_cards_();

  // Create the UI
  create_ui_from_room_cards_();

  ESP_LOGI(TAG, "=== Discovery Complete ===");
}

void DynamicEntityDiscovery::dump_config() {
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
  ESP_LOGI(TAG, "  Grid: %dx%d", this->grid_cols_, this->grid_rows_);
  ESP_LOGI(TAG, "  Discovered areas: %d", (int)discovered_areas_.size());
}

bool DynamicEntityDiscovery::is_area_excluded_(const std::string& area_name) const {
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

bool DynamicEntityDiscovery::is_entity_excluded_(const std::string& entity_id) const {
  for (const auto& excluded : this->exclude_entities_) {
    if (entity_id == excluded) return true;
  }
  return false;
}

bool DynamicEntityDiscovery::is_domain_included_(const std::string& domain) const {
  for (const auto& d : this->entity_domains_) {
    if (domain == d) return true;
  }
  return false;
}

void DynamicEntityDiscovery::filter_and_build_room_cards_() {
  room_cards_.clear();

  for (size_t i = 0; i < discovered_areas_.size(); i++) {
    const auto& area = discovered_areas_[i];

    if (is_area_excluded_(area.name)) {
      ESP_LOGI(TAG, "Skipping excluded area: %s", area.name.c_str());
      continue;
    }

    RoomCard card;
    card.area = area;
    card.grid_index = (int)room_cards_.size();
    card.x = get_card_x_(card.grid_index % this->grid_cols_);
    card.y = get_card_y_(card.grid_index / this->grid_cols_);
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

void DynamicEntityDiscovery::create_ui_from_room_cards_() {
  ESP_LOGI(TAG, "Creating dynamic UI for %d room cards", (int)room_cards_.size());

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

  // Create a scrollable container for the room cards
  // In LVGL 9, all objects can scroll - just make container larger than screen
  this->main_container_ = lv_obj_create(screen);
  lv_obj_set_pos(this->main_container_, 0, 0);
  lv_obj_set_size(this->main_container_, 1024, 600);  // Full screen size
  lv_obj_set_style_bg_color(this->main_container_, lv_color_hex(0x111827), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(this->main_container_, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_scrollbar_mode(this->main_container_, LV_SCROLLBAR_MODE_OFF);  // Clean, no scrollbar
  lv_obj_set_style_pad_all(this->main_container_, 0, 0);  // No padding
  lv_obj_set_style_border_width(this->main_container_, 0, 0);  // No border
  lv_obj_set_style_border_color(this->main_container_, lv_color_hex(0x111827), 0);  // Hide border
  lv_obj_set_scroll_snap_y(this->main_container_, LV_SCROLL_SNAP_START);  // Snap to row when scrolling

  for (const auto& room : room_cards_) {
    create_room_card_(this->main_container_, room);
  }

  // Expand page height to fit all cards + gap for scrolling
  int num_rows = (int)(room_cards_.size() + this->grid_cols_ - 1) / this->grid_cols_;
  int total_height = this->start_y_ + num_rows * (this->grid_card_height_ + this->grid_gap_y_) + 50;
  lv_obj_set_style_height(this->main_container_, total_height, LV_PART_MAIN);

  ESP_LOGI(TAG, "UI creation complete (page height: %d for %d rows)", total_height, num_rows);
}

void DynamicEntityDiscovery::create_room_card_(void* parent, const RoomCard& room) {
  lv_obj_t* card = lv_obj_create((lv_obj_t*) parent);
  lv_obj_set_pos(card, room.x, room.y);
  lv_obj_set_size(card, this->grid_card_width_, this->grid_card_height_);
  lv_obj_set_style_bg_color(card, lv_color_hex(0x1a1a2e), 0);
  lv_obj_set_style_radius(card, 12, 0);
  lv_obj_set_style_border_width(card, 0, 0);
  lv_obj_set_scrollbar_mode(card, LV_SCROLLBAR_MODE_OFF);
  lv_obj_remove_flag(card, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_remove_flag(card, LV_OBJ_FLAG_SEND_DRAW_TASK_EVENTS);  // Skip draw notifications

  // Arc for brightness - CREATE FIRST so it's below label button in z-order
  lv_obj_t* arc = lv_arc_create(card);
  lv_obj_set_size(arc, 240, 240);
  lv_obj_align(arc, LV_ALIGN_CENTER, 0, 15);
  lv_arc_set_min_value(arc, 0);
  lv_arc_set_max_value(arc, 100);
  lv_arc_set_value(arc, 50);
  lv_arc_set_bg_start_angle(arc, 135);
  lv_arc_set_bg_end_angle(arc, 405);  // 135 + 270
  lv_obj_set_style_arc_color(arc, lv_color_hex(0x404040), LV_PART_MAIN);
  lv_obj_set_style_arc_width(arc, 20, LV_PART_MAIN);
  lv_obj_set_style_arc_color(arc, lv_color_hex(room.color), LV_PART_INDICATOR);
  lv_obj_set_style_arc_width(arc, 20, LV_PART_INDICATOR);
  lv_obj_set_user_data(arc, (void*)(intptr_t)room.grid_index);

  lv_obj_add_event_cb(arc, [](lv_event_t* event) {
    lv_obj_t* arc = (lv_obj_t*)lv_event_get_target(event);
    int value = lv_arc_get_value(arc);
    int room_index = (int)(intptr_t)lv_obj_get_user_data(arc);
    ESP_LOGI(TAG, "Arc value changed: room=%d, value=%d", room_index, value);
  }, LV_EVENT_VALUE_CHANGED, nullptr);

  // ON/OFF button
  lv_obj_t* btn = lv_obj_create(card);
  lv_obj_set_size(btn, 120, 35);
  lv_obj_align(btn, LV_ALIGN_BOTTOM_MID, 0, -10);
  lv_obj_set_style_radius(btn, 6, 0);
  lv_obj_set_style_border_width(btn, 2, 0);
  lv_obj_set_style_border_color(btn, lv_color_hex(room.color), 0);
  lv_obj_set_style_bg_color(btn, lv_color_hex(0x222222), 0);
  lv_obj_set_scrollbar_mode(btn, LV_SCROLLBAR_MODE_OFF);

  lv_obj_t* btn_label = lv_label_create(btn);
  lv_label_set_text(btn_label, "ON/OFF");
  lv_obj_set_style_text_color(btn_label, lv_color_hex(room.color), 0);
  lv_obj_center(btn_label);
  lv_obj_set_user_data(btn, (void*)(intptr_t)room.grid_index);

  lv_obj_add_event_cb(btn, [](lv_event_t* event) {
    lv_obj_t* btn = (lv_obj_t*)lv_event_get_target(event);
    int room_index = (int)(intptr_t)lv_obj_get_user_data(btn);
    ESP_LOGI(TAG, "Button clicked: room=%d", room_index);
  }, LV_EVENT_CLICKED, nullptr);

  // Room name label - clickable area using transparent button
  // CREATE LAST so it's on TOP in z-order and receives touches
  lv_obj_t* label_btn = lv_obj_create(card);
  lv_obj_remove_style_all(label_btn);  // Start with clean slate to avoid default button styles
  lv_obj_set_size(label_btn, 180, 40);  // 180px wide for touch target
  lv_obj_align(label_btn, LV_ALIGN_CENTER, 0, 0);  // Centered in card
  lv_obj_set_style_bg_opa(label_btn, LV_OPA_TRANSP, 0);  // Invisible background
  lv_obj_set_style_border_width(label_btn, 0, 0);  // No border
  lv_obj_set_style_radius(label_btn, 6, 0);
  lv_obj_set_style_pad_all(label_btn, 5, 0);  // Ensure touch padding
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

void DynamicEntityDiscovery::show_entity_detail_(int room_index) {
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

void DynamicEntityDiscovery::show_room_grid_() {
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

void DynamicEntityDiscovery::create_entity_control_(void* parent, const Entity& entity, int entity_index, int y_pos, uint32_t color) {
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
    // Brightness arc for lights
    lv_obj_t* arc = lv_arc_create(control);
    lv_obj_set_size(arc, 50, 50);
    lv_obj_align(arc, LV_ALIGN_RIGHT_MID, -100, 0);
    lv_arc_set_min_value(arc, 0);
    lv_arc_set_max_value(arc, 100);
    lv_arc_set_value(arc, 75);  // Default to 75%
    lv_arc_set_bg_start_angle(arc, 135);
    lv_arc_set_bg_end_angle(arc, 405);
    lv_obj_set_style_arc_color(arc, lv_color_hex(0x404040), LV_PART_MAIN);
    lv_obj_set_style_arc_width(arc, 6, LV_PART_MAIN);
    lv_obj_set_style_arc_color(arc, lv_color_hex(color), LV_PART_INDICATOR);
    lv_obj_set_style_arc_width(arc, 6, LV_PART_INDICATOR);
    // (user_data is set below to the heap-allocated ArcCallbackData)

    // Brightness percentage label
    lv_obj_t* pct_label = lv_label_create(control);
    lv_label_set_text_fmt(pct_label, "%d%%", 75);
    lv_obj_set_style_text_color(pct_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(pct_label, LV_ALIGN_RIGHT_MID, -30, 0);

    // Allocate callback data struct - freed in LV_EVENT_DELETE handler
    ArcCallbackData* cb_data = new ArcCallbackData{entity_index, pct_label};
    lv_obj_set_user_data(arc, cb_data);

    lv_obj_add_event_cb(arc, [](lv_event_t* event) {
      lv_obj_t* arc = (lv_obj_t*)lv_event_get_target(event);
      int value = lv_arc_get_value(arc);
      ArcCallbackData* data = (ArcCallbackData*)lv_obj_get_user_data(arc);
      lv_label_set_text_fmt(data->label, "%d%%", value);
      ESP_LOGI(TAG, "Entity brightness changed: index=%d, value=%d%%", data->entity_idx, value);
      // TODO: Call HA API to set brightness
    }, LV_EVENT_VALUE_CHANGED, nullptr);

    // Free callback data when arc is destroyed to prevent memory leak
    lv_obj_add_event_cb(arc, [](lv_event_t* event) {
      lv_obj_t* target = (lv_obj_t*)lv_event_get_target(event);
      ArcCallbackData* data = (ArcCallbackData*)lv_obj_get_user_data(target);
      delete data;
    }, LV_EVENT_DELETE, nullptr);
  } else if (entity.domain == "switch") {
    // Switch has toggle button
    lv_obj_t* toggle_btn = lv_obj_create(control);
    lv_obj_set_size(toggle_btn, 100, 40);
    lv_obj_align(toggle_btn, LV_ALIGN_RIGHT_MID, -15, 0);
    lv_obj_set_style_radius(toggle_btn, 6, 0);
    lv_obj_set_style_bg_color(toggle_btn, lv_color_hex(0x333333), 0);
    lv_obj_set_user_data(toggle_btn, (void*)(intptr_t)entity_index);

    lv_obj_t* toggle_label = lv_label_create(toggle_btn);
    lv_label_set_text(toggle_label, "Toggle");
    lv_obj_set_style_text_color(toggle_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_center(toggle_label);

    lv_obj_add_event_cb(toggle_btn, [](lv_event_t* event) {
      lv_obj_t* btn = (lv_obj_t*)lv_event_get_target(event);
      int entity_idx = (int)(intptr_t)lv_obj_get_user_data(btn);
      ESP_LOGI(TAG, "Switch toggled: entity_idx=%d", entity_idx);
      // TODO: Call HA API to toggle switch
    }, LV_EVENT_CLICKED, nullptr);
  } else {
    // Read-only entities (sensor, binary_sensor, climate, cover, etc.) - just show domain text aligned right
    // Nothing to do - domain label already shows the type
  }

  ESP_LOGI(TAG, "  Created control for entity: %s (%s)", entity.name.c_str(), entity.domain.c_str());
}

int DynamicEntityDiscovery::get_card_x_(int col) const {
  return this->start_x_ + col * (this->grid_card_width_ + this->grid_gap_x_);
}

int DynamicEntityDiscovery::get_card_y_(int row) const {
  return this->start_y_ + row * (this->grid_card_height_ + this->grid_gap_y_);
}

uint32_t DynamicEntityDiscovery::get_room_color_(int index) const {
  return ROOM_COLORS_[index % MAX_ROOM_COLORS_];
}

}  // namespace dynamic_entity_discovery
}  // namespace esphome
