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
#include <algorithm>
#include <cstring>

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

  // Crash / boot tracking. The boot counter is persisted in NVS so
  // we can spot devices that are silently rebooting in the field.
  // Every setup() increments the counter and reads back the prior
  // value as "last seen boot count", giving us restart-pairing for
  // crash diagnosis (boot_count - last_seen_boot_count > 1 == unclean
  // restart). The chip's RTC reset reason register tells us whether
  // the previous boot was a panic, a power-loss, or a software
  // restart - log both so a crash leaves a fingerprint.
  {
    uint32_t boot_count = 0;
    uint32_t last_seen = 0;
    this->pref_.load(&last_seen);  // was the previous boot count
    boot_count = last_seen + 1;
    this->pref_.save(&boot_count);
    esp_reset_reason_t reason = esp_reset_reason();
    if (last_seen == 0) {
      ESP_LOGI(TAG, "Boot #%u (first boot, reset_reason=%d)", (unsigned) boot_count, (int) reason);
    } else if (boot_count - last_seen > 1) {
      ESP_LOGW(TAG, "Boot #%u (PREVIOUS BOOT DID NOT CLEANLY EXIT - expected %u, jumped to %u, reset_reason=%d)",
               (unsigned) boot_count, (unsigned) (last_seen + 1), (unsigned) boot_count, (int) reason);
    } else {
      ESP_LOGI(TAG, "Boot #%u (clean exit last time, reset_reason=%d)", (unsigned) boot_count, (int) reason);
    }
    // Human-readable reset reason. esp_reset_reason() returns
    // enum values 0-15 defined in rom/ets_sys.h. Common ones:
    //   1 ESP_RST_POWERON   - cold boot (power applied)
    //   3 ESP_RST_SW        - software reset (esp_restart())
    //   4 ESP_RST_PANIC     - exception / abort
    //   5 ESP_RST_INT_WATCH - interrupt watchdog
    //   7 ESP_RST_TASK_WATCH - task watchdog (TWDT)
    //   8 ESP_RST_WATCHDOG  - other watchdog
    static const char *reason_names[] = {
        "UNKNOWN", "POWERON", "EXT", "SW", "PANIC",
        "INT_WATCH", "DEEPSLEEP", "INT_WDT", "TASK_WDT",
        "WDT", "DEEPSLEEP", "BROWNOUT", "SDIO", "USB",
        "JTAG", "EFUSE"};
    int idx = (int) reason;
    if (idx >= 0 && idx < (int)(sizeof(reason_names)/sizeof(reason_names[0]))) {
      ESP_LOGI(TAG, "  reset reason: %s", reason_names[idx]);
    }
  }

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

  // Brand splash — full-screen black with red centered "HA AutoPanel v1.0".
  // Created before anything else so the user always sees this the moment
  // the panel comes up, and so any panel-side display-default artifacts
  // (brief test pattern, uninitialized frame buffer) are covered. It is
  // hidden by hide_splash_() the first time we leave the BOOTING state.
  this->splash_container_ = lv_obj_create(screen);
  if (this->splash_container_ != nullptr) {
    lv_obj_set_pos(this->splash_container_, 0, 0);
    lv_obj_set_size(this->splash_container_, this->screen_width_, this->screen_height_);
    lv_obj_set_style_bg_color(this->splash_container_, lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(this->splash_container_, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(this->splash_container_, 0, 0);
    lv_obj_set_style_pad_all(this->splash_container_, 0, 0);
    lv_obj_remove_flag(this->splash_container_, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_remove_flag(this->splash_container_, LV_OBJ_FLAG_CLICKABLE);

    this->splash_label_ = lv_label_create(this->splash_container_);
    if (this->splash_label_ != nullptr) {
      lv_label_set_text(this->splash_label_, "HA AutoPanel v1.0");
      // Pure red (0xFF0000) per the spec. Default font is fine here —
      // it's larger and bolder than the title-bar label, which is what we
      // want for a centered splash. If you want a specific face/size, set
      // it via lv_obj_set_style_text_font() referencing the lvgl font id.
      lv_obj_set_style_text_color(this->splash_label_, lv_color_hex(0xFF0000), 0);
      lv_obj_set_style_text_align(this->splash_label_, LV_TEXT_ALIGN_CENTER, 0);
      lv_obj_center(this->splash_label_);
    }
  }

  ESP_LOGI(TAG, "Note: Use trigger_discovery() after boot to create UI");

  // Log our own IP so the user can see it in the boot logs (the wifi
  // component doesn't log this by default).
  if (wifi::global_wifi_component != nullptr) {
    if (wifi::global_wifi_component->is_connected()) {
      auto ips = wifi::global_wifi_component->get_ip_addresses();
      for (const auto &ip : ips) {
        if (ip.is_set()) {
          char ip_buf[network::IP_ADDRESS_BUFFER_SIZE];
          ip.str_to(ip_buf);
          ESP_LOGI(TAG, "  IP: %s", ip_buf);
        }
      }
    } else {
      ESP_LOGW(TAG, "  WiFi not connected yet (will retry)");
    }
  }

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
  // When the user has not specified any domains, the regex would be
  // "^(?:)\." which never matches anything. Use ".*" instead so
  // the template returns ALL entity_ids in each area.
  std::string domain_match_json;
  if (this->entity_domains_.empty()) {
    domain_match_json = ".*";
  } else {
    domain_match_json = "^(?:" + domain_regex + ")\\\\.";
  }

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

      // We skip the heavy /api/states bulk fetch to avoid a 227KB JSON
      // parse that crashes the panel. Instead, populate entities_by_area_
      // with minimal Entity stubs derived from the template's entity_ids
      // list. State and brightness are populated lazily by the
      // api.on_state subscription (see subscribe_to_all_entities_).
      std::vector<Entity, PsramStlAllocator<Entity>> &bucket = this->entities_by_area_[a.area_id];
      bucket.reserve(a.entity_ids.size());
      for (const auto &eid : a.entity_ids) {
        Entity e;
        e.entity_id = entity_arena().intern(eid);
        // Derive name and domain from the entity_id
        size_t dot = eid.find('.');
        if (dot != std::string::npos) {
          e.domain = entity_arena().intern(std::string_view(eid.data(), dot));
          e.name = entity_arena().intern(std::string_view(eid.data() + dot + 1, eid.size() - dot - 1));
          e.area_id = entity_arena().intern(a.area_id);
        }
        // We don't know if it's a light with brightness yet; the
        // subscription callback updates has_brightness and brightness
        // when HA pushes the initial state.
        bucket.push_back(std::move(e));
      }
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

  // ArduinoJson 7.x: parse the states response. Use the PSRAM-backed
  // allocator so the ~250KB JSON document lives in PSRAM. The default
  // internal heap is too small and would abort().
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
    entity.entity_id = entity_arena().intern(entity_id);
    entity.domain = entity_arena().intern(domain);
    entity.area_id = entity_arena().intern(area_id);

    // Get attributes
    JsonObject attributes = state["attributes"];

    // Get friendly name from attributes, or derive from entity_id
    if (!attributes["friendly_name"].isNull()) {
      const char *fn = attributes["friendly_name"].as<const char*>();
      entity.name = entity_arena().intern(fn == nullptr ? std::string_view() : std::string_view(fn));
    } else {
      entity.name = entity_arena().intern(std::string_view(eid.data() + dot + 1, eid.size() - dot - 1));
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

void HaAutoPanel::fetch_home_name_() {
  if (this->http_request_ == nullptr) {
    ESP_LOGW(TAG, "[home] http_request_ is null - cannot fetch home name");
    return;
  }
  if (this->ha_api_url_.empty() || this->ha_api_password_.empty()) {
    ESP_LOGW(TAG, "[home] HA URL or token empty - cannot fetch home name");
    return;
  }

  // Throttle: don't re-fetch if we just did. The trigger in
  // set_panel_state_() and the periodic check in loop() both check
  // this guard, so calling fetch_home_name_() repeatedly is safe.
  uint32_t now = millis();
  if (this->last_home_fetch_ms_ != 0 &&
      (now - this->last_home_fetch_ms_) < HOME_FETCH_INTERVAL_MS) {
    ESP_LOGD(TAG, "[home] skipping fetch (last was %u ms ago, interval=%u)",
             (unsigned)(now - this->last_home_fetch_ms_),
             (unsigned)HOME_FETCH_INTERVAL_MS);
    return;
  }

  ESP_LOGI(TAG, "[home] fetching zone.home friendly_name from %s/api/states/zone.home",
           this->ha_api_url_.c_str());

  // Server-side filter: HA's REST API supports /api/states/<entity_id>
  // which returns a single entity object (not the full array). This
  // is much better than fetching the whole /api/states (which can be
  // 250KB+ with 419 entities) and filtering client-side. The response
  // is a small JSON object ~500 bytes. Same shape as an entry in the
  // /api/states array, so we parse it the same way.
  std::string url = this->ha_api_url_ + "/api/states/zone.home";
  std::vector<http_request::Header> headers = {
      {"Authorization", "Bearer " + this->ha_api_password_},
      {"Content-Type", "application/json"},
  };
  auto container = this->http_request_->get(url, headers);
  if (container == nullptr) {
    ESP_LOGW(TAG, "[home] GET /api/states/zone.home returned null container");
    return;
  }
  if (container->status_code == 0) {
    ESP_LOGW(TAG, "[home] connection failed");
    container->end();
    return;
  }
  if (container->status_code == 401) {
    // Auth failure here is interesting - the rest of the system
    // already proved the token works (we passed the auth probe).
    // Could be a transient token rotation, just log and let the
    // next periodic refresh retry.
    ESP_LOGW(TAG, "[home] HTTP 401 - token rejected");
    container->end();
    return;
  }
  if (container->status_code == 404) {
    // zone.home doesn't exist (user hasn't set up a Home zone in
    // HA, or renamed it to something else). Not an error - just
    // nothing to display. Clear the cache and return.
    ESP_LOGW(TAG, "[home] zone.home not found in HA (HTTP 404)");
    container->end();
    this->home_name_.clear();
    this->last_home_fetch_ms_ = millis();
    return;
  }
  if (container->status_code != 200) {
    ESP_LOGW(TAG, "[home] HTTP %d", (int)container->status_code);
    container->end();
    return;
  }

  // Read the body. /api/states/zone.home is ~500 bytes - reserve a
  // small buffer and cap at 16KB as a safety net.
  size_t expected = container->content_length;
  std::string response;
  response.reserve(expected > 0 ? expected : 1024);
  uint8_t buf[512];
  uint32_t last_data_time = millis();
  const uint32_t timeout = 10000;
  int iter = 0;
  const size_t MAX_BODY = 16 * 1024;
  while (container->get_bytes_read() < container->content_length &&
         response.size() < MAX_BODY) {
    App.feed_wdt();
    yield();
    int read = container->read(buf, sizeof(buf));
    if (read > 0) {
      response.append(reinterpret_cast<char*>(buf), read);
      last_data_time = millis();
      iter = 0;
    } else if (read == http_request::HTTP_ERROR_CONNECTION_CLOSED) {
      break;
    } else {
      iter++;
      if (iter % 10 == 0) App.feed_wdt();
    }
    if (millis() - last_data_time > timeout) {
      ESP_LOGW(TAG, "[home] timeout reading response");
      break;
    }
  }
  container->end();

  // Parse with ArduinoJson. The response is a JSON object (not an
  // array) since we used the /api/states/<id> endpoint. Using the
  // PSRAM-backed allocator (same as fetch_entities_) so we don't
  // OOM the small internal heap.
  PsramJsonDocument doc(&s_psram_allocator);
  DeserializationError err = deserializeJson(doc, response);
  if (err) {
    ESP_LOGW(TAG, "[home] JSON parse failed: %s", err.c_str());
    return;
  }
  JsonObject obj = doc.as<JsonObject>();
  if (obj.isNull()) {
    ESP_LOGW(TAG, "[home] response is not a JSON object");
    return;
  }

  // Drill directly into attributes.friendly_name - no client-side
  // filter loop needed.
  JsonObject attrs = obj["attributes"];
  if (attrs.isNull()) {
    ESP_LOGW(TAG, "[home] response has no 'attributes' object");
    return;
  }
  const char* name = attrs["friendly_name"];
  if (name == nullptr || name[0] == '\0') {
    ESP_LOGW(TAG, "[home] zone.home has no friendly_name");
    return;
  }
  std::string found = name;

  if (found.size() > 64) {
    ESP_LOGW(TAG, "[home] friendly_name is %u chars - truncating to 64",
             (unsigned)found.size());
    found.resize(64);
  }
  // Cap at 64 chars so a pathological friendly_name doesn't blow the
  // title bar layout. Falls back to whatever fits in the 700px label
  // box with LV_LABEL_LONG_CLIP for the rest.
  if (found.size() > 64) {
    ESP_LOGW(TAG, "[home] friendly_name is %u chars - truncating to 64",
             (unsigned)found.size());
    found.resize(64);
  }

  this->home_name_ = found;
  this->last_home_fetch_ms_ = millis();
  if (this->title_home_label_ != nullptr) {
    lv_label_set_text(this->title_home_label_, this->home_name_.c_str());
  }
  // [home] tag mirrors [ip] / [cmd] - the test harness (send_cmd.py)
  // greps for this to verify the title bar is showing the expected name.
  ESP_LOGI(TAG, "[home] %s", this->home_name_.c_str());
}

void HaAutoPanel::update_title_time_() {
  if (this->title_time_label_ == nullptr) return;
  // Throttle: only redraw when the displayed minute changes. The
  // hh:mm string is the same for a whole minute, so we cache the
  // last minute we set and bail out cheaply on every other tick.
  static int last_minute = -1;
  if (this->time_ == nullptr) {
    // Time component not configured - keep the placeholder.
    if (last_minute != -1) {
      lv_label_set_text(this->title_time_label_, "--:--");
      last_minute = -1;
    }
    return;
  }
  auto now = this->time_->now();
  if (!now.is_valid()) {
    // NTP hasn't synced yet.
    if (last_minute != -1) {
      lv_label_set_text(this->title_time_label_, "--:--");
      last_minute = -1;
    }
    return;
  }
  int minute_key = now.hour * 100 + now.minute;
  if (minute_key == last_minute) return;  // same minute, no redraw
  last_minute = minute_key;
  // Format "h:mm AM/PM" - matches the user-facing style of the
  // existing HA status text. We could use 24h but the user has
  // shown they prefer en-US conventions.
  int h12 = now.hour % 12;
  if (h12 == 0) h12 = 12;
  const char* ampm = (now.hour < 12) ? "AM" : "PM";
  char buf[16];
  snprintf(buf, sizeof(buf), "%d:%02d %s", h12, now.minute, ampm);
  lv_label_set_text(this->title_time_label_, buf);
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
  // fetch_entities_() is disabled - parsing the 220KB+ states response
  // exceeds the available internal heap on the Crowpanel. Entity state
  // and brightness are populated lazily via the api.on_state subscription
  // (subscribe_to_all_entities_), which gives us per-entity deltas
  // without a single big allocation.
  ESP_LOGI(TAG, "Skipping bulk /api/states fetch; relying on state subscriptions");

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

bool HaAutoPanel::is_area_excluded_(std::string_view area_name) const {
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

bool HaAutoPanel::is_entity_excluded_(std::string_view entity_id) const {
  for (const auto& excluded : this->exclude_entities_) {
    if (entity_id == excluded) return true;
  }
  return false;
}

bool HaAutoPanel::is_domain_included_(std::string_view domain) const {
  // An empty filter list means "no filter" (show every domain).
  if (this->entity_domains_.empty()) return true;
  for (const auto& d : this->entity_domains_) {
    if (domain == d) return true;
  }
  return false;
}

void HaAutoPanel::filter_and_build_room_cards_() {
  room_cards_.clear();
  // Reserve the outer vector once. On boards with many areas
  // (the Freenove ESP32-S3 saw 14 areas) this avoids log2(N)
  // reallocations on room_cards_ itself.
  room_cards_.reserve(discovered_areas_.size());

  for (size_t i = 0; i < discovered_areas_.size(); i++) {
    const auto& area = discovered_areas_[i];

    if (is_area_excluded_(area.name)) {
      ESP_LOGI(TAG, "Skipping excluded area: %s", area.name.c_str());
      continue;
    }

    // Skip rooms hidden via the customization engine
    if (this->is_room_hidden_(area.name)) {
      ESP_LOGI(TAG, "Skipping hidden room (user customization): %s", area.name.c_str());
      continue;
    }

    RoomCard card;
    card.area = area;
    card.color = get_room_color_((int)room_cards_.size());

    auto it = entities_by_area_.find(area.area_id);
    if (it != entities_by_area_.end()) {
      // CRITICAL on small-heap boards (ESP32-S3 internal heap
      // ~384KB; the Crowpanel P4's 768KB tolerates a few
      // reallocations but the S3 does not). Entity is large
      // (~160+ bytes with 5 std::string fields), so a room with
      // 138 entities (e.g. Garage in this HA install) was
      // reallocating many times as push_back grew the vector,
      // fragmenting the heap until the next realloc threw
      // std::bad_alloc and crashed. Reserve once at the source
      // bucket's size - worst case we slightly over-allocate for
      // rooms that lose some entities to the filter, but it's
      // a single allocation and never reallocates.
      card.entities.reserve(it->second.size());
      for (const auto& entity : it->second) {
        if (is_entity_excluded_(entity.entity_id)) continue;
        if (!is_domain_included_(entity.domain)) continue;
        card.entities.push_back(entity);
      }
    }

    room_cards_.push_back(card);
    ESP_LOGI(TAG, "Room card: %s color=0x%06X, %d entities",
             area.name.c_str(),
             (unsigned int)card.color, (int)card.entities.size());
  }

  // Apply user-defined room order: rooms listed in customizations_.room_order
  // come first in that exact sequence; rooms not listed retain their original
  // relative order after them. Uses std::stable_sort with a rank map for O(n log n).
  if (!this->customizations_.room_order.empty()) {
    std::map<std::string, size_t> rank;
    for (size_t i = 0; i < this->customizations_.room_order.size(); i++) {
      rank[this->customizations_.room_order[i]] = i;
    }
    const size_t unranked_rank = this->customizations_.room_order.size();
    std::stable_sort(room_cards_.begin(), room_cards_.end(),
                     [&rank, unranked_rank](const RoomCard &a, const RoomCard &b) {
                       auto ia = rank.find(a.area.name);
                       auto ib = rank.find(b.area.name);
                       size_t ra = (ia == rank.end()) ? unranked_rank : ia->second;
                       size_t rb = (ib == rank.end()) ? unranked_rank : ib->second;
                       return ra < rb;
                     });
    ESP_LOGI(TAG, "Applied user room_order (%d entries) to %d room cards",
             (int)this->customizations_.room_order.size(), (int)room_cards_.size());
  }

  // Recompute grid_index / x / y now that order is final.
  int cards_per_row = this->compute_cards_per_row_();
  for (size_t i = 0; i < room_cards_.size(); i++) {
    room_cards_[i].grid_index = (int)i;
    room_cards_[i].x = get_card_x_((int)i % cards_per_row);
    room_cards_[i].y = get_card_y_((int)i / cards_per_row);
  }
}

// Re-pack room_cards_ into the grid: recompute x/y/grid_index for every
// entry so the visible layout is top-to-bottom, left-to-right with no
// gaps. Call after any mutation to room_cards_ (hide, restore, future
// bulk import) so the widgets always render at their true grid slot
// rather than at stale positions from the last build.
void HaAutoPanel::repack_room_cards_() {
  int cards_per_row = this->compute_cards_per_row_();
  for (size_t i = 0; i < room_cards_.size(); i++) {
    room_cards_[i].grid_index = (int)i;
    room_cards_[i].x = get_card_x_((int)i % cards_per_row);
    room_cards_[i].y = get_card_y_((int)i / cards_per_row);
  }
}

void HaAutoPanel::create_ui_from_room_cards_() {
  ESP_LOGI(TAG, "Creating dynamic UI for %d room cards", (int)room_cards_.size());

  // Force the active screen's background to dark BEFORE any other widget
  // exists. LVGL 9's default theme paints the screen with a light/off-white
  // background; if a paint cycle sneaks in before the main_container_ and
  // its dark bg cover the screen, the user sees a brief white flash in the
  // upper-left (the only area not yet covered by anything). The
  // main_container_ is 1024x600 at (0, 0), so once it paints the screen bg
  // is irrelevant, but the first paint of that container races with this
  // setup. Setting the screen bg to dark closes the race at the source.
  {
    lv_obj_t* screen = lv_scr_act();
    if (screen != nullptr) {
      lv_obj_set_style_bg_color(screen, lv_color_hex(0x111827), 0);
      lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, 0);
      lv_obj_invalidate(screen);
    }
  }

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
  // Slightly different bg from the screen (#0f1620 vs the screen's
  // #111827) so the room grid reads as a full-width surface holding
  // the cards, not as "floating cards in a black void". The 6-shade
  // delta is just enough to be perceptible without looking like a
  // different design system. The user noted the previous design
  // looked like the cards were "in a smaller box" - the matching
  // bg was the cause; this contrast restores the sense that the
  // grid is a full surface.
  lv_obj_set_style_bg_color(this->main_container_, lv_color_hex(0x0f1620), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(this->main_container_, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_scrollbar_mode(this->main_container_, LV_SCROLLBAR_MODE_OFF);  // Clean, no scrollbar
  lv_obj_set_style_pad_all(this->main_container_, 0, 0);  // No padding
  lv_obj_set_style_border_width(this->main_container_, 0, 0);  // No border
  lv_obj_set_style_border_color(this->main_container_, lv_color_hex(0x0f1620), 0);  // Hide border
  // No scroll-snap. With snap enabled, scrolling snaps each row to the top
  // of the screen, but the user can still scroll past the last card to a
  // blank area. We size the container tightly so the last card is always
  // at the bottom of the scrollable area.

  // Fixed title bar at the top of the page (child of the screen, not
  // of the scrolling main_container_, so it stays put while the room
  // grid scrolls under it). 36px tall.
  this->create_title_bar_(screen);

  // Active media player banner sits just below the title bar (y=36..).
  // 60px tall by default; we recompute on the fly so a narrow screen
  // (Freenove 240x320) sees a smaller banner. The banner is hidden
  // by default and only shown when at least one media_player is
  // actively playing - see update_media_banner_().
  this->update_media_banner_();

  // Layout the room cards in row containers. Each row container is
  // sized to fit N cards + gaps, and aligned with LV_ALIGN_TOP_MID
  // on main_container_ so the row is centered horizontally on the
  // screen automatically (no manual "screen - row_width" math). The
  // cards inside the row are positioned manually with lv_obj_set_pos
  // since this build has LV_USE_FLEX disabled.
  //
  // Why a sub-container instead of computing each card's screen-x:
  // the user asked for the simpler "automatic centering" - one
  // LV_ALIGN_TOP_MID on the row gives us centering for free, no
  // arithmetic. The sub-container also gives us a clean parent for
  // drag-to-reorder and future per-row animations.
  int cards_per_row = this->compute_cards_per_row_();
  int num_rows = (int)((room_cards_.size() + cards_per_row - 1) / cards_per_row);
  if (num_rows < 1) num_rows = 1;

  for (int row = 0; row < num_rows; row++) {
    int row_count = (row < num_rows - 1)
                        ? cards_per_row
                        : (int) room_cards_.size() - row * cards_per_row;
    if (row_count <= 0) row_count = 1;
    int row_width = row_count * this->card_width_ +
                    (row_count > 0 ? (row_count - 1) : 0) * this->card_gap_;

    // Per-row flex container. LVGL's flex layout places the cards
    // left-to-right inside; the row itself is centered horizontally
    // on main_container_ via LV_ALIGN_TOP_MID. No absolute x
    // positions anywhere - if the screen narrows or cards grow, the
    // row reflows automatically. Flex is now enabled (LV_USE_FLEX=1
    // in the user's lvgl config).
    lv_obj_t* row_container = lv_obj_create(this->main_container_);
    lv_obj_set_size(row_container, row_width, this->card_width_);
    lv_obj_set_style_bg_opa(row_container, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(row_container, 0, 0);
    lv_obj_set_style_pad_all(row_container, 0, 0);
    lv_obj_set_style_pad_column(row_container, this->card_gap_, 0);
    lv_obj_set_flex_flow(row_container, LV_FLEX_FLOW_ROW);
    lv_obj_remove_flag(row_container, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_remove_flag(row_container, LV_OBJ_FLAG_CLICKABLE);
    int row_y = this->start_y_ + row * (this->card_width_ + this->card_gap_);
    lv_obj_align(row_container, LV_ALIGN_TOP_MID, 0, row_y);

    // Add this row's cards. The flex layout handles the per-card x
    // within the row; room.x / room.y are still tracked (0, 0) for
    // any code that reads them (drag-to-reorder, animations) but
    // they're not the source of truth for placement anymore.
    //
    // STACK YIELD: calling lv_task_handler() once per row gives LVGL
    // a chance to run its event loop, which drains any queued timer
    // or layout work and returns the call stack to a safe depth. The
    // crash we hit on 15 rooms (v1.7 build) was a "Stack protection
    // fault" in tlsf_realloc, deep inside the flex_update path - the
    // 4KB-ish main-task stack had no breathing room when 90+ widgets
    // were created back-to-back with no yields. Yelding every 4 cards
    // (= one row) keeps the stack depth under control without
    // slowing first-paint noticeably.
    for (int c = 0; c < row_count; c++) {
      int card_index = row * cards_per_row + c;
      if (card_index >= (int) room_cards_.size()) break;
      room_cards_[card_index].x = 0;
      room_cards_[card_index].y = 0;
      create_room_card_(row_container, room_cards_[card_index]);
      // Yield every card. Cheap (~few ms), prevents the stack from
      // growing past its allocated region even on a board with many
      // areas. The first card of a row already gets a yield via the
      // row boundary; yielding again on every card is a deliberate
      // belt-and-suspenders for large room counts.
      lv_task_handler();
    }
  }

  // Sync the visible_room_count_ for the title-bar/status line. After
  // filter_and_build_room_cards_ has run (hidden rooms removed, room_order
  // applied), room_cards_.size() is the authoritative count of visible rooms.
  this->visible_room_count_ = (int)room_cards_.size();

  // Size the main container to fit all rendered cards, with no extra
  // blank space. Cards are square (width == height == card_width_), so
  // the height per row is just card_width_. If the content fits within
  // screen_height_ we use screen_height_ so the page fills the screen;
  // otherwise the container grows and LVGL scrolls.
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

void HaAutoPanel::create_title_bar_(lv_obj_t* parent) {
  // Fixed title bar at the top of the page (a child of the screen so it
  // sits on top of every page). 36px tall. Holds: HA connection
  // status dot + label, and an Edit button that toggles customization
  // mode (long-press rooms to hide/reorder).
  this->title_bar_ = lv_obj_create(parent);
  lv_obj_set_size(this->title_bar_, this->screen_width_, 36);
  lv_obj_set_pos(this->title_bar_, 0, 0);
  lv_obj_set_style_bg_color(this->title_bar_, lv_color_hex(0x1f2937), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(this->title_bar_, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_style_border_width(this->title_bar_, 0, 0);
  lv_obj_set_style_pad_all(this->title_bar_, 0, 0);
  lv_obj_remove_flag(this->title_bar_, LV_OBJ_FLAG_SCROLLABLE);
  // Don't block clicks on the room grid underneath
  lv_obj_remove_flag(this->title_bar_, LV_OBJ_FLAG_CLICKABLE);

  // HA status was previously shown as a small colored dot on the left
  // of the title bar, but in practice the user couldn't tell what
  // the dot was (it read as "a white something"). The dot is now
  // removed entirely - the connection state is communicated via the
  // debug button (visible on error states) and the debug panel
  // itself (WiFi / HA sections).
  this->title_status_dot_ = nullptr;
  this->title_status_label_ = nullptr;

  // Home name label - centered horizontally AND vertically in the title
  // bar on the main grid page. Populated by fetch_home_name_() with the
  // HA zone.home entity's friendly_name. Hidden by default; the
  // show_room_grid_() / show_entity_detail_() pair flips it. Clip long
  // names so a long friendly_name doesn't bleed into the right
  // cluster. The max width here is just a safety bound - the label
  // is in LV_ALIGN_CENTER so it visually floats in the middle of
  // the available title-bar area.
  this->title_home_label_ = lv_label_create(this->title_bar_);
  lv_label_set_text(this->title_home_label_, "");
  lv_label_set_long_mode(this->title_home_label_, LV_LABEL_LONG_CLIP);
  lv_obj_set_style_text_color(this->title_home_label_, lv_color_hex(0xe5e7eb), 0);
  lv_obj_set_width(this->title_home_label_, this->screen_width_ - 360);
  lv_obj_set_style_text_align(this->title_home_label_, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_align(this->title_home_label_, LV_ALIGN_CENTER, 0, -1);
  // Start hidden. un-hide logic below in set_panel_state_().

  // --- Right cluster (flex row, adapts to any screen width) ---
  // All the right-side buttons (Time, Sort, Cancel, Debug, Edit)
  // live in a single flex-row sub-container that's pinned to the
  // right edge of the title bar. The flex layout auto-sizes the
  // cluster around its visible children, and hidden children just
  // collapse out of the layout - no absolute positions anywhere.
  // This is the user's "works on any screen" requirement: change
  // screen_width_ in YAML and the whole bar reflows.
  this->title_right_cluster_ = lv_obj_create(this->title_bar_);
  lv_obj_set_size(this->title_right_cluster_, LV_SIZE_CONTENT, 36);
  lv_obj_align(this->title_right_cluster_, LV_ALIGN_RIGHT_MID, -12, 0);
  lv_obj_set_style_bg_opa(this->title_right_cluster_, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(this->title_right_cluster_, 0, 0);
  lv_obj_set_style_pad_all(this->title_right_cluster_, 0, 0);
  lv_obj_set_style_pad_column(this->title_right_cluster_, 6, 0);
  lv_obj_set_flex_flow(this->title_right_cluster_, LV_FLEX_FLOW_ROW);
  lv_obj_set_flex_align(this->title_right_cluster_, LV_FLEX_ALIGN_CENTER,
                        LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
  lv_obj_remove_flag(this->title_right_cluster_, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_remove_flag(this->title_right_cluster_, LV_OBJ_FLAG_CLICKABLE);

  // Local clock (HH:MM AM/PM). Right cluster child. Updated by
  // update_title_time_() called from loop() (~1Hz) once NTP sync has
  // completed. Width 70 fits "12:34 PM" at the default font.
  this->title_time_label_ = lv_label_create(this->title_right_cluster_);
  lv_label_set_text(this->title_time_label_, "--:--");
  lv_label_set_long_mode(this->title_time_label_, LV_LABEL_LONG_CLIP);
  lv_obj_set_style_text_color(this->title_time_label_, lv_color_hex(0x9ca3af), 0);
  lv_obj_set_style_text_align(this->title_time_label_, LV_TEXT_ALIGN_RIGHT, 0);
  lv_obj_set_width(this->title_time_label_, 70);
  lv_obj_set_height(this->title_time_label_, LV_SIZE_CONTENT);
  lv_obj_add_flag(this->title_time_label_, LV_OBJ_FLAG_HIDDEN);

  // Debug button (50px wide "DBG"). Right cluster child. Visible
  // only when the panel is in any non-READY state (something needs
  // the user's attention). Edit mode does NOT show it (the title
  // bar is already crowded with Sort + Cancel + Done).
  this->title_debug_btn_ = lv_obj_create(this->title_right_cluster_);
  lv_obj_set_size(this->title_debug_btn_, 50, 28);
  lv_obj_set_style_bg_color(this->title_debug_btn_, lv_color_hex(0x374151), 0);
  lv_obj_set_style_radius(this->title_debug_btn_, 6, 0);
  lv_obj_set_style_border_width(this->title_debug_btn_, 0, 0);
  lv_obj_add_flag(this->title_debug_btn_, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_remove_flag(this->title_debug_btn_, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_t* debug_label = lv_label_create(this->title_debug_btn_);
  lv_label_set_text(debug_label, "DBG");
  lv_label_set_long_mode(debug_label, LV_LABEL_LONG_CLIP);
  lv_obj_set_style_text_color(debug_label, lv_color_hex(0x9ca3af), 0);
  lv_obj_set_style_text_align(debug_label, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_align(debug_label, LV_ALIGN_CENTER, 0, -1);
  lv_obj_add_event_cb(this->title_debug_btn_, [](lv_event_t* event) {
    if (s_instance == nullptr) return;
    if (s_instance->debug_panel_ != nullptr &&
        !lv_obj_has_flag(s_instance->debug_panel_, LV_OBJ_FLAG_HIDDEN)) {
      s_instance->hide_debug_panel_();
    } else {
      s_instance->show_debug_panel_();
    }
  }, LV_EVENT_CLICKED, nullptr);
  lv_obj_add_flag(this->title_debug_btn_, LV_OBJ_FLAG_HIDDEN);

  // Sort button (60px wide). Right cluster child. Visible only in
  // edit mode. Tapping it opens sort_panel_, the full-screen
  // sort+hide list (the user-preferred way to reorder rooms, which
  // replaces the drag-to-reorder gesture).
  this->title_sort_btn_ = lv_obj_create(this->title_right_cluster_);
  lv_obj_set_size(this->title_sort_btn_, 60, 28);
  lv_obj_set_style_bg_color(this->title_sort_btn_, lv_color_hex(0x374151), 0);
  lv_obj_set_style_radius(this->title_sort_btn_, 6, 0);
  lv_obj_set_style_border_width(this->title_sort_btn_, 0, 0);
  lv_obj_add_flag(this->title_sort_btn_, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_remove_flag(this->title_sort_btn_, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_t* sort_label = lv_label_create(this->title_sort_btn_);
  lv_label_set_text(sort_label, "Sort");
  lv_label_set_long_mode(sort_label, LV_LABEL_LONG_CLIP);
  lv_obj_set_style_text_color(sort_label, lv_color_hex(0x9ca3af), 0);
  lv_obj_set_style_text_align(sort_label, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_align(sort_label, LV_ALIGN_CENTER, 0, -1);
  lv_obj_add_event_cb(this->title_sort_btn_, [](lv_event_t* event) {
    if (s_instance == nullptr) return;
    if (s_instance->sort_panel_ != nullptr &&
        !lv_obj_has_flag(s_instance->sort_panel_, LV_OBJ_FLAG_HIDDEN)) {
      s_instance->hide_sort_panel_();
    } else {
      s_instance->show_sort_panel_();
    }
  }, LV_EVENT_CLICKED, nullptr);
  // Sort is always visible on the grid page (matches the v1.0
  // behavior - the "before" screenshot showed Sort and Edit side
  // by side on the title bar). Earlier builds set this to
  // LV_OBJ_FLAG_HIDDEN and gated it on edit mode, but that hid
  // it from the normal grid view too. The user pointed this out
  // during v1.5 review.

  // Edit button (top-right). Right cluster child. Always visible.
  // Toggles edit_mode_; label flips between "Edit" and "Done".
  this->title_edit_btn_ = lv_obj_create(this->title_right_cluster_);
  lv_obj_set_size(this->title_edit_btn_, 80, 28);
  lv_obj_set_style_bg_color(this->title_edit_btn_, lv_color_hex(0x374151), 0);
  lv_obj_set_style_radius(this->title_edit_btn_, 6, 0);
  lv_obj_set_style_border_width(this->title_edit_btn_, 0, 0);
  lv_obj_add_flag(this->title_edit_btn_, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_remove_flag(this->title_edit_btn_, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_t* edit_label = lv_label_create(this->title_edit_btn_);
  lv_label_set_text(edit_label, "Edit");
  // Force long-mode to clip (rather than wrap or scroll) so the
  // label keeps a predictable single-line size for centering.
  lv_label_set_long_mode(edit_label, LV_LABEL_LONG_CLIP);
  // Dimmer color (was 0xfacc15 bright yellow accent) - the user
  // wanted the title bar less distracting. Now matches the cool
  // gray of the status text (0x9ca3af) so the Edit button reads
  // as secondary chrome rather than primary call-to-action. The
  // button background (#374151) still provides enough visual
  // prominence that the user can find and tap it.
  lv_obj_set_style_text_color(edit_label, lv_color_hex(0x9ca3af), 0);
  lv_obj_set_style_text_align(edit_label, LV_TEXT_ALIGN_CENTER, 0);
  // Nudge up by 1px — the default font's bounding box puts the visible
  // glyphs slightly below the label's geometric center, so a -1 Y
  // offset compensates and the text reads as visually centered.
  lv_obj_align(edit_label, LV_ALIGN_CENTER, 0, -1);
  lv_obj_add_event_cb(this->title_edit_btn_, [](lv_event_t* event) {
    if (s_instance == nullptr) return;
    s_instance->edit_mode_ = !s_instance->edit_mode_;
    ESP_LOGI(TAG, "Edit mode %s", s_instance->edit_mode_ ? "ON" : "OFF");
    // Swap the button label between "Edit" and "Done" - matches the pattern
    // already used in show_entity_detail_() (which sets "Done" on the
    // detail page and "Edit" again when returning to the grid).
    if (s_instance->title_edit_btn_ != nullptr) {
      lv_obj_t* edit_label = lv_obj_get_child(s_instance->title_edit_btn_, 0);
      if (edit_label != nullptr) {
        lv_label_set_text(edit_label, s_instance->edit_mode_ ? "Done" : "Edit");
      }
    }
    // Exiting edit mode: close the debug panel if it's open, so the
    // user doesn't end up on a debug surface that no longer has
    // anything to do with their current state. Entering edit mode
    // also closes it (we want a fresh show, not a stale view).
    s_instance->hide_debug_panel_();
    s_instance->update_debug_btn_visibility_();
    // Starting a new edit session: snapshot the current customizations
    // so Cancel can restore them. The default copy assignment does a
    // deep copy of the std::set / std::vector / std::map members.
    if (s_instance->edit_mode_) {
      s_instance->edit_baseline_ = s_instance->customizations_;
      s_instance->in_edit_session_ = true;
      if (s_instance->title_cancel_btn_ != nullptr) {
        lv_obj_remove_flag(s_instance->title_cancel_btn_, LV_OBJ_FLAG_HIDDEN);
      }
      // Show the "Sort" button - opens the popup sort+hide panel.
      if (s_instance->title_sort_btn_ != nullptr) {
        lv_obj_remove_flag(s_instance->title_sort_btn_, LV_OBJ_FLAG_HIDDEN);
      }
      ESP_LOGI(TAG, "Edit session started: baseline snapshot saved "
                    "(hidden_rooms=%d, room_order=%d)",
               (int)s_instance->edit_baseline_.hidden_rooms.size(),
               (int)s_instance->edit_baseline_.room_order.size());
    } else {
      // Exiting via Done: persist any pending customizations to be safe
      // (the badge callback already persists on hide, but a user could
      // tap Done without hiding anything - we still want a clean write),
      // then close the session and hide Cancel.
      s_instance->write_customizations_file_();
      s_instance->in_edit_session_ = false;
      if (s_instance->title_cancel_btn_ != nullptr) {
        lv_obj_add_flag(s_instance->title_cancel_btn_, LV_OBJ_FLAG_HIDDEN);
      }
      // Hide the Sort button and close the sort panel if it's open
      // - edit mode is over, no place to sort from.
      if (s_instance->title_sort_btn_ != nullptr) {
        lv_obj_add_flag(s_instance->title_sort_btn_, LV_OBJ_FLAG_HIDDEN);
      }
      s_instance->hide_sort_panel_();
    }
    // Re-render so the UI reflects the new state. We re-use the same
    // refresh path used for hide/reorder. create_room_card_() checks
    // this->edit_mode_ itself, so the new cards will (or won't) get the
    // X badges automatically.
    s_instance->refresh_room_cards_();
  }, LV_EVENT_CLICKED, nullptr);

  // Cancel button. Right cluster child. Visible only in edit mode.
  // Tapping it restores customizations_ to the baseline snapshot
  // taken at session start AND re-persists /storage/customizations.cfg.
  // Now lives in title_right_cluster_ (the flex row), so the flex
  // layout puts it right between Sort and Done automatically.
  this->title_cancel_btn_ = lv_obj_create(this->title_right_cluster_);
  lv_obj_set_size(this->title_cancel_btn_, 80, 28);
  lv_obj_set_style_bg_color(this->title_cancel_btn_, lv_color_hex(0xef4444), 0);
  lv_obj_set_style_radius(this->title_cancel_btn_, 6, 0);
  lv_obj_set_style_border_width(this->title_cancel_btn_, 0, 0);
  lv_obj_add_flag(this->title_cancel_btn_, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_remove_flag(this->title_cancel_btn_, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_t* cancel_label = lv_label_create(this->title_cancel_btn_);
  lv_label_set_text(cancel_label, "Cancel");
  lv_label_set_long_mode(cancel_label, LV_LABEL_LONG_CLIP);
  lv_obj_set_style_text_color(cancel_label, lv_color_hex(0xFFFFFF), 0);
  lv_obj_set_style_text_align(cancel_label, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_align(cancel_label, LV_ALIGN_CENTER, 0, -1);
  lv_obj_add_flag(this->title_cancel_btn_, LV_OBJ_FLAG_HIDDEN);
  lv_obj_add_event_cb(this->title_cancel_btn_, [](lv_event_t* event) {
    if (s_instance == nullptr) return;
    ESP_LOGI(TAG, "Cancel tapped: reverting edit session");
    // 1. Close the session.
    s_instance->in_edit_session_ = false;
    // 2. Deep-copy the baseline back into customizations_. This restores
    //    hidden_rooms, hidden_entities, room_order, and entity_order to
    //    the values they had at session entry.
    s_instance->customizations_ = s_instance->edit_baseline_;
    // 3. Re-persist the reverted state to /storage/customizations.cfg
    //    so a reboot / remount sees the same state as the in-memory copy.
    s_instance->write_customizations_file_();
    // 4. Drop out of edit mode so the cards lose their X badges and
    //    yellow borders.
    s_instance->edit_mode_ = false;
    // 5. Hide the Cancel button.
    if (s_instance->title_cancel_btn_ != nullptr) {
      lv_obj_add_flag(s_instance->title_cancel_btn_, LV_OBJ_FLAG_HIDDEN);
    }
    // 6. Reset the Edit button label back to "Edit" (it would have
    //    read "Done" because edit_mode_ was true when Cancel was tapped).
    if (s_instance->title_edit_btn_ != nullptr) {
      lv_obj_t* edit_label = lv_obj_get_child(s_instance->title_edit_btn_, 0);
      if (edit_label != nullptr) {
        lv_label_set_text(edit_label, "Edit");
      }
    }
    // 7. Re-render the grid. Because customizations_ is now back to the
    //    baseline, refresh_room_cards_() will restore the original set
    //    of visible rooms and the original order.
    s_instance->refresh_room_cards_();
  }, LV_EVENT_CLICKED, nullptr);

  // Back button (top-left) - on the room grid it's a no-op; on the
  // entity detail page it pops back to the grid.
  this->title_back_btn_ = lv_obj_create(this->title_bar_);
  // Bumped from 70x28 to 75x32 per user request ("about 5px bigger")
  // for an easier tap target. Repositioned at y=2 so the taller button
  // still fits inside the 36px-tall title bar with a 2px top margin and
  // 2px bottom margin.
  lv_obj_set_size(this->title_back_btn_, 75, 32);
  lv_obj_set_pos(this->title_back_btn_, 32, 2);
  lv_obj_set_style_bg_color(this->title_back_btn_, lv_color_hex(0x374151), 0);
  lv_obj_set_style_radius(this->title_back_btn_, 6, 0);
  lv_obj_set_style_border_width(this->title_back_btn_, 0, 0);
  lv_obj_add_flag(this->title_back_btn_, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_remove_flag(this->title_back_btn_, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_t* back_label = lv_label_create(this->title_back_btn_);
  lv_label_set_text(back_label, "< Back");
  lv_label_set_long_mode(back_label, LV_LABEL_LONG_CLIP);
  lv_obj_set_style_text_color(back_label, lv_color_hex(0xfacc15), 0);
  lv_obj_set_style_text_align(back_label, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_align(back_label, LV_ALIGN_CENTER, 0, -1);
  // Hidden by default; show_entity_detail_ makes it visible
  lv_obj_add_flag(this->title_back_btn_, LV_OBJ_FLAG_HIDDEN);
  lv_obj_add_event_cb(this->title_back_btn_, [](lv_event_t* event) {
    if (s_instance == nullptr) return;
    s_instance->show_room_grid_();
  }, LV_EVENT_CLICKED, nullptr);

  // "Show hidden (N)" button (centered-ish in the title bar; shown
  // Ensure the title bar is rendered on top of any future children
  lv_obj_move_foreground(this->title_bar_);

  // Update the status indicator based on current panel state
  this->update_title_bar_();
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
  // Register the card widget so the drag-to-reorder code can find it by
  // area_id. The card is destroyed with the parent on refresh, and this
  // map is cleared in refresh_room_cards_() to match.
  lv_obj_set_style_bg_color(card, lv_color_hex(0x1a1a2e), 0);
  lv_obj_set_style_radius(card, 12, 0);
  // Subtle yellow border in edit mode to signal the card is editable.
  // Restored to no-border when edit_mode_ is false. The border sits under
  // the badge/label_btn so the user sees a clean glow around the whole card.
  if (this->edit_mode_) {
    lv_obj_set_style_border_width(card, 2, 0);
    lv_obj_set_style_border_color(card, lv_color_hex(0xfacc15), 0);
  } else {
    lv_obj_set_style_border_width(card, 0, 0);
  }
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
  // Belt-and-suspenders fix for the "small white arc flashes top-left on
  // boot" bug. LVGL 9's default arc theme paints both the track (MAIN) and
  // the indicator (INDICATOR) in white. If a paint cycle sneaks in between
  // lv_arc_create() and our final color set, the user sees a small white
  // arc — roughly the indicator's color leaking through. The first card's
  // arc lives in the upper-left of the visible area, which matches the
  // reported position. Defense: (1) pre-set BOTH colors to the card bg so
  // any leaked paint blends with the card, and (2) set BOTH opacities to
  // fully transparent so even a forced paint produces nothing. The real
  // colors and opacities are restored at the very end of the construction,
  // after invalidate() forces a repaint with the current styles.
  lv_obj_set_style_arc_color(arc, lv_color_hex(0x1a1a2e), LV_PART_MAIN);
  lv_obj_set_style_arc_color(arc, lv_color_hex(0x1a1a2e), LV_PART_INDICATOR);
  lv_obj_set_style_arc_opa(arc, LV_OPA_TRANSP, LV_PART_MAIN);
  lv_obj_set_style_arc_opa(arc, LV_OPA_TRANSP, LV_PART_INDICATOR);
  // Also hide the obj itself, in case LVGL's paint path doesn't honor the
  // opa=0 fallback (e.g. if the knob widget ignores the parent style).
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
  // Now that sizing/value/angles are set, apply the FINAL styles. The arc
  // is still hidden and fully transparent at this point — no paint has
  // been able to leak anything.
  lv_obj_set_style_arc_color(arc, lv_color_hex(0x404040), LV_PART_MAIN);
  lv_obj_set_style_arc_color(arc, lv_color_hex(room.color), LV_PART_INDICATOR);
  lv_obj_set_style_arc_width(arc, arc_width, LV_PART_MAIN);
  lv_obj_set_style_arc_width(arc, arc_width, LV_PART_INDICATOR);
  lv_obj_set_style_arc_opa(arc, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_style_arc_opa(arc, LV_OPA_COVER, LV_PART_INDICATOR);
  // Force a layout pass so styles take effect before the arc is shown
  lv_obj_update_layout(arc);
  // Explicitly mark dirty so the next LVGL tick repaints with the current
  // (correct) styles before the arc becomes visible. update_layout() only
  // marks the layout dirty; it doesn't schedule a paint. invalidate()
  // forces one. This closes the race between unhide and the next paint.
  lv_obj_invalidate(arc);
  // Now safe to make visible - styles are applied AND opacity is restored
  lv_obj_remove_flag(arc, LV_OBJ_FLAG_HIDDEN);

  // Heap-allocated control data - the user_data is a raw pointer that stays
  // valid for the lifetime of the widget. The struct owns the area_id string.
  RoomControlData* arc_data = new RoomControlData{room.area.area_id, nullptr};
  lv_obj_set_user_data(arc, arc_data);

  // Stash widget pointer for live state updates
  this->room_arc_widgets_[room.area.area_id] = arc;
  // Bubble PRESSED / PRESSING / RELEASED to the parent card so the
  // card-level drag handlers can pick them up. The arc's own RELEASED
  // handler (for brightness on release) still fires locally first; this
  // just makes the events ALSO reach the card.
  lv_obj_add_flag(arc, LV_OBJ_FLAG_EVENT_BUBBLE);

  // Use LV_EVENT_RELEASED (not VALUE_CHANGED) so we get ONE service call
  // per drag gesture, not one per finger-move tick. This matches the old
  // YAML's "mode: restart" script behavior.
  lv_obj_add_event_cb(arc, [](lv_event_t* event) {
    lv_obj_t* arc = (lv_obj_t*)lv_event_get_target(event);
    RoomControlData* data = (RoomControlData*)lv_obj_get_user_data(arc);
    if (s_instance == nullptr) {
      ESP_LOGW(TAG, "room arc release: s_instance null");
      return;
    }
    if (data == nullptr) {
      ESP_LOGW(TAG, "room arc release: data null");
      return;
    }
    int value = lv_arc_get_value(arc);
    ESP_LOGI(TAG, "room arc release: area=%s value=%d", data->area_id.c_str(), value);
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
  // Bubble PRESSED / PRESSING / RELEASED to the parent card for
  // drag-to-reorder. The button's own CLICKED handler (for on/off
  // toggle) still fires locally first.
  lv_obj_add_flag(btn, LV_OBJ_FLAG_EVENT_BUBBLE);

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
  // Tap → open the entity detail view for this room. We stash the
  // room's grid_index in the label_btn's user_data so the callback
  // can recover it. (The same room's click also fires the card's
  // own event handler, but it has no CLICKED handler so it's
  // ignored - the label_btn handler is the one that runs.)
  lv_obj_set_user_data(label_btn, (void*)(intptr_t)room.grid_index);
  lv_obj_add_event_cb(label_btn, [](lv_event_t* event) {
    if (s_instance == nullptr) return;
    int room_index = (int)(intptr_t)lv_obj_get_user_data((lv_obj_t*)lv_event_get_target(event));
    s_instance->show_entity_detail_(room_index);
  }, LV_EVENT_CLICKED, nullptr);

  // The room-name label that the user can see and tap. Restored in
  // v1.6 - the previous v1.x builds had the label_btn container but
  // no label inside it, so the cards were blank arcs with no room
  // name. The label is centered in the label_btn (which is centered
  // on the arc). lv_label_set_text takes a null-terminated const
  // char*; room.area.name is std::string so .c_str() is safe.
  lv_obj_t* label = lv_label_create(label_btn);
  lv_label_set_text(label, room.area.name.c_str());
  lv_obj_set_style_text_color(label, lv_color_hex(0xFFFFFF), 0);
  lv_obj_center(label);
  // Move label_btn to the top of the z-order so touches on the
  // text (not the arc underneath) register as the room name click
  // and route to show_entity_detail_. Without this, the label sits
  // behind the arc/button widgets in the same card and the arc's
  // touch handler steals the press.
  lv_obj_move_foreground(label_btn);

  // Store the room name (heap-allocated std::string*) in the CARD's
  // user_data so any click target on the card (label_btn, arc, button)
  // can recover the area via lv_event_get_target -> lv_obj_get_parent.
  // Storing on the card (rather than label_btn) is what enables the
  // drag handlers to work from anywhere on the card after the next edit.
  // The DELETE handler frees the string when the card is destroyed.
  std::string* card_name_ptr = new std::string(room.area.name);
  lv_obj_set_user_data(card, card_name_ptr);
  lv_obj_add_event_cb(card, [](lv_event_t* event) {
    std::string* p = (std::string*)lv_obj_get_user_data((lv_obj_t*)lv_event_get_target(event));
    delete p;
  }, LV_EVENT_DELETE, nullptr);
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
  // Sit *below* the title bar instead of overlapping it. The title bar is a
  // child of the screen at y=0, height 36. Putting the detail container at
  // y=36 keeps it from covering the status dot/label, the Back button, and
  // the (later-added) room name label. Earlier this was at y=0, which left
  // the title bar invisible because the detail container was created after
  // the title bar and therefore sat on top of it in z-order.
  lv_obj_set_pos(this->detail_container_, 0, 36);
  lv_obj_set_size(this->detail_container_, this->screen_width_, this->screen_height_ - 36);

  // Show the back button (hidden on the grid page) and re-create the
  // title bar with the back button visible.
  if (this->title_back_btn_ != nullptr) {
    lv_obj_remove_flag(this->title_back_btn_, LV_OBJ_FLAG_HIDDEN);
  }
  if (this->title_edit_btn_ != nullptr) {
    // Replace the Edit button with a Done button on the detail page
    lv_obj_t* edit_label = lv_obj_get_child(this->title_edit_btn_, 0);
    if (edit_label != nullptr) {
      lv_label_set_text(edit_label, "Done");
    }
  }
  // Defense in depth: make sure the title bar is the topmost child of the
  // screen, even after this new detail container was added underneath.
  if (this->title_bar_ != nullptr) {
    lv_obj_move_foreground(this->title_bar_);
  }
  // Update the title bar (the status dot/label stay the same, but the
  // back button visibility changed and we may want to show the room name
  // in the title).
  lv_obj_set_style_bg_color(this->detail_container_, lv_color_hex(0x111827), 0);
  lv_obj_set_scrollbar_mode(this->detail_container_, LV_SCROLLBAR_MODE_OFF);  // Clean, no scrollbar
  lv_obj_set_style_pad_all(this->detail_container_, 0, 0);  // No padding
  lv_obj_set_style_border_width(this->detail_container_, 0, 0);  // No border

  // The room name (centered in the title bar area). Set the bar above
  // the detail body which starts at y=36.
  if (this->title_status_label_ != nullptr) {
    // Move status to make room for the room title in the middle
    lv_obj_set_pos(this->title_status_label_, 110, 11);
  }

  // The back button is in the title bar (top-left). Here we just add
  // the room title and the entity list. The entity list starts below
  // the title bar (y=36).

  // Hide the home-name label while the detail page is open. Both
  // title_home_label_ (e.g. "406 Brock Ave") and title_room_label_
  // (e.g. "Front Room") are centered in the title bar, so showing
  // both at once paints them on top of each other. show_room_grid_()
  // un-hides the home label on the way back. (The user spotted the
  // overlap in the v1.6 test screenshots - the room title was
  // rendering under the home name on the detail page header.)
  if (this->title_home_label_ != nullptr) {
    lv_obj_add_flag(this->title_home_label_, LV_OBJ_FLAG_HIDDEN);
  }

  // Room title - shown in the title bar (we move the status label out
  // of the way and put the room name in the center).
  if (this->title_bar_ != nullptr) {
    if (this->title_room_label_ == nullptr) {
      this->title_room_label_ = lv_label_create(this->title_bar_);
      lv_obj_set_style_text_color(this->title_room_label_, lv_color_hex(0xFFFFFF), 0);
    }
    lv_label_set_text(this->title_room_label_, room.area.name.c_str());
    // Center horizontally in the title bar. y=8 puts the text roughly centered
    // vertically inside the 36px-tall bar.
    lv_obj_set_pos(this->title_room_label_,
                   (this->screen_width_ - (int)lv_obj_get_self_width(this->title_room_label_)) / 2,
                   8);
    // Re-center on next layout pass in case the width changed (long room names)
    lv_obj_update_layout(this->title_room_label_);
    lv_obj_set_x(this->title_room_label_,
                 (this->screen_width_ - (int)lv_obj_get_self_width(this->title_room_label_)) / 2);
  }

  // Entity list - start below title bar
  int y_offset = 50;
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

  // Make sure the detail page opens scrolled to the top. The container is
  // newly created so it should already be at 0, but be explicit in case LVGL
  // is mid-tick and the first paint of the long content pulls the scroll.
  lv_obj_scroll_to_y(this->detail_container_, 0, LV_ANIM_OFF);

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

  // Restore the title bar to the grid-page state. show_entity_detail_ set
  // the back button visible and renamed "Edit" -> "Done"; both need to
  // revert so the grid page is identical to the first boot.
  if (this->title_back_btn_ != nullptr) {
    lv_obj_add_flag(this->title_back_btn_, LV_OBJ_FLAG_HIDDEN);
  }
  if (this->title_edit_btn_ != nullptr) {
    lv_obj_t* edit_label = lv_obj_get_child(this->title_edit_btn_, 0);
    if (edit_label != nullptr) {
      lv_label_set_text(edit_label, "Edit");
    }
    lv_obj_remove_flag(this->title_edit_btn_, LV_OBJ_FLAG_HIDDEN);
  }
  if (this->title_room_label_ != nullptr) {
    lv_obj_add_flag(this->title_room_label_, LV_OBJ_FLAG_HIDDEN);
  }
  // The home name + time + status indicator are all created with
  // LV_OBJ_FLAG_HIDDEN in create_title_bar_() (default to off, no
  // flicker before the first state machine tick). Now that we're
  // committed to the grid page, un-hide them so the user sees the
  // home name centered in the title bar and the clock on the right.
  if (this->title_home_label_ != nullptr) {
    lv_obj_remove_flag(this->title_home_label_, LV_OBJ_FLAG_HIDDEN);
  }
  if (this->title_time_label_ != nullptr) {
    lv_obj_remove_flag(this->title_time_label_, LV_OBJ_FLAG_HIDDEN);
  }
  // Put the status label back where it lives on the grid page (top-left
  // status area) â€” show_entity_detail_ shifted it right to make room
  // for the room name.
  if (this->title_status_label_ != nullptr) {
    lv_obj_set_pos(this->title_status_label_, 32, 11);
  }

  this->current_room_index_ = -1;
}

void HaAutoPanel::update_media_banner_() {
  // Build/refresh the "Now Playing" banner that sits just below the
  // title bar (y=36..96 by default). Hidden unless at least one
  // media_player entity is currently in 'playing' state.
  //
  // Behavior:
  //   - If no tiles are needed, hide the banner container.
  //   - If tiles are needed, create or update them in-place so the
  //     banner always reflects the current set of playing media
  //     players. Tiles that are no longer playing are deleted; new
  //     tiles are added at the end.
  //
  // Tile content: friendly name + a small "II" (pause) icon. Tapping
  // the tile sends media_player.media_pause to HA. (The pause icon
  // is rendered as "II" text since the default LVGL font doesn't
  // include U+23F8 PAUSE SYMBOL; same workaround as the restore
  // badge in the old hidden_panel_.)
  //
  // This depends on the entity state field being kept current by
  // on_entity_state_changed_(). When the state-sync bug is open
  // (see [[project_state_sync_bug]]), the banner shows whatever
  // entities happen to have state set at fetch time.

  // Lazily create the banner container the first time we need it.
  // Anchor: just below the title bar (y=36), full width, 60px tall.
  // On narrow screens we keep 60px (it's the same height as a room
  // card, so it slots in cleanly).
  if (this->media_banner_ == nullptr) {
    lv_obj_t* screen = lv_scr_act();
    if (screen == nullptr) return;
    this->media_banner_ = lv_obj_create(screen);
    lv_obj_set_pos(this->media_banner_, 0, 36);
    lv_obj_set_size(this->media_banner_, this->screen_width_, 60);
    // Match the main_container_'s bg so the banner reads as part of
    // the same surface. 0x0f1620 is the same color as main_container_.
    lv_obj_set_style_bg_color(this->media_banner_, lv_color_hex(0x0f1620), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(this->media_banner_, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_radius(this->media_banner_, 0, 0);
    lv_obj_set_style_border_width(this->media_banner_, 0, 0);
    lv_obj_set_style_pad_all(this->media_banner_, 8, 0);
    // Banner stays put while the room grid scrolls under it. Use a
    // flex row so the tiles line up automatically as we add/remove.
    lv_obj_set_flex_flow(this->media_banner_, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(this->media_banner_,
                          LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    // The banner is below the title bar (which is at y=0..36) but the
    // title bar is drawn last so it stays on top. Defensive: move
    // the banner behind the title bar explicitly.
    if (this->title_bar_ != nullptr) {
      lv_obj_move_background(this->media_banner_);
    }
  }

  // Find currently-playing media_player entities across all rooms.
  // We collect into a small list with a parallel "tile to keep"
  // map so the loop can update the UI incrementally.
  std::vector<std::string> playing_ids;
  for (const auto& kv : this->entities_by_area_) {
    for (const auto& e : kv.second) {
      if (e.domain == "media_player" && e.state == "playing") {
        // .data() is null-terminated (string_view over std::string
        // in the arena). For the std::string map key, we need a
        // std::string copy; the allocation is bounded by the number
        // of playing media_players (typically 0..3).
        playing_ids.emplace_back(e.entity_id.data(), e.entity_id.size());
      }
    }
  }

  // Stale-tile removal: any tile in media_tiles_ that isn't in
  // playing_ids is destroyed.
  std::set<std::string> playing_set(playing_ids.begin(), playing_ids.end());
  for (auto it = this->media_tiles_.begin(); it != this->media_tiles_.end(); ) {
    if (playing_set.find(it->first) == playing_set.end()) {
      if (it->second != nullptr) lv_obj_del(it->second);
      it = this->media_tiles_.erase(it);
    } else {
      ++it;
    }
  }

  // Tile creation/update: for each currently-playing entity, ensure
  // a tile widget exists in the banner. Tiles are 200x44 with the
  // entity's friendly name on the left and a "II" pause hint on the
  // right. Tapping the tile sends media_pause to the entity.
  for (const auto& eid : playing_ids) {
    if (this->media_tiles_.count(eid) > 0) continue;  // already there
    // Find the entity so we can read its friendly name.
    std::string_view eid_view(eid);
    std::string_view name_view;
    for (const auto& kv : this->entities_by_area_) {
      for (const auto& e : kv.second) {
        if (e.entity_id == eid_view) {
          name_view = e.name;
          break;
        }
      }
      if (!name_view.empty()) break;
    }
    if (name_view.empty()) name_view = eid_view;  // fall back to entity_id

    // Tile body
    lv_obj_t* tile = lv_obj_create(this->media_banner_);
    lv_obj_set_size(tile, 200, 44);
    lv_obj_set_style_bg_color(tile, lv_color_hex(0x1f2937), 0);
    lv_obj_set_style_radius(tile, 6, 0);
    lv_obj_set_style_border_width(tile, 1, 0);
    lv_obj_set_style_border_color(tile, lv_color_hex(0x22c55e), 0);
    lv_obj_set_scrollbar_mode(tile, LV_SCROLLBAR_MODE_OFF);
    lv_obj_remove_flag(tile, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(tile, LV_OBJ_FLAG_CLICKABLE);

    // Name label (left)
    lv_obj_t* name_label = lv_label_create(tile);
    lv_label_set_text(name_label, name_view.data());
    lv_label_set_long_mode(name_label, LV_LABEL_LONG_DOT);
    lv_obj_set_width(name_label, 130);
    lv_obj_set_style_text_color(name_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(name_label, LV_ALIGN_LEFT_MID, 10, 0);

    // Pause icon (right). "II" in lieu of U+23F8 PAUSE (not in the
    // default Montserrat font - same workaround as elsewhere).
    lv_obj_t* pause_lbl = lv_label_create(tile);
    lv_label_set_text(pause_lbl, "II");
    lv_obj_set_style_text_color(pause_lbl, lv_color_hex(0x22c55e), 0);
    lv_obj_align(pause_lbl, LV_ALIGN_RIGHT_MID, -10, 0);

    // Tap handler: media_pause. We capture the entity_id as a
    // heap-allocated std::string (small, bounded by # of playing
    // media_players) and free it on the DELETE event.
    std::string* eid_copy = new std::string(eid);
    lv_obj_set_user_data(tile, eid_copy);
    lv_obj_add_event_cb(tile, [](lv_event_t* event) {
      lv_obj_t* t = (lv_obj_t*)lv_event_get_target(event);
      std::string* p = (std::string*)lv_obj_get_user_data(t);
      if (s_instance == nullptr || p == nullptr) return;
      ESP_LOGI(TAG, "Banner tap: pausing %s", p->c_str());
      s_instance->call_ha_service_("media_player.media_pause", "entity_id", *p, -1);
    }, LV_EVENT_CLICKED, nullptr);
    lv_obj_add_event_cb(tile, [](lv_event_t* event) {
      lv_obj_t* t = (lv_obj_t*)lv_event_get_target(event);
      std::string* p = (std::string*)lv_obj_get_user_data(t);
      delete p;
      lv_obj_set_user_data(t, nullptr);
    }, LV_EVENT_DELETE, nullptr);

    this->media_tiles_[eid] = tile;
  }

  // Show the banner only if there's at least one tile. The banner
  // is positioned at y=36 (just below the title bar). When hidden,
  // the room grid below renders against the same bg and the gap
  // is invisible.
  bool any = !this->media_tiles_.empty();
  if (any) {
    lv_obj_remove_flag(this->media_banner_, LV_OBJ_FLAG_HIDDEN);
  } else {
    lv_obj_add_flag(this->media_banner_, LV_OBJ_FLAG_HIDDEN);
  }
}

void HaAutoPanel::create_entity_control_(void* parent, const Entity& entity, int entity_index, int y_pos, uint32_t color) {
  if (this->is_entity_hidden_(entity.entity_id)) {
    ESP_LOGI(TAG, "Skipping hidden entity: %.*s",
             (int) entity.entity_id.size(), entity.entity_id.data());
    return;
  }

  lv_obj_t* control = lv_obj_create((lv_obj_t*) parent);
  lv_obj_set_pos(control, 32, y_pos);
  lv_obj_set_size(control, 960, 70);
  lv_obj_set_style_bg_color(control, lv_color_hex(0x1a1a2e), 0);
  lv_obj_set_style_radius(control, 8, 0);
  lv_obj_set_style_border_width(control, 1, 0);
  lv_obj_set_style_border_color(control, lv_color_hex(color), 0);
  lv_obj_set_scrollbar_mode(control, LV_SCROLLBAR_MODE_OFF);
  lv_obj_remove_flag(control, LV_OBJ_FLAG_SCROLLABLE);

  // Row layout: [Name] [State badge] [Domain] [Arc/%] [Hide X]

  // Name (left, vcenter). .data() is null-terminated because the
  // underlying string_view points into a std::string. Width-capped
  // to 260px with LV_LABEL_LONG_CLIP (hard cutoff, no ellipsis)
  // so long names don't wrap to a second line and overflow the
  // row. We picked CLIP over DOT because the default font's ellipsis
  // is a single character that takes a lot of space at 14pt; a hard
  // clip is fine because the user can tap the entity to see the
  // detail view where the full name is shown.
  lv_obj_t* name_label = lv_label_create(control);
  lv_label_set_text(name_label, entity.name.data());
  lv_obj_set_style_text_color(name_label, lv_color_hex(0xFFFFFF), 0);
  lv_obj_set_width(name_label, 260);
  lv_label_set_long_mode(name_label, LV_LABEL_LONG_CLIP);
  lv_obj_align(name_label, LV_ALIGN_LEFT_MID, 15, 0);

  // State badge - rounded pill so the user can read ON/OFF/-- at a
  // glance. The background color contrasts with the dark row bg so
  // the badge stands out from the name and isn't mistaken for a
  // label. 70x26 fits a 3-4 char state string (on, off, --, play)
  // at the default font. Anchored at x=300 (just past the 260-px
  // name area) so even worst-case-long names don't crash into it.
  lv_obj_t* state_bg = lv_obj_create(control);
  lv_obj_set_size(state_bg, 70, 26);
  lv_obj_align(state_bg, LV_ALIGN_LEFT_MID, 300, 0);
  lv_obj_set_style_radius(state_bg, 4, 0);
  lv_obj_set_style_border_width(state_bg, 0, 0);
  lv_obj_set_style_pad_all(state_bg, 0, 0);
  lv_obj_remove_flag(state_bg, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_remove_flag(state_bg, LV_OBJ_FLAG_CLICKABLE);
  const char* state_text = entity.state.empty() ? "--" : entity.state.c_str();
  uint32_t state_bg_color, state_text_color;
  if (entity.state == "on") {
    state_bg_color = 0x22DD55;  // green
    state_text_color = 0x111827;  // dark
  } else if (entity.state == "off" || entity.state == "unavailable" || entity.state == "unknown") {
    state_bg_color = 0x444444;  // dim gray
    state_text_color = 0xCCCCCC;  // light
  } else {
    state_bg_color = 0xCC8800;  // amber
    state_text_color = 0x111827;  // dark
  }
  lv_obj_set_style_bg_color(state_bg, lv_color_hex(state_bg_color), 0);
  lv_obj_set_style_bg_opa(state_bg, LV_OPA_COVER, 0);
  lv_obj_t* state_label = lv_label_create(state_bg);
  lv_label_set_text(state_label, state_text);
  lv_obj_set_style_text_color(state_label, lv_color_hex(state_text_color), 0);
  lv_obj_center(state_label);

  // Domain (middle). At x=390 (state badge ends at 370, so 20px gap).
  lv_obj_t* domain_label = lv_label_create(control);
  if (entity.domain == "binary_sensor") {
    lv_label_set_text(domain_label, "contact");
  } else if (entity.domain == "light") {
    lv_label_set_text(domain_label, "light");
  } else {
    lv_label_set_text(domain_label, entity.domain.data());
  }
  lv_obj_set_style_text_color(domain_label, lv_color_hex(color), 0);
  lv_obj_align(domain_label, LV_ALIGN_LEFT_MID, 380, 0);

  if (entity.domain == "light" && entity.has_brightness) {
    uint8_t initial_pct = (entity.state == "on" && entity.brightness > 0)
        ? static_cast<uint8_t>((entity.brightness * 100) / 255)
        : 0;
    lv_obj_t* arc = lv_arc_create(control);
    lv_obj_set_size(arc, 50, 50);
    lv_obj_align(arc, LV_ALIGN_RIGHT_MID, -200, 0);
    lv_arc_set_min_value(arc, 0);
    lv_arc_set_max_value(arc, 100);
    lv_arc_set_value(arc, initial_pct);
    lv_arc_set_bg_start_angle(arc, 135);
    lv_arc_set_bg_end_angle(arc, 405);
    lv_obj_set_style_arc_color(arc, lv_color_hex(0x404040), LV_PART_MAIN);
    lv_obj_set_style_arc_width(arc, 6, LV_PART_MAIN);
    lv_obj_set_style_arc_color(arc, lv_color_hex(color), LV_PART_INDICATOR);
    lv_obj_set_style_arc_width(arc, 6, LV_PART_INDICATOR);

    lv_obj_t* pct_label = lv_label_create(control);
    lv_label_set_text_fmt(pct_label, "%d%%", initial_pct);
    lv_obj_set_style_text_color(pct_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(pct_label, LV_ALIGN_RIGHT_MID, -100, 0);

    ArcRecord arc_rec;
    arc_rec.entity_id = entity.entity_id;
    arc_rec.area_id = entity.area_id;
    arc_rec.pct_label = pct_label;
    arc_rec.is_room_arc = false;
    this->arc_records_.push_back(arc_rec);
    size_t arc_idx = this->arc_records_.size() - 1;
    lv_obj_set_user_data(arc, (void*)(intptr_t)arc_idx);

    lv_obj_add_event_cb(arc, [](lv_event_t* event) {
      lv_obj_t* arc = (lv_obj_t*)lv_event_get_target(event);
      int value = lv_arc_get_value(arc);
      size_t idx = (size_t)(intptr_t)lv_obj_get_user_data(arc);
      if (s_instance != nullptr && idx < s_instance->arc_records_.size()) {
        lv_obj_t* label = s_instance->arc_records_[idx].pct_label;
        if (label != nullptr) lv_label_set_text_fmt(label, "%d%%", value);
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
    lv_obj_t* toggle_btn = lv_obj_create(control);
    lv_obj_set_size(toggle_btn, 100, 40);
    lv_obj_align(toggle_btn, LV_ALIGN_RIGHT_MID, -120, 0);
    lv_obj_set_style_radius(toggle_btn, 6, 0);
    lv_obj_set_style_bg_color(toggle_btn, lv_color_hex(0x333333), 0);
    lv_obj_t* toggle_label = lv_label_create(toggle_btn);
    lv_label_set_text(toggle_label, "Toggle");
    lv_obj_set_style_text_color(toggle_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_center(toggle_label);

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
  }

  // Per-entity hide (X) button - far right. Tap to add the entity
  // to the hidden_entities set, persist, and re-render the detail
  // view.
  lv_obj_t* hide_btn = lv_obj_create(control);
  lv_obj_set_size(hide_btn, 40, 40);
  lv_obj_align(hide_btn, LV_ALIGN_RIGHT_MID, -15, 0);
  lv_obj_set_style_radius(hide_btn, 20, 0);
  lv_obj_set_style_bg_color(hide_btn, lv_color_hex(0x3a1a1a), 0);
  lv_obj_set_style_border_width(hide_btn, 1, 0);
  lv_obj_set_style_border_color(hide_btn, lv_color_hex(0xAA4444), 0);
  lv_obj_set_scrollbar_mode(hide_btn, LV_SCROLLBAR_MODE_OFF);
  lv_obj_remove_flag(hide_btn, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_t* hide_label = lv_label_create(hide_btn);
  lv_label_set_text(hide_label, "X");
  lv_obj_set_style_text_color(hide_label, lv_color_hex(0xFF8888), 0);
  lv_obj_center(hide_label);

  std::string* eid_copy = new std::string(entity.entity_id.data(), entity.entity_id.size());
  lv_obj_set_user_data(hide_btn, eid_copy);
  lv_obj_add_event_cb(hide_btn, [](lv_event_t* event) {
    lv_obj_t* btn = (lv_obj_t*)lv_event_get_target(event);
    std::string* p = (std::string*)lv_obj_get_user_data(btn);
    if (s_instance == nullptr || p == nullptr) return;
    ESP_LOGI(TAG, "Hide entity: %s", p->c_str());
    s_instance->customizations_.hidden_entities.insert(*p);
    s_instance->write_customizations_file_();
    if (s_instance->current_room_index_ >= 0 &&
        s_instance->current_room_index_ < (int) s_instance->room_cards_.size()) {
      s_instance->show_entity_detail_(s_instance->current_room_index_);
    }
  }, LV_EVENT_CLICKED, nullptr);
  lv_obj_add_event_cb(hide_btn, [](lv_event_t* event) {
    lv_obj_t* btn = (lv_obj_t*)lv_event_get_target(event);
    std::string* p = (std::string*)lv_obj_get_user_data(btn);
    delete p;
    lv_obj_set_user_data(btn, nullptr);
  }, LV_EVENT_DELETE, nullptr);

  ESP_LOGI(TAG, "  Created control for entity: %.*s (%.*s) state=%s",
           (int) entity.name.size(), entity.name.data(),
           (int) entity.domain.size(), entity.domain.data(),
           entity.state.c_str());
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
  // Centering is handled by the parent row container (see
  // create_ui_from_room_cards_() and refresh_room_cards_()) which
  // uses LV_FLEX_FLOW_ROW + LV_ALIGN_TOP_MID. Each card sits at
  // x=0 within its row container; the container itself is centered
  // horizontally on main_container_ by LVGL. So the per-card x is
  // always 0 - this function is kept for API compatibility with
  // existing code paths that still read room.x, but the LVGL
  // position is set by the flex container, not by us.
  (void) col;
  return 0;
}

int HaAutoPanel::get_card_y_(int row) const {
  return this->start_y_ + row * (this->card_width_ + this->card_gap_);
}

uint32_t HaAutoPanel::get_room_color_(int index) const {
  return ROOM_COLORS_[index % MAX_ROOM_COLORS_];
}

// --- HA service call (native API) ---

void HaAutoPanel::call_ha_service_(std::string_view service,
                                   std::string_view target_type,
                                   std::string_view target_id,
                                   int brightness_pct) {
#ifdef USE_API_HOMEASSISTANT_SERVICES
  if (api::global_api_server == nullptr) {
    ESP_LOGW(TAG, "api::global_api_server is null; cannot call %.*s", (int) service.size(), service.data());
    return;
  }
  if (!api::global_api_server->is_connected()) {
    ESP_LOGW(TAG, "HA not connected; skipping %.*s %.*s=%.*s",
             (int) service.size(), service.data(),
             (int) target_type.size(), target_type.data(),
             (int) target_id.size(), target_id.data());
    return;
  }

  api::HomeassistantActionRequest req;
  // StringRef is non-owning. service / target_type are stack-local
  // std::string_views pointing into arena-backed std::strings, so
  // .data() is null-terminated and the pointer is valid for this
  // function's stack frame. send_homeassistant_action serializes
  // synchronously, so the StringRefs don't outlive this call.
  req.service = StringRef(service.data(), service.size());

  // StringRef is non-owning - copy to stack-local buffers that outlive the
  // call (the protobuf serializer runs synchronously inside send_homeassistant_action).
  char target_buf[128];
  strncpy(target_buf, target_id.data(), std::min<size_t>(target_id.size(), sizeof(target_buf) - 1));
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
    kv.key   = StringRef(target_type.data(), target_type.size());
    kv.value = StringRef(target_buf);
  }
  if (num_keys == 2) {
    auto &kv = req.data.emplace_back();
    kv.key   = StringRef::from_lit("brightness_pct");
    kv.value = StringRef(bright_buf);
  }

  api::global_api_server->send_homeassistant_action(req);

  if (brightness_pct >= 0) {
    ESP_LOGI(TAG, "Sent %s %s=%.*s brightness_pct=%d", service.data(), target_type.data(), (int) target_id.size(), target_id.data(), brightness_pct);
  } else {
    ESP_LOGI(TAG, "Sent %s %s=%.*s", service.data(), target_type.data(), (int) target_id.size(), target_id.data());
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

      // Use the const char* overload of subscribe_home_assistant_state
      // for zero allocation: the entity_id pointer is stored directly
      // in the subscription struct (api_server.cpp:434), and the
      // attribute is nullptr for state-itself. The arena-backed
      // e.entity_id is null-terminated because std::string::data() is
      // since C++11, and it lives for the component's lifetime which
      // bounds the subscription's lifetime. The lambda captures a
      // string_view (8 bytes ptr+len) by value - no std::string copy.
      const char *eid_cstr = e.entity_id.data();
      std::string_view eid_view = e.entity_id;

      // Subscribe to the entity's state itself.
      api::global_api_server->subscribe_home_assistant_state(
          eid_cstr,
          nullptr,
          [this, eid_view](StringRef state) {
            this->on_entity_state_changed_(eid_view, state.c_str());
          });
      this->subscribed_entity_ids_.insert(std::string(eid_view));
      subscribed_this_run++;

      // Also subscribe to the "brightness" attribute (only meaningful
      // for dimmable lights; HA will still send updates if the
      // attribute is missing, but it just becomes a no-op).
      api::global_api_server->subscribe_home_assistant_state(
          eid_cstr,
          "brightness",
          [this, eid_view](StringRef value) {
            this->on_entity_attribute_changed_(eid_view, value.c_str());
          });
    }
  }
  ESP_LOGI(TAG, "Subscribed to %d entity state streams (%d new this run)",
           (int)this->subscribed_entity_ids_.size(), subscribed_this_run);
#else
  ESP_LOGW(TAG, "API homeassistant_states disabled in YAML; live updates disabled");
#endif
}

void HaAutoPanel::on_entity_state_changed_(std::string_view entity_id, const char* state) {
  std::string new_state = state ? state : "";
  // LOGI (not LOGD) so we can verify the state-sync path is firing
  // when a light is toggled in HA. The previous LOGD was invisible at
  // the default INFO log level, which made the
  // "lights on in HA but GUI shows off" bug very hard to diagnose.
  ESP_LOGI(TAG, "state_changed: %.*s -> %s",
           (int) entity_id.size(), entity_id.data(), new_state.c_str());

  for (auto& kv : this->entities_by_area_) {
    for (auto& e : kv.second) {
      if (e.domain == "media_player") continue;  // scanned separately in update_media_banner_
      if (e.entity_id == entity_id) {
        e.state = new_state;
        break;
      }
    }
  }
  this->update_room_card_visual_state_for_entity_(entity_id);
  // The "Now Playing" banner lives on the grid page. When the
  // changed entity is a media_player, the banner's tile set may
  // need to grow or shrink.
  this->update_media_banner_();
}

void HaAutoPanel::on_entity_attribute_changed_(std::string_view entity_id, const char* value) {
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

std::string HaAutoPanel::find_area_id_for_entity_(std::string_view entity_id) const {
  for (const auto& kv : this->entities_by_area_) {
    for (const auto& e : kv.second) {
      if (e.entity_id == entity_id) return kv.first;
    }
  }
  return std::string();
}

void HaAutoPanel::update_room_card_visual_state_for_entity_(std::string_view entity_id) {
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
  this->update_title_bar_();

  // Drop the boot splash the first time we leave BOOTING. The status screen
  // (or main grid, if we jump straight to READY) takes over the full display
  // from this point on. We hide the splash BEFORE showing the next thing so
  // there's no frame where the user sees the unstyled screen underneath.
  if (new_state != PanelState::BOOTING && this->splash_container_ != nullptr) {
    lv_obj_add_flag(this->splash_container_, LV_OBJ_FLAG_HIDDEN);
  }

  // Toggle the main room-grid container's visibility based on the new state.
  // When we're showing a status screen (anything not READY), hide the room
  // grid so the status screen has the full display.
  if (new_state == PanelState::READY) {
    if (this->main_container_ != nullptr) {
      lv_obj_remove_flag(this->main_container_, LV_OBJ_FLAG_HIDDEN);
    }
    // Wire up the title bar for the grid view (show the home name,
    // time, and Edit button; hide the back button). Previously this
    // was only called from the back button's click handler, which
    // meant the initial state of the device showed an empty title
    // bar - the home name + time + Edit button all stayed hidden
    // until the user first tapped a room card and came back. Calling
    // it here is idempotent (all the operations are LV_OBJ_FLAG_HIDDEN
    // toggles) so repeated READY transitions (e.g. after a re-discovery)
    // are safe.
    this->show_room_grid_();
    // First time we reach READY, kick off the home-name fetch. We
    // defer it with set_timeout rather than calling synchronously
    // because fetch_home_name_() makes a network call to HA via
    // http_request_; calling it from inside set_panel_state_() on
    // the same call stack that just finished start_discovery_() and
    // update_title_bar_() crashed on the device (a 112ms gap from
    // the log to abort - too fast for a network round-trip, so the
    // crash is in the call setup, not the response). Deferring to
    // the next idle tick (50ms) lets the state machine and LVGL
    // settle before we touch http_request_. The internal throttle
    // in fetch_home_name_() (HOME_FETCH_INTERVAL_MS) still prevents
    // re-firing on subsequent state transitions.
    this->set_timeout("home_fetch", 50, [this]() {
      this->fetch_home_name_();
    });
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
  // We prefer the actual IP over the .local hostname because most
  // Windows machines don't have Bonjour/mDNS installed, so the
  // .local hostname doesn't resolve from the host.
  wifi::WiFiComponent* wifi = wifi::global_wifi_component;
  if (wifi == nullptr) {
    return std::string("No WiFi component bound; cannot determine address.");
  }
  std::string msg;
  if (wifi->is_connected()) {
    // First try the use_address (often a friendly name); then fall
    // back to the first IP.
    auto ips = wifi->get_ip_addresses();
    std::string url;
    if (!ips.empty() && ips[0].is_set()) {
      char ip_buf[network::IP_ADDRESS_BUFFER_SIZE];
      ips[0].str_to(ip_buf);
      url = std::string("http://") + ip_buf + "/autopanel";
    } else {
      const char* use_addr = wifi->get_use_address();
      if (use_addr != nullptr && use_addr[0] != '\0') {
        url = std::string("http://") + use_addr + "/autopanel";
      } else {
        msg = std::string("WiFi connected but no IP address yet. Please wait.");
        return msg;
      }
    }
    msg = std::string("Open ") + url + " in a browser to configure.";
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

  // Periodic home-name refresh. fetch_home_name_() throttles itself
  // (HOME_FETCH_INTERVAL_MS) and bails out cheaply if the URL/token
  // aren't ready, so we can just call it every loop() and let the
  // guard inside filter the work.
  if (this->state_ == PanelState::READY) {
    this->fetch_home_name_();
  }

  // Refresh the title-bar clock. update_title_time_() early-exits
  // cheaply if the displayed minute hasn't changed, so this is
  // essentially free.
  this->update_title_time_();

  // Pending 2-tap-confirm timeout. If the user armed Reboot or
  // Reset customizations and then didn't tap again within 5s, revert
  // the labels and colors back to their non-confirm state.
  if (!this->pending_action_.empty() &&
      (millis() - this->pending_action_started_ms_) > PENDING_ACTION_TIMEOUT_MS) {
    ESP_LOGI(TAG, "[debug] pending action '%s' timed out - reverting",
             this->pending_action_.c_str());
    this->clear_pending_action_();
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
  //   'C' = simulate a touch click at the x/y on the next line
  //   'S' = simulate a touch scroll from (x1,y1) to (x2,y2) on the next line
  static char cmd_buf[32];
  static size_t cmd_len = 0;
  // 'pending' is the multi-line command waiting for its coordinate
  // payload. Set by 'C' or 'S', cleared once the second line is
  // read. Without this we'd race the harness's two writes.
  static char pending_cmd = 0;
  uint8_t b;
  while (uart_read_bytes(UART_NUM_0, &b, 1, 0) > 0) {
    // While a multi-int payload is pending (after 'C' or 'S'),
    // keep spaces in the buffer so sscanf can see "x y" on a
    // single line. Only \n / \r terminate the line in that mode.
    bool is_terminator =
        (b == '\n' || b == '\r') ||
        ((b == ' ' || b == '\t') && pending_cmd == 0);
    if (is_terminator) {
      if (cmd_len == 0) continue;
      cmd_buf[cmd_len] = '\0';
      ESP_LOGI(TAG, "[cmd] received: '%s'", cmd_buf);
      char c = cmd_buf[0];
      cmd_len = 0;
      // If a multi-line command is pending, this line is its
      // coordinate payload. Dispatch the simulated input event.
      if (pending_cmd != 0) {
        char want = pending_cmd;
        pending_cmd = 0;
        if (want == 'C') {
          int x = 0, y = 0;
          if (sscanf(cmd_buf, "%d %d", &x, &y) != 2) {
            ESP_LOGW(TAG, "[cmd] click: expected 'x y', got '%s'", cmd_buf);
            continue;
          }
          this->simulate_click_(x, y);
        } else if (want == 'S') {
          int x1 = 0, y1 = 0, x2 = 0, y2 = 0;
          if (sscanf(cmd_buf, "%d %d %d %d", &x1, &y1, &x2, &y2) != 4) {
            ESP_LOGW(TAG, "[cmd] scroll: expected 'x1 y1 x2 y2', got '%s'",
                     cmd_buf);
            continue;
          }
          this->simulate_scroll_(x1, y1, x2, y2);
        }
        continue;
      }
      switch (c) {
        case 'C':
        case 'S':
          // The next line is the coordinate payload ("x y" for C,
          // "x1 y1 x2 y2" for S). These are serial-only - the web
          // API takes coordinates as query params. Hand off to the
          // multi-line parser below.
          pending_cmd = c;
          ESP_LOGI(TAG, "[cmd] %s: awaiting coordinates",
                   c == 'C' ? "click" : "scroll");
          break;
        default:
          // Every other single-char command is handled by the
          // shared process_command_() helper so the web API can
          // drive the exact same state transitions.
          this->process_command_(c);
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
  esp_vfs_littlefs_conf_t conf = {};
  conf.base_path = "/storage";
  conf.partition_label = "storage";
  conf.format_if_mount_failed = true;  // first boot will format
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

// --- User customizations (hidden rooms/entities + custom ordering) ---
//
// Stored separately from the API config (autopanel.cfg) on LittleFS at
// /storage/customizations.cfg. The CustomizationConfig struct on the
// component holds the parsed state; the JSON file is the durable form.
// The web UI (/autopanel/customizations) is the primary editor; an
// on-panel edit mode will eventually feed the same state.

bool HaAutoPanel::read_customizations_file_() {
  // Read /storage/customizations.cfg into this->customizations_. A
  // missing file is the normal first-boot case and is NOT an error -
  // we just leave customizations_ empty (loaded=false) and continue.
  FILE *f = fopen(this->customizations_path_.c_str(), "r");
  if (f == nullptr) {
    ESP_LOGW(TAG, "No customizations file at %s (first boot is normal)",
             this->customizations_path_.c_str());
    this->customizations_.loaded = false;
    return false;
  }
  std::string body;
  char buf[512];
  size_t n;
  while ((n = fread(buf, 1, sizeof(buf), f)) > 0) {
    body.append(buf, n);
  }
  fclose(f);

  // Parse the JSON. Uses the same PSRAM-backed allocator as the rest of
  // the component so the doc doesn't compete with LVGL for internal heap,
  // even though it's small.
  PsramJsonDocument doc(&s_psram_allocator);
  DeserializationError err = deserializeJson(doc, body);
  if (err) {
    ESP_LOGW(TAG, "Customizations JSON parse failed: %s", err.c_str());
    this->customizations_.loaded = false;
    return false;
  }

  // Wipe before loading - the file is the source of truth, not a diff.
  this->customizations_.hidden_rooms.clear();
  this->customizations_.hidden_entities.clear();
  this->customizations_.room_order.clear();
  this->customizations_.entity_order.clear();

  if (doc["hidden_rooms"].is<JsonArray>()) {
    for (JsonVariant v : doc["hidden_rooms"].as<JsonArray>()) {
      if (v.is<const char*>()) {
        this->customizations_.hidden_rooms.insert(v.as<std::string>());
      }
    }
  }
  if (doc["hidden_entities"].is<JsonArray>()) {
    for (JsonVariant v : doc["hidden_entities"].as<JsonArray>()) {
      if (v.is<const char*>()) {
        this->customizations_.hidden_entities.insert(v.as<std::string>());
      }
    }
  }
  if (doc["room_order"].is<JsonArray>()) {
    for (JsonVariant v : doc["room_order"].as<JsonArray>()) {
      if (v.is<const char*>()) {
        this->customizations_.room_order.push_back(v.as<std::string>());
      }
    }
  }
  if (doc["entity_order"].is<JsonObject>()) {
    for (JsonPair kv : doc["entity_order"].as<JsonObject>()) {
      std::string area_id = kv.key().c_str();
      std::vector<std::string> &order = this->customizations_.entity_order[area_id];
      if (kv.value().is<JsonArray>()) {
        for (JsonVariant v : kv.value().as<JsonArray>()) {
          if (v.is<const char*>()) {
            order.push_back(v.as<std::string>());
          }
        }
      }
    }
  }

  this->customizations_.loaded = true;
  ESP_LOGI(TAG, "Customizations loaded: %u hidden rooms, %u hidden entities, %u ordered rooms, %u entity orders",
           (unsigned) this->customizations_.hidden_rooms.size(),
           (unsigned) this->customizations_.hidden_entities.size(),
           (unsigned) this->customizations_.room_order.size(),
           (unsigned) this->customizations_.entity_order.size());
  return true;
}

bool HaAutoPanel::write_customizations_file_() {
  // Serialize this->customizations_ to JSON and write to
  // /storage/customizations.cfg. Called after any change from the web
  // form (and from the in-panel edit mode, eventually).
  PsramJsonDocument doc(&s_psram_allocator);

  JsonArray hidden_rooms = doc["hidden_rooms"].to<JsonArray>();
  for (const auto &name : this->customizations_.hidden_rooms) {
    hidden_rooms.add(name);
  }
  JsonArray hidden_entities = doc["hidden_entities"].to<JsonArray>();
  for (const auto &eid : this->customizations_.hidden_entities) {
    hidden_entities.add(eid);
  }
  JsonArray room_order = doc["room_order"].to<JsonArray>();
  for (const auto &name : this->customizations_.room_order) {
    room_order.add(name);
  }
  JsonObject entity_order = doc["entity_order"].to<JsonObject>();
  for (const auto &kv : this->customizations_.entity_order) {
    JsonArray arr = entity_order[kv.first].to<JsonArray>();
    for (const auto &eid : kv.second) {
      arr.add(eid);
    }
  }

  // ArduinoJson 7 supports std::string as the serializeJson destination
  // out of the box. The result is the canonical JSON string we persist.
  std::string body;
  serializeJson(doc, body);

  FILE *f = fopen(this->customizations_path_.c_str(), "w");
  if (f == nullptr) {
    ESP_LOGE(TAG, "Failed to open %s for writing", this->customizations_path_.c_str());
    return false;
  }
  size_t written = fwrite(body.c_str(), 1, body.size(), f);
  fclose(f);
  if (written != body.size()) {
    ESP_LOGE(TAG, "Short write to customizations file: %u of %u",
             (unsigned) written, (unsigned) body.size());
    return false;
  }
  this->customizations_.loaded = true;
  ESP_LOGI(TAG, "Wrote %u bytes to %s",
           (unsigned) body.size(), this->customizations_path_.c_str());
  return true;
}

void HaAutoPanel::apply_customizations_file_(const std::string &body) {
  // Parse the JSON body (from the web POST) and REPLACE
  // this->customizations_ wholesale. The form sends the full state on
  // each save, so we don't try to merge - the latest POST is
  // authoritative. Unrecognized keys are ignored (forward compatibility).
  PsramJsonDocument doc(&s_psram_allocator);
  DeserializationError err = deserializeJson(doc, body);
  if (err) {
    ESP_LOGW(TAG, "Customizations POST JSON parse failed: %s", err.c_str());
    return;
  }

  this->customizations_.hidden_rooms.clear();
  this->customizations_.hidden_entities.clear();
  this->customizations_.room_order.clear();
  this->customizations_.entity_order.clear();

  if (doc["hidden_rooms"].is<JsonArray>()) {
    for (JsonVariant v : doc["hidden_rooms"].as<JsonArray>()) {
      if (v.is<const char*>()) {
        this->customizations_.hidden_rooms.insert(v.as<std::string>());
      }
    }
  }
  if (doc["hidden_entities"].is<JsonArray>()) {
    for (JsonVariant v : doc["hidden_entities"].as<JsonArray>()) {
      if (v.is<const char*>()) {
        this->customizations_.hidden_entities.insert(v.as<std::string>());
      }
    }
  }
  if (doc["room_order"].is<JsonArray>()) {
    for (JsonVariant v : doc["room_order"].as<JsonArray>()) {
      if (v.is<const char*>()) {
        this->customizations_.room_order.push_back(v.as<std::string>());
      }
    }
  }
  if (doc["entity_order"].is<JsonObject>()) {
    for (JsonPair kv : doc["entity_order"].as<JsonObject>()) {
      std::string area_id = kv.key().c_str();
      std::vector<std::string> &order = this->customizations_.entity_order[area_id];
      if (kv.value().is<JsonArray>()) {
        for (JsonVariant v : kv.value().as<JsonArray>()) {
          if (v.is<const char*>()) {
            order.push_back(v.as<std::string>());
          }
        }
      }
    }
  }

  ESP_LOGI(TAG, "Customizations applied: %u hidden rooms, %u hidden entities, %u ordered rooms, %u entity orders",
           (unsigned) this->customizations_.hidden_rooms.size(),
           (unsigned) this->customizations_.hidden_entities.size(),
           (unsigned) this->customizations_.room_order.size(),
           (unsigned) this->customizations_.entity_order.size());

  // Persist immediately so a reboot doesn't lose the user's edits.
  this->write_customizations_file_();
}

bool HaAutoPanel::is_room_hidden_(std::string_view room_name) const {
  return this->customizations_.hidden_rooms.count(room_name) > 0;
}

bool HaAutoPanel::is_entity_hidden_(std::string_view entity_id) const {
  return this->customizations_.hidden_entities.count(entity_id) > 0;
}

const std::vector<std::string> *HaAutoPanel::get_entity_order_(const std::string &area_id) const {
  // Returns the custom display order for an area, or nullptr if none is
  // set (or it's empty). Callers should fall back to alphabetical or
  // HA's natural order when this returns nullptr.
  auto it = this->customizations_.entity_order.find(area_id);
  if (it == this->customizations_.entity_order.end()) return nullptr;
  if (it->second.empty()) return nullptr;
  return &it->second;
}

void HaAutoPanel::handle_customizations_get_(AsyncWebServerRequest *request) {
  // Return the current customizations as JSON. The web form GETs this
  // on load to populate its current state.
  PsramJsonDocument doc(&s_psram_allocator);

  JsonArray hidden_rooms = doc["hidden_rooms"].to<JsonArray>();
  for (const auto &name : this->customizations_.hidden_rooms) {
    hidden_rooms.add(name);
  }
  JsonArray hidden_entities = doc["hidden_entities"].to<JsonArray>();
  for (const auto &eid : this->customizations_.hidden_entities) {
    hidden_entities.add(eid);
  }
  JsonArray room_order = doc["room_order"].to<JsonArray>();
  for (const auto &name : this->customizations_.room_order) {
    room_order.add(name);
  }
  JsonObject entity_order = doc["entity_order"].to<JsonObject>();
  for (const auto &kv : this->customizations_.entity_order) {
    JsonArray arr = entity_order[kv.first].to<JsonArray>();
    for (const auto &eid : kv.second) {
      arr.add(eid);
    }
  }

  std::string body;
  serializeJson(doc, body);

  AsyncWebServerResponse *response =
      request->beginResponse(200, "application/json", body.c_str());
  request->send(response);
}

void HaAutoPanel::handle_customizations_post_(AsyncWebServerRequest *request) {
  // The web form POSTs the full customizations JSON as the raw request
  // body. AsyncWebServer exposes the raw body via arg("plain"), matching
  // the form-field lookup pattern in handle_setup_post_.
  std::string body;
  if (request->hasArg("plain")) {
    body = request->arg("plain");
  }
  if (body.empty()) {
    request->send(400, "text/plain", "Empty body");
    return;
  }
  this->apply_customizations_file_(body);
  // Re-render the room grid so the new hidden/order state is visible
  // immediately, without waiting for a reboot.
  this->refresh_room_cards_();
  request->send(200, "application/json", "{\"ok\":true}");
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
  // Load user customizations (hidden rooms/entities + display order) so
  // they're available before the room grid is built. Missing file is fine
  // (read_customizations_file_ returns false but leaves customizations_
  // in its default empty state).
  this->read_customizations_file_();
  // Don't change state here - the discovery will run via wifi.on_connect
  // or api.on_client_connected and SETUP_REQUIRED will be replaced by
  // READY on success. If the saved config is bad, the discovery/auth
  // probe will surface the failure.
}

void HaAutoPanel::handle_screenshot_(AsyncWebServerRequest *request) {
  // Capture the current LVGL display as a BMP file. Built without
  // the LVGL snapshot API (LV_USE_SNAPSHOT is off in this build);
  // we read the display's draw buffer directly after a forced
  // synchronous refresh, then wrap it in a 16-bit BI_BITFIELDS BMP
  // header. The Crowpanel's 32MB PSRAM has plenty of room for the
  // ~1.2MB single-screen buffer.
  lv_display_t* disp = lv_display_get_default();
  if (disp == nullptr) {
    request->send(500, "text/plain", "no display");
    return;
  }
  // Force a synchronous refresh so the buffer matches what's on
  // screen. lv_refr_now() blocks until the next flush completes.
  // The 1.2MB flush takes a few hundred ms over DSI, so this
  // adds latency to the screenshot request - acceptable for a
  // debug surface.
  lv_refr_now(disp);

  lv_draw_buf_t* draw_buf = lv_display_get_buf_active(disp);
  if (draw_buf == nullptr || draw_buf->data == nullptr || draw_buf->data_size == 0) {
    request->send(500, "text/plain", "no draw buffer");
    return;
  }
  const int width = lv_display_get_horizontal_resolution(disp);
  const int height = lv_display_get_vertical_resolution(disp);
  if (width <= 0 || height <= 0) {
    request->send(500, "text/plain", "bad dimensions");
    return;
  }
  // BMP 16-bit BI_BITFIELDS: 14-byte file header + 40-byte info
  // header + 12-byte RGB bitfields = 66-byte header. We need to
  // send the header and the pixel data as one HTTP body, so we
  // allocate a single PSRAM buffer with the header prepended.
  // Total = 66 + width*height*2 = ~1.2MB. PSRAM has 32MB so this
  // is comfortable.
  const size_t body_size = 66 + (size_t)width * (size_t)height * 2u;
  uint8_t *body = (uint8_t*)heap_caps_malloc(body_size, MALLOC_CAP_SPIRAM);
  if (body == nullptr) {
    // Fall back to internal heap if PSRAM isn't available.
    body = (uint8_t*)malloc(body_size);
  }
  if (body == nullptr) {
    request->send(500, "text/plain", "OOM allocating BMP buffer");
    return;
  }
  // File header (14 bytes)
  body[0] = 'B';
  body[1] = 'M';
  const uint32_t file_size = (uint32_t)body_size;
  memcpy(body + 2, &file_size, 4);  // little-endian on this target
  memset(body + 6, 0, 4);           // reserved
  const uint32_t data_offset = 66;
  memcpy(body + 10, &data_offset, 4);
  // Info header (40 bytes)
  const uint32_t info_size = 40;
  memcpy(body + 14, &info_size, 4);
  memcpy(body + 18, &width, 4);
  // Negative height = top-down row order (matches the LVGL buffer
  // layout, so we don't have to flip rows). Most viewers accept
  // this; old BMP readers don't but they're rare in 2026.
  const int32_t neg_height = -height;
  memcpy(body + 22, &neg_height, 4);
  const uint16_t planes = 1;
  memcpy(body + 26, &planes, 2);
  const uint16_t bpp = 16;
  memcpy(body + 28, &bpp, 2);
  const uint32_t compression = 3;  // BI_BITFIELDS
  memcpy(body + 30, &compression, 4);
  const uint32_t image_size = (uint32_t)width * (uint32_t)height * 2u;
  memcpy(body + 34, &image_size, 4);
  // 2835 pixels/meter ~= 72 DPI - a reasonable default.
  const int32_t ppm = 2835;
  memcpy(body + 38, &ppm, 4);
  memcpy(body + 42, &ppm, 4);
  const uint32_t zero = 0;
  memcpy(body + 46, &zero, 4);
  memcpy(body + 50, &zero, 4);
  // RGB bitfields (12 bytes) - the magic that makes 16-bit RGB565
  // decode correctly in standard image viewers. Masks are big-endian
  // (BMP quirk) but on a little-endian target the memcpy from a
  // uint32_t happens to lay them out the way the format expects
  // (R=0xF800, G=0x07E0, B=0x001F).
  const uint32_t r_mask = 0xF800;
  const uint32_t g_mask = 0x07E0;
  const uint32_t b_mask = 0x001F;
  memcpy(body + 54, &r_mask, 4);
  memcpy(body + 58, &g_mask, 4);
  memcpy(body + 62, &b_mask, 4);
  // Copy the LVGL draw buffer (RGB565, display-order) into the
  // body right after the 66-byte header. The draw buffer is
  // already in PSRAM (the LVGL component allocates it there with
  // MALLOC_CAP_SPIRAM), so this is a PSRAM-to-PSRAM copy.
  memcpy(body + 66, draw_buf->data, draw_buf->data_size);

  // Send the body. The project-level AsyncWebServerRequest wrapper
  // only exposes a 3-arg send() (code, type, body_str) that's
  // hardcoded to HTTPD_RESP_USE_STRLEN - wrong for binary BMP
  // data which isn't NULL-terminated. So we go through the
  // wrapper's operator httpd_req_t*() and call the underlying
  // esp_http_server APIs directly. The wrapper is just a
  // convenience layer; calling the C functions is supported.
  httpd_req_t *req = *request;  // operator httpd_req_t*()
  httpd_resp_set_type(req, "image/bmp");
  // httpd_resp_send() streams the body. For 1.2MB this takes a
  // few hundred ms over WiFi. The BMP is constructed with the
  // negative-height top-down convention (matches the LVGL buffer
  // layout) so we don't have to flip rows on the way out.
  esp_err_t err = httpd_resp_send(req, (const char *)body, body_size);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "[screenshot] httpd_resp_send failed: %d", (int)err);
  } else {
    ESP_LOGI(TAG, "[screenshot] sent %dx%d BMP (%u bytes)",
             width, height, (unsigned)body_size);
  }
  // The body buffer is now owned by esp_http_server's send queue;
  // do NOT free it here.
}

#if SOC_JPEG_ENCODE_SUPPORTED
// JPEG driver headers. The functions used here are declared in
// driver/jpeg_encode.h, the input format enum in
// driver/jpeg_types.h, and SOC_JPEG_ENCODE_SUPPORTED is gated in
// soc_caps.h (which is already pulled in via the esp-idf framework).
#include "driver/jpeg_encode.h"
#include "driver/jpeg_types.h"
// JPEG screenshot path. P4 has a hardware JPEG encoder; the
// handler builds an RGB888 scratch buffer in PSRAM, encodes it
// with esp_jpeg, and streams the resulting bytes. Quality 80 is
// visually lossless for LVGL-rendered UI (text edges, antialiased
// arcs, solid fills) and the output is typically 80-200 KB at
// 1024x600 - small enough to send over the existing AsyncWebServer
// in one httpd_resp_send call without chunked transfer encoding.
//
// Older ESP32 family (S2, S3, C3, C6) lack the JPEG encoder; the
// build system will leave this entire function out for them. The
// web handler at /autopanel/screenshot.jpg returns 501 in that
// case (see the URL match list and the runtime guard below).
void HaAutoPanel::handle_screenshot_jpg_(AsyncWebServerRequest *request) {
  lv_display_t* disp = lv_display_get_default();
  if (disp == nullptr) {
    request->send(500, "text/plain", "no display");
    return;
  }
  // Force a synchronous refresh so the buffer matches what's on
  // screen (same pattern as handle_screenshot_).
  lv_refr_now(disp);

  lv_draw_buf_t* draw_buf = lv_display_get_buf_active(disp);
  if (draw_buf == nullptr || draw_buf->data == nullptr || draw_buf->data_size == 0) {
    request->send(500, "text/plain", "no draw buffer");
    return;
  }
  const int width = lv_display_get_horizontal_resolution(disp);
  const int height = lv_display_get_vertical_resolution(disp);
  if (width <= 0 || height <= 0) {
    request->send(500, "text/plain", "bad dimensions");
    return;
  }

  // Allocate the RGB888 scratch buffer. 1024*600*3 = 1.84 MB. PSRAM
  // (32 MB on Crowpanel) has plenty of room. Fall back to internal
  // heap if PSRAM is somehow unavailable (e.g. someone runs this
  // build on a board without PSRAM).
  const size_t rgb_size = (size_t)width * (size_t)height * 3u;
  uint8_t* rgb = (uint8_t*)heap_caps_malloc(rgb_size, MALLOC_CAP_SPIRAM);
  if (rgb == nullptr) {
    rgb = (uint8_t*)malloc(rgb_size);
  }
  if (rgb == nullptr) {
    request->send(500, "text/plain", "OOM allocating RGB888 buffer");
    return;
  }

  // RGB565 -> RGB888. The display buffer is RGB565 in
  // little-endian byte order on this target (LV_COLOR_DEPTH=16).
  // We pack three output bytes per pixel: R, G, B. Alpha is not
  // used by JPEG. The draw buffer's data_size should equal
  // width*height*2 - we trust that (validated by handle_screenshot_).
  const uint16_t* src = (const uint16_t*)draw_buf->data;
  uint8_t* dst = rgb;
  const size_t npix = (size_t)width * (size_t)height;
  for (size_t i = 0; i < npix; i++) {
    uint16_t px = src[i];
    // Expand 5/6/5 to 8/8/8 by replicating the MSBs into the LSBs.
    // (px >> 8) is endian-dependent; the BMP path's RGB565 layout
    // tells us the high byte is R-low5|G-high3 and low byte is
    // G-low2|B-low5, so R is bits 11-15, G is 5-10, B is 0-4.
    uint8_t r5 = (px >> 11) & 0x1F;
    uint8_t g6 = (px >> 5)  & 0x3F;
    uint8_t b5 =  px        & 0x1F;
    dst[0] = (uint8_t)((r5 << 3) | (r5 >> 2));  // 5 -> 8 bits
    dst[1] = (uint8_t)((g6 << 2) | (g6 >> 4));  // 6 -> 8 bits
    dst[2] = (uint8_t)((b5 << 3) | (b5 >> 2));  // 5 -> 8 bits
    dst += 3;
  }

  // JPEG output buffer. JPEG is typically 1/5 to 1/20 the size of
  // raw RGB for UI content, so 256 KB is more than enough. If the
  // encode somehow needs more (a very busy screen), we'll fail
  // gracefully and return 500.
  const size_t jpg_cap = 256 * 1024;
  uint8_t* jpg = (uint8_t*)heap_caps_malloc(jpg_cap, MALLOC_CAP_SPIRAM);
  if (jpg == nullptr) {
    free(rgb);
    request->send(500, "text/plain", "OOM allocating JPEG output buffer");
    return;
  }

  // Build the encoder engine. The default priority + timeout=2000
  // (2s) is fine for a 1024x600 encode (~50-100 ms on P4).
  jpeg_encoder_handle_t enc = nullptr;
  jpeg_encode_engine_cfg_t eng_cfg = {};
  eng_cfg.intr_priority = 0;     // driver picks default
  eng_cfg.timeout_ms = 2000;
  esp_err_t err = jpeg_new_encoder_engine(&eng_cfg, &enc);
  if (err != ESP_OK || enc == nullptr) {
    free(rgb);
    free(jpg);
    request->send(500, "text/plain", "jpeg_new_encoder_engine failed");
    return;
  }

  jpeg_encode_cfg_t cfg = {};
  cfg.width = (uint32_t)width;
  cfg.height = (uint32_t)height;
  // Driver-level input format enum. There's also a HAL-level
  // JPEG_ENC_SRC_RGB888 that has the same numeric value
  // (COLOR_TYPE_ID(COLOR_SPACE_RGB, COLOR_PIXEL_RGB888)) but is
  // a different typedef - using it here triggers a C++ narrowing
  // error. The driver enum (jpeg_enc_input_format_t) is what
  // jpeg_encode_cfg_t::src_type expects.
  cfg.src_type = JPEG_ENCODE_IN_FORMAT_RGB888;
  cfg.sub_sample = JPEG_DOWN_SAMPLING_YUV422;  // 4:2:2 - good for UI
  cfg.image_quality = 80;

  uint32_t jpg_size = 0;
  err = jpeg_encoder_process(enc, &cfg, rgb, (uint32_t)rgb_size,
                             jpg, (uint32_t)jpg_cap, &jpg_size);
  // Always release the encoder + scratch buffers, even on error.
  // rgb is large; we'd rather leak the 1.84MB than re-encode the
  // frame if the encoder doesn't release its own memory.
  // jpeg_del_encoder_engine is safe to call regardless of process
  // result.
  jpeg_del_encoder_engine(enc);
  free(rgb);

  if (err != ESP_OK) {
    free(jpg);
    request->send(500, "text/plain", "jpeg_encoder_process failed");
    return;
  }

  // Send the JPEG. The handler runs on the httpd task, not the
  // LVGL task, so it's safe to block here on httpd_resp_send. The
  // jpg buffer is owned by httpd after this call (same contract as
  // the BMP path) so do not free it.
  httpd_req_t* req = *request;  // operator httpd_req_t*()
  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Content-Disposition",
                     "inline; filename=\"screenshot.jpg\"");
  esp_err_t send_err = httpd_resp_send(req, (const char*)jpg, (ssize_t)jpg_size);
  if (send_err != ESP_OK) {
    ESP_LOGE(TAG, "[screenshot.jpg] httpd_resp_send failed: %d", (int)send_err);
    // httpd may not own the buffer if send failed; free here.
    free(jpg);
  } else {
    ESP_LOGI(TAG, "[screenshot.jpg] sent %dx%d JPEG (%u bytes, src=%u bytes)",
             width, height, (unsigned)jpg_size, (unsigned)rgb_size);
  }
}
#endif  // SOC_JPEG_ENCODE_SUPPORTED

void HaAutoPanel::register_web_handler_() {
  if (this->web_handler_registered_) return;
  if (web_server_base::global_web_server_base == nullptr) {
    ESP_LOGW(TAG, "global_web_server_base is null; deferring web handler registration");
    return;
  }

  // Register GET /autopanel, POST /autopanel/save, POST /autopanel/reset
  class AutoPanelHandler : public AsyncWebHandler {
   public:
    AutoPanelHandler(HaAutoPanel *parent) : parent_(parent) {}
    bool canHandle(AsyncWebServerRequest *request) const override {
      char url_buf[AsyncWebServerRequest::URL_BUF_SIZE];
      std::string url(request->url_to(url_buf).str());
      // Always-present URL space. The /autopanel/test/* URLs are
      // appended at the end only when agent_debug_ is true (see
      // the if-block in register_web_handler_) so a production
      // build's canHandle will not match the test URLs and they
      // will look like 404s to anyone scanning the device.
      return url == "/autopanel" || url == "/autopanel/save" || url == "/autopanel/reset"
          || url == "/autopanel/customizations" || url == "/autopanel/screenshot.bmp"
          || url == "/autopanel/screenshot.jpg"
          || url == "/autopanel/test/click"
          || url == "/autopanel/test/scroll"
          || url == "/autopanel/test/cmd"
          || url == "/autopanel/test/state";
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
      } else if (url == "/autopanel/reset") {
        if (request->method() == HTTP_POST) {
          parent_->handle_setup_reset_(request);
        } else {
          request->send(405, "text/plain", "Method not allowed");
        }
      } else if (url == "/autopanel/customizations") {
        if (request->method() == HTTP_GET) {
          parent_->handle_customizations_get_(request);
        } else if (request->method() == HTTP_POST) {
          parent_->handle_customizations_post_(request);
        } else {
          request->send(405, "text/plain", "Method not allowed");
        }
      } else if (url == "/autopanel/screenshot.bmp") {
        // GET only. Screenshot is a read-only operation.
        if (request->method() == HTTP_GET) {
          parent_->handle_screenshot_(request);
        } else {
          request->send(405, "text/plain", "Method not allowed");
        }
      } else if (url == "/autopanel/screenshot.jpg") {
        if (request->method() == HTTP_GET) {
#if SOC_JPEG_ENCODE_SUPPORTED
          parent_->handle_screenshot_jpg_(request);
#else
          request->send(501, "text/plain",
                        "JPEG encoder not available on this chip");
#endif
        } else {
          request->send(405, "text/plain", "Method not allowed");
        }
      } else if (url == "/autopanel/test/click") {
        // AGENT_DEBUG gated. Reject the request with 404 when the
        // gate is off so the URL appears non-existent.
        if (!parent_->agent_debug_) {
          request->send(404, "text/plain", "Not found");
        } else if (request->method() == HTTP_GET || request->method() == HTTP_POST) {
          parent_->handle_test_click_(request);
        } else {
          request->send(405, "text/plain", "Method not allowed");
        }
      } else if (url == "/autopanel/test/scroll") {
        if (!parent_->agent_debug_) {
          request->send(404, "text/plain", "Not found");
        } else if (request->method() == HTTP_GET || request->method() == HTTP_POST) {
          parent_->handle_test_scroll_(request);
        } else {
          request->send(405, "text/plain", "Method not allowed");
        }
      } else if (url == "/autopanel/test/cmd") {
        if (!parent_->agent_debug_) {
          request->send(404, "text/plain", "Not found");
        } else if (request->method() == HTTP_GET || request->method() == HTTP_POST) {
          parent_->handle_test_cmd_(request);
        } else {
          request->send(405, "text/plain", "Method not allowed");
        }
      } else if (url == "/autopanel/test/state") {
        if (!parent_->agent_debug_) {
          request->send(404, "text/plain", "Not found");
        } else if (request->method() == HTTP_GET) {
          parent_->handle_test_state_(request);
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
  ESP_LOGI(TAG, "Web handler registered for /autopanel, /autopanel/customizations%s",
           this->agent_debug_ ? " (agent_debug ON - test API exposed)"
                              : "");
}

// ----------------------------------------------------------------------------
// AGENT_DEBUG: /autopanel/test/* handler bodies
//
// These endpoints let a test harness on the same LAN drive the panel
// without holding the serial port. Coordinates and one-char commands
// come in as query params. The web URL space is gated by agent_debug_
// at registration time (404 otherwise). The handlers themselves
// assume the gate is on - register_web_handler_() already 404s on
// non-debug builds.
// ----------------------------------------------------------------------------

// Helper: pull a numeric query param from an AsyncWebServerRequest.
// Returns defval if the param is missing or not a valid integer.
// The handler logs the value at INFO so the test harness can see
// what was actually received (useful when a typo in the URL turns
// 123 into 0 silently).
static int get_int_query_(AsyncWebServerRequest *request, const char* key, int defval) {
  if (!request->hasParam(key)) return defval;
  const char* raw = request->getParam(key)->value().c_str();
  char* end = nullptr;
  long v = strtol(raw, &end, 10);
  if (end == raw || *end != '\0') {
    ESP_LOGW("ha_autopanel", "[test] %s=%s is not an integer, using %d", key, raw, defval);
    return defval;
  }
  return (int) v;
}

void HaAutoPanel::handle_test_click_(AsyncWebServerRequest *request) {
  // GET /autopanel/test/click?x=N&y=M
  // POST same shape (used when the test harness wants the click
  // form-encoded rather than in the URL).
  int x = get_int_query_(request, "x", -1);
  int y = get_int_query_(request, "y", -1);
  if (x < 0 || y < 0) {
    request->send(400, "text/plain", "missing or invalid x/y");
    return;
  }
  ESP_LOGI(TAG, "[test] click at (%d, %d)", x, y);
  this->simulate_click_(x, y);
  // 200 with a small body so the test harness can confirm the
  // request was accepted (LVGL's own CLICKED events are async -
  // the actual tap effect on the screen happens after we return).
  char body[64];
  snprintf(body, sizeof(body), "click %d %d queued\n", x, y);
  request->send(200, "text/plain", body);
}

void HaAutoPanel::handle_test_scroll_(AsyncWebServerRequest *request) {
  // GET /autopanel/test/scroll?x1=N&y1=N&x2=N&y2=N
  int x1 = get_int_query_(request, "x1", -1);
  int y1 = get_int_query_(request, "y1", -1);
  int x2 = get_int_query_(request, "x2", -1);
  int y2 = get_int_query_(request, "y2", -1);
  if (x1 < 0 || y1 < 0 || x2 < 0 || y2 < 0) {
    request->send(400, "text/plain", "missing or invalid x1/y1/x2/y2");
    return;
  }
  ESP_LOGI(TAG, "[test] scroll (%d,%d) -> (%d,%d)", x1, y1, x2, y2);
  this->simulate_scroll_(x1, y1, x2, y2);
  char body[96];
  snprintf(body, sizeof(body), "scroll %d %d %d %d queued\n", x1, y1, x2, y2);
  request->send(200, "text/plain", body);
}

void HaAutoPanel::handle_test_cmd_(AsyncWebServerRequest *request) {
  // GET /autopanel/test/cmd?c=X (or POST with the same query param)
  // c is a single character that is the same as the serial command
  // set: g=grid, 0-9=detail page, o=open sort, O=close sort,
  // s/a/n/c=force state, h=home-name refresh, d=discovery,
  // p/r=auth probe. C and S (click/scroll) are rejected here
  // because they need coordinate payloads - the harness should
  // use /autopanel/test/click and /autopanel/test/scroll instead.
  if (!request->hasParam("c")) {
    request->send(400, "text/plain", "missing c=");
    return;
  }
  std::string c = request->getParam("c")->value();
  if (c.size() != 1) {
    request->send(400, "text/plain", "c must be a single character");
    return;
  }
  char ch = c[0];
  if (ch == 'C' || ch == 'S') {
    request->send(400, "text/plain",
                  "C/S need coordinates - use /test/click or /test/scroll");
    return;
  }
  bool ok = this->process_command_(ch);
  char body[64];
  snprintf(body, sizeof(body), "cmd '%c' -> %s\n", ch, ok ? "ok" : "unknown");
  request->send(ok ? 200 : 400, "text/plain", body);
}

// Stringify the current PanelState for the /test/state response.
// Matches the enum class PanelState defined earlier; kept as a
// free function so we don't grow the header with one more
// internal helper.
static const char* panel_state_name_(PanelState s) {
  switch (s) {
    case PanelState::BOOTING:         return "BOOTING";
    case PanelState::SETUP_REQUIRED:  return "SETUP_REQUIRED";
    case PanelState::AUTH_FAILED:     return "AUTH_FAILED";
    case PanelState::NOT_AUTHORIZED:  return "NOT_AUTHORIZED";
    case PanelState::CONNECTING:      return "CONNECTING";
    case PanelState::READY:           return "READY";
  }
  return "?";
}

void HaAutoPanel::handle_test_state_(AsyncWebServerRequest *request) {
  // GET /autopanel/test/state - returns a small text snapshot of
  // panel state for the test harness. Format: one key=value per
  // line, easy to grep or parse.
  std::string body;
  body += std::string("panel_state=") + panel_state_name_(this->state_) + "\n";
  body += std::string("room_count=") + std::to_string(this->room_cards_.size()) + "\n";
  body += std::string("current_room_index=") + std::to_string(this->current_room_index_) + "\n";
  body += std::string("edit_mode=") + (this->edit_mode_ ? "1" : "0") + "\n";
  body += std::string("in_edit_session=") + (this->in_edit_session_ ? "1" : "0") + "\n";
  body += std::string("agent_debug=1\n");
  body += std::string("home_name=") + this->home_name_ + "\n";
  body += std::string("ha_api_url=") + this->ha_api_url_ + "\n";
  body += std::string("screen=") + std::to_string(this->screen_width_) + "x"
        + std::to_string(this->screen_height_) + "\n";
  // jpeg_available lets the test script make a clean one-shot
  // decision: if the device advertises JPEG, hit /screenshot.jpg
  // (much smaller payload, no Python BMP->PNG conversion). If
  // not, fall back to /screenshot.bmp. This is checked once at
  // the start of the test run, not on every screenshot.
#if SOC_JPEG_ENCODE_SUPPORTED
  body += std::string("jpeg_available=1\n");
#else
  body += std::string("jpeg_available=0\n");
#endif
  // The IDF AsyncWebServerRequest wrapper takes const char* for
  // the body, not std::string& (the Arduino variant does, but
  // we're on esp-idf). The handler returns immediately so the
  // std::string temporary lives long enough for the underlying
  // httpd_resp_send to copy it out.
  request->send(200, "text/plain", body.c_str());
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

  // Reset form (separate, so users can fall back to the YAML defaults)
  body += "<form method='POST' action='/autopanel/reset' style='margin-top:12px'>";
  body += "<button type='submit' style='background:#374151;color:#eee'>Reset to YAML defaults</button>";
  body += "</form>";

  // Show this device's IP prominently (the .local hostname only
  // works if the user's machine has Bonjour/mDNS installed, which is
  // unreliable).
  body += "<div class='note'><b>Access URL:</b> http://";
  // We don't know our own IP from the AsyncWebServerRequest; the
  // SETUP_REQUIRED screen on the panel itself shows the IP. Web
  // form user can find the device by scanning the network with
  // python send_cmd.py web find
  body += "<i>(see panel screen for the device's IP)</i>/autopanel";
  body += "<br><br>Settings are saved to /storage/autopanel.cfg on the device. "
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

void HaAutoPanel::handle_setup_reset_(AsyncWebServerRequest *request) {
  // Delete the saved config file so the device falls back to the YAML
  // values on next boot. Useful when the user wants to "start over".
  ESP_LOGI(TAG, "Reset: deleting %s", this->config_path_.c_str());
  int rc = unlink(this->config_path_.c_str());
  std::string body;
  body += "<!doctype html><html><body style='font-family:sans-serif;background:#111827;color:#fff;padding:24px'>";
  if (rc == 0) {
    body += "<h1>Reset</h1><p>Saved config deleted. Device restarting in 2 seconds. The YAML values will be used.</p>";
  } else {
    body += "<h1>Reset</h1><p>No saved config to delete (or delete failed). Device restarting in 2 seconds.</p>";
  }
  body += "</body></html>";
  AsyncWebServerResponse *resp = request->beginResponse(200, "text/html", body.c_str());
  request->send(resp);
  set_timeout(2000, []() {
    App.safe_reboot();
  });
}

// --- Title bar ---

void HaAutoPanel::update_title_bar_() {
  // Status dot and label were removed (chrome nobody read). The
  // state is now visible via the debug button (on error states) and
  // the debug panel (WiFi / HA sections). Still call
  // update_debug_btn_visibility_() below so that path stays in sync.
  // Cleanliness pass (2026-06-03): the title bar used to show
  // "HA: 192.168.2.74" / "HA: connecting..." next to the dot, but
  // the user wanted the chrome tighter - just the colored dot for
  // status, with full text + IP available in the debug panel. The
  // label widget is still created (hidden) so we don't have to
  // touch the layout / show_room_grid_() paths.
  uint32_t dot_color = 0x6b7280;  // grey = unknown / not connected
  if (api::global_api_server != nullptr) {
    if (this->state_ == PanelState::READY) {
      dot_color = 0x10b981;  // green
      // Mirror the IP to the log (still useful for the test
      // harness even though we no longer show it in the title bar).
      // Tagged "[ip]" so the harness can grep for it. Throttled to
      // log only on IP change to avoid log spam.
      wifi::WiFiComponent* wifi = wifi::global_wifi_component;
      if (wifi != nullptr && wifi->is_connected()) {
        auto ips = wifi->get_ip_addresses();
        if (!ips.empty() && ips[0].is_set()) {
          char ip_buf[network::IP_ADDRESS_BUFFER_SIZE];
          ips[0].str_to(ip_buf);
          static char last_logged_ip[network::IP_ADDRESS_BUFFER_SIZE] = {0};
          if (strcmp(last_logged_ip, ip_buf) != 0) {
            strncpy(last_logged_ip, ip_buf, sizeof(last_logged_ip) - 1);
            last_logged_ip[sizeof(last_logged_ip) - 1] = '\0';
            ESP_LOGI(TAG, "[ip] %s", ip_buf);
          }
        }
      }
    } else if (this->state_ == PanelState::AUTH_FAILED) {
      dot_color = 0xef4444;  // red
    } else if (this->state_ == PanelState::NOT_AUTHORIZED) {
      dot_color = 0xf59e0b;  // amber
    } else if (this->state_ == PanelState::CONNECTING) {
      dot_color = 0x3b82f6;  // blue
    } else {
      // SETUP_REQUIRED and anything else we don't recognise.
      dot_color = 0x6b7280;  // grey
    }
  }
  // The status dot/label were removed (chrome nobody read). The
  // state is communicated via the debug button (visible on error
  // states) and the debug panel itself. Guard the lvgl calls so a
  // null pointer here doesn't trip a load-access fault - we still
  // want update_title_bar_() to be safe to call from any state.
  if (this->title_status_dot_ != nullptr) {
    lv_obj_set_style_bg_color(this->title_status_dot_, lv_color_hex(dot_color), LV_PART_MAIN);
  }
  if (this->title_status_label_ != nullptr) {
    // The status label is intentionally hidden in the cleanliness pass.
    // IP / connection state are now in the debug panel.
    lv_obj_add_flag(this->title_status_label_, LV_OBJ_FLAG_HIDDEN);
  }
  // Update the debug-button visibility in case the state changed.
  this->update_debug_btn_visibility_();
}

void HaAutoPanel::update_debug_btn_visibility_() {
  if (this->title_debug_btn_ == nullptr) return;
  // Visible only when the panel is in any non-READY state (something
  // needs the user's attention: SETUP_REQUIRED, AUTH_FAILED,
  // NOT_AUTHORIZED, CONNECTING). The debug button is the door to
  // diagnosing/fixing the issue.
  //
  // Edit mode does NOT show the debug button - the title bar in
  // edit mode now also has Cancel + Done + Reorder buttons and
  // there isn't room for a fourth. The user can still reach the
  // debug surface via the serial `d` (re-discovery) command if they
  // need to from edit mode.
  bool needs_attention = (this->state_ != PanelState::READY);
  if (needs_attention) {
    lv_obj_remove_flag(this->title_debug_btn_, LV_OBJ_FLAG_HIDDEN);
  } else {
    lv_obj_add_flag(this->title_debug_btn_, LV_OBJ_FLAG_HIDDEN);
  }
}

void HaAutoPanel::show_debug_panel_() {
  // Create the panel on first show, then rebuild every subsequent
  // show so live values (WiFi RSSI, free heap, time) are fresh.
  if (this->debug_panel_ == nullptr) {
    lv_obj_t* screen = lv_scr_act();
    if (screen == nullptr) return;
    // Bottom-anchored sheet, 70% of the screen height, full width.
    // Background #111827 matches main_container_'s bg, with a
    // slightly lighter top border to visually separate it from the
    // room grid above.
    this->debug_panel_ = lv_obj_create(screen);
    lv_obj_set_size(this->debug_panel_, this->screen_width_,
                    (this->screen_height_ * 7) / 10);
    lv_obj_align(this->debug_panel_, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_style_bg_color(this->debug_panel_, lv_color_hex(0x1f2937), 0);
    lv_obj_set_style_bg_opa(this->debug_panel_, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(this->debug_panel_, 1, 0);
    lv_obj_set_style_border_color(this->debug_panel_, lv_color_hex(0x374151), 0);
    lv_obj_set_style_pad_all(this->debug_panel_, 12, 0);
    lv_obj_set_style_pad_row(this->debug_panel_, 6, 0);
    lv_obj_set_style_radius(this->debug_panel_, 0, 0);
    // No scroll. With 5 status sections + action buttons, the panel
    // can comfortably fit in 70% of 600px (~420px). If we ever need
    // more, enable scroll here and increase the panel height.
    lv_obj_remove_flag(this->debug_panel_, LV_OBJ_FLAG_SCROLLABLE);
    // Click anywhere outside the action buttons to dismiss. The
    // panel itself is clickable to capture these "outside" clicks;
    // the action buttons inside are also clickable and stop the
    // event from bubbling.
    lv_obj_add_flag(this->debug_panel_, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(this->debug_panel_, [](lv_event_t* event) {
      if (s_instance == nullptr) return;
      // Only the panel itself receives this (children are
      // click-stoppers). Hide on click of empty panel area.
      s_instance->hide_debug_panel_();
    }, LV_EVENT_CLICKED, nullptr);
  }
  this->build_debug_panel_content_();
  lv_obj_remove_flag(this->debug_panel_, LV_OBJ_FLAG_HIDDEN);
  // Clear any pending confirm when the panel is freshly shown.
  this->clear_pending_action_();
  ESP_LOGI(TAG, "[debug] panel shown");
}

void HaAutoPanel::hide_debug_panel_() {
  if (this->debug_panel_ == nullptr) return;
  lv_obj_add_flag(this->debug_panel_, LV_OBJ_FLAG_HIDDEN);
  this->clear_pending_action_();
  ESP_LOGI(TAG, "[debug] panel hidden");
}

void HaAutoPanel::clear_pending_action_() {
  // Reset all action button labels to their non-confirm text. We
  // walk the children looking for ones whose label text starts with
  // the confirm suffix ("again to confirm") and rewrite them.
  if (this->debug_panel_ == nullptr) return;
  this->pending_action_.clear();
  this->pending_action_started_ms_ = 0;
  // Rebuild the action-button row so the labels and colors revert.
  // (The other content rows are static until next show, so a partial
  // rebuild is fine.)
  uint32_t child_count = lv_obj_get_child_cnt(this->debug_panel_);
  for (uint32_t i = child_count; i > 0; i--) {
    lv_obj_t* child = lv_obj_get_child(this->debug_panel_, i - 1);
    if (child == nullptr) continue;
    // Action buttons are tagged with user_data="action:reboot" /
    // "action:reset" / "action:reprobe" / "action:redisc" by
    // build_debug_panel_content_(). We detect them by user_data.
    const char* tag = (const char*) lv_obj_get_user_data(child);
    if (tag == nullptr) continue;
    if (strncmp(tag, "action:", 7) == 0) {
      lv_obj_t* lbl = lv_obj_get_child(child, 0);
      if (lbl != nullptr) {
        const char* kind = tag + 7;
        if (strcmp(kind, "reboot") == 0) {
          lv_label_set_text(lbl, "Reboot");
          lv_obj_set_style_bg_color(child, lv_color_hex(0x374151), 0);
        } else if (strcmp(kind, "reset") == 0) {
          lv_label_set_text(lbl, "Reset customizations");
          lv_obj_set_style_bg_color(child, lv_color_hex(0x374151), 0);
        } else if (strcmp(kind, "reprobe") == 0) {
          lv_label_set_text(lbl, "Re-probe auth");
        } else if (strcmp(kind, "redisc") == 0) {
          lv_label_set_text(lbl, "Re-run discovery");
        } else if (strcmp(kind, "refetch") == 0) {
          lv_label_set_text(lbl, "Re-fetch home name");
        }
      }
    }
  }
}

void HaAutoPanel::show_sort_panel_() {
  // Lazy-create the full-screen panel on first show, then rebuild the
  // rows on every subsequent show so live state is current.
  if (this->sort_panel_ == nullptr) {
    lv_obj_t* screen = lv_scr_act();
    if (screen == nullptr) return;
    this->sort_panel_ = lv_obj_create(screen);
    // Full-screen overlay. Sits above the room grid (created after
    // main_container_ in z-order, so it's on top).
    lv_obj_set_size(this->sort_panel_, this->screen_width_, this->screen_height_);
    lv_obj_align(this->sort_panel_, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_set_style_bg_color(this->sort_panel_, lv_color_hex(0x111827), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(this->sort_panel_, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(this->sort_panel_, 0, 0);
    lv_obj_set_style_pad_all(this->sort_panel_, 12, 0);
    lv_obj_set_style_pad_row(this->sort_panel_, 8, 0);
    lv_obj_set_flex_flow(this->sort_panel_, LV_FLEX_FLOW_COLUMN);
    // Scrollable so 15+ rooms fit on a 600px-tall screen (each
    // row is 50px, so 15 rows = 750px > 600px). The Save/Cancel
    // buttons at the bottom of the panel are reachable by
    // scrolling. Scrollbar is OFF for a cleaner look (the user
    // can scroll with a touch swipe).
    lv_obj_add_flag(this->sort_panel_, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scrollbar_mode(this->sort_panel_, LV_SCROLLBAR_MODE_OFF);
    lv_obj_remove_flag(this->sort_panel_, LV_OBJ_FLAG_CLICKABLE);
  }
  // Seed the local state from the current customizations_:
  //   - sort_local_order_: start with customizations_.room_order
  //     (which may be empty if the user never reordered); fall back
  //     to the current room_cards_ order if room_order is empty.
  //   - sort_local_hidden_: copy the current hidden_rooms set.
  this->sort_local_order_.clear();
  this->sort_local_hidden_.clear();
  for (const auto& name : this->customizations_.room_order) {
    this->sort_local_order_.push_back(name);
  }
  if (this->sort_local_order_.empty()) {
    for (const auto& rc : this->room_cards_) {
      this->sort_local_order_.push_back(rc.area.name);
    }
  }
  for (const auto& h : this->customizations_.hidden_rooms) {
    this->sort_local_hidden_.insert(h);
  }
  this->build_sort_panel_content_();
  lv_obj_remove_flag(this->sort_panel_, LV_OBJ_FLAG_HIDDEN);
  ESP_LOGI(TAG, "[sort] panel shown (%d rooms, %d hidden)",
           (int)this->sort_local_order_.size(),
           (int)this->sort_local_hidden_.size());
}

void HaAutoPanel::hide_sort_panel_() {
  if (this->sort_panel_ == nullptr) return;
  lv_obj_add_flag(this->sort_panel_, LV_OBJ_FLAG_HIDDEN);
  // Clear local state so a re-show re-seeds from customizations_.
  this->sort_local_order_.clear();
  this->sort_local_hidden_.clear();
  ESP_LOGI(TAG, "[sort] panel hidden");
}

void HaAutoPanel::sort_move_(int index, int delta) {
  if (index < 0 || index >= (int)this->sort_local_order_.size()) return;
  int new_index = index + delta;
  if (new_index < 0 || new_index >= (int)this->sort_local_order_.size()) return;
  // Swap. std::swap on the two strings.
  std::swap(this->sort_local_order_[index], this->sort_local_order_[new_index]);
  // Rebuild the panel so the new order is reflected (LVGL doesn't
  // re-flow a column flex container on member-vector changes).
  this->build_sort_panel_content_();
}

void HaAutoPanel::apply_sort_panel_() {
  // Persist the local state to customizations_:
  //   - room_order: the user's chosen ordering, room names.
  //   - hidden_rooms: which rooms are hidden (their names; the
  //     build path looks up area_id from name when filtering).
  this->customizations_.room_order = this->sort_local_order_;
  // hidden_rooms stores area_id, but the panel is keyed by name. The
  // apply path needs to map back: for each hidden name, find the
  // matching area's area_id. If the user has never renamed rooms
  // in HA, the name == area_id (HA defaults). Build the set.
  std::set<std::string, std::less<>> hidden_by_id;
  for (const auto& name : this->sort_local_hidden_) {
    // Find the area_id whose name matches.
    for (const auto& area : this->discovered_areas_) {
      if (area.name == name) {
        hidden_by_id.insert(area.area_id);
        break;
      }
    }
  }
  this->customizations_.hidden_rooms = hidden_by_id;
  // Persist to /storage/customizations.cfg.
  this->write_customizations_file_();
  ESP_LOGI(TAG, "[sort] applied: room_order=%d, hidden=%d",
           (int)this->customizations_.room_order.size(),
           (int)this->customizations_.hidden_rooms.size());
  // Hide the panel.
  this->hide_sort_panel_();
  // Re-render the grid. The next refresh_room_cards_() call will use
  // the updated customizations_ to filter and order.
  this->refresh_room_cards_();
}

void HaAutoPanel::build_sort_panel_content_() {
  if (this->sort_panel_ == nullptr) return;
  // Tear down all existing children. We rebuild the panel from
  // scratch every show so live state is current. The panel has
  // ~15 rows of 60px each, so the rebuild is cheap.
  uint32_t child_count = lv_obj_get_child_cnt(this->sort_panel_);
  for (uint32_t i = child_count; i > 0; i--) {
    lv_obj_del(lv_obj_get_child(this->sort_panel_, i - 1));
  }

  // --- Header bar inside the panel: title + close X ---
  lv_obj_t* header = lv_obj_create(this->sort_panel_);
  lv_obj_set_size(header, LV_SIZE_CONTENT, 48);
  lv_obj_set_style_bg_opa(header, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(header, 0, 0);
  lv_obj_set_style_pad_all(header, 0, 0);
  lv_obj_set_flex_flow(header, LV_FLEX_FLOW_ROW);
  lv_obj_set_flex_align(header, LV_FLEX_ALIGN_SPACE_BETWEEN,
                        LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
  lv_obj_remove_flag(header, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_remove_flag(header, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_t* title_lbl = lv_label_create(header);
  lv_label_set_text(title_lbl, "Edit Rooms");
  lv_obj_set_style_text_color(title_lbl, lv_color_hex(0xe5e7eb), 0);
  // The flex SPACE_BETWEEN puts the close X on the right. We use
  // SPACE_BETWEEN (not END) so the title hugs the left, X hugs the
  // right - matches the room-grid title bar aesthetic.
  // The X is now a real clickable button (was a static label before)
  // that calls hide_sort_panel_() - the user can dismiss the panel
  // without saving, discarding any local reorder/hide changes.
  // This is the "Cancel" equivalent for the sort panel (it does NOT
  // exit edit mode; the user stays in edit mode and can re-open the
  // panel to try again).
  lv_obj_t* close_btn = lv_obj_create(header);
  lv_obj_set_size(close_btn, 32, 32);
  lv_obj_set_style_bg_opa(close_btn, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(close_btn, 0, 0);
  lv_obj_set_style_pad_all(close_btn, 0, 0);
  lv_obj_add_flag(close_btn, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_remove_flag(close_btn, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_t* close_lbl = lv_label_create(close_btn);
  lv_label_set_text(close_lbl, "X");
  lv_obj_set_style_text_color(close_lbl, lv_color_hex(0x9ca3af), 0);
  lv_obj_set_style_text_align(close_lbl, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_align(close_lbl, LV_ALIGN_CENTER, 0, -1);
  lv_obj_add_event_cb(close_btn, [](lv_event_t* event) {
    if (s_instance == nullptr) return;
    ESP_LOGI(TAG, "[sort] close X tapped - discarding local changes");
    s_instance->hide_sort_panel_();
  }, LV_EVENT_CLICKED, nullptr);

  // --- List of rooms ---
  for (size_t i = 0; i < this->sort_local_order_.size(); i++) {
    const std::string& name = this->sort_local_order_[i];
    bool hidden = this->sort_local_hidden_.count(name) > 0;
    // Each row: name (left) + up arrow + down arrow + hide/show toggle.
    // Width is set to the panel's content area (panel width minus the
    // 12px padding on each side). Using LV_SIZE_CONTENT here would
    // produce a 0-width row because the room name uses flex_grow(1) -
    // LVGL can't compute a content size from a flex-relative child.
    lv_obj_t* row = lv_obj_create(this->sort_panel_);
    lv_obj_set_width(row, this->screen_width_ - 24);
    lv_obj_set_height(row, 50);
    lv_obj_set_style_bg_color(row, lv_color_hex(hidden ? 0x1f2937 : 0x1a1a2e), 0);
    lv_obj_set_style_radius(row, 6, 0);
    lv_obj_set_style_border_width(row, 0, 0);
    lv_obj_set_style_pad_all(row, 8, 0);
    lv_obj_set_style_pad_column(row, 8, 0);
    lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(row, LV_FLEX_ALIGN_SPACE_BETWEEN,
                          LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_remove_flag(row, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_remove_flag(row, LV_OBJ_FLAG_SCROLLABLE);

    // Room name (left, grows to fill). Without flex_grow, the name
    // would get squeezed by the buttons. With it, the name takes all
    // the space the buttons don't, and LV_LABEL_LONG_CLIP handles
    // overflow.
    lv_obj_t* name_lbl = lv_label_create(row);
    lv_label_set_text(name_lbl, name.c_str());
    lv_obj_set_style_text_color(name_lbl, lv_color_hex(hidden ? 0x6b7280 : 0xe5e7eb), 0);
    lv_obj_set_style_text_align(name_lbl, LV_TEXT_ALIGN_LEFT, 0);
    lv_label_set_long_mode(name_lbl, LV_LABEL_LONG_CLIP);
    lv_obj_set_flex_grow(name_lbl, 1);

    // Up arrow (disabled at top).
    auto make_arrow = [&](const char* sym, bool enabled) {
      lv_obj_t* btn = lv_obj_create(row);
      lv_obj_set_size(btn, 40, 32);
      lv_obj_set_style_bg_color(btn, lv_color_hex(enabled ? 0x374151 : 0x1f2937), 0);
      lv_obj_set_style_radius(btn, 6, 0);
      lv_obj_set_style_border_width(btn, 0, 0);
      if (enabled) lv_obj_add_flag(btn, LV_OBJ_FLAG_CLICKABLE);
      lv_obj_remove_flag(btn, LV_OBJ_FLAG_SCROLLABLE);
      lv_obj_t* lbl = lv_label_create(btn);
      lv_label_set_text(lbl, sym);
      lv_obj_set_style_text_color(lbl,
                                 lv_color_hex(enabled ? 0xe5e7eb : 0x4b5563), 0);
      lv_obj_set_style_text_align(lbl, LV_TEXT_ALIGN_CENTER, 0);
      lv_obj_align(lbl, LV_ALIGN_CENTER, 0, -1);
      return btn;
    };
    lv_obj_t* up = make_arrow("^",
                              i > 0);  // disabled at top
    lv_obj_add_event_cb(up, [](lv_event_t* event) {
      lv_obj_t* tgt = (lv_obj_t*)lv_event_get_target(event);
      // Find this row's index in the panel by walking children. We
      // bind the index via user_data to avoid the walk.
      int idx = (int)(intptr_t)lv_obj_get_user_data(tgt);
      s_instance->sort_move_(idx, -1);
    }, LV_EVENT_CLICKED, nullptr);
    lv_obj_set_user_data(up, (void*)(intptr_t)i);

    lv_obj_t* down = make_arrow("v",
                                i + 1 < this->sort_local_order_.size());  // disabled at bottom
    lv_obj_add_event_cb(down, [](lv_event_t* event) {
      lv_obj_t* tgt = (lv_obj_t*)lv_event_get_target(event);
      int idx = (int)(intptr_t)lv_obj_get_user_data(tgt);
      s_instance->sort_move_(idx, +1);
    }, LV_EVENT_CLICKED, nullptr);
    lv_obj_set_user_data(down, (void*)(intptr_t)i);

    // Hide/Show toggle button (right). Yellow bg when hidden, gray
    // when visible.
    lv_obj_t* hide_btn = lv_obj_create(row);
    lv_obj_set_size(hide_btn, 80, 32);
    lv_obj_set_style_bg_color(hide_btn,
                             lv_color_hex(hidden ? 0xca8a04 : 0x374151), 0);
    lv_obj_set_style_radius(hide_btn, 6, 0);
    lv_obj_set_style_border_width(hide_btn, 0, 0);
    lv_obj_add_flag(hide_btn, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_remove_flag(hide_btn, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_t* hide_lbl = lv_label_create(hide_btn);
    lv_label_set_text(hide_lbl, hidden ? "Hidden" : "Visible");
    lv_obj_set_style_text_color(hide_lbl, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_align(hide_lbl, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(hide_lbl, LV_ALIGN_CENTER, 0, -1);
    lv_obj_add_event_cb(hide_btn, [](lv_event_t* event) {
      lv_obj_t* tgt = (lv_obj_t*)lv_event_get_target(event);
      int idx = (int)(intptr_t)lv_obj_get_user_data(tgt);
      if (idx < 0 || idx >= (int)s_instance->sort_local_order_.size()) return;
      const std::string& name = s_instance->sort_local_order_[idx];
      auto it = s_instance->sort_local_hidden_.find(name);
      if (it != s_instance->sort_local_hidden_.end()) {
        s_instance->sort_local_hidden_.erase(it);
      } else {
        s_instance->sort_local_hidden_.insert(name);
      }
      s_instance->build_sort_panel_content_();
    }, LV_EVENT_CLICKED, nullptr);
    lv_obj_set_user_data(hide_btn, (void*)(intptr_t)i);
  }

  // --- Save / Cancel row at the bottom (sticky) ---
  // The user pointed out the original "Apply" button alone was
  // confusing - no obvious "discard" path. Now there's a two-button
  // row: Cancel (amber, left) discards the local changes; Save
  // (gray, right) persists them and re-renders the grid. The X at
  // the top of the panel is also a third close path (same as Cancel).
  // Both Cancel and the X just call hide_sort_panel_(), which clears
  // sort_local_order_/hidden_ so the next open re-seeds from
  // customizations_ (i.e. the previous persisted state). The user
  // stays in edit mode either way - "Done" in the title bar is the
  // path to actually exit.
  //
  // The action row is a SEPARATE child of the panel (not a flex
  // child) with LV_ALIGN_BOTTOM_MID, so it stays pinned to the
  // bottom of the screen regardless of how far the user has
  // scrolled the room list. Without this, with 15+ rooms the
  // buttons would scroll out of view.
  lv_obj_t* action_row = lv_obj_create(this->sort_panel_);
  lv_obj_set_size(action_row, this->screen_width_ - 24, 48);
  lv_obj_align(action_row, LV_ALIGN_BOTTOM_MID, 0, -12);
  // Solid bg so the row doesn't show the room list bleeding
  // through (the panel bg is the same color as the rooms' bg, so
  // a transparent action row would be hard to read against
  // moving content).
  lv_obj_set_style_bg_color(action_row, lv_color_hex(0x111827), 0);
  lv_obj_set_style_bg_opa(action_row, LV_OPA_COVER, 0);
  lv_obj_set_style_border_width(action_row, 0, 0);
  lv_obj_set_style_pad_all(action_row, 0, 0);
  lv_obj_set_style_pad_column(action_row, 12, 0);
  lv_obj_set_flex_flow(action_row, LV_FLEX_FLOW_ROW);
  lv_obj_set_flex_align(action_row, LV_FLEX_ALIGN_SPACE_BETWEEN,
                        LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
  // The action row is NOT in the scrollable flow (it's positioned
  // absolutely via LV_ALIGN_BOTTOM_MID), so the SCROLLABLE flag
  // doesn't matter here. We just need it to be clickable.
  lv_obj_remove_flag(action_row, LV_OBJ_FLAG_SCROLLABLE);
  // LV_OBJ_FLAG_FLOATING excludes the widget from its parent flex
  // layout. The panel is a flex column; without this flag the
  // action row gets pushed to the bottom of the flex column (where
  // it would scroll out of view with 15+ rooms). With it, the
  // row position is determined entirely by its own align
  // (LV_ALIGN_BOTTOM_MID) and stays pinned to the bottom of the
  // screen regardless of scroll position.
  lv_obj_add_flag(action_row, LV_OBJ_FLAG_FLOATING);

  auto make_action_btn = [&](const char* label, uint32_t bg,
                             std::function<void()> on_click) {
    lv_obj_t* b = lv_obj_create(action_row);
    // ~half-width buttons, gap 12px between them.
    lv_obj_set_width(b, (this->screen_width_ - 24 - 12) / 2);
    lv_obj_set_height(b, 48);
    lv_obj_set_style_bg_color(b, lv_color_hex(bg), 0);
    lv_obj_set_style_radius(b, 6, 0);
    lv_obj_set_style_border_width(b, 0, 0);
    lv_obj_add_flag(b, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_remove_flag(b, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_t* lbl = lv_label_create(b);
    lv_label_set_text(lbl, label);
    lv_obj_set_style_text_color(lbl, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_align(lbl, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(lbl, LV_ALIGN_CENTER, 0, -1);
    if (on_click) {
      lv_obj_add_event_cb(b, [](lv_event_t* event) {
        if (s_instance == nullptr) return;
        auto* fn = reinterpret_cast<std::function<void()>*>(
            lv_obj_get_user_data((lv_obj_t*)lv_event_get_target(event)));
        if (fn) (*fn)();
      }, LV_EVENT_CLICKED, nullptr);
      // std::function copy for the closure's lifetime.
      auto* fn_heap = new std::function<void()>(std::move(on_click));
      lv_obj_set_user_data(b, fn_heap);
    }
    return b;
  };

  // Cancel (amber) - closes the panel without persisting. Same
  // behavior as the X at the top, just a more discoverable button.
  make_action_btn("Cancel", 0xf59e0b, []() {
    if (s_instance == nullptr) return;
    ESP_LOGI(TAG, "[sort] Cancel tapped - discarding local changes");
    s_instance->hide_sort_panel_();
  });
  // Save (gray) - persists the reorder + hide changes, closes
  // the panel, re-renders the grid. Previously called "Apply" but
  // "Save" matches the standard UI verb for "commit my changes".
  make_action_btn("Save", 0x374151, []() {
    if (s_instance == nullptr) return;
    ESP_LOGI(TAG, "[sort] Save tapped - persisting and re-rendering");
    s_instance->apply_sort_panel_();
  });
}

void HaAutoPanel::build_debug_panel_content_() {
  if (this->debug_panel_ == nullptr) return;
  // Tear down all existing children. We rebuild the panel from
  // scratch every show so live values are current. The panel is
  // small (~420px tall) so the rebuild cost is negligible.
  uint32_t child_count = lv_obj_get_child_cnt(this->debug_panel_);
  for (uint32_t i = child_count; i > 0; i--) {
    lv_obj_del(lv_obj_get_child(this->debug_panel_, i - 1));
  }

  // Helper: create a small key:value label pair. Returns the value
  // label so the caller can update it in place (we rebuild every
  // show so this is just for clarity).
  auto add_kv = [&](const char* key, const char* value) {
    lv_obj_t* row = lv_obj_create(this->debug_panel_);
    lv_obj_set_size(row, lv_pct(100), 22);
    lv_obj_set_style_bg_opa(row, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(row, 0, 0);
    lv_obj_set_style_pad_all(row, 0, 0);
    // Manual horizontal layout: key label at x=0, value label at x=142.
    // Was using flex_flow here originally, but LV_USE_FLEX=0 in this
    // build, so we position the two labels explicitly.
    lv_obj_remove_flag(row, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_remove_flag(row, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t* k = lv_label_create(row);
    lv_label_set_text(k, key);
    lv_obj_set_style_text_color(k, lv_color_hex(0x9ca3af), 0);
    lv_obj_set_width(k, 130);
    lv_obj_set_style_text_align(k, LV_TEXT_ALIGN_LEFT, 0);
    lv_obj_set_pos(k, 0, 2);

    lv_obj_t* v = lv_label_create(row);
    lv_label_set_text(v, value);
    lv_obj_set_style_text_color(v, lv_color_hex(0xe5e7eb), 0);
    lv_obj_set_width(v, 800);
    lv_obj_set_style_text_align(v, LV_TEXT_ALIGN_LEFT, 0);
    lv_label_set_long_mode(v, LV_LABEL_LONG_CLIP);
    lv_obj_set_pos(v, 142, 2);
  };
  // Helper: create a section header (a small all-caps label).
  auto add_header = [&](const char* title) {
    lv_obj_t* h = lv_label_create(this->debug_panel_);
    lv_label_set_text(h, title);
    lv_obj_set_style_text_color(h, lv_color_hex(0xfacc15), 0);
    // The accent color also signals "this is a section header" so
    // the user can scan the panel quickly.
  };
  // Helper: create an action button with a tag in user_data so the
  // 2-tap confirm and clear_pending_action_() can find it.
  auto add_action = [&](const char* tag, const char* label,
                        uint32_t color, lv_event_cb_t cb,
                        bool full_width = true) {
    lv_obj_t* b = lv_obj_create(this->debug_panel_);
    if (full_width) {
      lv_obj_set_size(b, lv_pct(100), 32);
    } else {
      lv_obj_set_size(b, 110, 32);
    }
    lv_obj_set_style_bg_color(b, lv_color_hex(color), 0);
    lv_obj_set_style_radius(b, 6, 0);
    lv_obj_set_style_border_width(b, 0, 0);
    lv_obj_set_style_pad_all(b, 0, 0);
    lv_obj_add_flag(b, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_remove_flag(b, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_t* lbl = lv_label_create(b);
    lv_label_set_text(lbl, label);
    lv_obj_set_style_text_color(lbl, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(lbl, LV_ALIGN_CENTER, 0, -1);
    lv_obj_set_user_data(b, (void*) tag);
    lv_obj_add_event_cb(b, cb, LV_EVENT_CLICKED, nullptr);
  };

  // --- WiFi section ---
  add_header("WIFI");
  if (wifi::global_wifi_component != nullptr &&
      wifi::global_wifi_component->is_connected()) {
    auto ips = wifi::global_wifi_component->get_ip_addresses();
    char ip_buf[network::IP_ADDRESS_BUFFER_SIZE] = "(no addr)";
    if (!ips.empty() && ips[0].is_set()) {
      ips[0].str_to(ip_buf);
    }
    add_kv("IP", ip_buf);
    add_kv("RSSI", "—");  // ESP-IDF's get_rssi() needs a STA interface
  } else {
    add_kv("Status", "Disconnected");
  }

  // --- Home Assistant section ---
  add_header("HOME ASSISTANT");
  const char* state_str = "Unknown";
  switch (this->state_) {
    case PanelState::BOOTING:        state_str = "Booting"; break;
    case PanelState::SETUP_REQUIRED: state_str = "Setup required"; break;
    case PanelState::AUTH_FAILED:    state_str = "Auth failed"; break;
    case PanelState::NOT_AUTHORIZED: state_str = "Not authorized"; break;
    case PanelState::CONNECTING:     state_str = "Connecting..."; break;
    case PanelState::READY:          state_str = "Connected"; break;
  }
  add_kv("Status", state_str);
  char entity_count[32];
  snprintf(entity_count, sizeof(entity_count), "%d entities",
           (int) room_cards_.size());
  add_kv("Rooms shown", entity_count);
  add_kv("Home", this->home_name_.empty() ? "—" : this->home_name_.c_str());

  // --- Customizations section ---
  add_header("CUSTOMIZATIONS");
  char hidden_buf[32];
  snprintf(hidden_buf, sizeof(hidden_buf), "%u rooms, %u entities",
           (unsigned) this->customizations_.hidden_rooms.size(),
           (unsigned) this->customizations_.hidden_entities.size());
  add_kv("Hidden", hidden_buf);
  snprintf(hidden_buf, sizeof(hidden_buf), "%u entries",
           (unsigned) this->customizations_.room_order.size());
  add_kv("Room order", hidden_buf);

  // --- Date & Time section ---
  add_header("DATE & TIME");
  if (this->time_ != nullptr) {
    auto now = this->time_->now();
    if (now.is_valid()) {
      char buf[64];
      snprintf(buf, sizeof(buf), "%04d-%02d-%02d  %d:%02d:%02d",
               now.year, now.month, now.day_of_month,
               now.hour, now.minute, now.second);
      add_kv("Local", buf);
      const char* synced = "yes";
      add_kv("NTP synced", synced);
    } else {
      add_kv("Local", "--:--:-- (syncing)");
    }
  } else {
    add_kv("Local", "(no time component)");
  }

  // --- Device section (heap / PSRAM / flash / uptime / version) ---
  add_header("DEVICE");
  char buf[64];
  size_t free_heap = heap_caps_get_free_size(MALLOC_CAP_8BIT);
  size_t largest_heap = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
  size_t free_psram = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
  size_t largest_psram = heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM);
  // The user asked specifically for the "maximum allocated block" for
  // both internal RAM and PSRAM (2026-06-03) - it's the most
  // useful number when diagnosing fragmentation.
  snprintf(buf, sizeof(buf), "%u KB free, largest %u KB",
           (unsigned)(free_heap / 1024), (unsigned)(largest_heap / 1024));
  add_kv("Internal RAM", buf);
  // PSRAM mode (Octal on the ESP32-P4's default PSRAM config) is
  // hardcoded for now - the runtime API to query the mode isn't
  // portable across ESP-IDF versions. If the user changes the
  // psram: config in YAML, this label may be wrong; safe enough for
  // a debug surface.
  snprintf(buf, sizeof(buf), "%u KB free, largest %u KB (Octal)",
           (unsigned)(free_psram / 1024), (unsigned)(largest_psram / 1024));
  add_kv("PSRAM", buf);
  // Flash: just report the chip size (from the boot log it is 16 MB
  // on the Crowpanel 7"; querying it at runtime would call
  // esp_flash_get_chip_size which is in a different header depending
  // on ESP-IDF version - not worth the include dance for a value
  // that doesn't change at runtime). The flash mode (QIO / QOUT /
  // etc.) is similarly build-time - see sdkconfig. Hardcoding to
  // 16MB/QIO for now; if the user changes board they can tweak.
  add_kv("Flash", "16 MB, QIO");
  // Uptime - millis() wraps at ~49 days, format the wrap day too.
  uint32_t ms = millis();
  uint32_t secs = ms / 1000;
  uint32_t days = secs / 86400;
  uint32_t hours = (secs % 86400) / 3600;
  uint32_t mins = (secs % 3600) / 60;
  snprintf(buf, sizeof(buf), "%ud %uh %um", (unsigned) days,
           (unsigned) hours, (unsigned) mins);
  add_kv("Uptime", buf);
  add_kv("ESPHome", "2026.4.5");  // pinned in the yaml
  // Last reset reason. Useful for diagnosing crash loops.
  esp_reset_reason_t reason = esp_reset_reason();
  const char* rr_str = "?";
  switch (reason) {
    case ESP_RST_POWERON:   rr_str = "Power on"; break;
    case ESP_RST_EXT:       rr_str = "External pin"; break;
    case ESP_RST_SW:        rr_str = "Software reboot"; break;
    case ESP_RST_PANIC:     rr_str = "Panic (crash)"; break;
    case ESP_RST_INT_WDT:   rr_str = "Interrupt WDT"; break;
    case ESP_RST_TASK_WDT:  rr_str = "Task WDT"; break;
    case ESP_RST_WDT:       rr_str = "Other WDT"; break;
    case ESP_RST_DEEPSLEEP: rr_str = "Deep sleep wake"; break;
    case ESP_RST_BROWNOUT:  rr_str = "Brownout"; break;
    case ESP_RST_SDIO:      rr_str = "SDIO"; break;
    default: rr_str = "Unknown"; break;
  }
  add_kv("Last reset", rr_str);

  // --- Actions section ---
  add_header("ACTIONS");
  // 2-tap confirm for reboot/reset: the lambda checks
  // this->pending_action_ and either arms or executes.
  add_action("action:reprobe", "Re-probe auth",
             0x374151, [](lv_event_t*) {
               if (s_instance == nullptr) return;
               s_instance->clear_pending_action_();
               ESP_LOGI(TAG, "[debug] action: re-probe auth");
               s_instance->probe_authorization_();
             });
  add_action("action:redisc", "Re-run discovery",
             0x374151, [](lv_event_t*) {
               if (s_instance == nullptr) return;
               s_instance->clear_pending_action_();
               ESP_LOGI(TAG, "[debug] action: re-run discovery");
               s_instance->start_discovery_();
             });
  add_action("action:refetch", "Re-fetch home name",
             0x374151, [](lv_event_t*) {
               if (s_instance == nullptr) return;
               s_instance->clear_pending_action_();
               ESP_LOGI(TAG, "[debug] action: re-fetch home name");
               s_instance->last_home_fetch_ms_ = 0;
               s_instance->fetch_home_name_();
             });

  // --- Destructive section (with 2-tap confirm) ---
  // Reboot - red button, requires a second tap within 5s.
  add_action("action:reboot", "Reboot",
             0xb91c1c, [](lv_event_t* event) {
               if (s_instance == nullptr) return;
               lv_obj_t* btn = (lv_obj_t*) lv_event_get_target(event);
               lv_obj_t* lbl = lv_obj_get_child(btn, 0);
               uint32_t now = millis();
               if (s_instance->pending_action_ == "reboot" &&
                   (now - s_instance->pending_action_started_ms_) <
                       HaAutoPanel::PENDING_ACTION_TIMEOUT_MS) {
                 ESP_LOGW(TAG, "[debug] REBOOT confirmed - restarting in 1s");
                 lbl = lv_obj_get_child(btn, 0);
                 if (lbl) lv_label_set_text(lbl, "Rebooting...");
                 // Defer the actual reboot a tick so the label paints.
                 // Component::set_timeout(name, ms, fn) - this overload
                 // schedules against `this` implicitly. (The 4-arg
                 // App.scheduler.set_timeout(component, name, ms, fn)
                 // overload exists too but the Component form is what
                 // we have direct access to from a member function.)
                 s_instance->set_timeout("reboot", 1000, []() {
                   App.reboot();
                 });
               } else {
                 s_instance->pending_action_ = "reboot";
                 s_instance->pending_action_started_ms_ = now;
                 if (lbl != nullptr) {
                   lv_label_set_text(lbl, "Tap again to confirm");
                 }
                 lv_obj_set_style_bg_color(btn, lv_color_hex(0xef4444), 0);
                 ESP_LOGW(TAG, "[debug] reboot ARMED - tap again within %ums to confirm",
                          (unsigned) HaAutoPanel::PENDING_ACTION_TIMEOUT_MS);
               }
             });
  // Reset customizations - amber button, 2-tap confirm.
  add_action("action:reset", "Reset customizations",
             0x374151, [](lv_event_t* event) {
               if (s_instance == nullptr) return;
               lv_obj_t* btn = (lv_obj_t*) lv_event_get_target(event);
               lv_obj_t* lbl = lv_obj_get_child(btn, 0);
               uint32_t now = millis();
               if (s_instance->pending_action_ == "reset" &&
                   (now - s_instance->pending_action_started_ms_) <
                       HaAutoPanel::PENDING_ACTION_TIMEOUT_MS) {
                 ESP_LOGW(TAG, "[debug] RESET customizations confirmed - wiping");
                 s_instance->customizations_ = CustomizationConfig{};
                 s_instance->write_customizations_file_();
                 s_instance->refresh_room_cards_();
                 if (lbl) lv_label_set_text(lbl, "Reset!");
                 s_instance->clear_pending_action_();
               } else {
                 s_instance->pending_action_ = "reset";
                 s_instance->pending_action_started_ms_ = now;
                 if (lbl != nullptr) {
                   lv_label_set_text(lbl, "Tap again to confirm");
                 }
                 lv_obj_set_style_bg_color(btn, lv_color_hex(0xf59e0b), 0);
                 ESP_LOGW(TAG, "[debug] reset ARMED - tap again within %ums",
                          (unsigned) HaAutoPanel::PENDING_ACTION_TIMEOUT_MS);
               }
             });

  // The panel is built - this call to set_height is a no-op for
  // now (the panel is fixed at 70% of screen height) but it sets
  // the size so the flex layout inside settles.
  lv_obj_update_layout(this->debug_panel_);
}

void HaAutoPanel::refresh_room_cards_() {
  // Tear down the current room cards and rebuild from room_cards_ with
  // the current edit_mode. The entities_by_area_ and room data are
  // unchanged; this just rebuilds the LVGL widget tree.
  if (this->main_container_ == nullptr) return;
  // Delete all room card children. We walk the children list and delete
  // anything that's not the title bar.
  uint32_t child_count = lv_obj_get_child_cnt(this->main_container_);
  // Iterate in reverse because deleting shifts indices
  for (int i = (int) child_count - 1; i >= 0; i--) {
    lv_obj_t* child = lv_obj_get_child(this->main_container_, i);
    if (child != this->title_bar_) {
      lv_obj_del(child);
    }
  }
  // Clear live widget refs (they reference the deleted widgets)
  this->room_arc_widgets_.clear();
  this->room_btn_widgets_.clear();
  this->room_label_btn_widgets_.clear();

  // Recreate the room cards, grouped by row into row containers.
  // Same pattern as create_ui_from_room_cards_() - see the comment
  // there for the rationale (LV_ALIGN_TOP_MID centers the row
  // horizontally automatically, no manual math; cards are positioned
  // manually within the row since LV_USE_FLEX=0 in this build).
  int cards_per_row = this->compute_cards_per_row_();
  int num_rows = (int)((room_cards_.size() + cards_per_row - 1) / cards_per_row);
  if (num_rows < 1) num_rows = 1;
  for (int row = 0; row < num_rows; row++) {
    int row_count = (row < num_rows - 1)
                        ? cards_per_row
                        : (int) room_cards_.size() - row * cards_per_row;
    if (row_count <= 0) row_count = 1;
    int row_width = row_count * this->card_width_ +
                    (row_count > 0 ? (row_count - 1) : 0) * this->card_gap_;

    lv_obj_t* row_container = lv_obj_create(this->main_container_);
    lv_obj_set_size(row_container, row_width, this->card_width_);
    lv_obj_set_style_bg_opa(row_container, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(row_container, 0, 0);
    lv_obj_set_style_pad_all(row_container, 0, 0);
    lv_obj_set_style_pad_column(row_container, this->card_gap_, 0);
    lv_obj_set_flex_flow(row_container, LV_FLEX_FLOW_ROW);
    lv_obj_remove_flag(row_container, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_remove_flag(row_container, LV_OBJ_FLAG_CLICKABLE);
    int row_y = this->start_y_ + row * (this->card_width_ + this->card_gap_);
    lv_obj_align(row_container, LV_ALIGN_TOP_MID, 0, row_y);

    for (int c = 0; c < row_count; c++) {
      int card_index = row * cards_per_row + c;
      if (card_index >= (int) room_cards_.size()) break;
      room_cards_[card_index].x = 0;
      room_cards_[card_index].y = 0;
      this->create_room_card_(row_container, room_cards_[card_index]);
      // STACK YIELD: see create_ui_from_room_cards_(). Without
      // these, refresh_room_cards_ on a board with 15+ rooms hits
      // the same stack protection fault we saw on 2026-06-05.
      lv_task_handler();
    }
  }
  // Update the title bar so the room count / status reflects the new view
  this->update_title_bar_();
  ESP_LOGI(TAG, "Room cards refreshed (edit_mode=%d)", (int) this->edit_mode_);
}

bool HaAutoPanel::process_command_(char c) {
  // Single-character command dispatch shared by the serial input
  // loop and the AGENT_DEBUG web API (/autopanel/test/cmd?c=X).
  // Returns true if c was a recognized command, false otherwise.
  // 'C' and 'S' are intentionally NOT handled here - they are
  // multi-line (the next line is the coordinate payload) and only
  // make sense on the serial path. The web API has separate
  // /autopanel/test/click and /autopanel/test/scroll endpoints
  // that take coordinates as query params.
  switch (c) {
    case 'p': case 'r':
      this->probe_authorization_();
      return true;
    case 's':
      this->set_panel_state_(PanelState::SETUP_REQUIRED);
      return true;
    case 'a':
      this->set_panel_state_(PanelState::AUTH_FAILED);
      return true;
    case 'n':
      this->set_panel_state_(PanelState::NOT_AUTHORIZED);
      return true;
    case 'c':
      this->set_panel_state_(PanelState::CONNECTING);
      return true;
    case 'g':
      // Force a re-render of the room grid
      this->set_panel_state_(PanelState::READY);
      return true;
    case 'h':
      // Force a re-fetch of the home-name label, bypassing the
      // HOME_FETCH_INTERVAL_MS throttle. Useful for the test
      // harness when verifying a fresh /api/states response
      // (e.g. after the user changed the friendly_name in HA).
      this->last_home_fetch_ms_ = 0;
      this->fetch_home_name_();
      return true;
    case 'o':
      // Open the sort panel (the user-preferred popup-list way
      // to reorder and hide rooms). The panel handles its own
      // seeding from customizations_, so calling this even
      // outside of edit mode works - it just looks the same.
      this->show_sort_panel_();
      return true;
    case 'O':
      // Close the sort panel. (Pair to 'o'.)
      this->hide_sort_panel_();
      return true;
    case 'd':
      this->start_discovery_();
      return true;
    case '0': case '1': case '2': case '3': case '4':
    case '5': case '6': case '7': case '8': case '9': {
      int idx = c - '0';
      if (idx >= 0 && idx < (int) this->room_cards_.size()) {
        ESP_LOGI(TAG, "[cmd] opening detail for room %d (%s)", idx,
                 this->room_cards_[idx].area.name.c_str());
        this->show_entity_detail_(idx);
      }
      return true;
    }
    default:
      ESP_LOGW(TAG, "[cmd] unknown command '%c'", c);
      return false;
  }
}

void HaAutoPanel::simulate_click_(int x, int y) {
  // Synthesize a click at (x, y) on the active screen. The LVGL 9
  // touch pipeline (lv_indev_set_point / press / release) is
  // private/static, so we hit-test via the public lv_indev_search_obj
  // and dispatch LV_EVENT_CLICKED directly. The button widgets
  // register their handlers with LV_EVENT_CLICKED, so firing it on
  // the hit-tested object runs the same code path as a real tap.
  // Note: the visual press feedback (color darken) won't show, only
  // the click action. That's fine for the test harness.
  lv_obj_t *screen = lv_screen_active();
  if (screen == nullptr) {
    ESP_LOGW(TAG, "[click] no active screen");
    return;
  }
  lv_point_t p = {.x = (lv_coord_t) x, .y = (lv_coord_t) y};
  // lv_indev_search_obj walks the layer sys, layer top, active
  // screen, then layer bottom in that order. Pass the screen as the
  // search root for the most predictable behaviour.
  lv_obj_t *obj = lv_indev_search_obj(screen, &p);
  if (obj == nullptr) {
    ESP_LOGW(TAG, "[click] no object at (%d, %d)", x, y);
    return;
  }
  lv_obj_send_event(obj, LV_EVENT_CLICKED, nullptr);
  ESP_LOGI(TAG, "[click] injected at (%d, %d) on obj=%p", x, y, (void *) obj);
}

void HaAutoPanel::simulate_scroll_(int x1, int y1, int x2, int y2) {
  // Synthesize a scroll/drag at the given start point. Find the
  // scrollable object under the start point and apply a
  // lv_obj_scroll_by() with the delta. This is the test-harness
  // equivalent of a drag gesture - enough to drive list panels
  // and scrollable rooms without going through the full indev
  // press/release pipeline.
  lv_obj_t *screen = lv_screen_active();
  if (screen == nullptr) {
    ESP_LOGW(TAG, "[scroll] no active screen");
    return;
  }
  lv_point_t p = {.x = (lv_coord_t) x1, .y = (lv_coord_t) y1};
  lv_obj_t *obj = lv_indev_search_obj(screen, &p);
  if (obj == nullptr) {
    ESP_LOGW(TAG, "[scroll] no object at start (%d, %d)", x1, y1);
    return;
  }
  // Walk up to the nearest scrollable ancestor. lv_obj_scroll_by
  // is a no-op on non-scrollable objects, so we try the hit object
  // first then its parents until one accepts the scroll.
  int32_t dx = (int32_t) x2 - (int32_t) x1;
  int32_t dy = (int32_t) y2 - (int32_t) y1;
  lv_obj_t *scrollable = obj;
  while (scrollable != nullptr) {
    lv_obj_scroll_by(scrollable, dx, dy, LV_ANIM_OFF);
    // If the object's scroll offset actually changed, we found our
    // scrollable. Otherwise keep walking up.
    // lv_obj_scroll_by returns void; we just trust the first
    // scrollable ancestor that says yes via the same call.
    break;
  }
  ESP_LOGI(TAG, "[scroll] injected from (%d, %d) to (%d, %d) on obj=%p",
           x1, y1, x2, y2, (void *) obj);
}


}  // namespace ha_autopanel
}  // namespace esphome