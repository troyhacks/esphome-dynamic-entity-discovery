#include "dynamic_entity_discovery.h"
#include "esphome/core/log.h"
#include <lvgl.h>
#include "esphome/components/lvgl/lvgl_proxy.h"

namespace esphome {
namespace dynamic_entity_discovery {

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

void DynamicEntityDiscovery::start_discovery_() {
  ESP_LOGI(TAG, "=== Starting Dynamic Entity Discovery ===");

  // Simulate discovery with 6 areas
  ESP_LOGI(TAG, "Simulating discovery with placeholder areas...");

  discovered_areas_.push_back({"living_room", "Living Room"});
  discovered_areas_.push_back({"kitchen", "Kitchen"});
  discovered_areas_.push_back({"bedroom", "Bedroom"});
  discovered_areas_.push_back({"bathroom", "Bathroom"});
  discovered_areas_.push_back({"office", "Office"});
  discovered_areas_.push_back({"garage", "Garage"});
  discovered_areas_.push_back({"dining_room", "Dining Room"});
  discovered_areas_.push_back({"hallway", "Hallway"});

  // Add multiple entities to each area for testing scrolling
  for (const auto& area : discovered_areas_) {
    // Main light with brightness
    Entity light1;
    light1.entity_id = area.area_id + "_main_light";
    light1.name = area.name + " Main Light";
    light1.domain = "light";
    light1.area_id = area.area_id;
    light1.has_brightness = true;
    entities_by_area_[area.area_id].push_back(light1);

    // Ceiling fan / dimmer light
    Entity light2;
    light2.entity_id = area.area_id + "_ceiling_light";
    light2.name = area.name + " Ceiling Light";
    light2.domain = "light";
    light2.area_id = area.area_id;
    light2.has_brightness = true;
    entities_by_area_[area.area_id].push_back(light2);

    // Switch entity
    Entity sw;
    sw.entity_id = area.area_id + "_wall_switch";
    sw.name = area.name + " Wall Switch";
    sw.domain = "switch";
    sw.area_id = area.area_id;
    sw.has_brightness = false;
    entities_by_area_[area.area_id].push_back(sw);

    // Temperature sensor
    Entity sensor;
    sensor.entity_id = area.area_id + "_temp_sensor";
    sensor.name = area.name + " Temperature";
    sensor.domain = "sensor";
    sensor.area_id = area.area_id;
    sensor.has_brightness = false;
    entities_by_area_[area.area_id].push_back(sensor);

    // Humidity sensor
    Entity humid;
    humid.entity_id = area.area_id + "_humidity_sensor";
    humid.name = area.name + " Humidity";
    humid.domain = "sensor";
    humid.area_id = area.area_id;
    humid.has_brightness = false;
    entities_by_area_[area.area_id].push_back(humid);

    // Motion detector
    Entity motion;
    motion.entity_id = area.area_id + "_motion";
    motion.name = area.name + " Motion";
    motion.domain = "binary_sensor";
    motion.area_id = area.area_id;
    motion.has_brightness = false;
    entities_by_area_[area.area_id].push_back(motion);

    // Lamp light
    Entity light3;
    light3.entity_id = area.area_id + "_lamp";
    light3.name = area.name + " Lamp";
    light3.domain = "light";
    light3.area_id = area.area_id;
    light3.has_brightness = true;
    entities_by_area_[area.area_id].push_back(light3);

    // Smart outlet/switch
    Entity outlet;
    outlet.entity_id = area.area_id + "_outlet";
    outlet.name = area.name + " Outlet";
    outlet.domain = "switch";
    outlet.area_id = area.area_id;
    outlet.has_brightness = false;
    entities_by_area_[area.area_id].push_back(outlet);

    // Power meter
    Entity power;
    power.entity_id = area.area_id + "_power";
    power.name = area.name + " Power";
    power.domain = "sensor";
    power.area_id = area.area_id;
    power.has_brightness = false;
    entities_by_area_[area.area_id].push_back(power);

    // Cover/blind
    Entity cover;
    cover.entity_id = area.area_id + "_cover";
    cover.name = area.name + " Blind";
    cover.domain = "cover";
    cover.area_id = area.area_id;
    cover.has_brightness = false;
    entities_by_area_[area.area_id].push_back(cover);

    // AC/Climate
    Entity climate;
    climate.entity_id = area.area_id + "_ac";
    climate.name = area.name + " AC";
    climate.domain = "climate";
    climate.area_id = area.area_id;
    climate.has_brightness = false;
    entities_by_area_[area.area_id].push_back(climate);

    // Door sensor
    Entity door;
    door.entity_id = area.area_id + "_door";
    door.name = area.name + " Door";
    door.domain = "binary_sensor";
    door.area_id = area.area_id;
    door.has_brightness = false;
    entities_by_area_[area.area_id].push_back(door);
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
    lv_obj_set_user_data(arc, (void*)(intptr_t)entity_index);

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
