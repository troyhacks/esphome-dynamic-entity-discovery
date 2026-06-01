#pragma once

#include <string>
#include <vector>
#include <map>
#include "esphome/core/component.h"
#include "esphome/components/lvgl/lvgl_proxy.h"
#include "esphome/components/http_request/http_request.h"

namespace esphome {
namespace dynamic_entity_discovery {

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
  bool has_brightness{false};
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

class DynamicEntityDiscovery : public Component {
 public:
  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::AFTER_WIFI; }

  // Called from YAML lambda to trigger discovery after HA connects
  void trigger_discovery();

  void set_ha_api_url(const std::string& url) { this->ha_api_url_ = url; }
  void set_ha_api_password(const std::string& password) { this->ha_api_password_ = password; }
  void set_http_request(http_request::HttpRequestComponent* http) { this->http_request_ = http; }
  void set_include_all(bool include_all) { this->include_all_ = include_all; }
  void set_include_areas(const std::vector<std::string>& areas) { this->include_areas_ = areas; }
  void set_exclude_areas(const std::vector<std::string>& areas) { this->exclude_areas_ = areas; }
  void set_entity_domains(const std::vector<std::string>& domains) { this->entity_domains_ = domains; }
  void set_exclude_entities(const std::vector<std::string>& entities) { this->exclude_entities_ = entities; }
  void set_grid_cols(int cols) { this->grid_cols_ = cols; }
  void set_grid_rows(int rows) { this->grid_rows_ = rows; }
  void set_grid_card_width(int width) { this->grid_card_width_ = width; }
  void set_grid_card_height(int height) { this->grid_card_height_ = height; }
  void set_grid_gap_x(int gap) { this->grid_gap_x_ = gap; }
  void set_grid_gap_y(int gap) { this->grid_gap_y_ = gap; }
  void set_start_x(int x) { this->start_x_ = x; }
  void set_start_y(int y) { this->start_y_ = y; }

 protected:
  std::string ha_api_url_;
  std::string ha_api_password_;
  http_request::HttpRequestComponent* http_request_{nullptr};
  bool include_all_{true};
  std::vector<std::string> include_areas_;
  std::vector<std::string> exclude_areas_;
  std::vector<std::string> entity_domains_{"light"};
  std::vector<std::string> exclude_entities_;
  int grid_cols_{3};
  int grid_rows_{2};
  int grid_card_width_{330};
  int grid_card_height_{250};
  int grid_gap_x_{7};
  int grid_gap_y_{15};
  int start_x_{10};
  int start_y_{12};

  std::vector<Area> discovered_areas_;
  std::map<std::string, std::vector<Entity>> entities_by_area_;
  std::vector<RoomCard> room_cards_;

  // UI state - stored for navigation between room grid and entity detail
  lv_obj_t* main_container_{nullptr};
  lv_obj_t* detail_container_{nullptr};
  int current_room_index_{-1};

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
  int get_card_x_(int col) const;
  int get_card_y_(int row) const;
  uint32_t get_room_color_(int index) const;
};

}  // namespace dynamic_entity_discovery
}  // namespace esphome