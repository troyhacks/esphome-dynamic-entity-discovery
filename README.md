# ESPHome Dynamic Entity Discovery

A custom ESPHome component that dynamically queries Home Assistant for areas and entities, then creates LVGL widgets at runtime.

## Features

- Queries HA API to discover areas and entities at runtime
- Dynamically creates LVGL room cards with brightness arcs and toggle buttons
- Supports filtering by domain, area inclusion/exclusion
- Scrollable room grid and entity detail views
- No compile-time entity hardcoding - truly DRY

## Requirements

- ESPHome 2024.x or later
- LVGL component configured in your YAML
- Home Assistant instance with API access

## Installation

### Option 1: External Components (Recommended)

Add to your ESPHome YAML:

```yaml
external_components:
  - source:
      type: git
      url: https://github.com/YOURGITHUBUSER/esphome-dynamic-entity-discovery
    components: [dynamic_entity_discovery]
```

### Option 2: Local Copy

Copy the `esphome_dynamic_entity_discovery` folder to your project's `esphome/` directory.

## Configuration

```yaml
dynamic_entity_discovery:
  ha_api_url: "http://homeassistant.local:8123"
  ha_api_password: !secret ha_api_password
  include_all: true
  exclude_areas:
    - "Guest Bedroom"
  domains:
    - light
    - switch
    - sensor
    - binary_sensor
  exclude_entities:
    - "light.fake_light"
  grid_cols: 3
  grid_rows: 2
  grid_card_width: 330
  grid_card_height: 250
  grid_gap_x: 7
  grid_gap_y: 15
  start_x: 10
  start_y: 12
```

### Configuration Keys

| Key | Type | Default | Description |
|-----|------|---------|-------------|
| `ha_api_url` | string | **required** | Home Assistant API URL |
| `ha_api_password` | string | **required** | HA API password/token |
| `include_all` | boolean | `true` | Include all areas |
| `include_areas` | list | `[]` | Specific areas to include |
| `exclude_areas` | list | `[]` | Areas to exclude |
| `domains` | list | `["light"]` | Entity domains to show |
| `exclude_entities` | list | `[]` | Specific entities to exclude |
| `grid_cols` | int | `3` | Number of grid columns |
| `grid_rows` | int | `2` | Number of grid rows |
| `grid_card_width` | int | `330` | Room card width in pixels |
| `grid_card_height` | int | `250` | Room card height in pixels |
| `grid_gap_x` | int | `7` | Horizontal gap between cards |
| `grid_gap_y` | int | `15` | Vertical gap between cards |
| `start_x` | int | `10` | Starting X position |
| `start_y` | int | `12` | Starting Y position |

## Usage

Trigger discovery after Home Assistant connects:

```yaml
api:
  on_client_connected:
    - lambda: 'id(dynamic_discovery).trigger_discovery();'
```

## How It Works

1. `trigger_discovery()` is called after HA connection
2. Component queries HA API for areas and entities
3. Entities are filtered by domain and area rules
4. Room cards are created with:
   - Room name (clickable to show entity detail)
   - Brightness arc (for dimmable lights)
   - ON/OFF button
5. Tapping a room card shows entity detail view with individual controls

## Development Notes

This component uses LVGL's native C API for runtime widget creation. Key patterns:

```cpp
// Create widgets directly with LVGL functions
lv_obj_t* card = lv_obj_create(parent);
lv_obj_t* arc = lv_arc_create(card);
lv_obj_t* btn = lv_button_create(card);

// Event callbacks via lambdas
lv_obj_add_event_cb(arc, [](lv_event_t* event) {
    // Handle event
}, LV_EVENT_VALUE_CHANGED, nullptr);
```

## License

MIT