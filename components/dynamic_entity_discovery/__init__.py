import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID

DEPENDENCIES = ["lvgl"]

dynamic_entity_discovery_ns = cg.esphome_ns.namespace("dynamic_entity_discovery")

DynamicEntityDiscovery = dynamic_entity_discovery_ns.class_(
    "DynamicEntityDiscovery", cg.Component
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(DynamicEntityDiscovery),
        cv.Required("ha_api_url"): cv.url,
        cv.Required("ha_api_password"): cv.string,
        cv.Optional("include_all", default=True): cv.boolean,
        cv.Optional("include_areas", default=[]): cv.ensure_list(cv.string),
        cv.Optional("exclude_areas", default=[]): cv.ensure_list(cv.string),
        cv.Optional("domains", default=["light"]): cv.ensure_list(cv.string),
        cv.Optional("exclude_entities", default=[]): cv.ensure_list(cv.string),
        cv.Optional("grid_cols", default=3): cv.int_range(min=1, max=6),
        cv.Optional("grid_rows", default=2): cv.int_range(min=1, max=4),
        cv.Optional("grid_card_width", default=330): cv.int_range(min=100, max=500),
        cv.Optional("grid_card_height", default=250): cv.int_range(min=100, max=500),
        cv.Optional("grid_gap_x", default=7): cv.int_range(min=0, max=50),
        cv.Optional("grid_gap_y", default=15): cv.int_range(min=0, max=50),
        cv.Optional("start_x", default=10): cv.int_range(min=0, max=200),
        cv.Optional("start_y", default=12): cv.int_range(min=0, max=200),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    # Force-enable LVGL widgets needed for dynamic UI creation
    from esphome.components.lvgl.helpers import add_lv_use
    add_lv_use("ARC")
    add_lv_use("LABEL")
    add_lv_use("BUTTON")

    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    cg.add(var.set_ha_api_url(config["ha_api_url"]))
    cg.add(var.set_ha_api_password(config["ha_api_password"]))
    cg.add(var.set_include_all(config["include_all"]))
    cg.add(var.set_include_areas(config["include_areas"]))
    cg.add(var.set_exclude_areas(config["exclude_areas"]))
    cg.add(var.set_entity_domains(config["domains"]))
    cg.add(var.set_exclude_entities(config["exclude_entities"]))
    cg.add(var.set_grid_cols(config["grid_cols"]))
    cg.add(var.set_grid_rows(config["grid_rows"]))
    cg.add(var.set_grid_card_width(config["grid_card_width"]))
    cg.add(var.set_grid_card_height(config["grid_card_height"]))
    cg.add(var.set_grid_gap_x(config["grid_gap_x"]))
    cg.add(var.set_grid_gap_y(config["grid_gap_y"]))
    cg.add(var.set_start_x(config["start_x"]))
    cg.add(var.set_start_y(config["start_y"]))