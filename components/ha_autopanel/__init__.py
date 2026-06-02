import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID
from esphome.components import http_request, wifi, web_server_base
from esphome.components.esp32 import add_idf_component, include_builtin_idf_component

DEPENDENCIES = ["lvgl", "http_request", "api", "wifi", "web_server_base"]

ha_autopanel_ns = cg.esphome_ns.namespace("ha_autopanel")

HaAutoPanel = ha_autopanel_ns.class_(
    "HaAutoPanel", cg.Component
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(HaAutoPanel),
        cv.Required("ha_api_url"): cv.url,
        cv.Required("ha_api_password"): cv.string,
        cv.Optional("http_request_ref"): cv.use_id(http_request.HttpRequestComponent),
        cv.Optional("include_all", default=True): cv.boolean,
        # All filtering options default to empty so the user doesn't have
        # to hardcode them in YAML. In the no-code vision, these are
        # configured via the on-device web UI (LittleFS-backed). For now
        # if a user wants pre-configured filters they can still set them
        # in YAML.
        cv.Optional("include_areas", default=[]): cv.ensure_list(cv.string),
        cv.Optional("exclude_areas", default=[]): cv.ensure_list(cv.string),
        cv.Optional("domains", default=[]): cv.ensure_list(cv.string),
        cv.Optional("exclude_entities", default=[]): cv.ensure_list(cv.string),
        # Auto-fit layout: cards are square (width == height). The number of
        # cards per row is computed from the screen width and card_width.
        # Override screen_width/screen_height for non-1024x600 panels
        # (e.g. a square dial board).
        cv.Optional("card_width", default=250): cv.int_range(min=100, max=500),
        cv.Optional("card_gap", default=12): cv.int_range(min=0, max=50),
        cv.Optional("screen_width", default=1024): cv.int_range(min=200, max=4096),
        cv.Optional("screen_height", default=600): cv.int_range(min=200, max=4096),
        cv.Optional("start_x", default=10): cv.int_range(min=0, max=200),
        cv.Optional("start_y", default=12): cv.int_range(min=0, max=200),
        # Default brightness percentage when the ON/OFF button is tapped on
        # a room that has no recorded previous brightness. Used as the
        # bounce-back value.
        cv.Optional("default_on_pct", default=30): cv.int_range(min=1, max=100),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    # NOTE: add_lv_use() is intentionally not called here because it runs
    # too late to affect lv_conf.h generation. Widgets used by the C++ code
    # must be declared in the user's YAML under lvgl.widgets:.

    # Enable the action-response defines so we can build HomeassistantActionRequest
    # with a call_id and surface the response back to C++ for the auth probe.
    # The api component only sets these if the user has a YAML automation with
    # homeassistant.action on_success/on_error. We need them at the proto
    # level regardless.
    cg.add_define("USE_API_HOMEASSISTANT_ACTION_RESPONSES")
    cg.add_define("USE_API_HOMEASSISTANT_ACTION_RESPONSES_ERRORS")
    cg.add_define("USE_API_HOMEASSISTANT_ACTION_RESPONSES_JSON")

    # Re-enable the IDF managed joltwallet/littlefs component so we
    # can mount the 'storage' partition (declared in partitions.csv)
    # at /storage. ESPHome excludes joltwallet__littlefs (Arduino) by
    # default to keep build times down; this is the IDF managed variant
    # which is what we want for the esp-idf framework. The component
    # is fetched from its GitHub repo so the include path is updated.
    include_builtin_idf_component("joltwallet__littlefs")
    add_idf_component(
        name="joltwallet/littlefs",
        repo="https://github.com/joltwallet/esp_littlefs.git",
    )

    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    # Get the http_request component by its ID
    if "http_request_ref" in config:
        http_request_var = await cg.get_variable(config["http_request_ref"])
        cg.add(var.set_http_request(http_request_var))

    # The wifi component is accessed directly via wifi::global_wifi_component
    # in C++ - no Python-side reference needed. The component has a hard
    # dependency on 'wifi' which guarantees the global is populated.

    cg.add(var.set_ha_api_url(config["ha_api_url"]))
    cg.add(var.set_ha_api_password(config["ha_api_password"]))
    cg.add(var.set_include_all(config["include_all"]))
    cg.add(var.set_include_areas(config["include_areas"]))
    cg.add(var.set_exclude_areas(config["exclude_areas"]))
    cg.add(var.set_entity_domains(config["domains"]))
    cg.add(var.set_exclude_entities(config["exclude_entities"]))
    cg.add(var.set_card_width(config["card_width"]))
    cg.add(var.set_card_gap(config["card_gap"]))
    cg.add(var.set_screen_width(config["screen_width"]))
    cg.add(var.set_screen_height(config["screen_height"]))
    cg.add(var.set_start_x(config["start_x"]))
    cg.add(var.set_start_y(config["start_y"]))
    cg.add(var.set_default_on_pct(config["default_on_pct"]))
