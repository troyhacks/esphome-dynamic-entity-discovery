import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID
from esphome.components import http_request, time, wifi, web_server_base
from esphome.components.esp32 import add_idf_component, include_builtin_idf_component
from esphome.components.lvgl.defines import add_define as lv_add_define

DEPENDENCIES = ["lvgl", "http_request", "api", "time", "wifi", "web_server_base"]

ha_autopanel_ns = cg.esphome_ns.namespace("ha_autopanel")

HaAutoPanel = ha_autopanel_ns.class_(
    "HaAutoPanel", cg.Component
)

# Force-enable the LVGL flex layout at module-load time. ESPHome's
# lvgl component defaults LV_USE_FLEX to 0 in the generated
# lv_conf.h. Our title-bar right cluster and the room grid both
# use lv_obj_set_flex_flow so the layout reflows across screen
# widths and rotation - absolute x/y positions would break under
# rotation or any other screen-size change.
#
# This call has to happen at module load (NOT inside to_code())
# because lvgl's to_code() reads its defines dict and generates
# lv_conf.h in a single pass. ha_autopanel is processed AFTER
# lvgl (lvgl is in our DEPENDENCIES), so calling lv_add_define()
# from our to_code() would happen too late - lv_conf.h is already
# written. Module-load is the earliest point the import system
# gives us, which is well before any to_code() runs.
lv_add_define("LV_USE_FLEX", "1")

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(HaAutoPanel),
        cv.Required("ha_api_url"): cv.url,
        # ha_api_password is now optional. The user might be still
        # setting up their token, or the device might be running
        # purely as a read-only display before they wire it to HA.
        # The C++ side already checks for an empty string and
        # gracefully degrades (auth-required features get skipped;
        # fetch_home_name_() exits early; the panel shows
        # SETUP_REQUIRED if HA calls would be made).
        cv.Optional("ha_api_password", default=""): cv.string,
        cv.Optional("http_request_ref"): cv.use_id(http_request.HttpRequestComponent),
        # Optional SNTP-backed time component. When set, the title bar's
        # clock + the debug panel's Date/Time section read from it.
        # The user's yaml must declare one (see test_dynamic_component.yaml).
        cv.Optional("time_ref"): cv.use_id(time.RealTimeClock),
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
        # AGENT_DEBUG: opt-in switch that exposes /autopanel/test/* HTTP
        # endpoints (click, scroll, cmd, state). Default OFF so a
        # production build does not allow an attacker on the same LAN
        # to drive the panel, dump state, or trigger discovery by
        # spoofing a few GET requests. Test/CI builds (e.g.
        # test_dynamic_component.yaml) set this to true. There is no
        # auth on the test endpoints by design - they are only useful
        # in a controlled test environment, and adding a key check
        # would make the test harness more brittle without raising
        # the security bar meaningfully (an attacker on the LAN with
        # the same WiFi key can already see the panel's full state).
        cv.Optional("agent_debug", default=False): cv.boolean,
        # v1.22s: title bar time format. False (default) keeps the
        # 12-hour "10:52 PM" style; True renders 24-hour "22:52".
        # The 12-hour default matches the v1.22r screenshot the user
        # signed off on; 24h is opt-in.
        cv.Optional("use_24h_time", default=False): cv.boolean,
        # v1.22s: show/hide the title bar time label entirely. Default
        # True (visible). Set False to keep the title bar minimal -
        # useful for digital photo-frame / kiosk style deployments
        # where the wall-clock isn't relevant.
        cv.Optional("show_time", default=True): cv.boolean,
        # v1.22s: HA weather entity id to fetch for the title bar
        # weather label. Default "weather.home" matches HA's
        # auto-created weather entity. Set to "" to disable the
        # weather label entirely (the label is hidden until a
        # successful fetch anyway, so a bad/missing entity_id is
        # silently no-op rather than an error spam).
        cv.Optional("weather_entity_id", default="weather.home"): cv.string,
        # v1.22w: subscription scope. The "all" mode subscribes
        # to every entity + 5s bulk poll - the v1.22u behavior.
        # "none" disables all subscriptions (rely on the bulk
        # poll). "per_room" subscribes only to entities in the
        # currently-displayed room + global media_players, with
        # maybe_poll_current_room_states_() running a 3s per-room
        # poll.
        #
        # v1.22w: default CHANGED from "all" to "per_room". The
        # v1.22v "all" default allocates ~200+ std::function
        # callbacks in a row (one per subscribe_home_assistant_state
        # call) during a single trigger_subscription() invocation.
        # When the C6 SDIO is wedged (the priority-inversion
        # deadlock documented in
        # [[project_crowpanel_sdio_is_symptom]]), the heap is
        # too fragmented to hold all of them and one allocation
        # throws std::bad_alloc. -fno-exceptions turns the throw
        # into a direct abort() at PC 0x480dbxxx. The first live
        # evidence was at 12:21:05.762 on the user's panel -
        # see [[feedback_yaml_lambda_std_function_throw]].
        # "per_room" caps the count at ~5 callbacks (the lights
        # in the visible room + media_players), so the abort
        # path is closed for typical use. See
        # .claude/plans/greedy-discovering-koala.md for the
        # full v1.22v plan.
        cv.Optional("subscribe_mode", default="per_room"): cv.one_of(
            "all", "none", "per_room"
        ),
        # v1.22v: WLED-pattern stuck-task recovery knob. When
        # true, the stuck-task detector also takes corrective
        # action (drop httpd worker priority, then C6 reset).
        # When false (default), the detector is pure
        # observability - the user can see the stuck state in
        # the log without the detector doing anything
        # destructive. Per the user's note: "I am providing
        # it as another 'knob' we can turn and monitor" - the
        # user wants visibility first, recovery second.
        cv.Optional("enable_stuck_task_recovery", default=False): cv.boolean,
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    # NOTE: add_lv_use() is intentionally not called here because it runs
    # too late to affect lv_conf.h generation. Widgets used by the C++ code
    # must be declared in the user's YAML under lvgl.widgets:.

    # The LV_USE_FLEX define is set at module load time (see the top
    # of this file) so it's in CORE.data before lvgl's to_code()
    # generates lv_conf.h. Adding it here would be too late.

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

    # v1.24: ESP-IDF's esp_websocket_client. We open a raw WebSocket
    # to Home Assistant at <api_url> converted to ws:// + /api/websocket,
    # authenticate with the long-lived access token, and use HA's
    # get_states + subscribe_events messages for state sync. This
    # REPLACES the v1.22w esphome-native-API subscribe_home_assistant_state
    # path (which triggered PC 0x480dxxxx abort ~300ms after Panel READY
    # due to std::function allocation under heap pressure). The WS path
    # does no std::function allocations in the httpd worker.
    add_idf_component(
        name="espressif/esp_websocket_client",
        ref="~1.4.0",
    )

    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    # Get the http_request component by its ID
    if "http_request_ref" in config:
        http_request_var = await cg.get_variable(config["http_request_ref"])
        cg.add(var.set_http_request(http_request_var))

    # Get the SNTP time component (optional - the title bar shows
    # "--:--" until this is wired up). The user's YAML must declare
    # a `time:` block with a `time_ref` here.
    if "time_ref" in config:
        time_var = await cg.get_variable(config["time_ref"])
        cg.add(var.set_time(time_var))

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
    # AGENT_DEBUG: see schema comment. Off by default for production
    # safety. Test yaml flips it on.
    cg.add(var.set_agent_debug(config["agent_debug"]))
    # v1.22s: title bar time + weather knobs.
    cg.add(var.set_use_24h_time(config["use_24h_time"]))
    cg.add(var.set_show_time(config["show_time"]))
    cg.add(var.set_weather_entity_id(config["weather_entity_id"]))
    # v1.22v: subscription scope (per-room poll opt-in).
    cg.add(var.set_subscribe_mode(config["subscribe_mode"]))
    # v1.22v: stuck-task recovery knob (default OFF - the user
    # wants visibility first, recovery second).
    cg.add(var.set_enable_stuck_task_recovery(config["enable_stuck_task_recovery"]))
