#pragma once

#include <string>
#include <string_view>
#include <vector>
#include <deque>
#include <memory>
#include <map>
#include <set>
#include <utility>
#include <functional>
#include "esphome/core/component.h"
#include "esphome/components/lvgl/lvgl_proxy.h"
#include "esphome/components/http_request/http_request.h"
#include "esphome/components/wifi/wifi_component.h"
#include "esphome/components/web_server_base/web_server_base.h"
#include "esphome/components/time/real_time_clock.h"
#include "template_api.h"  // v1.27: TemplateApi (render() + subscribe() to HA)
#include "esp_http_server.h"  // for AsyncWebHandler / AsyncWebServerRequest
#include "esphome/core/preferences.h"
#include "esp_heap_caps.h"
#include "freertos/task.h"   // uxTaskPriorityGet + eTaskGetState for the task monitor
#include "esp_task.h"        // pcTaskGetHandle for the task monitor

// Raw WebSocket-to-Home-Assistant client. The full definition
// is in ha_ws_client.h, which is included from ha_autopanel.cpp
// (NOT this header) because the .h-inline method bodies need
// HaAutoPanel to be complete. Forward-declared here so
// unique_ptr<HaWsClient> ws_client_ can be declared.
// ~HaAutoPanel() is also declared here and defined in the .cpp
// where HaWsClient is complete (PIMPL idiom).
namespace esphome::ha_autopanel {
class HaWsClient;  // forward decl
}

// ----- v1.27: set_label_text_if_changed -----
//
// Replaces the pattern of: snprintf(buf, ...); lv_label_set_text(lbl, buf);
// at every "update a label" site. The early-out is a 1-line no-op when
// the text is already current, which is the common case for snprintf
// calls that produce the same value 60x/second (clock, brightness %).
// We compare against lv_label_get_text (LVGL 9 API) so the helper is
// always correct even if the caller hasn't kept a local copy.
//
// Place this BEFORE the class declaration so it's visible to the
// templates; it can also be called as set_label_text_if_changed(lbl, s)
// from anywhere inside the namespace.
namespace esphome {
namespace ha_autopanel {
inline void set_label_text_if_changed(lv_obj_t* label, const std::string& text) {
  if (label == nullptr) return;
  const char* cur = lv_label_get_text(label);
  if (cur != nullptr && std::strcmp(cur, text.c_str()) == 0) return;
  lv_label_set_text(label, text.c_str());
}
}  // namespace ha_autopanel
}  // namespace esphome

namespace esphome {
namespace ha_autopanel {

// ----------------------------------------------------------------------------
// Memory infrastructure
// ----------------------------------------------------------------------------

// STL allocator that uses heap_caps_malloc_prefer(PSRAM, INTERNAL).
// On boards with PSRAM (Freenove N16R8V, ESP32-P4, etc.) the storage
// lands in the 8MB+ OPI/QSPI PSRAM. On boards without PSRAM it falls
// back to internal SRAM - same call site, no branch. This means the
// same binary works on both board types without #ifdefs.
//
// Without this, std::vector<Entity> on a small-heap board (ESP32-S3
// ~384KB internal) crashes during push_back when a single room has
// many entities (Garage in the user's HA install: 138 entities).
// log2(N) reallocations fragment the heap; reserve() fixes the
// churn (see filter_and_build_room_cards_) and this allocator
// moves the storage out of the precious internal heap entirely.
//
// ESPHome builds with -fno-exceptions, so we cannot throw
// std::bad_alloc on OOM. Instead we log via ESP_LOGE and abort();
// the panic gets a backtrace pointing here, which is much more
// useful than a later null-deref inside std::vector.
template <typename T>
struct PsramStlAllocator {
  using value_type = T;

  T *allocate(size_t n) __attribute__((nothrow)) {
    if (n == 0) return nullptr;
    if (n > (SIZE_MAX / sizeof(T))) {
      ESP_LOGE("psram_alloc", "PsramStlAllocator: overflow n=%zu * sizeof=%zu",
               n, sizeof(T));
      abort();
    }
    void *p = heap_caps_malloc_prefer(
        n * sizeof(T),
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT,
        MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (p == nullptr) {
      ESP_LOGE("psram_alloc", "PsramStlAllocator: OOM n=%zu * sizeof=%zu (no PSRAM and no internal free)",
               n, sizeof(T));
      abort();
    }
    return static_cast<T *>(p);
  }

  void deallocate(T *p, size_t /*n*/) noexcept {
    if (p != nullptr) heap_caps_free(p);
  }

  // C++17 allocator-awareness: vector<Pair, Alloc> needs rebind to
  // vector<Value, Alloc> when constructed from a different element type.
  template <typename U>
  struct rebind { using other = PsramStlAllocator<U>; };

  // Stateless - all PsramStlAllocator<T> instances are interchangeable.
  bool operator==(const PsramStlAllocator &) const noexcept { return true; }
  bool operator!=(const PsramStlAllocator &) const noexcept { return false; }
};

// Convenience aliases. std::vector<Entity, PsramStlAllocator<Entity>>
// is verbose enough to obscure intent at the call site; psram_xxx
// is shorthand and is what most of the project uses for new code.
// The aliases below are NOT used by PsramStlAllocator itself; they
// are the recommended spelling for downstream container types so
// the PSRAM-routing decision is visible at the type level.
template <typename T>
using psram_vector = std::vector<T, PsramStlAllocator<T>>;
using psram_string = std::basic_string<char, std::char_traits<char>,
                                       PsramStlAllocator<char>>;
template <typename T>
using psram_set = std::set<T, std::less<>, PsramStlAllocator<T>>;
template <typename K, typename V>
using psram_map = std::map<K, V, std::less<>,
                           PsramStlAllocator<std::pair<const K, V>>>;

// Heap-allocate a psram_string in PSRAM, forwarding the
// constructor args. Use with destroy_psram_string() to
// avoid the global operator delete (which would call free()
// on a pointer heap_caps_malloc'd - usually fine, but the
// explicit destroy is safer and matches the allocator).
//
// The site pattern is "store a string in LVGL user_data,
// free it on LV_EVENT_DELETE". Typical payloads are
// entity_ids (~17 chars) or short packed buffers
// ("domain\0entity_id"). All fit the libstdc++ SSO
// threshold of 22 chars, so the psram_string object and
// its payload both live in PSRAM with no second alloc.
inline psram_string *make_psram_string() {
  PsramStlAllocator<psram_string> alloc;
  psram_string *p = alloc.allocate(1);
  new (p) psram_string();
  return p;
}
template <typename... Args>
psram_string *make_psram_string(Args &&...args) {
  PsramStlAllocator<psram_string> alloc;
  psram_string *p = alloc.allocate(1);
  new (p) psram_string(std::forward<Args>(args)...);
  return p;
}
inline void destroy_psram_string(psram_string *p) {
  if (p == nullptr) return;
  p->~basic_string();
  PsramStlAllocator<psram_string> alloc;
  alloc.deallocate(p, 1);
}

// Stable-address string arena. Entity's string fields are
// std::string_view (16 B each) pointing into this arena instead of
// std::string (32 B each, 5x = 160 B). For 419 entities the savings
// are ~30 KB on a 384 KB heap.
//
// Stability: std::deque<std::string> is a sequence of fixed-size
// chunks. push_back only adds new chunks; existing elements are not
// moved. std::string::data() is stable after construction (SSO buffer
// is inline, heap-allocated payload is freed only with the string).
// Therefore string_views returned from intern() stay valid for the
// component's lifetime. Each std::string itself owns its own
// allocation (or uses SSO) - this arena doesn't try to share the
// payload, it just gives Entity cheap reference storage.
class StringArena {
 public:
  StringArena() {
    // No-op. std::deque grows in chunks (~512 elements by
    // default in libstdc++); references to existing elements
    // are stable across emplace_back(), so the intern() function
    // can hand out string_views safely for the arena's whole
    // lifetime. (The previous std::vector used reserve(2048)
    // to delay reallocation; std::deque doesn't need that.)
  }

  // Intern a C string + length. Returns a view into arena-owned
  // storage; the view remains valid until the arena is destroyed
  // (component lifetime).
  std::string_view intern(const char *s, size_t n) {
    if (s == nullptr || n == 0) return std::string_view();
    // Special case: empty string is common (e.g. entities with no
    // icon). Hand back a view of a static "" to avoid burning arena
    // slots on duplicates.
    if (n == 0) return std::string_view("", 0);
    // Construct the std::string in the vector. The vector was
    // reserve()d in the ctor, so emplace_back is a pointer
    // bump and string addresses stay stable. If the string
    // payload is short (entity_id ~17 chars, domain ~6, name
    // ~12, area_id ~10) SSO keeps it inline in the struct;
    // longer strings fall back to a separate heap alloc but
    // the std::string itself is still stable in the vector.
    strings_.emplace_back(s, n);
    return std::string_view(strings_.back().data(), n);
  }

  std::string_view intern(const std::string &s) {
    return intern(s.data(), s.size());
  }

  std::string_view intern(const char *s) {
    return intern(s == nullptr ? "" : s, s == nullptr ? 0 : strlen(s));
  }

  // Intern a string_view directly (for substr() / substr(0, dot) etc.).
  // Implementation note: substr() on a std::string returns a new
  // std::string, but on a std::string_view it returns another
  // string_view into the same backing. We need a null-terminated
  // copy, so we round-trip through std::string. This is a single
  // allocation per intern() call; the std::string is constructed
  // in-place in the deque slot and the allocation cost is
  // amortized to one heap_malloc per unique string.
  std::string_view intern(std::string_view v) {
    if (v.empty()) return std::string_view();
    strings_.emplace_back(v.data(), v.size());
    return std::string_view(strings_.back().data(), v.size());
  }

  // For diagnostics.
  size_t size() const { return strings_.size(); }
  size_t bytes() const {
    size_t total = 0;
    for (const auto &s : strings_) total += s.size();
    return total;
  }

 private:
  // v1.28: PSRAM-backed vector (reserved upfront to avoid
  // realloc). The previous std::deque<std::string> with the
  // default allocator was throwing std::bad_alloc on
  // fragmented internal heap when the deque grew - the throw
  // bubbled to cxx_exception_stubs which assert()s and
  // reboots. With 32MB of PSRAM available, the vector's
  // backing array (and the std::string SSO buffers inside
  // it) live in PSRAM. reserve(2048) covers ~415 entities *
  // 4-5 strings each (entity_id, domain, name, area_id)
  // with headroom. If we ever exceed 2048, the realloc will
  // move all elements and invalidate the string_views.
  // std::string SSO keeps entity_id (~17 chars), domain
  // (~6), name (~12), area_id (~10) all in the struct, so
  // no separate string-allocator allocs happen either.
  //
  // v1.28 fix: was std::vector; switched to std::deque.
  // std::vector's reallocation moves every std::string in
  // the container to a fresh heap allocation, which transfers
  // each string's payload pointer. Every string_view
  // previously returned by intern() (and stored in
  // Entity::entity_id / .domain / .name / .area_id, or
  // RoomAggregate::Entry's eid, or agg.entities keys) was
  // pointing at the OLD allocation and silently dangles the
  // moment the next growth boundary fires. The 2048-element
  // ctor reserve() set the bar, but 419 entities * 4-5 strings
  // each (~1800-2100) blew past it during fetch_areas_(), so
  // by sync_entities_from_aggregates_() the e.domain views
  // were already pointing at freed PSRAM. std::deque's
  // chunked growth never invalidates references to existing
  // elements, which is what the long-standing comment two
  // paragraphs up already said ("std::deque<std::string>").
  //
  // Note on the allocator: the deque's TEMPLATE allocator arg
  // is std::allocator<std::string>, NOT
// PsramStlAllocator<std::string>. libstdc++ has a known
// issue with stateful allocators and deque
// (bits/stl_deque.h's _Map_alloc_type rebind fails to
// construct with the stateful allocator's extra state
// - "too many initializers for _Map_alloc_type" error).
// The std::string PAYLOADS therefore use std::allocator<char>
// (the default); short strings (~22 chars or fewer) live in
// the SSO buffer inside the std::string struct, and longer
// strings (>22 chars) go to internal RAM via the default
// malloc. This is acceptable for typical entity_ids / room
// names / area_ids which all fit in SSO; very long names
// (a description, an attribute value) would fragment the
// internal heap. Only the deque's internal chunk-pointer
// array goes to internal RAM regardless, which is a few
// KB max.
  std::deque<std::string, std::allocator<std::string>> strings_;
};

// One global arena, owned by the singleton instance. The Entity
// string_views are valid as long as this arena is, which is the
// lifetime of the HaAutoPanel instance (and the component lives
// for the device's uptime).
inline StringArena &entity_arena() {
  static StringArena arena;
  return arena;
}

// Area structure from HA
struct Area {
  std::string area_id;
  std::string name;
  std::vector<std::string> entity_ids;  // Entity IDs in this area
};

// Entity structure from HA.
//
// The five string fields are std::string_view (16 B each) backed by
// entity_arena(). This shrinks Entity from ~168 B to ~88 B and keeps
// all the heavy string payloads in a single arena container. The
// state field stays std::string because it changes during runtime
// (HA pushes state updates; arena_emplace would invalidate any
// existing string_view pointing at the same Entity).
struct Entity {
  std::string_view entity_id;
  std::string_view name;
  std::string_view domain;
  std::string_view area_id;
  std::string_view icon;
  // HA's friendly_name from the entity attributes. Populated
  // from the per-area aggregate template (state_attr's
  // friendly_name); empty when HA doesn't expose one.
  // Display sites prefer this over `name` (which is the
  // entity_id-derived placeholder) when it's non-empty.
  std::string_view friendly_name;
  // v1.28: psram_string. The state field is the only one that
  // mutates at runtime (HA pushes state changes via the WS
  // subscription; aggregate push updates also write here).
  // With 419 entities, the default std::string payloads were
  // the largest source of internal-RAM fragmentation during
  // heavy state-update bursts - every state push allocated
  // (or SSO'd) into internal heap, and the OOM aborted
  // reached cxx_exception_stubs. psram_string routes the
  // payload to PSRAM via PsramStlAllocator<char>. The
  // operator==(const char*), operator=(const char*),
  // operator=(const std::string&), c_str(), empty() etc.
  // surface is identical to std::string, so all the
  // existing read/write sites (e.state == "on", e.state =
  // "on", e.state = new_state, etc.) compile unchanged.
  psram_string state;        // "on" / "off" / etc. - mutates at runtime
  uint8_t brightness{0};    // 0-255, valid only when state == "on" and has_brightness
  bool has_brightness{false};
  bool is_hue_group{false};
  // Set by the room-card click / arc-drag handlers when they
  // optimistically write a new state. While true,
  // sync_entities_from_aggregates_() skips this entity so the
  // render_template aggregate push (which may have been
  // generated BEFORE HA processed the user's turn_off/toggle)
  // can't overwrite the optimistic "off" and cause a flicker
  // back to the old "on" state for a frame. Cleared in
  // on_entity_state_changed_() when the matching state event
  // arrives, and also by a 10s timeout in loop() so an HA
  // outage doesn't leave the entity stuck-out-of-sync forever.
  bool pending_optimistic_update{false};
};

// Room card data for UI
struct RoomCard {
  Area area;
  std::vector<Entity, PsramStlAllocator<Entity>> entities;
  int grid_index{0};
  int x{0};
  int y{0};
  uint32_t color;
};

// v1.27: per-room aggregate produced by HA's render_template
// subscription. Populated by apply_room_aggregates_() from
// the JSON HA pushes when any of the panel's entities
// changes state (or on the initial one-shot fetch). The
// panel reads from room_aggregates_ during sync to update
// the Entity records in entities_by_area_ in place, so the
// room cards pick up the new state without a full re-render.
//
// We key entities by string_view into the entity_arena()
// (same trick as Entity::entity_id) so the per-entity
// entries are just 16 B per entity. For 419 entities the
// aggregate is ~12 KB on the heap - much smaller than the
// 200KB+ get_states response it replaces.
struct RoomAggregate {
  // v1.28: psram_string for the three string fields. The
  // outer room_aggregates_ map (key=std::string) is still
  // std::string-keyed so cross-map lookups via rkv.first
  // against entities_by_area_ (also std::string-keyed)
  // don't need a conversion. But the *payload* strings
  // here are the ones that grow: state values can be any
  // string from HA (some sensors report long state
  // strings, attributes arrive as JSON blobs, etc.). When
  // any of these exceeds the libstdc++ SSO threshold of
  // ~22 chars the std::string default allocator routes
  // the buffer to internal RAM and operator[]'s throw
  // reaches cxx_exception_stubs. The psram_string variant
  // routes the same buffer to PSRAM via
  // PsramStlAllocator<char>::allocate, so the entire
  // payload (object + buffer) lands in the 32MB OPI
  // PSRAM and the internal-heap pressure that was
  // triggering the abort is gone.
  //
  // Note: psram_string and std::string both expose the
  // same operator==(const char*) and operator=(const char*)
  // surface that the call sites use, so the assignment
  // sites in apply_room_aggregates_() and the read sites
  // in sync_entities_from_aggregates_() don't need any
  // changes - they pass through the standard
  // basic_string interface.
  psram_string area_id;
  psram_string name;
  // Per-entity state snapshot. Populated from the
  // aggregate's "states" map. The Entry holds a copy of
  // the state string (it's mutable at runtime), the raw
  // brightness 0-255, and the friendly_name string (set
  // once at discovery, intern'd into entity_arena()).
  struct Entry {
    psram_string state;
    uint8_t brightness{0};
    bool has_brightness{false};
    // friendly_name from HA's entity attributes; empty when
    // unset. Intern'd into entity_arena() so the view stays
    // valid for the component's lifetime.
    std::string_view friendly_name;
  };
  // string_view into entity_arena() intern'd entity_ids.
  // std::less<> (transparent) enables heterogeneous lookup
  // against Entity::entity_id without allocating a
  // std::string key.
  // v1.28: PSRAM-backed. The default std::allocator was throwing
  // std::bad_alloc on fragmented internal heap when operator[]
  // created a new tree node (the throw bubbled to
  // cxx_exception_stubs and rebooted the P4). The outer
  // room_aggregates_ map is also PSRAM-backed; this completes
  // the chain so ALL map nodes for the aggregate live in PSRAM.
  std::map<std::string_view, Entry, std::less<>,
           PsramStlAllocator<std::pair<const std::string_view, Entry>>>
      entities;
};

// Per-arc registry entry - replaces heap-allocated ArcCallbackData.
// Lives for the component lifetime; index is stable per render but the
// vector is cleared at the top of each create_*_ call that appends to it.
//
// string_view fields are populated directly from Entity fields
// (which are also string_views into entity_arena()) - the
// assignment is a bitwise pointer copy, no allocation. The views
// are valid as long as the arena is (component lifetime).
struct ArcRecord {
  std::string_view entity_id;   // empty for room-level arc
  std::string_view area_id;     // always populated
  lv_obj_t* pct_label{nullptr};  // the %-label widget, may be null (room arc)
  bool is_room_arc{false};  // true = room-level big 240px arc
};

// Per-button/toggle registry entry.
struct ControlRecord {
  std::string_view entity_id;       // empty for room-level controls
  std::string_view area_id;         // empty for per-entity controls
  std::string_view domain;          // "light", "switch", etc.
  lv_obj_t* btn{nullptr};
  lv_obj_t* state_label{nullptr};
};

// Persisted config struct (NVS). Char arrays, not std::string, so the
// type is trivially_copyable (required by global_preferences).
// Note: hidden_rooms / hidden_entities / room_order / entity_order
// are stored separately as a JSON blob (CustomizationConfig) on
// LittleFS, NOT in this struct, because the size would blow past
// the NVS blob limit (508 bytes).
struct StoredConfig {
  char api_url[128];
  char api_token[256];
  bool configured;
  uint8_t reserved[7];  // pad to 4-byte alignment for NVS
};
static_assert(sizeof(StoredConfig) <= 512, "StoredConfig must fit in a single NVS blob");

// User customizations persisted as JSON on LittleFS at
// /storage/customizations.cfg. Plain key-value fields, not NVS.
//
// All containers here are PSRAM-backed (the vector/set/map
// storage is in PSRAM). The string elements stay std::string
// (default allocator) because they fit the SSO threshold
// (~22 chars) for typical room/entity IDs, and because Area
// also uses std::string - mixing the two in the customization
// struct (e.g. hidden_rooms vs Area::name) would require
// per-call conversions. The container allocator handles the
// bigger win: keeping the tree nodes and vector array in
// PSRAM so re-discovery doesn't fragment internal RAM.
struct CustomizationConfig {
  psram_set<std::string> hidden_rooms;
  psram_set<std::string> hidden_entities;
  psram_vector<std::string> room_order;
  // entity_order is per-area, keyed by area_id. Outer map
  // nodes and inner vector's array are in PSRAM; the strings
  // themselves stay std::string (SSO covers entity_ids).
  std::map<std::string, psram_vector<std::string>, std::less<>,
           PsramStlAllocator<std::pair<const std::string,
                                       psram_vector<std::string>>>>
      entity_order;
  bool loaded{false};
};

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

// v1.30: state machine for incremental UI build. Was synchronous
// (create_ui_from_room_cards_() built all 15 cards in one tight
// loop), which exhausted internal SRAM via per-card LVGL flushes
// -> SPI DMA buffer allocs -> fragmented heap -> post-burst
// cascade failures (event loop queue, esp-tls sockets, more DMA
// allocs all failed). Deferring one card per loop tick gives the
// SPI driver time to free buffers between cards, and gives the
// main loop time to service other subsystems (WiFi, esp-tls,
// mDNS, http_request).
enum class UIBuildState {
  IDLE,            // not building
  BUILDING_CARDS,  // create one card per loop tick
  FINALIZING,      // sizing + completion log, then IDLE
};

class HaAutoPanel : public Component {
 public:
  // Declared here (defaulted in ha_autopanel.cpp) so
  // unique_ptr<HaWsClient> ws_client_ is destructed where the
  // full HaWsClient type is visible (PIMPL idiom for
  // forward-declared unique_ptr members).
  ~HaAutoPanel();
  // HaWsClient (the raw WebSocket-to-HA client) needs
  // access to entities_by_area_, on_entity_state_changed_, and
  // on_entity_attribute_changed_, all of which are protected
  // below. Friend is the surgical way to expose them without
  // making them public to the rest of the codebase.
  friend class HaWsClient;

 public:
  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::AFTER_WIFI; }

  // Boot counter preference. Each setup() reads the prior value,
  // increments, and saves - so the gap between consecutive saved
  // values reveals unclean restarts. See setup() for the
  // reset-reason logging that pairs with this.
  ESPPreferenceObject pref_ = global_preferences->make_preference<uint32_t>(0xA0B00700);

  // Called from YAML lambda to trigger discovery after HA connects
  void trigger_discovery();

  // Called from YAML lambda to (re-)subscribe to state changes. Safe to call
  // multiple times. No-op if the API isn't connected yet or if there's
  // nothing to subscribe to.

  // Called from YAML on_client_connected to fire a single probe that
  // checks whether HA is allowing this device to call services. If the
  // probe times out, the panel transitions to NOT_AUTHORIZED. If the
  // probe gets a response, the panel stays in READY (or transitions to
  // it once rooms are rendered).
  void trigger_auth_probe();

  void set_ha_api_url(const std::string& url) { this->ha_api_url_ = url; }
  void set_ha_api_password(const std::string& password) { this->ha_api_password_ = password; }
  void set_http_request(http_request::HttpRequestComponent* http) { this->http_request_ = http; }
  // Optional SNTP time source. When set, the title bar's clock and
  // the debug panel's Date/Time section read from this. When null,
  // those widgets show "--:--" / a placeholder.
  void set_time(time::RealTimeClock* t) { this->time_ = t; }
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
  // AGENT_DEBUG opt-in switch. When true, /autopanel/test/* HTTP
  // endpoints are registered (click, scroll, cmd, state). Default
  // false so a production build does not expose them. See
  // __init__.py for the full security rationale.
  void set_agent_debug(bool v) { this->agent_debug_ = v; }
  // Title bar time format. False = 12-hour "10:52 PM";
  // True = 24-hour "22:52". Wired into update_title_time_().
  void set_use_24h_time(bool v) { this->use_24h_time_ = v; }
  // Title bar time visibility. False hides the time
  // label entirely (kiosk / digital-frame mode); the right
  // cluster auto-collapses to just the DBG button. Wired in
  // create_title_bar_().
  void set_show_time(bool v) { this->show_time_ = v; }
  // HA weather entity id for the title bar weather
  // label. Default "weather.home". Empty string disables
  // fetch_weather_() entirely.
  void set_weather_entity_id(const std::string& id) { this->weather_entity_id_ = id; }

 protected:
  std::string ha_api_url_;
  std::string ha_api_password_;
  http_request::HttpRequestComponent* http_request_{nullptr};
  // Optional time source - set by the YAML's `time_ref` (via the
  // set_time setter) when the user wires up a SNTP block.
  time::RealTimeClock* time_{nullptr};
  // Cached friendly name of HA's zone.home entity, shown centered in
  // the title bar on the grid page. Populated by fetch_home_name_().
  std::string home_name_;
  // Wall-clock timestamp (millis()) of the last successful home-name
  // fetch. Used to throttle periodic refresh in loop() and to skip
  // redundant work when a state change re-triggers the fetch.
  uint32_t last_home_fetch_ms_{0};
  bool include_all_{true};
  std::vector<std::string> include_areas_;
  std::vector<std::string> exclude_areas_;
  std::vector<std::string> entity_domains_{"light"};
  std::vector<std::string> exclude_entities_;
  // Auto-fit square-card layout. card_width is both the width and height
  // (square cards). cards_per_row is computed from screen_width and
  // card_gap. main_container_ height is computed from card count.
  int card_width_{250};
  // card_height is the card's vertical extent. For the current
  // 7" Crowpanel design card_height_ == card_width_ (square
  // cards). The arc_size_() formula uses min(w,h) so a future
  // rectangular card (e.g. 4" 480x320) just works - the arc
  // will be sized off the smaller dimension.
  int card_height_{250};
  // Parametric arc size - 96% of the smaller card dimension
  // with a 2% padding on each side. For 250x250 that's 240.
  // The 2% padding means the arc leaves a visible gap on all
  // sides (the user's "breathing room" request). 96% chosen so
  // the arc and button together fit a 250x250 card with the
  // button pinned to the bottom (250 - 240 = 10px gap below
  // the arc, plus the button is 32px tall with 8px bottom
  // margin).
  int arc_size_() const { return (std::min(this->card_width_, this->card_height_) * 96) / 100; }
  int card_gap_{12};
  int screen_width_{1024};
  int screen_height_{600};
  int start_x_{10};
  int start_y_{12};
  int default_on_pct_{30};

  // v1.28: PSRAM-backed. ~15 entries, grown during fetch_areas_().
  std::vector<Area, PsramStlAllocator<Area>> discovered_areas_;
  // entities_by_area_ uses the PSRAM-preferring allocator so all
  // Entity storage (the bulk of ha_autopanel's RAM) lands in PSRAM
  // on boards that have it, and falls back to internal SRAM on
  // boards that don't. The Entity structs themselves use
  // string_view fields pointing into entity_arena(), so the
  // std::string payloads share a single arena container.
  // v1.28: also PSRAM the outer map (inner vector's allocator
  // v1.28: PSRAM-backed outer map. The inner vector's
  // allocator MUST be the default std::allocator to satisfy
  // the libstdc++ "value_type matches allocator" static_assert
  // (the map's value_type and the allocator's value_type must
  // be EXACTLY the same type - they can't differ in inner
  // template arguments). So the inner vector uses default
  // alloc (its strings are short enough for SSO anyway, so
  // no separate heap alloc).
  std::map<std::string, psram_vector<Entity>, std::less<>,
           PsramStlAllocator<std::pair<const std::string, psram_vector<Entity>>>>
      entities_by_area_;
  // v1.28: PSRAM-backed. ~15 entries, each ~80B.
  std::vector<RoomCard, PsramStlAllocator<RoomCard>> room_cards_;

  // v1.27: per-room aggregate from the render_template
  // subscription. Keyed by area_id (the same key as
  // entities_by_area_). The aggregate holds per-entity
  // state snapshots; sync_entities_from_aggregates_()
  // copies them back into Entity records and repaints the
  // room cards.
  // v1.28: PSRAM-backed map. The default std::allocator was
  // throwing std::bad_alloc on fragmented internal heap when
  // operator[] created a new tree node - the throw bubbled to
  // cxx_exception_stubs and rebooted the P4. With 32MB of
  // PSRAM, the tree nodes land there instead.
  std::map<std::string, RoomAggregate, std::less<>,
           PsramStlAllocator<std::pair<const std::string, RoomAggregate>>>
      room_aggregates_;

  // v1.27: the per-area aggregate template body. Built
  // once by build_room_aggregate_template_() after
  // fetch_areas_() populates the area list. R"DELIM(...)
  // raw string syntax means the Jinja2 template is pasted
  // verbatim with no escape sequence hell.
  std::string room_aggregate_template_;

  // v1.27: render-time variables for the per-area
  // aggregate template. {"included_areas": ["kitchen",
  // "living_room", ...]} - a JSON array of the
  // post-include/exclude-filter area ids from
  // discovered_areas_.
  std::string room_aggregate_variables_;

  // UI state - stored for navigation between room grid and entity detail
  lv_obj_t* main_container_{nullptr};
  lv_obj_t* detail_container_{nullptr};
  int current_room_index_{-1};

  // Per-render registries (component-owned, no heap, no LV_EVENT_DELETE)
  // v1.28: PSRAM-backed. ~15 entries + per-entity entries; could
  // OOM the internal heap on boot-time growth after many rediscovers.
  std::vector<ArcRecord, PsramStlAllocator<ArcRecord>> arc_records_;
  std::vector<ControlRecord, PsramStlAllocator<ControlRecord>> control_records_;

  // Live-update widget maps (keyed by stable string id)
  // v1.28: PSRAM-backed. ~15 entries each at boot.
  std::map<std::string, lv_obj_t*, std::less<>,
           PsramStlAllocator<std::pair<const std::string, lv_obj_t*>>>
      room_arc_widgets_;                  // area_id -> big 240px arc
  std::map<std::string, std::pair<lv_obj_t*, lv_obj_t*>, std::less<>,
           PsramStlAllocator<std::pair<const std::string, std::pair<lv_obj_t*, lv_obj_t*>>>>
      room_btn_widgets_;  // area_id -> (btn, label)
  std::map<std::string, lv_obj_t*, std::less<>,
           PsramStlAllocator<std::pair<const std::string, lv_obj_t*>>>
      room_label_btn_widgets_;            // area_id -> transparent room-name button

  // Bounce-back storage: last non-zero brightness for each area
  // v1.28: PSRAM-backed. ~15 entries, written per user interaction.
  std::map<std::string, uint8_t, std::less<>,
           PsramStlAllocator<std::pair<const std::string, uint8_t>>>
      last_brightness_pct_;

  // Track which entity_ids we've subscribed to (avoids double-subscribe)
  // std::less<> (transparent comparator) enables heterogeneous
  // lookup so we can call .count(e.entity_id) where e.entity_id is
  // a std::string_view without allocating a std::string first.

  // Raw WebSocket-to-HA client. Constructed lazily in
  // start_discovery_() after fetch_areas_() populates the
  // entity_id set; the WS client then opens, authenticates,
  // and subscribes to the clock + per-area aggregate render_template
  // events (which deliver all state changes server-side, no
  // per-entity std::function allocations).
  std::unique_ptr<HaWsClient> ws_client_;

  // v1.27: TemplateApi - one-shot render() and push subscribe()
  // wrappers for HA's /api/template REST endpoint and the
  // WebSocket render_template subscription. Used for the
  // clock (push), weather + home name + per-area aggregate
  // (one-shot). Constructed in setup() with http_request_ and
  // the current URL/token; the WS-send callback is wired in
  // start_discovery_() once ws_client_ exists.
  TemplateApi template_api_{this->http_request_, std::string(), std::string()};

  // Panel state machine
  PanelState state_{PanelState::BOOTING};
  // Boot splash — full-screen black with red centered "HA AutoPanel v1.0".
  // Shown immediately at the start of setup() so the user never sees a
  // bare/white display, then hidden as soon as the status screen or room
  // grid takes over. Doubles as branding and as a cover for the brief
  // display-default artifacts the panel can show before LVGL takes over.
  lv_obj_t* splash_container_{nullptr};
  lv_obj_t* splash_label_{nullptr};
  // Holds the BOOTING / SETUP_REQUIRED / etc. screens
  lv_obj_t* status_container_{nullptr};
  lv_obj_t* status_title_{nullptr};
  lv_obj_t* status_message_{nullptr};
  lv_obj_t* status_retry_btn_{nullptr};

  // Top title bar (HA status + Edit button) on the main grid page.
  // Always visible, room cards scroll under it.
  lv_obj_t* title_bar_{nullptr};
  // Right-side cluster of the title bar - a flex-row sub-container
  // that holds Time, Sort, Cancel, Debug, Edit (in that left-to-right
  // order, with hidden children collapsing out of the layout).
  // Positioned at LV_ALIGN_RIGHT_MID on the title bar with a 12px
  // right margin; the flex layout auto-sizes around whatever
  // children are currently visible, so the whole bar reflows when
  // the user changes screen_width in YAML.
  lv_obj_t* title_right_cluster_{nullptr};
  lv_obj_t* title_status_dot_{nullptr};
  lv_obj_t* title_status_label_{nullptr};
  // title_sort_btn_ (label "Edit") is the only always-visible
  // right-cluster button on the grid. It opens the sort_panel_
  // which handles both reorder and show/hide. title_save_btn_
  // and title_cancel_btn_ are HIDDEN by default and only show
  // when the sort_panel_ is open (so the user can apply or
  // discard their changes via the title bar).
  lv_obj_t* title_sort_btn_{nullptr};  // label "Edit", always visible on the grid
  lv_obj_t* title_save_btn_{nullptr};  // hidden; shown on Edit page
  lv_obj_t* title_cancel_btn_{nullptr};  // hidden; shown on Edit page
  lv_obj_t* title_back_btn_{nullptr};  // shown only on the entity detail page
  lv_obj_t* title_room_label_{nullptr};  // room name in the center of the title bar (detail page only)
  // Version label at the bottom-left of the title bar.
  // Shows the build version (git short hash + build time) so
  // we can verify which build is loaded. Default hidden; the
  // test harness's /autopanel/test/state endpoint reports
  // the same value as 'version=' so the harness can confirm
  // the firmware matches the expected build without flashing.
  lv_obj_t* title_version_label_{nullptr};
  // Reboot button in the title bar. Only created when
  // agent_debug_ is true (so production builds don't have a
  // way for a user to trigger a soft reset). The Crowpanel
  // is battery-powered and the case makes the physical
  // reset button hard to reach, so a soft-reboot path on the
  // panel itself is needed for in-field recovery. The button
  // is RED (destructive) and always visible in agent_debug
  // builds (so the user can reboot without first turning on
  // the test harness).
  lv_obj_t* title_reboot_btn_{nullptr};
  // The version string baked into the build. Set in the .cpp
  // from #define FIRMWARE_VERSION + the __DATE__ / __TIME__
  // macros so every build has a unique fingerprint. The test
  // harness reads this via /autopanel/test/state so a test
  // suite can refuse to run if the device's firmware is older
  // than what the test was written against.
  std::string firmware_version_{};
  // v1.27: removed HA-derived time baseline (parse_iso_to_unix_,
  // set_time_from_iso_, time_unix_seconds_, time_baseline_millis_,
  // time_valid_, maybe_refresh_time_baseline_,
  // last_time_baseline_refresh_ms_, TIME_BASELINE_REFRESH_INTERVAL_MS,
  // last_state_poll_ms_, STATE_POLL_INTERVAL_MS). The title-bar
  // clock now subscribes to {{ now().strftime('%-H:%M') }} via
  // the WS render_template subscription (see
  // setup_render_template_subscriptions_() in this header).
  // HA's template engine handles timezone + DST + format
  // conversion; the panel just stamps the resulting string
  // into the time label. No device-side ISO-8601 parsing,
  // no unix epoch math, no localtime_r/tzset/TZ env vars.
  // HA zone.home friendly_name, fetched at runtime and shown in the
  // center of the title bar on the main grid page. Hidden on the
  // detail page (where title_room_label_ takes that spot) and during
  // the splash / status screens. Cached in home_name_ so the title
  // bar has something to show on the next boot before the network
  // call returns.
  lv_obj_t* title_home_label_{nullptr};
  // Weather label on the LEFT of the title bar. Text-only
  // (e.g. "Cloudy 3°C"). Populated by fetch_weather_() from the
  // HA weather entity (default: weather.home). Hidden until the
  // first fetch completes; if no weather entity is configured,
  // stays hidden.
  lv_obj_t* title_weather_label_{nullptr};
  // Cached "Cloudy 3°C" string. Refreshed on each fetch_weather_().
  // Cached so the title bar has something to show on the next
  // boot before the network call returns.
  std::string weather_text_;
  // HA weather entity id (yaml "weather_entity_id", default
  // "weather.home"). Empty disables fetch_weather_().
  std::string weather_entity_id_{"weather.home"};
  // Wall-clock millis of the last successful weather fetch.
  // Throttles the periodic refresh in loop(). Mirrors
  // last_home_fetch_ms_ for home_name_.
  uint32_t last_weather_fetch_ms_{0};
  // 24h time format flag (yaml "use_24h_time", default
  // false = 12h "10:52 PM"). update_title_time_() reads this.
  bool use_24h_time_{false};
  // Title bar time visibility (yaml "show_time", default
  // true). create_title_bar_() reads this once to set the initial
  // HIDDEN flag; the tap-toggling click handler in the time label
  // is registered regardless so this flag is only the default
  // state, not a runtime override.
  bool show_time_{true};
  // Left cluster (the weather + future left-side widgets).
  // Anchored to the LEFT edge of the title bar with LV_ALIGN_LEFT_MID
  // + a small left padding (8px). Lives in a flex row so adding
  // more left-side widgets (e.g. a status icon next to the weather)
  // is a one-line change.
  lv_obj_t* title_left_cluster_{nullptr};
  // State of the Edit/Reboot button toggle. The time
  // label is the tap target: tapping it shows/hides the Edit
  // and Reboot buttons (which are in the right cluster). Default
  // is hidden so the title bar stays tidy (user request: "the
  // edit and reboot buttons should be hidden").
  bool title_chrome_visible_{false};
  // Current room's area_id (or empty for grid view).
  // Set in show_entity_detail_(); cleared in show_room_grid_().
  std::string current_room_area_id_;
  // Per-room poll throttle + last success ms (kept for backward
  // compatibility; the actual poll code is no longer called -
  // state sync is via the WS render_template subscription).
  uint32_t last_room_poll_ms_{0};
  static constexpr uint32_t ROOM_POLL_INTERVAL_MS = 3 * 1000;  // 3s
  // WLED-style task monitor state. Polls uxTaskGetSystemState()
  // once per second, diffs against the previous snapshot, and
  // logs only meaningful changes (state transitions to/from
  // eBlocked, priority changes, stack HWM drain below 512).
  // High-frequency Ready↔Running flips are filtered to DEBUG.
  // enable_task_monitor_ defaults to true; enable_stuck_task_recovery_
  // is kept for future use (no code path currently consumes it).
  bool enable_task_monitor_{true};
  bool enable_stuck_task_recovery_{false};
  uint32_t last_task_monitor_ms_{0};
  // Per-task snapshot. std::string name would be simpler but
  // adds heap allocation; the pcTaskName pointer is only safe
  // to use while the task is alive, so we look it up fresh from
  // TaskStatus_t each pass.
  struct TaskSnapshot {
    eTaskState state{eBlocked};      // sentinel so first pass always logs
    UBaseType_t priority{0};
    uint16_t stack_hwm{0};
    uint32_t blocked_since_ms{0};
    TaskSnapshot() = default;
  };
  // PSRAM-backed. ~16-20 entries (one per FreeRTOS task),
  // written once per loop iteration.
  std::map<TaskHandle_t, TaskSnapshot, std::less<>,
           PsramStlAllocator<std::pair<const TaskHandle_t, TaskSnapshot>>>
      task_snapshots_;
  // Max recoveries per window before falling back to App.reboot().
  // 3 is enough to give the C6 a fair chance to re-init the
  // SDIO link without looping forever.
  static constexpr uint8_t STUCK_RECOVERY_MAX_PER_WINDOW = 3;
  // Last time the user tapped the time label. Used to debounce
  // rapid taps (so a finger drag across the title bar doesn't
  // accidentally trigger multiple toggles). Set to millis() on
  // each click; only toggle if >250ms since last.
  uint32_t last_title_tap_ms_{0};
 public:
  // Stuck-task recovery knob. When true, future code can call
  // wedge_trigger_c6_reset_() to recover from a C6 wedge. The
  // task monitor is pure observability and never auto-fires the
  // reset. Kept as a callable for future use. Default OFF.
  void set_enable_stuck_task_recovery(bool v) {
    this->enable_stuck_task_recovery_ = v;
  }
  // Task-monitor enable knob. Default true. When false, the
  // monitor stops polling entirely (zero overhead).
  void set_enable_task_monitor(bool v) {
    this->enable_task_monitor_ = v;
  }
  // WLED-style task state monitor. Polls uxTaskGetSystemState()
  // once per second, diffs against the previous snapshot, and
  // logs only meaningful changes (state transitions to/from
  // eBlocked, priority changes, stack HWM drain below 512).
  // High-frequency Ready↔Running flips are filtered to DEBUG.
  // Gives priority-inversion visibility (high-prio task in
  // eBlocked while mid-prio task is eRunning) without the
  // sdio_write "blocked for X ms" noise an SDIO-only check
  // would generate. Called from loop(), throttled to 1 Hz
  // internally.
  void monitor_task_states_();
  // C6 reset via GPIO32 (the esp32_hosted reset_pin). Drive
  // GPIO32 low for WEDGE_C6_RESET_LOW_MS (100ms), release; the
  // C6 re-inits the SDIO link on its next boot. Kept as a
  // callable for future use; the task monitor does not
  // auto-fire it.
  void wedge_trigger_c6_reset_();
  static constexpr uint32_t WEDGE_C6_RESET_LOW_MS = 100;
  // Canonical "stuck-task" threshold (30s). The real wedge is a
  // sustained >30s block (the sdio_rx_get_buffer assert at
  // sdio_drv.c:896 fires after sustained failure to drain the
  // ring buffer). Kept for reference; no code path currently
  // consumes it - any future auto-recovery should use this
  // value.
  static constexpr uint32_t STUCK_TASK_THRESHOLD_MS = 30 * 1000;
 protected:
  // Local clock, populated by the SNTP time component (yaml id
  // `sntp_time`) and refreshed every loop() tick (~1Hz). Sits to
  // the right of the home name, just left of the Edit button.
  // Shows "--:--" until NTP sync succeeds.
  //
  // The time label is CLICKABLE. Tapping it toggles the
  // Edit/Reboot button visibility (see title_chrome_visible_).
  lv_obj_t* title_time_label_{nullptr};
  // "DBG" button in the title bar. Shown when the panel is in any
  // non-READY state (the user needs to diagnose AUTH_FAILED /
  // NOT_AUTHORIZED / SETUP_REQUIRED / CONNECTING).
  // Tapping it opens debug_panel_, the bottom-anchored overlay that
  // shows WiFi / HA / Customizations / Device / Date+Time status and
  // exposes action buttons (re-probe auth, re-run discovery, reset
  // customizations, reboot).
  lv_obj_t* title_debug_btn_{nullptr};
  // Bottom-anchored overlay, ~70% screen height, full width. Created
  // lazily on the first tap of title_debug_btn_ (one-time per
  // session) and rebuilt on every show so live status is always
  // fresh. Hidden by default. Follows the same lazy-create pattern
  // as the sort panel below.
  lv_obj_t* debug_panel_{nullptr};
  // True between the first and second tap of a 2-tap confirm button
  // (reboot or reset). Set to "" / cleared after 5s of inactivity or
  // when the second tap fires. Maps to a colour change on the
  // pending button so the user has a visible cue that a confirmation
  // tap will actually execute the action.
  std::string pending_action_;
  uint32_t pending_action_started_ms_{0};
  static constexpr uint32_t PENDING_ACTION_TIMEOUT_MS = 5000;
  // Sort & hide button in the title bar. Visible only in edit mode.
  // Tapping it opens sort_panel_, a full-screen list view of every
  // room with up/down arrow buttons and a Hide/Show toggle per row.
  // The user sorts and toggles there, then taps Apply to persist to
  // /storage/customizations.cfg and re-render the grid. This is the
  // recommended way to reorder and hide rooms (replaces the
  // older "Show hidden (N)" button + hidden_panel_ + drag-to-reorder
  // gesture paths, which the user found jumpy and didn't save).
  // (title_sort_btn_ is declared earlier in the title-bar widget block
  //  at line ~407. Keep that one and only that one.)
  // Full-screen sort & hide overlay. Created lazily on the first tap
  // of title_sort_btn_, rebuilt every show so live state is current.
  // Lives as a child of the screen. Same lazy-create / rebuild-on-show
  // pattern as debug_panel_.
  lv_obj_t* sort_panel_{nullptr};
  // Local copy of the room order while the sort panel is open. On
  // Apply, this is what we persist to customizations_.room_order.
  // Cleared on close. PSRAM-backed (outer vector array in PSRAM;
  // string elements stay std::string so the assignment at apply
  // time doesn't require per-element conversion).
  psram_vector<std::string> sort_local_order_;
  // Local copy of which rooms are hidden while the sort panel is
  // open. On Apply, this is what we persist to customizations_.
  // hidden_rooms. Cleared on close.
  psram_set<std::string> sort_local_hidden_;
  // Number of rooms currently visible (for the status line)
  int visible_room_count_{0};

  // Active media player banner. A horizontal strip at the top of the
  // grid (just below the title bar) that surfaces any media_player
  // entity currently in 'playing' state. Tapping the banner pauses
  // the media player. Hidden when no media is playing.
  //
  // Design: a single banner row holds up to N "tile" widgets, one
  // per currently-playing media_player. Each tile shows the friendly
  // name + a small pause icon. The whole banner row is hidden
  // (LV_OBJ_FLAG_HIDDEN) when there are no active tiles.
  //
  // The banner's visibility is recomputed from on_entity_state_changed_
  // (real-time, when state-sync works) and from refresh_room_cards_()
  // (initial render, and after hide/reveal).
  //
  // When the state-sync bug is open (state_changed never fires), the
  // banner shows whatever entities have state set at fetch time. This
  // is documented in [[project_state_sync_bug]].
  lv_obj_t* media_banner_{nullptr};
  // Map: entity_id (as string) -> tile widget. Cleared on rebuild.
  // v1.28: PSRAM-backed.
  std::map<std::string, lv_obj_t*, std::less<>,
           PsramStlAllocator<std::pair<const std::string, lv_obj_t*>>>
      media_tiles_;

  // Authorization probe state
  bool auth_probe_pending_{false};
  // trigger_auth_probe() sets this flag from the httpd worker;
  // loop() consumes it. The deferred-trigger pattern avoids
  // calling probe_authorization_() (which allocates a
  // std::function for the response callback) from the httpd
  // worker context where a throw would land in the worker
  // and -fno-exceptions would abort().
  bool pending_auth_probe_{false};
  uint32_t auth_probe_started_ms_{0};

  // v1.30: incremental UI build state. Spreads the 15-card widget
  // creation across multiple loop ticks so the per-card LVGL flushes
  // don't starve the SPI DMA buffer pool and fragment internal
  // SRAM in one tight loop. See continue_ui_build_() in the .cpp.
  UIBuildState ui_build_state_{UIBuildState::IDLE};
  // Cached row metadata for the in-progress build.
  int ui_build_cards_per_row_{0};
  int ui_build_num_rows_{0};
  int ui_build_current_row_{-1};       // -1 = no row started yet
  int ui_build_current_card_in_row_{0};
  int ui_build_current_row_count_{0};
  int ui_build_current_row_width_{0};
  lv_obj_t *ui_build_current_row_container_{nullptr};
  // Throttle for the safety-net force-refresh (see loop()).
  uint32_t last_force_refresh_ms_{0};
  static constexpr uint32_t FORCE_REFRESH_INTERVAL_MS = 5000;
  // 15s: on cold boot the HA native API encryption handshake +
  // first service call can take >5s, causing the probe to time
  // out and the panel to flip to NOT_AUTHORIZED until the user
  // taps Retry. 15s gives the cold-boot path enough headroom
  // while still catching genuine auth failures promptly.
  static constexpr uint32_t AUTH_PROBE_TIMEOUT_MS = 15000;
  static constexpr uint32_t AUTH_PROBE_CALL_ID = 0xA1701ACE;  // unique probe id
  // How many times to auto-retry the probe before declaring
  // NOT_AUTHORIZED. Each retry waits AUTH_PROBE_RETRY_DELAY_MS
  // between attempts. 3 retries at 5s intervals = up to 15s of
  // recovery time before the panel gives up and shows the
  // NOT_AUTHORIZED screen.
  static constexpr int AUTH_PROBE_MAX_RETRIES = 3;
  static constexpr uint32_t AUTH_PROBE_RETRY_DELAY_MS = 5000;
  int auth_probe_retries_left_{AUTH_PROBE_MAX_RETRIES};
  uint32_t auth_probe_next_retry_ms_{0};

  // Persistent config (LittleFS at /storage/autopanel.cfg)
  bool config_loaded_{false};
  std::string config_path_{"/storage/autopanel.cfg"};

  // User customizations (LittleFS at /storage/customizations.cfg).
  // Separate from the API token config because it's larger and more
  // frequently changed.
  CustomizationConfig customizations_;
  std::string customizations_path_{"/storage/customizations.cfg"};

  // Web UI handler
  bool web_handler_registered_{false};
  // Set true after the first successful fetch_areas_() /
  // fetch_room_aggregates_() pass. Used to gate downstream
  // work that depends on entities_by_area_ being populated.
  bool entities_by_area_ready_{false};
  // AGENT_DEBUG opt-in. When true, /autopanel/test/* handlers are
  // registered and respond. Default false (set in the schema), so
  // production builds do not expose the test API.
  bool agent_debug_{false};
  void register_web_handler_();
  void handle_setup_get_(class AsyncWebServerRequest *request);
  void handle_setup_post_(class AsyncWebServerRequest *request);
  void handle_setup_reset_(class AsyncWebServerRequest *request);
  void handle_customizations_get_(class AsyncWebServerRequest *request);
  void handle_customizations_post_(class AsyncWebServerRequest *request);
  // Screenshot handler. Returns a BMP file of the current display
  // buffer. The Crowpanel has 32MB PSRAM, so a 1.2MB single-screen
  // BMP fits without issue. The handler is registered at
  // GET /autopanel/screenshot.bmp. Note this is NOT the LVGL
  // snapshot API (LV_USE_SNAPSHOT is off in this build); we
  // directly read the display's draw buffer (which is populated
  // by lv_refr_now()).
  void handle_screenshot_(class AsyncWebServerRequest *request);
  // JPEG screenshot handler. Returns a JPEG-compressed screenshot
  // suitable for direct viewing by an AI agent (no PIL conversion
  // needed). Uses the P4's hardware JPEG encoder when available
  // (SOC_JPEG_ENCODE_SUPPORTED); on chips without it (S2, S3,
  // C3, C6) the handler returns 501 Not Implemented so the test
  // harness can fall back to /autopanel/screenshot.bmp.
  // The LVGL display buffer is RGB565; the handler converts to
  // RGB888 in-place into a PSRAM scratch buffer before encoding.
  // Output quality is fixed at 80 (visually lossless for UI text
  // and arcs, ~5-10x smaller than the BMP at 1024x600).
  void handle_screenshot_jpg_(class AsyncWebServerRequest *request);
  // Simulated input device (test harness helpers). Both drive the
  // default LVGL input device (the GT911 touchscreen) by injecting
  // press/release + point events. Triggered by 'C' and 'S' serial
  // commands (see loop()) and exercised by send_cmd.py click /
  // scroll subcommands. The same functions are also called from
  // the /autopanel/test/click and /autopanel/test/scroll HTTP
  // handlers when agent_debug_ is true.
  void simulate_click_(int x, int y);
  // Same as simulate_click_ but returns the hit-tested widget
  // (nullptr when no widget is at the coord). Used by the test
  // harness's /test/click handler so the HTTP response can
  // report "missed - no widget at (x, y)" instead of silently
  // 200-OK on a coordinate that hit empty space.
  lv_obj_t* simulate_click_with_hitscan_(int x, int y);
  void simulate_scroll_(int x1, int y1, int x2, int y2);
  // Process a single-character command the same way the serial
  // command parser does. Extracted so the web API can trigger the
  // same state transitions ('g' for grid, '0'..'9' for detail, 'o'
  // for sort, etc.) without needing the serial port to be free.
  // Returns true if the command was recognized, false otherwise.
  bool process_command_(char c);

  // AGENT_DEBUG: opt-in /autopanel/test/* endpoints. Each handler
  // checks agent_debug_ at the top and returns 404 if disabled, so
  // the URL space is invisible to a non-test build.
  void handle_test_click_(class AsyncWebServerRequest *request);
  void handle_test_scroll_(class AsyncWebServerRequest *request);
  void handle_test_cmd_(class AsyncWebServerRequest *request);
  void handle_test_state_(class AsyncWebServerRequest *request);
  // Toggle a title-bar banner that says "AUTO-TEST" so the
  // human (or any other user) can see at a glance that an automated
  // test harness is driving the panel. The test harness calls
  // /autopanel/test/banner?on=1 at the start of a run and
  // ?on=0 at the end. While the banner is on, the test harness
  // is in control - tapping the screen will fire the test's
  // commands (or be lost in a layout pass), and the user should
  // not interact with the panel.
  void handle_test_banner_(class AsyncWebServerRequest *request);
  // True while the test harness has set the AUTO-TEST banner. The
  // title bar shows the banner label; off, the label is hidden.
  bool test_banner_active_{false};
  // The actual label widget. Created in create_title_bar_().
  lv_obj_t* title_test_banner_{nullptr};

  // Over-the-air log capture. The C++ side writes every log line
  // (after ESP_LOG formatting) into a small ring buffer that the
  // /autopanel/test/logs endpoint serves on demand. The buffer is
  // intentionally small (4 KB ≈ ~40 lines) to keep RAM cost near
  // zero; it's only meant for "what just happened before this
  // crash" forensics, not for tailing a long session. The serial
  // log (when a cable is attached) remains the source of truth.
  void log_buffer_append_(const std::string& line);
  std::string log_buffer_dump_(size_t max_lines, const std::string& tag) const;
  // Static ESPHome log callback. Fires on every ESP_LOG line
  // from any task; writes the formatted line into the ring
  // buffer. Required to be static (raw function pointer) per
  // logger::add_log_callback's signature.
  static void on_log_callback_(void* instance, uint8_t level,
                               const char* tag, const char* message,
                               size_t message_len);
  // Idempotent registration of the log callback. Called from
  // loop() on each tick until the logger component is ready
  // (logger::global_logger != nullptr). Becomes a no-op after
  // the first successful registration.
  void maybe_register_log_callback_();
  mutable std::mutex log_buffer_mu_;
  std::string log_buffer_;  // newline-separated ring of recent log lines
  // Logger component isn't fully initialized when our setup()
  // runs (it's a separate component that registers after us),
  // so we defer the add_log_callback call until the first
  // loop() tick when logger::global_logger is guaranteed
  // non-null. Set true once the callback is registered.
  bool log_callback_registered_{false};
  // Diagnostic counters so the test harness can see whether
  // the logger callback is actually being invoked (the log
  // buffer itself is empty until the first callback fires, so
  // we need a separate "we tried N times" signal).
  uint32_t log_callback_attempts_{0};
  uint32_t log_callback_fires_{0};
  // Number of loop ticks where logger::global_logger was
  // STILL null when we tried to register. If this stays 0 the
  // logger was non-null on the first poll (good); if it grows
  // we know the logger component initialized late.
  uint32_t log_callback_global_logger_null_{0};

  // Heap stats endpoint (GET /autopanel/test/heap). Returns one
  // line per stat (free_internal_kb=, largest_internal_kb=,
  // free_psram_kb=, largest_psram_kb=). Cheap to add and very
  // useful for catching leaks (e.g. the JPEG OOM bug would have
  // shown largest_psram_kb dropping to a few KB before the
  // screenshot endpoint started 500-ing).
  void handle_test_heap_(class AsyncWebServerRequest *request);

  // Entities dump endpoint (GET /autopanel/test/entities).
  // Returns JSON { area_id: [ {entity_id, name, friendly_name,
  // state, brightness, domain}, ... ] } - a flattened view of
  // entities_by_area_ with friendly_name resolved. Lets the test
  // harness verify display content without parsing screenshots.
  void handle_test_entities_(class AsyncWebServerRequest *request);

  // Aggregate dump endpoint (GET /autopanel/test/aggregate).
  // Returns JSON { area_id: {name, on_count, max_pct, entities: {
  // entity_id: {state, brightness} } } - the most recent
  // room_aggregates_ snapshot. Useful for verifying the
  // render_template subscription is alive and in sync.
  void handle_test_aggregate_(class AsyncWebServerRequest *request);

  // Over-the-air service-call dispatch (POST /autopanel/test/service).
  // Mirrors what the room-card button click handlers do (calls
  // the panel's own call_ha_service_ path) but lets the test
  // harness trigger the same code path without a real tap. Useful
  // for race-condition reproduction - we can flood the path with
  // calls and watch what the panel state does.
  void handle_test_service_(class AsyncWebServerRequest *request);

  // LittleFS helpers
  bool mount_storage_();
  bool read_config_file_(std::string &out);
  bool write_config_file_(const std::string &body);
  void apply_config_file_(const std::string &body);
  void apply_runtime_config_();
  // v1.27: update_title_time_() was deleted. The title-bar
  // clock is now driven by the on_clock_update_() callback,
  // which receives the HA-rendered clock string from the
  // render_template subscription. No per-tick loop work.
  // Show / hide the debug button in the title bar. Called whenever
  // the panel state changes. No-op if the button hasn't been
  // created yet (still in setup()).
  void update_debug_btn_visibility_();
  // Show / hide the debug overlay. show_debug_panel_() creates the
  // panel lazily (one-time) and rebuilds content every time it's
  // shown so live status is current. hide_debug_panel_() just sets
  // the HIDDEN flag; the widgets are reused on the next show.
  void show_debug_panel_();
  void hide_debug_panel_();
  // Build (or rebuild) the rows inside debug_panel_ from current
  // device state. Each section is a label pair: section header
  // (bold) and key:value lines. The action buttons live at the
  // bottom. Called from show_debug_panel_() and from a timer while
  // the panel is open (so the WiFi RSSI / heap values update
  // without the user having to re-open).
  void build_debug_panel_content_();
  // Reset a 2-tap confirm (called when a non-confirm button is
  // tapped, or when PENDING_ACTION_TIMEOUT_MS elapses).
  void clear_pending_action_();
  // Show / hide the sort panel. show_sort_panel_() creates the panel
  // lazily (one-time) and rebuilds content every time it's shown
  // from the current customizations_. hide_sort_panel_() just sets
  // the HIDDEN flag; widgets are reused on the next show.
  void show_sort_panel_();
  void hide_sort_panel_();
  // Build (or rebuild) the rows inside sort_panel_ from the current
  // sort_local_order_ and sort_local_hidden_ state. Each row is a
  // 60-px-tall panel with: room name, up arrow, down arrow, and a
  // hide/show toggle. The Apply button at the bottom persists the
  // local state to customizations_ and re-renders the grid.
  void build_sort_panel_content_();
  // Apply the sort panel's local state to customizations_ (room_order
  // and hidden_rooms), persist to /storage/customizations.cfg, and
  // trigger a grid refresh. Called by the Apply button.
  void apply_sort_panel_();
  // Move a room up/down in sort_local_order_ and rebuild the panel.
  // Used by the ↑/↓ button click handlers in build_sort_panel_content_().
  void sort_move_(int index, int delta);
  // GET <ha_api>/api/states, find entity_id == "zone.home", extract
  // attributes.friendly_name, cache it in home_name_ and update
  // title_home_label_. Triggered on state transition to READY and
  // throttled to one call per HOME_FETCH_INTERVAL_MS in loop(). The
  // [home] tag in the log lets the host harness verify the result.
  void fetch_home_name_();
  static constexpr uint32_t HOME_FETCH_INTERVAL_MS = 5 * 60 * 1000;  // 5 min
  // GET <ha_api>/api/states/<weather_entity_id_> and render
  // "Cloudy 3°C" into title_weather_label_. Throttled to one call
  // per WEATHER_FETCH_INTERVAL_MS. Silently no-ops on 404 (no
  // weather entity configured) so the label stays hidden rather
  // than spamming the log.
  void fetch_weather_();
  static constexpr uint32_t WEATHER_FETCH_INTERVAL_MS = 10 * 60 * 1000;  // 10 min
  // Recompute x/y/grid_index for every entry in room_cards_ in current
  // vector order (top-to-bottom, left-to-right). Call after any
  // mutation (hide, restore, future bulk import) so the grid always
  // reflects the room_cards_ vector with no gaps.
  void repack_room_cards_();
  // Rebuild the reverse map (entity_id -> area_id) from the current
  // entities_by_area_. Called after fetch_areas_() and after any
  // room/entity mutation so find_area_id_for_entity_() can answer
  // in O(log N) instead of doing an O(N*M) scan on every state push.
  void rebuild_entity_to_area_map_();
  // PSRAM-backed reverse lookup for find_area_id_for_entity_().
  // Updated by rebuild_entity_to_area_map_() (above). Keys are
  // string_views into entity_arena(); values are std::strings
  // because the outer entities_by_area_ map is std::string-keyed
  // and heterogeneous lookup across two std::string_views wouldn't
  // be portable.
  std::map<std::string_view, std::string, std::less<>,
           PsramStlAllocator<std::pair<const std::string_view, std::string>>>
      entity_to_area_map_;
  // Tracks when each entity was optimistically updated, so
  // sync_entities_from_aggregates_() can skip the entity until
  // the matching state_changed event lands (or a 10 s timeout
  // expires, whichever comes first). A small vector of pairs
  // (rather than std::unordered_map) because libstdc++'s
  // unordered_map internal allocator types don't match our
  // PsramStlAllocator's ctor signature (same issue as std::deque).
  // Bounded to a few entries (the user only toggles 1-2 at a
  // time), so the linear scan in expire_pending_optimistic_
  // updates_() is fine. entity_id is a string_view into the
  // entity_arena, so no copy.
  std::vector<std::pair<std::string_view, uint32_t>>
      optimistic_update_pending_at_;
  // Clear the pending flag for any entity whose timestamp is
  // older than the timeout. Called from loop() each tick.
  void expire_pending_optimistic_updates_();
  static constexpr uint32_t OPTIMISTIC_UPDATE_TIMEOUT_MS = 10000;
  // Pending scroll request deferred from /test/scroll. The
  // httpd handler just sets these and returns 202; loop()
  // picks them up and does the actual lv_indev_search_obj +
  // lv_obj_scroll_by walk. Doing the LVGL work in the httpd
  // task corrupts the newlib FILE struct (Guru Meditation:
  // Breakpoint in __ssprint_r at vfprintf.c:268 after the
  // walk-up loop returns).
  bool pending_scroll_{false};
  int pending_scroll_x1_{0};
  int pending_scroll_y1_{0};
  int pending_scroll_x2_{0};
  int pending_scroll_y2_{0};
  bool read_customizations_file_();
  bool write_customizations_file_();
  void apply_customizations_file_(const std::string &body);
  // Returns true if the named room/entity is hidden in the user config
  bool is_room_hidden_(std::string_view room_name) const;
  bool is_entity_hidden_(std::string_view entity_id) const;
  // Returns the custom display order for a room/area, or an empty
  // vector if none is set (callers should fall back to alphabetical
  // or HA's order).
  const psram_vector<std::string> *get_entity_order_(std::string_view area_id) const;

  static const uint32_t ROOM_COLORS_[];
  static const int MAX_ROOM_COLORS_ = 8;

  void fetch_areas_();
  void filter_and_build_room_cards_();
  // Predicate helpers take std::string_view so callers can pass
  // either an Entity field (now string_view) or a std::string from
  // other code paths. The std::string_view -> comparison is
  // allocation-free; std::string still works via implicit conversion
  // to string_view.
  bool is_area_excluded_(std::string_view area_name) const;
  bool is_entity_excluded_(std::string_view entity_id) const;
  bool is_domain_included_(std::string_view domain) const;
  void start_discovery_();
  void create_ui_from_room_cards_();
  // v1.30: incremental UI build. create_ui_from_room_cards_() now
  // calls start_ui_build_() (does screen/title/container setup,
  // computes row layout, transitions to BUILDING_CARDS) and returns.
  // continue_ui_build_() is called once per loop() tick and creates
  // exactly one card (or finalizes the container height). Spreading
  // the work across ticks lets the SPI DMA buffer pool replenish
  // between cards and lets other subsystems (WiFi, esp-tls, mDNS)
  // run their tasks, which they couldn't during a single 75-widget
  // burst that exhausted internal SRAM.
  void start_ui_build_();
  void continue_ui_build_();
  // Safety-net force-refresh. If a single LVGL flush silently fails
  // (DMA buffer alloc fail leaves the dirty rect unflushed), the
  // display shows a stale frame. Every FORCE_REFRESH_INTERVAL_MS
  // we invalidate the active screen so the next flush attempt
  // re-pushes everything. With partial flush working most of the
  // time, this is a 5-second backstop, not a per-tick hammer.
  void force_refresh_if_due_();
  void create_room_card_(void* parent, const RoomCard& room);
  void show_room_grid_();
  void show_entity_detail_(int room_index);
  void create_entity_control_(void* parent, const Entity& entity, uint32_t color);
  int compute_cards_per_row_() const;
  int get_card_x_(int col) const;
  int get_card_y_(int row) const;
  uint32_t get_room_color_(int index) const;

  // Data-driven button/label sizing. Measures the actual rendered
  // width for a given font + text so the widget is always the right
  // size no matter the locale, label wording, or font swap. The
  // signature is intentionally minimal: callers pass the font
  // they're using and horizontal padding (the LVGL default pad is
  // 0; we add 12-16px to make the touch target bigger than the
  // text itself). 4px vertical pad keeps the label from touching
  // the top/bottom edge of the button.
  static int button_width_for_text_(const char* text, const lv_font_t* font, int pad_x = 14);
  // If a single-line room name overflows the arc width, split on
  // the first space to find the longest balanced two-line split.
  // Returns the original string when it fits on one line, or
  // "Line1\nLine2" (LVGL escape for newline) when it doesn't.
  // Output is written to `out` (caller-owned, must be at least
  // 128 bytes for any reasonable room name).
  void split_room_name_to_fit_(const char* name, int max_width_px,
                                const lv_font_t* font, char* out, size_t out_size) const;
  // Auto-fit room-name font picker. Intended to return the LARGEST
  // font in a ladder whose single-line width for `name` fits
  // within `max_width_px`; the caller falls back to
  // split_room_name_to_fit_() for two-line wraps. Currently a
  // no-op (returns nullptr) - the static-linkage block on
  // font::Font* externs prevents referencing the user's yaml
  // fonts from a custom component.
  static const lv_font_t* pick_room_name_font_(const char* name, int max_width_px);

  // HA service call helper - uses native ESPHome API
  // service:    e.g. "light.turn_on", "light.turn_off", "switch.toggle"
  // target_type: "entity_id" or "area_id"
  // target_id:   the entity_id or area_id value
  // brightness_pct: 0-100, or -1 to omit
  // Takes string_view for the string params so we can pass Entity
  // fields and ControlRecord/ArcRecord fields without allocating
  // std::string temporaries at every call site.
  void call_ha_service_(std::string_view service,
                        std::string_view target_type,
                        std::string_view target_id,
                        int brightness_pct);

  // Local computation - replaces the per-room HA templates
  bool is_room_any_light_on_(const std::string& area_id,
                             const std::string& exclude_entity_id = "") const;
  uint8_t compute_room_brightness_pct_(const std::string& area_id) const;

  // Native API state subscription
  // True if HA API has ever been connected since boot. The
  // loop() polls this to auto-trigger the auth probe on
  // first connect (replaces the YAML on_client_connected
  // lambdas, which allocated std::function each invocation).
  bool ha_connected_once_{false};
  // Callbacks fired by the api server when HA pushes a state change
  // or attribute change. Take string_view because the const char*
  // overload of subscribe_home_assistant_state is zero-allocation
  // and the captured entity_id (a string_view into the arena) is
  // passed straight through.
  void on_entity_state_changed_(std::string_view entity_id, const char* state);
  void on_entity_attribute_changed_(std::string_view entity_id, const char* value);
  void update_room_card_visual_state_for_entity_(std::string_view entity_id);
  void update_room_card_visual_state_for_area_(const std::string& area_id);
  std::string find_area_id_for_entity_(std::string_view entity_id) const;

  // v1.27: render-template subscription handlers. Both are
  // called from loopTask (the events are pushed to a
  // loopTask-drained queue in ha_ws_client.h, so the actual
  // lv_label_set_text / ArduinoJson parse happens on the
  // LVGL-safe context).
  //
  // on_clock_update_ takes the rendered clock string
  // (e.g. "14:30") and stamps it into the time label. The
  // format string lives in the WS subscribe message
  // ({{ now().strftime(...) }}).
  void on_clock_update_(const std::string& rendered);
  // v1.27 (Phase 4): the per-area aggregate JSON handler.
  // Forwarded by HaWsClient::drain_aggregate_events. Decodes
  // the JSON and updates entities_by_area_ in place. Takes
  // a string_view pointing into PSRAM (the AggregateEvent's
  // psram-backed std::string) so we don't allocate a 25KB
  // std::string copy on the internal heap.
  void on_aggregate_update_(std::string_view json);

  // v1.27: set up the clock + aggregate render_template
  // subscriptions. Called from HaWsClient::parse_auth_ok_
  // (which is the right moment to subscribe - we know
  // auth_ok came back, so the WS is ready). Returns true
  // if both subscriptions succeeded.
  bool setup_render_template_subscriptions_();

  // v1.27: subscription ids from TemplateApi::subscribe().
  // Stored so we can unsubscribe on shutdown (future) and
  // log them in dump_config.
  uint32_t clock_sub_id_{0};
  uint32_t aggregate_sub_id_{0};

  // v1.27: build the per-area aggregate template body
  // (room_aggregate_template_) and the included_areas
  // variables JSON (room_aggregate_variables_) from the
  // discovered_areas_ list. Called once after fetch_areas_()
  // populates the area list. Idempotent.
  void build_room_aggregate_template_();

  // v1.27: one-shot fetch of the per-area aggregate via
  // POST /api/template with the included_areas variable.
  // Called from start_discovery_() after fetch_areas_() so
  // the panel's initial state is populated before the WS
  // subscription's first event arrives (or in case the WS
  // subscription is delayed by a few hundred ms).
  void fetch_room_aggregates_();

  // v1.27: parse the aggregate JSON and update
  // room_aggregates_ + entities_by_area_ in place.
  // Shared by the initial fetch and every WS push.
  // No-op on parse error. Takes a string_view so the
  // initial fetch (which has a std::string response) and
  // the WS push (which has a PSRAM-backed string) can
  // both pass without an intermediate copy.
  void apply_room_aggregates_(std::string_view json);

  // v1.27: walk room_aggregates_ and copy each Entry's
  // state/brightness into the corresponding Entity record
  // in entities_by_area_. Then call
  // update_room_card_visual_state_for_area_() for each
  // changed area so the room cards repaint. Also calls
  // update_media_banner_() since the media state can
  // change too.
  void sync_entities_from_aggregates_();

  // Panel state machine
  void set_panel_state_(PanelState new_state);
  void show_status_screen_(const char* title, const char* message, bool show_retry);
  void hide_status_screen_();
  // Build the appropriate "how to reach this device" message for the
  // SETUP_REQUIRED screen. Picks the right info based on whether WiFi is
  // connected (use use_address) or in AP fallback (use AP SSID/password).
  std::string build_setup_message_();

  // Update the title bar status indicator (called on every state change
  // and on api.on_client_connected / disconnected).
  void update_title_bar_();
  // Create the top title bar (HA status + Edit button + optional back
  // button) as a child of the given parent (the screen). Called from
  // both create_ui_from_room_cards_ and show_entity_detail_ so it's
  // visible on every page.
  void create_title_bar_(lv_obj_t *parent);
  // Re-render the room grid (called when customizations change,
  // when a room is hidden/shown via the Sort panel, or after
  // discovery rebuilds room_cards_).
  void refresh_room_cards_();
  // Update the "Now Playing" banner at the top of the grid. Reads
  // the current entity state fields and shows one tile per media_player
  // in 'playing' state. Lazy-creates the banner container on first
  // call. Called from create_ui_from_room_cards_() at render time
  // and from on_entity_state_changed_() for live updates.
  void update_media_banner_();

  // Boot: mount LittleFS, read config, decide what state to start in.
  void boot_from_storage_();
  void probe_authorization_();
  void on_auth_probe_response_(bool success, const char* error);
  void loop() override;  // for auth-probe timeout
};

}  // namespace ha_autopanel
}  // namespace esphome
