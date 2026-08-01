#include "navgrid.h"
#include "internal/navgrid_private.hpp"
#include <algorithm>
#include <atomic>
#include <unordered_set>
#include <unordered_map>
#include <vector>
#include <cmath>
#include <cstdint>
#include <limits>
#include <new>
#include "coord.h"
#include "coord_list.h"
#include "coord_hash.h"
#include "internal/navgrid_callback.hpp"
#include "internal/navgrid_overlay.hpp"

namespace {

using overlay_coord_key_t = uint64_t;

constexpr uint32_t navgrid_abi1_version = 1;
constexpr uint64_t navgrid_abi1_fingerprint = UINT64_C(0x4e47524944010028);
constexpr size_t overlay_cancel_poll_interval = 64;

overlay_coord_key_t overlay_coord_key(int x, int y) {
    return (static_cast<uint64_t>(static_cast<uint32_t>(x)) << 32)
        | static_cast<uint32_t>(y);
}

struct overlay_source_key_t {
    byul::navsys::internal::navgrid_overlay_source_kind kind;
    const void* source;

    bool operator==(const overlay_source_key_t& other) const {
        return kind == other.kind && source == other.source;
    }
};

struct overlay_source_hash_t {
    size_t operator()(const overlay_source_key_t& value) const {
        const auto kind = static_cast<size_t>(value.kind);
        const auto pointer = reinterpret_cast<uintptr_t>(value.source);
        return (pointer >> 4) ^ (kind * 0x9e3779b9u);
    }
};

using overlay_coord_set_t = std::unordered_set<overlay_coord_key_t>;

struct navgrid_overlay_state_t {
    uint64_t owner_cookie = 0;
    navgrid_overlay_id_t next_id = 1;
    std::unordered_map<navgrid_overlay_id_t, overlay_coord_set_t> layers;
    std::unordered_map<navgrid_overlay_id_t, overlay_coord_set_t> open_layers;
    std::unordered_map<
        overlay_source_key_t,
        navgrid_overlay_id_t,
        overlay_source_hash_t> sources;
};

std::unordered_map<const navgrid_t*, navgrid_overlay_state_t> overlay_states;
std::atomic<uint64_t> next_overlay_owner_cookie{1};

navsys_status_t ensure_overlay_owner(navgrid_overlay_state_t& state) {
    if (state.owner_cookie != 0) return NAVSYS_STATUS_OK;
    const uint64_t candidate = next_overlay_owner_cookie.fetch_add(
        1, std::memory_order_relaxed);
    if (candidate == 0) return NAVSYS_STATUS_LIMIT_REACHED;
    state.owner_cookie = candidate;
    return NAVSYS_STATUS_OK;
}

navsys_status_t poll_overlay_cancel(
    byul::navsys::internal::navgrid_overlay_cancel_func cancel_func,
    void* cancel_userdata) {
    if (!cancel_func) return NAVSYS_STATUS_OK;
    try {
        return cancel_func(cancel_userdata)
            ? NAVSYS_STATUS_CANCELLED
            : NAVSYS_STATUS_OK;
    } catch (...) {
        return NAVSYS_STATUS_CALLBACK_FAILED;
    }
}

const navgrid_overlay_state_t* find_overlay_state(const navgrid_t* navgrid) {
    const auto iter = overlay_states.find(navgrid);
    return iter == overlay_states.end() ? nullptr : &iter->second;
}

bool overlay_layers_contain(
    const std::unordered_map<navgrid_overlay_id_t, overlay_coord_set_t>& layers,
    overlay_coord_key_t key,
    navgrid_overlay_id_t* out_latest) {
    bool found = false;
    navgrid_overlay_id_t latest = 0;
    for (const auto& [id, coords] : layers) {
        if (coords.find(key) == coords.end()) continue;
        if (!found || id > latest) latest = id;
        found = true;
    }
    if (out_latest) *out_latest = latest;
    return found;
}

bool overlay_state_contains(
    const navgrid_overlay_state_t* state, overlay_coord_key_t key) {
    if (!state) return false;
    return overlay_layers_contain(state->layers, key, nullptr);
}

bool navgrid_base_is_blocked(const navgrid_t* navgrid, int x, int y) {
    if (!navgrid || !navgrid->cell_map) return false;
    const coord_t key{x, y};
    const auto* cell = static_cast<const navcell_t*>(
        coord_hash_get(navgrid->cell_map, &key));
    if (!cell) return false;
    if (navcell_validate(cell) != NAVSYS_STATUS_OK) return true;
    return cell->terrain == TERRAIN_TYPE_FORBIDDEN;
}

bool navgrid_effectively_blocked(
    const navgrid_t* navgrid,
    const navgrid_overlay_state_t* state,
    overlay_coord_key_t key,
    int x,
    int y) {
    navgrid_overlay_id_t latest_block = 0;
    const bool blocked = navgrid_base_is_blocked(navgrid, x, y)
        || (state && overlay_layers_contain(
            state->layers, key, &latest_block));
    if (!blocked || !state) return blocked;
    navgrid_overlay_id_t latest_open = 0;
    const bool opened = overlay_layers_contain(
        state->open_layers, key, &latest_open);
    return !opened || latest_block >= latest_open;
}

navsys_status_t commit_overlay_state(
    navgrid_t* navgrid, navgrid_overlay_state_t&& prepared) {
    auto current = overlay_states.find(navgrid);
    if (prepared.layers.empty() && prepared.open_layers.empty()
        && prepared.sources.empty()) {
        if (current != overlay_states.end()) overlay_states.erase(current);
        return NAVSYS_STATUS_OK;
    }
    if (current != overlay_states.end()) {
        current->second.layers.swap(prepared.layers);
        current->second.open_layers.swap(prepared.open_layers);
        current->second.sources.swap(prepared.sources);
        current->second.owner_cookie = prepared.owner_cookie;
        current->second.next_id = prepared.next_id;
        return NAVSYS_STATUS_OK;
    }
    try {
        overlay_states.emplace(navgrid, std::move(prepared));
        return NAVSYS_STATUS_OK;
    } catch (const std::bad_alloc&) {
        return NAVSYS_STATUS_OUT_OF_MEMORY;
    } catch (...) {
        return NAVSYS_STATUS_CORRUPT_STATE;
    }
}

navsys_status_t copy_overlay_state(
    const navgrid_t* source, navgrid_t* destination) {
    const auto* state = find_overlay_state(source);
    if (!state) return NAVSYS_STATUS_OK;
    try {
        navgrid_overlay_state_t copied = *state;
        copied.owner_cookie = 0;
        const navsys_status_t status = ensure_overlay_owner(copied);
        if (status != NAVSYS_STATUS_OK) return status;
        return commit_overlay_state(destination, std::move(copied));
    } catch (const std::bad_alloc&) {
        return NAVSYS_STATUS_OUT_OF_MEMORY;
    } catch (...) {
        return NAVSYS_STATUS_CORRUPT_STATE;
    }
}

navgrid_overlay_id_t next_overlay_id(navgrid_overlay_state_t& state) {
    while (state.next_id == 0
        || state.layers.find(state.next_id) != state.layers.end()
        || state.open_layers.find(state.next_id) != state.open_layers.end()) {
        if (state.next_id == std::numeric_limits<navgrid_overlay_id_t>::max())
            return 0;
        ++state.next_id;
    }
    const navgrid_overlay_id_t result = state.next_id;
    ++state.next_id;
    if (state.next_id == 0) state.next_id = 1;
    return result;
}

navsys_status_t set_default_overlay_coord(
    navgrid_t* navgrid, int x, int y, bool blocked, bool* out_changed) {
    if (!navgrid || !out_changed) return NAVSYS_STATUS_INVALID_ARGUMENT;
    if (byul::navsys::internal::navgrid_callback_is_active(navgrid))
        return NAVSYS_STATUS_IN_PROGRESS;
    if (!navgrid_is_inside(navgrid, x, y)) return NAVSYS_STATUS_NOT_FOUND;

    const overlay_coord_key_t key = overlay_coord_key(x, y);
    const auto* current = find_overlay_state(navgrid);
    const bool before = navgrid_effectively_blocked(
        navgrid, current, key, x, y);
    if (!blocked) {
        if (!current) {
            *out_changed = false;
            return NAVSYS_STATUS_OK;
        }
        const auto layer = current->layers.find(0);
        if (layer == current->layers.end()
            || layer->second.find(key) == layer->second.end()) {
            *out_changed = false;
            return NAVSYS_STATUS_OK;
        }
    }

    try {
        navgrid_overlay_state_t prepared = current
            ? *current
            : navgrid_overlay_state_t{};
        if (blocked) {
            const navsys_status_t status = ensure_overlay_owner(prepared);
            if (status != NAVSYS_STATUS_OK) return status;
        }
        if (blocked) {
            prepared.layers[0].insert(key);
        } else {
            auto layer = prepared.layers.find(0);
            layer->second.erase(key);
            if (layer->second.empty()) prepared.layers.erase(layer);
        }
        const bool after = navgrid_effectively_blocked(
            navgrid, &prepared, key, x, y);
        const navsys_status_t status = commit_overlay_state(
            navgrid, std::move(prepared));
        if (status != NAVSYS_STATUS_OK) return status;
        *out_changed = before != after;
        return NAVSYS_STATUS_OK;
    } catch (const std::bad_alloc&) {
        return NAVSYS_STATUS_OUT_OF_MEMORY;
    } catch (...) {
        return NAVSYS_STATUS_CORRUPT_STATE;
    }
}

navsys_status_t replace_source_overlay(
    navgrid_t* navgrid,
    const overlay_source_key_t& source_key,
    const coord_t* coords,
    size_t count,
    size_t* out_changed_count) {
    if (!navgrid || !source_key.source || (!coords && count != 0)
        || !out_changed_count) {
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    }
    if (byul::navsys::internal::navgrid_callback_is_active(navgrid))
        return NAVSYS_STATUS_IN_PROGRESS;

    const auto* current = find_overlay_state(navgrid);
    try {
        navgrid_overlay_state_t prepared = current
            ? *current
            : navgrid_overlay_state_t{};
        const navsys_status_t owner_status = ensure_overlay_owner(prepared);
        if (owner_status != NAVSYS_STATUS_OK) return owner_status;
        navgrid_overlay_id_t id = 0;
        const auto source = prepared.sources.find(source_key);
        if (source != prepared.sources.end()) {
            id = source->second;
        } else {
            id = next_overlay_id(prepared);
            if (id == 0) return NAVSYS_STATUS_LIMIT_REACHED;
            prepared.sources.emplace(source_key, id);
        }

        overlay_coord_set_t replacement;
        replacement.reserve(count);
        for (size_t index = 0; index < count; ++index) {
            replacement.insert(overlay_coord_key(coords[index].x, coords[index].y));
        }

        overlay_coord_set_t candidates = replacement;
        const auto old_layer = prepared.layers.find(id);
        if (old_layer != prepared.layers.end()) {
            candidates.insert(old_layer->second.begin(), old_layer->second.end());
        }
        prepared.layers[id] = std::move(replacement);

        size_t changed = 0;
        for (const overlay_coord_key_t key : candidates) {
            const int x = static_cast<int32_t>(key >> 32);
            const int y = static_cast<int32_t>(key & 0xffffffffu);
            if (navgrid_effectively_blocked(navgrid, current, key, x, y)
                != navgrid_effectively_blocked(navgrid, &prepared, key, x, y)) {
                ++changed;
            }
        }
        const navsys_status_t status = commit_overlay_state(
            navgrid, std::move(prepared));
        if (status != NAVSYS_STATUS_OK) return status;
        *out_changed_count = changed;
        return NAVSYS_STATUS_OK;
    } catch (const std::bad_alloc&) {
        return NAVSYS_STATUS_OUT_OF_MEMORY;
    } catch (...) {
        return NAVSYS_STATUS_CORRUPT_STATE;
    }
}

navsys_status_t apply_blocked_overlay_impl(
    navgrid_t* navgrid,
    const coord_t* coords,
    size_t count,
    byul::navsys::internal::navgrid_overlay_cancel_func cancel_func,
    void* cancel_userdata,
    byul::navsys::internal::navgrid_tracked_overlay_t* out_overlay,
    size_t* out_changed_count) {
    if (!navgrid || (!coords && count != 0)
        || !out_overlay || !out_changed_count) {
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    }
    if (byul::navsys::internal::navgrid_callback_is_active(navgrid))
        return NAVSYS_STATUS_IN_PROGRESS;
    navsys_status_t status = poll_overlay_cancel(
        cancel_func, cancel_userdata);
    if (status != NAVSYS_STATUS_OK) return status;
    for (size_t index = 0; index < count; ++index) {
        status = poll_overlay_cancel(cancel_func, cancel_userdata);
        if (status != NAVSYS_STATUS_OK) return status;
        if (!navgrid_is_inside(navgrid, coords[index].x, coords[index].y))
            return NAVSYS_STATUS_NOT_FOUND;
    }

    const auto* current = find_overlay_state(navgrid);
    try {
        navgrid_overlay_state_t prepared = current
            ? *current
            : navgrid_overlay_state_t{};
        status = ensure_overlay_owner(prepared);
        if (status != NAVSYS_STATUS_OK) return status;
        const navgrid_overlay_id_t id = next_overlay_id(prepared);
        if (id == 0) return NAVSYS_STATUS_LIMIT_REACHED;
        overlay_coord_set_t layer;
        layer.reserve(count);
        for (size_t index = 0; index < count; ++index) {
            status = poll_overlay_cancel(cancel_func, cancel_userdata);
            if (status != NAVSYS_STATUS_OK) return status;
            layer.insert(overlay_coord_key(coords[index].x, coords[index].y));
        }
        size_t changed = 0;
        for (const overlay_coord_key_t key : layer) {
            status = poll_overlay_cancel(cancel_func, cancel_userdata);
            if (status != NAVSYS_STATUS_OK) return status;
            const int x = static_cast<int32_t>(key >> 32);
            const int y = static_cast<int32_t>(key & 0xffffffffu);
            if (!navgrid_effectively_blocked(navgrid, current, key, x, y))
                ++changed;
        }
        prepared.layers.emplace(id, std::move(layer));
        status = commit_overlay_state(navgrid, std::move(prepared));
        if (status != NAVSYS_STATUS_OK) return status;
        out_overlay->owner_cookie = find_overlay_state(navgrid)->owner_cookie;
        out_overlay->overlay = id;
        *out_changed_count = changed;
        return NAVSYS_STATUS_OK;
    } catch (const std::bad_alloc&) {
        return NAVSYS_STATUS_OUT_OF_MEMORY;
    } catch (...) {
        return NAVSYS_STATUS_CORRUPT_STATE;
    }
}

navsys_status_t remove_blocked_overlay_impl(
    navgrid_t* navgrid,
    navgrid_overlay_id_t overlay,
    uint64_t expected_owner_cookie,
    size_t* out_changed_count) {
    if (!navgrid || overlay == 0 || !out_changed_count)
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    if (byul::navsys::internal::navgrid_callback_is_active(navgrid))
        return NAVSYS_STATUS_IN_PROGRESS;
    const auto* current = find_overlay_state(navgrid);
    if (!current) {
        return expected_owner_cookie == 0
            ? NAVSYS_STATUS_NOT_FOUND
            : NAVSYS_STATUS_INVALIDATED;
    }
    if (expected_owner_cookie != 0
        && current->owner_cookie != expected_owner_cookie) {
        return NAVSYS_STATUS_INVALIDATED;
    }
    const auto layer = current->layers.find(overlay);
    if (layer == current->layers.end()) {
        return expected_owner_cookie == 0
            ? NAVSYS_STATUS_NOT_FOUND
            : NAVSYS_STATUS_INVALIDATED;
    }
    try {
        navgrid_overlay_state_t prepared = *current;
        prepared.layers.erase(overlay);
        for (auto source = prepared.sources.begin();
             source != prepared.sources.end();) {
            if (source->second == overlay)
                source = prepared.sources.erase(source);
            else
                ++source;
        }
        size_t changed = 0;
        for (const overlay_coord_key_t key : layer->second) {
            const int x = static_cast<int32_t>(key >> 32);
            const int y = static_cast<int32_t>(key & 0xffffffffu);
            if (navgrid_effectively_blocked(navgrid, current, key, x, y)
                != navgrid_effectively_blocked(
                    navgrid, &prepared, key, x, y)) {
                ++changed;
            }
        }
        const navsys_status_t status = commit_overlay_state(
            navgrid, std::move(prepared));
        if (status != NAVSYS_STATUS_OK) return status;
        *out_changed_count = changed;
        return NAVSYS_STATUS_OK;
    } catch (const std::bad_alloc&) {
        return NAVSYS_STATUS_OUT_OF_MEMORY;
    } catch (...) {
        return NAVSYS_STATUS_CORRUPT_STATE;
    }
}

constexpr int canonical_dx4[] = {1, 0, -1, 0};
constexpr int canonical_dy4[] = {0, 1, 0, -1};
constexpr int canonical_dx8[] = {1, 1, 0, -1, -1, -1, 0, 1};
constexpr int canonical_dy8[] = {0, 1, 1, 1, 0, -1, -1, -1};
constexpr int legacy_dx4[] = {0, -1, 1, 0};
constexpr int legacy_dy4[] = {-1, 0, 0, 1};
constexpr int legacy_dx8[] = {0, -1, 1, 0, -1, -1, 1, 1};
constexpr int legacy_dy8[] = {-1, 0, 0, 1, -1, 1, -1, 1};

bool valid_export_buffer(
    const void* out_buffer, size_t capacity, const size_t* out_count) {
    return out_count
        && ((!out_buffer && capacity == 0) || (out_buffer && capacity != 0));
}

navsys_status_t collect_immediate_neighbors(
    const navgrid_t* navgrid,
    int x,
    int y,
    bool traversable_only,
    bool legacy_order,
    coord_t (&result)[8],
    size_t& result_count) {
    if (!navgrid) return NAVSYS_STATUS_INVALID_ARGUMENT;
    const int* dx = nullptr;
    const int* dy = nullptr;
    int candidate_count = 0;
    if (navgrid->mode == NAVGRID_DIR_4) {
        dx = legacy_order ? legacy_dx4 : canonical_dx4;
        dy = legacy_order ? legacy_dy4 : canonical_dy4;
        candidate_count = 4;
    } else if (navgrid->mode == NAVGRID_DIR_8) {
        dx = legacy_order ? legacy_dx8 : canonical_dx8;
        dy = legacy_order ? legacy_dy8 : canonical_dy8;
        candidate_count = 8;
    } else {
        return NAVSYS_STATUS_CORRUPT_STATE;
    }

    result_count = 0;
    for (int index = 0; index < candidate_count; ++index) {
        const int64_t nx64 = static_cast<int64_t>(x) + dx[index];
        const int64_t ny64 = static_cast<int64_t>(y) + dy[index];
        if (nx64 < std::numeric_limits<int>::min()
            || nx64 > std::numeric_limits<int>::max()
            || ny64 < std::numeric_limits<int>::min()
            || ny64 > std::numeric_limits<int>::max()) {
            continue;
        }
        const int nx = static_cast<int>(nx64);
        const int ny = static_cast<int>(ny64);
        if (!navgrid_is_inside(navgrid, nx, ny)) continue;
        if (traversable_only) {
            bool blocked = false;
            const navsys_status_t status =
                byul::navsys::internal::navgrid_invoke_is_coord_blocked_checked(
                    navgrid, nx, ny, &blocked);
            if (status != NAVSYS_STATUS_OK) return status;
            if (blocked) continue;
        }
        result[result_count++] = coord_t{nx, ny};
    }
    return NAVSYS_STATUS_OK;
}

bool range_candidate(navgrid_dir_mode_t mode, int range, int64_t dx, int64_t dy) {
    const int64_t absolute_x = dx < 0 ? -dx : dx;
    const int64_t absolute_y = dy < 0 ? -dy : dy;
    if (mode == NAVGRID_DIR_8) {
        return absolute_x <= static_cast<int64_t>(range) + 1
            && absolute_y <= static_cast<int64_t>(range) + 1
            && !(range == 0 && dx == 0 && dy == 0);
    }
    if (mode != NAVGRID_DIR_4) return false;
    if (range != 0 && absolute_x <= range && absolute_y <= range) return true;
    return (absolute_x == static_cast<int64_t>(range) + 1 && absolute_y <= range)
        || (absolute_y == static_cast<int64_t>(range) + 1 && absolute_x <= range);
}

bool overlay_key_has_canonical_owner(
    const navgrid_overlay_state_t* state,
    navgrid_overlay_id_t owner,
    overlay_coord_key_t key) {
    if (!state) return false;
    for (const auto& [id, coords] : state->layers) {
        if (id < owner && coords.find(key) != coords.end()) return false;
    }
    for (const auto& [id, coords] : state->open_layers) {
        if (id < owner && coords.find(key) != coords.end()) return false;
    }
    return true;
}

struct cell_validation_context_t {
    bool valid = true;
};

void validate_materialized_cell(
    const coord_t*, void* value, void* userdata) {
    auto* context = static_cast<cell_validation_context_t*>(userdata);
    if (!value
        || navcell_validate(static_cast<const navcell_t*>(value))
            != NAVSYS_STATUS_OK) {
        context->valid = false;
    }
}

struct cell_fill_context_t {
    const navgrid_t* navgrid;
    const navgrid_overlay_state_t* overlays;
    navgrid_cell_entry_t* entries;
    size_t index = 0;
};

void fill_materialized_cell(
    const coord_t* key, void* value, void* userdata) {
    auto* context = static_cast<cell_fill_context_t*>(userdata);
    navgrid_cell_entry_t& entry = context->entries[context->index++];
    entry.coord = *key;
    entry.cell = *static_cast<const navcell_t*>(value);
    entry.present = true;
    entry.blocked = navgrid_effectively_blocked(
        context->navgrid,
        context->overlays,
        overlay_coord_key(key->x, key->y),
        key->x,
        key->y);
}

bool cell_entry_less(
    const navgrid_cell_entry_t& left, const navgrid_cell_entry_t& right) {
    return left.coord.x < right.coord.x
        || (left.coord.x == right.coord.x && left.coord.y < right.coord.y);
}

} // namespace

uint32_t navgrid_get_abi_version(void) {
    return BYUL_NAVGRID_ABI_VERSION;
}

uint64_t navgrid_get_abi_fingerprint(void) {
    return BYUL_NAVGRID_ABI_FINGERPRINT;
}

navsys_status_t navgrid_check_abi(
    uint32_t expected_version,
    uint64_t expected_fingerprint,
    navgrid_abi_mismatch_t* out_mismatch) {
    if (!out_mismatch) return NAVSYS_STATUS_INVALID_ARGUMENT;

    uint64_t supported_fingerprint = 0;
    if (expected_version == BYUL_NAVGRID_ABI_VERSION) {
        supported_fingerprint = BYUL_NAVGRID_ABI_FINGERPRINT;
    } else if (expected_version == navgrid_abi1_version) {
        supported_fingerprint = navgrid_abi1_fingerprint;
    } else {
        *out_mismatch = NAVGRID_ABI_VERSION_MISMATCH;
        return NAVSYS_STATUS_UNSUPPORTED;
    }

    if (expected_fingerprint != supported_fingerprint) {
        *out_mismatch = NAVGRID_ABI_FINGERPRINT_MISMATCH;
        return NAVSYS_STATUS_UNSUPPORTED;
    }
    *out_mismatch = NAVGRID_ABI_MATCH;
    return NAVSYS_STATUS_OK;
}

bool is_coord_blocked_navgrid(const void* context, 
    int x, int y, void* userdata) {

   const navgrid_t* navgrid = (const navgrid_t*)context;
    if (!navgrid) return false;
    const overlay_coord_key_t key = overlay_coord_key(x, y);
    return navgrid_effectively_blocked(
        navgrid, find_overlay_state(navgrid), key, x, y);
}

navgrid_t* navgrid_create() {
    return navgrid_create_full(0, 0, NAVGRID_DIR_8, 
        (is_coord_blocked_func) is_coord_blocked_navgrid);
}

namespace {

void* navcell_copy_for_coord_hash(const void* value) {
    return navcell_copy(static_cast<const navcell_t*>(value));
}

void navcell_destroy_for_coord_hash(void* value) {
    navcell_destroy(static_cast<navcell_t*>(value));
}

} // namespace

navgrid_t* navgrid_create_full(int width, int height, navgrid_dir_mode_t mode, 
    is_coord_blocked_func is_coord_blocked_fn) {

    if (!is_coord_blocked_fn) {
        is_coord_blocked_fn = is_coord_blocked_navgrid;
    }

    navgrid_t* navgrid = nullptr;
    try {
        navgrid = new navgrid_t{};
        navgrid->width = width;
        navgrid->height = height;
        navgrid->mode = mode;
        navgrid->cell_map = coord_hash_create_full(
            navcell_copy_for_coord_hash,
            navcell_destroy_for_coord_hash
        );
        if (!navgrid->cell_map) {
            navgrid_destroy(navgrid);
            return nullptr;
        }

        navgrid->is_coord_blocked_fn = is_coord_blocked_fn;
        navgrid->is_coord_blocked_fn_userdata = nullptr;
        return navgrid;
    } catch (...) {
        navgrid_destroy(navgrid);
        return nullptr;
    }
}

void navgrid_destroy(navgrid_t* navgrid) {
    if (!navgrid) return;
    if (byul::navsys::internal::navgrid_callback_is_active(navgrid))
        return;
    overlay_states.erase(navgrid);
    coord_hash_destroy(navgrid->cell_map);
    delete navgrid;
}

navgrid_t* navgrid_copy(const navgrid_t* navgrid) {
    if (!navgrid) return nullptr;
    if (byul::navsys::internal::navgrid_callback_is_active(navgrid))
        return nullptr;
    navgrid_t* c = navgrid_create_full(
        navgrid->width, navgrid->height, navgrid->mode, 
        navgrid->is_coord_blocked_fn);
    if (!c) return nullptr;

    coord_hash_t* copied_map = nullptr;
    if (coord_hash_copy_ex(navgrid->cell_map, &copied_map)
        != NAVSYS_STATUS_OK) {
        navgrid_destroy(c);
        return nullptr;
    }
    coord_hash_destroy(c->cell_map);
    c->cell_map = copied_map;
    c->is_coord_blocked_fn_userdata =
        navgrid->is_coord_blocked_fn_userdata;
    if (copy_overlay_state(navgrid, c) != NAVSYS_STATUS_OK) {
        navgrid_destroy(c);
        return nullptr;
    }
    return c;
}

uint32_t navgrid_hash(const navgrid_t* navgrid) {
    if (!navgrid) return 0;
    uint32_t h = 17;
    h = h * 31 + navgrid->width;
    h = h * 31 + navgrid->height;
    h = h * 31 + navgrid->mode;
    h = h * 31 + coord_hash_length(navgrid->cell_map);
    return h;
}

bool navgrid_equal(const navgrid_t* a, const navgrid_t* b) {
    if (!a || !b) return false;
    return a->width == b->width && a->height == b->height &&
           a->mode == b->mode &&
           coord_hash_equal(a->cell_map, b->cell_map);
}

int navgrid_get_width(const navgrid_t* navgrid) { 
    return navgrid ? navgrid->width : 0; 
}

void navgrid_set_width(navgrid_t* navgrid, int w) { 
    if (navgrid) navgrid->width = w; 
}

int navgrid_get_height(const navgrid_t* navgrid) { 
    return navgrid ? navgrid->height : 0; 
}

void navgrid_set_height(navgrid_t* navgrid, int h) { 
    if (navgrid) navgrid->height = h; 
}

void navgrid_set_is_coord_blocked_func(
    navgrid_t* navgrid, is_coord_blocked_func fn){

    if (!navgrid) return;
    navgrid->is_coord_blocked_fn = fn;
}

is_coord_blocked_func navgrid_get_is_coord_blocked_fn(
    const navgrid_t* navgrid){
    if (!navgrid) return nullptr;
    return navgrid->is_coord_blocked_fn;
}

navsys_status_t navgrid_fetch_is_coord_blocked_binding(
    const navgrid_t* navgrid,
    is_coord_blocked_func* out_fn,
    void** out_userdata) {
    if (!navgrid || !out_fn || !out_userdata)
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    *out_fn = navgrid->is_coord_blocked_fn;
    *out_userdata = navgrid->is_coord_blocked_fn_userdata;
    return NAVSYS_STATUS_OK;
}

navsys_status_t navgrid_bind_is_coord_blocked_func(
    navgrid_t* navgrid, is_coord_blocked_func fn, void* userdata) {
    if (!navgrid || !fn) return NAVSYS_STATUS_INVALID_ARGUMENT;
    if (byul::navsys::internal::navgrid_callback_is_active(navgrid))
        return NAVSYS_STATUS_IN_PROGRESS;
    navgrid->is_coord_blocked_fn = fn;
    navgrid->is_coord_blocked_fn_userdata = userdata;
    return NAVSYS_STATUS_OK;
}

navsys_status_t navgrid_unbind_is_coord_blocked_func(navgrid_t* navgrid) {
    if (!navgrid) return NAVSYS_STATUS_INVALID_ARGUMENT;
    if (byul::navsys::internal::navgrid_callback_is_active(navgrid))
        return NAVSYS_STATUS_IN_PROGRESS;
    navgrid->is_coord_blocked_fn = nullptr;
    navgrid->is_coord_blocked_fn_userdata = nullptr;
    return NAVSYS_STATUS_OK;
}

navgrid_dir_mode_t navgrid_get_mode(const navgrid_t* navgrid) {
     return navgrid ? navgrid->mode : NAVGRID_DIR_4; 
    }

void navgrid_set_mode(navgrid_t* navgrid, navgrid_dir_mode_t mode) {
     if (navgrid) navgrid->mode = mode; 
    }

bool navgrid_block_coord(navgrid_t* navgrid, int x, int y) {
    if (!navgrid) return false;
    const coord_t c = {x, y};
    if (coord_hash_contains(navgrid->cell_map, &c)) {
        const auto* current = static_cast<const navcell_t*>(
            coord_hash_get(navgrid->cell_map, &c));
        return current && current->terrain == TERRAIN_TYPE_FORBIDDEN;
    }

    navcell_t nc{};
    if (navcell_init_checked(&nc, TERRAIN_TYPE_FORBIDDEN, 0)
        != NAVSYS_STATUS_OK) {
        return false;
    }
    return coord_hash_upsert_copy(navgrid->cell_map, &c, &nc, nullptr)
        == NAVSYS_STATUS_OK;
}

bool navgrid_unblock_coord(navgrid_t* navgrid, int x, int y) {
    if (!navgrid) return false;
    const coord_t c = {x, y};
    const auto* current = static_cast<const navcell_t*>(
        coord_hash_get(navgrid->cell_map, &c));
    if (!current || current->terrain != TERRAIN_TYPE_FORBIDDEN) return false;
    return coord_hash_remove(navgrid->cell_map, &c);
}

bool navgrid_is_inside(const navgrid_t* navgrid, int x, int y) {
    if (!navgrid) return false;

    int min_x = (navgrid->width >= 0) ? 0 : navgrid->width;
    int max_x = (navgrid->width >= 0) ? navgrid->width : 0;
    int min_y = (navgrid->height >= 0) ? 0 : navgrid->height;
    int max_y = (navgrid->height >= 0) ? navgrid->height : 0;

    bool x_ok = (navgrid->width == 0 || (x >= min_x && x < max_x));
    bool y_ok = (navgrid->height == 0 || (y >= min_y && y < max_y));

    return x_ok && y_ok;
}

void navgrid_clear(navgrid_t* navgrid) {
    bool changed = false;
    (void)navgrid_clear_ex(navgrid, &changed);
}

navsys_status_t navgrid_block_coord_ex(
    navgrid_t* navgrid, int x, int y, bool* out_changed) {
    return set_default_overlay_coord(navgrid, x, y, true, out_changed);
}

navsys_status_t navgrid_unblock_coord_ex(
    navgrid_t* navgrid, int x, int y, bool* out_changed) {
    return set_default_overlay_coord(navgrid, x, y, false, out_changed);
}

navsys_status_t navgrid_clear_ex(navgrid_t* navgrid, bool* out_changed) {
    if (!navgrid || !out_changed) return NAVSYS_STATUS_INVALID_ARGUMENT;
    if (byul::navsys::internal::navgrid_callback_is_active(navgrid))
        return NAVSYS_STATUS_IN_PROGRESS;
    const bool changed = coord_hash_size(navgrid->cell_map) != 0
        || find_overlay_state(navgrid) != nullptr;
    coord_hash_clear(navgrid->cell_map);
    overlay_states.erase(navgrid);
    *out_changed = changed;
    return NAVSYS_STATUS_OK;
}

bool navgrid_set_cell(
    navgrid_t* navgrid, int x, int y, const navcell_t* cell) {
    navcell_t prior{};
    bool had_prior = false;
    bool changed = false;
    return navgrid_set_cell_ex(
        navgrid, x, y, cell, &prior, &had_prior, &changed)
        == NAVSYS_STATUS_OK;
}

navsys_status_t navgrid_set_cell_ex(
    navgrid_t* navgrid, int x, int y, const navcell_t* cell,
    navcell_t* out_prior, bool* out_had_prior, bool* out_changed) {
    if (!navgrid || !cell || !out_prior || !out_had_prior || !out_changed)
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    if (byul::navsys::internal::navgrid_callback_is_active(navgrid))
        return NAVSYS_STATUS_IN_PROGRESS;
    if (!navgrid_is_inside(navgrid, x, y)) return NAVSYS_STATUS_NOT_FOUND;
    const navsys_status_t validation = navcell_validate(cell);
    if (validation != NAVSYS_STATUS_OK) return validation;

    const coord_t key{x, y};
    const auto* current = static_cast<const navcell_t*>(
        coord_hash_get(navgrid->cell_map, &key));
    const bool present = current != nullptr;
    const navcell_t prior = present
        ? *current
        : navcell_t{TERRAIN_TYPE_NORMAL, 0};
    const bool changed = !present
        || prior.terrain != cell->terrain
        || prior.height != cell->height;
    if (changed) {
        const navsys_status_t status = coord_hash_upsert_copy(
            navgrid->cell_map, &key, cell, nullptr);
        if (status != NAVSYS_STATUS_OK) return status;
    }
    *out_prior = prior;
    *out_had_prior = present;
    *out_changed = changed;
    return NAVSYS_STATUS_OK;
}

int navgrid_fetch_cell(
    const navgrid_t* navgrid, int x, int y, navcell_t* out) {

    if (!navgrid || !out) return -1;

    coord_t c;
    coord_init_full(&c, x, y);

    if (!coord_hash_contains(navgrid->cell_map, &c)){
        return -1;
    }

    *out = *(navcell_t*) coord_hash_get(navgrid->cell_map, &c);
    return 0;
}

navsys_status_t navgrid_fetch_cell_ex(
    const navgrid_t* navgrid, int x, int y,
    navcell_t* out_cell, bool* out_present) {
    if (!navgrid || !out_cell || !out_present)
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    if (!navgrid_is_inside(navgrid, x, y)) return NAVSYS_STATUS_NOT_FOUND;
    const coord_t key{x, y};
    const auto* current = static_cast<const navcell_t*>(
        coord_hash_get(navgrid->cell_map, &key));
    if (current && navcell_validate(current) != NAVSYS_STATUS_OK)
        return NAVSYS_STATUS_CORRUPT_STATE;
    const navcell_t result = current
        ? *current
        : navcell_t{TERRAIN_TYPE_NORMAL, 0};
    *out_cell = result;
    *out_present = current != nullptr;
    return NAVSYS_STATUS_OK;
}

navsys_status_t navgrid_apply_blocked_overlay(
    navgrid_t* navgrid, const coord_t* coords, size_t count,
    navgrid_overlay_id_t* out_overlay, size_t* out_changed_count) {
    if (!out_overlay || !out_changed_count)
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    byul::navsys::internal::navgrid_tracked_overlay_t tracked{};
    size_t changed = 0;
    const navsys_status_t status = apply_blocked_overlay_impl(
        navgrid, coords, count, nullptr, nullptr, &tracked, &changed);
    if (status != NAVSYS_STATUS_OK) return status;
    *out_overlay = tracked.overlay;
    *out_changed_count = changed;
    return NAVSYS_STATUS_OK;
}

navsys_status_t navgrid_remove_blocked_overlay(
    navgrid_t* navgrid, navgrid_overlay_id_t overlay,
    size_t* out_changed_count) {
    return remove_blocked_overlay_impl(
        navgrid, overlay, 0, out_changed_count);
}

namespace byul::navsys::internal {

navsys_status_t navgrid_apply_blocked_overlay_tracked(
    navgrid_t* navgrid,
    const coord_t* coords,
    size_t count,
    navgrid_overlay_cancel_func cancel_func,
    void* cancel_userdata,
    navgrid_tracked_overlay_t* out_overlay,
    size_t* out_changed_count) {
    return apply_blocked_overlay_impl(
        navgrid, coords, count, cancel_func, cancel_userdata,
        out_overlay, out_changed_count);
}

navsys_status_t navgrid_remove_blocked_overlay_tracked(
    navgrid_t* navgrid,
    const navgrid_tracked_overlay_t* overlay,
    size_t* out_changed_count) {
    if (!overlay || overlay->owner_cookie == 0 || overlay->overlay == 0)
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    return remove_blocked_overlay_impl(
        navgrid, overlay->overlay, overlay->owner_cookie,
        out_changed_count);
}

navsys_status_t navgrid_replace_blocked_overlay_source(
    navgrid_t* navgrid,
    navgrid_overlay_source_kind kind,
    const void* source,
    const coord_t* coords,
    size_t count,
    size_t* out_changed_count) {
    return replace_source_overlay(
        navgrid, overlay_source_key_t{kind, source},
        coords, count, out_changed_count);
}

navsys_status_t navgrid_remove_blocked_overlay_source(
    navgrid_t* navgrid,
    navgrid_overlay_source_kind kind,
    const void* source,
    size_t* out_changed_count) {
    if (!navgrid || !source || !out_changed_count)
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    const auto* state = find_overlay_state(navgrid);
    if (!state) return NAVSYS_STATUS_NOT_FOUND;
    const auto found = state->sources.find(overlay_source_key_t{kind, source});
    if (found == state->sources.end()) return NAVSYS_STATUS_NOT_FOUND;
    return navgrid_remove_blocked_overlay(
        navgrid, found->second, out_changed_count);
}

navsys_status_t navgrid_clear_blocked_at_coord(
    navgrid_t* navgrid, int x, int y, bool* out_changed) {
    if (!navgrid || !out_changed) return NAVSYS_STATUS_INVALID_ARGUMENT;
    if (navgrid_callback_is_active(navgrid)) return NAVSYS_STATUS_IN_PROGRESS;

    const overlay_coord_key_t key = overlay_coord_key(x, y);
    const auto* current = find_overlay_state(navgrid);
    const bool before = navgrid_effectively_blocked(
        navgrid, current, key, x, y);
    try {
        if (current) {
            navgrid_overlay_state_t prepared = *current;
            for (auto layer = prepared.layers.begin();
                 layer != prepared.layers.end();) {
                layer->second.erase(key);
                if (layer->second.empty() && layer->first == 0)
                    layer = prepared.layers.erase(layer);
                else
                    ++layer;
            }
            const navsys_status_t status = commit_overlay_state(
                navgrid, std::move(prepared));
            if (status != NAVSYS_STATUS_OK) return status;
        }

        const coord_t coord{x, y};
        const auto* base = static_cast<const navcell_t*>(
            coord_hash_get(navgrid->cell_map, &coord));
        if (base && base->terrain == TERRAIN_TYPE_FORBIDDEN)
            (void)coord_hash_remove(navgrid->cell_map, &coord);
        const bool after = navgrid_base_is_blocked(navgrid, x, y)
            || overlay_state_contains(find_overlay_state(navgrid), key);
        *out_changed = before != after;
        return NAVSYS_STATUS_OK;
    } catch (const std::bad_alloc&) {
        return NAVSYS_STATUS_OUT_OF_MEMORY;
    } catch (...) {
        return NAVSYS_STATUS_CORRUPT_STATE;
    }
}

navsys_status_t navgrid_apply_open_overlay_atomic(
    navgrid_t* navgrid,
    const coord_t* coords,
    size_t count,
    bool dry_run,
    navgrid_overlay_cancel_func cancel_func,
    void* cancel_userdata,
    size_t* out_changed_count) {
    if (!navgrid || (!coords && count != 0) || !out_changed_count)
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    if (navgrid_callback_is_active(navgrid)) return NAVSYS_STATUS_IN_PROGRESS;
    navsys_status_t cancel_status = poll_overlay_cancel(
        cancel_func, cancel_userdata);
    if (cancel_status != NAVSYS_STATUS_OK) return cancel_status;
    for (size_t index = 0; index < count; ++index) {
        if (index != 0
            && index % overlay_cancel_poll_interval == 0) {
            cancel_status = poll_overlay_cancel(cancel_func, cancel_userdata);
            if (cancel_status != NAVSYS_STATUS_OK) return cancel_status;
        }
        if (!navgrid_is_inside(navgrid, coords[index].x, coords[index].y))
            return NAVSYS_STATUS_NOT_FOUND;
    }
    if (count == 0) {
        *out_changed_count = 0;
        return NAVSYS_STATUS_OK;
    }

    const auto* current = find_overlay_state(navgrid);
    try {
        navgrid_overlay_state_t prepared = current
            ? *current
            : navgrid_overlay_state_t{};
        const navsys_status_t owner_status = ensure_overlay_owner(prepared);
        if (owner_status != NAVSYS_STATUS_OK) return owner_status;
        const navgrid_overlay_id_t id = next_overlay_id(prepared);
        if (id == 0) return NAVSYS_STATUS_LIMIT_REACHED;

        overlay_coord_set_t opened;
        opened.reserve(count);
        for (size_t index = 0; index < count; ++index)
            opened.insert(overlay_coord_key(coords[index].x, coords[index].y));
        prepared.open_layers.emplace(id, std::move(opened));

        size_t changed = 0;
        const auto& layer = prepared.open_layers.find(id)->second;
        size_t examined = 0;
        for (const overlay_coord_key_t key : layer) {
            if (examined++ != 0
                && examined
                    % overlay_cancel_poll_interval == 0) {
                cancel_status = poll_overlay_cancel(
                    cancel_func, cancel_userdata);
                if (cancel_status != NAVSYS_STATUS_OK) return cancel_status;
            }
            const int x = static_cast<int32_t>(key >> 32);
            const int y = static_cast<int32_t>(key & 0xffffffffu);
            if (navgrid_effectively_blocked(navgrid, current, key, x, y)
                != navgrid_effectively_blocked(
                    navgrid, &prepared, key, x, y)) {
                ++changed;
            }
        }
        if (!dry_run) {
            cancel_status = poll_overlay_cancel(cancel_func, cancel_userdata);
            if (cancel_status != NAVSYS_STATUS_OK) return cancel_status;
            const navsys_status_t status = commit_overlay_state(
                navgrid, std::move(prepared));
            if (status != NAVSYS_STATUS_OK) return status;
        }
        *out_changed_count = changed;
        return NAVSYS_STATUS_OK;
    } catch (const std::bad_alloc&) {
        return NAVSYS_STATUS_OUT_OF_MEMORY;
    } catch (...) {
        return NAVSYS_STATUS_CORRUPT_STATE;
    }
}

} // namespace byul::navsys::internal

const coord_hash_t* navgrid_get_cell_map(const navgrid_t* navgrid) {
    return navgrid ? navgrid->cell_map : nullptr;
}

navsys_status_t navgrid_export_neighbors(
    const navgrid_t* navgrid,
    int x,
    int y,
    bool traversable_only,
    coord_t* out_coords,
    size_t capacity,
    size_t* out_count) {
    if (!navgrid || !valid_export_buffer(out_coords, capacity, out_count))
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    if (!navgrid_is_inside(navgrid, x, y)) return NAVSYS_STATUS_NOT_FOUND;
    coord_t neighbors[8]{};
    size_t count = 0;
    const navsys_status_t status = collect_immediate_neighbors(
        navgrid, x, y, traversable_only, false, neighbors, count);
    if (status != NAVSYS_STATUS_OK) return status;
    if (!out_coords) {
        *out_count = count;
        return NAVSYS_STATUS_OK;
    }
    if (capacity < count) {
        *out_count = count;
        return NAVSYS_STATUS_INCOMPLETE;
    }
    std::copy_n(neighbors, count, out_coords);
    *out_count = count;
    return NAVSYS_STATUS_OK;
}

navsys_status_t navgrid_export_neighbors_range(
    const navgrid_t* navgrid,
    int x,
    int y,
    int range,
    coord_t* out_coords,
    size_t capacity,
    size_t* out_count) {
    if (!navgrid || range < 0
        || !valid_export_buffer(out_coords, capacity, out_count)) {
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    }
    if (!navgrid_is_inside(navgrid, x, y)) return NAVSYS_STATUS_NOT_FOUND;
    if (navgrid->mode != NAVGRID_DIR_4 && navgrid->mode != NAVGRID_DIR_8)
        return NAVSYS_STATUS_CORRUPT_STATE;

    const int64_t radius = static_cast<int64_t>(range) + 1;
    size_t required = 0;
    for (int64_t dx = -radius; dx <= radius; ++dx) {
        for (int64_t dy = -radius; dy <= radius; ++dy) {
            if (!range_candidate(navgrid->mode, range, dx, dy)) continue;
            const int64_t nx = static_cast<int64_t>(x) + dx;
            const int64_t ny = static_cast<int64_t>(y) + dy;
            if (nx < std::numeric_limits<int>::min()
                || nx > std::numeric_limits<int>::max()
                || ny < std::numeric_limits<int>::min()
                || ny > std::numeric_limits<int>::max()
                || !navgrid_is_inside(
                    navgrid, static_cast<int>(nx), static_cast<int>(ny))) {
                continue;
            }
            if (required == std::numeric_limits<size_t>::max())
                return NAVSYS_STATUS_LIMIT_REACHED;
            ++required;
        }
    }
    if (!out_coords) {
        *out_count = required;
        return NAVSYS_STATUS_OK;
    }
    if (capacity < required) {
        *out_count = required;
        return NAVSYS_STATUS_INCOMPLETE;
    }
    size_t index = 0;
    for (int64_t dx = -radius; dx <= radius; ++dx) {
        for (int64_t dy = -radius; dy <= radius; ++dy) {
            if (!range_candidate(navgrid->mode, range, dx, dy)) continue;
            const int64_t nx = static_cast<int64_t>(x) + dx;
            const int64_t ny = static_cast<int64_t>(y) + dy;
            if (nx < std::numeric_limits<int>::min()
                || nx > std::numeric_limits<int>::max()
                || ny < std::numeric_limits<int>::min()
                || ny > std::numeric_limits<int>::max()
                || !navgrid_is_inside(
                    navgrid, static_cast<int>(nx), static_cast<int>(ny))) {
                continue;
            }
            out_coords[index++] = {
                static_cast<int>(nx), static_cast<int>(ny)};
        }
    }
    *out_count = required;
    return NAVSYS_STATUS_OK;
}

navsys_status_t navgrid_fetch_neighbor_at_degree(
    const navgrid_t* navgrid,
    int x,
    int y,
    double degree,
    coord_t* out_coord) {
    if (!navgrid || !out_coord || !std::isfinite(degree))
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    if (!navgrid_is_inside(navgrid, x, y)) return NAVSYS_STATUS_NOT_FOUND;
    coord_t neighbors[8]{};
    size_t count = 0;
    const navsys_status_t status = collect_immediate_neighbors(
        navgrid, x, y, false, false, neighbors, count);
    if (status != NAVSYS_STATUS_OK) return status;
    if (count == 0) return NAVSYS_STATUS_NOT_FOUND;

    double normalized = std::fmod(degree, 360.0);
    if (normalized < 0.0) normalized += 360.0;
    size_t best = 0;
    double best_difference = 361.0;
    double best_angle = 361.0;
    for (size_t index = 0; index < count; ++index) {
        double angle = std::atan2(
            static_cast<double>(neighbors[index].y - y),
            static_cast<double>(neighbors[index].x - x)) * 180.0 / 3.14159265358979323846;
        if (angle < 0.0) angle += 360.0;
        double difference = std::fabs(normalized - angle);
        if (difference > 180.0) difference = 360.0 - difference;
        if (difference < best_difference
            || (difference == best_difference && angle < best_angle)) {
            best = index;
            best_difference = difference;
            best_angle = angle;
        }
    }
    *out_coord = neighbors[best];
    return NAVSYS_STATUS_OK;
}

navsys_status_t navgrid_fetch_neighbor_at_goal(
    const navgrid_t* navgrid,
    const coord_t* center,
    const coord_t* goal,
    coord_t* out_coord) {
    if (!navgrid || !center || !goal || !out_coord
        || (center->x == goal->x && center->y == goal->y)) {
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    }
    const double degree = std::atan2(
        static_cast<double>(goal->y) - center->y,
        static_cast<double>(goal->x) - center->x) * 180.0 / 3.14159265358979323846;
    return navgrid_fetch_neighbor_at_degree(
        navgrid, center->x, center->y, degree, out_coord);
}

navsys_status_t navgrid_export_neighbors_at_degree_range(
    const navgrid_t* navgrid,
    const coord_t* center,
    const coord_t* goal,
    double start_deg,
    double end_deg,
    int range,
    coord_t* out_coords,
    size_t capacity,
    size_t* out_count) {
    if (!navgrid || !center || !goal || range < 0
        || !std::isfinite(start_deg) || !std::isfinite(end_deg)
        || (center->x == goal->x && center->y == goal->y)
        || !valid_export_buffer(out_coords, capacity, out_count)) {
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    }
    if (!navgrid_is_inside(navgrid, center->x, center->y))
        return NAVSYS_STATUS_NOT_FOUND;
    double center_degree = std::atan2(
        static_cast<double>(goal->y) - center->y,
        static_cast<double>(goal->x) - center->x) * 180.0 / 3.14159265358979323846;
    if (center_degree < 0.0) center_degree += 360.0;
    auto normalize = [](double value) {
        value = std::fmod(value, 360.0);
        return value < 0.0 ? value + 360.0 : value;
    };
    const double minimum = normalize(center_degree + start_deg);
    const double maximum = normalize(center_degree + end_deg);
    const bool wraps = minimum > maximum;

    auto included = [&](int64_t dx, int64_t dy) {
        if (dx == 0 && dy == 0) return false;
        double angle = std::atan2(
            static_cast<double>(dy), static_cast<double>(dx))
            * 180.0 / 3.14159265358979323846;
        if (angle < 0.0) angle += 360.0;
        return wraps ? (angle >= minimum || angle <= maximum)
                     : (angle >= minimum && angle <= maximum);
    };

    size_t required = 0;
    for (int64_t dx = -static_cast<int64_t>(range); dx <= range; ++dx) {
        for (int64_t dy = -static_cast<int64_t>(range); dy <= range; ++dy) {
            if (!included(dx, dy)) continue;
            const int64_t nx = static_cast<int64_t>(center->x) + dx;
            const int64_t ny = static_cast<int64_t>(center->y) + dy;
            if (nx < std::numeric_limits<int>::min()
                || nx > std::numeric_limits<int>::max()
                || ny < std::numeric_limits<int>::min()
                || ny > std::numeric_limits<int>::max()
                || !navgrid_is_inside(
                    navgrid, static_cast<int>(nx), static_cast<int>(ny))) {
                continue;
            }
            ++required;
        }
    }
    if (!out_coords) {
        *out_count = required;
        return NAVSYS_STATUS_OK;
    }
    if (capacity < required) {
        *out_count = required;
        return NAVSYS_STATUS_INCOMPLETE;
    }
    size_t index = 0;
    for (int64_t dx = -static_cast<int64_t>(range); dx <= range; ++dx) {
        for (int64_t dy = -static_cast<int64_t>(range); dy <= range; ++dy) {
            if (!included(dx, dy)) continue;
            const int64_t nx = static_cast<int64_t>(center->x) + dx;
            const int64_t ny = static_cast<int64_t>(center->y) + dy;
            if (nx < std::numeric_limits<int>::min()
                || nx > std::numeric_limits<int>::max()
                || ny < std::numeric_limits<int>::min()
                || ny > std::numeric_limits<int>::max()
                || !navgrid_is_inside(
                    navgrid, static_cast<int>(nx), static_cast<int>(ny))) {
                continue;
            }
            out_coords[index++] = {
                static_cast<int>(nx), static_cast<int>(ny)};
        }
    }
    *out_count = required;
    return NAVSYS_STATUS_OK;
}

navsys_status_t navgrid_export_cells(
    const navgrid_t* navgrid,
    navgrid_cell_entry_t* out_entries,
    size_t capacity,
    size_t* out_count) {
    if (!navgrid || !navgrid->cell_map
        || !valid_export_buffer(out_entries, capacity, out_count)) {
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    }
    const navgrid_overlay_state_t* overlays = find_overlay_state(navgrid);
    size_t required = coord_hash_size(navgrid->cell_map);
    if (overlays) {
        for (const auto& [id, coords] : overlays->layers) {
            for (const overlay_coord_key_t key : coords) {
                const coord_t coord{
                    static_cast<int32_t>(key >> 32),
                    static_cast<int32_t>(key & 0xffffffffu)};
                if (!coord_hash_contains(navgrid->cell_map, &coord)
                    && overlay_key_has_canonical_owner(overlays, id, key)) {
                    if (required == std::numeric_limits<size_t>::max())
                        return NAVSYS_STATUS_LIMIT_REACHED;
                    ++required;
                }
            }
        }
        for (const auto& [id, coords] : overlays->open_layers) {
            for (const overlay_coord_key_t key : coords) {
                const coord_t coord{
                    static_cast<int32_t>(key >> 32),
                    static_cast<int32_t>(key & 0xffffffffu)};
                if (!coord_hash_contains(navgrid->cell_map, &coord)
                    && overlay_key_has_canonical_owner(overlays, id, key)) {
                    if (required == std::numeric_limits<size_t>::max())
                        return NAVSYS_STATUS_LIMIT_REACHED;
                    ++required;
                }
            }
        }
    }
    if (!out_entries) {
        *out_count = required;
        return NAVSYS_STATUS_OK;
    }
    if (capacity < required) {
        *out_count = required;
        return NAVSYS_STATUS_INCOMPLETE;
    }

    cell_validation_context_t validation{};
    coord_hash_foreach(
        const_cast<coord_hash_t*>(navgrid->cell_map),
        validate_materialized_cell,
        &validation);
    if (!validation.valid) return NAVSYS_STATUS_CORRUPT_STATE;

    cell_fill_context_t fill{navgrid, overlays, out_entries, 0};
    coord_hash_foreach(
        const_cast<coord_hash_t*>(navgrid->cell_map),
        fill_materialized_cell,
        &fill);
    if (overlays) {
        for (const auto& [id, coords] : overlays->layers) {
            for (const overlay_coord_key_t key : coords) {
                const coord_t coord{
                    static_cast<int32_t>(key >> 32),
                    static_cast<int32_t>(key & 0xffffffffu)};
                if (coord_hash_contains(navgrid->cell_map, &coord)
                    || !overlay_key_has_canonical_owner(overlays, id, key)) {
                    continue;
                }
                out_entries[fill.index++] = {
                    coord,
                    navcell_t{TERRAIN_TYPE_NORMAL, 0},
                    false,
                    navgrid_effectively_blocked(
                        navgrid, overlays, key, coord.x, coord.y)};
            }
        }
        for (const auto& [id, coords] : overlays->open_layers) {
            for (const overlay_coord_key_t key : coords) {
                const coord_t coord{
                    static_cast<int32_t>(key >> 32),
                    static_cast<int32_t>(key & 0xffffffffu)};
                if (coord_hash_contains(navgrid->cell_map, &coord)
                    || !overlay_key_has_canonical_owner(overlays, id, key)) {
                    continue;
                }
                out_entries[fill.index++] = {
                    coord,
                    navcell_t{TERRAIN_TYPE_NORMAL, 0},
                    false,
                    navgrid_effectively_blocked(
                        navgrid, overlays, key, coord.x, coord.y)};
            }
        }
    }
    std::sort(out_entries, out_entries + required, cell_entry_less);
    *out_count = required;
    return NAVSYS_STATUS_OK;
}

namespace {

coord_list_t* create_coord_list(const coord_t* coords, size_t count) {
    coord_list_t* list = nullptr;
    if (coord_list_create_ex(&list) != NAVSYS_STATUS_OK) return nullptr;
    if (coord_list_reserve(list, count) != NAVSYS_STATUS_OK) {
        coord_list_destroy(list);
        return nullptr;
    }
    for (size_t index = 0; index < count; ++index) {
        if (coord_list_push_back_ex(list, &coords[index]) != NAVSYS_STATUS_OK) {
            coord_list_destroy(list);
            return nullptr;
        }
    }
    return list;
}

template <typename Exporter>
coord_list_t* copy_exported_coords(Exporter exporter) {
    size_t count = 0;
    if (exporter(nullptr, 0, &count) != NAVSYS_STATUS_OK) return nullptr;
    try {
        std::vector<coord_t> coords(count);
        if (count != 0
            && exporter(coords.data(), coords.size(), &count)
                != NAVSYS_STATUS_OK) {
            return nullptr;
        }
        return create_coord_list(coords.data(), count);
    } catch (...) {
        return nullptr;
    }
}

} // namespace

coord_list_t* navgrid_copy_neighbors(
    const navgrid_t* navgrid, int x, int y) {
    coord_t neighbors[8]{};
    size_t count = 0;
    if (collect_immediate_neighbors(
            navgrid, x, y, true, true, neighbors, count)
        != NAVSYS_STATUS_OK) {
        return nullptr;
    }
    return create_coord_list(neighbors, count);
}

coord_list_t* navgrid_copy_neighbors_all(
    const navgrid_t* navgrid, int x, int y) {
    coord_t neighbors[8]{};
    size_t count = 0;
    if (collect_immediate_neighbors(
            navgrid, x, y, false, true, neighbors, count)
        != NAVSYS_STATUS_OK) {
        return nullptr;
    }
    return create_coord_list(neighbors, count);
}

coord_list_t* navgrid_copy_neighbors_all_range(
    navgrid_t* navgrid, int x, int y, int range) {
    return copy_exported_coords(
        [&](coord_t* output, size_t capacity, size_t* count) {
            return navgrid_export_neighbors_range(
                navgrid, x, y, range, output, capacity, count);
        });
}

coord_t* navgrid_copy_neighbor_at_degree(
    const navgrid_t* navgrid, int x, int y, double degree) {
    coord_t result{};
    if (navgrid_fetch_neighbor_at_degree(navgrid, x, y, degree, &result)
        != NAVSYS_STATUS_OK) {
        return nullptr;
    }
    return coord_create_full(result.x, result.y);
}

coord_t* navgrid_copy_neighbor_at_goal(
    const navgrid_t* navgrid, const coord_t* center, const coord_t* goal) {
    coord_t result{};
    if (navgrid_fetch_neighbor_at_goal(navgrid, center, goal, &result)
        != NAVSYS_STATUS_OK) {
        return nullptr;
    }
    return coord_create_full(result.x, result.y);
}

coord_list_t* navgrid_copy_neighbors_at_degree_range(
    const navgrid_t* navgrid,
    const coord_t* center, const coord_t* goal,
    double start_deg, double end_deg,
    int range) {
    return copy_exported_coords(
        [&](coord_t* output, size_t capacity, size_t* count) {
            return navgrid_export_neighbors_at_degree_range(
                navgrid, center, goal, start_deg, end_deg, range,
                output, capacity, count);
        });
}
