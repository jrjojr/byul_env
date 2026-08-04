#include "obstacle_core.h"
#include "internal/obstacle_private.hpp"
#include <stdlib.h>
#include <string.h>
#include <cmath>
#include <cstdint>
#include <limits>
#include <new>
#include <vector>
#include "../navgrid/internal/navgrid_overlay.hpp"

namespace {

constexpr uint32_t obstacle_abi1_version = 1;
constexpr uint64_t obstacle_abi1_fingerprint =
    UINT64_C(0x4f42535401000018);

struct obstacle_extent_t {
    int64_t min_x;
    int64_t max_x;
    int64_t min_y;
    int64_t max_y;
};

obstacle_extent_t obstacle_extent(
    int x0, int y0, int width, int height) {
    const int64_t x1 = static_cast<int64_t>(x0) + width;
    const int64_t y1 = static_cast<int64_t>(y0) + height;
    return {
        width >= 0 ? x0 : x1,
        width >= 0 ? x1 : x0,
        height >= 0 ? y0 : y1,
        height >= 0 ? y1 : y0
    };
}

bool obstacle_extent_contains(
    const obstacle_extent_t& extent, int64_t x, int64_t y) {
    return x >= extent.min_x && x < extent.max_x
        && y >= extent.min_y && y < extent.max_y;
}

bool obstacle_rebuild_extent(
    obstacle_t* obstacle,
    int x0,
    int y0,
    int width,
    int height,
    bool translate) {
    if (!obstacle || !obstacle->blocked) return false;

    coord_hash_t* replacement = nullptr;
    try {
        const size_t count = coord_hash_size(obstacle->blocked);
        std::vector<coord_t> coordinates(count);
        size_t exported = 0;
        if (coord_hash_export_keys(
                obstacle->blocked,
                coordinates.empty() ? nullptr : coordinates.data(),
                coordinates.size(),
                &exported) != NAVSYS_STATUS_OK) {
            return false;
        }

        replacement = coord_hash_create();
        if (!replacement) return false;

        const int64_t delta_x = translate
            ? static_cast<int64_t>(x0) - obstacle->x0
            : 0;
        const int64_t delta_y = translate
            ? static_cast<int64_t>(y0) - obstacle->y0
            : 0;
        const obstacle_extent_t extent =
            obstacle_extent(x0, y0, width, height);

        for (size_t index = 0; index < exported; ++index) {
            const int64_t translated_x =
                static_cast<int64_t>(coordinates[index].x) + delta_x;
            const int64_t translated_y =
                static_cast<int64_t>(coordinates[index].y) + delta_y;
            if (translated_x < std::numeric_limits<int>::min()
                || translated_x > std::numeric_limits<int>::max()
                || translated_y < std::numeric_limits<int>::min()
                || translated_y > std::numeric_limits<int>::max()) {
                coord_hash_destroy(replacement);
                return false;
            }
            if (!obstacle_extent_contains(
                    extent, translated_x, translated_y)) {
                continue;
            }

            const coord_t translated = {
                static_cast<int>(translated_x),
                static_cast<int>(translated_y)
            };
            if (coord_hash_upsert_copy(
                    replacement, &translated, nullptr, nullptr)
                != NAVSYS_STATUS_OK) {
                coord_hash_destroy(replacement);
                return false;
            }
        }

        coord_hash_t* previous = obstacle->blocked;
        obstacle->blocked = replacement;
        obstacle->x0 = x0;
        obstacle->y0 = y0;
        obstacle->width = width;
        obstacle->height = height;
        coord_hash_destroy(previous);
        return true;
    } catch (...) {
        coord_hash_destroy(replacement);
        return false;
    }
}

} // namespace

uint32_t obstacle_get_abi_version(void) {
    return BYUL_OBSTACLE_ABI_VERSION;
}

uint64_t obstacle_get_abi_fingerprint(void) {
    return BYUL_OBSTACLE_ABI_FINGERPRINT;
}

navsys_status_t obstacle_check_abi(
    uint32_t expected_version,
    uint64_t expected_fingerprint,
    obstacle_abi_mismatch_t* out_mismatch) {
    if (!out_mismatch) return NAVSYS_STATUS_INVALID_ARGUMENT;
    uint64_t supported_fingerprint = 0;
    if (expected_version == BYUL_OBSTACLE_ABI_VERSION) {
        supported_fingerprint = BYUL_OBSTACLE_ABI_FINGERPRINT;
    } else if (expected_version == obstacle_abi1_version) {
        supported_fingerprint = obstacle_abi1_fingerprint;
    } else {
        *out_mismatch = OBSTACLE_ABI_VERSION_MISMATCH;
        return NAVSYS_STATUS_UNSUPPORTED;
    }
    if (expected_fingerprint != supported_fingerprint) {
        *out_mismatch = OBSTACLE_ABI_FINGERPRINT_MISMATCH;
        return NAVSYS_STATUS_UNSUPPORTED;
    }
    *out_mismatch = OBSTACLE_ABI_MATCH;
    return NAVSYS_STATUS_OK;
}

size_t obstacle_sizeof(void) {
    return sizeof(obstacle_t);
}

size_t obstacle_alignof(void) {
    return alignof(obstacle_t);
}

navsys_status_t obstacle_create_checked(
    int32_t x0,
    int32_t y0,
    int32_t width,
    int32_t height,
    obstacle_t** out_obstacle) {
    if (!out_obstacle) return NAVSYS_STATUS_INVALID_ARGUMENT;

    obstacle_t* result = static_cast<obstacle_t*>(malloc(sizeof(obstacle_t)));
    if (!result) return NAVSYS_STATUS_OUT_OF_MEMORY;

    result->x0 = static_cast<int>(x0);
    result->y0 = static_cast<int>(y0);
    result->width = static_cast<int>(width);
    result->height = static_cast<int>(height);
    result->blocked = nullptr;
    try {
        result->blocked = coord_hash_create();
        if (!result->blocked) {
            free(result);
            return NAVSYS_STATUS_OUT_OF_MEMORY;
        }
        *out_obstacle = result;
        return NAVSYS_STATUS_OK;
    } catch (const std::bad_alloc&) {
        coord_hash_destroy(result->blocked);
        free(result);
        return NAVSYS_STATUS_OUT_OF_MEMORY;
    } catch (...) {
        coord_hash_destroy(result->blocked);
        free(result);
        return NAVSYS_STATUS_CORRUPT_STATE;
    }
}

obstacle_t* obstacle_create() {
    obstacle_t* result = nullptr;
    return obstacle_create_checked(0, 0, 0, 0, &result)
            == NAVSYS_STATUS_OK
        ? result
        : nullptr;
}

obstacle_t* obstacle_create_full(
    int x0, int y0, int width, int height) {
    obstacle_t* result = nullptr;
    return obstacle_create_checked(x0, y0, width, height, &result)
            == NAVSYS_STATUS_OK
        ? result
        : nullptr;
}

void obstacle_clear(obstacle_t* obstacle) {
    if (!obstacle || !obstacle->blocked) return;

    coord_hash_clear(obstacle->blocked);
}

void obstacle_destroy(obstacle_t* obstacle) {
    if (!obstacle) return;
    coord_hash_destroy(obstacle->blocked);
    free(obstacle);
}

navsys_status_t obstacle_copy_checked(
    const obstacle_t* source,
    obstacle_t** out_obstacle) {
    if (!source || !out_obstacle) return NAVSYS_STATUS_INVALID_ARGUMENT;
    if (!source->blocked) return NAVSYS_STATUS_CORRUPT_STATE;

    coord_hash_t* copied_blocked = nullptr;
    try {
        const navsys_status_t copy_status =
            coord_hash_copy_ex(source->blocked, &copied_blocked);
        if (copy_status != NAVSYS_STATUS_OK) {
            return copy_status == NAVSYS_STATUS_OUT_OF_MEMORY
                ? NAVSYS_STATUS_OUT_OF_MEMORY
                : NAVSYS_STATUS_CORRUPT_STATE;
        }

        obstacle_t* result =
            static_cast<obstacle_t*>(malloc(sizeof(obstacle_t)));
        if (!result) {
            coord_hash_destroy(copied_blocked);
            return NAVSYS_STATUS_OUT_OF_MEMORY;
        }

        result->x0 = source->x0;
        result->y0 = source->y0;
        result->width = source->width;
        result->height = source->height;
        result->blocked = copied_blocked;
        *out_obstacle = result;
        return NAVSYS_STATUS_OK;
    } catch (const std::bad_alloc&) {
        coord_hash_destroy(copied_blocked);
        return NAVSYS_STATUS_OUT_OF_MEMORY;
    } catch (...) {
        coord_hash_destroy(copied_blocked);
        return NAVSYS_STATUS_CORRUPT_STATE;
    }
}

obstacle_t* obstacle_copy(const obstacle_t* obstacle) {
    obstacle_t* result = nullptr;
    return obstacle_copy_checked(obstacle, &result) == NAVSYS_STATUS_OK
        ? result
        : nullptr;
}

bool obstacle_equal(const obstacle_t* a, const obstacle_t* b) {
    if (!a || !b) return false;
    return a->x0 == b->x0 &&
           a->y0 == b->y0 &&
           a->width == b->width &&
           a->height == b->height &&
           coord_hash_equal(a->blocked, b->blocked);
}

uint32_t obstacle_hash(const obstacle_t* obstacle) {
    if (!obstacle) return 0;
    uint32_t hash = 17;
    hash = 31 * hash + obstacle->x0;
    hash = 31 * hash + obstacle->y0;
    hash = 31 * hash + obstacle->width;
    hash = 31 * hash + obstacle->height;
    hash = 31 * hash + coord_hash_hash(obstacle->blocked);
    return hash;
}

void obstacle_set_origin(obstacle_t* obstacle, int x0, int y0) {
    if (!obstacle) return;
    (void)obstacle_rebuild_extent(
        obstacle,
        x0,
        y0,
        obstacle->width,
        obstacle->height,
        true);
}

void obstacle_fetch_origin(
    const obstacle_t* obstacle, int* out_x0, int* out_y0) {
    if (!obstacle) return;
    if (out_x0) *out_x0 = obstacle->x0;
    if (out_y0) *out_y0 = obstacle->y0;
}

navsys_status_t obstacle_apply_to_navgrid_checked(
    const obstacle_t* obstacle,
    navgrid_t* navgrid,
    const obstacle_navgrid_apply_options_t* options,
    obstacle_navgrid_overlay_token_t* out_overlay,
    size_t* out_changed_count) {
    if (!obstacle || !navgrid || !out_overlay || !out_changed_count)
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    if (!obstacle->blocked) return NAVSYS_STATUS_CORRUPT_STATE;

    obstacle_navgrid_cancel_func cancel_func = nullptr;
    void* cancel_userdata = nullptr;
    if (options) {
        if (options->struct_size < sizeof(*options))
            return NAVSYS_STATUS_INVALID_ARGUMENT;
        if (options->abi_version
            != OBSTACLE_NAVGRID_APPLY_OPTIONS_ABI_VERSION) {
            return NAVSYS_STATUS_UNSUPPORTED;
        }
        if (options->merge_policy
            != OBSTACLE_NAVGRID_MERGE_PRESERVE_BASE) {
            return NAVSYS_STATUS_UNSUPPORTED;
        }
        cancel_func = options->cancel_func;
        cancel_userdata = options->cancel_userdata;
    }

    if (cancel_func) {
        try {
            if (cancel_func(cancel_userdata))
                return NAVSYS_STATUS_CANCELLED;
        } catch (...) {
            return NAVSYS_STATUS_CALLBACK_FAILED;
        }
    }
    try {
        const size_t count = coord_hash_size(obstacle->blocked);
        std::vector<coord_t> coords(count);
        size_t exported = 0;
        const navsys_status_t export_status = coord_hash_export_keys(
            obstacle->blocked,
            coords.empty() ? nullptr : coords.data(),
            coords.size(),
            &exported);
        if (export_status != NAVSYS_STATUS_OK) return export_status;

        byul::navsys::internal::navgrid_tracked_overlay_t tracked{};
        size_t changed = 0;
        const navsys_status_t status =
            byul::navsys::internal::navgrid_apply_blocked_overlay_tracked(
                navgrid,
                coords.empty() ? nullptr : coords.data(),
                exported,
                cancel_func,
                cancel_userdata,
                &tracked,
                &changed);
        if (status != NAVSYS_STATUS_OK) return status;
        const obstacle_navgrid_overlay_token_t result{
            sizeof(obstacle_navgrid_overlay_token_t),
            OBSTACLE_NAVGRID_OVERLAY_TOKEN_ABI_VERSION,
            tracked.owner_cookie,
            tracked.overlay
        };
        *out_overlay = result;
        *out_changed_count = changed;
        return NAVSYS_STATUS_OK;
    } catch (const std::bad_alloc&) {
        return NAVSYS_STATUS_OUT_OF_MEMORY;
    } catch (...) {
        return NAVSYS_STATUS_CORRUPT_STATE;
    }
}

navsys_status_t obstacle_remove_from_navgrid_checked(
    navgrid_t* navgrid,
    obstacle_navgrid_overlay_token_t* overlay,
    size_t* out_changed_count) {
    if (!navgrid || !overlay || !out_changed_count)
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    if (overlay->struct_size != sizeof(*overlay))
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    if (overlay->abi_version
        != OBSTACLE_NAVGRID_OVERLAY_TOKEN_ABI_VERSION) {
        return NAVSYS_STATUS_UNSUPPORTED;
    }
    if (overlay->owner_cookie == 0 || overlay->overlay == 0)
        return NAVSYS_STATUS_INVALIDATED;

    const byul::navsys::internal::navgrid_tracked_overlay_t tracked{
        overlay->owner_cookie,
        overlay->overlay
    };
    size_t changed = 0;
    const navsys_status_t status =
        byul::navsys::internal::navgrid_remove_blocked_overlay_tracked(
            navgrid, &tracked, &changed);
    if (status != NAVSYS_STATUS_OK) return status;
    *overlay = obstacle_navgrid_overlay_token_t{
        sizeof(obstacle_navgrid_overlay_token_t),
        OBSTACLE_NAVGRID_OVERLAY_TOKEN_ABI_VERSION,
        0,
        0
    };
    *out_changed_count = changed;
    return NAVSYS_STATUS_OK;
}

void obstacle_apply_to_navgrid(const obstacle_t* obstacle, navgrid_t* navgrid) {
    if (!obstacle || !navgrid) return;
    try {
        const size_t count = coord_hash_size(obstacle->blocked);
        std::vector<coord_t> coords(count);
        size_t exported = 0;
        if (coord_hash_export_keys(
                obstacle->blocked,
                coords.empty() ? nullptr : coords.data(),
                coords.size(),
                &exported) != NAVSYS_STATUS_OK) {
            return;
        }
        size_t changed = 0;
        (void)byul::navsys::internal::navgrid_replace_blocked_overlay_source(
            navgrid,
            byul::navsys::internal::navgrid_overlay_source_kind::obstacle,
            obstacle,
            coords.empty() ? nullptr : coords.data(),
            exported,
            &changed);
    } catch (...) {
        return;
    }
}

void obstacle_remove_from_navgrid(const obstacle_t* obstacle, navgrid_t* navgrid) {
    if (!obstacle || !navgrid) return;
    size_t changed = 0;
    (void)byul::navsys::internal::navgrid_remove_blocked_overlay_source(
        navgrid,
        byul::navsys::internal::navgrid_overlay_source_kind::obstacle,
        obstacle,
        &changed);
}

int obstacle_get_width(const obstacle_t* obs) {
     return obs ? obs->width : 0; 
}

void obstacle_set_width(obstacle_t* obs, int w) {
    if (!obs) return;
    (void)obstacle_rebuild_extent(
        obs, obs->x0, obs->y0, w, obs->height, false);
}

int obstacle_get_height(const obstacle_t* obs) {
    return obs ? obs->height : 0; 
}

void obstacle_set_height(obstacle_t* obs, int h) {
    if (!obs) return;
    (void)obstacle_rebuild_extent(
        obs, obs->x0, obs->y0, obs->width, h, false);
}

navsys_status_t obstacle_set_blocked(
    obstacle_t* obstacle,
    int32_t x,
    int32_t y,
    bool blocked,
    bool* out_changed) {
    if (!obstacle || !out_changed) return NAVSYS_STATUS_INVALID_ARGUMENT;
    if (!obstacle->blocked) return NAVSYS_STATUS_CORRUPT_STATE;
    if (!obstacle_is_inside(obstacle, x, y)) return NAVSYS_STATUS_NOT_FOUND;

    const coord_t key{static_cast<int>(x), static_cast<int>(y)};
    try {
        const bool current = coord_hash_contains(obstacle->blocked, &key);
        if (current == blocked) {
            *out_changed = false;
            return NAVSYS_STATUS_OK;
        }
        if (blocked) {
            bool inserted = false;
            const navsys_status_t status = coord_hash_upsert_copy(
                obstacle->blocked, &key, nullptr, &inserted);
            if (status != NAVSYS_STATUS_OK) return status;
            if (!inserted) return NAVSYS_STATUS_CORRUPT_STATE;
        } else if (!coord_hash_remove(obstacle->blocked, &key)) {
            return NAVSYS_STATUS_CORRUPT_STATE;
        }
        *out_changed = true;
        return NAVSYS_STATUS_OK;
    } catch (const std::bad_alloc&) {
        return NAVSYS_STATUS_OUT_OF_MEMORY;
    } catch (...) {
        return NAVSYS_STATUS_CORRUPT_STATE;
    }
}

navsys_status_t obstacle_export_blocked(
    const obstacle_t* obstacle,
    coord_t* out_coords,
    size_t capacity,
    size_t* out_count) {
    if (!obstacle || !out_count) return NAVSYS_STATUS_INVALID_ARGUMENT;
    if (!obstacle->blocked) return NAVSYS_STATUS_CORRUPT_STATE;
    try {
        return coord_hash_export_keys(
            obstacle->blocked, out_coords, capacity, out_count);
    } catch (const std::bad_alloc&) {
        return NAVSYS_STATUS_OUT_OF_MEMORY;
    } catch (...) {
        return NAVSYS_STATUS_CORRUPT_STATE;
    }
}

bool obstacle_block_coord(obstacle_t* obs, int x, int y) {
    bool changed = false;
    return obstacle_set_blocked(obs, x, y, true, &changed)
        == NAVSYS_STATUS_OK;
}

bool obstacle_unblock_coord(obstacle_t* obs, int x, int y) {
    bool changed = false;
    return obstacle_set_blocked(obs, x, y, false, &changed)
            == NAVSYS_STATUS_OK
        && changed;
}

bool obstacle_is_inside(const obstacle_t* obs, int x, int y) {
    if (!obs) return false;
    return obstacle_extent_contains(
        obstacle_extent(obs->x0, obs->y0, obs->width, obs->height),
        x,
        y);
}

const coord_hash_t* obstacle_get_blocked_coords(const obstacle_t* obs) {
    return obs ? obs->blocked : nullptr;
}

bool obstacle_is_coord_blocked(const obstacle_t* obstacle, int x, int y){
    if (!obstacle) return false;

    coord_t tmp = {x, y};

    return coord_hash_contains(obstacle->blocked, &tmp);
}
