#include "maze.h"
#include "maze_core.h"
#include "internal/maze_private.hpp"
#include <stdlib.h>
#include <string.h>
#include <cstdint>
#include <limits>
#include <new>
#include <vector>
#include "../navgrid/internal/navgrid_overlay.hpp"

namespace {

constexpr uint32_t maze_abi1_version = 1;
constexpr uint64_t maze_abi1_fingerprint = UINT64_C(0x4d415a4501000018);

struct maze_extent_t {
    int64_t min_x;
    int64_t max_x;
    int64_t min_y;
    int64_t max_y;
};

maze_extent_t maze_extent(
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

bool maze_extent_contains(
    const maze_extent_t& extent, int64_t x, int64_t y) {
    return x >= extent.min_x && x < extent.max_x
        && y >= extent.min_y && y < extent.max_y;
}

navsys_status_t maze_create_signed_checked(
    int x0, int y0, int width, int height, maze_t** out_maze) {
    if (!out_maze) return NAVSYS_STATUS_INVALID_ARGUMENT;
    maze_t* candidate = static_cast<maze_t*>(malloc(sizeof(maze_t)));
    if (!candidate) return NAVSYS_STATUS_OUT_OF_MEMORY;
    candidate->x0 = x0;
    candidate->y0 = y0;
    candidate->width = width;
    candidate->height = height;
    candidate->blocked = coord_hash_create();
    if (!candidate->blocked) {
        free(candidate);
        return NAVSYS_STATUS_OUT_OF_MEMORY;
    }
    *out_maze = candidate;
    return NAVSYS_STATUS_OK;
}

navsys_status_t maze_translate_impl(
    maze_t* maze, int64_t delta_x, int64_t delta_y) {
    if (!maze) return NAVSYS_STATUS_INVALID_ARGUMENT;
    if (!maze->blocked) return NAVSYS_STATUS_CORRUPT_STATE;

    const int64_t translated_x0 = static_cast<int64_t>(maze->x0) + delta_x;
    const int64_t translated_y0 = static_cast<int64_t>(maze->y0) + delta_y;
    if (translated_x0 < std::numeric_limits<int>::min()
        || translated_x0 > std::numeric_limits<int>::max()
        || translated_y0 < std::numeric_limits<int>::min()
        || translated_y0 > std::numeric_limits<int>::max()) {
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    }

    coord_hash_t* replacement = nullptr;
    try {
        const size_t count = coord_hash_size(maze->blocked);
        std::vector<coord_t> coordinates(count);
        size_t exported = 0;
        const navsys_status_t export_status = coord_hash_export_keys(
            maze->blocked,
            coordinates.empty() ? nullptr : coordinates.data(),
            coordinates.size(),
            &exported);
        if (export_status != NAVSYS_STATUS_OK) return export_status;

        const maze_extent_t source_extent = maze_extent(
            maze->x0, maze->y0, maze->width, maze->height);
        replacement = coord_hash_create();
        if (!replacement) return NAVSYS_STATUS_OUT_OF_MEMORY;

        for (size_t index = 0; index < exported; ++index) {
            const coord_t& coordinate = coordinates[index];
            if (!maze_extent_contains(
                    source_extent, coordinate.x, coordinate.y)) {
                coord_hash_destroy(replacement);
                return NAVSYS_STATUS_CORRUPT_STATE;
            }
            const int64_t translated_x =
                static_cast<int64_t>(coordinate.x) + delta_x;
            const int64_t translated_y =
                static_cast<int64_t>(coordinate.y) + delta_y;
            if (translated_x < std::numeric_limits<int>::min()
                || translated_x > std::numeric_limits<int>::max()
                || translated_y < std::numeric_limits<int>::min()
                || translated_y > std::numeric_limits<int>::max()) {
                coord_hash_destroy(replacement);
                return NAVSYS_STATUS_INVALID_ARGUMENT;
            }
            const coord_t translated{
                static_cast<int>(translated_x),
                static_cast<int>(translated_y)
            };
            const navsys_status_t insert_status = coord_hash_upsert_copy(
                replacement, &translated, nullptr, nullptr);
            if (insert_status != NAVSYS_STATUS_OK) {
                coord_hash_destroy(replacement);
                return insert_status;
            }
        }

        coord_hash_t* previous = maze->blocked;
        maze->blocked = replacement;
        maze->x0 = static_cast<int>(translated_x0);
        maze->y0 = static_cast<int>(translated_y0);
        coord_hash_destroy(previous);
        return NAVSYS_STATUS_OK;
    } catch (const std::bad_alloc&) {
        coord_hash_destroy(replacement);
        return NAVSYS_STATUS_OUT_OF_MEMORY;
    } catch (...) {
        coord_hash_destroy(replacement);
        return NAVSYS_STATUS_CORRUPT_STATE;
    }
}

} // namespace

uint32_t byul_maze_get_abi_version(void) {
    return BYUL_MAZE_ABI_VERSION;
}

uint64_t byul_maze_get_abi_fingerprint(void) {
    return BYUL_MAZE_ABI_FINGERPRINT;
}

navsys_status_t byul_maze_check_abi(
    uint32_t expected_version,
    uint64_t expected_fingerprint,
    byul_maze_abi_mismatch_t* out_mismatch) {
    if (!out_mismatch) return NAVSYS_STATUS_INVALID_ARGUMENT;
    uint64_t supported_fingerprint = 0;
    if (expected_version == BYUL_MAZE_ABI_VERSION) {
        supported_fingerprint = BYUL_MAZE_ABI_FINGERPRINT;
    } else if (expected_version == maze_abi1_version) {
        supported_fingerprint = maze_abi1_fingerprint;
    } else {
        *out_mismatch = BYUL_MAZE_ABI_VERSION_MISMATCH;
        return NAVSYS_STATUS_UNSUPPORTED;
    }
    if (expected_fingerprint != supported_fingerprint) {
        *out_mismatch = BYUL_MAZE_ABI_FINGERPRINT_MISMATCH;
        return NAVSYS_STATUS_UNSUPPORTED;
    }
    *out_mismatch = BYUL_MAZE_ABI_MATCH;
    return NAVSYS_STATUS_OK;
}

size_t byul_maze_sizeof(void) {
    return sizeof(maze_t);
}

size_t byul_maze_alignof(void) {
    return alignof(maze_t);
}

navsys_status_t byul_maze_create(
    const byul_maze_extent_t* extent, maze_t** out_maze) {
    if (!extent || !out_maze) return NAVSYS_STATUS_INVALID_ARGUMENT;
    if (extent->width > static_cast<uint32_t>(std::numeric_limits<int>::max())
        || extent->height
            > static_cast<uint32_t>(std::numeric_limits<int>::max())) {
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    }
    const int64_t max_x = static_cast<int64_t>(extent->origin_x)
        + extent->width;
    const int64_t max_y = static_cast<int64_t>(extent->origin_y)
        + extent->height;
    if (max_x > static_cast<int64_t>(std::numeric_limits<int>::max()) + 1
        || max_y
            > static_cast<int64_t>(std::numeric_limits<int>::max()) + 1) {
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    }
    return maze_create_signed_checked(
        extent->origin_x,
        extent->origin_y,
        static_cast<int>(extent->width),
        static_cast<int>(extent->height),
        out_maze);
}

maze_t* maze_create(void) {
    return maze_create_full(0, 0, 0, 0);
}

maze_t* maze_create_full(
    int x0, int y0, int width, int height) {

    maze_t* maze = nullptr;
    return maze_create_signed_checked(x0, y0, width, height, &maze)
            == NAVSYS_STATUS_OK
        ? maze
        : nullptr;
}

void maze_clear(maze_t* maze) {
    if (!maze || !maze->blocked) return;

    coord_hash_clear(maze->blocked);
}

void maze_destroy(maze_t* maze) {
    if (!maze) return;
    coord_hash_destroy(maze->blocked);
    free(maze);
}

maze_t* maze_copy(const maze_t* maze) {
    maze_t* copy = nullptr;
    return byul_maze_copy(maze, &copy) == NAVSYS_STATUS_OK
        ? copy
        : nullptr;
}

navsys_status_t byul_maze_copy(
    const maze_t* source, maze_t** out_maze) {
    if (!source || !out_maze) return NAVSYS_STATUS_INVALID_ARGUMENT;
    if (!source->blocked) return NAVSYS_STATUS_CORRUPT_STATE;

    maze_t* candidate = static_cast<maze_t*>(malloc(sizeof(maze_t)));
    if (!candidate) return NAVSYS_STATUS_OUT_OF_MEMORY;
    candidate->x0 = source->x0;
    candidate->y0 = source->y0;
    candidate->width = source->width;
    candidate->height = source->height;
    candidate->blocked = coord_hash_copy(source->blocked);
    if (!candidate->blocked) {
        free(candidate);
        return NAVSYS_STATUS_OUT_OF_MEMORY;
    }
    *out_maze = candidate;
    return NAVSYS_STATUS_OK;
}

bool maze_equal(const maze_t* a, const maze_t* b) {
    if (!a || !b) return false;
    return a->x0 == b->x0 &&
           a->y0 == b->y0 &&
           a->width == b->width &&
           a->height == b->height &&
           coord_hash_equal(a->blocked, b->blocked);
}

uint32_t maze_hash(const maze_t* maze) {
    if (!maze) return 0;
    uint32_t hash = 17;
    hash = 31 * hash + maze->x0;
    hash = 31 * hash + maze->y0;
    hash = 31 * hash + maze->width;
    hash = 31 * hash + maze->height;
    hash = 31 * hash + coord_hash_hash(maze->blocked);
    return hash;
}

void maze_set_origin(maze_t* maze, int x0, int y0) {
    if (!maze) return;
    const int64_t delta_x = static_cast<int64_t>(x0) - maze->x0;
    const int64_t delta_y = static_cast<int64_t>(y0) - maze->y0;
    (void)maze_translate_impl(maze, delta_x, delta_y);
}

navsys_status_t byul_maze_translate(
    maze_t* maze, int32_t delta_x, int32_t delta_y) {
    return maze_translate_impl(maze, delta_x, delta_y);
}

void maze_get_origin(const maze_t* maze, int* out_x0, int* out_y0) {
    if (!maze) return;
    if (out_x0) *out_x0 = maze->x0;
    if (out_y0) *out_y0 = maze->y0;
}

navsys_status_t byul_maze_get_extent(
    const maze_t* maze, byul_maze_extent_t* out_extent) {
    if (!maze || !out_extent) return NAVSYS_STATUS_INVALID_ARGUMENT;
    if (!maze->blocked) return NAVSYS_STATUS_CORRUPT_STATE;
    const maze_extent_t extent = maze_extent(
        maze->x0, maze->y0, maze->width, maze->height);
    if (extent.min_x < std::numeric_limits<int32_t>::min()
        || extent.min_x > std::numeric_limits<int32_t>::max()
        || extent.min_y < std::numeric_limits<int32_t>::min()
        || extent.min_y > std::numeric_limits<int32_t>::max()) {
        return NAVSYS_STATUS_CORRUPT_STATE;
    }
    const uint64_t width = static_cast<uint64_t>(extent.max_x - extent.min_x);
    const uint64_t height = static_cast<uint64_t>(extent.max_y - extent.min_y);
    if (width > std::numeric_limits<uint32_t>::max()
        || height > std::numeric_limits<uint32_t>::max()) {
        return NAVSYS_STATUS_CORRUPT_STATE;
    }
    const byul_maze_extent_t result{
        static_cast<int32_t>(extent.min_x),
        static_cast<int32_t>(extent.min_y),
        static_cast<uint32_t>(width),
        static_cast<uint32_t>(height)
    };
    *out_extent = result;
    return NAVSYS_STATUS_OK;
}

int maze_get_width(const maze_t* maze) {
    return maze ? maze->width : 0;
}

int maze_get_height(const maze_t* maze) {
    return maze ? maze->height : 0;
}

const coord_hash_t* maze_get_blocked_coords(const maze_t* maze) {
    return maze ? maze->blocked : NULL;
}

navsys_status_t byul_maze_get_blocked_count(
    const maze_t* maze, size_t* out_count) {
    if (!maze || !out_count) return NAVSYS_STATUS_INVALID_ARGUMENT;
    if (!maze->blocked) return NAVSYS_STATUS_CORRUPT_STATE;
    *out_count = coord_hash_size(maze->blocked);
    return NAVSYS_STATUS_OK;
}

navsys_status_t byul_maze_fetch_blocked(
    const maze_t* maze,
    coord_t* buffer,
    size_t capacity,
    size_t* out_count) {
    if (!maze || !out_count || (!buffer && capacity != 0))
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    if (!maze->blocked) return NAVSYS_STATUS_CORRUPT_STATE;
    const size_t required = coord_hash_size(maze->blocked);
    if (!buffer) {
        *out_count = required;
        return NAVSYS_STATUS_OK;
    }
    if (capacity < required) {
        *out_count = required;
        return NAVSYS_STATUS_INCOMPLETE;
    }
    size_t exported = 0;
    const navsys_status_t status = coord_hash_export_keys(
        maze->blocked, buffer, capacity, &exported);
    if (status != NAVSYS_STATUS_OK) return status;
    *out_count = exported;
    return NAVSYS_STATUS_OK;
}

navsys_status_t byul_maze_is_blocked(
    const maze_t* maze, int32_t x, int32_t y, bool* out_blocked) {
    if (!maze || !out_blocked) return NAVSYS_STATUS_INVALID_ARGUMENT;
    if (!maze->blocked) return NAVSYS_STATUS_CORRUPT_STATE;
    const maze_extent_t extent = maze_extent(
        maze->x0, maze->y0, maze->width, maze->height);
    if (!maze_extent_contains(extent, x, y)) return NAVSYS_STATUS_NOT_FOUND;
    const coord_t coordinate{x, y};
    *out_blocked = coord_hash_contains(maze->blocked, &coordinate);
    return NAVSYS_STATUS_OK;
}

navsys_status_t byul_maze_set_blocked(
    maze_t* maze,
    int32_t x,
    int32_t y,
    bool blocked,
    bool* out_changed) {
    if (!maze || !out_changed) return NAVSYS_STATUS_INVALID_ARGUMENT;
    if (!maze->blocked) return NAVSYS_STATUS_CORRUPT_STATE;
    const maze_extent_t extent = maze_extent(
        maze->x0, maze->y0, maze->width, maze->height);
    if (!maze_extent_contains(extent, x, y)) return NAVSYS_STATUS_NOT_FOUND;

    const coord_t coordinate{x, y};
    if (blocked) {
        bool inserted = false;
        const navsys_status_t status = coord_hash_upsert_copy(
            maze->blocked, &coordinate, nullptr, &inserted);
        if (status != NAVSYS_STATUS_OK) return status;
        *out_changed = inserted;
        return NAVSYS_STATUS_OK;
    }

    *out_changed = coord_hash_remove(maze->blocked, &coordinate);
    return NAVSYS_STATUS_OK;
}

navsys_status_t byul_maze_apply(
    const maze_t* maze,
    navgrid_t* navgrid,
    const byul_maze_navgrid_apply_options_t* options,
    byul_maze_navgrid_overlay_token_t* out_overlay,
    size_t* out_changed_count) {
    if (!maze || !navgrid || !out_overlay || !out_changed_count)
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    if (!maze->blocked) return NAVSYS_STATUS_CORRUPT_STATE;

    byul_maze_navgrid_cancel_func cancel_func = nullptr;
    void* cancel_userdata = nullptr;
    if (options) {
        if (options->struct_size < sizeof(*options))
            return NAVSYS_STATUS_INVALID_ARGUMENT;
        if (options->abi_version
            != BYUL_MAZE_NAVGRID_APPLY_OPTIONS_ABI_VERSION) {
            return NAVSYS_STATUS_UNSUPPORTED;
        }
        if (options->merge_policy
            != BYUL_MAZE_NAVGRID_MERGE_PRESERVE_BASE) {
            return NAVSYS_STATUS_UNSUPPORTED;
        }
        cancel_func = options->cancel_func;
        cancel_userdata = options->cancel_userdata;
    }

    try {
        const size_t count = coord_hash_size(maze->blocked);
        std::vector<coord_t> coords(count);
        size_t exported = 0;
        const navsys_status_t export_status = coord_hash_export_keys(
            maze->blocked,
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
        const byul_maze_navgrid_overlay_token_t result{
            sizeof(byul_maze_navgrid_overlay_token_t),
            BYUL_MAZE_NAVGRID_OVERLAY_TOKEN_ABI_VERSION,
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

navsys_status_t byul_maze_remove_overlay(
    navgrid_t* navgrid,
    byul_maze_navgrid_overlay_token_t* overlay,
    size_t* out_changed_count) {
    if (!navgrid || !overlay || !out_changed_count)
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    if (overlay->struct_size != sizeof(*overlay))
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    if (overlay->abi_version
        != BYUL_MAZE_NAVGRID_OVERLAY_TOKEN_ABI_VERSION) {
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
    *overlay = byul_maze_navgrid_overlay_token_t{
        sizeof(byul_maze_navgrid_overlay_token_t),
        BYUL_MAZE_NAVGRID_OVERLAY_TOKEN_ABI_VERSION,
        0,
        0
    };
    *out_changed_count = changed;
    return NAVSYS_STATUS_OK;
}

void maze_apply_to_navgrid(const maze_t* maze, navgrid_t* navgrid) {
    if (!maze || !navgrid) return;
    try {
        const size_t count = coord_hash_size(maze->blocked);
        std::vector<coord_t> coords(count);
        size_t exported = 0;
        if (coord_hash_export_keys(
                maze->blocked,
                coords.empty() ? nullptr : coords.data(),
                coords.size(),
                &exported) != NAVSYS_STATUS_OK) {
            return;
        }
        size_t changed = 0;
        if (byul::navsys::internal::navgrid_replace_blocked_overlay_source(
                navgrid,
                byul::navsys::internal::navgrid_overlay_source_kind::maze,
                maze,
                coords.empty() ? nullptr : coords.data(),
                exported,
                &changed) != NAVSYS_STATUS_OK) {
            return;
        }

        const int maze_width = maze_get_width(maze);
        const int maze_height = maze_get_height(maze);
        if (navgrid_get_width(navgrid) < maze_width)
            navgrid_set_width(navgrid, maze_width);
        if (navgrid_get_height(navgrid) < maze_height)
            navgrid_set_height(navgrid, maze_height);
    } catch (...) {
        return;
    }
}

void maze_remove_from_navgrid(const maze_t* maze, navgrid_t* navgrid) {
    if (!maze || !navgrid) return;
    size_t changed = 0;
    (void)byul::navsys::internal::navgrid_remove_blocked_overlay_source(
        navgrid,
        byul::navsys::internal::navgrid_overlay_source_kind::maze,
        maze,
        &changed);
}
