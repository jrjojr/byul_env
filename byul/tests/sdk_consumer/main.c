#include <assert.h>
#include <limits.h>
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "byul.h"
#include "coord.h"
#include "coord_hash.h"
#include "cost_coord_pq.h"
#include "dstar_lite_key.h"
#include "maze_core.h"
#include "maze_eller.h"
#include "maze_hunt_and_kill.h"
#include "maze_recursive_division.h"
#include "maze_sidewinder.h"
#include "navcell.h"
#include "navsys_status.h"

static bool cancel_immediately(void* userdata) {
    int* calls = (int*)userdata;
    ++*calls;
    return true;
}

typedef struct sdk_coord_hash_callbacks {
    int copy_calls;
    int destroy_calls;
    int equal_calls;
} sdk_coord_hash_callbacks_t;

static navsys_status_t sdk_coord_hash_copy(
    const void* source,
    void** out_copy,
    void* userdata) {
    sdk_coord_hash_callbacks_t* callbacks =
        (sdk_coord_hash_callbacks_t*)userdata;
    if (source == NULL || out_copy == NULL || callbacks == NULL) {
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    }
    void* copied = coord_hash_int_copy(source);
    if (copied == NULL) {
        return NAVSYS_STATUS_OUT_OF_MEMORY;
    }
    ++callbacks->copy_calls;
    *out_copy = copied;
    return NAVSYS_STATUS_OK;
}

static void sdk_coord_hash_destroy(void* value, void* userdata) {
    sdk_coord_hash_callbacks_t* callbacks =
        (sdk_coord_hash_callbacks_t*)userdata;
    if (callbacks != NULL) {
        ++callbacks->destroy_calls;
    }
    coord_hash_int_destroy(value);
}

static bool sdk_coord_hash_equal(
    const void* lhs,
    const void* rhs,
    void* userdata) {
    sdk_coord_hash_callbacks_t* callbacks =
        (sdk_coord_hash_callbacks_t*)userdata;
    if (lhs == NULL || rhs == NULL || callbacks == NULL) {
        return false;
    }
    ++callbacks->equal_calls;
    return *(const int*)lhs == *(const int*)rhs;
}

static_assert(NAVSYS_STATUS_OK == 0, "NAVSYS_STATUS_OK ABI");
static_assert(NAVSYS_STATUS_INVALID_ARGUMENT == -1, "NAVSYS_STATUS_INVALID_ARGUMENT ABI");
static_assert(NAVSYS_STATUS_OUT_OF_MEMORY == -2, "NAVSYS_STATUS_OUT_OF_MEMORY ABI");
static_assert(NAVSYS_STATUS_UNSUPPORTED == -3, "NAVSYS_STATUS_UNSUPPORTED ABI");
static_assert(NAVSYS_STATUS_CALLBACK_FAILED == -4, "NAVSYS_STATUS_CALLBACK_FAILED ABI");
static_assert(NAVSYS_STATUS_CORRUPT_STATE == -5, "NAVSYS_STATUS_CORRUPT_STATE ABI");
static_assert(NAVSYS_STATUS_NOT_FOUND == -6, "NAVSYS_STATUS_NOT_FOUND ABI");
static_assert(NAVSYS_STATUS_INVALIDATED == -7, "NAVSYS_STATUS_INVALIDATED ABI");
static_assert(NAVSYS_STATUS_NO_PATH == -8, "NAVSYS_STATUS_NO_PATH ABI");
static_assert(NAVSYS_STATUS_CANCELLED == -9, "NAVSYS_STATUS_CANCELLED ABI");
static_assert(NAVSYS_STATUS_LIMIT_REACHED == -10, "NAVSYS_STATUS_LIMIT_REACHED ABI");
static_assert(NAVSYS_STATUS_INCOMPLETE == -11, "NAVSYS_STATUS_INCOMPLETE ABI");
static_assert(NAVSYS_STATUS_IN_PROGRESS == -12, "NAVSYS_STATUS_IN_PROGRESS ABI");
static_assert(ROUTE_COMPLETION_NONE == 0, "ROUTE_COMPLETION_NONE ABI");
static_assert(ROUTE_COMPLETION_COMPLETE == 1, "ROUTE_COMPLETION_COMPLETE ABI");
static_assert(ROUTE_COMPLETION_PARTIAL == 2, "ROUTE_COMPLETION_PARTIAL ABI");
static_assert(ROUTE_DIR_UNKNOWN == 0, "ROUTE_DIR_UNKNOWN ABI");
static_assert(ROUTE_DIR_RIGHT == 1, "ROUTE_DIR_RIGHT ABI");
static_assert(ROUTE_DIR_UP_RIGHT == 2, "ROUTE_DIR_UP_RIGHT ABI");
static_assert(ROUTE_DIR_UP == 3, "ROUTE_DIR_UP ABI");
static_assert(ROUTE_DIR_UP_LEFT == 4, "ROUTE_DIR_UP_LEFT ABI");
static_assert(ROUTE_DIR_LEFT == 5, "ROUTE_DIR_LEFT ABI");
static_assert(ROUTE_DIR_DOWN_LEFT == 6, "ROUTE_DIR_DOWN_LEFT ABI");
static_assert(ROUTE_DIR_DOWN == 7, "ROUTE_DIR_DOWN ABI");
static_assert(ROUTE_DIR_DOWN_RIGHT == 8, "ROUTE_DIR_DOWN_RIGHT ABI");
static_assert(ROUTE_DIR_COUNT == 9, "ROUTE_DIR_COUNT ABI");
static_assert(TERRAIN_TYPE_NORMAL == 0, "TERRAIN_TYPE_NORMAL ABI");
static_assert(TERRAIN_TYPE_WATER == 1, "TERRAIN_TYPE_WATER ABI");
static_assert(TERRAIN_TYPE_FOREST == 2, "TERRAIN_TYPE_FOREST ABI");
static_assert(TERRAIN_TYPE_MOUNTAIN == 3, "TERRAIN_TYPE_MOUNTAIN ABI");
static_assert(TERRAIN_TYPE_FORBIDDEN == 100, "TERRAIN_TYPE_FORBIDDEN ABI");

#define ABI1_TYPE_LAYOUT(type, expected_size, expected_align) \
    static_assert(sizeof(type) == expected_size, #type " ABI 1 size"); \
    static_assert(_Alignof(type) == expected_align, #type " ABI 1 alignment")
#define ABI1_FIELD_OFFSET(type, field, expected_offset) \
    static_assert(offsetof(type, field) == expected_offset, \
        #type "." #field " ABI 1 offset")

static_assert(sizeof(void*) == 8, "Navsys ABI 1 layout fixture requires x64");
static_assert(
    _Generic(
        &sdk_coord_hash_copy,
        coord_hash_value_copy_func_ex: 1,
        default: 0),
    "coord hash copy callback calling convention");
static_assert(
    _Generic(
        &sdk_coord_hash_destroy,
        coord_hash_value_destroy_func_ex: 1,
        default: 0),
    "coord hash destroy callback calling convention");
static_assert(
    _Generic(
        &sdk_coord_hash_equal,
        coord_hash_value_equal_func_ex: 1,
        default: 0),
    "coord hash equal callback calling convention");

ABI1_TYPE_LAYOUT(coord_hash_create_info_t, 40, 8);
ABI1_FIELD_OFFSET(coord_hash_create_info_t, struct_size, 0);
ABI1_FIELD_OFFSET(coord_hash_create_info_t, abi_version, 4);
ABI1_FIELD_OFFSET(coord_hash_create_info_t, copy_value, 8);
ABI1_FIELD_OFFSET(coord_hash_create_info_t, destroy_value, 16);
ABI1_FIELD_OFFSET(coord_hash_create_info_t, equal_value, 24);
ABI1_FIELD_OFFSET(coord_hash_create_info_t, userdata, 32);

ABI1_TYPE_LAYOUT(coord_hash_entry_view_t, 16, 8);
ABI1_FIELD_OFFSET(coord_hash_entry_view_t, key, 0);
ABI1_FIELD_OFFSET(coord_hash_entry_view_t, value, 8);

ABI1_TYPE_LAYOUT(dstar_lite_t, 184, 8);
ABI1_FIELD_OFFSET(dstar_lite_t, navgrid, 0);
ABI1_FIELD_OFFSET(dstar_lite_t, start, 8);
ABI1_FIELD_OFFSET(dstar_lite_t, goal, 16);
ABI1_FIELD_OFFSET(dstar_lite_t, cost_fn, 24);
ABI1_FIELD_OFFSET(dstar_lite_t, cost_fn_userdata, 32);
ABI1_FIELD_OFFSET(dstar_lite_t, heuristic_fn, 40);
ABI1_FIELD_OFFSET(dstar_lite_t, heuristic_fn_userdata, 48);
ABI1_FIELD_OFFSET(dstar_lite_t, max_retry, 56);
ABI1_FIELD_OFFSET(dstar_lite_t, debug_mode_enabled, 60);
ABI1_FIELD_OFFSET(dstar_lite_t, km, 64);
ABI1_FIELD_OFFSET(dstar_lite_t, g_table, 72);
ABI1_FIELD_OFFSET(dstar_lite_t, rhs_table, 80);
ABI1_FIELD_OFFSET(dstar_lite_t, frontier, 88);
ABI1_FIELD_OFFSET(dstar_lite_t, move_fn, 96);
ABI1_FIELD_OFFSET(dstar_lite_t, move_fn_userdata, 104);
ABI1_FIELD_OFFSET(dstar_lite_t, changed_coords_fn, 112);
ABI1_FIELD_OFFSET(dstar_lite_t, changed_coords_fn_userdata, 120);
ABI1_FIELD_OFFSET(dstar_lite_t, proto_route, 128);
ABI1_FIELD_OFFSET(dstar_lite_t, real_route, 136);
ABI1_FIELD_OFFSET(dstar_lite_t, real_loop_max_retry, 144);
ABI1_FIELD_OFFSET(dstar_lite_t, reconstruct_max_retry, 148);
ABI1_FIELD_OFFSET(dstar_lite_t, proto_compute_retry_count, 152);
ABI1_FIELD_OFFSET(dstar_lite_t, real_compute_retry_count, 156);
ABI1_FIELD_OFFSET(dstar_lite_t, real_loop_retry_count, 160);
ABI1_FIELD_OFFSET(dstar_lite_t, reconstruct_retry_count, 164);
ABI1_FIELD_OFFSET(dstar_lite_t, max_range, 168);
ABI1_FIELD_OFFSET(dstar_lite_t, interval_sec, 172);
ABI1_FIELD_OFFSET(dstar_lite_t, force_quit, 176);

ABI1_TYPE_LAYOUT(terrain_type_t, 4, 4);
ABI1_TYPE_LAYOUT(navcell_t, 8, 4);
ABI1_FIELD_OFFSET(navcell_t, terrain, 0);
ABI1_FIELD_OFFSET(navcell_t, height, 4);

ABI1_TYPE_LAYOUT(route_finder_t, 80, 8);
ABI1_FIELD_OFFSET(route_finder_t, navgrid, 0);
ABI1_FIELD_OFFSET(route_finder_t, start, 8);
ABI1_FIELD_OFFSET(route_finder_t, goal, 16);
ABI1_FIELD_OFFSET(route_finder_t, type, 24);
ABI1_FIELD_OFFSET(route_finder_t, typedata, 32);
ABI1_FIELD_OFFSET(route_finder_t, max_retry, 40);
ABI1_FIELD_OFFSET(route_finder_t, debug_mode_enabled, 44);
ABI1_FIELD_OFFSET(route_finder_t, cost_fn, 48);
ABI1_FIELD_OFFSET(route_finder_t, cost_fn_userdata, 56);
ABI1_FIELD_OFFSET(route_finder_t, heuristic_fn, 64);
ABI1_FIELD_OFFSET(route_finder_t, heuristic_fn_userdata, 72);

ABI1_TYPE_LAYOUT(route_t, 48, 8);
ABI1_FIELD_OFFSET(route_t, coords, 0);
ABI1_FIELD_OFFSET(route_t, visited_order, 8);
ABI1_FIELD_OFFSET(route_t, visited_count, 16);
ABI1_FIELD_OFFSET(route_t, cost, 24);
ABI1_FIELD_OFFSET(route_t, success, 28);
ABI1_FIELD_OFFSET(route_t, total_retry_count, 32);
ABI1_FIELD_OFFSET(route_t, avg_vec_x, 36);
ABI1_FIELD_OFFSET(route_t, avg_vec_y, 40);
ABI1_FIELD_OFFSET(route_t, vec_count, 44);

ABI1_TYPE_LAYOUT(byul_maze_extent_t, 16, 4);
ABI1_FIELD_OFFSET(byul_maze_extent_t, origin_x, 0);
ABI1_FIELD_OFFSET(byul_maze_extent_t, origin_y, 4);
ABI1_FIELD_OFFSET(byul_maze_extent_t, width, 8);
ABI1_FIELD_OFFSET(byul_maze_extent_t, height, 12);
ABI1_TYPE_LAYOUT(byul_maze_navgrid_apply_options_t, 32, 8);
ABI1_FIELD_OFFSET(byul_maze_navgrid_apply_options_t, struct_size, 0);
ABI1_FIELD_OFFSET(byul_maze_navgrid_apply_options_t, abi_version, 4);
ABI1_FIELD_OFFSET(byul_maze_navgrid_apply_options_t, merge_policy, 8);
ABI1_FIELD_OFFSET(byul_maze_navgrid_apply_options_t, cancel_func, 16);
ABI1_FIELD_OFFSET(byul_maze_navgrid_apply_options_t, cancel_userdata, 24);
ABI1_TYPE_LAYOUT(byul_maze_navgrid_overlay_token_t, 24, 8);
ABI1_FIELD_OFFSET(byul_maze_navgrid_overlay_token_t, struct_size, 0);
ABI1_FIELD_OFFSET(byul_maze_navgrid_overlay_token_t, abi_version, 4);
ABI1_FIELD_OFFSET(byul_maze_navgrid_overlay_token_t, owner_cookie, 8);
ABI1_FIELD_OFFSET(byul_maze_navgrid_overlay_token_t, overlay, 16);

#undef ABI1_FIELD_OFFSET
#undef ABI1_TYPE_LAYOUT

static float sdk_cost(
    const navgrid_t* navgrid,
    const coord_t* start,
    const coord_t* goal,
    void* userdata) {
    (void)navgrid;
    (void)start;
    (void)goal;
    return userdata != NULL ? 1.0f : 2.0f;
}

static float sdk_heuristic(
    const coord_t* start,
    const coord_t* goal,
    void* userdata) {
    (void)start;
    (void)goal;
    return userdata != NULL ? 0.0f : 1.0f;
}

static void sdk_move(const coord_t* coord, void* userdata) {
    (void)coord;
    (void)userdata;
}

static coord_list_t* sdk_changed_coords(void* userdata) {
    (void)userdata;
    return NULL;
}

static bool sdk_is_blocked(
    const void* context,
    int x,
    int y,
    void* userdata) {
    (void)context;
    (void)x;
    (void)y;
    return userdata != NULL;
}

static_assert(
    _Generic(&sdk_is_blocked, is_coord_blocked_func: 1, default: 0),
    "navgrid blocked callback calling convention");
static_assert(
    _Generic(
        &byul_maze_translate,
        navsys_status_t (*)(maze_t*, int32_t, int32_t): 1,
        default: 0),
    "maze translate C calling convention");
static_assert(
    _Generic(
        &byul_maze_create,
        navsys_status_t (*)(const byul_maze_extent_t*, maze_t**): 1,
        default: 0),
    "maze checked create C calling convention");
static_assert(
    _Generic(
        &byul_maze_apply,
        navsys_status_t (*)(
            const maze_t*, navgrid_t*,
            const byul_maze_navgrid_apply_options_t*,
            byul_maze_navgrid_overlay_token_t*, size_t*): 1,
        default: 0),
    "maze checked apply C calling convention");
static_assert(
    _Generic(
        &byul_maze_set_blocked,
        navsys_status_t (*)(maze_t*, int32_t, int32_t, bool, bool*): 1,
        default: 0),
    "maze checked mutation C calling convention");
static_assert(
    _Generic(
        &byul_maze_is_blocked,
        navsys_status_t (*)(const maze_t*, int32_t, int32_t, bool*): 1,
        default: 0),
    "maze checked query C calling convention");
static_assert(
    _Generic(
        &byul_maze_check_abi,
        navsys_status_t (*)(
            uint32_t, uint64_t, byul_maze_abi_mismatch_t*): 1,
        default: 0),
    "maze ABI check C calling convention");

int main(void) {
    navcell_t zero_cell = {0};
    navcell_t compound_cell = {TERRAIN_TYPE_FOREST, INT_MAX};
    if (zero_cell.terrain != TERRAIN_TYPE_NORMAL
        || zero_cell.height != 0
        || compound_cell.terrain != TERRAIN_TYPE_FOREST
        || compound_cell.height != INT_MAX) {
        fprintf(stderr, "unexpected navcell C value ABI\n");
        return 15;
    }
    bool terrain_supported = false;
    navcell_t checked_cell = {TERRAIN_TYPE_WATER, 7};
    navcell_t assigned_cell = {0};
    navcell_t* allocated_cell = NULL;
    navcell_t* copied_cell = NULL;
    if (navcell_is_terrain_supported(
            TERRAIN_TYPE_MOUNTAIN, &terrain_supported) != NAVSYS_STATUS_OK
        || !terrain_supported
        || navcell_init_checked(
            &checked_cell, TERRAIN_TYPE_MOUNTAIN, INT32_MIN) != NAVSYS_STATUS_OK
        || navcell_validate(&checked_cell) != NAVSYS_STATUS_OK
        || navcell_assign_checked(
            &assigned_cell, &checked_cell) != NAVSYS_STATUS_OK
        || navcell_create_checked(
            assigned_cell.terrain, assigned_cell.height,
            &allocated_cell) != NAVSYS_STATUS_OK
        || navcell_copy_checked(
            allocated_cell, &copied_cell) != NAVSYS_STATUS_OK
        || copied_cell == NULL
        || copied_cell->terrain != TERRAIN_TYPE_MOUNTAIN
        || copied_cell->height != INT32_MIN) {
        navcell_destroy(copied_cell);
        navcell_destroy(allocated_cell);
        fprintf(stderr, "unexpected navcell checked C ABI\n");
        return 16;
    }
    navcell_destroy(copied_cell);
    navcell_destroy(allocated_cell);

    const char* version = byul_version_string();
    if (version == NULL || strcmp(version, BYUL_VERSION_STRING) != 0) {
        fprintf(stderr, "unexpected BYUL version\n");
        return 1;
    }
    if (coord_sizeof() != sizeof(coord_t)
        || coord_alignof() != _Alignof(coord_t)
        || coord_offsetof_x() != offsetof(coord_t, x)
        || coord_offsetof_y() != offsetof(coord_t, y)) {
        fprintf(stderr, "unexpected coord_t ABI layout\n");
        return 2;
    }
    coord_t formatted_coord = {0, 0};
    size_t required = 0;
    char formatted[32] = {0};
    if (coord_init_checked(&formatted_coord, -3, 7) != NAVSYS_STATUS_OK
        || coord_format(
            &formatted_coord, NULL, 0, &required) != NAVSYS_STATUS_OK
        || required != sizeof("(-3, 7)")
        || coord_format(
            &formatted_coord, formatted, sizeof(formatted), &required)
            != NAVSYS_STATUS_OK
        || strcmp(formatted, "(-3, 7)") != 0) {
        fprintf(stderr, "unexpected coord checked/format ABI\n");
        return 8;
    }

    sdk_coord_hash_callbacks_t hash_callbacks = {0, 0, 0};
    coord_hash_create_info_t hash_create_info = {
        (uint32_t)sizeof(coord_hash_create_info_t),
        BYUL_COORD_HASH_CREATE_INFO_ABI_VERSION,
        sdk_coord_hash_copy,
        sdk_coord_hash_destroy,
        sdk_coord_hash_equal,
        &hash_callbacks
    };
    coord_hash_t* coord_values = NULL;
    coord_t hash_key = {4, 2};
    int hash_value = 17;
    bool inserted = false;
    const void* borrowed_value = NULL;
    bool found = false;
    if (coord_hash_create_ex(&hash_create_info, &coord_values)
            != NAVSYS_STATUS_OK
        || coord_values == NULL
        || coord_hash_upsert_copy(
            coord_values, &hash_key, &hash_value, &inserted)
            != NAVSYS_STATUS_OK
        || !inserted
        || coord_hash_find(
            coord_values, &hash_key, &borrowed_value, &found)
            != NAVSYS_STATUS_OK
        || !found
        || borrowed_value == NULL
        || *(const int*)borrowed_value != hash_value
        || !coord_hash_insert(coord_values, &hash_key, &hash_value)) {
        fprintf(stderr, "unexpected coord hash old/new ABI\n");
        coord_hash_destroy(coord_values);
        return 9;
    }
    int legacy_value_count = 0;
    void** legacy_values =
        coord_hash_values(coord_values, &legacy_value_count);
    char* legacy_text = coord_hash_to_string(coord_values);
    if (legacy_values == NULL
        || legacy_value_count != 1
        || legacy_text == NULL
        || strcmp(legacy_text, "(4,2) ") != 0) {
        fprintf(stderr, "unexpected coord hash legacy allocation ABI\n");
        coord_hash_buffer_destroy(legacy_values);
        coord_hash_buffer_destroy(legacy_text);
        coord_hash_destroy(coord_values);
        return 10;
    }
    coord_hash_buffer_destroy(legacy_values);
    coord_hash_buffer_destroy(legacy_text);
    coord_hash_destroy(coord_values);
    if (hash_callbacks.copy_calls != 2
        || hash_callbacks.destroy_calls != 2
        || hash_callbacks.equal_calls != 0) {
        fprintf(stderr, "unexpected coord hash callback ABI\n");
        return 11;
    }

    coord_list_t* legacy_coords = coord_list_create();
    coord_t zero_coord = {0, 0};
    coord_t other_coord = {7, 9};
    coord_t empty_pop = coord_list_pop_back(legacy_coords);
    if (legacy_coords == NULL
        || !coord_equal(&empty_pop, &zero_coord)
        || !coord_list_push_back(legacy_coords, &zero_coord)
        || !coord_list_push_back(legacy_coords, &other_coord)
        || !coord_list_insert(
            legacy_coords, coord_list_length(legacy_coords), &zero_coord)
        || coord_list_length(legacy_coords) != 3) {
        fprintf(stderr, "unexpected coord list legacy mutation ABI\n");
        coord_list_destroy(legacy_coords);
        return 12;
    }
    coord_list_remove_value(legacy_coords, &zero_coord);
    coord_list_t* legacy_slice =
        coord_list_sublist(legacy_coords, 0, coord_list_length(legacy_coords));
    if (coord_list_length(legacy_coords) != 2
        || !coord_equal(coord_list_front(legacy_coords), &other_coord)
        || coord_list_find(legacy_coords, &zero_coord) != 1
        || legacy_slice == NULL
        || !coord_list_equals(legacy_coords, legacy_slice)
        || coord_list_sublist(legacy_coords, 0, 0) != NULL) {
        fprintf(stderr, "unexpected coord list legacy query ABI\n");
        coord_list_destroy(legacy_slice);
        coord_list_destroy(legacy_coords);
        return 13;
    }
    coord_list_destroy(legacy_slice);
    coord_list_destroy(legacy_coords);

    coord_list_t* checked_coords = NULL;
    coord_t checked_output = {91, 92};
    if (coord_list_create_ex(&checked_coords) != NAVSYS_STATUS_OK
        || checked_coords == NULL
        || coord_list_try_pop_back(checked_coords, &checked_output)
            != NAVSYS_STATUS_NOT_FOUND
        || checked_output.x != 91
        || checked_output.y != 92
        || coord_list_push_back_ex(checked_coords, &zero_coord)
            != NAVSYS_STATUS_OK
        || coord_list_push_back_ex(checked_coords, &other_coord)
            != NAVSYS_STATUS_OK
        || coord_list_fetch(checked_coords, 1, &checked_output)
            != NAVSYS_STATUS_OK
        || !coord_equal(&checked_output, &other_coord)) {
        fprintf(stderr, "unexpected coord list checked mutation ABI\n");
        coord_list_destroy(checked_coords);
        return 14;
    }
    coord_list_t* checked_copy = NULL;
    size_t checked_index = 99;
    bool checked_found = false;
    bool checked_removed = false;
    if (coord_list_copy_ex(checked_coords, &checked_copy)
            != NAVSYS_STATUS_OK
        || checked_copy == NULL
        || coord_list_size(checked_copy) != 2
        || coord_list_find_ex(
            checked_copy,
            &zero_coord,
            &checked_index,
            &checked_found) != NAVSYS_STATUS_OK
        || !checked_found
        || checked_index != 0
        || coord_list_remove_value_ex(
            checked_coords,
            &zero_coord,
            &checked_removed) != NAVSYS_STATUS_OK
        || !checked_removed
        || coord_list_size(checked_coords) != 1
        || coord_list_size(checked_copy) != 2) {
        fprintf(stderr, "unexpected coord list checked query ABI\n");
        coord_list_destroy(checked_copy);
        coord_list_destroy(checked_coords);
        return 15;
    }
    coord_list_t* checked_slice = NULL;
    coord_t exported_coords[2] = {{91, 92}, {93, 94}};
    size_t exported_count = 0;
    bool checked_equal = false;
    if (coord_list_reserve(checked_copy, 8) != NAVSYS_STATUS_OK
        || coord_list_export(
            checked_copy, NULL, 0, &exported_count) != NAVSYS_STATUS_OK
        || exported_count != 2
        || coord_list_export(
            checked_copy, exported_coords, 1, &exported_count)
            != NAVSYS_STATUS_INCOMPLETE
        || exported_count != 2
        || exported_coords[0].x != 91
        || exported_coords[0].y != 92
        || coord_list_create_slice(
            checked_copy, 0, 2, &checked_slice) != NAVSYS_STATUS_OK
        || checked_slice == NULL
        || coord_list_equal(
            checked_copy, checked_slice, &checked_equal)
            != NAVSYS_STATUS_OK
        || !checked_equal
        || coord_list_export(
            checked_slice, exported_coords, 2, &exported_count)
            != NAVSYS_STATUS_OK
        || exported_count != 2
        || !coord_equal(&exported_coords[0], &zero_coord)
        || !coord_equal(&exported_coords[1], &other_coord)) {
        fprintf(stderr, "unexpected coord list bulk ABI\n");
        coord_list_destroy(checked_slice);
        coord_list_destroy(checked_copy);
        coord_list_destroy(checked_coords);
        return 16;
    }
    coord_list_destroy(checked_slice);
    coord_list_destroy(checked_copy);
    coord_list_destroy(checked_coords);

    cost_coord_pq_t* legacy_pq = cost_coord_pq_create();
    coord_t pq_first = {1, 2};
    coord_t pq_second = {3, 4};
    if (legacy_pq == NULL) {
        fprintf(stderr, "unexpected cost coord pq allocation failure\n");
        return 17;
    }
    cost_coord_pq_push(legacy_pq, 5.0f, &pq_first);
    cost_coord_pq_push(legacy_pq, 5.0f, &pq_second);
    coord_t* borrowed_min = cost_coord_pq_peek(legacy_pq);
    coord_t* owned_min = cost_coord_pq_pop(legacy_pq);
    if (borrowed_min == NULL
        || owned_min != borrowed_min
        || owned_min->x != pq_second.x
        || owned_min->y != pq_second.y
        || cost_coord_pq_length(legacy_pq) != 1) {
        fprintf(stderr, "unexpected cost coord pq ownership/tie ABI\n");
        coord_destroy(owned_min);
        cost_coord_pq_destroy(legacy_pq);
        return 18;
    }
    coord_destroy(owned_min);
    cost_coord_pq_push(legacy_pq, 6.0f, &pq_second);
    cost_coord_pq_trim_worst(legacy_pq, 1);
    owned_min = cost_coord_pq_pop(legacy_pq);
    if (owned_min == NULL
        || owned_min->x != pq_first.x
        || owned_min->y != pq_first.y
        || !cost_coord_pq_is_empty(legacy_pq)) {
        fprintf(stderr, "unexpected cost coord pq trim ABI\n");
        coord_destroy(owned_min);
        cost_coord_pq_destroy(legacy_pq);
        return 19;
    }
    coord_destroy(owned_min);
    cost_coord_pq_destroy(legacy_pq);

    const cost_coord_pq_create_info_t pq_info = {
        (uint32_t)sizeof(cost_coord_pq_create_info_t),
        BYUL_COST_COORD_PQ_CREATE_INFO_ABI_VERSION,
        0
    };
    cost_coord_pq_t* checked_pq = NULL;
    coord_t pq_output = {91, 92};
    float pq_cost = 93.0f;
    if (cost_coord_pq_create_ex(&pq_info, &checked_pq)
            != NAVSYS_STATUS_OK
        || checked_pq == NULL
        || cost_coord_pq_pop_min(checked_pq, &pq_cost, &pq_output)
            != NAVSYS_STATUS_NOT_FOUND
        || pq_cost != 93.0f
        || pq_output.x != 91
        || pq_output.y != 92
        || cost_coord_pq_push_ex(checked_pq, 2.0f, &pq_first)
            != NAVSYS_STATUS_OK
        || cost_coord_pq_push_ex(checked_pq, 2.0f, &pq_second)
            != NAVSYS_STATUS_OK
        || cost_coord_pq_push_ex(checked_pq, -1.0f, &pq_first)
            != NAVSYS_STATUS_INVALID_ARGUMENT
        || cost_coord_pq_pop_min(checked_pq, &pq_cost, &pq_output)
            != NAVSYS_STATUS_OK
        || pq_cost != 2.0f
        || pq_output.x != pq_first.x
        || pq_output.y != pq_first.y
        || cost_coord_pq_peek_min(checked_pq, &pq_cost, &pq_output)
            != NAVSYS_STATUS_OK
        || pq_output.x != pq_second.x
        || pq_output.y != pq_second.y) {
        fprintf(stderr, "unexpected checked cost coord pq ABI\n");
        cost_coord_pq_destroy(checked_pq);
        return 20;
    }
    bool pq_removed = false;
    size_t pq_removed_count = 99;
    if (cost_coord_pq_size(checked_pq) != 1
        || cost_coord_pq_empty(checked_pq)
        || cost_coord_pq_remove_one(
            checked_pq, 2.0f, &pq_second, &pq_removed)
            != NAVSYS_STATUS_OK
        || !pq_removed
        || !cost_coord_pq_empty(checked_pq)
        || cost_coord_pq_push_ex(checked_pq, 1.0f, &pq_first)
            != NAVSYS_STATUS_OK
        || cost_coord_pq_push_ex(checked_pq, 3.0f, &pq_first)
            != NAVSYS_STATUS_OK
        || cost_coord_pq_push_ex(checked_pq, 5.0f, &pq_second)
            != NAVSYS_STATUS_OK
        || cost_coord_pq_remove_all(
            checked_pq, &pq_first, &pq_removed_count)
            != NAVSYS_STATUS_OK
        || pq_removed_count != 2
        || cost_coord_pq_trim_to_size(
            checked_pq, 0, &pq_removed_count)
            != NAVSYS_STATUS_OK
        || pq_removed_count != 1
        || !cost_coord_pq_empty(checked_pq)) {
        fprintf(stderr, "unexpected checked cost coord pq collection ABI\n");
        cost_coord_pq_destroy(checked_pq);
        return 21;
    }
    cost_coord_pq_clear(checked_pq);
    cost_coord_pq_destroy(checked_pq);

    if (sizeof(dstar_lite_key_t) != 8
        || _Alignof(dstar_lite_key_t) != 4
        || offsetof(dstar_lite_key_t, k1) != 0
        || offsetof(dstar_lite_key_t, k2) != 4) {
        fprintf(stderr, "unexpected D* Lite key layout ABI\n");
        return 22;
    }
    dstar_lite_key_t* legacy_key =
        dstar_lite_key_create_full(1.0f, 2.0f);
    dstar_lite_key_t* legacy_close_key =
        dstar_lite_key_create_full(1.000005f, 2.0f);
    dstar_lite_key_t* legacy_key_copy = dstar_lite_key_copy(legacy_key);
    bool legacy_is_close = false;
    if (legacy_key == NULL
        || legacy_close_key == NULL
        || legacy_key_copy == NULL
        || dstar_lite_key_equal(legacy_key, legacy_close_key)
        || dstar_lite_key_compare(legacy_key, legacy_close_key) >= 0
        || dstar_lite_key_hash(legacy_key)
            == dstar_lite_key_hash(legacy_close_key)
        || !dstar_lite_key_equal(legacy_key, legacy_key_copy)
        || dstar_lite_key_hash(legacy_key)
            != dstar_lite_key_hash(legacy_key_copy)
        || dstar_lite_key_is_close(
            legacy_key, legacy_close_key, 0.0f, 1e-5f,
            &legacy_is_close) != NAVSYS_STATUS_OK
        || !legacy_is_close
        || dstar_lite_key_create_full(NAN, 0.0f) != NULL) {
        fprintf(stderr, "unexpected D* Lite key legacy relation ABI\n");
        dstar_lite_key_destroy(legacy_key_copy);
        dstar_lite_key_destroy(legacy_close_key);
        dstar_lite_key_destroy(legacy_key);
        return 23;
    }
    dstar_lite_key_destroy(legacy_key_copy);
    dstar_lite_key_destroy(legacy_close_key);
    dstar_lite_key_destroy(legacy_key);

    legacy_is_close = true;
    const dstar_lite_key_t close_lhs = {1.0f, 2.0f};
    const dstar_lite_key_t close_rhs = {1.000005f, 2.0f};
    if (dstar_lite_key_is_close(
            &close_lhs, &close_rhs, -1.0f, 0.0f,
            &legacy_is_close) != NAVSYS_STATUS_INVALID_ARGUMENT
        || !legacy_is_close) {
        fprintf(stderr, "D* Lite key closeness failure output changed\n");
        return 23;
    }

    if (dstar_lite_key_sizeof() != sizeof(dstar_lite_key_t)
        || dstar_lite_key_alignof() != _Alignof(dstar_lite_key_t)
        || dstar_lite_key_offsetof_k1() != offsetof(dstar_lite_key_t, k1)
        || dstar_lite_key_offsetof_k2() != offsetof(dstar_lite_key_t, k2)) {
        fprintf(stderr, "unexpected D* Lite key runtime layout ABI\n");
        return 24;
    }
    dstar_lite_key_t canonical_key = {17.0f, 19.0f};
    const dstar_lite_key_t preserved_key = canonical_key;
    dstar_lite_key_t* const key_sentinel =
        (dstar_lite_key_t*)(uintptr_t)1;
    dstar_lite_key_t* exact_key = key_sentinel;
    dstar_lite_key_t* exact_key_copy = key_sentinel;
    int exact_compare = 23;
    if (dstar_lite_key_init(&canonical_key, -0.0f, 2.0f)
            != NAVSYS_STATUS_OK
        || canonical_key.k1 != 0.0f
        || signbit(canonical_key.k1)
        || dstar_lite_key_init(&canonical_key, NAN, 0.0f)
            != NAVSYS_STATUS_INVALID_ARGUMENT
        || canonical_key.k1 != 0.0f
        || canonical_key.k2 != 2.0f
        || dstar_lite_key_create_ex(-0.0f, INFINITY, &exact_key)
            != NAVSYS_STATUS_OK
        || exact_key == NULL
        || exact_key == key_sentinel
        || dstar_lite_key_copy_ex(exact_key, &exact_key_copy)
            != NAVSYS_STATUS_OK
        || exact_key_copy == NULL
        || exact_key_copy == key_sentinel
        || !dstar_lite_key_equal_exact(exact_key, exact_key_copy)
        || dstar_lite_key_hash_exact(exact_key)
            != dstar_lite_key_hash_exact(exact_key_copy)
        || dstar_lite_key_compare_exact(
            exact_key, exact_key_copy, &exact_compare) != NAVSYS_STATUS_OK
        || exact_compare != 0) {
        fprintf(stderr, "unexpected D* Lite key exact value ABI\n");
        if (exact_key_copy != key_sentinel) {
            dstar_lite_key_destroy(exact_key_copy);
        }
        if (exact_key != key_sentinel) {
            dstar_lite_key_destroy(exact_key);
        }
        return 25;
    }
    canonical_key = preserved_key;
    exact_compare = 23;
    if (dstar_lite_key_init(&canonical_key, -INFINITY, 0.0f)
            != NAVSYS_STATUS_INVALID_ARGUMENT
        || canonical_key.k1 != preserved_key.k1
        || canonical_key.k2 != preserved_key.k2
        || dstar_lite_key_compare_exact(
            &canonical_key, NULL, &exact_compare)
            != NAVSYS_STATUS_INVALID_ARGUMENT
        || exact_compare != 23) {
        fprintf(stderr, "D* Lite key failure output was not preserved\n");
        dstar_lite_key_destroy(exact_key_copy);
        dstar_lite_key_destroy(exact_key);
        return 26;
    }
    dstar_lite_key_destroy(exact_key_copy);
    dstar_lite_key_destroy(exact_key);

    if (!route_finder_is_supported(ROUTE_FINDER_ASTAR)
        || !route_finder_is_supported(ROUTE_FINDER_WEIGHTED_ASTAR)
        || route_finder_is_supported(ROUTE_FINDER_BELLMAN_FORD)
        || route_finder_is_supported(ROUTE_FINDER_DSTAR_LITE)) {
        fprintf(stderr, "unexpected route finder capability set\n");
        return 3;
    }

    int callback_identity = 7;
    route_finder_fringe_search_config_t fringe = {0.5f};
    route_finder_rta_star_config_t rta = {7};
    route_finder_sma_star_config_t sma = {64};
    route_finder_weighted_astar_config_t weighted = {2.0f};
    navgrid_t* navgrid = navgrid_create();
    route_finder_t* finder = route_finder_create(navgrid);
    dstar_lite_t* dsl = dstar_lite_create(navgrid);
    route_t* route = NULL;
    route_finder_run_stats_t run_stats = {0};
    navcell_t sdk_cell = {TERRAIN_TYPE_FOREST, 17};
    navcell_t sdk_prior = {TERRAIN_TYPE_MOUNTAIN, -1};
    bool sdk_had_prior = true;
    bool sdk_changed = false;
    coord_t sdk_overlay_coords[2] = {{2, 3}, {2, 3}};
    navgrid_overlay_id_t sdk_overlay = 0;
    size_t sdk_changed_count = 0;
    coord_t sdk_neighbors[8] = {{0, 0}};
    size_t sdk_neighbor_count = 0;
    navgrid_cell_entry_t sdk_entries[1] = {{{0, 0}, {TERRAIN_TYPE_NORMAL, 0}, false, false}};
    size_t sdk_entry_count = 0;
    is_coord_blocked_func sdk_blocked_fn = NULL;
    void* sdk_blocked_userdata = (void*)1;
    navgrid_abi_mismatch_t sdk_abi_mismatch = NAVGRID_ABI_VERSION_MISMATCH;
    if (navgrid == NULL || finder == NULL || dsl == NULL
        || navgrid_get_width(navgrid) != 0
        || navgrid_get_height(navgrid) != 0
        || navgrid_get_mode(navgrid) != NAVGRID_DIR_8
        || navgrid_get_abi_version() != BYUL_NAVGRID_ABI_VERSION
        || navgrid_get_abi_fingerprint() != BYUL_NAVGRID_ABI_FINGERPRINT
        || navgrid_check_abi(
            BYUL_NAVGRID_ABI_VERSION,
            BYUL_NAVGRID_ABI_FINGERPRINT,
            &sdk_abi_mismatch) != NAVSYS_STATUS_OK
        || sdk_abi_mismatch != NAVGRID_ABI_MATCH
        || navgrid_fetch_is_coord_blocked_binding(
            navgrid, &sdk_blocked_fn, &sdk_blocked_userdata)
            != NAVSYS_STATUS_OK
        || sdk_blocked_fn != is_coord_blocked_navgrid
        || sdk_blocked_userdata != NULL
        || navgrid_set_cell_ex(
            navgrid, 2, 3, &sdk_cell, &sdk_prior,
            &sdk_had_prior, &sdk_changed) != NAVSYS_STATUS_OK
        || sdk_had_prior
        || !sdk_changed
        || sdk_prior.terrain != TERRAIN_TYPE_NORMAL
        || navgrid_apply_blocked_overlay(
            navgrid, sdk_overlay_coords, 2, &sdk_overlay,
            &sdk_changed_count) != NAVSYS_STATUS_OK
        || sdk_overlay == 0
        || sdk_changed_count != 1
        || navgrid_remove_blocked_overlay(
            navgrid, sdk_overlay, &sdk_changed_count) != NAVSYS_STATUS_OK
        || sdk_changed_count != 1
        || sizeof(navgrid_cell_entry_t) != 20
        || navgrid_export_neighbors(
            navgrid, 2, 3, false, NULL, 0, &sdk_neighbor_count)
            != NAVSYS_STATUS_OK
        || sdk_neighbor_count != 8
        || navgrid_export_neighbors(
            navgrid, 2, 3, false, sdk_neighbors, 8, &sdk_neighbor_count)
            != NAVSYS_STATUS_OK
        || sdk_neighbors[0].x != 3
        || sdk_neighbors[0].y != 3
        || navgrid_export_cells(
            navgrid, NULL, 0, &sdk_entry_count) != NAVSYS_STATUS_OK
        || sdk_entry_count != 1
        || navgrid_export_cells(
            navgrid, sdk_entries, 1, &sdk_entry_count) != NAVSYS_STATUS_OK
        || sdk_entries[0].coord.x != 2
        || sdk_entries[0].coord.y != 3
        || !sdk_entries[0].present
        || sdk_entries[0].blocked
        || sdk_entries[0].cell.terrain != TERRAIN_TYPE_FOREST
        || route_finder_set_type_checked(
            finder, ROUTE_FINDER_ASTAR) != NAVSYS_STATUS_OK
        || route_finder_set_max_retry_checked(
            finder, 1000) != NAVSYS_STATUS_OK
        || route_finder_run_ex(
            finder, &route, &run_stats) != NAVSYS_STATUS_OK
        || route == NULL
        || !run_stats.complete
        || run_stats.partial) {
        fprintf(stderr, "unexpected route finder run ABI\n");
        return 4;
    }
    size_t route_coord_count = 0;
    coord_t route_coords[2] = {{0, 0}, {0, 0}};
    coord_t fetched_coord = {-1, -1};
    double total_cost = -1.0;
    route_completion_t completion = ROUTE_COMPLETION_NONE;
    if (route_get_coord_count(route) != 1
        || route_fetch_coord(route, 0, &fetched_coord)
            != NAVSYS_STATUS_OK
        || fetched_coord.x != 0
        || fetched_coord.y != 0
        || route_fetch_total_cost(route, &total_cost)
            != NAVSYS_STATUS_OK
        || total_cost != 0.0
        || route_fetch_completion(route, &completion)
            != NAVSYS_STATUS_OK
        || completion != ROUTE_COMPLETION_COMPLETE
        || route_export_coords(route, NULL, 0, &route_coord_count)
            != NAVSYS_STATUS_OK
        || route_coord_count != 1
        || route_export_coords(
            route, route_coords, 1, &route_coord_count)
            != NAVSYS_STATUS_OK
        || route_coord_count != 1
        || route_coords[0].x != 0
        || route_coords[0].y != 0) {
        fprintf(stderr, "unexpected route coordinate export ABI\n");
        return 5;
    }
    route_destroy(route);
    route = NULL;

    route_builder_t* route_builder = NULL;
    navsys_search_trace_t* search_trace = NULL;
    navsys_search_trace_t* search_trace_copy = NULL;
    if (route_builder_create(&route_builder) != NAVSYS_STATUS_OK
        || route_builder == NULL
        || route_builder_push_coord(route_builder, &fetched_coord)
            != NAVSYS_STATUS_OK
        || route_builder_set_total_cost(route_builder, 2.5)
            != NAVSYS_STATUS_OK
        || route_builder_set_completion(
            route_builder, ROUTE_COMPLETION_COMPLETE) != NAVSYS_STATUS_OK
        || route_builder_finish(route_builder, &route) != NAVSYS_STATUS_OK
        || route == NULL
        || route_get_coord_count(route) != 1
        || navsys_search_trace_create(&search_trace) != NAVSYS_STATUS_OK
        || search_trace == NULL
        || navsys_search_trace_clone_ex(
            search_trace, &search_trace_copy) != NAVSYS_STATUS_OK
        || search_trace_copy == NULL
        || navsys_search_trace_get_visit_count(search_trace_copy) != 0) {
        fprintf(stderr, "unexpected route builder/trace ABI\n");
        return 6;
    }
    navsys_search_trace_destroy(search_trace_copy);
    navsys_search_trace_destroy(search_trace);
    route_builder_destroy(route_builder);
    route_destroy(route);
    route = NULL;

    int cancel_calls = 0;
    route_finder_run_options_t run_options = {
        (uint32_t)sizeof(route_finder_run_options_t),
        cancel_immediately,
        &cancel_calls
    };
    if (route_finder_run_with_options(
            finder, &run_options, &route, &run_stats)
            != NAVSYS_STATUS_CANCELLED
        || cancel_calls != 1
        || route == NULL
        || run_stats.complete) {
        fprintf(stderr, "unexpected route finder cancellation ABI\n");
        return 7;
    }
    route_destroy(route);

    if (navgrid_bind_is_coord_blocked_func(
            navgrid, sdk_is_blocked, &callback_identity) != NAVSYS_STATUS_OK
        || route_finder_bind_cost_func(
            finder, sdk_cost, &callback_identity) != NAVSYS_STATUS_OK
        || route_finder_bind_heuristic_func(
            finder, sdk_heuristic, &callback_identity) != NAVSYS_STATUS_OK
        || route_finder_bind_fringe_search_config(
            finder, &fringe) != NAVSYS_STATUS_OK
        || route_finder_bind_rta_star_config(
            finder, &rta) != NAVSYS_STATUS_OK
        || route_finder_bind_sma_star_config(
            finder, &sma) != NAVSYS_STATUS_OK
        || route_finder_bind_weighted_astar_config(
            finder, &weighted) != NAVSYS_STATUS_OK
        || route_finder_get_type(finder) != ROUTE_FINDER_WEIGHTED_ASTAR
        || route_finder_get_typedata(finder) != &weighted
        || route_finder_unbind_algorithm_config(finder) != NAVSYS_STATUS_OK
        || route_finder_get_typedata(finder) != NULL
        || dstar_lite_bind_cost_func(
            dsl, sdk_cost, &callback_identity) != NAVSYS_STATUS_OK
        || dstar_lite_bind_heuristic_func(
            dsl, sdk_heuristic, &callback_identity) != NAVSYS_STATUS_OK
        || dstar_lite_bind_move_func(
            dsl, sdk_move, &callback_identity) != NAVSYS_STATUS_OK
        || dstar_lite_bind_changed_coords_func(
            dsl, sdk_changed_coords, &callback_identity) != NAVSYS_STATUS_OK
        || dstar_lite_unbind_changed_coords_func(dsl) != NAVSYS_STATUS_OK
        || dstar_lite_unbind_move_func(dsl) != NAVSYS_STATUS_OK
        || dstar_lite_unbind_heuristic_func(dsl) != NAVSYS_STATUS_OK
        || dstar_lite_unbind_cost_func(dsl) != NAVSYS_STATUS_OK
        || route_finder_unbind_heuristic_func(finder) != NAVSYS_STATUS_OK
        || route_finder_unbind_cost_func(finder) != NAVSYS_STATUS_OK
        || navgrid_unbind_is_coord_blocked_func(navgrid) != NAVSYS_STATUS_OK) {
        fprintf(stderr, "unexpected Navsys callback binding ABI\n");
        return 7;
    }
    dstar_lite_destroy(dsl);
    route_finder_destroy(finder);
    navgrid_destroy(navgrid);

    obstacle_t* obstacle = obstacle_create_full(0, 0, 3, 3);
    if (obstacle == NULL
        || !obstacle_block_coord(obstacle, 0, 0)
        || !obstacle_is_coord_blocked(obstacle, 0, 0)) {
        fprintf(stderr, "unexpected obstacle core ABI\n");
        obstacle_destroy(obstacle);
        return 8;
    }
    coord_list_t* obstacle_neighbors =
        obstacle_clone_neighbors(obstacle, 1, 1);
    if (obstacle_neighbors == NULL
        || coord_list_size(obstacle_neighbors) != 7) {
        fprintf(stderr, "unexpected obstacle neighbor ABI\n");
        coord_list_destroy(obstacle_neighbors);
        obstacle_destroy(obstacle);
        return 8;
    }
    coord_list_destroy(obstacle_neighbors);
    obstacle_destroy(obstacle);

    obstacle_t* checked_obstacle = NULL;
    obstacle_t* checked_obstacle_copy = NULL;
    bool obstacle_changed = false;
    size_t obstacle_count = 0;
    coord_t obstacle_coords[1] = {{0, 0}};
    if (obstacle_create_checked(0, 0, 3, 3, &checked_obstacle)
            != NAVSYS_STATUS_OK
        || obstacle_set_blocked(
            checked_obstacle, 1, 2, true, &obstacle_changed)
            != NAVSYS_STATUS_OK
        || !obstacle_changed
        || obstacle_export_blocked(
            checked_obstacle, NULL, 0, &obstacle_count)
            != NAVSYS_STATUS_OK
        || obstacle_count != 1
        || obstacle_export_blocked(
            checked_obstacle, obstacle_coords, 1, &obstacle_count)
            != NAVSYS_STATUS_OK
        || obstacle_coords[0].x != 1
        || obstacle_coords[0].y != 2
        || obstacle_copy_checked(checked_obstacle, &checked_obstacle_copy)
            != NAVSYS_STATUS_OK
        || !obstacle_equal(checked_obstacle, checked_obstacle_copy)) {
        fprintf(stderr, "unexpected checked obstacle ABI\n");
        obstacle_destroy(checked_obstacle_copy);
        obstacle_destroy(checked_obstacle);
        return 8;
    }

    coord_t neighbor_coords[8] = {{0, 0}};
    coord_t selected_neighbor = {0, 0};
    size_t raster_changed = 0;
    coord_list_t* canonical_neighbors = obstacle_create_neighbors(
        checked_obstacle, 1, 1);
    if (canonical_neighbors == NULL
        || coord_list_size(canonical_neighbors) != 7
        || obstacle_export_neighbors(
            checked_obstacle, 1, 1, false,
            neighbor_coords, 8, &obstacle_count) != NAVSYS_STATUS_OK
        || obstacle_count != 8
        || obstacle_fetch_neighbor_at_degree(
            checked_obstacle, 1, 1, 0.0, &selected_neighbor)
            != NAVSYS_STATUS_OK
        || selected_neighbor.x != 2
        || selected_neighbor.y != 1
        || obstacle_block_square(
            checked_obstacle, 1, 1, 0, &raster_changed)
            != NAVSYS_STATUS_OK
        || raster_changed != 1
        || obstacle_block_line(
            checked_obstacle, 0, 0, 2, 0, 0, &raster_changed)
            != NAVSYS_STATUS_OK
        || raster_changed != 3) {
        fprintf(stderr, "unexpected canonical obstacle geometry ABI\n");
        coord_list_destroy(canonical_neighbors);
        obstacle_destroy(checked_obstacle_copy);
        obstacle_destroy(checked_obstacle);
        return 8;
    }
    navgrid_t* obstacle_grid = navgrid_create_full(
        3, 3, NAVGRID_DIR_8, NULL);
    obstacle_navgrid_apply_options_t overlay_options = {
        sizeof(obstacle_navgrid_apply_options_t),
        OBSTACLE_NAVGRID_APPLY_OPTIONS_ABI_VERSION,
        OBSTACLE_NAVGRID_MERGE_PRESERVE_BASE,
        NULL,
        NULL
    };
    obstacle_navgrid_overlay_token_t obstacle_overlay = {0};
    size_t overlay_changed = 0;
    if (obstacle_grid == NULL
        || obstacle_apply_to_navgrid_checked(
            checked_obstacle,
            obstacle_grid,
            &overlay_options,
            &obstacle_overlay,
            &overlay_changed) != NAVSYS_STATUS_OK
        || overlay_changed != 5
        || obstacle_remove_from_navgrid_checked(
            obstacle_grid, &obstacle_overlay, &overlay_changed)
            != NAVSYS_STATUS_OK
        || overlay_changed != 5
        || obstacle_overlay.owner_cookie != 0
        || obstacle_overlay.overlay != 0) {
        fprintf(stderr, "unexpected obstacle overlay ABI\n");
        navgrid_destroy(obstacle_grid);
        coord_list_destroy(canonical_neighbors);
        obstacle_destroy(checked_obstacle_copy);
        obstacle_destroy(checked_obstacle);
        return 8;
    }
    navgrid_destroy(obstacle_grid);
    coord_list_destroy(canonical_neighbors);
    obstacle_destroy(checked_obstacle_copy);
    obstacle_destroy(checked_obstacle);

    {
        const byul_maze_extent_t maze_extent = {-2, 4, 5, 3};
        maze_t* checked_maze = NULL;
        bool maze_changed = false;
        bool maze_blocked = false;
        size_t maze_blocked_count = 0;
        byul_maze_abi_mismatch_t maze_mismatch =
            BYUL_MAZE_ABI_VERSION_MISMATCH;
        if (byul_maze_check_abi(
                BYUL_MAZE_ABI_VERSION,
                BYUL_MAZE_ABI_FINGERPRINT,
                &maze_mismatch) != NAVSYS_STATUS_OK
            || maze_mismatch != BYUL_MAZE_ABI_MATCH
            || byul_maze_create(&maze_extent, &checked_maze)
                != NAVSYS_STATUS_OK
            || byul_maze_set_blocked(
                checked_maze, -1, 5, true, &maze_changed)
                != NAVSYS_STATUS_OK
            || !maze_changed
            || byul_maze_is_blocked(
                checked_maze, -1, 5, &maze_blocked) != NAVSYS_STATUS_OK
            || !maze_blocked
            || byul_maze_get_blocked_count(
                checked_maze, &maze_blocked_count) != NAVSYS_STATUS_OK
            || maze_blocked_count != 1) {
            fprintf(stderr, "unexpected checked maze accessor ABI\n");
            maze_destroy(checked_maze);
            return 8;
        }
        maze_destroy(checked_maze);
    }

    {
        const byul_maze_generate_options_t options = {
            sizeof(byul_maze_generate_options_t),
            BYUL_MAZE_GENERATE_OPTIONS_ABI_VERSION,
            UINT64_C(0),
            UINT64_C(16),
            UINT64_C(81),
            NULL,
            NULL
        };
        maze_t* eller = NULL;
        if (byul_maze_generate_eller(
                -5, 8, 9, 9, &options, &eller) != NAVSYS_STATUS_OK
            || eller == NULL
            || maze_hash(eller) != UINT32_C(789167229)) {
            fprintf(stderr, "unexpected checked Eller SDK ABI\n");
            maze_destroy(eller);
            return 8;
        }
        maze_destroy(eller);
    }

    {
        const byul_maze_generate_options_t options = {
            sizeof(byul_maze_generate_options_t),
            BYUL_MAZE_GENERATE_OPTIONS_ABI_VERSION,
            UINT64_C(0),
            UINT64_C(1296),
            UINT64_C(81),
            NULL,
            NULL
        };
        maze_t* hunt_and_kill = NULL;
        if (byul_maze_generate_hunt_and_kill(
                -5, 8, 9, 9, &options, &hunt_and_kill) != NAVSYS_STATUS_OK
            || hunt_and_kill == NULL
            || maze_hash(hunt_and_kill) != UINT32_C(26398801)) {
            fprintf(stderr, "unexpected checked Hunt-and-Kill SDK ABI\n");
            maze_destroy(hunt_and_kill);
            return 8;
        }
        maze_destroy(hunt_and_kill);
    }

    {
        const byul_maze_generate_options_t options = {
            sizeof(byul_maze_generate_options_t),
            BYUL_MAZE_GENERATE_OPTIONS_ABI_VERSION,
            UINT64_C(0),
            UINT64_C(64),
            UINT64_C(81),
            NULL,
            NULL
        };
        bool supported = false;
        maze_t* sidewinder = NULL;
        if (byul_maze_sidewinder_sweep_is_supported(
                BYUL_MAZE_SIDEWINDER_WEST_SOUTH, &supported)
                != NAVSYS_STATUS_OK
            || !supported
            || byul_maze_generate_sidewinder(
                -5, 8, 9, 9, BYUL_MAZE_SIDEWINDER_WEST_SOUTH,
                &options, &sidewinder) != NAVSYS_STATUS_OK
            || sidewinder == NULL
            || maze_hash(sidewinder) != UINT32_C(455990389)) {
            fprintf(stderr, "unexpected checked Sidewinder SDK ABI\n");
            maze_destroy(sidewinder);
            return 8;
        }
        maze_destroy(sidewinder);
    }

    {
        const byul_maze_generate_options_t options = {
            sizeof(byul_maze_generate_options_t),
            BYUL_MAZE_GENERATE_OPTIONS_ABI_VERSION,
            UINT64_C(0),
            UINT64_C(64),
            UINT64_C(81),
            NULL,
            NULL
        };
        maze_t* recursive_division = NULL;
        if (byul_maze_generate_recursive_division(
                -5, 8, 9, 9, &options, &recursive_division)
                != NAVSYS_STATUS_OK
            || recursive_division == NULL
            || maze_hash(recursive_division) != UINT32_C(669558005)) {
            fprintf(stderr, "unexpected checked Recursive Division SDK ABI\n");
            maze_destroy(recursive_division);
            return 8;
        }
        maze_destroy(recursive_division);
    }

    navgrid_t* legacy_carver_grid = navgrid_create_full(
        3, 3, NAVGRID_DIR_8, NULL);
    coord_t legacy_carver_start = {0, 1};
    coord_t legacy_carver_goal = {2, 1};
    if (legacy_carver_grid == NULL
        || !navgrid_block_coord(legacy_carver_grid, 0, 1)
        || !navgrid_block_coord(legacy_carver_grid, 1, 1)
        || !navgrid_block_coord(legacy_carver_grid, 2, 1)
        || route_carve_beam(
            legacy_carver_grid,
            &legacy_carver_start,
            &legacy_carver_goal,
            0) != 2
        || route_carve_bomb(
            legacy_carver_grid, &legacy_carver_start, 0) != 1) {
        fprintf(stderr, "unexpected legacy route carver ABI\n");
        navgrid_destroy(legacy_carver_grid);
        return 8;
    }
    navgrid_destroy(legacy_carver_grid);

    navgrid_t* checked_carver_grid = navgrid_create_full(
        3, 3, NAVGRID_DIR_8, NULL);
    const navgrid_carve_options_t carve_options = {
        sizeof(navgrid_carve_options_t),
        NAVGRID_CARVE_OPTIONS_ABI_VERSION,
        0,
        NAVGRID_CARVE_CHEBYSHEV_SQUARE,
        NAVGRID_LINE_CENTER_CELLS,
        NAVGRID_CARVE_EFFECTIVE_BLOCKED,
        NAVGRID_CARVE_INCLUDE_START | NAVGRID_CARVE_INCLUDE_END
            | NAVGRID_CARVE_ATOMIC,
        0,
        16,
        NULL,
        NULL
    };
    size_t carved_count = 99;
    if (checked_carver_grid == NULL
        || !navgrid_block_coord(checked_carver_grid, 0, 1)
        || !navgrid_block_coord(checked_carver_grid, 1, 1)
        || !navgrid_block_coord(checked_carver_grid, 2, 1)
        || navgrid_carve_line(
            checked_carver_grid,
            &legacy_carver_start,
            &legacy_carver_goal,
            &carve_options,
            &carved_count) != NAVSYS_STATUS_OK
        || carved_count != 3) {
        fprintf(stderr, "unexpected checked route carver ABI\n");
        navgrid_destroy(checked_carver_grid);
        return 9;
    }
    navgrid_destroy(checked_carver_grid);

    byul_print_version();
    return 0;
}
