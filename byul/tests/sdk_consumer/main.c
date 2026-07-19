#include <assert.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "byul.h"
#include "coord.h"
#include "coord_hash.h"
#include "navsys_status.h"

static bool cancel_immediately(void* userdata) {
    int* calls = (int*)userdata;
    ++*calls;
    return true;
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

#define ABI1_TYPE_LAYOUT(type, expected_size, expected_align) \
    static_assert(sizeof(type) == expected_size, #type " ABI 1 size"); \
    static_assert(_Alignof(type) == expected_align, #type " ABI 1 alignment")
#define ABI1_FIELD_OFFSET(type, field, expected_offset) \
    static_assert(offsetof(type, field) == expected_offset, \
        #type "." #field " ABI 1 offset")

static_assert(sizeof(void*) == 8, "Navsys ABI 1 layout fixture requires x64");

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

ABI1_TYPE_LAYOUT(navgrid_t, 40, 8);
ABI1_FIELD_OFFSET(navgrid_t, width, 0);
ABI1_FIELD_OFFSET(navgrid_t, height, 4);
ABI1_FIELD_OFFSET(navgrid_t, mode, 8);
ABI1_FIELD_OFFSET(navgrid_t, cell_map, 16);
ABI1_FIELD_OFFSET(navgrid_t, is_coord_blocked_fn, 24);
ABI1_FIELD_OFFSET(navgrid_t, is_coord_blocked_fn_userdata, 32);

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

int main(void) {
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

    coord_hash_t* coord_values = coord_hash_create_full(
        coord_hash_int_copy, coord_hash_int_destroy);
    coord_t hash_key = {4, 2};
    int hash_value = 17;
    bool inserted = false;
    const void* borrowed_value = NULL;
    bool found = false;
    if (coord_values == NULL
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
    if (navgrid == NULL || finder == NULL || dsl == NULL
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
        return 6;
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

    byul_print_version();
    return 0;
}
