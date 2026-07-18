#include <assert.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "byul.h"
#include "navsys_status.h"

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
    if (navgrid == NULL || finder == NULL || dsl == NULL
        || navgrid_bind_is_coord_blocked_func(
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
        return 4;
    }
    dstar_lite_destroy(dsl);
    route_finder_destroy(finder);
    navgrid_destroy(navgrid);

    byul_print_version();
    return 0;
}
