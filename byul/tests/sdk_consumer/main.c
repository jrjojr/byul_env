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

    int callback_identity = 7;
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
        return 3;
    }
    dstar_lite_destroy(dsl);
    route_finder_destroy(finder);
    navgrid_destroy(navgrid);

    byul_print_version();
    return 0;
}
