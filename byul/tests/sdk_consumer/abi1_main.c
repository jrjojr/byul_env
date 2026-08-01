#include <assert.h>
#include <stddef.h>

#include "navgrid_abi1.h"
#include "obstacle_abi1.h"

static_assert(sizeof(void*) == 8, "Navgrid ABI 1 fixture requires x64");
static_assert(sizeof(navgrid_t) == 40, "Navgrid ABI 1 size");
static_assert(_Alignof(navgrid_t) == 8, "Navgrid ABI 1 alignment");
static_assert(offsetof(navgrid_t, width) == 0, "Navgrid ABI 1 width");
static_assert(offsetof(navgrid_t, height) == 4, "Navgrid ABI 1 height");
static_assert(offsetof(navgrid_t, mode) == 8, "Navgrid ABI 1 mode");
static_assert(offsetof(navgrid_t, cell_map) == 16, "Navgrid ABI 1 cell_map");
static_assert(
    offsetof(navgrid_t, is_coord_blocked_fn) == 24,
    "Navgrid ABI 1 callback");
static_assert(
    offsetof(navgrid_t, is_coord_blocked_fn_userdata) == 32,
    "Navgrid ABI 1 userdata");
static_assert(sizeof(obstacle_t) == 24, "Obstacle ABI 1 size");
static_assert(_Alignof(obstacle_t) == 8, "Obstacle ABI 1 alignment");
static_assert(offsetof(obstacle_t, x0) == 0, "Obstacle ABI 1 x0");
static_assert(offsetof(obstacle_t, y0) == 4, "Obstacle ABI 1 y0");
static_assert(offsetof(obstacle_t, width) == 8, "Obstacle ABI 1 width");
static_assert(offsetof(obstacle_t, height) == 12, "Obstacle ABI 1 height");
static_assert(offsetof(obstacle_t, blocked) == 16, "Obstacle ABI 1 blocked");

int main(void) {
    navgrid_abi_mismatch_t mismatch = NAVGRID_ABI_VERSION_MISMATCH;
    if (navgrid_check_abi(
            BYUL_NAVGRID_ABI1_VERSION,
            BYUL_NAVGRID_ABI1_FINGERPRINT,
            &mismatch) != NAVSYS_STATUS_OK
        || mismatch != NAVGRID_ABI_MATCH) {
        return 1;
    }
    navgrid_t* grid = navgrid_create_full(7, 9, NAVGRID_DIR_4, NULL);
    if (!grid) return 2;
    obstacle_abi_mismatch_t obstacle_mismatch = OBSTACLE_ABI_VERSION_MISMATCH;
    if (obstacle_check_abi(
            BYUL_OBSTACLE_ABI1_VERSION,
            BYUL_OBSTACLE_ABI1_FINGERPRINT,
            &obstacle_mismatch) != NAVSYS_STATUS_OK
        || obstacle_mismatch != OBSTACLE_ABI_MATCH) {
        navgrid_destroy(grid);
        return 3;
    }
    obstacle_t* obstacle = obstacle_create_full(1, 2, 7, 9);
    if (!obstacle) {
        navgrid_destroy(grid);
        return 4;
    }
    const int result = grid->width == 7
        && grid->height == 9
        && grid->mode == NAVGRID_DIR_4
        && grid->cell_map != NULL
        && grid->is_coord_blocked_fn == is_coord_blocked_navgrid
        && grid->is_coord_blocked_fn_userdata == NULL
        && obstacle->x0 == 1
        && obstacle->y0 == 2
        && obstacle->width == 7
        && obstacle->height == 9
        && obstacle->blocked != NULL
        ? 0 : 5;
    obstacle_destroy(obstacle);
    navgrid_destroy(grid);
    return result;
}
