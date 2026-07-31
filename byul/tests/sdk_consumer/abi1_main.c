#include <assert.h>
#include <stddef.h>

#include "navgrid_abi1.h"

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
    const int result = grid->width == 7
        && grid->height == 9
        && grid->mode == NAVGRID_DIR_4
        && grid->cell_map != NULL
        && grid->is_coord_blocked_fn == is_coord_blocked_navgrid
        && grid->is_coord_blocked_fn_userdata == NULL
        ? 0 : 3;
    navgrid_destroy(grid);
    return result;
}
