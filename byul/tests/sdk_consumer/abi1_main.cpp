#include <cassert>
#include <cstddef>
#include <type_traits>

#include "navgrid_abi1.h"

int main() {
    static_assert(sizeof(void*) == 8);
    static_assert(std::is_standard_layout_v<navgrid_t>);
    static_assert(sizeof(navgrid_t) == 40);
    static_assert(alignof(navgrid_t) == 8);
    static_assert(offsetof(navgrid_t, width) == 0);
    static_assert(offsetof(navgrid_t, height) == 4);
    static_assert(offsetof(navgrid_t, mode) == 8);
    static_assert(offsetof(navgrid_t, cell_map) == 16);
    static_assert(offsetof(navgrid_t, is_coord_blocked_fn) == 24);
    static_assert(offsetof(navgrid_t, is_coord_blocked_fn_userdata) == 32);

    navgrid_abi_mismatch_t mismatch = NAVGRID_ABI_VERSION_MISMATCH;
    assert(navgrid_check_abi(
        BYUL_NAVGRID_ABI1_VERSION,
        BYUL_NAVGRID_ABI1_FINGERPRINT,
        &mismatch) == NAVSYS_STATUS_OK);
    assert(mismatch == NAVGRID_ABI_MATCH);
    navgrid_t* grid = navgrid_create_full(7, 9, NAVGRID_DIR_4, nullptr);
    assert(grid != nullptr);
    assert(grid->width == 7);
    assert(grid->height == 9);
    assert(grid->mode == NAVGRID_DIR_4);
    assert(grid->cell_map != nullptr);
    assert(grid->is_coord_blocked_fn == is_coord_blocked_navgrid);
    assert(grid->is_coord_blocked_fn_userdata == nullptr);
    navgrid_destroy(grid);
    return 0;
}
