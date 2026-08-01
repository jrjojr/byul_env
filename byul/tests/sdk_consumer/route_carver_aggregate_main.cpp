#include "navsys.h"

#include <cstddef>

int main() {
    navgrid_t* grid = navgrid_create_full(1, 1, NAVGRID_DIR_4, nullptr);
    if (!grid) return 1;
    const coord_t center{0, 0};
    const navgrid_carve_options_t options{
        sizeof(navgrid_carve_options_t),
        NAVGRID_CARVE_OPTIONS_ABI_VERSION,
        0,
        NAVGRID_CARVE_CHEBYSHEV_SQUARE,
        NAVGRID_LINE_CENTER_CELLS,
        NAVGRID_CARVE_EFFECTIVE_BLOCKED,
        NAVGRID_CARVE_ATOMIC,
        0,
        1,
        nullptr,
        nullptr
    };
    std::size_t changed = 99;
    const navsys_status_t status = navgrid_carve_area(
        grid, &center, &options, &changed);
    navgrid_destroy(grid);
    return status == NAVSYS_STATUS_OK && changed == 0 ? 0 : 2;
}
