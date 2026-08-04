#include "route_carver.h"

#include <cstddef>
#include <cstdint>
#include <type_traits>

static_assert(NAVGRID_CARVE_OPTIONS_ABI_VERSION == UINT32_C(1));
static_assert(std::is_standard_layout_v<navgrid_carve_options_t>);
#if UINTPTR_MAX == UINT64_MAX
static_assert(sizeof(navgrid_carve_options_t) == 56);
static_assert(offsetof(navgrid_carve_options_t, cancel_func) == 40);
#endif

int main() {
    navgrid_t* grid = navgrid_create_full(1, 1, NAVGRID_DIR_8, nullptr);
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
