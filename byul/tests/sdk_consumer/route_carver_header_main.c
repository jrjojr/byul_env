#include "route_carver.h"

#include <stddef.h>
#include <stdint.h>

_Static_assert(NAVGRID_CARVE_OPTIONS_ABI_VERSION == UINT32_C(1),
    "unexpected route-carver options ABI version");
#if UINTPTR_MAX == UINT64_MAX
_Static_assert(sizeof(navgrid_carve_options_t) == 56,
    "unexpected x64 route-carver options layout");
_Static_assert(offsetof(navgrid_carve_options_t, cancel_func) == 40,
    "unexpected x64 route-carver callback offset");
#endif

int main(void) {
    navgrid_t* grid = navgrid_create_full(1, 1, NAVGRID_DIR_8, NULL);
    if (grid == NULL) return 1;

    const coord_t center = {0, 0};
    const navgrid_carve_options_t options = {
        sizeof(navgrid_carve_options_t),
        NAVGRID_CARVE_OPTIONS_ABI_VERSION,
        0,
        NAVGRID_CARVE_CHEBYSHEV_SQUARE,
        NAVGRID_LINE_CENTER_CELLS,
        NAVGRID_CARVE_EFFECTIVE_BLOCKED,
        NAVGRID_CARVE_ATOMIC,
        0,
        1,
        NULL,
        NULL
    };
    size_t changed = 99;
    const navsys_status_t status = navgrid_carve_area(
        grid, &center, &options, &changed);
    navgrid_destroy(grid);
    return status == NAVSYS_STATUS_OK && changed == 0 ? 0 : 2;
}
