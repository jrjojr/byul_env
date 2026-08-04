#include "navgrid.h"

_Static_assert(BYUL_NAVGRID_ABI_VERSION == UINT32_C(2),
    "unexpected Navgrid ABI version");

int main(void) {
    navgrid_t* grid = navgrid_create();
    if (grid == NULL) {
        return 1;
    }
    const int valid = navgrid_get_abi_version() == BYUL_NAVGRID_ABI_VERSION;
    navgrid_destroy(grid);
    return valid ? 0 : 2;
}
