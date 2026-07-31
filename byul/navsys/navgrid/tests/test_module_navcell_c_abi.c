#include "navcell.h"

int navcell_c_abi_reports_terrain_supported(
    int terrain_value, int* out_supported) {
    bool supported = true;
    const navsys_status_t status = navcell_is_terrain_supported(
        (terrain_type_t)terrain_value, &supported);
    if (status != NAVSYS_STATUS_OK) return (int)status;
    *out_supported = supported ? 1 : 0;
    return (int)NAVSYS_STATUS_OK;
}
