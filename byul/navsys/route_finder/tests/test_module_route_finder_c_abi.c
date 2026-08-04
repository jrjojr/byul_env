#include "route_finder.h"

int route_finder_c_abi_reports_type_supported(int type_value) {
    return route_finder_is_supported((route_finder_type_t)type_value) ? 1 : 0;
}
