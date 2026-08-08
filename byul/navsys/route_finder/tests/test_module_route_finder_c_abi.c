#include "route_finder.h"

int route_finder_c_abi_reports_type_supported(int type_value) {
    return route_finder_is_supported((route_finder_type_t)type_value) ? 1 : 0;
}

int route_finder_c_abi_exercises_stage4_symbols(void) {
    coord_t start = {0, 0};
    coord_t goal = {3, 4};
    float estimate = -1.0f;

    if (route_finder_type_get_name(ROUTE_FINDER_ASTAR) == 0)
        return 0;
    if (!route_finder_is_type_supported(ROUTE_FINDER_ASTAR))
        return 0;
    if (route_finder_heuristic_euclidean(
            &start, &goal, &estimate, 0) != NAVSYS_STATUS_OK)
        return 0;
    return estimate == 5.0f ? 1 : 0;
}
