#ifndef GREEDY_BEST_FIRST_H
#define GREEDY_BEST_FIRST_H

#include "route_finder_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Finds a path using the Greedy Best-First Search algorithm.
 *
 * This algorithm ignores the accumulated cost (g) and searches for the path
 * that appears "closest" to the goal based solely on the heuristic value (h),
 * using a priority queue for node selection.
 *
 * - The heuristic function (heuristic_fn) must be provided (e.g., default_heuristic).
 * - The search terminates if the number of iterations exceeds max_retry.
 * - If visited_logging is true, the search order is recorded in route->visited.
 *
 * @param m               Map information
 * @param start           Start coordinate
 * @param goal            Goal coordinate
 * @param heuristic_fn    Heuristic function (required)
 * @param max_retry       Maximum iteration count (if <= 0, unlimited)
 * @param visited_logging Whether to log the search order
 *
 * @return route_t* The resulting path. If the search fails, success == false.
 */
BYUL_API route_t* find_greedy_best_first(const navgrid_t* m, 
    const coord_t* start, const coord_t* goal,
    heuristic_func heuristic_fn,
    int max_retry, bool visited_logging);

#ifdef __cplusplus
}
#endif

#endif // GREEDY_BEST_FIRST_H
