#ifndef ASTAR_H
#define ASTAR_H

#include "byul_common.h"

#include "route_finder_common.h"
#include "navgrid.h"
#include "coord.h"
#include "route.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Finds the shortest path using the A* algorithm.
 *
 * A* is a heuristic-based algorithm that explores paths using the cost function:
 *
 *     f(n) = g(n) + h(n)
 *
 * where g(n) is the actual cost to reach node n, and h(n) is the estimated distance to the goal.
 *
 * This function searches for a path from @p start to @p goal on the map @p m,  
 * using the cost function @p cost_fn and the heuristic function @p heuristic_fn
 * (if NULL, default values are used: cost = 1.0 and heuristic = Euclidean distance).
 *
 * Internally, a priority queue, cost table, and path tracing table are used.  
 * The final path is returned as a `route_t*` structure.
 *
 * @param m                The map to search on.
 * @param start            Start coordinate.
 * @param goal             Goal coordinate.
 * @param cost_fn          Function to calculate the movement cost between coordinates.  
 *                         Defaults to 1.0 if NULL.
 * @param heuristic_fn     Function to calculate the heuristic distance.  
 *                         Defaults to Euclidean distance if NULL.
 * @param max_retry        Maximum number of retries allowed during search.
 * @param debug_mode_enabled  Whether to log visited nodes.
 * @return                 A pointer to the route object.  
 *                         Check @c route_get_success(route_t*) to determine if a valid path was found.
 *
 * @see route_get_success(), cost_func, heuristic_func
 */
BYUL_API route_t* find_astar(const navgrid_t* m, const coord_t* start, const coord_t* goal,
    cost_func cost_fn, heuristic_func heuristic_fn,
    int max_retry, bool debug_mode_enabled);

#ifdef __cplusplus
}
#endif

#endif // ASTAR_H
