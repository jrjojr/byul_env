#ifndef DFS_H
#define DFS_H

#include "byul_config.h"
#include "scalar.h"
#include "coord.h"
#include "navgrid.h"
#include "route.h"
#include "route_finder_core.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Finds a path on the map using DFS (Depth-First Search).
 *
 * This function uses a stack-based LIFO DFS algorithm to search for a path
 * from the start coordinate (@p start) to the goal coordinate (@p goal) on the given map (@p m).
 *
 * DFS explores as deeply as possible along one branch before backtracking.
 * Since it does not consider weights or heuristics, it does not guarantee
 * an optimal path but is simple and effective for fast path exploration.
 *
 * - If a path is found: a route from start -> goal is returned, and success is true.
 * - If no path is found: a route up to the last explored coordinate is returned, and success is false.
 * - If @p debug_mode_enabled is true, visit counts for all visited coordinates are recorded.
 *
 * @param m                The map object for pathfinding.
 * @param start            Start coordinate.
 * @param goal             Goal coordinate.
 * @param max_retry        Maximum number of iterations (recommended: width * height).
 * @param debug_mode_enabled  Whether to log visited coordinate counts.
 * 
 * @return A @c route_t* object.  
 *         - If @c route_get_success(result) is true, the route successfully reached the goal.
 *         - If false, the route includes the path up to the last explored point.
 */
BYUL_API route_t* find_dfs(const navgrid_t* m, 
    const coord_t* start, const coord_t* goal, 
    int max_retry, bool debug_mode_enabled);

#ifdef __cplusplus
}
#endif

#endif // DFS_H
