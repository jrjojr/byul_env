#ifndef BFS_H
#define BFS_H

#include "byul_common.h"
#include "coord.h"
#include "navgrid.h"
#include "route.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Finds the shortest path on a map using BFS (Breadth-First Search).
 *
 * This function performs a BFS (FIFO queue) search on the given map (@p m)
 * to find the shortest path (in terms of steps) from the start coordinate (@p start)
 * to the goal coordinate (@p goal).
 *
 * Internally, it tracks the set of visited coordinates and the path of movement,
 * with a limit of @p max_retry iterations to prevent infinite loops.
 *
 * - If a path is found: a route from start -> goal is returned, and success is true.
 * - If no path exists: a route up to the last explored coordinate is returned, and success is false.
 * - If @p debug_mode_enabled is true, visit counts for all explored coordinates are recorded.
 *
 * @param m                The map object where the search is performed.
 * @param start            Start coordinate.
 * @param goal             Goal coordinate.
 * @param max_retry        Maximum number of iterations (recommended: width * height).
 * @param debug_mode_enabled  Whether to record visited coordinate counts.
 * 
 * @return A @c route_t* object.
 *         - If @c route_get_success(result) is true, the path search succeeded.
 *         - If failed, the path includes the last reached coordinate.
 */
BYUL_API route_t* find_bfs(const navgrid_t* m, 
    const coord_t* start, const coord_t* goal, 
    int max_retry, bool debug_mode_enabled);

#ifdef __cplusplus
}
#endif

#endif // BFS_H
