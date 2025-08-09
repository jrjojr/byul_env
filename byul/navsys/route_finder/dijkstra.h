#ifndef DIJKSTRA_H
#define DIJKSTRA_H

#include "byul_common.h"
#include "coord.h"
#include "navgrid.h"
#include "route.h"
#include "route_finder_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Finds the shortest path using the Dijkstra algorithm.
 *
 * This function implements the Dijkstra algorithm, which explores paths
 * based solely on the actual accumulated cost without heuristics.
 * The movement cost for each adjacent coordinate is determined by @p cost_fn,
 * and the algorithm accumulates these costs to find the path with the lowest total cost.
 *
 * Unlike A*, since it does not use a heuristic, it always guarantees the shortest path,
 * but it may explore more nodes in the process.
 *
 * Internally, the function constructs and manages a priority queue, cost table,
 * and path tracing table. The input requires the map, start/goal coordinates,
 * and an optional cost function.
 *
 * @param m              The map to search.
 * @param start          Start coordinate.
 * @param goal           Goal coordinate.
 * @param cost_fn        Function to calculate the movement cost between coordinates.
 *                       If NULL, a default cost of 1.0f is used.
 * @param max_retry      Maximum number of iterations (for loop prevention).
 * @param debug_mode_enabled Whether to log visited coordinates.
 * @return               A pointer to a route_t object.
 *                       If @c route_get_success(route_t*) is TRUE, the search succeeded.
 */
BYUL_API route_t* find_dijkstra(const navgrid_t* m, 
    const coord_t* start, const coord_t* goal, cost_func cost_fn,
    int max_retry, bool debug_mode_enabled);

#ifdef __cplusplus
}
#endif

#endif // DIJKSTRA_H
