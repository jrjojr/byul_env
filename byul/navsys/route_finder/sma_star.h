#ifndef SMA_STAR_H
#define SMA_STAR_H

#include "route_finder_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Finds the shortest path under memory constraints using the SMA* algorithm.
 *
 * SMA* (Simplified Memory-Bounded A*) operates similarly to A* by performing
 * priority-based search using f = g + h, but it limits the number of nodes
 * stored in memory. If the limit is exceeded, it removes the node with the
 * lowest priority and continues the search.
 *
 * This approach is suitable for environments with limited memory and is designed
 * to find the optimal path possible within the given constraints.
 *
 * Removed nodes lose their `came_from` path-tracing data, which may result
 * in partial or failed path reconstruction. The larger the limit, the closer
 * the results will be to A*, while a smaller limit reduces path quality
 * or may cause search failure.
 *
 * @param m             Map object (created via navgrid_create or navgrid_load).
 * @param start         Starting coordinate (created via coord_create or coord_create_full).
 * @param goal          Goal coordinate (coord_t struct, passed as const).
 * @param cost_fn       Movement cost function (default_cost is used if NULL).
 * @param heuristic_fn  Heuristic function (default_heuristic is used if NULL).
 * @param memory_limit  Maximum number of nodes to keep in memory during search.
 *                      (0 or too small values may cause search failure.)
 * @param max_retry     Maximum number of retries if the path fails (0 means single attempt).
 * @param visited_logging If TRUE, logs the number of visited cells internally.
 *
 * @return route_t* path result
 *         - On success: route_get_success(route_t*) == true
 *         - On failure: route is empty and success == false
 *
 * @note
 * memory_limit should be adjusted based on map size and complexity.
 * General recommendation:
 *   - memory_limit ~= max(L * (1 + epsilon), N * alpha)
 *     (L: expected path length, N: number of map cells)
 *     (epsilon in [0.5, 1.0], alpha in [0.01, 0.05])
 *
 * @par Example of recommended memory limits:
 *   - 10x10 map  : memory_limit ~= 20 ~ 30
 *   - 100x100 map: memory_limit ~= 500 ~ 1000
 *   - 1000x1000 map: memory_limit ~= 50,000 ~ 100,000
 *
 */
BYUL_API route_t* find_sma_star(const navgrid_t* m,
    const coord_t* start, const coord_t* goal,
    cost_func cost_fn, heuristic_func heuristic_fn,
    int memory_limit,
    int max_retry, bool visited_logging);

#ifdef __cplusplus
}
#endif

#endif // SMA_STAR_H
