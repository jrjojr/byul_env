#ifndef WEIGHTED_ASTAR_H
#define WEIGHTED_ASTAR_H

#include "byul_config.h"
#include "scalar.h"

#include "route_finder_core.h"
#include "navgrid.h"
#include "coord.h"
#include "route.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Finds a path using the Weighted A* algorithm.
 *
 * Weighted A* is an extension of the classic A* algorithm that introduces
 * a weight parameter to adjust the influence of the heuristic h(n).
 * The f-score is calculated as follows:
 *
 *     f(n) = g(n) + weight * h(n)
 *
 * - g(n): The accumulated cost from the start node to the current node.
 * - h(n): The heuristic estimate from the current node to the goal.
 * - weight: A coefficient that adjusts the influence of h(n) (recommended >= 1.0).
 *
 * When weight = 1.0, this behaves like standard A*. Higher values speed up
 * the search by relying more on the heuristic but may reduce path quality.
 *
 * @par Recommended weight range
 * - weight = 1.0 : Standard A* (optimal path, potentially slower)
 * - weight = 1.2 ~ 2.5 : Balanced tradeoff between performance and path quality
 * - weight >= 5.0 : Approaches greedy search (non-optimal paths possible)
 *
 * @note
 * - If @c weight <= 0.0, it is internally corrected to 1.0.
 * - If @c cost_fn or @c heuristic_fn is nullptr, default internal functions are used.
 * - Even if the search fails, the best possible partial path to the last reached node is returned.
 *
 * @param m               Map information
 * @param start           Start coordinate
 * @param goal            Goal coordinate
 * @param cost_fn         Cost function (can be nullptr)
 * @param heuristic_fn    Heuristic function (can be nullptr)
 * @param weight          Weight applied to the heuristic
 * @param max_retry       Maximum number of iterations (0 or less means unlimited)
 * @param debug_mode_enabled Whether to log visited cells
 * @return route_t*       Path object (check the success field for the result)
 */
BYUL_API route_t* find_weighted_astar(const navgrid_t* m,
    const coord_t* start, const coord_t* goal,
    cost_func cost_fn, heuristic_func heuristic_fn,
    float weight,
    int max_retry, bool debug_mode_enabled);

#ifdef __cplusplus
}
#endif

#endif // WEIGHTED_ASTAR_H
