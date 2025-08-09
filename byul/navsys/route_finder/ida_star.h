#ifndef IDA_STAR_H
#define IDA_STAR_H

#include "route_finder_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Finds a path using the IDA* (Iterative Deepening A*) algorithm.
 *
 * IDA* combines depth-first search (DFS) with heuristic-based cost evaluation
 * similar to A*. In each iteration, it explores only nodes whose
 * f = g + h values do not exceed the current threshold, 
 * and gradually increases the threshold until the optimal path is found.
 *
 * This algorithm is designed to use less memory than A*, and can find optimal
 * paths with a relatively simple implementation without complex data structures.
 *
 * ### Advantages
 * - Very low memory usage. Unlike A*, it does not maintain large open/closed lists,
 *   making it suitable for large maps, mobile, or embedded environments.
 * - Guarantees optimal paths by iteratively adjusting the threshold.
 * - Simple to implement since it is based on DFS with an added threshold condition.
 *
 * ### Disadvantages
 * - May require many iterations, especially when the heuristic increases slowly.
 * - Can be unsuitable for real-time applications due to repeated iterations.
 * - Performance may be less predictable since it does not expand nodes via
 *   a priority queue like A*.
 *
 * ### Heuristic Function
 * - The heuristic_fn can be set to NULL, in which case the Manhattan distance is used by default.
 * - IDA* works best with the Manhattan distance. For example, on the same map:
 *   - Using Euclidean distance might require 760 iterations.
 *   - Using Manhattan distance might require only 88 iterations to reach the same goal.
 * - If you are unsure which heuristic to use, setting it to NULL is the safest choice.
 *
 * @param m               Map information.
 * @param start           Start coordinate.
 * @param goal            Goal coordinate.
 * @param cost_fn         Cost function for movement (can be NULL).
 * @param heuristic_fn    Heuristic function (can be NULL, defaults to Manhattan distance).
 * @param max_retry       Maximum number of iterations (0 or less means unlimited).
 * @param debug_mode_enabled Whether to log the visited path (true logs it to route->visited).
 * @return route_t*       The resulting path. If search fails, success == false.
 */
BYUL_API route_t* find_ida_star(const navgrid_t* m, 
    const coord_t* start, const coord_t* goal,
    cost_func cost_fn, heuristic_func heuristic_fn,
    int max_retry, bool debug_mode_enabled);

#ifdef __cplusplus
}
#endif

#endif // IDA_STAR_H
