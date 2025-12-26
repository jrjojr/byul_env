#ifndef RTA_STAR_H
#define RTA_STAR_H

#include "route_finder_core.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct s_rta_star_config{
    int depth_limit; // Search depth limit
} rta_star_config_t;
typedef rta_star_config_t* rta_star_config;

BYUL_API rta_star_config rta_star_config_create();

/**
 * @brief Creates the depth limit setting for the RTA* algorithm.
 *
 * RTA* (Real-Time A*) does not explore the entire path at once,
 * but instead looks ahead only a limited depth from the current position
 * to select the most promising next step.
 *
 * This function sets the "lookahead depth."
 * The algorithm evaluates neighbor nodes up to the given depth
 * and selects the direction with the lowest g + h cost.
 *
 * The depth limit is closely related to the structure of obstacles.
 *
 * - A low depth limit gives quick responses but may fail to avoid complex obstacles.
 * - For example, if an obstacle spans 5 cells, the depth_limit must be at least 6
 *   to detect a detour.
 * - If the depth_limit is too small, the algorithm might consider a blocked path as the best route and fail.
 *
 * General recommendations:
 * - Real-time reactive movement (NPC AI): 2~4
 * - Complex obstacle navigation: 6~8
 * - For maze-like structures, A* or Dijkstra is often more suitable.
 *
 * @param depth_limit The lookahead depth (recommended: 1 or higher)
 * @return rta_star_config configuration object (must be freed)
 */
BYUL_API rta_star_config rta_star_config_create_full(int depth_limit);

BYUL_API void rta_star_config_destroy(rta_star_config cfg);

/**
 * @brief Finds a path using the RTA* (Real-Time A*) algorithm.
 *
 * This algorithm does not compute the full path at once.
 * It uses a heuristic lookahead up to a specified depth
 * and moves one step at a time towards the most promising direction.
 *
 * Each step performs:
 * - Lookahead search up to depth_limit from the current position.
 * - Selects the neighbor node with the lowest g + h evaluation.
 * - Moves one step and repeats.
 *
 * If the depth limit is insufficient, the algorithm may fail to avoid obstacles
 * and terminate early with an incorrect path.
 *
 * Example: On a 10x10 map with a vertical wall in the center,
 *          if depth_limit <= 6, the detour might not be detected (failure),
 *          but with depth_limit >= 7, the algorithm will successfully reach the goal.
 *
 * Usage example:
 * @code
 * coord_t* start = coord_create_full(0, 0);
 * coord_t* goal = coord_create_full(9, 9);
 *
 * rta_star_config cfg = rta_star_config_create_full(7); // Depth limit = 7
 * route_finder al = route_finder_create_full(
 *     10, 10,
 *     NAVGRID_DIR_8,
 *     ROUTE_FINDER_RTA_STAR,
 *     default_cost,
 *     default_heuristic,
 *     NULL,
 *     cfg,
 *     true
 * );
 *
 * // Add vertical wall in the center
 * for (int y = 1; y < 10; y++)
 *     navgrid_block_coord(al->m, 5, y);
 *
 * route_t* p = route_finder_run(al, start, goal);
 * navgrid_print_ascii_with_visited_count(al->m, p, start, goal);
 *
 * route_destroy(p);
 * coord_destroy(start);
 * coord_destroy(goal);
 * route_finder_destroy(al);
 * rta_star_config_destroy(cfg);
 * @endcode
 *
 * @param m             The map
 * @param start         Starting coordinate
 * @param goal          Goal coordinate
 * @param cost_fn       Cost function
 * @param heuristic_fn  Heuristic function
 * @param depth_limit   Lookahead search depth limit
 * @param max_retry     Maximum number of iterations
 * @param debug_mode_enabled Whether to log visited nodes
 * @return route_t* Path result. success == true if a path is found, false otherwise.
 */
BYUL_API route_t* find_rta_star(const navgrid_t* m,
    const coord_t* start, const coord_t* goal,
    cost_func cost_fn, heuristic_func heuristic_fn,
    int depth_limit,
    int max_retry, bool debug_mode_enabled);

#ifdef __cplusplus
}
#endif

#endif // RTA_STAR_H
