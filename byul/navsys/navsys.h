// Copyright (c) 2025 ByulPapa (byuldev@outlook.kr)
// This file is part of the Byul World project.
// Licensed under the Byul World Public License v1.0
// See the LICENSE file for details.

#ifndef NAVSYS_H
#define NAVSYS_H

#include "coord.h"
#include "coord_list.h"
#include "coord_hash.h"

#include "route.h"
#include "navgrid.h"

#include "cost_coord_pq.h"
#include "route_finder.h"
#include "dstar_lite.h"

#include "maze.h"
#include "route_carver.h"
#include "obstacle.h"

#ifdef __cplusplus
extern "C" {
#endif

// navgrid, start, goal
BYUL_API route_t* navsys_find_astar(
    navgrid_t* ng, const coord_t* start, const coord_t* goal);

BYUL_API route_t* navsys_find_bfs(
    navgrid_t* ng, const coord_t* start, const coord_t* goal);

BYUL_API route_t* navsys_find_dfs(
    navgrid_t* ng, const coord_t* start, const coord_t* goal);

BYUL_API route_t* navsys_find_dijkstra(
    navgrid_t* ng, const coord_t* start, const coord_t* goal);

BYUL_API route_t* navsys_find_greedy_best_first(
    navgrid_t* ng, const coord_t* start, const coord_t* goal);

BYUL_API route_t* navsys_find_fast_marching(
    navgrid_t* ng, const coord_t* start, const coord_t* goal);

// For IDA*, the heuristic should be set to 
// Manhattan distance instead of Euclidean.    
BYUL_API route_t* navsys_find_ida_star(
    navgrid_t* ng, const coord_t* start, const coord_t* goal);

/**
 * @brief Runs the Fringe Search algorithm.
 *
 * This algorithm expands nodes by fringe threshold and
 * uses delta_epsilon for controlling search efficiency.
 *
 * @param a Pointer to route_finder_t containing execution settings.
 *          - userdata should be a float* pointing to delta_epsilon.
 *          - Recommended: 0.1 ~ 0.5 (no default, must be set by user).
 *
 * @return Calculated route_t* or NULL on failure.
 */
BYUL_API route_t* navsys_find_fringe_search(navgrid_t* ng, 
    const coord_t* start, const coord_t* goal, float delta_epsilon);

/**
 * @brief Runs Real-Time A* (RTA*) algorithm.
 *
 * This algorithm performs limited-depth search for real-time responsiveness.
 *
 * @param a Pointer to route_finder_t containing execution settings.
 *          - userdata should be an int* pointing to depth_limit.
 *          - Recommended: 3 ~ 10 (higher is more accurate but slower).
 *
 * @return Calculated route_t* or NULL on failure.
 */
BYUL_API route_t* navsys_find_rta_star(navgrid_t* ng, 
    const coord_t* start, const coord_t* goal, int depth_limit);

/**
* Memory limit should depend on map size and complexity. Recommended values:
 *   - memory_limit === max(L × (1 + e), N x a)
 *     (L: expected path length, N: number of map cells)
 *     (e <== [0.5, 1.0], a <== [0.01, 0.05])
 *
 * @par Example memory limits:
 *   - 10x10 map  : memory_limit === 20 ~ 30
 *   - 100x100 map: memory_limit === 500 ~ 1000
 *   - 1000x1000 map: memory_limit === 50,000 ~ 100,000
 *
 */
BYUL_API route_t* navsys_find_sma_star(navgrid_t* ng, 
    const coord_t* start, const coord_t* goal, int memory_limit);

/**
 * @brief Runs the Weighted A* algorithm.
 *
 * This algorithm applies a weight to the heuristic
 * to speed up pathfinding.
 *
 * @param a Pointer to route_finder_t containing execution settings.
 *          - userdata should be a float* pointing to the weight.
 *          - Recommended: 
                * 1.0 (standard A*), 
                * 1.2 ~ 2.5 (faster), 
                * 5.0+ may be inaccurate.
 *
 * @return Calculated route_t* or NULL on failure.
 */
BYUL_API route_t* navsys_find_weighted_astar(navgrid_t* ng, 
    const coord_t* start, const coord_t* goal, float weight);

#ifdef __cplusplus
}
#endif

#endif // NAVSYS_H
