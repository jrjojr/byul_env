/*
 * Copyright (c) 2025-2026 ByulPapa (byuldev@outlook.kr)
 * This file is part of the Byul World project.
 * Licensed under the Byul World Source-Available Non-Commercial License v1.0 (2025).
 * See the LICENSE file in the project root for full license terms.
 */

/**
 * @file fringe_search.h
 * @brief Declares the legacy Fringe Search route search C ABI.
 */

#ifndef BYUL_FRINGE_SEARCH_H
#define BYUL_FRINGE_SEARCH_H

#include "route_finder_core.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Finds a path using the Fringe Search algorithm.
 *
 * Fringe Search removes the Open List sorting bottleneck of A*
 * by exploring nodes based on a gradually increasing threshold of
 * f = g + h values. Two priority queues are alternated:
 * nodes within the current threshold are processed,
 * and nodes exceeding threshold + delta_epsilon are deferred to the next round.
 *
 * While Fringe Search is faster than A*, it does not guarantee an optimal path
 * due to the lack of strict f-value sorting.
 *
 * ### Meaning and recommended values of delta_epsilon
 * - `delta_epsilon` defines the tolerance for the threshold.
 * - Larger values allow a broader search with reduced pruning, which may slow performance.
 * - Smaller values allow tighter pruning but risk failing to find a path.
 *
 * Example:
 * - For a 10x10 map using Euclidean heuristics:
 *   - `delta_epsilon = 0.5f`: tight pruning, higher failure risk
 *   - `delta_epsilon = 1.5f`: balanced setting (recommended)
 *   - `delta_epsilon >= 3.0f`: wide search, potential performance issues
 *
 * @param[in] m               Map information
 * @param[in] start           Start coordinate
 * @param[in] goal            Goal coordinate
 * @param[in] cost_fn         Movement cost function (if NULL, default_cost is used)
 * @param[in] heuristic_fn    Heuristic function (if NULL, default_heuristic is used)
 * @param[in] delta_epsilon   Threshold tolerance (if <= 0, defaults to 0.5f)
 * @param[in] max_retry       Maximum iteration count (if <= 0, unlimited)
 * @param[in] debug_mode_enabled Whether to log visited nodes (true logs visit order into the route)
 *
 * @return route_t* The search result. If success is true, a path was found.
 *         Even on failure, the path to the last explored node is recorded.
 * @byul.nullable m false
 * @byul.nullable start false
 * @byul.nullable goal false
 * @byul.nullable cost_fn true
 * @byul.nullable heuristic_fn true
 * @byul.nullable return true
 * @byul.lifetime return caller-owned
 */
BYUL_API route_t* find_fringe_search(const navgrid_t* m, 
    const coord_t* start, const coord_t* goal,
    cost_func cost_fn, heuristic_func heuristic_fn, float delta_epsilon,
    int max_retry, bool debug_mode_enabled);

#ifdef __cplusplus
}
#endif

#endif /* BYUL_FRINGE_SEARCH_H */
