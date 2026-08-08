/*
 * Copyright (c) 2025-2026 ByulPapa (byuldev@outlook.kr)
 * This file is part of the Byul World project.
 * Licensed under the Byul World Source-Available Non-Commercial License v1.0 (2025).
 * See the LICENSE file in the project root for full license terms.
 */

/**
 * @file greedy_best_first.h
 * @brief Declares the greedy best-first route search public C ABI.
 */

#ifndef BYUL_GREEDY_BEST_FIRST_H
#define BYUL_GREEDY_BEST_FIRST_H

#include "route_finder_core.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Finds a path using the Greedy Best-First Search algorithm.
 *
 * This algorithm ignores the accumulated cost (g) and searches for the path
 * that appears "closest" to the goal based solely on the heuristic value (h),
 * using a priority queue for node selection.
 *
 * - The heuristic function (heuristic_fn) must be provided (e.g., default_heuristic).
 * - The search terminates if the number of iterations exceeds max_retry.
 * - If debug_mode_enabled is true, the search order is recorded in route->visited.
 *
 * @param[in] m               Map information
 * @param[in] start           Start coordinate
 * @param[in] goal            Goal coordinate
 * @param[in] heuristic_fn    Heuristic function (required)
 * @param[in] max_retry       Maximum iteration count (if <= 0, unlimited)
 * @param[in] debug_mode_enabled Whether to log the search order
 *
 * @return route_t* The resulting path. If the search fails, success == false.
 * @byul.nullable m false
 * @byul.nullable start false
 * @byul.nullable goal false
 * @byul.nullable heuristic_fn false
 * @byul.nullable return true
 * @byul.lifetime return caller-owned
 */
BYUL_API route_t* find_greedy_best_first(const navgrid_t* m, 
    const coord_t* start, const coord_t* goal,
    heuristic_func heuristic_fn,
    int max_retry, bool debug_mode_enabled);

#ifdef __cplusplus
}
#endif

#endif /* BYUL_GREEDY_BEST_FIRST_H */
