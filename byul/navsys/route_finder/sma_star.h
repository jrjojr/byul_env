/*
 * Copyright (c) 2025-2026 ByulPapa (byuldev@outlook.kr)
 * This file is part of the Byul World project.
 * Licensed under the Byul World Source-Available Non-Commercial License v1.0 (2025).
 * See the LICENSE file in the project root for full license terms.
 */

/**
 * @file sma_star.h
 * @brief Declares the legacy Simplified Memory-Bounded A* route search C ABI.
 */

#ifndef BYUL_SMA_STAR_H
#define BYUL_SMA_STAR_H

#include "route_finder_core.h"

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
 * @param[in] m             Map object (created via navgrid_create or navgrid_load).
 * @param[in] start         Starting coordinate (created via coord_create or coord_create_full).
 * @param[in] goal          Goal coordinate (coord_t struct, passed as const).
 * @param[in] cost_fn       Movement cost function (default_cost is used if NULL).
 * @param[in] heuristic_fn  Heuristic function (default_heuristic is used if NULL).
 * @param[in] memory_limit  Maximum number of nodes to keep in memory during search.
 *                      (0 or too small values may cause search failure.)
 * @param[in] max_retry     Maximum number of retries if the path fails (0 means single attempt).
 * @param[in] debug_mode_enabled If TRUE, logs the number of visited cells internally.
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
 * @byul.nullable m false
 * @byul.nullable start false
 * @byul.nullable goal false
 * @byul.nullable cost_fn true
 * @byul.nullable heuristic_fn true
 * @byul.nullable return true
 * @byul.lifetime return caller-owned
 */
BYUL_API route_t* find_sma_star(const navgrid_t* m,
    const coord_t* start, const coord_t* goal,
    cost_func cost_fn, heuristic_func heuristic_fn,
    int memory_limit,
    int max_retry, bool debug_mode_enabled);

#ifdef __cplusplus
}
#endif

#endif /* BYUL_SMA_STAR_H */
