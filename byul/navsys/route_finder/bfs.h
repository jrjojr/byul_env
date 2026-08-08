/*
 * Copyright (c) 2025-2026 ByulPapa (byuldev@outlook.kr)
 * This file is part of the Byul World project.
 * Licensed under the Byul World Source-Available Non-Commercial License v1.0 (2025).
 * See the LICENSE file in the project root for full license terms.
 */

/**
 * @file bfs.h
 * @brief Declares the breadth-first route search public C ABI.
 */

#ifndef BYUL_BFS_H
#define BYUL_BFS_H

#include "byul_config.h"
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
 * @param[in] m                The map object where the search is performed.
 * @param[in] start            Start coordinate.
 * @param[in] goal             Goal coordinate.
 * @param[in] max_retry        Maximum number of iterations (recommended: width * height).
 * @param[in] debug_mode_enabled  Whether to record visited coordinate counts.
 * 
 * @return A @c route_t* object.
 *         - If @c route_get_success(result) is true, the path search succeeded.
 *         - If failed, the path includes the last reached coordinate.
 * @byul.nullable m false
 * @byul.nullable start false
 * @byul.nullable goal false
 * @byul.nullable return true
 * @byul.lifetime return caller-owned
 */
BYUL_API route_t* find_bfs(const navgrid_t* m, 
    const coord_t* start, const coord_t* goal, 
    int max_retry, bool debug_mode_enabled);

#ifdef __cplusplus
}
#endif

#endif /* BYUL_BFS_H */
