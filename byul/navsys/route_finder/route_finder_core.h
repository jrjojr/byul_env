/*
 * Copyright (c) 2025-2026 ByulPapa (byuldev@outlook.kr)
 * This file is part of the Byul World project.
 * Licensed under the Byul World Source-Available Non-Commercial License v1.0 (2025).
 * See the LICENSE file in the project root for full license terms.
 */

/**
 * @file route_finder_core.h
 * @brief Declares legacy Route Finder evaluation callback helpers.
 */

#ifndef BYUL_ROUTE_FINDER_CORE_H
#define BYUL_ROUTE_FINDER_CORE_H

#include "route_finder_evaluation.h"
#include "byul_config.h"
#include "navgrid.h"
#include "coord.h"
#include "route.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DIAGONAL_COST 1.4142135f  // Approximation of sqrt 2

/**
 * @brief Cost function type.
 * 
 * @param m        Map object.
 * @param start    Start coordinate.
 * @param goal     Goal coordinate.
 * @param userdata User-defined data.
 * @return float   Cost value.
 */
typedef float (*cost_func)(
    const navgrid_t* m, const coord_t* start, const coord_t* goal,
    void* userdata);

/**
 * @brief Heuristic function type.
 * 
 * @param start    Start coordinate.
 * @param goal     Goal coordinate.
 * @param userdata User-defined data.
 * @return float   Estimated distance.
 */
typedef float (*heuristic_func)(
    const coord_t* start, const coord_t* goal, void* userdata);

/**
 * @brief Default cost function (always returns 1.0).
 * @param[in] m Map object.
 * @param[in] start Edge start.
 * @param[in] goal Edge end.
 * @param[in] userdata Unused caller data.
 * @return Unit cost 1.0.
 * @byul.nullable m false
 * @byul.nullable start false
 * @byul.nullable goal false
 * @byul.nullable userdata true
 */
BYUL_API float default_cost(
    const navgrid_t* m, const coord_t* start, const coord_t* goal,
    void* userdata);

/**
 * @brief Cost function returning 0 (all paths have equal cost).
 * @param[in] m Map object.
 * @param[in] start Edge start.
 * @param[in] goal Edge end.
 * @param[in] userdata Unused caller data.
 * @return Zero cost.
 * @byul.nullable m false
 * @byul.nullable start false
 * @byul.nullable goal false
 * @byul.nullable userdata true
 */
BYUL_API float zero_cost(
    const navgrid_t* m, const coord_t* start, const coord_t* goal,
    void* userdata);

/**
 * @brief Diagonal movement cost function (uses sqrt 2 approximation).
 * @param[in] m Map object.
 * @param[in] start Edge start.
 * @param[in] goal Edge end.
 * @param[in] userdata Unused caller data.
 * @return Unit or diagonal movement cost.
 * @byul.nullable m false
 * @byul.nullable start false
 * @byul.nullable goal false
 * @byul.nullable userdata true
 */
BYUL_API float diagonal_cost(
    const navgrid_t* m, const coord_t* start, const coord_t* goal,
    void* userdata);

/**
 * @brief Euclidean distance heuristic.
 * @param[in] start Start coordinate.
 * @param[in] goal Goal coordinate.
 * @param[in] userdata Unused caller data.
 * @return Euclidean distance.
 * @byul.nullable start false
 * @byul.nullable goal false
 * @byul.nullable userdata true
 */
BYUL_API float euclidean_heuristic(
    const coord_t* start, const coord_t* goal, void* userdata);

/**
 * @brief Manhattan distance heuristic.
 * @param[in] start Start coordinate.
 * @param[in] goal Goal coordinate.
 * @param[in] userdata Unused caller data.
 * @return Manhattan distance.
 * @byul.nullable start false
 * @byul.nullable goal false
 * @byul.nullable userdata true
 */
BYUL_API float manhattan_heuristic(
    const coord_t* start, const coord_t* goal, void* userdata);

/**
 * @brief Chebyshev distance heuristic.
 * @param[in] start Start coordinate.
 * @param[in] goal Goal coordinate.
 * @param[in] userdata Unused caller data.
 * @return Chebyshev distance.
 * @byul.nullable start false
 * @byul.nullable goal false
 * @byul.nullable userdata true
 */
BYUL_API float chebyshev_heuristic(
    const coord_t* start, const coord_t* goal, void* userdata);

/**
 * @brief Octile distance heuristic (8-direction movement).
 * @param[in] start Start coordinate.
 * @param[in] goal Goal coordinate.
 * @param[in] userdata Unused caller data.
 * @return Octile distance.
 * @byul.nullable start false
 * @byul.nullable goal false
 * @byul.nullable userdata true
 */
BYUL_API float octile_heuristic(
    const coord_t* start, const coord_t* goal, void* userdata);

/**
 * @brief Heuristic function that always returns 0 (for minimal search).
 * @param[in] start Start coordinate.
 * @param[in] goal Goal coordinate.
 * @param[in] userdata Unused caller data.
 * @return Zero estimate.
 * @byul.nullable start false
 * @byul.nullable goal false
 * @byul.nullable userdata true
 */
BYUL_API float zero_heuristic(
    const coord_t* start, const coord_t* goal, void* userdata);

/**
 * @brief Default heuristic (Euclidean).
 * @param[in] start Start coordinate.
 * @param[in] goal Goal coordinate.
 * @param[in] userdata Unused caller data.
 * @return Euclidean distance.
 * @byul.nullable start false
 * @byul.nullable goal false
 * @byul.nullable userdata true
 */
BYUL_API float default_heuristic(
    const coord_t* start, const coord_t* goal, void* userdata);

#ifdef __cplusplus
}
#endif

#endif /* BYUL_ROUTE_FINDER_CORE_H */
