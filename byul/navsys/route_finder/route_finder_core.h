#ifndef ROUTE_FINDER_CORE_H
#define ROUTE_FINDER_CORE_H

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
    const navgrid_t*, const coord_t*, const coord_t*, void*);

/**
 * @brief Heuristic function type.
 * 
 * @param start    Start coordinate.
 * @param goal     Goal coordinate.
 * @param userdata User-defined data.
 * @return float   Estimated distance.
 */
typedef float (*heuristic_func)(const coord_t*, const coord_t*, void*);

/**
 * @brief Default cost function (always returns 1.0).
 */
BYUL_API float default_cost(
    const navgrid_t*, const coord_t*, const coord_t*, void*);

/**
 * @brief Cost function returning 0 (all paths have equal cost).
 */
BYUL_API float zero_cost(const navgrid_t*, const coord_t*, const coord_t*, void*);

/**
 * @brief Diagonal movement cost function (uses sqrt 2 approximation).
 */
BYUL_API float diagonal_cost(
    const navgrid_t*, const coord_t*, const coord_t*, void*);

/**
 * @brief Euclidean distance heuristic.
 */
BYUL_API float euclidean_heuristic(const coord_t*, const coord_t*, void*);

/**
 * @brief Manhattan distance heuristic.
 */
BYUL_API float manhattan_heuristic(const coord_t*, const coord_t*, void*);

/**
 * @brief Chebyshev distance heuristic.
 */
BYUL_API float chebyshev_heuristic(const coord_t*, const coord_t*, void*);

/**
 * @brief Octile distance heuristic (8-direction movement).
 */
BYUL_API float octile_heuristic(const coord_t*, const coord_t*, void*);

/**
 * @brief Heuristic function that always returns 0 (for minimal search).
 */
BYUL_API float zero_heuristic(const coord_t*, const coord_t*, void*);

/**
 * @brief Default heuristic (Euclidean).
 */
BYUL_API float default_heuristic(const coord_t*, const coord_t*, void*);

#ifdef __cplusplus
}
#endif

#endif // ROUTE_FINDER_CORE_H
