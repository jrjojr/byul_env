/*
 * Copyright (c) 2025-2026 ByulPapa (byuldev@outlook.kr)
 * This file is part of the Byul World project.
 * Licensed under the Byul World Source-Available Non-Commercial License v1.0 (2025).
 * See the LICENSE file in the project root for full license terms.
 */

/**
 * @file fast_marching.h
 * @brief Declares the legacy Fast Marching route search C ABI.
 */

#ifndef BYUL_FAST_MARCHING_H
#define BYUL_FAST_MARCHING_H

#include "route_finder_core.h"
#include "cost_coord_pq.h"
#include "navgrid.h"

#ifdef __cplusplus
extern "C" {
#endif

/// Maximum propagation radius limit (values <= 0 or greater than this are replaced by MAX_RADIUS)
#define MAX_RADIUS 1e6f

typedef enum e_fmm_state {
    FMM_FAR = 0,
    FMM_NARROW,
    FMM_KNOWN
} fmm_state_t;

typedef struct s_fmm_cell {
    fmm_state_t state;
    float value;  // Distance value T
} fmm_cell_t;

/**
 * @brief Initializes a Fast Marching cell to the far state.
 * @param[out] out Cell to initialize.
 * @byul.nullable out false
 */
BYUL_API void fmm_cell_init(fmm_cell_t* out);

/**
 * @brief Initializes a Fast Marching cell from explicit values.
 * @param[out] out Cell to initialize.
 * @param[in] state Cell state.
 * @param[in] value Distance value.
 * @byul.nullable out false
 */
BYUL_API void fmm_cell_init_full(
    fmm_cell_t* out, fmm_state_t state, float value);

/**
 * @brief Assigns one Fast Marching cell value to another.
 * @param[out] out Destination cell.
 * @param[in] src Source cell.
 * @byul.nullable out false
 * @byul.nullable src false
 */
BYUL_API void fmm_cell_assign(fmm_cell_t* out, const fmm_cell_t* src);

typedef struct s_fmm_grid{
    int width;
    int height;
    coord_hash_t* cells;        // coord_t* -> fmm_cell_t*
    coord_list_t* visit_order;  // Visit history
    int total_retry_count;
} fmm_grid_t;

/**
 * @brief Computes a legacy distance field from a start point.
 *
 * @param[in] m             Map information
 * @param[in] start         Start coordinate
 * @param[in] cost_fn       Movement cost function (if nullptr, fixed cost 1.0 is used)
 * @param[in] radius_limit  Maximum search radius (if <= 0, MAX_RADIUS is used)
 * @param[in] max_retry     Maximum iteration limit (if <= 0, unlimited)
 *
 * @return fmm_grid_t*  The computed distance field structure (allocated dynamically)
 * @byul.nullable m false
 * @byul.nullable start false
 * @byul.nullable cost_fn true
 * @byul.nullable return true
 * @byul.lifetime return caller-owned
 */
BYUL_API fmm_grid_t* fmm_grid_create_full(const navgrid_t* m, const coord_t* start, 
    cost_func cost_fn, float radius_limit, int max_retry);

/**
 * @brief Frees all dynamic memory owned by a legacy distance field.
 *
 * @param[in] grid The distance field created by fmm_grid_create_full().
 * @byul.nullable grid true
 */
BYUL_API void fmm_grid_destroy(fmm_grid_t* grid);

/**
 * @brief Dumps the distance field result as ASCII output to stdout.
 * If a cell does not exist, " .. " is displayed,
 * and if a value exists, only the integer part is shown.
 *
 * @param[in] grid The distance field structure.
 * @byul.nullable grid true
 */
BYUL_API void fmm_dump_ascii(const fmm_grid_t* grid);

/**
 * @brief Reconstructs a legacy path from a Fast Marching distance field.
 * Returns a failed route if the goal is unreachable.
 * If debug_mode_enabled is true, visit order from fmm_grid_t is recorded into route->visited.
 *
 * @param[in] m               Map information
 * @param[in] start           Start coordinate
 * @param[in] goal            Goal coordinate
 * @param[in] cost_fn         Movement cost function (if nullptr, fixed cost 1.0 is used)
 * @param[in] max_retry       Maximum number of iterations (recommended: width * height)
 * @param[in] debug_mode_enabled Whether to log visited coordinates
 *
 * @return route_t*       Path structure (with success flag)
 * @byul.nullable m false
 * @byul.nullable start false
 * @byul.nullable goal false
 * @byul.nullable cost_fn true
 * @byul.nullable return true
 * @byul.lifetime return caller-owned
 */
BYUL_API route_t* find_fast_marching(const navgrid_t* m, 
    const coord_t* start, const coord_t* goal,
    cost_func cost_fn, int max_retry, bool debug_mode_enabled);

#ifdef __cplusplus
}
#endif

#endif /* BYUL_FAST_MARCHING_H */
