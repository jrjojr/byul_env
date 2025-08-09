#ifndef FAST_MARCHING_H
#define FAST_MARCHING_H

#include "route_finder_common.h"
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

BYUL_API void fmm_cell_init(fmm_cell_t* out);

BYUL_API void fmm_cell_init_full(
    fmm_cell_t* out, fmm_state_t state, float value);

BYUL_API void fmm_cell_assign(fmm_cell_t* out, const fmm_cell_t* src);

// typedef void* (*coord_hash_copy_func)(const void* value);
void* fmm_cell_copy(const void* p);

// typedef void* (*coord_hash_copy_func)(const void* value); 
void fmm_cell_destroy(void* p);

typedef struct s_fmm_grid{
    int width;
    int height;
    coord_hash_t* cells;        // coord_t* -> fmm_cell_t*
    coord_list_t* visit_order;  // Visit history
    int total_retry_count;
} fmm_grid_t;

/**
 * Computes the distance field from the start point using the Fast Marching Method (FMM).
 *
 * @param m             Map information
 * @param start         Start coordinate
 * @param cost_fn       Movement cost function (if nullptr, fixed cost 1.0 is used)
 * @param radius_limit  Maximum search radius (if <= 0, MAX_RADIUS is used)
 * @param max_retry     Maximum iteration limit (if <= 0, unlimited)
 *
 * @return fmm_grid_t*  The computed distance field structure (allocated dynamically)
 */
BYUL_API fmm_grid_t* fmm_grid_create_full(const navgrid_t* m, const coord_t* start, 
    cost_func cost_fn, float radius_limit, int max_retry);

/**
 * Frees all dynamic memory owned by the fmm_grid_t structure.
 *
 * @param grid  The distance field structure created by fmm_grid_create_full()
 */
BYUL_API void fmm_grid_destroy(fmm_grid_t* grid);

/**
 * Dumps the distance field result as ASCII output to stdout.
 * If a cell does not exist, " .. " is displayed,
 * and if a value exists, only the integer part is shown.
 *
 * @param grid  The distance field structure
 */
BYUL_API void fmm_dump_ascii(const fmm_grid_t* grid);

/**
 * Reconstructs the shortest path from start -> goal based on the Fast Marching Method.
 * Returns a failed route if the goal is unreachable.
 * If debug_mode_enabled is true, visit order from fmm_grid_t is recorded into route->visited.
 *
 * @param m               Map information
 * @param start           Start coordinate
 * @param goal            Goal coordinate
 * @param cost_fn         Movement cost function (if nullptr, fixed cost 1.0 is used)
 * @param max_retry       Maximum number of iterations (recommended: width * height)
 * @param debug_mode_enabled Whether to log visited coordinates
 *
 * @return route_t*       Path structure (with success flag)
 */
BYUL_API route_t* find_fast_marching(const navgrid_t* m, 
    const coord_t* start, const coord_t* goal,
    cost_func cost_fn, int max_retry, bool debug_mode_enabled);

#ifdef __cplusplus
}
#endif

#endif // FAST_MARCHING_H
