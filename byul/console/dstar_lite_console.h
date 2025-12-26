#ifndef DSTAR_LITE_UTILS_H
#define DSTAR_LITE_UTILS_H

#include "byul_config.h"
#include "float_core.h"
#include "navgrid.h"
#include "coord.h"
#include "route.h"
#include "dstar_lite.h"
#include "coord_hash.h"
#include "dstar_lite_pqueue.h"

#ifdef __cplusplus
extern "C" {
#endif

// ------------------ Debug Table Output ------------------

/// @brief Print g table (g values per coordinate)
BYUL_API void dsl_debug_print_g_table(const navgrid_t* m, coord_hash_t* g_table);

/// @brief Print rhs table (rhs values per coordinate)
BYUL_API void dsl_debug_print_rhs_table(
    const navgrid_t* m, coord_hash_t* rhs_table);

// ------------------ D* Lite Full State Output ------------------

/// @brief Print all internal states of D* Lite.
/// @param dsl               D* Lite object
/// @param goal              Goal coordinate
/// @param km                Current km value
/// @param g_table           g value table
/// @param rhs_table         rhs value table
/// @param frontier          Priority queue
/// @param max_range         Maximum search range
/// @param retry_limit       Maximum retry count
/// @param debug_mode        Debug mode flag
/// @param update_counter    update_vertex count table
BYUL_API void dsl_debug_print_full_state(
    const dstar_lite_t* dsl,
    const coord_t* goal,
    float km,
    coord_hash_t* g_table,
    coord_hash_t* rhs_table,
    dstar_lite_pqueue_t* frontier,
    int max_range,
    int retry_limit,
    bool debug_mode,
    coord_hash_t* update_counter);

// ------------------ Simplified Output ------------------

/// @brief Print only core variables (g/rhs/priority queue)
BYUL_API void dsl_debug_print_state(
    const dstar_lite_t* dsl,
    const coord_t* goal,
    float km,
    coord_hash_t* g_table,
    coord_hash_t* rhs_table,
    dstar_lite_pqueue_t* frontier);

// ------------------ ASCII Map Output ------------------

BYUL_API void dsl_print_info(const dstar_lite_t* dsl);

/// @brief Print only the map (# and .)
BYUL_API void dsl_print_ascii_only_navgrid(const dstar_lite_t* dsl);

/// @brief Print map including start, goal, and path (S, G, *, ., #)
BYUL_API void dsl_print_ascii_route(
    const dstar_lite_t* dsl, const route_t* route, int margin);

/// @brief Print map including update_vertex count (S, G, *, #, numbers)
BYUL_API void dsl_print_ascii_update_count(
    const dstar_lite_t* dsl, route_t* route, int margin);

#ifdef __cplusplus
}
#endif

#endif // DSTAR_LITE_UTILS_H
