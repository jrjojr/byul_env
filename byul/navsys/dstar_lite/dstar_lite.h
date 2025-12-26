/**
 * @file dstar_lite.h
 * @brief D* Lite Algorithm Header
 *
 * 1. Overview of dstar_lite
 *    - After the initial route calculation, if obstacles on the map
 *      change dynamically, this algorithm is designed to quickly
 *      replan the route.
 *    - For this replanning, the core function used is update_vertex(),
 *      and nodes within a certain range (max_range) of the changed
 *      obstacle location must be updated.
 *
 * 2. Practical Guidelines for max_range (by Byeol Mom)
 *    - A suitable max_range depends on the map size and obstacle distribution.
 *    - Generally, the following guidelines can be used:
 *
 *      - Static / large maps (e.g., 100x100 or larger)  -> max_range = 10 ~ 20
 *      - Medium maps (e.g., 50x50)                     -> max_range = 5 ~ 10
 *      - Real-time / small maps (e.g., 20x20 or smaller) -> max_range = 3 ~ 5
 *
 *    - There is no formal formula, but the following approximation can be used:
 *
 *        max_range ~= ( |goal.x - start.x| + |goal.y - start.y| ) / 10
 *
 *      For example, if start=(0,0) and goal=(40,40), then max_range ~= 8.
 *
 *    - In the end, the optimal value should be tuned through experiments.
 *      If it is too small, a route may not be found; if it is too large,
 *      the computation cost can be excessive.
 *
 *    - Byeol Mom recommends using 10 as the initial setting.
 */

#ifndef DSTAR_LITE_H
#define DSTAR_LITE_H

#include "byul_config.h"

#include "float_core.h"

#include "navgrid.h"
#include "coord.h"
#include "coord_list.h"
#include "coord_hash.h"
#include "route.h"
#include "dstar_lite_key.h"
#include "dstar_lite_pqueue.h"
#include "route_finder.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*move_func)(const coord_t* c, void* userdata);

typedef coord_list_t* (*changed_coords_func)(void* userdata);

BYUL_API float dstar_lite_cost(
    const navgrid_t* navgrid, 
    const coord_t* start, 
    const coord_t* goal, 
    void* userdata);

/**
 * @brief Cost function for dynamic D* Lite pathfinding.
 *
 * Returns Euclidean distance between two coordinates.
 * If goal is an obstacle, returns FLT_MAX.
 *
 * The userdata can provide a coord_hash_t* that holds visit counts
 * to penalize repeatedly visited nodes and avoid route loops.
 */
BYUL_API float dstar_lite_dynamic_cost(
    const navgrid_t* navgrid,
    const coord_t* start,
    const coord_t* goal,
    void* userdata);

BYUL_API float dstar_lite_heuristic(
    const coord_t* start, const coord_t* goal, void* userdata);

BYUL_API void move_to(const coord_t* c, void* userdata);

// Default implementation for changed_coords_func function pointer.
// Provides a single coordinate. 
// userdata: coord* c
BYUL_API coord_list_t* get_changed_coord(void* userdata);

// Default implementation for changed_coords_func function pointer.
// Provides multiple coordinates via a coord_list.
// userdata: coord_list_t* list
BYUL_API coord_list_t* get_changed_coords(void* userdata);

typedef struct s_dstar_lite {
    // Shared members with route_finder
    // Intended to be unified into route_finder later
    navgrid_t* navgrid;
    coord_t start;
    coord_t goal;

    cost_func cost_fn;
    void* cost_fn_userdata;

    heuristic_func heuristic_fn;
    void* heuristic_fn_userdata;

    int max_retry;
    bool debug_mode_enabled;

    // D* Lite specific members
    float km;
    coord_hash_t* g_table;    // coord_t** -> float*
    coord_hash_t* rhs_table;  // coord_t** -> float*

    dstar_lite_pqueue_t* frontier;

    move_func move_fn;
    void* move_fn_userdata;

    changed_coords_func changed_coords_fn;
    void* changed_coords_fn_userdata;

    route_t* proto_route;
    route_t* real_route;

    int real_loop_max_retry;
    int reconstruct_max_retry;

    int proto_compute_retry_count;
    int real_compute_retry_count;
    int real_loop_retry_count;
    int reconstruct_retry_count;

    int max_range;

    // Loop timing interval (in seconds)
    float interval_sec;

    // Forced termination flag (used during loops)
    bool force_quit;

} dstar_lite_t;

/**
 * @brief Creates a D* Lite configuration object with default settings.
 *
 * Returns NULL if navgrid is not provided.
 * 
 * This function creates a D* Lite configuration structure 
 * with the following default values:
 * - start : (0, 0)
 * - goal : (0, 0)
 * - km : 0.0f
 * - max_range : 0 (only neighbors of the center coordinate are checked)
 * - real_loop_max_retry : 0 (no repeated searches)
 * - width : 0 (infinite map)
 * - height : 0 (infinite map)
 *
 * 8-directional movement, Euclidean distance, 
 * D* Lite cost, debug mode disabled.
 * The created configuration object is then used by the algorithm.
 *
 * @return Newly created dstar_lite_t* object.
 *         Must be freed using dstar_lite_destroy() after use.
 */
BYUL_API dstar_lite_t* dstar_lite_create(navgrid_t* navgrid);

/**
 * @brief Creates a D* Lite configuration object with user-defined settings.
 *
 * Returns NULL if navgrid is not provided.
 * 
 * @param debug_mode_enabled  Whether to enable debug mode.
 * @return Newly created dstar_lite_t* object.
 *         Must be freed using dstar_lite_destroy() after use.
 */
BYUL_API dstar_lite_t* dstar_lite_create_full(
    navgrid_t* navgrid, 
    const coord_t* start, 
    const coord_t* goal, 
    cost_func cost_fn, 
    heuristic_func heuristic_fn,
    bool debug_mode_enabled);

BYUL_API void dstar_lite_destroy(dstar_lite_t* dsl);

BYUL_API dstar_lite_t* dstar_lite_copy(const dstar_lite_t* src);

BYUL_API int dstar_lite_fetch_start(const dstar_lite_t* dsl, coord_t* out);

BYUL_API void  dstar_lite_set_start(dstar_lite_t* dsl, const coord_t* c);

BYUL_API int dstar_lite_fetch_goal(const dstar_lite_t* dsl, coord_t* out);

BYUL_API void  dstar_lite_set_goal(dstar_lite_t* dsl, const coord_t* c);

BYUL_API coord_hash_t* dstar_lite_get_g_table(const dstar_lite_t* dsl);

BYUL_API coord_hash_t* dstar_lite_get_rhs_table(const dstar_lite_t* dsl);

BYUL_API dstar_lite_pqueue_t* dstar_lite_get_frontier(
    const dstar_lite_t* dsl);

BYUL_API void     dstar_lite_set_frontier(
    dstar_lite_t* dsl, dstar_lite_pqueue_t* frontier);

BYUL_API float dstar_lite_get_km(const dstar_lite_t* dsl);
BYUL_API void   dstar_lite_set_km(dstar_lite_t* dsl, float km);

BYUL_API int   dstar_lite_get_max_range(const dstar_lite_t* dsl);
BYUL_API void   dstar_lite_set_max_range(dstar_lite_t* dsl, int value);

// Maximum number of loops inside the find_loop function
// When moving in real-time at 4 km/h, it loops continuously
// for the time equal to interval_sec multiplied by this value.
// This needs improvement; for a 10x10 map, around 100 might be needed.
BYUL_API int   dstar_lite_get_real_loop_max_retry(const dstar_lite_t* dsl);
BYUL_API void  dstar_lite_set_real_loop_max_retry(
    dstar_lite_t* dsl, int value);
BYUL_API int   dstar_lite_real_loop_retry_count(const dstar_lite_t* dsl);

// On a 10x10 map, 100 seems to work well.
BYUL_API int dstar_lite_get_max_retry(const dstar_lite_t* dsl);
BYUL_API void dstar_lite_set_max_retry(
    dstar_lite_t* dsl, int v);

BYUL_API int dstar_lite_proto_compute_retry_count(const dstar_lite_t* dsl);

BYUL_API int dstar_lite_real_compute_retry_count(const dstar_lite_t* dsl);

// When creating proto route_t*, reconstruct_route is used.
// For a 10x10 map, 100 is too much and 10 is too small;
// around 40 seems reasonable.
BYUL_API int dstar_lite_get_reconstruct_max_retry(const dstar_lite_t* dsl);
BYUL_API void dstar_lite_set_reconstruct_max_retry(dstar_lite_t* dsl, int v);
BYUL_API int dstar_lite_reconstruct_retry_count(const dstar_lite_t* dsl);

BYUL_API bool dstar_lite_is_debug_mode_enabled(const dstar_lite_t* dsl);

BYUL_API void     dstar_lite_enable_debug_mode(
    dstar_lite_t* dsl, bool enabled);

BYUL_API const navgrid_t*    dstar_lite_get_navgrid(const dstar_lite_t* dsl);
BYUL_API void    dstar_lite_set_navgrid(dstar_lite_t* dsl, navgrid_t* navgrid);

BYUL_API const route_t* dstar_lite_get_proto_route(const dstar_lite_t* dsl);

BYUL_API const route_t* dstar_lite_get_real_route(const dstar_lite_t* dsl);

// Resets and initializes settings such as start, goal, and map,
// as well as hash tables and the priority queue.
BYUL_API void dstar_lite_reset(dstar_lite_t* dsl);

BYUL_API float dstar_lite_get_interval_sec(dstar_lite_t* dsl);

BYUL_API void dstar_lite_set_interval_sec(
    dstar_lite_t* dsl, float interval_sec);

BYUL_API cost_func    dstar_lite_get_cost_func(const dstar_lite_t* dsl);
BYUL_API void dstar_lite_set_cost_func(dstar_lite_t* dsl, cost_func fn);

BYUL_API void*    dstar_lite_get_cost_func_userdata(const dstar_lite_t* dsl);

BYUL_API void dstar_lite_set_cost_func_userdata(
    dstar_lite_t* dsl, void* userdata);    

BYUL_API heuristic_func dstar_lite_get_heuristic_func(
    const dstar_lite_t* dsl);
BYUL_API void         dstar_lite_set_heuristic_func(
    dstar_lite_t* dsl, heuristic_func func);
BYUL_API void* dstar_lite_get_heuristic_func_userdata(dstar_lite_t* dsl);
BYUL_API void dstar_lite_set_heuristic_func_userdata(
    dstar_lite_t* dsl, void* userdata);    

BYUL_API move_func dstar_lite_get_move_func(const dstar_lite_t* dsl);
BYUL_API void dstar_lite_set_move_func(dstar_lite_t* dsl, move_func fn);
BYUL_API void* dstar_lite_get_move_func_userdata(const dstar_lite_t* dsl);
BYUL_API void dstar_lite_set_move_func_userdata(
    dstar_lite_t* dsl, void* userdata);

BYUL_API changed_coords_func dstar_lite_get_changed_coords_func(
    const dstar_lite_t* dsl);
BYUL_API void dstar_lite_set_changed_coords_func(
    dstar_lite_t* dsl, changed_coords_func fn);
BYUL_API void* dstar_lite_get_changed_coords_func_userdata(
    const dstar_lite_t* dsl);
BYUL_API void dstar_lite_set_changed_coords_func_userdata(
    dstar_lite_t* dsl, void* userdata);

/**
 * @brief Priority key calculation function for D* Lite
 *
 * Sets k2 to the smaller value between g[s] and rhs[s],
 * and calculates k1 = k2 + heuristic(start, s) + km.
 *
 * @param dsl D* Lite object
 * @param s   Target coordinate
 * @return Calculated dstar_lite_key_t structure
 */
BYUL_API dstar_lite_key_t* dstar_lite_calc_key(
    dstar_lite_t* dsl, const coord_t* s);

BYUL_API void dstar_lite_init(dstar_lite_t* dsl);

/**
 * @brief Recalculates the rhs value of the given node 
 *      and updates the open list if necessary
 * @param al Algorithm context
 * @param u  Coordinate to update
 */
BYUL_API void dstar_lite_update_vertex(dstar_lite_t* dsl, const coord_t* u);

/**
 * @brief Performs update_vertex() for all coordinates 
 *      within the given range from a center coordinate
 * 
 * @param al         Algorithm context
 * @param s          Center coordinate
 * @param max_range  Range (0 means only the s coordinate is updated)
 */
BYUL_API void dstar_lite_update_vertex_range(dstar_lite_t* dsl, 
    const coord_t* s, int max_range);

/**
 * @brief Executes update_vertex_range() 
 *      based on the max_range defined in the config
 * 
 * @param al Algorithm context
 * @param s  Center coordinate
 */
BYUL_API void dstar_lite_update_vertex_auto_range(
    dstar_lite_t* dsl, const coord_t* s);

/**
 * @brief Computes the shortest route based on the open list
 * 
 * @param al Algorithm context
 */    
BYUL_API void dstar_lite_compute_shortest_route(dstar_lite_t* dsl);

/**
 * @brief Reconstructs the route between two coordinates.
 *
 * When the condition g ~= rhs is satisfied, 
 * the actual route is extracted and returned.
 * If the condition is not satisfied, NULL is returned.
 *
 * @param dsl Algorithm context
 */
BYUL_API bool dstar_lite_reconstruct_route(dstar_lite_t* dsl);

// One-time pathfinding in its simplest form, 
// equivalent to static pathfinding.
BYUL_API route_t* dstar_lite_find(dstar_lite_t* dsl);

// Integrated pathfinding, combining find_proto and find_loop.
BYUL_API void dstar_lite_find_full(dstar_lite_t* dsl);

// Generates an initial route for dynamic pathfinding.
BYUL_API bool dstar_lite_find_proto(dstar_lite_t* dsl);

// Using the initial route created by dstar_lite_find_proto, 
// this function searches for a dynamic route.
// dstar_lite_find_proto must be executed first.
// Otherwise, pathfinding will fail.
// If asked why find_proto must be called separately, 
// the answer is that this is for advanced users.
// Some users may want to perform certain operations 
// between find_proto and find_loop.
// If asked why callbacks are not used, 
// the answer is that refactoring for that is undesirable for now.
// Callback support may be added later.
// If you simply want a dynamic route, use dstar_lite_find_full().
BYUL_API void dstar_lite_find_loop(dstar_lite_t* dsl);

/**
 * @brief Executes update_vertex for all coordinates 
 * included in the given route.
 *
 * @param al Algorithm context
 * @param p  Route to update
 */
BYUL_API void dstar_lite_update_vertex_by_route(
    dstar_lite_t* dsl, route_t* p);

// Forcefully terminates the loop.
BYUL_API void dstar_lite_force_quit(dstar_lite_t* dsl);

// Checks if a forced termination has been requested.
BYUL_API bool dstar_lite_is_quit_forced(dstar_lite_t* dsl);

BYUL_API void dstar_lite_set_force_quit(dstar_lite_t* dsl, bool v);

BYUL_API bool dstar_lite_fetch_next(
    const dstar_lite_t* dsl, const coord_t* start, coord_t* out);

#ifdef __cplusplus
}
#endif

#endif // DSTAR_LITE_H
