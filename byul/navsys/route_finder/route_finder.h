#ifndef ROUTE_FINDER_H
#define ROUTE_FINDER_H

#include "route_finder_common.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MAX_RETRY 1000

typedef enum e_route_finder_type{
    ROUTE_FINDER_UNKNOWN = 0,

    // 1950s~1960s
    ROUTE_FINDER_BELLMAN_FORD,            // 1958
    ROUTE_FINDER_DFS,                     // 1959
    ROUTE_FINDER_BFS,                     // 1959
    ROUTE_FINDER_DIJKSTRA,                // 1959
    ROUTE_FINDER_FLOYD_WARSHALL,          // 1959~
    ROUTE_FINDER_ASTAR,                   // 1968

    // 1970s
    ROUTE_FINDER_BIDIRECTIONAL_DIJKSTRA,  // 1971
    ROUTE_FINDER_BIDIRECTIONAL_ASTAR,     // 1971
    ROUTE_FINDER_WEIGHTED_ASTAR,          // 1977~
    ROUTE_FINDER_JOHNSON,                 // 1977
    ROUTE_FINDER_K_SHORTEST_PATH,         // 1977~
    ROUTE_FINDER_DIAL,                    // 1969

    // 1980s
    ROUTE_FINDER_ITERATIVE_DEEPENING,     // 1980
    ROUTE_FINDER_GREEDY_BEST_FIRST,       // 1985
    ROUTE_FINDER_IDA_STAR,                // 1985

    // 1990s
    ROUTE_FINDER_RTA_STAR,                // 1990
    ROUTE_FINDER_SMA_STAR,                // 1991
    ROUTE_FINDER_DSTAR,                   // 1994
    ROUTE_FINDER_FAST_MARCHING,           // 1996
    ROUTE_FINDER_ANT_COLONY,              // 1996
    ROUTE_FINDER_FRINGE_SEARCH,           // 1997

    // 2000s
    ROUTE_FINDER_FOCAL_SEARCH,            // 2001
    ROUTE_FINDER_DSTAR_LITE,              // 2002
    ROUTE_FINDER_LPA_STAR,                // 2004
    ROUTE_FINDER_HPA_STAR,                // 2004
    ROUTE_FINDER_ALT,                     // 2005
    ROUTE_FINDER_ANY_ANGLE_ASTAR,         // 2005~
    ROUTE_FINDER_HCA_STAR,                // 2005
    ROUTE_FINDER_RTAA_STAR,               // 2006
    ROUTE_FINDER_THETA_STAR,              // 2007
    ROUTE_FINDER_CONTRACTION_HIERARCHIES, // 2008

    // 2010s
    ROUTE_FINDER_LAZY_THETA_STAR,         // 2010
    ROUTE_FINDER_JUMP_POINT_SEARCH,       // 2011
    ROUTE_FINDER_SIPP,                    // 2011
    ROUTE_FINDER_JPS_PLUS,                // 2012
    ROUTE_FINDER_EPEA_STAR,               // 2012
    ROUTE_FINDER_MHA_STAR,                // 2012
    ROUTE_FINDER_ANYA,                    // 2013

    // Special Purpose / Extended
    ROUTE_FINDER_DAG_SP,                  // 1960s (DAG shortest path O(V+E))
    ROUTE_FINDER_MULTI_SOURCE_BFS,        // 2000s (multi-source BFS)
    ROUTE_FINDER_MCTS                     // 2006
} route_finder_type_t;

BYUL_API const char* get_route_finder_name(route_finder_type_t pa);

/** 
 * @brief Static pathfinding configuration structure.
 */
typedef struct s_route_finder {
    navgrid_t* navgrid;                        ///< Map for pathfinding
    coord_t start;                            ///< Start coordinate
    coord_t goal;                             ///< Goal coordinate

    route_finder_type_t type;
    void* typedata;

    int max_retry;                             ///< Maximum iterations
    bool debug_mode_enabled;                      ///< Log visited nodes    

    cost_func cost_fn;                         ///< Cost function
    void* cost_fn_userdata;

    heuristic_func heuristic_fn;               ///< Heuristic function
    void* heuristic_fn_userdata;
} route_finder_t;

/**
 * @brief Creates a route_finder_t structure with default settings.
 *
 * This function uses ROUTE_FINDER_ASTAR as the default pathfinding algorithm
 * and initializes the route_finder_t object with the following defaults:
 * - cost function: default_cost
 * - heuristic function: euclidean_heuristic
 * - max_retry: MAX_RETRY
 * - debug_mode_enabled: false
 * - typedata: nullptr by default, used for algorithm-specific data
 *
 * @param navgrid The navigation grid to use
 * @return A pointer to the initialized route_finder_t (allocated on heap).
 *         Must be freed using route_finder_destroy.
 */
BYUL_API route_finder_t* route_finder_create(navgrid_t* navgrid);

BYUL_API route_finder_t* route_finder_create_full(
    navgrid_t* navgrid, 
    const coord_t* start, 
    const coord_t* goal,
    
    route_finder_type_t type,
    void* typedata,

    int max_retry, 
    bool debug_mode_enabled,

    cost_func cost_fn,
    void* cost_fn_userdata,

    heuristic_func heuristic_fn,
    void* heuristic_fn_userdata
);

BYUL_API int route_finder_init(route_finder_t* out, navgrid_t* navgrid);

BYUL_API int route_finder_init_full(
    route_finder_t* out, 
    navgrid_t* navgrid, 
    const coord_t* start, 
    const coord_t* goal,
    route_finder_type_t type, 
    void* typedata,    
    int max_retry, 
    bool debug_mode_enabled,
    
    cost_func cost_fn,
    void* cost_fn_userdata,

    heuristic_func heuristic_fn,
    void* heuristic_fn_userdata    
);

BYUL_API int route_finder_free(route_finder_t* out);

BYUL_API int route_finder_destroy(route_finder_t* a);

BYUL_API route_finder_t* route_finder_copy(const route_finder_t* src);

/**
 * @brief Getters/Setters for route_finder_t configuration.
 */
BYUL_API void route_finder_set_navgrid(route_finder_t* a, navgrid_t* navgrid);
BYUL_API void route_finder_set_start(route_finder_t* a, const coord_t* start);
BYUL_API void route_finder_set_goal(route_finder_t* a, const coord_t* goal);

BYUL_API const navgrid_t* route_finder_get_navgrid(const route_finder_t* a);
BYUL_API int route_finder_fetch_start(const route_finder_t* a, coord_t* out);
BYUL_API int route_finder_fetch_goal(const route_finder_t* a, coord_t* out);

BYUL_API void route_finder_set_type(
    route_finder_t* a, route_finder_type_t type);

BYUL_API route_finder_type_t route_finder_get_type(const route_finder_t* a);

BYUL_API void route_finder_set_typedata(
    route_finder_t* a, void* typedata);

BYUL_API void* route_finder_get_typedata(const route_finder_t* a);    

BYUL_API void route_finder_set_max_retry(route_finder_t* a, int max_retry);
BYUL_API int route_finder_get_max_retry(route_finder_t* a);

BYUL_API void route_finder_enable_debug_mode(
    route_finder_t* a, bool is_logging);

BYUL_API bool route_finder_is_debug_mode_enabled(route_finder_t* a);

BYUL_API void route_finder_set_cost_func(
    route_finder_t* a, cost_func cost_fn);

BYUL_API cost_func route_finder_get_cost_func(route_finder_t* a);

BYUL_API void route_finder_set_cost_fn_userdata(
    route_finder_t* a, void* cost_fn_userdata);

BYUL_API void* route_finder_get_cost_fn_userdata(const route_finder_t* a);

BYUL_API void route_finder_set_heuristic_func(
    route_finder_t* a, heuristic_func heuristic_fn);

BYUL_API heuristic_func route_finder_get_heuristic_func(route_finder_t* a);

BYUL_API void route_finder_set_heuristic_fn_userdata(
    route_finder_t* a, void* heuristic_fn_userdata);

BYUL_API void* route_finder_get_heuristic_fn_userdata(
    const route_finder_t* a);

/**
 * @brief Resets and validates the configuration.
 */
BYUL_API void route_finder_clear(route_finder_t* a);

/**
 * @brief Sets the default values for a route_finder_t structure.
 *
 * - cost function: default_cost
 * - heuristic function: euclidean_heuristic
 * - max_retry: 10000
 * - debug_mode_enabled: false
 *
 * @param a Pointer to the route_finder_t to initialize.
 */
BYUL_API void route_finder_set_defaults(route_finder_t* a);

BYUL_API bool route_finder_is_valid(const route_finder_t* a);
BYUL_API void route_finder_print(const route_finder_t* a);

/**
 * @brief Direct run functions for specific algorithms.
 */
BYUL_API route_t* route_finder_run(route_finder_t* a);


#ifdef __cplusplus
}
#endif

#endif // ROUTE_FINDER_H
