#ifndef ROUTE_FINDER_H
#define ROUTE_FINDER_H

#include <stdint.h>

#include "route_finder_core.h"
#include "navsys_status.h"

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

/**
 * @brief Fringe Search algorithm configuration.
 *
 * @byul.storage basic-value
 * @byul.zero_valid false
 * @byul.copy_semantics trivial-copy
 * @byul.thread_safety thread-compatible
 */
typedef struct s_route_finder_fringe_search_config {
    float delta_epsilon; /**< Accepted range: [0.001, 5.0]. */
} route_finder_fringe_search_config_t;

/**
 * @brief Real-Time A* algorithm configuration.
 *
 * @byul.storage basic-value
 * @byul.zero_valid false
 * @byul.copy_semantics trivial-copy
 * @byul.thread_safety thread-compatible
 */
typedef struct s_route_finder_rta_star_config {
    int depth_limit; /**< Accepted range: [1, 100]. */
} route_finder_rta_star_config_t;

/**
 * @brief Simplified Memory-Bounded A* algorithm configuration.
 *
 * @byul.storage basic-value
 * @byul.zero_valid false
 * @byul.copy_semantics trivial-copy
 * @byul.thread_safety thread-compatible
 */
typedef struct s_route_finder_sma_star_config {
    int memory_limit; /**< Accepted range: [10, 1000000]. */
} route_finder_sma_star_config_t;

/**
 * @brief Weighted A* algorithm configuration.
 *
 * @byul.storage basic-value
 * @byul.zero_valid false
 * @byul.copy_semantics trivial-copy
 * @byul.thread_safety thread-compatible
 */
typedef struct s_route_finder_weighted_astar_config {
    float weight; /**< Accepted range: [0.1, 10.0]. */
} route_finder_weighted_astar_config_t;

/**
 * @brief Reports observable results from one route finder execution.
 *
 * @byul.storage basic-value
 * @byul.zero_valid true
 * @byul.copy_semantics trivial-copy
 * @byul.thread_safety thread-compatible
 */
typedef struct s_route_finder_run_stats {
    int total_retry_count; /**< Algorithm-reported expansion/retry count. */
    int route_length; /**< Number of coordinates in the returned route. */
    float route_cost; /**< Algorithm-reported route cost. */
    bool complete; /**< True only when the goal was reached. */
    bool partial; /**< True when a non-empty route did not reach the goal. */
} route_finder_run_stats_t;

/**
 * @brief Requests cooperative cancellation for one route finder execution.
 *
 * The callback runs synchronously on the thread executing the search. Returning
 * true requests cancellation; returning false allows the search to continue.
 */
typedef bool (*route_finder_cancel_func)(void* userdata);

/**
 * @brief Supplies call-scoped controls for one route finder execution.
 *
 * Set struct_size to sizeof(route_finder_run_options_t). The callback and its
 * userdata are borrowed only for the duration of route_finder_run_with_options.
 *
 * @byul.storage basic-value
 * @byul.zero_valid false
 * @byul.copy_semantics trivial-copy
 * @byul.thread_safety thread-compatible
 */
typedef struct s_route_finder_run_options {
    uint32_t struct_size;
    route_finder_cancel_func cancel_func;
    void* cancel_userdata;
} route_finder_run_options_t;

BYUL_API const char* get_route_finder_name(route_finder_type_t pa);

/**
 * @brief 지정한 route finder type을 공통 dispatcher가 실행하는지 확인한다.
 *
 * Enum에 이름이 존재하더라도 현재 build의 공통 dispatcher에 구현이 연결되지 않은
 * type이면 false를 반환한다.
 *
 * @param[in] type 확인할 route finder type.
 * @return 공통 dispatcher가 지원하면 true, 아니면 false.
 * @byul.side_effect none
 * @byul.thread_safety thread-safe
 * @byul.blocking false
 * @byul.reentrant true
 */
BYUL_API bool route_finder_is_supported(route_finder_type_t type);

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

/**
 * @brief Selects a dispatcher-supported algorithm without partial mutation.
 * @param[in,out] finder Route finder to update.
 * @param[in] type Algorithm type to select.
 * @return Common Navsys status value.
 * @retval NAVSYS_STATUS_OK The type was selected.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT finder is NULL.
 * @retval NAVSYS_STATUS_UNSUPPORTED type has no dispatcher implementation.
 * @retval NAVSYS_STATUS_IN_PROGRESS A callback on the same finder is active.
 * @byul.nullable finder false
 * @byul.side_effect mutates:finder
 * @byul.thread_safety externally-synchronized
 * @byul.blocking false
 * @byul.reentrant false
 */
BYUL_API navsys_status_t route_finder_set_type_checked(
    route_finder_t* finder, route_finder_type_t type);

BYUL_API void route_finder_set_typedata(
    route_finder_t* a, void* typedata);

BYUL_API void* route_finder_get_typedata(const route_finder_t* a);    

/**
 * @brief Binds a retained Fringe Search configuration and selects that algorithm.
 * @param[in,out] finder Route finder to update.
 * @param[in] config Caller-owned configuration retained by the finder.
 * @return Common Navsys status value.
 * @retval NAVSYS_STATUS_OK The configuration was bound.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT An argument or value is invalid.
 * @retval NAVSYS_STATUS_IN_PROGRESS A callback on the same finder is active.
 * @byul.nullable finder false
 * @byul.nullable config false
 * @byul.lifetime config until-unbind
 * @byul.side_effect mutates:finder
 * @byul.thread_safety externally-synchronized
 * @byul.blocking false
 * @byul.reentrant false
 */
BYUL_API navsys_status_t route_finder_bind_fringe_search_config(
    route_finder_t* finder,
    const route_finder_fringe_search_config_t* config);

/**
 * @brief Binds a retained Real-Time A* configuration and selects that algorithm.
 * @param[in,out] finder Route finder to update.
 * @param[in] config Caller-owned configuration retained by the finder.
 * @return Common Navsys status value.
 * @retval NAVSYS_STATUS_OK The configuration was bound.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT An argument or value is invalid.
 * @retval NAVSYS_STATUS_IN_PROGRESS A callback on the same finder is active.
 * @byul.nullable finder false
 * @byul.nullable config false
 * @byul.lifetime config until-unbind
 * @byul.side_effect mutates:finder
 * @byul.thread_safety externally-synchronized
 * @byul.blocking false
 * @byul.reentrant false
 */
BYUL_API navsys_status_t route_finder_bind_rta_star_config(
    route_finder_t* finder,
    const route_finder_rta_star_config_t* config);

/**
 * @brief Binds a retained SMA* configuration and selects that algorithm.
 * @param[in,out] finder Route finder to update.
 * @param[in] config Caller-owned configuration retained by the finder.
 * @return Common Navsys status value.
 * @retval NAVSYS_STATUS_OK The configuration was bound.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT An argument or value is invalid.
 * @retval NAVSYS_STATUS_IN_PROGRESS A callback on the same finder is active.
 * @byul.nullable finder false
 * @byul.nullable config false
 * @byul.lifetime config until-unbind
 * @byul.side_effect mutates:finder
 * @byul.thread_safety externally-synchronized
 * @byul.blocking false
 * @byul.reentrant false
 */
BYUL_API navsys_status_t route_finder_bind_sma_star_config(
    route_finder_t* finder,
    const route_finder_sma_star_config_t* config);

/**
 * @brief Binds a retained Weighted A* configuration and selects that algorithm.
 * @param[in,out] finder Route finder to update.
 * @param[in] config Caller-owned configuration retained by the finder.
 * @return Common Navsys status value.
 * @retval NAVSYS_STATUS_OK The configuration was bound.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT An argument or value is invalid.
 * @retval NAVSYS_STATUS_IN_PROGRESS A callback on the same finder is active.
 * @byul.nullable finder false
 * @byul.nullable config false
 * @byul.lifetime config until-unbind
 * @byul.side_effect mutates:finder
 * @byul.thread_safety externally-synchronized
 * @byul.blocking false
 * @byul.reentrant false
 */
BYUL_API navsys_status_t route_finder_bind_weighted_astar_config(
    route_finder_t* finder,
    const route_finder_weighted_astar_config_t* config);

/**
 * @brief Clears the retained algorithm configuration without changing the type.
 * @param[in,out] finder Route finder to update.
 * @return Common Navsys status value.
 * @retval NAVSYS_STATUS_OK The retained configuration was cleared.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT finder is NULL.
 * @retval NAVSYS_STATUS_IN_PROGRESS A callback on the same finder is active.
 * @byul.nullable finder false
 * @byul.side_effect mutates:finder
 * @byul.thread_safety externally-synchronized
 * @byul.blocking false
 * @byul.reentrant false
 */
BYUL_API navsys_status_t route_finder_unbind_algorithm_config(
    route_finder_t* finder);

BYUL_API void route_finder_set_max_retry(route_finder_t* a, int max_retry);
BYUL_API int route_finder_get_max_retry(route_finder_t* a);

/**
 * @brief Sets a positive deterministic expansion/retry limit.
 * @param[in,out] finder Route finder to update.
 * @param[in] max_retry Positive algorithm expansion/retry limit.
 * @return Common Navsys status value.
 * @retval NAVSYS_STATUS_OK The limit was updated.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT finder is NULL or max_retry is not positive.
 * @retval NAVSYS_STATUS_IN_PROGRESS A callback on the same finder is active.
 * @byul.nullable finder false
 * @byul.side_effect mutates:finder
 * @byul.thread_safety externally-synchronized
 * @byul.blocking false
 * @byul.reentrant false
 */
BYUL_API navsys_status_t route_finder_set_max_retry_checked(
    route_finder_t* finder, int max_retry);

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
 * @brief Cost callback과 userdata를 하나의 binding으로 교체한다.
 * @param[in,out] finder 변경할 route finder.
 * @param[in] fn bind할 cost callback.
 * @param[in] userdata callback에 전달할 caller 소유 data.
 * @return 공통 Navsys 상태 값.
 * @retval NAVSYS_STATUS_OK binding이 교체됐다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT finder 또는 fn이 NULL이다.
 * @retval NAVSYS_STATUS_IN_PROGRESS 같은 finder의 callback 실행 중이다.
 * @byul.nullable finder false
 * @byul.nullable fn false
 * @byul.nullable userdata true
 * @byul.lifetime fn until-unbind
 * @byul.lifetime userdata until-unbind
 * @byul.side_effect mutates:finder
 * @byul.thread_safety externally-synchronized
 * @byul.blocking false
 * @byul.reentrant false
 */
BYUL_API navsys_status_t route_finder_bind_cost_func(
    route_finder_t* finder, cost_func fn, void* userdata);

/**
 * @brief Cost binding을 기본 callback과 NULL userdata로 되돌린다.
 * @param[in,out] finder 변경할 route finder.
 * @return 공통 Navsys 상태 값.
 * @retval NAVSYS_STATUS_OK 기본 binding으로 변경됐다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT finder가 NULL이다.
 * @retval NAVSYS_STATUS_IN_PROGRESS 같은 finder의 callback 실행 중이다.
 * @byul.nullable finder false
 * @byul.side_effect mutates:finder
 * @byul.thread_safety externally-synchronized
 * @byul.blocking false
 * @byul.reentrant false
 */
BYUL_API navsys_status_t route_finder_unbind_cost_func(
    route_finder_t* finder);

/**
 * @brief Heuristic callback과 userdata를 하나의 binding으로 교체한다.
 * @param[in,out] finder 변경할 route finder.
 * @param[in] fn bind할 heuristic callback.
 * @param[in] userdata callback에 전달할 caller 소유 data.
 * @return 공통 Navsys 상태 값.
 * @retval NAVSYS_STATUS_OK binding이 교체됐다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT finder 또는 fn이 NULL이다.
 * @retval NAVSYS_STATUS_IN_PROGRESS 같은 finder의 callback 실행 중이다.
 * @byul.nullable finder false
 * @byul.nullable fn false
 * @byul.nullable userdata true
 * @byul.lifetime fn until-unbind
 * @byul.lifetime userdata until-unbind
 * @byul.side_effect mutates:finder
 * @byul.thread_safety externally-synchronized
 * @byul.blocking false
 * @byul.reentrant false
 */
BYUL_API navsys_status_t route_finder_bind_heuristic_func(
    route_finder_t* finder, heuristic_func fn, void* userdata);

/**
 * @brief Heuristic binding을 기본 callback과 NULL userdata로 되돌린다.
 * @param[in,out] finder 변경할 route finder.
 * @return 공통 Navsys 상태 값.
 * @retval NAVSYS_STATUS_OK 기본 binding으로 변경됐다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT finder가 NULL이다.
 * @retval NAVSYS_STATUS_IN_PROGRESS 같은 finder의 callback 실행 중이다.
 * @byul.nullable finder false
 * @byul.side_effect mutates:finder
 * @byul.thread_safety externally-synchronized
 * @byul.blocking false
 * @byul.reentrant false
 */
BYUL_API navsys_status_t route_finder_unbind_heuristic_func(
    route_finder_t* finder);

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

/**
 * @brief Runs the selected algorithm and separates normal termination causes.
 *
 * On OK, NO_PATH, or LIMIT_REACHED, this function stores a caller-owned route
 * in out_route and fills out_stats. The caller destroys that route with
 * route_destroy. On every other status, both outputs remain unchanged.
 *
 * @param[in,out] finder Configured route finder.
 * @param[out] out_route Receives the owned route for normal termination.
 * @param[out] out_stats Receives execution statistics for normal termination.
 * @return Common Navsys status value.
 * @retval NAVSYS_STATUS_OK The goal was reached.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT A pointer or finder configuration is invalid.
 * @retval NAVSYS_STATUS_UNSUPPORTED The selected algorithm is not implemented.
 * @retval NAVSYS_STATUS_OUT_OF_MEMORY The algorithm could not create a result.
 * @retval NAVSYS_STATUS_NO_PATH Search terminated without reaching the goal.
 * @retval NAVSYS_STATUS_LIMIT_REACHED The configured retry limit was reached.
 * @retval NAVSYS_STATUS_IN_PROGRESS A callback on the same finder is active.
 * @byul.nullable finder false
 * @byul.nullable out_route false
 * @byul.nullable out_stats false
 * @byul.lifetime out_route caller-owned
 * @byul.side_effect mutates:finder,allocates:out_route,invokes-callbacks
 * @byul.thread_safety externally-synchronized
 * @byul.blocking true
 * @byul.reentrant false
 */
BYUL_API navsys_status_t route_finder_run_ex(
    route_finder_t* finder,
    route_t** out_route,
    route_finder_run_stats_t* out_stats);

/**
 * @brief Runs the selected algorithm with call-scoped cancellation controls.
 *
 * The algorithm polls cancel_func before each expansion and during unbounded
 * reconstruction loops. If cancellation is observed after a route result is
 * created, CANCELLED returns that caller-owned partial route and its statistics.
 * The caller destroys it with route_destroy. INVALID_ARGUMENT, UNSUPPORTED,
 * OUT_OF_MEMORY, CALLBACK_FAILED, and IN_PROGRESS preserve both outputs.
 *
 * @param[in,out] finder Configured route finder.
 * @param[in] options Optional call-scoped controls, or NULL for no cancellation.
 * @param[out] out_route Receives an owned route on normal or cancelled termination.
 * @param[out] out_stats Receives statistics on normal or cancelled termination.
 * @return Common Navsys status value.
 * @retval NAVSYS_STATUS_OK The goal was reached.
 * @retval NAVSYS_STATUS_CANCELLED cancel_func requested cancellation.
 * @retval NAVSYS_STATUS_CALLBACK_FAILED cancel_func raised a C++ exception.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT An argument or struct_size is invalid.
 * @retval NAVSYS_STATUS_UNSUPPORTED The selected algorithm is not implemented.
 * @retval NAVSYS_STATUS_OUT_OF_MEMORY The algorithm could not create a result.
 * @retval NAVSYS_STATUS_NO_PATH Search terminated without reaching the goal.
 * @retval NAVSYS_STATUS_LIMIT_REACHED The configured retry limit was reached.
 * @retval NAVSYS_STATUS_IN_PROGRESS A callback on the same finder is active.
 * @byul.nullable finder false
 * @byul.nullable options true
 * @byul.nullable out_route false
 * @byul.nullable out_stats false
 * @byul.lifetime options call-only
 * @byul.lifetime out_route caller-owned
 * @byul.side_effect mutates:finder,allocates:out_route,invokes-callbacks
 * @byul.thread_safety externally-synchronized
 * @byul.blocking true
 * @byul.reentrant false
 */
BYUL_API navsys_status_t route_finder_run_with_options(
    route_finder_t* finder,
    const route_finder_run_options_t* options,
    route_t** out_route,
    route_finder_run_stats_t* out_stats);


#ifdef __cplusplus
}
#endif

#endif // ROUTE_FINDER_H
