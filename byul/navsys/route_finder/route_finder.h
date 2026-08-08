/*
 * Copyright (c) 2025-2026 ByulPapa (byuldev@outlook.kr)
 * This file is part of the Byul World project.
 * Licensed under the Byul World Source-Available Non-Commercial License v1.0 (2025).
 * See the LICENSE file in the project root for full license terms.
 */

/**
 * @file route_finder.h
 * @brief Declares the Route Finder dispatcher and configuration public C ABI.
 */

#ifndef BYUL_ROUTE_FINDER_H
#define BYUL_ROUTE_FINDER_H

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
 * @brief Returns the stable name assigned to every route finder type value.
 * @param[in] type Route finder type.
 * @return Static UTF-8 name; "unknown" for values outside the enum.
 * @byul.nullable return false
 * @byul.lifetime return borrowed:static-storage
 * @byul.side_effect none
 * @byul.thread_safety thread-safe
 * @byul.blocking false
 * @byul.reentrant true
 */
BYUL_API const char* route_finder_type_get_name(route_finder_type_t type);

/**
 * @brief Reports whether the canonical dispatcher supports a type.
 * @param[in] type Route finder type.
 * @return true when route_finder_run_ex can dispatch the type.
 * @byul.side_effect none
 * @byul.thread_safety thread-safe
 * @byul.blocking false
 * @byul.reentrant true
 */
BYUL_API bool route_finder_is_type_supported(route_finder_type_t type);

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
 * @param[in] navgrid The navigation grid to use.
 * @return A pointer to the initialized route_finder_t (allocated on heap).
 *         Must be freed using route_finder_destroy.
 * @byul.nullable navgrid false
 * @byul.nullable return true
 * @byul.lifetime return caller-owned
 */
BYUL_API route_finder_t* route_finder_create(navgrid_t* navgrid);

/**
 * @brief Creates a fully configured legacy route finder.
 * @param[in] navgrid Borrowed navigation grid.
 * @param[in] start Start coordinate.
 * @param[in] goal Goal coordinate.
 * @param[in] type Algorithm type.
 * @param[in] typedata Borrowed legacy algorithm data.
 * @param[in] max_retry Positive expansion limit.
 * @param[in] debug_mode_enabled Whether tracing is enabled.
 * @param[in] cost_fn Legacy cost callback.
 * @param[in] cost_fn_userdata Borrowed cost callback data.
 * @param[in] heuristic_fn Legacy heuristic callback.
 * @param[in] heuristic_fn_userdata Borrowed heuristic callback data.
 * @return Caller-owned finder, or NULL on invalid input or allocation failure.
 * @byul.nullable navgrid false
 * @byul.nullable start false
 * @byul.nullable goal false
 * @byul.nullable typedata true
 * @byul.nullable cost_fn false
 * @byul.nullable cost_fn_userdata true
 * @byul.nullable heuristic_fn false
 * @byul.nullable heuristic_fn_userdata true
 * @byul.nullable return true
 * @byul.lifetime return caller-owned
 */
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

/**
 * @brief Initializes caller storage as a minimal legacy finder.
 * @param[out] out Caller storage.
 * @param[in] navgrid Borrowed navigation grid.
 * @return Zero on success, otherwise -1.
 * @byul.nullable out false
 * @byul.nullable navgrid false
 */
BYUL_API int route_finder_init(route_finder_t* out, navgrid_t* navgrid);

/**
 * @brief Initializes caller storage from all legacy configuration fields.
 * @param[out] out Caller storage.
 * @param[in] navgrid Borrowed navigation grid.
 * @param[in] start Start coordinate.
 * @param[in] goal Goal coordinate.
 * @param[in] type Algorithm type.
 * @param[in] typedata Borrowed legacy algorithm data.
 * @param[in] max_retry Positive expansion limit.
 * @param[in] debug_mode_enabled Whether tracing is enabled.
 * @param[in] cost_fn Legacy cost callback.
 * @param[in] cost_fn_userdata Borrowed cost callback data.
 * @param[in] heuristic_fn Legacy heuristic callback.
 * @param[in] heuristic_fn_userdata Borrowed heuristic callback data.
 * @return Zero on success, otherwise -1.
 * @byul.nullable out false
 * @byul.nullable navgrid false
 * @byul.nullable start false
 * @byul.nullable goal false
 * @byul.nullable typedata true
 * @byul.nullable cost_fn false
 * @byul.nullable cost_fn_userdata true
 * @byul.nullable heuristic_fn false
 * @byul.nullable heuristic_fn_userdata true
 */
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

/**
 * @brief Releases sidecar state and clears caller-owned finder storage.
 * @param[in,out] out Finder storage.
 * @return Zero on success, otherwise -1.
 * @byul.nullable out false
 */
BYUL_API int route_finder_free(route_finder_t* out);

/**
 * @brief Destroys a heap-allocated route finder.
 * @param[in] a Finder to destroy.
 * @return Zero on success, otherwise -1.
 * @byul.nullable a false
 */
BYUL_API int route_finder_destroy(route_finder_t* a);

/**
 * @brief Copies a finder and its retained evaluation binding.
 * @param[in] src Finder to copy.
 * @return Caller-owned copy, or NULL on failure.
 * @byul.nullable src false
 * @byul.nullable return true
 * @byul.lifetime return caller-owned
 */
BYUL_API route_finder_t* route_finder_copy(const route_finder_t* src);

/**
 * @brief Sets the borrowed navigation grid.
 * @param[in,out] a Finder to update.
 * @param[in] navgrid Grid retained as a borrowed pointer.
 * @byul.nullable a false
 * @byul.nullable navgrid false
 */
BYUL_API void route_finder_set_navgrid(route_finder_t* a, navgrid_t* navgrid);

/**
 * @brief Sets the start coordinate.
 * @param[in,out] a Finder to update.
 * @param[in] start Coordinate to copy.
 * @byul.nullable a false
 * @byul.nullable start false
 */
BYUL_API void route_finder_set_start(route_finder_t* a, const coord_t* start);

/**
 * @brief Sets the goal coordinate.
 * @param[in,out] a Finder to update.
 * @param[in] goal Coordinate to copy.
 * @byul.nullable a false
 * @byul.nullable goal false
 */
BYUL_API void route_finder_set_goal(route_finder_t* a, const coord_t* goal);

/**
 * @brief Returns the borrowed navigation grid.
 * @param[in] a Finder to inspect.
 * @return Borrowed grid, or NULL when none is configured.
 * @byul.nullable a false
 * @byul.nullable return true
 * @byul.lifetime return borrowed:a
 */
BYUL_API const navgrid_t* route_finder_get_navgrid(const route_finder_t* a);

/**
 * @brief Copies the configured start coordinate.
 * @param[in] a Finder to inspect.
 * @param[out] out Destination coordinate.
 * @return Zero on success, otherwise -1.
 * @byul.nullable a false
 * @byul.nullable out false
 */
BYUL_API int route_finder_fetch_start(const route_finder_t* a, coord_t* out);

/**
 * @brief Copies the configured goal coordinate.
 * @param[in] a Finder to inspect.
 * @param[out] out Destination coordinate.
 * @return Zero on success, otherwise -1.
 * @byul.nullable a false
 * @byul.nullable out false
 */
BYUL_API int route_finder_fetch_goal(const route_finder_t* a, coord_t* out);

/**
 * @brief Sets a legacy algorithm type without checking capability.
 * @param[in,out] a Finder to update.
 * @param[in] type Algorithm type.
 * @byul.nullable a false
 */
BYUL_API void route_finder_set_type(
    route_finder_t* a, route_finder_type_t type);

/**
 * @brief Returns the selected algorithm type.
 * @param[in] a Finder to inspect.
 * @return Selected algorithm type.
 * @byul.nullable a false
 */
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

/**
 * @brief Sets borrowed legacy algorithm data.
 * @param[in,out] a Finder to update.
 * @param[in] typedata Borrowed algorithm data, or NULL.
 * @byul.nullable a false
 * @byul.nullable typedata true
 */
BYUL_API void route_finder_set_typedata(
    route_finder_t* a, void* typedata);

/**
 * @brief Returns borrowed legacy algorithm data.
 * @param[in] a Finder to inspect.
 * @return Borrowed algorithm data, or NULL.
 * @byul.nullable a false
 * @byul.nullable return true
 * @byul.lifetime return borrowed:a
 */
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

/**
 * @brief Sets the legacy retry limit without validation.
 * @param[in,out] a Finder to update.
 * @param[in] max_retry Retry limit to store.
 * @byul.nullable a false
 */
BYUL_API void route_finder_set_max_retry(route_finder_t* a, int max_retry);

/**
 * @brief Returns the configured retry limit.
 * @param[in] a Finder to inspect.
 * @return Configured retry limit.
 * @byul.nullable a false
 */
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

/**
 * @brief Enables or disables legacy diagnostic logging.
 * @param[in,out] a Finder to update.
 * @param[in] is_logging Whether logging is enabled.
 * @byul.nullable a false
 */
BYUL_API void route_finder_enable_debug_mode(
    route_finder_t* a, bool is_logging);

/**
 * @brief Reports whether diagnostic logging is enabled.
 * @param[in] a Finder to inspect.
 * @return true when logging is enabled.
 * @byul.nullable a false
 */
BYUL_API bool route_finder_is_debug_mode_enabled(route_finder_t* a);

/**
 * @brief Replaces the legacy cost callback.
 * @param[in,out] a Finder to update.
 * @param[in] cost_fn Cost callback retained by the finder.
 * @byul.nullable a false
 * @byul.nullable cost_fn false
 * @byul.lifetime cost_fn until-unbind
 */
BYUL_API void route_finder_set_cost_func(
    route_finder_t* a, cost_func cost_fn);

/**
 * @brief Returns the retained legacy cost callback.
 * @param[in] a Finder to inspect.
 * @return Retained callback.
 * @byul.nullable a false
 * @byul.nullable return true
 * @byul.lifetime return borrowed:a
 */
BYUL_API cost_func route_finder_get_cost_func(route_finder_t* a);

/**
 * @brief Replaces the borrowed legacy cost callback data.
 * @param[in,out] a Finder to update.
 * @param[in] cost_fn_userdata Caller-owned callback data.
 * @byul.nullable a false
 * @byul.nullable cost_fn_userdata true
 * @byul.lifetime cost_fn_userdata until-unbind
 */
BYUL_API void route_finder_set_cost_fn_userdata(
    route_finder_t* a, void* cost_fn_userdata);

/**
 * @brief Returns the borrowed legacy cost callback data.
 * @param[in] a Finder to inspect.
 * @return Retained callback data.
 * @byul.nullable a false
 * @byul.nullable return true
 * @byul.lifetime return borrowed:a
 */
BYUL_API void* route_finder_get_cost_fn_userdata(const route_finder_t* a);

/**
 * @brief Replaces the legacy heuristic callback.
 * @param[in,out] a Finder to update.
 * @param[in] heuristic_fn Heuristic callback retained by the finder.
 * @byul.nullable a false
 * @byul.nullable heuristic_fn false
 * @byul.lifetime heuristic_fn until-unbind
 */
BYUL_API void route_finder_set_heuristic_func(
    route_finder_t* a, heuristic_func heuristic_fn);

/**
 * @brief Returns the retained legacy heuristic callback.
 * @param[in] a Finder to inspect.
 * @return Retained callback.
 * @byul.nullable a false
 * @byul.nullable return true
 * @byul.lifetime return borrowed:a
 */
BYUL_API heuristic_func route_finder_get_heuristic_func(route_finder_t* a);

/**
 * @brief Replaces the borrowed legacy heuristic callback data.
 * @param[in,out] a Finder to update.
 * @param[in] heuristic_fn_userdata Caller-owned callback data.
 * @byul.nullable a false
 * @byul.nullable heuristic_fn_userdata true
 * @byul.lifetime heuristic_fn_userdata until-unbind
 */
BYUL_API void route_finder_set_heuristic_fn_userdata(
    route_finder_t* a, void* heuristic_fn_userdata);

/**
 * @brief Returns the borrowed legacy heuristic callback data.
 * @param[in] a Finder to inspect.
 * @return Retained callback data.
 * @byul.nullable a false
 * @byul.nullable return true
 * @byul.lifetime return borrowed:a
 */
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
 * @brief Atomically binds a status-returning cost callback and userdata.
 * @param[in,out] finder Route finder to update.
 * @param[in] fn Callback retained until unbound or the finder is released.
 * @param[in] userdata Caller-owned callback data retained as a borrowed pointer.
 * @return Common Navsys status value.
 * @retval NAVSYS_STATUS_OK The binding was replaced.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT finder or fn is NULL.
 * @retval NAVSYS_STATUS_IN_PROGRESS The finder is executing a callback.
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
BYUL_API navsys_status_t route_finder_bind_cost_func_ex(
    route_finder_t* finder, route_finder_cost_func_ex fn, void* userdata);

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
 * @brief Atomically binds a status-returning heuristic callback and userdata.
 * @param[in,out] finder Route finder to update.
 * @param[in] fn Callback retained until unbound or the finder is released.
 * @param[in] userdata Caller-owned callback data retained as a borrowed pointer.
 * @return Common Navsys status value.
 * @retval NAVSYS_STATUS_OK The binding was replaced.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT finder or fn is NULL.
 * @retval NAVSYS_STATUS_IN_PROGRESS The finder is executing a callback.
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
BYUL_API navsys_status_t route_finder_bind_heuristic_func_ex(
    route_finder_t* finder, route_finder_heuristic_func_ex fn,
    void* userdata);

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
 * @brief Resets the finder to its default configuration.
 * @param[in,out] a Finder to reset; NULL is ignored.
 * @byul.nullable a true
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
 * @param[out] a Finder storage to initialize.
 * @byul.nullable a false
 */
BYUL_API void route_finder_set_defaults(route_finder_t* a);

/**
 * @brief Validates the legacy finder fields.
 * @param[in] a Finder to validate.
 * @return true when all required fields and values are valid.
 * @byul.nullable a false
 */
BYUL_API bool route_finder_is_valid(const route_finder_t* a);

/**
 * @brief Prints a diagnostic representation of the finder.
 * @param[in] a Finder to print; NULL prints a null diagnostic.
 * @byul.nullable a true
 */
BYUL_API void route_finder_print(const route_finder_t* a);

/**
 * @brief Runs the selected legacy dispatcher algorithm.
 * @param[in,out] a Configured finder.
 * @return Caller-owned route, or NULL when validation or search fails.
 * @byul.nullable a false
 * @byul.nullable return true
 * @byul.lifetime return caller-owned
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

#endif /* BYUL_ROUTE_FINDER_H */
