/*
 * Copyright (c) 2025-2026 ByulPapa (byuldev@outlook.kr)
 * This file is part of the Byul World project.
 * Licensed under the Byul World Source-Available Non-Commercial License v1.0 (2025).
 * See the LICENSE file in the project root for full license terms.
 */

/**
 * @file navsys.h
 * @brief 경로 검색을 위한 안정된 Navsys public C facade를 선언한다.
 *
 * Versioned one-shot query, capability descriptor와 ABI 1 호환 helper를 제공한다.
 * 다른 Navsys component 전체가 필요하면 navsys_all.h를 명시적으로 include한다.
 */

#ifndef BYUL_NAVSYS_H
#define BYUL_NAVSYS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "byul_config.h"
#include "coord.h"
#include "navsys_status.h"
#include "route_finder.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct s_navgrid navgrid_t;
typedef struct s_route route_t;

#define NAVSYS_PATH_QUERY_VERSION 1u
#define NAVSYS_ALGORITHM_CONFIG_VERSION 1u
#define NAVSYS_ALGORITHM_DESCRIPTOR_VERSION 1u
#define NAVSYS_DEFAULT_MAX_EXPANSIONS 10000u

/** Algorithm-specific configuration discriminator. */
typedef enum e_navsys_algorithm_config_kind {
    NAVSYS_ALGORITHM_CONFIG_NONE = 0,
    NAVSYS_ALGORITHM_CONFIG_FRINGE_SEARCH = 1,
    NAVSYS_ALGORITHM_CONFIG_RTA_STAR = 2,
    NAVSYS_ALGORITHM_CONFIG_SMA_STAR = 3,
    NAVSYS_ALGORITHM_CONFIG_WEIGHTED_ASTAR = 4
} navsys_algorithm_config_kind_t;

/** Versioned, typed algorithm configuration value. */
typedef struct s_navsys_algorithm_config {
    size_t struct_size;
    uint32_t version;
    navsys_algorithm_config_kind_t kind;
    union {
        route_finder_fringe_search_config_t fringe_search;
        route_finder_rta_star_config_t rta_star;
        route_finder_sma_star_config_t sma_star;
        route_finder_weighted_astar_config_t weighted_astar;
    } value;
} navsys_algorithm_config_t;

/** Versioned one-shot path query. The grid is borrowed for the call only. */
typedef struct s_navsys_path_query {
    size_t struct_size;
    uint32_t version;
    navgrid_t* grid;
    coord_t start;
    coord_t goal;
    route_finder_type_t algorithm;
    const navsys_algorithm_config_t* algorithm_config;
    size_t max_expansions;
    route_finder_cancel_func cancel_func;
    void* cancel_userdata;
} navsys_path_query_t;

/** Normalized statistics returned by navsys_find_path. */
typedef struct s_navsys_search_stats {
    size_t expansions;
    size_t route_length;
    float route_cost;
    bool complete;
    bool partial;
    route_finder_type_t algorithm;
    navsys_status_t status;
} navsys_search_stats_t;

/** Machine-readable semantic claims for one algorithm. */
typedef struct s_navsys_algorithm_descriptor {
    size_t struct_size;
    uint32_t version;
    route_finder_type_t algorithm;
    navsys_algorithm_config_kind_t required_config;
    bool supported;
    bool experimental;
    bool complete;
    bool optimal;
    bool bounded_suboptimal;
    bool incremental;
} navsys_algorithm_descriptor_t;

/**
 * @brief Initializes a path query with stable defaults.
 * @param[out] out_query Destination query.
 * @param[in] grid Grid borrowed during navsys_find_path.
 * @param[in] start Start coordinate copied into the query.
 * @param[in] goal Goal coordinate copied into the query.
 * @return Common Navsys status value.
 * @byul.nullable out_query false
 * @byul.nullable grid false
 * @byul.nullable start false
 * @byul.nullable goal false
 */
BYUL_API navsys_status_t navsys_path_query_init(
    navsys_path_query_t* out_query, navgrid_t* grid,
    const coord_t* start, const coord_t* goal);

/**
 * @brief Initializes a typed algorithm configuration value.
 * @param[out] out_config Destination configuration.
 * @param[in] kind Configuration discriminator.
 * @return Common Navsys status value.
 * @byul.nullable out_config false
 */
BYUL_API navsys_status_t navsys_algorithm_config_init(
    navsys_algorithm_config_t* out_config,
    navsys_algorithm_config_kind_t kind);

/**
 * @brief Reports whether the canonical one-shot facade supports an algorithm.
 * @param[in] algorithm Algorithm to inspect.
 * @return true only when navsys_find_path can execute the algorithm.
 */
BYUL_API bool navsys_is_algorithm_supported(
    route_finder_type_t algorithm);

/**
 * @brief Fetches capability and guarantee metadata for an algorithm.
 * @param[in] algorithm Algorithm to inspect.
 * @param[out] out_descriptor Destination descriptor.
 * @return Common Navsys status value.
 * @byul.nullable out_descriptor false
 */
BYUL_API navsys_status_t navsys_get_algorithm_descriptor(
    route_finder_type_t algorithm,
    navsys_algorithm_descriptor_t* out_descriptor);

/**
 * @brief Executes a validated one-shot path query.
 *
 * OK returns a complete caller-owned route. NO_PATH commits a NULL route and
 * statistics. CANCELLED or LIMIT_REACHED commit statistics and may return a
 * caller-owned partial route. Validation, unsupported, allocation, callback,
 * and corrupt-state failures preserve both outputs. Returned routes are
 * destroyed with route_destroy.
 *
 * @param[in] query Versioned query borrowed for the call.
 * @param[out] out_route Receives a caller-owned route on documented statuses.
 * @param[out] out_stats Receives normalized statistics on documented statuses.
 * @return Common Navsys status value.
 * @byul.nullable query false
 * @byul.nullable out_route false
 * @byul.nullable out_stats false
 * @byul.lifetime out_route caller-owned
 * @byul.side_effect allocates:out_route,invokes-callbacks
 * @byul.thread_safety thread-compatible
 * @byul.blocking true
 * @byul.reentrant true
 */
BYUL_API navsys_status_t navsys_find_path(
    const navsys_path_query_t* query,
    route_t** out_route,
    navsys_search_stats_t* out_stats);

/**
 * @brief ABI 1 A* convenience helper.
 * @param[in] ng Borrowed grid.
 * @param[in] start Optional start; NULL means origin.
 * @param[in] goal Goal.
 * @return Caller-owned route or NULL.
 * @byul.nullable ng false
 * @byul.nullable start true
 * @byul.nullable goal false
 * @byul.nullable return true
 * @byul.lifetime return caller-owned
 */
BYUL_API route_t* navsys_find_astar(navgrid_t* ng, const coord_t* start, const coord_t* goal);
/**
 * @brief ABI 1 BFS convenience helper.
 * @param[in] ng Borrowed grid.
 * @param[in] start Optional start; NULL means origin.
 * @param[in] goal Goal.
 * @return Caller-owned route or NULL.
 * @byul.nullable ng false
 * @byul.nullable start true
 * @byul.nullable goal false
 * @byul.nullable return true
 * @byul.lifetime return caller-owned
 */
BYUL_API route_t* navsys_find_bfs(navgrid_t* ng, const coord_t* start, const coord_t* goal);
/**
 * @brief ABI 1 DFS convenience helper.
 * @param[in] ng Borrowed grid.
 * @param[in] start Optional start; NULL means origin.
 * @param[in] goal Goal.
 * @return Caller-owned route or NULL.
 * @byul.nullable ng false
 * @byul.nullable start true
 * @byul.nullable goal false
 * @byul.nullable return true
 * @byul.lifetime return caller-owned
 */
BYUL_API route_t* navsys_find_dfs(navgrid_t* ng, const coord_t* start, const coord_t* goal);
/**
 * @brief ABI 1 Dijkstra convenience helper.
 * @param[in] ng Borrowed grid.
 * @param[in] start Optional start; NULL means origin.
 * @param[in] goal Goal.
 * @return Caller-owned route or NULL.
 * @byul.nullable ng false
 * @byul.nullable start true
 * @byul.nullable goal false
 * @byul.nullable return true
 * @byul.lifetime return caller-owned
 */
BYUL_API route_t* navsys_find_dijkstra(navgrid_t* ng, const coord_t* start, const coord_t* goal);
/**
 * @brief ABI 1 Greedy Best-First convenience helper.
 * @param[in] ng Borrowed grid.
 * @param[in] start Optional start; NULL means origin.
 * @param[in] goal Goal.
 * @return Caller-owned route or NULL.
 * @byul.nullable ng false
 * @byul.nullable start true
 * @byul.nullable goal false
 * @byul.nullable return true
 * @byul.lifetime return caller-owned
 */
BYUL_API route_t* navsys_find_greedy_best_first(navgrid_t* ng, const coord_t* start, const coord_t* goal);
/**
 * @brief ABI 1 Fast Marching compatibility helper.
 * @param[in] ng Borrowed grid.
 * @param[in] start Optional start; NULL means origin.
 * @param[in] goal Goal.
 * @return Caller-owned route or NULL.
 * @byul.nullable ng false
 * @byul.nullable start true
 * @byul.nullable goal false
 * @byul.nullable return true
 * @byul.lifetime return caller-owned
 */
BYUL_API route_t* navsys_find_fast_marching(navgrid_t* ng, const coord_t* start, const coord_t* goal);
/**
 * @brief ABI 1 IDA* compatibility helper.
 * @param[in] ng Borrowed grid.
 * @param[in] start Optional start; NULL means origin.
 * @param[in] goal Goal.
 * @return Caller-owned route or NULL.
 * @byul.nullable ng false
 * @byul.nullable start true
 * @byul.nullable goal false
 * @byul.nullable return true
 * @byul.lifetime return caller-owned
 */
BYUL_API route_t* navsys_find_ida_star(navgrid_t* ng, const coord_t* start, const coord_t* goal);
/**
 * @brief ABI 1 Fringe Search compatibility helper.
 * @param[in] ng Borrowed grid.
 * @param[in] start Optional start; NULL means origin.
 * @param[in] goal Goal.
 * @param[in] delta_epsilon Legacy threshold delta.
 * @return Caller-owned route or NULL.
 * @byul.nullable ng false
 * @byul.nullable start true
 * @byul.nullable goal false
 * @byul.nullable return true
 * @byul.lifetime return caller-owned
 */
BYUL_API route_t* navsys_find_fringe_search(navgrid_t* ng, const coord_t* start, const coord_t* goal, float delta_epsilon);
/**
 * @brief ABI 1 RTA* compatibility helper.
 * @param[in] ng Borrowed grid.
 * @param[in] start Optional start; NULL means origin.
 * @param[in] goal Goal.
 * @param[in] depth_limit Legacy depth limit.
 * @return Caller-owned route or NULL.
 * @byul.nullable ng false
 * @byul.nullable start true
 * @byul.nullable goal false
 * @byul.nullable return true
 * @byul.lifetime return caller-owned
 */
BYUL_API route_t* navsys_find_rta_star(navgrid_t* ng, const coord_t* start, const coord_t* goal, int depth_limit);
/**
 * @brief ABI 1 SMA* compatibility helper.
 * @param[in] ng Borrowed grid.
 * @param[in] start Optional start; NULL means origin.
 * @param[in] goal Goal.
 * @param[in] memory_limit Legacy node limit.
 * @return Caller-owned route or NULL.
 * @byul.nullable ng false
 * @byul.nullable start true
 * @byul.nullable goal false
 * @byul.nullable return true
 * @byul.lifetime return caller-owned
 */
BYUL_API route_t* navsys_find_sma_star(navgrid_t* ng, const coord_t* start, const coord_t* goal, int memory_limit);
/**
 * @brief ABI 1 Weighted A* convenience helper.
 * @param[in] ng Borrowed grid.
 * @param[in] start Optional start; NULL means origin.
 * @param[in] goal Goal.
 * @param[in] weight Heuristic weight.
 * @return Caller-owned route or NULL.
 * @byul.nullable ng false
 * @byul.nullable start true
 * @byul.nullable goal false
 * @byul.nullable return true
 * @byul.lifetime return caller-owned
 */
BYUL_API route_t* navsys_find_weighted_astar(navgrid_t* ng, const coord_t* start, const coord_t* goal, float weight);
/**
 * @brief ABI 1 static D* Lite convenience helper.
 * @param[in] ng Borrowed grid.
 * @param[in] start Start.
 * @param[in] goal Goal.
 * @return Caller-owned route or NULL.
 * @byul.nullable ng false
 * @byul.nullable start false
 * @byul.nullable goal false
 * @byul.nullable return true
 * @byul.lifetime return caller-owned
 */
BYUL_API route_t* navsys_find_dstar_lite(navgrid_t* ng, const coord_t* start, const coord_t* goal);

#ifdef __cplusplus
}
#endif

#endif /* BYUL_NAVSYS_H */
