#include "navsys.h"

#include "dstar_lite.h"
#include "navgrid.h"
#include "route.h"
#include "route_finder_evaluation.h"

#include <climits>
#include <new>

namespace {

bool is_known_algorithm(route_finder_type_t algorithm) {
    return algorithm > ROUTE_FINDER_UNKNOWN && algorithm <= ROUTE_FINDER_MCTS;
}

navsys_algorithm_config_kind_t required_config(
    route_finder_type_t algorithm) {
    switch (algorithm) {
        case ROUTE_FINDER_FRINGE_SEARCH:
            return NAVSYS_ALGORITHM_CONFIG_FRINGE_SEARCH;
        case ROUTE_FINDER_RTA_STAR:
            return NAVSYS_ALGORITHM_CONFIG_RTA_STAR;
        case ROUTE_FINDER_SMA_STAR:
            return NAVSYS_ALGORITHM_CONFIG_SMA_STAR;
        case ROUTE_FINDER_WEIGHTED_ASTAR:
            return NAVSYS_ALGORITHM_CONFIG_WEIGHTED_ASTAR;
        default:
            return NAVSYS_ALGORITHM_CONFIG_NONE;
    }
}

bool is_canonical_algorithm(route_finder_type_t algorithm) {
    return route_finder_is_supported(algorithm)
        || algorithm == ROUTE_FINDER_DSTAR_LITE;
}

bool config_header_valid(const navsys_algorithm_config_t* config) {
    return config
        && config->struct_size >= sizeof(navsys_algorithm_config_t)
        && config->version == NAVSYS_ALGORITHM_CONFIG_VERSION;
}

navsys_status_t validate_config(const navsys_path_query_t& query) {
    const auto expected = required_config(query.algorithm);
    if (expected == NAVSYS_ALGORITHM_CONFIG_NONE) {
        if (!query.algorithm_config) return NAVSYS_STATUS_OK;
        return config_header_valid(query.algorithm_config)
                && query.algorithm_config->kind == NAVSYS_ALGORITHM_CONFIG_NONE
            ? NAVSYS_STATUS_OK : NAVSYS_STATUS_INVALID_ARGUMENT;
    }
    if (!config_header_valid(query.algorithm_config)
        || query.algorithm_config->kind != expected)
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    return NAVSYS_STATUS_OK;
}

void fill_route_stats(
    route_finder_type_t algorithm, navsys_status_t status,
    const route_finder_run_stats_t& source,
    navsys_search_stats_t& destination) {
    destination = {};
    destination.expansions = source.total_retry_count < 0
        ? 0u : static_cast<size_t>(source.total_retry_count);
    destination.route_length = source.route_length < 0
        ? 0u : static_cast<size_t>(source.route_length);
    destination.route_cost = source.route_cost;
    destination.complete = source.complete;
    destination.partial = source.partial;
    destination.algorithm = algorithm;
    destination.status = status;
}

navsys_status_t bind_query_config(
    route_finder_t* finder, const navsys_path_query_t& query) {
    if (!query.algorithm_config) return NAVSYS_STATUS_OK;
    switch (query.algorithm_config->kind) {
        case NAVSYS_ALGORITHM_CONFIG_NONE:
            return NAVSYS_STATUS_OK;
        case NAVSYS_ALGORITHM_CONFIG_WEIGHTED_ASTAR:
            return route_finder_bind_weighted_astar_config(
                finder, &query.algorithm_config->value.weighted_astar);
        default:
            return NAVSYS_STATUS_UNSUPPORTED;
    }
}

navsys_status_t run_route_finder_query(
    const navsys_path_query_t& query, route_t** out_route,
    navsys_search_stats_t* out_stats) {
    if (query.max_expansions == 0
        || query.max_expansions > static_cast<size_t>(INT_MAX))
        return NAVSYS_STATUS_INVALID_ARGUMENT;

    route_finder_t* finder = route_finder_create_full(
        query.grid, &query.start, &query.goal, query.algorithm, nullptr,
        static_cast<int>(query.max_expansions), false,
        default_cost, nullptr, euclidean_heuristic, nullptr);
    if (!finder) return NAVSYS_STATUS_OUT_OF_MEMORY;

    navsys_status_t status = NAVSYS_STATUS_OK;
    if (query.algorithm == ROUTE_FINDER_ASTAR) {
        status = route_finder_bind_heuristic_func_ex(
            finder, route_finder_heuristic_chebyshev, nullptr);
    }
    if (status == NAVSYS_STATUS_OK)
        status = bind_query_config(finder, query);

    route_t* route = nullptr;
    route_finder_run_stats_t raw_stats{};
    if (status == NAVSYS_STATUS_OK) {
        route_finder_run_options_t options{
            sizeof(route_finder_run_options_t),
            query.cancel_func,
            query.cancel_userdata};
        status = route_finder_run_with_options(
            finder, &options, &route, &raw_stats);
    }
    route_finder_destroy(finder);

    if (status == NAVSYS_STATUS_OK
        || status == NAVSYS_STATUS_NO_PATH
        || status == NAVSYS_STATUS_LIMIT_REACHED
        || status == NAVSYS_STATUS_CANCELLED) {
        navsys_search_stats_t stats{};
        fill_route_stats(query.algorithm, status, raw_stats, stats);
        *out_route = route;
        *out_stats = stats;
    } else {
        route_destroy(route);
    }
    return status;
}

navsys_status_t run_dstar_query(
    const navsys_path_query_t& query, route_t** out_route,
    navsys_search_stats_t* out_stats) {
    dstar_lite_create_info_t create_info{};
    navsys_status_t status = dstar_lite_create_info_init(
        &create_info, query.grid, &query.start, &query.goal);
    if (status != NAVSYS_STATUS_OK) return status;
    dstar_lite_t* planner = nullptr;
    status = dstar_lite_create_ex(&create_info, &planner);
    if (status != NAVSYS_STATUS_OK) return status;

    dstar_lite_replan_options_t options{};
    status = dstar_lite_replan_options_init(&options);
    if (status == NAVSYS_STATUS_OK) {
        options.max_expansions = query.max_expansions;
        options.cancel_func = query.cancel_func;
        options.cancel_userdata = query.cancel_userdata;
    }
    route_t* route = nullptr;
    dstar_lite_stats_t raw_stats{};
    if (status == NAVSYS_STATUS_OK)
        status = dstar_lite_replan(planner, &options, &route, &raw_stats);
    dstar_lite_destroy(planner);

    if (status == NAVSYS_STATUS_OK
        || status == NAVSYS_STATUS_NO_PATH
        || status == NAVSYS_STATUS_LIMIT_REACHED
        || status == NAVSYS_STATUS_CANCELLED) {
        navsys_search_stats_t stats{};
        stats.expansions = raw_stats.expansions;
        stats.route_length = raw_stats.route_steps;
        double route_cost = 0.0;
        if (route) route_fetch_total_cost(route, &route_cost);
        stats.route_cost = static_cast<float>(route_cost);
        stats.complete = status == NAVSYS_STATUS_OK;
        stats.partial = route && status != NAVSYS_STATUS_OK;
        stats.algorithm = query.algorithm;
        stats.status = status;
        *out_route = route;
        *out_stats = stats;
    } else {
        route_destroy(route);
    }
    return status;
}

route_t* run_legacy_route_finder(
    navgrid_t* grid, const coord_t* start, const coord_t* goal,
    route_finder_type_t algorithm,
    const navsys_algorithm_config_t* config) {
    if (!grid || !goal) return nullptr;
    const coord_t origin{0, 0};
    const coord_t* actual_start = start ? start : &origin;
    try {
        route_finder_t* finder = route_finder_create_full(
            grid, actual_start, goal, algorithm, nullptr, MAX_RETRY, false,
            default_cost, nullptr,
            algorithm == ROUTE_FINDER_IDA_STAR
                ? manhattan_heuristic : euclidean_heuristic,
            nullptr);
        if (!finder) return nullptr;
        navsys_status_t bind_status = NAVSYS_STATUS_OK;
        if (config) {
            switch (config->kind) {
                case NAVSYS_ALGORITHM_CONFIG_FRINGE_SEARCH:
                    bind_status = route_finder_bind_fringe_search_config(
                        finder, &config->value.fringe_search);
                    break;
                case NAVSYS_ALGORITHM_CONFIG_RTA_STAR:
                    bind_status = route_finder_bind_rta_star_config(
                        finder, &config->value.rta_star);
                    break;
                case NAVSYS_ALGORITHM_CONFIG_SMA_STAR:
                    bind_status = route_finder_bind_sma_star_config(
                        finder, &config->value.sma_star);
                    break;
                case NAVSYS_ALGORITHM_CONFIG_WEIGHTED_ASTAR:
                    bind_status = route_finder_bind_weighted_astar_config(
                        finder, &config->value.weighted_astar);
                    break;
                default:
                    bind_status = NAVSYS_STATUS_INVALID_ARGUMENT;
                    break;
            }
        }
        route_t* result = bind_status == NAVSYS_STATUS_OK
            ? route_finder_run(finder) : nullptr;
        route_finder_destroy(finder);
        return result;
    } catch (...) {
        return nullptr;
    }
}

navsys_algorithm_config_t make_config(navsys_algorithm_config_kind_t kind) {
    navsys_algorithm_config_t config{};
    navsys_algorithm_config_init(&config, kind);
    return config;
}

} // namespace

navsys_status_t navsys_path_query_init(
    navsys_path_query_t* out_query, navgrid_t* grid,
    const coord_t* start, const coord_t* goal) {
    if (!out_query || !grid || !start || !goal)
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    navsys_path_query_t query{};
    query.struct_size = sizeof(query);
    query.version = NAVSYS_PATH_QUERY_VERSION;
    query.grid = grid;
    query.start = *start;
    query.goal = *goal;
    query.algorithm = ROUTE_FINDER_ASTAR;
    query.max_expansions = NAVSYS_DEFAULT_MAX_EXPANSIONS;
    *out_query = query;
    return NAVSYS_STATUS_OK;
}

navsys_status_t navsys_algorithm_config_init(
    navsys_algorithm_config_t* out_config,
    navsys_algorithm_config_kind_t kind) {
    if (!out_config || kind < NAVSYS_ALGORITHM_CONFIG_NONE
        || kind > NAVSYS_ALGORITHM_CONFIG_WEIGHTED_ASTAR)
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    navsys_algorithm_config_t config{};
    config.struct_size = sizeof(config);
    config.version = NAVSYS_ALGORITHM_CONFIG_VERSION;
    config.kind = kind;
    switch (kind) {
        case NAVSYS_ALGORITHM_CONFIG_FRINGE_SEARCH:
            config.value.fringe_search.delta_epsilon = 0.3f;
            break;
        case NAVSYS_ALGORITHM_CONFIG_RTA_STAR:
            config.value.rta_star.depth_limit = 5;
            break;
        case NAVSYS_ALGORITHM_CONFIG_SMA_STAR:
            config.value.sma_star.memory_limit = 20;
            break;
        case NAVSYS_ALGORITHM_CONFIG_WEIGHTED_ASTAR:
            config.value.weighted_astar.weight = 1.5f;
            break;
        default:
            break;
    }
    *out_config = config;
    return NAVSYS_STATUS_OK;
}

bool navsys_is_algorithm_supported(route_finder_type_t algorithm) {
    return is_canonical_algorithm(algorithm);
}

navsys_status_t navsys_get_algorithm_descriptor(
    route_finder_type_t algorithm,
    navsys_algorithm_descriptor_t* out_descriptor) {
    if (!out_descriptor || !is_known_algorithm(algorithm))
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    navsys_algorithm_descriptor_t descriptor{};
    descriptor.struct_size = sizeof(descriptor);
    descriptor.version = NAVSYS_ALGORITHM_DESCRIPTOR_VERSION;
    descriptor.algorithm = algorithm;
    descriptor.required_config = required_config(algorithm);
    descriptor.supported = is_canonical_algorithm(algorithm);
    descriptor.experimental = !descriptor.supported;
    descriptor.incremental = algorithm == ROUTE_FINDER_DSTAR_LITE;
    descriptor.complete = algorithm == ROUTE_FINDER_ASTAR
        || algorithm == ROUTE_FINDER_BFS
        || algorithm == ROUTE_FINDER_DFS
        || algorithm == ROUTE_FINDER_DIJKSTRA
        || algorithm == ROUTE_FINDER_DSTAR_LITE;
    descriptor.optimal = algorithm == ROUTE_FINDER_ASTAR
        || algorithm == ROUTE_FINDER_BFS
        || algorithm == ROUTE_FINDER_DIJKSTRA
        || algorithm == ROUTE_FINDER_DSTAR_LITE;
    descriptor.bounded_suboptimal = false;
    *out_descriptor = descriptor;
    return NAVSYS_STATUS_OK;
}

navsys_status_t navsys_find_path(
    const navsys_path_query_t* query, route_t** out_route,
    navsys_search_stats_t* out_stats) {
    if (!query || !out_route || !out_stats
        || query->struct_size < sizeof(navsys_path_query_t)
        || query->version != NAVSYS_PATH_QUERY_VERSION
        || !query->grid
        || !is_known_algorithm(query->algorithm)
        || !navgrid_is_inside(query->grid, query->start.x, query->start.y)
        || !navgrid_is_inside(query->grid, query->goal.x, query->goal.y))
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    if (!is_canonical_algorithm(query->algorithm))
        return NAVSYS_STATUS_UNSUPPORTED;
    const navsys_status_t config_status = validate_config(*query);
    if (config_status != NAVSYS_STATUS_OK) return config_status;
    try {
        return query->algorithm == ROUTE_FINDER_DSTAR_LITE
            ? run_dstar_query(*query, out_route, out_stats)
            : run_route_finder_query(*query, out_route, out_stats);
    } catch (const std::bad_alloc&) {
        return NAVSYS_STATUS_OUT_OF_MEMORY;
    } catch (...) {
        return NAVSYS_STATUS_CORRUPT_STATE;
    }
}

route_t* navsys_find_astar(navgrid_t* ng, const coord_t* start, const coord_t* goal) { return run_legacy_route_finder(ng, start, goal, ROUTE_FINDER_ASTAR, nullptr); }
route_t* navsys_find_bfs(navgrid_t* ng, const coord_t* start, const coord_t* goal) { return run_legacy_route_finder(ng, start, goal, ROUTE_FINDER_BFS, nullptr); }
route_t* navsys_find_dfs(navgrid_t* ng, const coord_t* start, const coord_t* goal) { return run_legacy_route_finder(ng, start, goal, ROUTE_FINDER_DFS, nullptr); }
route_t* navsys_find_dijkstra(navgrid_t* ng, const coord_t* start, const coord_t* goal) { return run_legacy_route_finder(ng, start, goal, ROUTE_FINDER_DIJKSTRA, nullptr); }
route_t* navsys_find_greedy_best_first(navgrid_t* ng, const coord_t* start, const coord_t* goal) { return run_legacy_route_finder(ng, start, goal, ROUTE_FINDER_GREEDY_BEST_FIRST, nullptr); }
route_t* navsys_find_fast_marching(navgrid_t* ng, const coord_t* start, const coord_t* goal) { return run_legacy_route_finder(ng, start, goal, ROUTE_FINDER_FAST_MARCHING, nullptr); }
route_t* navsys_find_ida_star(navgrid_t* ng, const coord_t* start, const coord_t* goal) { return run_legacy_route_finder(ng, start, goal, ROUTE_FINDER_IDA_STAR, nullptr); }

route_t* navsys_find_fringe_search(navgrid_t* ng, const coord_t* start, const coord_t* goal, float delta_epsilon) {
    auto config = make_config(NAVSYS_ALGORITHM_CONFIG_FRINGE_SEARCH);
    config.value.fringe_search.delta_epsilon = delta_epsilon;
    return run_legacy_route_finder(ng, start, goal, ROUTE_FINDER_FRINGE_SEARCH, &config);
}

route_t* navsys_find_rta_star(navgrid_t* ng, const coord_t* start, const coord_t* goal, int depth_limit) {
    auto config = make_config(NAVSYS_ALGORITHM_CONFIG_RTA_STAR);
    config.value.rta_star.depth_limit = depth_limit;
    return run_legacy_route_finder(ng, start, goal, ROUTE_FINDER_RTA_STAR, &config);
}

route_t* navsys_find_sma_star(navgrid_t* ng, const coord_t* start, const coord_t* goal, int memory_limit) {
    auto config = make_config(NAVSYS_ALGORITHM_CONFIG_SMA_STAR);
    config.value.sma_star.memory_limit = memory_limit;
    return run_legacy_route_finder(ng, start, goal, ROUTE_FINDER_SMA_STAR, &config);
}

route_t* navsys_find_weighted_astar(navgrid_t* ng, const coord_t* start, const coord_t* goal, float weight) {
    auto config = make_config(NAVSYS_ALGORITHM_CONFIG_WEIGHTED_ASTAR);
    config.value.weighted_astar.weight = weight;
    return run_legacy_route_finder(ng, start, goal, ROUTE_FINDER_WEIGHTED_ASTAR, &config);
}

route_t* navsys_find_dstar_lite(navgrid_t* ng, const coord_t* start, const coord_t* goal) {
    if (!ng || !start || !goal) return nullptr;
    navsys_path_query_t query{};
    if (navsys_path_query_init(&query, ng, start, goal) != NAVSYS_STATUS_OK)
        return nullptr;
    query.algorithm = ROUTE_FINDER_DSTAR_LITE;
    query.max_expansions = 0;
    route_t* route = nullptr;
    navsys_search_stats_t stats{};
    return navsys_find_path(&query, &route, &stats) == NAVSYS_STATUS_OK
        ? route : nullptr;
}
