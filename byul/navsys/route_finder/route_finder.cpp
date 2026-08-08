// route_finder.cpp

#include "route_finder.h"
#include "cost_coord_pq.h"
#include "coord_list.h"
#include "coord.h"

#include <cmath>
#include <cstdio>
#include <vector>
#include <limits>
#include <cstring>
#include <mutex>
#include <type_traits>
#include <unordered_map>

#include "astar.h"
#include "bfs.h"
#include "dfs.h"
#include "dijkstra.h"
#include "fast_marching.h"
#include "fringe_search.h"
#include "greedy_best_first.h"
#include "ida_star.h"
#include "rta_star.h"
#include "sma_star.h"
#include "weighted_astar.h"

static thread_local route_finder_t* active_callback_finder = nullptr;
static thread_local navsys_status_t active_evaluation_status =
    NAVSYS_STATUS_OK;

struct route_finder_evaluation_binding {
    route_finder_cost_func_ex cost_func;
    void* cost_userdata;
    route_finder_heuristic_func_ex heuristic_func;
    void* heuristic_userdata;
};

static std::mutex evaluation_bindings_mutex;
static std::unordered_map<const route_finder_t*, route_finder_evaluation_binding>
    evaluation_bindings;

static route_finder_evaluation_binding route_finder_get_evaluation_binding(
    const route_finder_t* finder) {
    std::lock_guard<std::mutex> lock(evaluation_bindings_mutex);
    const auto found = evaluation_bindings.find(finder);
    return found == evaluation_bindings.end()
        ? route_finder_evaluation_binding{}
        : found->second;
}

static void route_finder_erase_evaluation_binding(
    const route_finder_t* finder) {
    std::lock_guard<std::mutex> lock(evaluation_bindings_mutex);
    evaluation_bindings.erase(finder);
}

struct route_finder_cancel_context {
    route_finder_cancel_func callback;
    void* userdata;
    bool requested;
    bool failed;
};

static thread_local route_finder_cancel_context* active_cancel_context = nullptr;

bool route_finder_poll_cancel_internal(void) {
    if (active_evaluation_status != NAVSYS_STATUS_OK) return true;
    route_finder_cancel_context* context = active_cancel_context;
    if (!context || context->requested || context->failed)
        return context && (context->requested || context->failed);
    if (!context->callback) return false;
    try {
        context->requested = context->callback(context->userdata);
    } catch (...) {
        context->failed = true;
    }
    return context->requested || context->failed;
}

class route_finder_cancel_scope {
public:
    explicit route_finder_cancel_scope(route_finder_cancel_context* context)
        : previous_(active_cancel_context) {
        active_cancel_context = context;
    }

    ~route_finder_cancel_scope() {
        active_cancel_context = previous_;
    }

private:
    route_finder_cancel_context* previous_;
};

const char* route_finder_type_get_name(route_finder_type_t type) {
    switch (type) {
        case ROUTE_FINDER_UNKNOWN: return "unknown";
        case ROUTE_FINDER_BELLMAN_FORD: return "bellman_ford";
        case ROUTE_FINDER_BFS: return "bfs";
        case ROUTE_FINDER_DFS: return "dfs";
        case ROUTE_FINDER_DIJKSTRA: return "dijkstra";
        case ROUTE_FINDER_FLOYD_WARSHALL: return "floyd_warshall";
        case ROUTE_FINDER_ASTAR: return "astar";
        case ROUTE_FINDER_BIDIRECTIONAL_DIJKSTRA:
            return "bidirectional_dijkstra";
        case ROUTE_FINDER_BIDIRECTIONAL_ASTAR:
            return "bidirectional_astar";
        case ROUTE_FINDER_WEIGHTED_ASTAR: return "weighted_astar";
        case ROUTE_FINDER_JOHNSON: return "johnson";
        case ROUTE_FINDER_K_SHORTEST_PATH: return "k_shortest_path";
        case ROUTE_FINDER_DIAL: return "dial";
        case ROUTE_FINDER_ITERATIVE_DEEPENING:
            return "iterative_deepening";
        case ROUTE_FINDER_GREEDY_BEST_FIRST: return "greedy_best_first";
        case ROUTE_FINDER_IDA_STAR: return "ida_star";
        case ROUTE_FINDER_RTA_STAR: return "rta_star";
        case ROUTE_FINDER_SMA_STAR: return "sma_star";
        case ROUTE_FINDER_DSTAR: return "dstar";
        case ROUTE_FINDER_FAST_MARCHING: return "fast_marching";
        case ROUTE_FINDER_ANT_COLONY: return "ant_colony";
        case ROUTE_FINDER_FRINGE_SEARCH: return "fringe_search";
        case ROUTE_FINDER_FOCAL_SEARCH: return "focal_search";
        case ROUTE_FINDER_DSTAR_LITE: return "dstar_lite";
        case ROUTE_FINDER_LPA_STAR: return "lpa_star";
        case ROUTE_FINDER_HPA_STAR: return "hpa_star";
        case ROUTE_FINDER_ALT: return "alt";
        case ROUTE_FINDER_ANY_ANGLE_ASTAR: return "any_angle_astar";
        case ROUTE_FINDER_HCA_STAR: return "hca_star";
        case ROUTE_FINDER_RTAA_STAR: return "rtaa_star";
        case ROUTE_FINDER_THETA_STAR: return "theta_star";
        case ROUTE_FINDER_CONTRACTION_HIERARCHIES:
            return "contraction_hierarchies";
        case ROUTE_FINDER_LAZY_THETA_STAR: return "lazy_theta_star";
        case ROUTE_FINDER_JUMP_POINT_SEARCH: return "jump_point_search";
        case ROUTE_FINDER_SIPP: return "sipp";
        case ROUTE_FINDER_JPS_PLUS: return "jps_plus";
        case ROUTE_FINDER_EPEA_STAR: return "epea_star";
        case ROUTE_FINDER_MHA_STAR: return "mha_star";
        case ROUTE_FINDER_ANYA: return "anya";
        case ROUTE_FINDER_DAG_SP: return "dag_sp";
        case ROUTE_FINDER_MULTI_SOURCE_BFS: return "multi_source_bfs";
        case ROUTE_FINDER_MCTS: return "mcts";
        default: return "unknown";
    }
}

const char* get_route_finder_name(route_finder_type_t type) {
    return route_finder_type_get_name(type);
}

route_finder_t* route_finder_create_full(
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
) {
    if (!navgrid || !start || !goal || !cost_fn || !heuristic_fn ||
        max_retry <= 0)
        return nullptr;

    route_finder_t* a = nullptr;
    try {
        a = new route_finder_t{};
    } catch (...) {
        return nullptr;
    }

    a->navgrid = navgrid;
    a->start = *start;
    a->goal = *goal;
    a->type = type;
    a->typedata = typedata;    
    a->max_retry = max_retry;
    a->debug_mode_enabled = debug_mode_enabled;

    a->cost_fn = cost_fn;
    a->cost_fn_userdata = cost_fn_userdata;

    a->heuristic_fn = heuristic_fn;
    a->heuristic_fn_userdata = heuristic_fn_userdata;
    return a;
}

route_finder_t* route_finder_create(navgrid_t* navgrid) {
    coord_t start;
    start.x = 0;
    start.y = 0;
    return route_finder_create_full(
        navgrid, &start, &start, 
        ROUTE_FINDER_ASTAR, nullptr,
        MAX_RETRY, false, 
        default_cost, nullptr,
        euclidean_heuristic, nullptr);
}

int route_finder_init(route_finder_t* out, navgrid_t* navgrid){
    if (!out || !navgrid) return -1;

    route_finder_erase_evaluation_binding(out);
    memset(out, 0, sizeof(route_finder_t));
    out->navgrid = navgrid;
    return 0;
}

int  route_finder_init_full(
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
){

    if (!out || !navgrid || !start || !goal || !cost_fn || !heuristic_fn
        || max_retry <= 0)
        return -1;

    route_finder_erase_evaluation_binding(out);
    out->type = type;
    out->navgrid = navgrid;
    out->start = *start;
    out->goal = *goal;
    out->cost_fn = cost_fn;
    out->cost_fn_userdata = cost_fn_userdata;
    out->heuristic_fn = heuristic_fn;
    out->heuristic_fn_userdata = heuristic_fn_userdata;
    out->max_retry = max_retry;
    out->debug_mode_enabled = debug_mode_enabled;
    out->typedata = typedata;

    return 0;
}

int route_finder_free(route_finder_t* out){
    if(!out) return -1;
    if (active_callback_finder == out) return -1;

    route_finder_erase_evaluation_binding(out);
    memset(out, 0, sizeof(route_finder_t));
    return 0;
}


int route_finder_destroy(route_finder_t* a) {
    if(!a) return -1;
    if (active_callback_finder == a) return -1;
    route_finder_erase_evaluation_binding(a);
    delete a;
    return 0;
}

route_finder_t* route_finder_copy(const route_finder_t* src) {
    if (!src) return nullptr;
    route_finder_t* copy = route_finder_create_full(
        src->navgrid,
        &src->start,
        &src->goal,
        src->type,
        src->typedata,
        src->max_retry,
        src->debug_mode_enabled,
        src->cost_fn,
        src->cost_fn_userdata,

        src->heuristic_fn,
        src->heuristic_fn_userdata
    );
    if (!copy) return nullptr;
    const route_finder_evaluation_binding binding =
        route_finder_get_evaluation_binding(src);
    if (binding.cost_func || binding.heuristic_func) {
        try {
            std::lock_guard<std::mutex> lock(evaluation_bindings_mutex);
            evaluation_bindings[copy] = binding;
        } catch (...) {
            delete copy;
            return nullptr;
        }
    }
    return copy;
}

void route_finder_clear(route_finder_t* a) {
    if (!a || active_callback_finder == a) return;
    route_finder_erase_evaluation_binding(a);
    memset(a, 0, sizeof(route_finder_t));
}

void route_finder_set_defaults(route_finder_t* a) {
    a->cost_fn = default_cost;
    a->heuristic_fn = euclidean_heuristic;
    a->max_retry = MAX_RETRY;
    a->debug_mode_enabled = false;
}

bool route_finder_is_valid(const route_finder_t* a) {
    if (!a) return false;
    return a && a->navgrid && a->cost_fn && a->heuristic_fn;
}

void route_finder_print(const route_finder_t* a) {
    if (!a) {
        printf("(route_finder: NULL)\n");
        return;
    }

    printf("route_finder_t {\n");
    printf("  navgrid:         %p\n", (void*)a->navgrid);
    printf("  start:       (%d, %d)\n", a->start.x, a->start.y);
    printf("  goal:        (%d, %d)\n", a->goal.x, a->goal.y);
    printf("  type:        %s\n", get_route_finder_name(a->type));
    printf("  typedata:    %p\n", a->typedata);
    printf("  max_retry:   %d\n", a->max_retry);
    printf("  logging:     %s\n", a->debug_mode_enabled ? "true" : "false");
    printf("  cost_fn:     %p\n", (void*)a->cost_fn);
    printf("  heuristic_fn:%p\n", (void*)a->heuristic_fn);    
    printf("}\n");
}

void route_finder_set_navgrid(route_finder_t* a, navgrid_t* navgrid) {
     a->navgrid = navgrid; 
}

void route_finder_set_start(route_finder_t* a, const coord_t* start) { 
    coord_set(&a->start, start->x, start->y);
}

void route_finder_set_goal(route_finder_t* a, const coord_t* goal) { 
    coord_set(&a->goal, goal->x, goal->y); 
}

const navgrid_t* route_finder_get_navgrid(const route_finder_t* a) { 
    return a->navgrid; 
}

int route_finder_fetch_start(const route_finder_t* a, coord_t* out) { 
    *out = a->start;
    return 0;
}

int route_finder_fetch_goal(const route_finder_t* a, coord_t* out) { 
    *out = a->goal;
    return 0;
}

void route_finder_set_type(route_finder_t* a, route_finder_type_t type){
    a->type = type;
}

route_finder_type_t route_finder_get_type(const route_finder_t* a){
    return a->type;
}

navsys_status_t route_finder_set_type_checked(
    route_finder_t* finder, route_finder_type_t type) {
    if (!finder) return NAVSYS_STATUS_INVALID_ARGUMENT;
    if (active_callback_finder == finder)
        return NAVSYS_STATUS_IN_PROGRESS;
    if (!route_finder_is_supported(type))
        return NAVSYS_STATUS_UNSUPPORTED;
    finder->type = type;
    return NAVSYS_STATUS_OK;
}

void route_finder_set_typedata(route_finder_t* a, void* typedata){
    a->typedata = typedata;
}

void* route_finder_get_typedata(const route_finder_t* a){
    return a->typedata;
}

static navsys_status_t route_finder_bind_algorithm_config(
    route_finder_t* finder, route_finder_type_t type, const void* config) {
    if (!finder || !config) return NAVSYS_STATUS_INVALID_ARGUMENT;
    if (active_callback_finder == finder)
        return NAVSYS_STATUS_IN_PROGRESS;
    finder->type = type;
    finder->typedata = const_cast<void*>(config);
    return NAVSYS_STATUS_OK;
}

navsys_status_t route_finder_bind_fringe_search_config(
    route_finder_t* finder,
    const route_finder_fringe_search_config_t* config) {
    if (!config ||
        !std::isfinite(config->delta_epsilon) ||
        config->delta_epsilon < 0.001f ||
        config->delta_epsilon > 5.0f)
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    return route_finder_bind_algorithm_config(
        finder, ROUTE_FINDER_FRINGE_SEARCH, config);
}

navsys_status_t route_finder_bind_rta_star_config(
    route_finder_t* finder,
    const route_finder_rta_star_config_t* config) {
    if (!config || config->depth_limit < 1 || config->depth_limit > 100)
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    return route_finder_bind_algorithm_config(
        finder, ROUTE_FINDER_RTA_STAR, config);
}

navsys_status_t route_finder_bind_sma_star_config(
    route_finder_t* finder,
    const route_finder_sma_star_config_t* config) {
    if (!config ||
        config->memory_limit < 10 ||
        config->memory_limit > 1000000)
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    return route_finder_bind_algorithm_config(
        finder, ROUTE_FINDER_SMA_STAR, config);
}

navsys_status_t route_finder_bind_weighted_astar_config(
    route_finder_t* finder,
    const route_finder_weighted_astar_config_t* config) {
    if (!config ||
        !std::isfinite(config->weight) ||
        config->weight < 0.1f ||
        config->weight > 10.0f)
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    return route_finder_bind_algorithm_config(
        finder, ROUTE_FINDER_WEIGHTED_ASTAR, config);
}

navsys_status_t route_finder_unbind_algorithm_config(
    route_finder_t* finder) {
    if (!finder) return NAVSYS_STATUS_INVALID_ARGUMENT;
    if (active_callback_finder == finder)
        return NAVSYS_STATUS_IN_PROGRESS;
    finder->typedata = nullptr;
    return NAVSYS_STATUS_OK;
}

void route_finder_set_max_retry(route_finder_t* a, int max_retry){
    if (!a) return;
    (void)route_finder_set_max_retry_checked(a, max_retry);
}

int route_finder_get_max_retry(route_finder_t* a){
    return a->max_retry;
}

navsys_status_t route_finder_set_max_retry_checked(
    route_finder_t* finder, int max_retry) {
    if (!finder || max_retry <= 0)
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    if (active_callback_finder == finder)
        return NAVSYS_STATUS_IN_PROGRESS;
    finder->max_retry = max_retry;
    return NAVSYS_STATUS_OK;
}

void route_finder_enable_debug_mode(route_finder_t* a, bool is_logging){
    a->debug_mode_enabled = is_logging;
}

bool route_finder_is_debug_mode_enabled(route_finder_t* a){
    return a->debug_mode_enabled;
}

void route_finder_set_cost_func(route_finder_t* a, cost_func cost_fn){
    a->cost_fn = cost_fn;
}

cost_func route_finder_get_cost_func(route_finder_t* a){
    return a->cost_fn;
}

void route_finder_set_cost_fn_userdata(route_finder_t* a, void* userdata){
    a->cost_fn_userdata = userdata;
}

void* route_finder_get_cost_fn_userdata(const route_finder_t* a){
    return a->cost_fn_userdata;
}

void route_finder_set_heuristic_func(
    route_finder_t* a, heuristic_func heuristic_fn){
    a->heuristic_fn = heuristic_fn;
}

heuristic_func route_finder_get_heuristic_func(route_finder_t* a){
    return a->heuristic_fn;
}

void route_finder_set_heuristic_fn_userdata(route_finder_t* a, void* userdata){
    a->heuristic_fn_userdata = userdata;
}

void* route_finder_get_heuristic_fn_userdata(const route_finder_t* a){
    return a->heuristic_fn_userdata;
}

navsys_status_t route_finder_bind_cost_func(
    route_finder_t* finder, cost_func fn, void* userdata) {
    if (!finder || !fn) return NAVSYS_STATUS_INVALID_ARGUMENT;
    if (active_callback_finder == finder)
        return NAVSYS_STATUS_IN_PROGRESS;
    finder->cost_fn = fn;
    finder->cost_fn_userdata = userdata;
    {
        std::lock_guard<std::mutex> lock(evaluation_bindings_mutex);
        auto found = evaluation_bindings.find(finder);
        if (found != evaluation_bindings.end()) {
            found->second.cost_func = nullptr;
            found->second.cost_userdata = nullptr;
            if (!found->second.heuristic_func)
                evaluation_bindings.erase(found);
        }
    }
    return NAVSYS_STATUS_OK;
}

navsys_status_t route_finder_bind_cost_func_ex(
    route_finder_t* finder,
    route_finder_cost_func_ex fn,
    void* userdata) {
    if (!finder || !fn) return NAVSYS_STATUS_INVALID_ARGUMENT;
    if (active_callback_finder == finder)
        return NAVSYS_STATUS_IN_PROGRESS;
    try {
        std::lock_guard<std::mutex> lock(evaluation_bindings_mutex);
        auto& binding = evaluation_bindings[finder];
        binding.cost_func = fn;
        binding.cost_userdata = userdata;
    } catch (...) {
        return NAVSYS_STATUS_OUT_OF_MEMORY;
    }
    return NAVSYS_STATUS_OK;
}

navsys_status_t route_finder_unbind_cost_func(route_finder_t* finder) {
    if (!finder) return NAVSYS_STATUS_INVALID_ARGUMENT;
    if (active_callback_finder == finder)
        return NAVSYS_STATUS_IN_PROGRESS;
    finder->cost_fn = default_cost;
    finder->cost_fn_userdata = nullptr;
    {
        std::lock_guard<std::mutex> lock(evaluation_bindings_mutex);
        auto found = evaluation_bindings.find(finder);
        if (found != evaluation_bindings.end()) {
            found->second.cost_func = nullptr;
            found->second.cost_userdata = nullptr;
            if (!found->second.heuristic_func)
                evaluation_bindings.erase(found);
        }
    }
    return NAVSYS_STATUS_OK;
}

navsys_status_t route_finder_bind_heuristic_func(
    route_finder_t* finder, heuristic_func fn, void* userdata) {
    if (!finder || !fn) return NAVSYS_STATUS_INVALID_ARGUMENT;
    if (active_callback_finder == finder)
        return NAVSYS_STATUS_IN_PROGRESS;
    finder->heuristic_fn = fn;
    finder->heuristic_fn_userdata = userdata;
    {
        std::lock_guard<std::mutex> lock(evaluation_bindings_mutex);
        auto found = evaluation_bindings.find(finder);
        if (found != evaluation_bindings.end()) {
            found->second.heuristic_func = nullptr;
            found->second.heuristic_userdata = nullptr;
            if (!found->second.cost_func)
                evaluation_bindings.erase(found);
        }
    }
    return NAVSYS_STATUS_OK;
}

navsys_status_t route_finder_bind_heuristic_func_ex(
    route_finder_t* finder,
    route_finder_heuristic_func_ex fn,
    void* userdata) {
    if (!finder || !fn) return NAVSYS_STATUS_INVALID_ARGUMENT;
    if (active_callback_finder == finder)
        return NAVSYS_STATUS_IN_PROGRESS;
    try {
        std::lock_guard<std::mutex> lock(evaluation_bindings_mutex);
        auto& binding = evaluation_bindings[finder];
        binding.heuristic_func = fn;
        binding.heuristic_userdata = userdata;
    } catch (...) {
        return NAVSYS_STATUS_OUT_OF_MEMORY;
    }
    return NAVSYS_STATUS_OK;
}

navsys_status_t route_finder_unbind_heuristic_func(
    route_finder_t* finder) {
    if (!finder) return NAVSYS_STATUS_INVALID_ARGUMENT;
    if (active_callback_finder == finder)
        return NAVSYS_STATUS_IN_PROGRESS;
    finder->heuristic_fn = euclidean_heuristic;
    finder->heuristic_fn_userdata = nullptr;
    {
        std::lock_guard<std::mutex> lock(evaluation_bindings_mutex);
        auto found = evaluation_bindings.find(finder);
        if (found != evaluation_bindings.end()) {
            found->second.heuristic_func = nullptr;
            found->second.heuristic_userdata = nullptr;
            if (!found->second.cost_func)
                evaluation_bindings.erase(found);
        }
    }
    return NAVSYS_STATUS_OK;
}

class route_finder_callback_scope {
public:
    explicit route_finder_callback_scope(route_finder_t* finder)
        : previous_(active_callback_finder),
          previous_status_(active_evaluation_status) {
        active_callback_finder = finder;
        active_evaluation_status = NAVSYS_STATUS_OK;
    }

    ~route_finder_callback_scope() {
        active_callback_finder = previous_;
        active_evaluation_status = previous_status_;
    }

    navsys_status_t evaluation_status() const {
        return active_evaluation_status;
    }

private:
    route_finder_t* previous_;
    navsys_status_t previous_status_;
};

static float route_finder_cost_bridge(
    const navgrid_t* navgrid,
    const coord_t* start,
    const coord_t* goal,
    void*) {
    route_finder_t* finder = active_callback_finder;
    if (!finder || active_evaluation_status != NAVSYS_STATUS_OK)
        return 0.0f;
    float value = 0.0f;
    try {
        const route_finder_evaluation_binding binding =
            route_finder_get_evaluation_binding(finder);
        if (binding.cost_func) {
            const navsys_status_t status = binding.cost_func(
                navgrid, start, goal, &value, binding.cost_userdata);
            if (status != NAVSYS_STATUS_OK) {
                active_evaluation_status = NAVSYS_STATUS_CALLBACK_FAILED;
                return 0.0f;
            }
        } else {
            value = finder->cost_fn(
                navgrid, start, goal, finder->cost_fn_userdata);
        }
    } catch (...) {
        active_evaluation_status = NAVSYS_STATUS_CALLBACK_FAILED;
        return 0.0f;
    }
    if (!std::isfinite(value) || value < 0.0f) {
        active_evaluation_status = NAVSYS_STATUS_CALLBACK_FAILED;
        return 0.0f;
    }
    return value;
}

static float route_finder_heuristic_bridge(
    const coord_t* start,
    const coord_t* goal,
    void*) {
    route_finder_t* finder = active_callback_finder;
    if (!finder || active_evaluation_status != NAVSYS_STATUS_OK)
        return 0.0f;
    float value = 0.0f;
    try {
        const route_finder_evaluation_binding binding =
            route_finder_get_evaluation_binding(finder);
        if (binding.heuristic_func) {
            const navsys_status_t status = binding.heuristic_func(
                start, goal, &value, binding.heuristic_userdata);
            if (status != NAVSYS_STATUS_OK) {
                active_evaluation_status = NAVSYS_STATUS_CALLBACK_FAILED;
                return 0.0f;
            }
        } else {
            value = finder->heuristic_fn(
                start, goal, finder->heuristic_fn_userdata);
        }
    } catch (...) {
        active_evaluation_status = NAVSYS_STATUS_CALLBACK_FAILED;
        return 0.0f;
    }
    if (!std::isfinite(value) || value < 0.0f) {
        active_evaluation_status = NAVSYS_STATUS_CALLBACK_FAILED;
        return 0.0f;
    }
    return value;
}

static route_t* route_finder_run_astar(route_finder_t* a){
    return find_astar(a->navgrid, &a->start, &a->goal, 
        route_finder_cost_bridge, route_finder_heuristic_bridge,
        a->max_retry, a->debug_mode_enabled);
}

static route_t* route_finder_run_bfs(route_finder_t* a){
    return find_bfs(a->navgrid, &a->start, &a->goal, 
        a->max_retry, a->debug_mode_enabled);
}

static route_t* route_finder_run_dfs(route_finder_t* a){
    return find_dfs(a->navgrid, &a->start, &a->goal, a->max_retry,
        a->debug_mode_enabled);
}

static route_t* route_finder_run_dijkstra(route_finder_t* a){
    return find_dijkstra(
        a->navgrid, &a->start, &a->goal, route_finder_cost_bridge,
        a->max_retry, a->debug_mode_enabled);
}

static route_t* route_finder_run_fringe_search(route_finder_t* a) {
    float delta_epsilon = 0.3f;

    if (a->typedata) {
        float v = static_cast<const route_finder_fringe_search_config_t*>(
            a->typedata)->delta_epsilon;
        if (v >= 0.001f && v <= 5.0f) {
            delta_epsilon = v;
        }
    }

    return find_fringe_search(a->navgrid, &a->start, &a->goal,
        route_finder_cost_bridge, route_finder_heuristic_bridge, delta_epsilon,
        a->max_retry, a->debug_mode_enabled);
}

static route_t* route_finder_run_greedy_best_first(route_finder_t* a){
    return find_greedy_best_first(a->navgrid, &a->start, &a->goal,
    route_finder_heuristic_bridge, a->max_retry, a->debug_mode_enabled);
}

static route_t* route_finder_run_ida_star(route_finder_t* a){
    return find_ida_star(a->navgrid, &a->start, &a->goal,
    route_finder_cost_bridge, route_finder_heuristic_bridge,
    a->max_retry, a->debug_mode_enabled);
}

static route_t* route_finder_run_rta_star(route_finder_t* a) {
    int depth_limit = 5;

    if (a->typedata) {
        int v = static_cast<const route_finder_rta_star_config_t*>(
            a->typedata)->depth_limit;
        if (v >= 1 && v <= 100) {
            depth_limit = v;
        }
    }

    return find_rta_star(a->navgrid, &a->start, &a->goal,
        route_finder_cost_bridge, route_finder_heuristic_bridge, depth_limit,
        a->max_retry, a->debug_mode_enabled);
}

static route_t* route_finder_run_sma_star(route_finder_t* a) {
    if (!a || !a->navgrid) return NULL;

    int memory_limit = 0;

    if (a->typedata) {
        int val = static_cast<const route_finder_sma_star_config_t*>(
            a->typedata)->memory_limit;

        if (val >= 10 && val <= 1000000) {
            memory_limit = val;
        }
    }

    if (memory_limit <= 0) {
        int w = navgrid_get_width(a->navgrid);
        int h = navgrid_get_height(a->navgrid);
        int n = w * h;

        // Recommended default value: a = 0.02
        memory_limit = (int)(n * 0.02f);
        if (memory_limit < 20) memory_limit = 20;
    }

    return find_sma_star(a->navgrid, &a->start, &a->goal,
        route_finder_cost_bridge, route_finder_heuristic_bridge, memory_limit,
        a->max_retry, a->debug_mode_enabled);
}

static route_t* route_finder_run_weighted_astar(route_finder_t* a) {
    float weight = 1.5f;

    if (a->typedata) {
        float v = static_cast<const route_finder_weighted_astar_config_t*>(
            a->typedata)->weight;
        if (v >= 0.1f && v <= 10.0f) {
            weight = v;
        }
    }

    return find_weighted_astar(a->navgrid, &a->start, &a->goal,
        route_finder_cost_bridge, route_finder_heuristic_bridge, weight,
        a->max_retry, a->debug_mode_enabled);
}

static route_t* route_finder_run_fast_marching(route_finder_t* a){
    return find_fast_marching(a->navgrid, &a->start, &a->goal,
    route_finder_cost_bridge, a->max_retry, a->debug_mode_enabled);
}

using route_finder_run_func = route_t* (*)(route_finder_t*);

static route_finder_run_func route_finder_get_legacy_run_func(
    const route_finder_type_t* type) {
    std::underlying_type_t<route_finder_type_t> type_value{};
    std::memcpy(&type_value, type, sizeof(type_value));
    switch (type_value) {
        case ROUTE_FINDER_ASTAR:
            return route_finder_run_astar;
        case ROUTE_FINDER_BFS:
            return route_finder_run_bfs;
        case ROUTE_FINDER_DFS:
            return route_finder_run_dfs;
        case ROUTE_FINDER_DIJKSTRA:
            return route_finder_run_dijkstra;
        case ROUTE_FINDER_FAST_MARCHING:
            return route_finder_run_fast_marching;
        case ROUTE_FINDER_FRINGE_SEARCH:
            return route_finder_run_fringe_search;
        case ROUTE_FINDER_GREEDY_BEST_FIRST:
            return route_finder_run_greedy_best_first;
        case ROUTE_FINDER_IDA_STAR:
            return route_finder_run_ida_star;
        case ROUTE_FINDER_RTA_STAR:
            return route_finder_run_rta_star;
        case ROUTE_FINDER_SMA_STAR:
            return route_finder_run_sma_star;
        case ROUTE_FINDER_WEIGHTED_ASTAR:
            return route_finder_run_weighted_astar;
        default:
            return nullptr;
    }
}

static route_finder_run_func route_finder_get_run_func(
    const route_finder_type_t* type) {
    std::underlying_type_t<route_finder_type_t> type_value{};
    std::memcpy(&type_value, type, sizeof(type_value));
    switch (type_value) {
        case ROUTE_FINDER_ASTAR:
            return route_finder_run_astar;
        case ROUTE_FINDER_BFS:
            return route_finder_run_bfs;
        case ROUTE_FINDER_DFS:
            return route_finder_run_dfs;
        case ROUTE_FINDER_DIJKSTRA:
            return route_finder_run_dijkstra;
        case ROUTE_FINDER_GREEDY_BEST_FIRST:
            return route_finder_run_greedy_best_first;
        case ROUTE_FINDER_WEIGHTED_ASTAR:
            return route_finder_run_weighted_astar;
        default:
            return nullptr;
    }
}

bool route_finder_is_supported(route_finder_type_t type) {
    return route_finder_get_run_func(&type) != nullptr;
}

static navsys_status_t route_finder_update_route_cost(route_t* route) {
    if (!route) return NAVSYS_STATUS_OK;
    const size_t count = route_get_coord_count(route);
    double total = 0.0;
    coord_t from = {};
    if (count > 0 && route_fetch_coord(route, 0, &from) != NAVSYS_STATUS_OK)
        return NAVSYS_STATUS_CORRUPT_STATE;
    for (size_t index = 1; index < count; ++index) {
        coord_t to = {};
        if (route_fetch_coord(route, index, &to) != NAVSYS_STATUS_OK)
            return NAVSYS_STATUS_CORRUPT_STATE;
        total += route_finder_cost_bridge(
            active_callback_finder->navgrid, &from, &to, nullptr);
        if (active_evaluation_status != NAVSYS_STATUS_OK ||
            !std::isfinite(total))
            return NAVSYS_STATUS_CALLBACK_FAILED;
        from = to;
    }
    route_set_cost(route, static_cast<float>(total));
    return NAVSYS_STATUS_OK;
}

bool route_finder_is_type_supported(route_finder_type_t type) {
    return route_finder_is_supported(type);
}

navsys_status_t route_finder_run_with_options(
    route_finder_t* finder,
    const route_finder_run_options_t* options,
    route_t** out_route,
    route_finder_run_stats_t* out_stats) {
    if (!finder || !out_route || !out_stats || !route_finder_is_valid(finder)
        || finder->max_retry <= 0)
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    if (options && options->struct_size < sizeof(route_finder_run_options_t))
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    if (active_callback_finder == finder)
        return NAVSYS_STATUS_IN_PROGRESS;

    route_finder_run_func run = route_finder_get_run_func(&finder->type);
    if (!run) return NAVSYS_STATUS_UNSUPPORTED;

    route_finder_cancel_context cancel_context = {
        options ? options->cancel_func : nullptr,
        options ? options->cancel_userdata : nullptr,
        false,
        false
    };
    route_finder_callback_scope callback_scope(finder);
    route_finder_cancel_scope cancel_scope(&cancel_context);
    route_t* route = run(finder);
    const navsys_status_t cost_status = route_finder_update_route_cost(route);
    if (callback_scope.evaluation_status() != NAVSYS_STATUS_OK) {
        if (route) route_destroy(route);
        return NAVSYS_STATUS_CALLBACK_FAILED;
    }
    if (cost_status != NAVSYS_STATUS_OK) {
        if (route) route_destroy(route);
        return cost_status;
    }
    if (cancel_context.failed) {
        if (route) route_destroy(route);
        return NAVSYS_STATUS_CALLBACK_FAILED;
    }
    if (!route) return NAVSYS_STATUS_OUT_OF_MEMORY;

    route_finder_run_stats_t stats = {};
    stats.total_retry_count = route_get_total_retry_count(route);
    stats.route_length = route_length(route);
    stats.route_cost = route_get_cost(route);
    stats.complete = route_get_success(route) != 0;
    stats.partial = !stats.complete && stats.route_length > 0;

    navsys_status_t status = NAVSYS_STATUS_OK;
    if (cancel_context.requested) {
        status = NAVSYS_STATUS_CANCELLED;
    } else if (!stats.complete) {
        status = stats.total_retry_count >= finder->max_retry
            ? NAVSYS_STATUS_LIMIT_REACHED
            : NAVSYS_STATUS_NO_PATH;
    }

    *out_route = route;
    *out_stats = stats;
    return status;
}

navsys_status_t route_finder_run_ex(
    route_finder_t* finder,
    route_t** out_route,
    route_finder_run_stats_t* out_stats) {
    return route_finder_run_with_options(
        finder, nullptr, out_route, out_stats);
}

route_t* route_finder_run(route_finder_t* finder) {
    if (!finder || !route_finder_is_valid(finder) || finder->max_retry <= 0 ||
        active_callback_finder == finder)
        return nullptr;
    route_finder_run_func run =
        route_finder_get_legacy_run_func(&finder->type);
    if (!run) return nullptr;

    route_finder_cancel_context cancel_context = {
        nullptr, nullptr, false, false
    };
    route_finder_callback_scope callback_scope(finder);
    route_finder_cancel_scope cancel_scope(&cancel_context);
    route_t* route = run(finder);
    const navsys_status_t cost_status = route_finder_update_route_cost(route);
    if (callback_scope.evaluation_status() != NAVSYS_STATUS_OK ||
        cost_status != NAVSYS_STATUS_OK) {
        if (route) route_destroy(route);
        return nullptr;
    }
    return route;
}
