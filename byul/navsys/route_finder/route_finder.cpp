// route_finder.cpp

#include "route_finder.h"
#include "cost_coord_pq.h"
#include "coord_list.h"
#include "coord.hpp"

#include <cmath>
#include <vector>
#include <limits>
#include <cstring>

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

const char* get_route_finder_name(route_finder_type_t pa) {
    switch (pa) {
        case ROUTE_FINDER_BFS: return "bfs";
        case ROUTE_FINDER_DFS: return "dfs";
        case ROUTE_FINDER_DIJKSTRA: return "dijkstra";
        case ROUTE_FINDER_ASTAR: return "astar";
        case ROUTE_FINDER_WEIGHTED_ASTAR: return "weighted_astar";
        case ROUTE_FINDER_GREEDY_BEST_FIRST: return "greedy_best_first";
        case ROUTE_FINDER_IDA_STAR: return "ida_star";
        case ROUTE_FINDER_RTA_STAR: return "rta_star";
        case ROUTE_FINDER_SMA_STAR: return "sma_star";
        case ROUTE_FINDER_FAST_MARCHING: return "fast_marching";
        case ROUTE_FINDER_FRINGE_SEARCH: return "fringe_search";
        case ROUTE_FINDER_DSTAR_LITE: return "dstar_lite";
        case ROUTE_FINDER_DSTAR: return "dynamic_astar";
        case ROUTE_FINDER_LPA_STAR: return "lpa_star";
        case ROUTE_FINDER_HPA_STAR: return "hpa_star";
        case ROUTE_FINDER_ANY_ANGLE_ASTAR: return "any_angle_astar";
        case ROUTE_FINDER_ALT: return "alt";
        case ROUTE_FINDER_THETA_STAR: return "theta_star";
        case ROUTE_FINDER_LAZY_THETA_STAR: return "lazy_theta_star";
        case ROUTE_FINDER_JUMP_POINT_SEARCH: return "jump_point_search";
        case ROUTE_FINDER_JPS_PLUS: return "jps_plus";
        case ROUTE_FINDER_BIDIRECTIONAL_ASTAR: return "bidirectional_astar";
        default: return "unknown";
    }
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

    route_finder_t* a = new route_finder_t{};

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

    if(!out || !navgrid) return -1;
    
    out->type = type;
    out->navgrid = navgrid;
    out->start = *start;
    out->goal = *goal;
    out->cost_fn = cost_fn;
    out->heuristic_fn = heuristic_fn;
    out->max_retry = max_retry;
    out->debug_mode_enabled = debug_mode_enabled;
    out->typedata = typedata;

    return 0;
}

int route_finder_free(route_finder_t* out){
    if(!out) return -1;

    memset(out, 0, sizeof(route_finder_t));
    return 0;
}


int route_finder_destroy(route_finder_t* a) {
    if(!a) return -1;
    delete a;
    return 0;
}

route_finder_t* route_finder_copy(const route_finder_t* src) {
    if (!src) return nullptr;
    return route_finder_create_full(
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
}

void route_finder_clear(route_finder_t* a) {
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

void route_finder_set_typedata(route_finder_t* a, void* typedata){
    a->typedata = typedata;
}

void* route_finder_get_typedata(const route_finder_t* a){
    return a->typedata;
}

void route_finder_set_max_retry(route_finder_t* a, int max_retry){
    a->max_retry;
}

int route_finder_get_max_retry(route_finder_t* a){
    return a->max_retry;
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

static route_t* route_finder_run_astar(route_finder_t* a){
    return find_astar(a->navgrid, &a->start, &a->goal, 
        a->cost_fn, a->heuristic_fn, a->max_retry, a->debug_mode_enabled);
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
    return find_dijkstra(a->navgrid, &a->start, &a->goal, a->cost_fn,
        a->max_retry, a->debug_mode_enabled);
}

static route_t* route_finder_run_fringe_search(route_finder_t* a) {
    float delta_epsilon = 0.3f;

    if (a->typedata) {
        float v = *(float*)a->typedata;
        if (v >= 0.001f && v <= 5.0f) {
            delta_epsilon = v;
        }
    }

    return find_fringe_search(a->navgrid, &a->start, &a->goal,
        a->cost_fn, a->heuristic_fn, delta_epsilon,
        a->max_retry, a->debug_mode_enabled);
}

static route_t* route_finder_run_greedy_best_first(route_finder_t* a){
    return find_greedy_best_first(a->navgrid, &a->start, &a->goal,
    a->heuristic_fn, a->max_retry, a->debug_mode_enabled);
}

static route_t* route_finder_run_ida_star(route_finder_t* a){
    // For IDA*, the heuristic should be set to 
    // Manhattan distance instead of Euclidean.

    route_finder_set_heuristic_func(a, manhattan_heuristic);
    return find_ida_star(a->navgrid, &a->start, &a->goal,
    a->cost_fn, a->heuristic_fn, a->max_retry, a->debug_mode_enabled);
}

static route_t* route_finder_run_rta_star(route_finder_t* a) {
    int depth_limit = 5;

    if (a->typedata) {
        int v = *(int*)a->typedata;
        if (v >= 1 && v <= 100) {
            depth_limit = v;
        }
    }

    return find_rta_star(a->navgrid, &a->start, &a->goal,
        a->cost_fn, a->heuristic_fn, depth_limit,
        a->max_retry, a->debug_mode_enabled);
}

static route_t* route_finder_run_sma_star(route_finder_t* a) {
    if (!a || !a->navgrid) return NULL;

    int memory_limit = 0;

    if (a->typedata) {
        int val = *(int*)a->typedata;

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
        a->cost_fn, a->heuristic_fn, memory_limit,
        a->max_retry, a->debug_mode_enabled);
}

static route_t* route_finder_run_weighted_astar(route_finder_t* a) {
    float weight = 1.5f;

    if (a->typedata) {
        float v = *(float*)a->typedata;
        if (v >= 0.1f && v <= 10.0f) {
            weight = v;
        }
    }

    return find_weighted_astar(a->navgrid, &a->start, &a->goal,
        a->cost_fn, a->heuristic_fn, weight,
        a->max_retry, a->debug_mode_enabled);
}

static route_t* route_finder_run_fast_marching(route_finder_t* a){
    return find_fast_marching(a->navgrid, &a->start, &a->goal,
    a->cost_fn, a->max_retry, a->debug_mode_enabled);
}

route_t* route_finder_run(route_finder_t* a) {
    if (!a) return NULL;
    switch (a->type) {
        case ROUTE_FINDER_ASTAR: 
            return route_finder_run_astar(a);
        case ROUTE_FINDER_BFS: 
            return route_finder_run_bfs(a);
        case ROUTE_FINDER_DFS: 
            return route_finder_run_dfs(a);
        case ROUTE_FINDER_DIJKSTRA: 
            return route_finder_run_dijkstra(a);
        case ROUTE_FINDER_FAST_MARCHING: 
            return route_finder_run_fast_marching(a);
        case ROUTE_FINDER_FRINGE_SEARCH: 
            return route_finder_run_fringe_search(a);
        case ROUTE_FINDER_GREEDY_BEST_FIRST: 
            return route_finder_run_greedy_best_first(a);
        case ROUTE_FINDER_IDA_STAR: 
            return route_finder_run_ida_star(a);
        case ROUTE_FINDER_RTA_STAR: 
            return route_finder_run_rta_star(a);
        case ROUTE_FINDER_SMA_STAR: 
            return route_finder_run_sma_star(a);
        case ROUTE_FINDER_WEIGHTED_ASTAR: 
            return route_finder_run_weighted_astar(a);
        default: return NULL;
    }
}

