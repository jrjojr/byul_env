// route_finder.cpp

#include "internal/route_finder.h"
#include "internal/cost_coord_pq.h"
#include "internal/coord_list.h"
#include "internal/coord.hpp"

#include <cmath>
#include <vector>
#include <limits>

#include <cstring>

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

route_finder_t* route_finder_new_full(navgrid_t* navgrid, 
    route_finder_type_t type, 
    coord_t* start, coord_t* goal,
    cost_func cost_fn, heuristic_func heuristic_fn,
    int max_retry, bool visited_logging, void* userdata) {

    route_finder_t* a = new route_finder_t;
    a->type = type;
    a->navgrid = navgrid;
    a->start = coord_copy(start);
    a->goal = coord_copy(goal);
    a->cost_fn = cost_fn;
    a->heuristic_fn = heuristic_fn;
    a->max_retry = max_retry;
    a->visited_logging = visited_logging;
    a->userdata = userdata;
    return a;
}

route_finder_t* route_finder_new(navgrid_t* navgrid) {
    coord_t start;
    start.x = 0;
    start.y = 0;
    return route_finder_new_full(navgrid, ROUTE_FINDER_ASTAR, &start, &start, 
        default_cost, euclidean_heuristic, 10000, false, nullptr);
}

void route_finder_free(route_finder_t* a) {
    if(a->start) coord_free(a->start);
    if(a->goal) coord_free(a->goal);
    delete a;
}

route_finder_t* route_finder_copy(const route_finder_t* src) {
    if (!src) return nullptr;
    return route_finder_new_full(
        src->navgrid,
        src->type,
        src->start,
        src->goal,
        src->cost_fn,
        src->heuristic_fn,
        src->max_retry,
        src->visited_logging,
        src->userdata
    );
}

void route_finder_clear(route_finder_t* a) {
    memset(a, 0, sizeof(route_finder_t));
}

void route_finder_set_defaults(route_finder_t* a) {
    a->cost_fn = default_cost;
    a->heuristic_fn = euclidean_heuristic;
    a->max_retry = 10000;
    a->visited_logging = false;
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
    printf("  type:        %s\n", get_route_finder_name(a->type));
    printf("  navgrid:         %p\n", (void*)a->navgrid);
    printf("  start:       (%d, %d)\n", a->start->x, a->start->y);
    printf("  goal:        (%d, %d)\n", a->goal->x, a->goal->y);
    printf("  cost_fn:     %p\n", (void*)a->cost_fn);
    printf("  heuristic_fn:%p\n", (void*)a->heuristic_fn);
    printf("  max_retry:   %d\n", a->max_retry);
    printf("  logging:     %s\n", a->visited_logging ? "true" : "false");
    printf("  userdata:    %p\n", a->userdata);
    printf("}\n");
}

void route_finder_set_navgrid(route_finder_t* a, navgrid_t* navgrid) {
     a->navgrid = navgrid; 
}

void route_finder_set_start(route_finder_t* a, const coord_t* start) { 
    coord_set(a->start, start->x, start->y);
}

void route_finder_set_goal(route_finder_t* a, const coord_t* goal) { 
    coord_set(a->goal, goal->x, goal->y); 
}

navgrid_t* route_finder_get_navgrid(const route_finder_t* a) { 
    return a->navgrid; 
}

coord_t* route_finder_get_start(const route_finder_t* a) { 
    return a->start; 
}

coord_t* route_finder_get_goal(const route_finder_t* a) { 
    return a->goal; 
}

void route_finder_set_userdata(route_finder_t* a, void* userdata){
    a->userdata = userdata;
}

void* route_finder_get_userdata(const route_finder_t* a){
    return a->userdata;
}

void route_finder_set_type(route_finder_t* a, route_finder_type_t type){
    a->type = type;
}

route_finder_type_t route_finder_get_type(const route_finder_t* a){
    return a->type;
}

void route_finder_set_visited_logging(route_finder_t* a, bool is_logging){
    a->visited_logging = is_logging;
}

bool route_finder_is_visited_logging(route_finder_t* a){
    return a->visited_logging;
}

void route_finder_set_cost_func(route_finder_t* a, cost_func cost_fn){
    a->cost_fn = cost_fn;
}

cost_func route_finder_get_cost_func(route_finder_t* a){
    return a->cost_fn;
}

void route_finder_set_heuristic_func(route_finder_t* a, heuristic_func heuristic_fn){
    a->heuristic_fn = heuristic_fn;
}

heuristic_func route_finder_get_heuristic_func(route_finder_t* a){
    return a->heuristic_fn;
}

void route_finder_set_max_retry(route_finder_t* a, int max_retry){
    a->max_retry;
}

int route_finder_get_max_retry(route_finder_t* a){
    return a->max_retry;
}

route_t* route_finder_find_with_type(route_finder_t* a, route_finder_type_t type) {
    if (!a) return NULL;
    switch (type) {
        case ROUTE_FINDER_ASTAR: return route_finder_find_astar(a);
        case ROUTE_FINDER_BFS: return route_finder_find_bfs(a);
        case ROUTE_FINDER_DFS: return route_finder_find_dfs(a);
        case ROUTE_FINDER_DIJKSTRA: return route_finder_find_dijkstra(a);
        case ROUTE_FINDER_FAST_MARCHING: return route_finder_find_fast_marching(a);
        case ROUTE_FINDER_FRINGE_SEARCH: return route_finder_find_fringe_search(a);
        case ROUTE_FINDER_GREEDY_BEST_FIRST: return route_finder_find_greedy_best_first(a);
        case ROUTE_FINDER_IDA_STAR: return route_finder_find_ida_star(a);
        case ROUTE_FINDER_RTA_STAR: return route_finder_find_rta_star(a);
        case ROUTE_FINDER_SMA_STAR: return route_finder_find_sma_star(a);
        case ROUTE_FINDER_WEIGHTED_ASTAR: return route_finder_find_weighted_astar(a);
        default: return NULL;
    }
}

route_t* route_finder_find(route_finder_t* a) {
    if (!a) return NULL;
    return route_finder_find_with_type(a, a->type);
}

route_t* route_finder_find_astar(route_finder_t* a){
    return find_astar(a->navgrid, a->start, a->goal, 
        a->cost_fn, a->heuristic_fn, a->max_retry, a->visited_logging);
}

route_t* route_finder_find_bfs(route_finder_t* a){
    return find_bfs(a->navgrid, a->start, a->goal, 
        a->max_retry, a->visited_logging);
}

route_t* route_finder_find_dfs(route_finder_t* a){
    return find_dfs(a->navgrid, a->start, a->goal, a->max_retry,
        a->visited_logging);
}

route_t* route_finder_find_dijkstra(route_finder_t* a){
    return find_dijkstra(a->navgrid, a->start, a->goal, a->cost_fn,
        a->max_retry, a->visited_logging);
}

route_t* route_finder_find_fringe_search(route_finder_t* a) {
    float delta_epsilon = 0.3f;

    if (a->userdata) {
        float v = *(float*)a->userdata;
        if (v >= 0.001f && v <= 5.0f) {
            delta_epsilon = v;
        }
    }

    return find_fringe_search(a->navgrid, a->start, a->goal,
        a->cost_fn, a->heuristic_fn, delta_epsilon,
        a->max_retry, a->visited_logging);
}

route_t* route_finder_find_greedy_best_first(route_finder_t* a){
    return find_greedy_best_first(a->navgrid, a->start, a->goal,
    a->heuristic_fn, a->max_retry, a->visited_logging);
}

route_t* route_finder_find_ida_star(route_finder_t* a){
    // ida는 유클리드가 아니라 맨하탄으로 휴리스틱을 설정해야 한다.
    route_finder_set_heuristic_func(a, manhattan_heuristic);
    return find_ida_star(a->navgrid, a->start, a->goal,
    a->cost_fn, a->heuristic_fn, a->max_retry, a->visited_logging);
}

route_t* route_finder_find_rta_star(route_finder_t* a) {
    int depth_limit = 5;

    if (a->userdata) {
        int v = *(int*)a->userdata;
        if (v >= 1 && v <= 100) {
            depth_limit = v;
        }
    }

    return find_rta_star(a->navgrid, a->start, a->goal,
        a->cost_fn, a->heuristic_fn, depth_limit,
        a->max_retry, a->visited_logging);
}

route_t* route_finder_find_sma_star(route_finder_t* a) {
    if (!a || !a->navgrid) return NULL;

    int memory_limit = 0;

    if (a->userdata) {
        int val = *(int*)a->userdata;
        // 유효 범위 검사 (적당한 하한선 및 상한선 예시)
        if (val >= 10 && val <= 1000000) {
            memory_limit = val;
        }
    }

    // 비정상적인 값이면 맵 크기로 기본값 계산
    if (memory_limit <= 0) {
        int w = navgrid_get_width(a->navgrid);
        int h = navgrid_get_height(a->navgrid);
        int n = w * h;

        // 기본 권장값: α = 0.02
        memory_limit = (int)(n * 0.02f);
        if (memory_limit < 20) memory_limit = 20; // 최소 한도
    }

    return find_sma_star(a->navgrid, a->start, a->goal,
        a->cost_fn, a->heuristic_fn, memory_limit,
        a->max_retry, a->visited_logging);
}

route_t* route_finder_find_weighted_astar(route_finder_t* a) {
    float weight = 1.5f;

    if (a->userdata) {
        float v = *(float*)a->userdata;
        if (v >= 0.1f && v <= 10.0f) {
            weight = v;
        }
    }

    return find_weighted_astar(a->navgrid, a->start, a->goal,
        a->cost_fn, a->heuristic_fn, weight,
        a->max_retry, a->visited_logging);
}

route_t* route_finder_find_fast_marching(route_finder_t* a){
    return find_fast_marching(a->navgrid, a->start, a->goal,
    a->cost_fn, a->max_retry, a->visited_logging);
}