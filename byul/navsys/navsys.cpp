#include <iostream>

#include "navsys.h"

// navgrid, start, goal
route_t* navsys_find_astar(
    navgrid_t* ng, const coord_t* start, const coord_t* goal){

    if (!ng || !goal) return nullptr;

    coord_t a_start = {0, 0};
    if (start) {
        coord_assign(&a_start, start);
    }

    route_finder_t rf;
    route_finder_init_full(&rf, ng, ROUTE_FINDER_ASTAR, 
        &a_start, goal, default_cost, euclidean_heuristic,
        MAX_RETRY, false, nullptr);

    return route_finder_run(&rf);
}

route_t* navsys_find_bfs(
    navgrid_t* ng, const coord_t* start, const coord_t* goal){

    if (!ng || !goal) return nullptr;

    coord_t a_start = {0, 0};
    if (start) {
        coord_assign(&a_start, start);
    }

    route_finder_t rf;
    route_finder_init_full(&rf, ng, ROUTE_FINDER_BFS, 
        &a_start, goal, default_cost, euclidean_heuristic,
        MAX_RETRY, false, nullptr);

    return route_finder_run(&rf);        
}

route_t* navsys_find_dfs(
    navgrid_t* ng, const coord_t* start, const coord_t* goal){

    if (!ng || !goal) return nullptr;

    coord_t a_start = {0, 0};
    if (start) {
        coord_assign(&a_start, start);
    }

    route_finder_t rf;
    route_finder_init_full(&rf, ng, ROUTE_FINDER_DFS, 
        &a_start, goal, default_cost, euclidean_heuristic,
        MAX_RETRY, false, nullptr);

    return route_finder_run(&rf);        
}

route_t* navsys_find_dijkstra(
    navgrid_t* ng, const coord_t* start, const coord_t* goal){

    if (!ng || !goal) return nullptr;

    coord_t a_start = {0, 0};
    if (start) {
        coord_assign(&a_start, start);
    }

    route_finder_t rf;
    route_finder_init_full(&rf, ng, ROUTE_FINDER_DIJKSTRA, 
        &a_start, goal, default_cost, euclidean_heuristic,
        MAX_RETRY, false, nullptr);

    return route_finder_run(&rf);        
}

route_t* navsys_find_greedy_best_first(
    navgrid_t* ng, const coord_t* start, const coord_t* goal){
    if (!ng || !goal) return nullptr;

    coord_t a_start = {0, 0};
    if (start) {
        coord_assign(&a_start, start);
    }

    route_finder_t rf;
    route_finder_init_full(&rf, ng, ROUTE_FINDER_GREEDY_BEST_FIRST, 
        &a_start, goal, default_cost, euclidean_heuristic,
        MAX_RETRY, false, nullptr);

    return route_finder_run(&rf);        
}

route_t* navsys_find_ida_star(
    navgrid_t* ng, const coord_t* start, const coord_t* goal){

    if (!ng || !goal) return nullptr;

    coord_t a_start = {0, 0};
    if (start) {
        coord_assign(&a_start, start);
    }

    route_finder_t rf;
    route_finder_init_full(&rf, ng, ROUTE_FINDER_IDA_STAR, 
        &a_start, goal, default_cost, manhattan_heuristic,
        MAX_RETRY, false, nullptr);

    return route_finder_run(&rf);        
}

route_t* navsys_find_fast_marching(
    navgrid_t* ng, const coord_t* start, const coord_t* goal){

    if (!ng || !goal) return nullptr;

    coord_t a_start = {0, 0};
    if (start) {
        coord_assign(&a_start, start);
    }

    route_finder_t rf;
    route_finder_init_full(&rf, ng, ROUTE_FINDER_FAST_MARCHING, 
        &a_start, goal, default_cost, euclidean_heuristic,
        MAX_RETRY, false, nullptr);

    return route_finder_run(&rf);        
}

route_t* navsys_find_fringe_search(navgrid_t* ng, 
    const coord_t* start, const coord_t* goal, float delta_epsilon) 
{
    if (!ng || !goal) return nullptr;

    coord_t a_start = {0, 0};
    if (start) {
        coord_assign(&a_start, start);
    }

    float* spec = new float{delta_epsilon};

    route_finder_t rf;
    route_finder_init_full(&rf, ng, ROUTE_FINDER_FRINGE_SEARCH, 
        &a_start, goal,
        default_cost, euclidean_heuristic,
        MAX_RETRY, false,
        spec);

    route_t* result = route_finder_run(&rf);

    delete spec;

    return result;
}

/**
 * @brief Runs Real-Time A* (RTA*) algorithm.
 *
 * This algorithm performs limited-depth search for real-time responsiveness.
 *
 * @param a Pointer to route_finder_t containing execution settings.
 *          - userdata should be an int* pointing to depth_limit.
 *          - Recommended: 3 ~ 10 (higher is more accurate but slower).
 *
 * @return Calculated route_t* or NULL on failure.
 */
route_t* navsys_find_rta_star(navgrid_t* ng, 
    const coord_t* start, const coord_t* goal, int depth_limit){
    if (!ng || !goal) return nullptr;

    coord_t a_start = {0, 0};
    if (start) {
        coord_assign(&a_start, start);
    }

    int* spec = new int{depth_limit};

    route_finder_t rf;
    route_finder_init_full(&rf, ng, ROUTE_FINDER_RTA_STAR, 
        &a_start, goal,
        default_cost, euclidean_heuristic,
        MAX_RETRY, false,
        spec);

    route_t* result = route_finder_run(&rf);

    delete spec;

    return result;
}

/**
* Memory limit should depend on map size and complexity. Recommended values:
 *   - memory_limit === max(L × (1 + e), N x a)
 *     (L: expected path length, N: number of map cells)
 *     (e <== [0.5, 1.0], a <== [0.01, 0.05])
 *
 * @par Example memory limits:
 *   - 10x10 map  : memory_limit === 20 ~ 30
 *   - 100x100 map: memory_limit === 500 ~ 1000
 *   - 1000x1000 map: memory_limit === 50,000 ~ 100,000
 *
 */
route_t* navsys_find_sma_star(navgrid_t* ng, 
    const coord_t* start, const coord_t* goal, int memory_limit){
    if (!ng || !goal) return nullptr;

    coord_t a_start = {0, 0};
    if (start) {
        coord_assign(&a_start, start);
    }

    int* spec = new int{memory_limit};

    route_finder_t rf;
    route_finder_init_full(&rf, ng, ROUTE_FINDER_SMA_STAR, 
        &a_start, goal,
        default_cost, euclidean_heuristic,
        MAX_RETRY, false,
        spec);

    route_t* result = route_finder_run(&rf);

    delete spec;
    
    return result;
}

/**
 * @brief Runs the Weighted A* algorithm.
 *
 * This algorithm applies a weight to the heuristic
 * to speed up pathfinding.
 *
 * @param a Pointer to route_finder_t containing execution settings.
 *          - userdata should be a float* pointing to the weight.
 *          - Recommended: 
                * 1.0 (standard A*), 
                * 1.2 ~ 2.5 (faster), 
                * 5.0+ may be inaccurate.
 *
 * @return Calculated route_t* or NULL on failure.
 */
route_t* navsys_find_weighted_astar(navgrid_t* ng, 
    const coord_t* start, const coord_t* goal, float weight){
    if (!ng || !goal) return nullptr;

    coord_t a_start = {0, 0};
    if (start) {
        coord_assign(&a_start, start);
    }

    float* spec = new float{weight};

    route_finder_t rf;
    route_finder_init_full(&rf, ng, ROUTE_FINDER_WEIGHTED_ASTAR, 
        &a_start, goal,
        default_cost, euclidean_heuristic,
        MAX_RETRY, false,
        spec);

    route_t* result = route_finder_run(&rf);

    delete spec;
    
    return result;
}
