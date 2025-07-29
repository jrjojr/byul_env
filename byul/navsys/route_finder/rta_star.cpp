#include "rta_star.h"
#include "navgrid.h"
#include "coord.hpp"
#include "coord_list.h"
#include "coord_hash.h"
#include "cost_coord_pq.h"
#include "route.h"
// #include "console.h"
#include <float.h>
#include <cmath>

static float rta_iterative_eval(const navgrid_t* m,
    const coord_t* start, const coord_t* goal, int max_depth,
    cost_func cost_fn, heuristic_func heuristic_fn,
    route_t* route, bool visited_logging) {

    if (!cost_fn) cost_fn = default_cost;
    if (!heuristic_fn) heuristic_fn = default_heuristic;
    // coord_t* current = const_cast<coord_t*>(start);
    coord_t* current = coord_copy(start);
    float g = 0.0f;

    for (int d = 0; d < max_depth; ++d) {
        if (coord_equal(current, goal)) break;

        coord_list_t* neighbors = navgrid_clone_adjacent(m, current->x, current->y);
        coord_t* best = nullptr;
        float best_f = FLT_MAX;

        int len = coord_list_length(neighbors);
        for (int i = 0; i < len; ++i) {
            const coord_t* next = coord_list_get(neighbors, i);
            float cost = cost_fn(m, current, next, nullptr);
            float h = heuristic_fn(next, goal, nullptr);
            float f = g + cost + h;

            // if (visited_logging) route_add_visited(route, coord_copy(next));

            if (f < best_f) {
                best_f = f;
                if (best) coord_destroy(best);
                best = coord_copy(next);
                // best = const_cast<coord_t*>(next);
            }
        }

        coord_list_destroy(neighbors);
        if (!best) break;

        if (visited_logging) route_add_visited(route, best);        

        g += cost_fn(m, current, best, nullptr);

        coord_destroy(current);
        current = best;
    }

    float h_final = heuristic_fn(current, goal, nullptr);
    float total_f = g + h_final;
    coord_destroy(current);
    return total_f;
}

route_t* find_rta_star(const navgrid_t* m,
    const coord_t* start, const coord_t* goal,
    cost_func cost_fn, heuristic_func heuristic_fn,
    int depth_limit, int max_retry, bool visited_logging) {

    if (!m || !start || !goal || max_retry <= 0)
        return nullptr;

    if (!cost_fn) cost_fn = default_cost;
    if (!heuristic_fn) heuristic_fn = default_heuristic;

    route_t* result = route_create();
    coord_t* current = coord_copy(start);
    route_add_coord(result, current);

    coord_hash_t* visited = coord_hash_create();

    int* new_int = new int(1);
    coord_hash_replace(visited, current, new_int);
    delete new_int;

    if (visited_logging) route_add_visited(result, current);

    int retry = 0;
    while (!coord_equal(current, goal) && retry++ < max_retry) {
        coord_list_t* neighbors = navgrid_clone_adjacent(m, current->x, current->y);
        coord_t* best = nullptr;
        float best_f = FLT_MAX;

        int len = coord_list_length(neighbors);
        for (int i = 0; i < len; ++i) {
            const coord_t* next = coord_list_get(neighbors, i);
            if (coord_hash_get(visited, next)) continue;

            float eval = rta_iterative_eval(m, next, goal, depth_limit - 1, 
                cost_fn, heuristic_fn, result, visited_logging);

            if (eval < best_f) {
                best_f = eval;
                if (best) coord_destroy(best);
                best = coord_copy(next);
                // best = const_cast<coord_t*>(next);
            }
        }

        coord_list_destroy(neighbors);
        if (!best) break;

        coord_destroy(current);
        current = best;
        route_add_coord(result, current);

        int* new_int = new int(1);
        coord_hash_replace(visited, current, new_int);
        delete new_int;

        if (visited_logging) route_add_visited(result, current);
    }

    if (coord_equal(current, goal)) {
        route_set_success(result, true);
    } else {
        route_set_success(result, false);
    }

    coord_destroy(current);
    coord_hash_destroy(visited);
    route_set_total_retry_count(result, retry);
    return result;
}
