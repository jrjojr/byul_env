#include "ida_star.h"
#include "navgrid.h"
#include "coord.hpp"
#include "coord_list.h"
#include "coord_hash.h"
#include "cost_coord_pq.h"
#include "route.h"
// #include "console.h"
#include <float.h>
#include <cmath>

route_t* find_ida_star(const navgrid_t* m,
    const coord_t* start, const coord_t* goal,
    cost_func cost_fn, heuristic_func heuristic_fn,
    int max_retry, bool debug_mode_enabled) {

    if (!m || !start || !goal) return nullptr;
    if (!heuristic_fn) heuristic_fn = manhattan_heuristic;

    float threshold = heuristic_fn(start, goal, nullptr);

    route_t* result = route_create();
    int retry = 0;

    coord_t* best_coord = nullptr;
    float best_f = FLT_MAX;

    while (true) {
        float next_threshold = FLT_MAX;

        coord_hash_t* cost_so_far = coord_hash_create_full(
            (coord_hash_copy_func) float_copy,
            (coord_hash_destroy_func) float_destroy
        );

        coord_hash_t* came_from = coord_hash_create_full(
            (coord_hash_copy_func) coord_copy,
            (coord_hash_destroy_func) coord_destroy
        );

        coord_hash_t* visited = coord_hash_create();
        cost_coord_pq_t* frontier = cost_coord_pq_create();

        float* new_float = new float(0.0);
        coord_hash_replace(cost_so_far, start, new_float);
        delete new_float;

        int* new_int = new int(1);
        coord_hash_replace(visited, start, new_int);
        delete new_int;

        cost_coord_pq_push(frontier, 0.0f, start);
        if (debug_mode_enabled)
            route_add_visited(result, start);

        bool found = false;
        coord_t* final = nullptr;

        while (!cost_coord_pq_is_empty(frontier) && retry++ < max_retry) {
            coord_t* current = cost_coord_pq_pop(frontier);

            float* g_ptr = (float*)coord_hash_get(cost_so_far, current);
            float g = g_ptr ? *g_ptr : 0.0f;
            float h = heuristic_fn(current, goal, nullptr);
            float f = g + h;

            if (f > threshold) {
                if (f < next_threshold) next_threshold = f;
                coord_destroy(current);
                continue;
            }

            if (f < best_f) {
                best_f = f;
                if (best_coord) coord_destroy(best_coord);
                best_coord = coord_copy(current);
            }

            if (coord_equal(current, goal)) {
                found = true;
                if (final) coord_destroy(final);
                final = coord_copy(current);
                coord_destroy(current);
                break;
            }

            coord_list_t* neighbors = navgrid_copy_neighbors(
                m, current->x, current->y);

            int len = coord_list_length(neighbors);

            for (int i = 0; i < len; ++i) {
                const coord_t* next = coord_list_get(neighbors, i);

                float move_cost = cost_fn ? cost_fn(
                    m, current, next, nullptr) : 1.0f;

                float new_cost = g + move_cost;

                float* prev_cost = (float*)coord_hash_get(cost_so_far, next);
                if (prev_cost && new_cost >= *prev_cost)
                    continue;

                float* new_float = new float(new_cost);
                coord_hash_replace(cost_so_far, next, new_float);
                delete new_float;

                coord_hash_replace(came_from, next, current);
                    
                int* new_int = new int(1);
                coord_hash_replace(visited, next, new_int);
                delete new_int; 

                cost_coord_pq_push(frontier, new_cost, next);

                if (debug_mode_enabled)
                    route_add_visited(result, next);
            }

            coord_list_destroy(neighbors);
            coord_destroy(current);
        }

        cost_coord_pq_destroy(frontier);

        coord_hash_destroy(cost_so_far);
        coord_hash_destroy(visited);

        if (found && final) {

            route_reconstruct(result, came_from, start, final);
            route_set_success(result, true);
            coord_destroy(final);
            coord_hash_destroy(came_from);
            route_set_total_retry_count(result, retry);
            if (best_coord) coord_destroy(best_coord);
            return result;
        } else if (best_coord) {

            route_reconstruct(result, came_from, start, best_coord);
            coord_destroy(best_coord);
            best_coord = nullptr;
        }

        coord_hash_destroy(came_from);

        if (next_threshold == FLT_MAX || retry >= max_retry)
            break;
        threshold = next_threshold;
    }

    route_set_success(result, false);
    route_set_total_retry_count(result, retry);
    return result;
}
