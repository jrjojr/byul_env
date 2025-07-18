#include "internal/fringe_search.h"
#include "internal/navgrid.h"
#include "internal/coord.hpp"
#include "internal/coord_list.h"
#include "internal/coord_hash.h"
#include "internal/cost_coord_pq.h"
#include "internal/route.h"
#include "internal/console.h"
#include <float.h>
#include <cmath>

route_t* find_fringe_search(const navgrid_t* m,
    const coord_t* start, const coord_t* goal,
    cost_func cost_fn, heuristic_func heuristic_fn, float delta_epsilon,
    int max_retry, bool visited_logging) {

    if (!m || !start || !goal) return nullptr;
    if (!heuristic_fn) heuristic_fn = default_heuristic;
    if (!cost_fn) cost_fn = default_cost;

    route_t* result = route_new();

    float threshold = heuristic_fn(start, goal, nullptr);

    coord_hash_t* cost_so_far = coord_hash_new_full(
        (coord_hash_copy_func) float_copy,
        (coord_hash_free_func) float_free
    );

    coord_hash_t* came_from = coord_hash_new_full(
        (coord_hash_copy_func) coord_copy,
        (coord_hash_free_func) coord_free
    );

    coord_hash_t* visited = coord_hash_new();
    cost_coord_pq_t* frontier = cost_coord_pq_new();

    float g_start = 0.0f;
    float f_start = g_start + threshold;

    float* new_float = new float(g_start);
    coord_hash_replace(cost_so_far, start, new_float);
    delete new_float;

    int* new_int = new int(1);
    coord_hash_replace(visited, start, new_int);
    delete new_int;

    cost_coord_pq_push(frontier, f_start, start);
    if (visited_logging)
        route_add_visited(result, start);

    bool found = false;
    coord_t* final = nullptr;
    int total_retry = 0;

    float delta = (delta_epsilon > 0.0f) ? delta_epsilon : 0.5f;

    cost_coord_pq_t* next_frontier = cost_coord_pq_new();

    while (!cost_coord_pq_is_empty(frontier) && 
        (max_retry <= 0 || total_retry < max_retry)) {

        float next_threshold = FLT_MAX;
        bool expanded = false;

        while (!cost_coord_pq_is_empty(frontier) && 
            (max_retry <= 0 || total_retry < max_retry)) {

            ++total_retry;
            coord_t* current = cost_coord_pq_pop(frontier);
            float* g_ptr = (float*)coord_hash_get(cost_so_far, current);
            float g = g_ptr ? *g_ptr : 0.0f;
            float h = heuristic_fn(current, goal, nullptr);
            float f = g + h;

            if (f > threshold + delta) {
                if (f < next_threshold) next_threshold = f;
                cost_coord_pq_push(next_frontier, f, current);
                coord_free(current);
                continue;
            }

            if (!final || f < threshold + delta) {
                if (final) coord_free(final);
                final = coord_copy(current);
            }

            if (coord_equal(current, goal)) {
                found = true;
                coord_free(current);
                break;
            }

            coord_list_t* neighbors = navgrid_clone_adjacent(m, current->x, current->y);
            int len = coord_list_length(neighbors);
            for (int j = 0; j < len; ++j) {
                const coord_t* next = coord_list_get(neighbors, j);
                float move_cost = cost_fn(m, current, next, nullptr);
                float new_g = g + move_cost;
                float* old_g = (float*)coord_hash_get(cost_so_far, next);

                if (!old_g || new_g < *old_g) {
                    float* new_float = new float(new_g);
                    coord_hash_replace(cost_so_far, next, new_float);
                    delete new_float;

                    coord_hash_replace(came_from, next, current);

                    float new_f = new_g + heuristic_fn(next, goal, nullptr);
                    cost_coord_pq_push(frontier, new_f, next);

                    int* new_int = new int(1);
                    coord_hash_replace(visited, next, new_int);
                    delete new_int;

                    if (visited_logging)
                        route_add_visited(result, next);
                    expanded = true;
                }
            }
            coord_list_free(neighbors);
            coord_free(current);
        }

        cost_coord_pq_free(frontier);
        frontier = next_frontier;
        next_frontier = cost_coord_pq_new();

        if (found || cost_coord_pq_is_empty(frontier) || !expanded)
            break;

        if (next_threshold <= threshold + delta)
            threshold += 1.0f;
        else
            threshold = next_threshold;
    }

    if (final) {
        route_reconstruct_path(result, came_from, start, final);
        route_set_success(result, found);
        coord_free(final);
    } else {
        route_set_success(result, false);
    }

    coord_hash_free(cost_so_far);
    coord_hash_free(came_from);
    coord_hash_free(visited);
    cost_coord_pq_free(frontier);
    cost_coord_pq_free(next_frontier);

    route_set_total_retry_count(result, total_retry);
    return result;
}
