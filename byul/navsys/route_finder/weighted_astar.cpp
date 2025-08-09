#include "weighted_astar.h"
#include "navgrid.h"
#include "coord.hpp"
#include "coord_list.h"
#include "coord_hash.h"
#include "cost_coord_pq.h"
#include "route.h"
// #include "console.h"
#include <float.h>
#include <cmath>

route_t* find_weighted_astar(const navgrid_t* m,
    const coord_t* start, const coord_t* goal,
    cost_func cost_fn, heuristic_func heuristic_fn,
    float weight,
    int max_retry, bool debug_mode_enabled) {

    if (!m || !start || !goal || max_retry <= 0)
        return nullptr;

    if (!cost_fn) cost_fn = default_cost;
    if (!heuristic_fn) heuristic_fn = default_heuristic;
    if (weight <= 0.0f) weight = 1.0f;

    route_t* result = route_create();

    cost_coord_pq_t* frontier = cost_coord_pq_create();

    coord_hash_t* cost_so_far = coord_hash_create_full(
        (coord_hash_copy_func) float_copy,
        (coord_hash_destroy_func) float_destroy
    );

    coord_hash_t* came_from = coord_hash_create_full(
        (coord_hash_copy_func) coord_copy,
        (coord_hash_destroy_func) coord_destroy
    );

    float* new_float = new float(0.0);
    coord_hash_replace(cost_so_far, start, new_float);
    delete new_float;

    float h_start = heuristic_fn(start, goal, nullptr);
    float f_start = weight * h_start;
    cost_coord_pq_push(frontier, f_start, start);

    if (debug_mode_enabled)
        route_add_visited(result, start);

    bool found = false;
    coord_t* final = nullptr;
    int retry = 0;

    while (!cost_coord_pq_is_empty(frontier) && retry++ < max_retry) {
        coord_t* current = cost_coord_pq_pop(frontier);

        if (coord_equal(current, goal)) {
            found = true;
            if (final) coord_destroy(final);
            final = coord_copy(current);
            delete current;
            break;
        }

        float* g_ptr = (float*)coord_hash_get(cost_so_far, current);
        float g = g_ptr ? *g_ptr : 0.0f;

        coord_list_t* neighbors = navgrid_copy_neighbors(m, current->x, current->y);
        int len = coord_list_length(neighbors);

        for (int i = 0; i < len; ++i) {
            const coord_t* next = coord_list_get(neighbors, i);
            float move_cost = cost_fn(m, current, next, nullptr);
            float new_g = g + move_cost;

            float* known_g = (float*)coord_hash_get(cost_so_far, next);
            if (!known_g || new_g < *known_g) {

                float* new_float = new float(new_g);
                coord_hash_replace(cost_so_far, next, new_float);
                delete new_float;

                coord_hash_replace(came_from, next, current);

                float h = heuristic_fn(next, goal, nullptr);
                float f = new_g + weight * h;
                cost_coord_pq_push(frontier, f, next);

                if (debug_mode_enabled)
                    route_add_visited(result, next);
            }
        }

        coord_list_destroy(neighbors);

        if (final) coord_destroy(final);
        final = coord_copy(current);                

        delete current;
    }

    // if (final) {
    if (route_reconstruct(result, came_from, start, final)) {
        route_set_success(result, found);

    } else {
        route_set_success(result, false);
    }

    coord_destroy(final);

    coord_hash_destroy(cost_so_far);
    coord_hash_destroy(came_from);
    cost_coord_pq_destroy(frontier);

    route_set_total_retry_count(result, retry);
    return result;
}
