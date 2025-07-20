#include "internal/greedy_best_first.h"
#include "internal/navgrid.h"
#include "internal/coord.hpp"
#include "internal/coord_list.h"
#include "internal/coord_hash.h"
#include "internal/cost_coord_pq.h"
#include "internal/route.h"
#include "internal/console.h"
#include <float.h>
#include <cmath>

route_t* find_greedy_best_first(const navgrid_t* m,
    const coord_t* start, const coord_t* goal,
    heuristic_func heuristic_fn,
    int max_retry, bool visited_logging) {

    if (!m || !start || !goal) return nullptr;

    if(!heuristic_fn) heuristic_fn = default_heuristic;

    coord_hash_t* came_from = coord_hash_create_full(
        (coord_hash_copy_func) coord_copy,
        (coord_hash_destroy_func) coord_destroy
    );

    coord_hash_t* visited = coord_hash_create();
    cost_coord_pq_t* frontier = cost_coord_pq_create();
    route_t* result = route_create();

    float h_start = heuristic_fn(start, goal, nullptr);

    cost_coord_pq_push(frontier, h_start, start);

    int* new_int = new int(1);
    coord_hash_replace(visited, start, new_int);
    delete new_int;

    if (visited_logging)
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
            coord_destroy(current);
            break;
        }

        coord_list_t* neighbors = navgrid_clone_adjacent(m, current->x, current->y);
        int len = coord_list_length(neighbors);

        for (int i = 0; i < len; ++i) {
            const coord_t* next = coord_list_get(neighbors, i);
            if (coord_hash_contains(visited, next)) continue;

            float h = heuristic_fn(next, goal, nullptr);
            float f = h; // g값 무시, h만 사용

            cost_coord_pq_push(frontier, f, next);
            coord_hash_replace(came_from, next, current);

            int* new_int = new int(1);
            coord_hash_replace(visited, next, new_int);
            delete new_int;

            if (visited_logging)
                route_add_visited(result, next);
        }

        coord_list_destroy(neighbors);

        if (final) coord_destroy(final);
        final = coord_copy(current);        

        coord_destroy(current);
    }

    if (route_reconstruct_path(result, came_from, start, final)) {
        route_set_success(result, found);
    } else {
        route_set_success(result, false);
    }
    if (final) coord_destroy(final);

    cost_coord_pq_destroy(frontier);
    coord_hash_destroy(came_from);
    coord_hash_destroy(visited);
    route_set_total_retry_count(result, retry);
    return result;
}
