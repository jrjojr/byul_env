#include "internal/astar.h"
#include "internal/map.h"
#include "coord.hpp"
#include "internal/coord_list.h"
#include "internal/coord_hash.h"
#include "internal/cost_coord_pq.h"
#include "internal/route.h"
#include <cmath>

route_t* find_astar(const map_t* m, const coord_t* start, const coord_t* goal,
    cost_func cost_fn, heuristic_func heuristic_fn,
    int max_retry, bool visited_logging) {

    if (!m || !start || !goal || max_retry <= 0) return nullptr;

    if(!cost_fn) cost_fn = default_cost;
    if(!heuristic_fn) heuristic_fn = default_heuristic;

    cost_coord_pq_t* pq = cost_coord_pq_new();
    coord_hash_t* cost_so_far = coord_hash_new_full(
        (coord_hash_copy_func) float_copy,
        (coord_hash_free_func) float_free
    );   // coord_t* → float*
    
    // coord_t* → coord_t*
    coord_hash_t* came_from = coord_hash_new_full(
        (coord_hash_copy_func) coord_copy, 
        (coord_hash_free_func) coord_free
    );

    route_t* result = route_new();

    float* zero = new float(0.0f);
    coord_hash_replace(cost_so_far, start, zero);
    delete zero;

    float h_start = heuristic_fn(start, goal, nullptr);
    cost_coord_pq_push(pq, h_start, start);

    if (visited_logging)
        route_add_visited(result, start);

    bool found = false;
    coord_t* final = nullptr;
    int retry = 0;

    while (!cost_coord_pq_is_empty(pq) && retry++ < max_retry) {
        coord_t* current = cost_coord_pq_pop(pq);

        if (coord_equal(current, goal)) {
            found = true;
            if (final) coord_free(final);
            final = coord_copy(current);
            delete current;
            break;
        }

        float* current_cost_ptr = (float*)coord_hash_get(cost_so_far, current);
        float current_cost = current_cost_ptr ? *current_cost_ptr : 0.0f;

        coord_list_t* neighbors = map_make_neighbors(m, current->x, current->y);
        int len = coord_list_length(neighbors);

        for (int i = 0; i < len; ++i) {
            const coord_t* next = coord_list_get(neighbors, i);
            float move_cost = cost_fn(m, current, next, nullptr);
            float new_cost = current_cost + move_cost;

            float* known_cost = (float*)coord_hash_get(cost_so_far, next);
            if (!known_cost || new_cost < *known_cost) {
                float* new_cost_ptr = new float(new_cost);
                coord_hash_replace(cost_so_far, next, new_cost_ptr);
                delete new_cost_ptr;

                float h = heuristic_fn(next, goal, nullptr);
                float f = new_cost + h;
                cost_coord_pq_push(pq, f, next);

                coord_hash_replace(came_from, next, current);
                if (visited_logging)
                    route_add_visited(result, next);
            }
        }

        coord_list_free(neighbors);
        // if (!final) final = coord_copy(current);  // 마지막 방문 지점 저장
        if (final) coord_free(final);
        final = coord_copy(current);        
        delete current;
    }

    // if (final) {
        if (route_reconstruct_path(result, came_from, start, final)) {
            route_set_success(result, found);
        } else {
            route_set_success(result, false);
        }
        delete final;
    // } else {
    //     route_reconstruct_path(result, came_from, start, final);
    //     route_set_success(result, false);
    // }

    cost_coord_pq_free(pq);

    // 수동 해제: float* 값들
    // coord_list_t* keys = coord_hash_keys(cost_so_far);
    // int n = coord_list_length(keys);
    // for (int i = 0; i < n; ++i) {
    //     const coord_t* key = coord_list_get(keys, i);
    //     float* val = (float*)coord_hash_get(cost_so_far, key);
    //     delete val;
    // }
    // coord_list_free(keys);

    coord_hash_free(cost_so_far);
    coord_hash_free(came_from);

    route_set_total_retry_count(result, retry);    
    return result;
}
