#include "bfs.h"
#include "navgrid.h"
#include "coord.h"
#include "coord_list.h"
#include "coord_hash.h"
#include "route.h"
#include <stdint.h>

route_t* find_bfs(const navgrid_t* m, const coord_t* start, const coord_t* goal, 
    int max_retry, bool visited_logging) {

    if (!m || !start || !goal || max_retry <= 0) return nullptr;

    coord_list_t* frontier = coord_list_create();  // 큐
    coord_hash_t* visited = coord_hash_create();
    coord_hash_t* came_from = coord_hash_create_full(
        (coord_hash_copy_func)coord_copy, 
        (coord_hash_destroy_func) coord_destroy);
    route_t* result = route_create_full(0.0f);

    coord_list_push_back(frontier, start);

    int* new_int = new int(1);
    coord_hash_replace(visited, start, new_int);
    delete new_int;

    if (visited_logging) route_add_visited(result, start);

    bool found = false;
    coord_t* final = nullptr;
    int retry = 0;

    while (!coord_list_empty(frontier) && retry++ < max_retry) {
        coord_t* current = coord_list_pop_front(frontier);  // 큐: pop_front

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

            if (!coord_hash_contains(visited, next)) {
                coord_list_push_back(frontier, next); // push_back

                int* new_int = new int(1);
                coord_hash_replace(visited, next, new_int);
                delete new_int;

                coord_hash_replace(came_from, next, current);
                if (visited_logging) route_add_visited(result, next);
            }
        }

        coord_list_destroy(neighbors);
        // if (!final) final = coord_copy(current); // 최종 시도한 노드 기록
        if (final) coord_destroy(final);
        final = coord_copy(current);
        coord_destroy(current);
    }

    if (route_reconstruct_path(result, came_from, start, final)) {
        route_set_success(result, found);
    } else {
        route_set_success(result, false);
    }
    coord_destroy(final);
    // } else {
    //     route_set_success(result, false);
    // }

    coord_list_destroy(frontier);
    coord_hash_destroy(visited);
    coord_hash_destroy(came_from);

    route_set_total_retry_count(result, retry);
    return result;
}
