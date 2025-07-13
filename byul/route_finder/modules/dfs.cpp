#include "internal/dfs.h"
#include "internal/map.h"
#include "internal/coord.h"
#include "internal/coord_list.h"
#include "internal/coord_hash.h"
#include "internal/route.h"
#include <stdint.h>

route_t* find_dfs(const map_t* m, const coord_t* start, const coord_t* goal, 
    int max_retry, bool visited_logging) {

    if (!m || !start || !goal || max_retry <= 0) return NULL;

    coord_list_t* frontier = coord_list_new();  // 스택
    coord_hash_t* visited = coord_hash_new();
    coord_hash_t* came_from = coord_hash_new_full(
        (coord_hash_copy_func) coord_copy,
        (coord_hash_free_func) coord_free);
    
    route_t* result = route_new();

    coord_list_insert(frontier, 0, start);

    int* new_int = new int(1);
    coord_hash_replace(visited, start, new_int);
    delete new_int;

    if (visited_logging) route_add_visited(result, start);

    bool found = false;
    coord_t* final = NULL;
    int retry = 0;

    while (!coord_list_empty(frontier) && retry++ < max_retry) {
        coord_t* current = coord_list_pop_front(frontier);  // 스택: pop_head

        if (coord_equal(current, goal)) {
            found = true;
            if(final) coord_free(final);
            final = coord_copy(current);
            coord_free(current);
            break;
        }

        coord_list_t* neighbors = map_make_neighbors(m, current->x, current->y);
        int len = coord_list_length(neighbors);
        for (int i = 0; i < len; ++i) {
            const coord_t* next = coord_list_get(neighbors, i);

            if (!coord_hash_contains(visited, next)) {
                coord_list_insert(frontier, 0, next); // push_head

                int* new_int = new int(1);
                coord_hash_replace(visited, next, new_int);
                delete new_int;

                coord_hash_replace(came_from, next, current);

                if (visited_logging) route_add_visited(result, next);                
            }
        }

        coord_list_free(neighbors);
        // if (!final) final = current); // 최종 시도한 노드 기록
        if (final) coord_free(final);
        final = coord_copy(current);
        coord_free(current);
    }

    // if (final) {
        if (route_reconstruct_path(result, came_from, start, final)) {
            route_set_success(result, found);
        } else {
            route_set_success(result, false);
        }
        coord_free(final);
    // } else {
    //     route_set_success(result, false);
    // }

    coord_list_free(frontier);
    coord_hash_free(visited);
    coord_hash_free(came_from);

    route_set_total_retry_count(result, retry);
    return result;
}
