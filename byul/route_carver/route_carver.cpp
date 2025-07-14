#include "internal/route_carver.h"
#include <cmath>
#include <algorithm>

int route_carve_beam(map_t* map, 
    const coord_t* start, const coord_t* goal, int range){
    if (!map || !start || !goal) return 0;

    int removed = 0;

    coord_t* cur = coord_copy(start);

    if(range <= 0){
        while(!coord_equal(cur, goal)){
            // 해당 좌표만 제거한다.
            coord_t* next = coord_clone_next_to_goal(cur, goal);

            if (is_coord_blocked_map(map, next->x, next->y, nullptr)){
                map_unblock_coord(map, next->x, next->y);
                removed++;
            }
            coord_set(cur, next->x, next->y);

            coord_free(next);
        }
        coord_free(cur);
        return removed;        
    }

    // while(cur != goal){
    while(!coord_equal(cur, goal)){
        coord_t* next = coord_clone_next_to_goal(cur, goal);
        coord_list_t* neighbors = map_clone_neighbors_all_range(
            map, next->x, next->y, range-1);
        for(int i=0; i < coord_list_length(neighbors); i++){
            const coord_t* c = coord_list_get(neighbors, i);
            if (is_coord_blocked_map(map, c->x, c->y, nullptr)){
                map_unblock_coord(map, c->x, c->y);
                removed++;
            }
        }
        coord_set(cur, next->x, next->y);
        coord_list_free(neighbors);
        coord_free(next);
    }
    coord_free(cur);
    return removed;
}

int route_carve_bomb(map_t* map, const coord_t* center, int range){
    if (!map || !center) return 0;

    int removed = 0;

    if (range <= 0){
        // 해당 좌표만 제거한다.
        if (is_coord_blocked_map(map, center->x, center->y, nullptr)){
            map_unblock_coord(map, center->x, center->y);
            removed++;
        }
        return removed;
    }

    // 해당 좌표를 제거한다.
    if (is_coord_blocked_map(map, center->x, center->y, nullptr)){
        map_unblock_coord(map, center->x, center->y);
        removed++;
    }    

    // 주변 좌표를 제거한다.
    coord_list_t* neighbors = map_clone_neighbors_all_range(
        map, center->x, center->y, range-1);

    for(int i=0; i < coord_list_length(neighbors); i++){
        const coord_t* c = coord_list_get(neighbors, i);
        if (is_coord_blocked_map(map, c->x, c->y, nullptr)){
            map_unblock_coord(map, c->x, c->y);
            removed++;
        }
    }
    coord_list_free(neighbors);
    return removed;
}
