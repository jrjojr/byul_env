#include "route_carver.h"
#include <cmath>
#include <algorithm>

int route_carve_beam(navgrid_t* navgrid, 
    const coord_t* start, const coord_t* goal, int range){
    if (!navgrid || !start || !goal) return 0;

    int removed = 0;

    coord_t* cur = coord_copy(start);

    if(range <= 0){
        while(!coord_equal(cur, goal)){
            // 해당 좌표만 제거한다.
            coord_t* next = coord_clone_next_to_goal(cur, goal);

            if (is_coord_blocked_navgrid(navgrid, next->x, next->y, nullptr)){
                navgrid_unblock_coord(navgrid, next->x, next->y);
                removed++;
            }
            coord_set(cur, next->x, next->y);

            coord_destroy(next);
        }
        coord_destroy(cur);
        return removed;        
    }

    // while(cur != goal){
    while(!coord_equal(cur, goal)){
        coord_t* next = coord_clone_next_to_goal(cur, goal);
        coord_list_t* neighbors = navgrid_clone_adjacent_all_range(
            navgrid, next->x, next->y, range-1);
        for(int i=0; i < coord_list_length(neighbors); i++){
            const coord_t* c = coord_list_get(neighbors, i);
            if (is_coord_blocked_navgrid(navgrid, c->x, c->y, nullptr)){
                navgrid_unblock_coord(navgrid, c->x, c->y);
                removed++;
            }
        }
        coord_set(cur, next->x, next->y);
        coord_list_destroy(neighbors);
        coord_destroy(next);
    }
    coord_destroy(cur);
    return removed;
}

int route_carve_bomb(navgrid_t* navgrid, const coord_t* center, int range){
    if (!navgrid || !center) return 0;

    int removed = 0;

    if (range <= 0){
        // 해당 좌표만 제거한다.
        if (is_coord_blocked_navgrid(navgrid, center->x, center->y, nullptr)){
            navgrid_unblock_coord(navgrid, center->x, center->y);
            removed++;
        }
        return removed;
    }

    // 해당 좌표를 제거한다.
    if (is_coord_blocked_navgrid(navgrid, center->x, center->y, nullptr)){
        navgrid_unblock_coord(navgrid, center->x, center->y);
        removed++;
    }    

    // 주변 좌표를 제거한다.
    coord_list_t* neighbors = navgrid_clone_adjacent_all_range(
        navgrid, center->x, center->y, range-1);

    for(int i=0; i < coord_list_length(neighbors); i++){
        const coord_t* c = coord_list_get(neighbors, i);
        if (is_coord_blocked_navgrid(navgrid, c->x, c->y, nullptr)){
            navgrid_unblock_coord(navgrid, c->x, c->y);
            removed++;
        }
    }
    coord_list_destroy(neighbors);
    return removed;
}
