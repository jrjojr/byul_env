#include "internal/obstacle.h"
#include <stdlib.h>
#include <string.h>
#include <time.h>

obstacle_t* obstacle_make_rect_all_blocked(int x0, int y0, int width, int height) {
    if (width <= 0 || height <= 0) return nullptr;

    obstacle_t* obstacle = obstacle_new_full(x0, y0, width, height);
    if (!obstacle) return nullptr;

    for (int dy = 0; dy < height; ++dy) {
        for (int dx = 0; dx < width; ++dx) {
            coord_t c = { x0 + dx, y0 + dy };
            coord_hash_insert(obstacle->blocked, &c, nullptr);
        }
    }
    return obstacle;
}

obstacle_t* obstacle_make_rect_random_blocked(
    int x0, int y0, int width, int height, float ratio) {

    if (width <= 0 || height <= 0 || ratio <= 0.0f) return nullptr;
    if (ratio > 1.0f) ratio = 1.0f;

    obstacle_t* obstacle = obstacle_new_full(x0, y0, width, height);
    if (!obstacle) return nullptr;

    srand((unsigned int)time(nullptr));

    for (int dy = 0; dy < height; ++dy) {
        for (int dx = 0; dx < width; ++dx) {
            if ((float)rand() / RAND_MAX <= ratio) {
                coord_t c = { x0 + dx, y0 + dy };
                coord_hash_insert(obstacle->blocked, &c, nullptr);
            }
        }
    }
    return obstacle;
}

// obstacle_t* obstacle_make_beam(
//     const coord_t* start, const coord_t* goal, int range){

//     if (!start || !goal) return nullptr;

//     int width = goal->x - start->x;
//     int height = goal->y - start->y;
//     obstacle_t* obstacle = obstacle_new_full(
//         start->x, start->y, width, height);

//     coord_t* cur = coord_copy(start);

//     if(range <= 0){
//         while(!coord_equal(cur, goal)){
//             // 해당 좌표만 블락한다.
//             double deg = coord_degree(cur, goal);
//             coord_t* next = obstacle_clone_neighbor_at_degree(
//                 obstacle, cur->x, cur->y, deg);

//             if (!obstacle_is_coord_blocked(obstacle, next->x, next->y)){
//                 obstacle_block_coord(obstacle, next->x, next->y);
//             }
//             coord_set(cur, next->x, next->y);

//             coord_free(next);
//         }
//         coord_free(cur);
//         return obstacle;        
//     }

//     // while(cur != goal){
//     while(!coord_equal(cur, goal)){
//         double deg = coord_degree(cur, goal);
//         coord_t* next = obstacle_clone_neighbor_at_degree(
//             obstacle, cur->x, cur->y, deg);

//         coord_list_t* neighbors = obstacle_clone_neighbors_all_range(
//             obstacle, next->x, next->y, range-1);
//         for(int i=0; i < coord_list_length(neighbors); i++){
//             const coord_t* c = coord_list_get(neighbors, i);
//             if (!obstacle_is_coord_blocked(obstacle, c->x, c->y)){
//                 obstacle_block_coord(obstacle, c->x, c->y);
//             }
//         }
//         coord_set(cur, next->x, next->y);
//         coord_list_free(neighbors);
//         coord_free(next);
//     }
//     coord_free(cur);
//     return obstacle;        
// }

obstacle_t* obstacle_make_beam(
    const coord_t* start, const coord_t* goal, int range){

    if (!start || !goal) return nullptr;

    int width = goal->x - start->x;
    int height = goal->y - start->y;
    obstacle_t* obstacle = obstacle_new_full(
        start->x, start->y, width, height);

    coord_t* cur = coord_copy(start);

    if(range <= 0){
        while(!coord_equal(cur, goal)){
            coord_t* next = coord_clone_next_to_goal(cur, goal);

            if (!obstacle_is_coord_blocked(obstacle, next->x, next->y)) {
                obstacle_block_coord(obstacle, next->x, next->y);
            }
            coord_set(cur, next->x, next->y);
            coord_free(next);
        }
    }

    // while(cur != goal){
    while(!coord_equal(cur, goal)){
        coord_t* next = coord_clone_next_to_goal(cur, goal);

        coord_list_t* neighbors = obstacle_clone_neighbors_all_range(
            obstacle, next->x, next->y, range-1);
        for(int i=0; i < coord_list_length(neighbors); i++){
            const coord_t* c = coord_list_get(neighbors, i);

            if (!obstacle_is_coord_blocked(obstacle, c->x, c->y)){
                obstacle_block_coord(obstacle, c->x, c->y);
            }
        }
        coord_set(cur, next->x, next->y);
        coord_list_free(neighbors);
        coord_free(next);
    }
    coord_free(cur);
    return obstacle;        
}