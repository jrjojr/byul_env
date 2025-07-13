#include "internal/obstacle.h"
#include <stdlib.h>
#include <string.h>

obstacle_t* obstacle_new() {
    return obstacle_new_full(0, 0, 0, 0);
}

obstacle_t* obstacle_new_full(
    int x0, int y0, int width, int height) {

    obstacle_t* obstacle = (obstacle_t*)malloc(sizeof(obstacle_t));
    if (!obstacle) return NULL;

    obstacle->x0 = x0;
    obstacle->y0 = y0;
    obstacle->width = width;
    obstacle->height = height;
    obstacle->blocked = coord_hash_new();

    return obstacle;
}

void obstacle_clear(obstacle_t* obstacle) {
    if (!obstacle || !obstacle->blocked) return;

    coord_hash_clear(obstacle->blocked);
}

void obstacle_free(obstacle_t* obstacle) {
    if (!obstacle) return;
    coord_hash_free(obstacle->blocked);
    free(obstacle);
}

obstacle_t* obstacle_copy(const obstacle_t* obstacle) {
    if (!obstacle) return NULL;
    obstacle_t* copy = obstacle_new_full(obstacle->x0, obstacle->y0, 
        obstacle->width, obstacle->height);

    coord_hash_free(copy->blocked);
    copy->blocked = coord_hash_copy(obstacle->blocked);
    return copy;
}

bool obstacle_equal(const obstacle_t* a, const obstacle_t* b) {
    if (!a || !b) return false;
    return a->x0 == b->x0 &&
           a->y0 == b->y0 &&
           a->width == b->width &&
           a->height == b->height &&
           coord_hash_equal(a->blocked, b->blocked);
}

uint32_t obstacle_hash(const obstacle_t* obstacle) {
    if (!obstacle) return 0;
    uint32_t hash = 17;
    hash = 31 * hash + obstacle->x0;
    hash = 31 * hash + obstacle->y0;
    hash = 31 * hash + obstacle->width;
    hash = 31 * hash + obstacle->height;
    hash = 31 * hash + coord_hash_hash(obstacle->blocked);
    return hash;
}

void obstacle_set_origin(obstacle_t* obstacle, int x0, int y0) {
    if (!obstacle) return;
    obstacle->x0 = x0;
    obstacle->y0 = y0;
}

void obstacle_get_origin(const obstacle_t* obstacle, int* out_x0, int* out_y0) {
    if (!obstacle) return;
    if (out_x0) *out_x0 = obstacle->x0;
    if (out_y0) *out_y0 = obstacle->y0;
}

int obstacle_get_width(const obstacle_t* obstacle) {
    return obstacle ? obstacle->width : 0;
}

int obstacle_get_height(const obstacle_t* obstacle) {
    return obstacle ? obstacle->height : 0;
}

const coord_hash_t* obstacle_get_blocked_coords(const obstacle_t* obstacle) {
    return obstacle ? obstacle->blocked : NULL;
}

void obstacle_apply_to_map(const obstacle_t* obstacle, map_t* map) {
    if (!obstacle || !map) return;

    coord_hash_iter_t* iter = coord_hash_iter_new((coord_hash_t*)obstacle->blocked);
    coord_t* key;
    while (coord_hash_iter_next(iter, &key, NULL)) {
        map_block_coord(map, key->x, key->y);
    }
    coord_hash_iter_free(iter);
}

void obstacle_remove_from_map(const obstacle_t* obstacle, map_t* map) {
    if (!obstacle || !map) return;

    coord_hash_iter_t* iter = coord_hash_iter_new((coord_hash_t*)obstacle->blocked);
    coord_t* key;
    while (coord_hash_iter_next(iter, &key, NULL)) {
        map_unblock_coord(map, key->x, key->y);
    }
    coord_hash_iter_free(iter);
}
