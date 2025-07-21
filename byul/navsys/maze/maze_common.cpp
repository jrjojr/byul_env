#include "internal/maze.h"
#include "internal/maze_common.h"
#include <stdlib.h>
#include <string.h>

maze_t* maze_create() {
    return maze_create_full(0, 0, 0, 0);
}

maze_t* maze_create_full(
    int x0, int y0, int width, int height) {

    maze_t* maze = (maze_t*)malloc(sizeof(maze_t));
    if (!maze) return NULL;

    maze->x0 = x0;
    maze->y0 = y0;
    maze->width = width;
    maze->height = height;
    maze->blocked = coord_hash_create();

    return maze;
}

void maze_clear(maze_t* maze) {
    if (!maze || !maze->blocked) return;

    coord_hash_clear(maze->blocked);
}

void maze_destroy(maze_t* maze) {
    if (!maze) return;
    coord_hash_destroy(maze->blocked);
    free(maze);
}

maze_t* maze_copy(const maze_t* maze) {
    if (!maze) return NULL;
    maze_t* copy = maze_create_full(maze->x0, maze->y0, 
        maze->width, maze->height);

    coord_hash_destroy(copy->blocked);
    copy->blocked = coord_hash_copy(maze->blocked);
    return copy;
}

bool maze_equal(const maze_t* a, const maze_t* b) {
    if (!a || !b) return false;
    return a->x0 == b->x0 &&
           a->y0 == b->y0 &&
           a->width == b->width &&
           a->height == b->height &&
           coord_hash_equal(a->blocked, b->blocked);
}

uint32_t maze_hash(const maze_t* maze) {
    if (!maze) return 0;
    uint32_t hash = 17;
    hash = 31 * hash + maze->x0;
    hash = 31 * hash + maze->y0;
    hash = 31 * hash + maze->width;
    hash = 31 * hash + maze->height;
    hash = 31 * hash + coord_hash_hash(maze->blocked);
    return hash;
}

void maze_set_origin(maze_t* maze, int x0, int y0) {
    if (!maze) return;
    maze->x0 = x0;
    maze->y0 = y0;
}

void maze_get_origin(const maze_t* maze, int* out_x0, int* out_y0) {
    if (!maze) return;
    if (out_x0) *out_x0 = maze->x0;
    if (out_y0) *out_y0 = maze->y0;
}

int maze_get_width(const maze_t* maze) {
    return maze ? maze->width : 0;
}

int maze_get_height(const maze_t* maze) {
    return maze ? maze->height : 0;
}

const coord_hash_t* maze_get_blocked_coords(const maze_t* maze) {
    return maze ? maze->blocked : NULL;
}

void maze_apply_to_navgrid(const maze_t* maze, navgrid_t* navgrid) {
    if (!maze || !navgrid) return;

    coord_hash_iter_t* iter = coord_hash_iter_create((coord_hash_t*)maze->blocked);
    coord_t* key;
    while (coord_hash_iter_next(iter, &key, NULL)) {
        navgrid_block_coord(navgrid, key->x, key->y);
    }
    coord_hash_iter_destroy(iter);
}

void maze_remove_from_navgrid(const maze_t* maze, navgrid_t* navgrid) {
    if (!maze || !navgrid) return;

    coord_hash_iter_t* iter = coord_hash_iter_create((coord_hash_t*)maze->blocked);
    coord_t* key;
    while (coord_hash_iter_next(iter, &key, NULL)) {
        navgrid_unblock_coord(navgrid, key->x, key->y);
    }
    coord_hash_iter_destroy(iter);
}
