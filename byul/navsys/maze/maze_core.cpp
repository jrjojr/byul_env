#include "maze.h"
#include "maze_core.h"
#include <stdlib.h>
#include <string.h>
#include <vector>
#include "../navgrid/internal/navgrid_overlay.hpp"

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
    try {
        const size_t count = coord_hash_size(maze->blocked);
        std::vector<coord_t> coords(count);
        size_t exported = 0;
        if (coord_hash_export_keys(
                maze->blocked,
                coords.empty() ? nullptr : coords.data(),
                coords.size(),
                &exported) != NAVSYS_STATUS_OK) {
            return;
        }
        size_t changed = 0;
        if (byul::navsys::internal::navgrid_replace_blocked_overlay_source(
                navgrid,
                byul::navsys::internal::navgrid_overlay_source_kind::maze,
                maze,
                coords.empty() ? nullptr : coords.data(),
                exported,
                &changed) != NAVSYS_STATUS_OK) {
            return;
        }

        const int maze_width = maze_get_width(maze);
        const int maze_height = maze_get_height(maze);
        if (navgrid_get_width(navgrid) < maze_width)
            navgrid_set_width(navgrid, maze_width);
        if (navgrid_get_height(navgrid) < maze_height)
            navgrid_set_height(navgrid, maze_height);
    } catch (...) {
        return;
    }
}

void maze_remove_from_navgrid(const maze_t* maze, navgrid_t* navgrid) {
    if (!maze || !navgrid) return;
    size_t changed = 0;
    (void)byul::navsys::internal::navgrid_remove_blocked_overlay_source(
        navgrid,
        byul::navsys::internal::navgrid_overlay_source_kind::maze,
        maze,
        &changed);
}
