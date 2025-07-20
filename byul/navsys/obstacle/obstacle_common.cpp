#include "internal/obstacle_common.h"
#include <stdlib.h>
#include <string.h>
#include <cmath>

obstacle_t* obstacle_create() {
    return obstacle_create_full(0, 0, 0, 0);
}

obstacle_t* obstacle_create_full(
    int x0, int y0, int width, int height) {

    obstacle_t* obstacle = (obstacle_t*)malloc(sizeof(obstacle_t));
    if (!obstacle) return NULL;

    obstacle->x0 = x0;
    obstacle->y0 = y0;
    obstacle->width = width;
    obstacle->height = height;
    obstacle->blocked = coord_hash_create();

    return obstacle;
}

void obstacle_clear(obstacle_t* obstacle) {
    if (!obstacle || !obstacle->blocked) return;

    coord_hash_clear(obstacle->blocked);
}

void obstacle_destroy(obstacle_t* obstacle) {
    if (!obstacle) return;
    coord_hash_destroy(obstacle->blocked);
    free(obstacle);
}

obstacle_t* obstacle_copy(const obstacle_t* obstacle) {
    if (!obstacle) return NULL;
    obstacle_t* copy = obstacle_create_full(obstacle->x0, obstacle->y0, 
        obstacle->width, obstacle->height);

    coord_hash_destroy(copy->blocked);
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

void obstacle_fetch_origin(
    const obstacle_t* obstacle, int* out_x0, int* out_y0) {
    if (!obstacle) return;
    if (out_x0) *out_x0 = obstacle->x0;
    if (out_y0) *out_y0 = obstacle->y0;
}

void obstacle_apply_to_navgrid(const obstacle_t* obstacle, navgrid_t* navgrid) {
    if (!obstacle || !navgrid) return;

    coord_hash_iter_t* iter = coord_hash_iter_create(
        (coord_hash_t*)obstacle->blocked);
    coord_t* key;
    while (coord_hash_iter_next(iter, &key, NULL)) {
        navgrid_block_coord(navgrid, key->x, key->y);
    }
    coord_hash_iter_destroy(iter);
}

void obstacle_remove_from_navgrid(const obstacle_t* obstacle, navgrid_t* navgrid) {
    if (!obstacle || !navgrid) return;

    coord_hash_iter_t* iter = coord_hash_iter_create(
        (coord_hash_t*)obstacle->blocked);
    coord_t* key;
    while (coord_hash_iter_next(iter, &key, NULL)) {
        navgrid_unblock_coord(navgrid, key->x, key->y);
    }
    coord_hash_iter_destroy(iter);
}

int obstacle_get_width(const obstacle_t* obs) {
     return obs ? obs->width : 0; 
}

void obstacle_set_width(obstacle_t* obs, int w) {
     if (obs) obs->width = w; 
}

int obstacle_get_height(const obstacle_t* obs) {
    return obs ? obs->height : 0; 
}

void obstacle_set_height(obstacle_t* obs, int h) {
    if (obs) obs->height = h; 
}

bool obstacle_block_coord(obstacle_t* obs, int x, int y) {
    if (!obs) return false;
    coord_t* c = coord_create_full(x, y);
    coord_hash_replace(obs->blocked, c, nullptr);
    coord_destroy(c);
    return true;
}

bool obstacle_unblock_coord(obstacle_t* obs, int x, int y) {
    if (!obs) return false;
    coord_t* c = coord_create_full(x, y);
    bool result = coord_hash_remove(obs->blocked, c);
    coord_destroy(c);
    return result;
}

bool obstacle_is_inside(const obstacle_t* obs, int x, int y) {
    if (!obs) return false;

    int min_x = (obs->width >= 0) ? obs->x0 : obs->x0 + obs->width;
    int max_x = (obs->width >= 0) ? obs->x0 + obs->width : obs->x0;

    int min_y = (obs->height >= 0) ? obs->y0 : obs->y0 + obs->height;
    int max_y = (obs->height >= 0) ? obs->y0 + obs->height : obs->y0;

    return (x >= min_x && x < max_x && y >= min_y && y < max_y);
}

const coord_hash_t* obstacle_get_blocked_coords(const obstacle_t* obs) {
    return obs ? obs->blocked : nullptr;
}

coord_list_t* obstacle_clone_neighbors_all(
    const obstacle_t* obs, int x, int y) {

    if (!obs) return nullptr;
    coord_list_t* list = coord_list_create();
    static const int dx4[] = {0, -1, 1, 0};
    static const int dy4[] = {-1, 0, 0, 1};
    static const int dx8[] = {0, -1, 1, 0, -1, -1, 1, 1};
    static const int dy8[] = {-1, 0, 0, 1, -1, 1, -1, 1};

    const int* dx = dx8;
    const int* dy = dy8;
    int count = 8;

    for (int i = 0; i < count; ++i) {
        int nx = x + dx[i];
        int ny = y + dy[i];
        if (!obstacle_is_inside(obs, nx, ny)) continue;
        coord_list_push_back(list, make_tmp_coord(nx, ny));
    }
    return list;
}

coord_list_t* obstacle_clone_neighbors_all_range(
    obstacle_t* obs, int x, int y, int range) {

    if (!obs || range < 0) return nullptr;
    coord_hash_t* seen = coord_hash_create();
    for (int dx = -range; dx <= range; ++dx) {
        for (int dy = -range; dy <= range; ++dy) {
            int cx = x + dx;
            int cy = y + dy;
            if (!obstacle_is_inside(obs, cx, cy)) continue;
            coord_list_t* part = obstacle_clone_neighbors_all(obs, cx, cy);
            int len = coord_list_length(part);
            for (int i = 0; i < len; ++i) {
                const coord_t* c = coord_list_get(part, i);
                if (!coord_hash_contains(seen, c))
                    coord_hash_replace(seen, c, nullptr);
            }
            coord_list_destroy(part);
        }
    }
    coord_list_t* result = coord_hash_to_list(seen);
    coord_hash_destroy(seen);
    return result;
}

coord_t* obstacle_clone_neighbor_at_degree(
    const obstacle_t* obs, int x, int y, double degree) {

    if (!obs) return nullptr;
    static const int dx8[] = {1,  1, 0, -1, -1, -1,  0, 1};
    static const int dy8[] = {0, -1, -1, -1,  0,  1,  1, 1};
    int count = 8;
    int best_index = -1;
    double min_diff = 360.0;

    coord_t* origin = coord_create_full(x, y);
    for (int i = 0; i < count; ++i) {
        int nx = x + dx8[i];
        int ny = y + dy8[i];
        if (!obstacle_is_inside(obs, nx, ny)) continue;
        coord_t* target = coord_create_full(nx, ny);
        double deg = coord_degree(origin, target);
        double diff = fabs(degree - deg);
        if (diff > 180.0) diff = 360.0 - diff;
        if (diff < min_diff) {
            min_diff = diff;
            best_index = i;
        }
        coord_destroy(target);
    }
    coord_destroy(origin);
    if (best_index == -1) return nullptr;
    return coord_create_full(x + dx8[best_index], y + dy8[best_index]);
}

coord_t* obstacle_clone_neighbor_at_goal(
    const obstacle_t* obs, const coord_t* center, const coord_t* goal) {

    if (!obs || !center || !goal) return nullptr;
    int x = coord_get_x(center);
    int y = coord_get_y(center);
    coord_list_t* neighbors = obstacle_clone_neighbors_all(obs, x, y);
    if (!neighbors) return nullptr;
    double target_deg = coord_degree(center, goal);
    coord_t* best = nullptr;
    double min_diff = 360.0;

    int len = coord_list_length(neighbors);
    for (int i = 0; i < len; ++i) {
        const coord_t* c = coord_list_get(neighbors, i);
        double deg = coord_degree(center, c);
        double diff = fabs(target_deg - deg);
        if (diff > 180.0) diff = 360.0 - diff;
        if (diff < min_diff) {
            min_diff = diff;
            if (best) coord_destroy(best);
            best = coord_copy(c);
        }
    }
    coord_list_destroy(neighbors);
    return best;
}

coord_list_t* obstacle_clone_neighbors_at_degree_range(
    const obstacle_t* obs,
    const coord_t* center, const coord_t* goal,
    double start_deg, double end_deg,
    int range
) {
    if (!obs || !center || !goal || range < 0) return nullptr;
    double center_deg = coord_degree(center, goal);
    double deg_min = fmod(center_deg + start_deg + 360.0, 360.0);
    double deg_max = fmod(center_deg + end_deg + 360.0, 360.0);
    bool wraps = deg_min > deg_max;

    coord_hash_t* seen = coord_hash_create();
    int cx = coord_get_x(center);
    int cy = coord_get_y(center);

    for (int dx = -range; dx <= range; ++dx) {
        for (int dy = -range; dy <= range; ++dy) {
            if (dx == 0 && dy == 0) continue;
            int nx = cx + dx;
            int ny = cy + dy;
            if (!obstacle_is_inside(obs, nx, ny)) continue;
            coord_t* target = coord_create_full(nx, ny);
            double deg = coord_degree(center, target);
            bool in_range = (!wraps) ? (deg >= deg_min && deg <= deg_max)
                                     : (deg >= deg_min || deg <= deg_max);
            if (in_range && !coord_hash_contains(seen, target)) {
                coord_hash_replace(seen, target, nullptr);
            }
            coord_destroy(target);
        }
    }
    coord_list_t* result = coord_hash_to_list(seen);
    coord_hash_destroy(seen);
    return result;
}

bool obstacle_is_coord_blocked(const obstacle_t* obstacle, int x, int y){
    if (!obstacle) return false;

    return coord_hash_contains(obstacle->blocked, make_tmp_coord(x, y));
}

void obstacle_block_range(obstacle_t* obs, int x, int y, int range) {
    if (!obs || range < 0) return;

    for (int dx = -range; dx <= range; ++dx) {
        for (int dy = -range; dy <= range; ++dy) {
            obstacle_block_coord(obs, x + dx, y + dy);
        }
    }
}

void obstacle_block_straight(obstacle_t* obs, 
    int x0, int y0, int x1, int y1, int range) {

    int dx = abs(x1 - x0), sx = (x0 < x1) ? 1 : -1;
    int dy = -abs(y1 - y0), sy = (y0 < y1) ? 1 : -1;
    int err = dx + dy;

    while (true) {
        if (range <= 0)
            obstacle_block_coord(obs, x0, y0);
        else
            obstacle_block_range(obs, x0, y0, range);

        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}