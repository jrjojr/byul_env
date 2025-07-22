#include "internal/navgrid.h"
#include <unordered_set>
#include <unordered_map>
#include <vector>
#include <cmath>
#include <cstdint>
#include "internal/coord.h"
#include "internal/coord_list.h"
#include "internal/coord_hash.h"

bool is_coord_blocked_navgrid(const void* context, 
    int x, int y, void* userdata) {

    if (!context) return false;
    coord_t* c = coord_create_full(x, y);
    bool result = coord_hash_contains(
        (const coord_hash_t*)((const navgrid_t*)context)->blocked_coords, c);
    coord_destroy(c);
    return result;
}

navgrid_t* navgrid_create() {
    return navgrid_create_full(0, 0, NAVGRID_DIR_8, 
        (is_coord_blocked_func) is_coord_blocked_navgrid);
}

navgrid_t* navgrid_create_full(int width, int height, navgrid_dir_mode_t mode, 
    is_coord_blocked_func is_coord_blocked_fn) {

    if (!is_coord_blocked_fn) {
        is_coord_blocked_fn = is_coord_blocked_navgrid;
    }

    navgrid_t* m = new navgrid_t();
    m->width = width;
    m->height = height;
    m->mode = mode;
    m->blocked_coords = coord_hash_create();
    m->is_coord_blocked_fn = is_coord_blocked_fn;
    return m;
}

void navgrid_destroy(navgrid_t* m) {
    if (!m) return;
    coord_hash_destroy(m->blocked_coords);
    delete m;
}

navgrid_t* navgrid_copy(const navgrid_t* m) {
    if (!m) return nullptr;
    navgrid_t* c = navgrid_create_full(m->width, m->height, m->mode, 
        m->is_coord_blocked_fn);

    c->blocked_coords = coord_hash_copy(m->blocked_coords);
    return c;
}

uint32_t navgrid_hash(const navgrid_t* m) {
    if (!m) return 0;
    uint32_t h = 17;
    h = h * 31 + m->width;
    h = h * 31 + m->height;
    h = h * 31 + m->mode;
    h = h * 31 + coord_hash_length(m->blocked_coords);
    return h;
}

bool navgrid_equal(const navgrid_t* a, const navgrid_t* b) {
    if (!a || !b) return false;
    return a->width == b->width && a->height == b->height &&
           a->mode == b->mode &&
           coord_hash_equal(a->blocked_coords, b->blocked_coords);
}

int navgrid_get_width(const navgrid_t* m) { return m ? m->width : 0; }
void navgrid_set_width(navgrid_t* m, int w) { if (m) m->width = w; }

int navgrid_get_height(const navgrid_t* m) { return m ? m->height : 0; }
void navgrid_set_height(navgrid_t* m, int h) { if (m) m->height = h; }

void navgrid_set_is_coord_blocked_func(navgrid_t* m, is_coord_blocked_func fn){
    if (!m) return;
    m->is_coord_blocked_fn = fn;
}

is_coord_blocked_func navgrid_get_is_coord_blocked_fn(const navgrid_t* m){
    if (!m) return nullptr;
    return m->is_coord_blocked_fn;
}

navgrid_dir_mode_t navgrid_get_mode(const navgrid_t* m) {
     return m ? m->mode : NAVGRID_DIR_4; 
    }

void navgrid_set_mode(navgrid_t* m, navgrid_dir_mode_t mode) {
     if (m) m->mode = mode; 
    }

bool navgrid_block_coord(navgrid_t* m, int x, int y) {
    if (!m) return false;
    coord_t* c = coord_create_full(x, y);
    coord_hash_replace(m->blocked_coords, c, nullptr);
    coord_destroy(c);
    return true;
}

bool navgrid_unblock_coord(navgrid_t* m, int x, int y) {
    if (!m) return false;
    coord_t* c = coord_create_full(x, y);
    bool result = coord_hash_remove(m->blocked_coords, c);
    coord_destroy(c);
    return result;
}

// bool navgrid_is_inside(const navgrid_t* m, int x, int y) {
//     if (!m) return false;
//     bool x_ok = (m->width == 0 || (x >= 0 && x < m->width));
//     bool y_ok = (m->height == 0 || (y >= 0 && y < m->height));
//     return x_ok && y_ok;
// }

bool navgrid_is_inside(const navgrid_t* m, int x, int y) {
    if (!m) return false;

    int min_x = (m->width >= 0) ? 0 : m->width;
    int max_x = (m->width >= 0) ? m->width : 0;
    int min_y = (m->height >= 0) ? 0 : m->height;
    int max_y = (m->height >= 0) ? m->height : 0;

    bool x_ok = (m->width == 0 || (x >= min_x && x < max_x));
    bool y_ok = (m->height == 0 || (y >= min_y && y < max_y));

    return x_ok && y_ok;
}

void navgrid_clear(navgrid_t* m) {
    if (!m) return;
    coord_hash_clear(m->blocked_coords);
}

const coord_hash_t* navgrid_get_blocked_coords(const navgrid_t* m) {
    return m ? m->blocked_coords : nullptr;
}

coord_list_t* navgrid_clone_adjacent(const navgrid_t* m, int x, int y) {
    if (!m) return nullptr;
    coord_list_t* list = coord_list_create();
    static const int dx4[] = {0, -1, 1, 0};
    static const int dy4[] = {-1, 0, 0, 1};
    static const int dx8[] = {0, -1, 1, 0, -1, -1, 1, 1};
    static const int dy8[] = {-1, 0, 0, 1, -1, 1, -1, 1};

    const int* dx = (m->mode == NAVGRID_DIR_8) ? dx8 : dx4;
    const int* dy = (m->mode == NAVGRID_DIR_8) ? dy8 : dy4;
    int count = (m->mode == NAVGRID_DIR_8) ? 8 : 4;

    for (int i = 0; i < count; ++i) {
        int nx = x + dx[i];
        int ny = y + dy[i];
        if (!navgrid_is_inside(m, nx, ny)) continue;
        if (m->is_coord_blocked_fn(m, nx, ny, nullptr)) 
            continue;
        coord_list_push_back(list, make_tmp_coord(nx, ny));
    }
    return list;
}

coord_list_t* navgrid_clone_adjacent_all(const navgrid_t* m, int x, int y) {
    if (!m) return nullptr;
    coord_list_t* list = coord_list_create();
    static const int dx4[] = {0, -1, 1, 0};
    static const int dy4[] = {-1, 0, 0, 1};
    static const int dx8[] = {0, -1, 1, 0, -1, -1, 1, 1};
    static const int dy8[] = {-1, 0, 0, 1, -1, 1, -1, 1};

    const int* dx = (m->mode == NAVGRID_DIR_8) ? dx8 : dx4;
    const int* dy = (m->mode == NAVGRID_DIR_8) ? dy8 : dy4;
    int count = (m->mode == NAVGRID_DIR_8) ? 8 : 4;

    for (int i = 0; i < count; ++i) {
        int nx = x + dx[i];
        int ny = y + dy[i];
        if (!navgrid_is_inside(m, nx, ny)) continue;
        coord_list_push_back(list, make_tmp_coord(nx, ny));
    }
    return list;
}

coord_list_t* navgrid_clone_adjacent_all_range(
    navgrid_t* m, int x, int y, int range) {

    if (!m || range < 0) return nullptr;
    coord_hash_t* seen = coord_hash_create();
    for (int dx = -range; dx <= range; ++dx) {
        for (int dy = -range; dy <= range; ++dy) {
            int cx = x + dx;
            int cy = y + dy;
            if (!navgrid_is_inside(m, cx, cy)) continue;
            coord_list_t* part = navgrid_clone_adjacent_all(m, cx, cy);
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

coord_t* navgrid_clone_neighbor_at_degree(const navgrid_t* m, int x, int y, double degree) {
    if (!m) return nullptr;
    static const int dx8[] = {1,  1, 0, -1, -1, -1,  0, 1};
    static const int dy8[] = {0, -1, -1, -1,  0,  1,  1, 1};
    int count = (m->mode == NAVGRID_DIR_8) ? 8 : 4;
    int best_index = -1;
    double min_diff = 360.0;

    coord_t* origin = coord_create_full(x, y);
    for (int i = 0; i < count; ++i) {
        int nx = x + dx8[i];
        int ny = y + dy8[i];
        if (!navgrid_is_inside(m, nx, ny)) continue;
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

coord_t* navgrid_clone_neighbor_at_goal(const navgrid_t* m, const coord_t* center, const coord_t* goal) {
    if (!m || !center || !goal) return nullptr;
    int x = coord_get_x(center);
    int y = coord_get_y(center);
    coord_list_t* neighbors = navgrid_clone_adjacent_all(m, x, y);
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

coord_list_t* navgrid_clone_adjacent_at_degree_range(
    const navgrid_t* m,
    const coord_t* center, const coord_t* goal,
    double start_deg, double end_deg,
    int range
) {
    if (!m || !center || !goal || range < 0) return nullptr;
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
            if (!navgrid_is_inside(m, nx, ny)) continue;
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

