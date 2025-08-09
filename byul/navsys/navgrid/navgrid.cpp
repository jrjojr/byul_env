#include "navgrid.h"
#include <unordered_set>
#include <unordered_map>
#include <vector>
#include <cmath>
#include <cstdint>
#include "coord.h"
#include "coord_list.h"
#include "coord_hash.h"

bool is_coord_blocked_navgrid(const void* context, 
    int x, int y, void* userdata) {

   const navgrid_t* navgrid = (const navgrid_t*)context;
    if (!navgrid) return false;
    coord_t key = {x, y};
    if (!coord_hash_contains(navgrid->cell_map, &key)) {
        return false;
    }

    navcell_t out = {};
    navgrid_fetch_cell(navgrid, x, y, &out);
    return out.terrain == TERRAIN_TYPE_FORBIDDEN;
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

    navgrid_t* navgrid = new navgrid_t();
    navgrid->width = width;
    navgrid->height = height;
    navgrid->mode = mode;
    navgrid->cell_map = coord_hash_create_full(
        (coord_hash_copy_func) navcell_copy,
        (coord_hash_destroy_func) navcell_destroy
    );

    navgrid->is_coord_blocked_fn = is_coord_blocked_fn;
    return navgrid;
}

void navgrid_destroy(navgrid_t* navgrid) {
    if (!navgrid) return;
    coord_hash_destroy(navgrid->cell_map);
    delete navgrid;
}

navgrid_t* navgrid_copy(const navgrid_t* navgrid) {
    if (!navgrid) return nullptr;
    navgrid_t* c = navgrid_create_full(
        navgrid->width, navgrid->height, navgrid->mode, 
        navgrid->is_coord_blocked_fn);

    c->cell_map = coord_hash_copy(navgrid->cell_map);
    return c;
}

uint32_t navgrid_hash(const navgrid_t* navgrid) {
    if (!navgrid) return 0;
    uint32_t h = 17;
    h = h * 31 + navgrid->width;
    h = h * 31 + navgrid->height;
    h = h * 31 + navgrid->mode;
    h = h * 31 + coord_hash_length(navgrid->cell_map);
    return h;
}

bool navgrid_equal(const navgrid_t* a, const navgrid_t* b) {
    if (!a || !b) return false;
    return a->width == b->width && a->height == b->height &&
           a->mode == b->mode &&
           coord_hash_equal(a->cell_map, b->cell_map);
}

int navgrid_get_width(const navgrid_t* navgrid) { 
    return navgrid ? navgrid->width : 0; 
}

void navgrid_set_width(navgrid_t* navgrid, int w) { 
    if (navgrid) navgrid->width = w; 
}

int navgrid_get_height(const navgrid_t* navgrid) { 
    return navgrid ? navgrid->height : 0; 
}

void navgrid_set_height(navgrid_t* navgrid, int h) { 
    if (navgrid) navgrid->height = h; 
}

void navgrid_set_is_coord_blocked_func(
    navgrid_t* navgrid, is_coord_blocked_func fn){

    if (!navgrid) return;
    navgrid->is_coord_blocked_fn = fn;
}

is_coord_blocked_func navgrid_get_is_coord_blocked_fn(
    const navgrid_t* navgrid){
    if (!navgrid) return nullptr;
    return navgrid->is_coord_blocked_fn;
}

navgrid_dir_mode_t navgrid_get_mode(const navgrid_t* navgrid) {
     return navgrid ? navgrid->mode : NAVGRID_DIR_4; 
    }

void navgrid_set_mode(navgrid_t* navgrid, navgrid_dir_mode_t mode) {
     if (navgrid) navgrid->mode = mode; 
    }

bool navgrid_block_coord(navgrid_t* navgrid, int x, int y) {
    if (!navgrid) return false;
    coord_t c;
    coord_init_full(&c, x, y);
    navcell_t nc;
    navcell_init_full(&nc, TERRAIN_TYPE_FORBIDDEN, 0);
    return coord_hash_replace(navgrid->cell_map, &c, &nc);
}

bool navgrid_unblock_coord(navgrid_t* navgrid, int x, int y) {
    if (!navgrid) return false;
    coord_t c;
    coord_init_full(&c, x, y);
    navcell_t nc;
    navcell_init_full(&nc, TERRAIN_TYPE_NORMAL, 0);
    return coord_hash_replace(navgrid->cell_map, &c, &nc);
}

bool navgrid_is_inside(const navgrid_t* navgrid, int x, int y) {
    if (!navgrid) return false;

    int min_x = (navgrid->width >= 0) ? 0 : navgrid->width;
    int max_x = (navgrid->width >= 0) ? navgrid->width : 0;
    int min_y = (navgrid->height >= 0) ? 0 : navgrid->height;
    int max_y = (navgrid->height >= 0) ? navgrid->height : 0;

    bool x_ok = (navgrid->width == 0 || (x >= min_x && x < max_x));
    bool y_ok = (navgrid->height == 0 || (y >= min_y && y < max_y));

    return x_ok && y_ok;
}

void navgrid_clear(navgrid_t* navgrid) {
    if (!navgrid) return;
    coord_hash_clear(navgrid->cell_map);
}

bool navgrid_set_cell(
    navgrid_t* navgrid, int x, int y, const navcell_t* cell) {

    if (!navgrid || !cell) return false;
    coord_t c;
    coord_init_full(&c, x, y);
    navcell_t copy;
    navcell_assign(&copy, cell);
    coord_hash_replace(navgrid->cell_map, &c, &copy);
    return true;
}

int navgrid_fetch_cell(
    const navgrid_t* navgrid, int x, int y, navcell_t* out) {

    if (!navgrid) return -1;

    coord_t c;
    coord_init_full(&c, x, y);

    if (!coord_hash_contains(navgrid->cell_map, &c)){
        return -1;
    }

    *out = *(navcell_t*) coord_hash_get(navgrid->cell_map, &c);
    return 0;
}

const coord_hash_t* navgrid_get_cell_map(const navgrid_t* navgrid) {
    return navgrid ? navgrid->cell_map : nullptr;
}

coord_list_t* navgrid_copy_neighbors(
    const navgrid_t* navgrid, int x, int y) {

    if (!navgrid) return nullptr;
    coord_list_t* list = coord_list_create();
    static const int dx4[] = {0, -1, 1, 0};
    static const int dy4[] = {-1, 0, 0, 1};
    static const int dx8[] = {0, -1, 1, 0, -1, -1, 1, 1};
    static const int dy8[] = {-1, 0, 0, 1, -1, 1, -1, 1};

    const int* dx = (navgrid->mode == NAVGRID_DIR_8) ? dx8 : dx4;
    const int* dy = (navgrid->mode == NAVGRID_DIR_8) ? dy8 : dy4;
    int count = (navgrid->mode == NAVGRID_DIR_8) ? 8 : 4;

    for (int i = 0; i < count; ++i) {
        int nx = x + dx[i];
        int ny = y + dy[i];
        if (!navgrid_is_inside(navgrid, nx, ny)) continue;
        if (navgrid->is_coord_blocked_fn(navgrid, nx, ny, nullptr)) 
            continue;

        coord_t tmp = coord_t{nx, ny};
        coord_list_push_back(list, &tmp);
    }
    return list;
}

coord_list_t* navgrid_copy_neighbors_all(
    const navgrid_t* navgrid, int x, int y) {

    if (!navgrid) return nullptr;
    coord_list_t* list = coord_list_create();
    static const int dx4[] = {0, -1, 1, 0};
    static const int dy4[] = {-1, 0, 0, 1};
    static const int dx8[] = {0, -1, 1, 0, -1, -1, 1, 1};
    static const int dy8[] = {-1, 0, 0, 1, -1, 1, -1, 1};

    const int* dx = (navgrid->mode == NAVGRID_DIR_8) ? dx8 : dx4;
    const int* dy = (navgrid->mode == NAVGRID_DIR_8) ? dy8 : dy4;
    int count = (navgrid->mode == NAVGRID_DIR_8) ? 8 : 4;

    for (int i = 0; i < count; ++i) {
        int nx = x + dx[i];
        int ny = y + dy[i];
        if (!navgrid_is_inside(navgrid, nx, ny)) continue;

        coord_t tmp = coord_t{nx, ny};
        coord_list_push_back(list, &tmp);
    }
    return list;
}

coord_list_t* navgrid_copy_neighbors_all_range(
    navgrid_t* navgrid, int x, int y, int range) {

    if (!navgrid || range < 0) return nullptr;
    coord_hash_t* seen = coord_hash_create();
    for (int dx = -range; dx <= range; ++dx) {
        for (int dy = -range; dy <= range; ++dy) {
            int cx = x + dx;
            int cy = y + dy;
            if (!navgrid_is_inside(navgrid, cx, cy)) continue;
            coord_list_t* part = navgrid_copy_neighbors_all(navgrid, cx, cy);
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

coord_t* navgrid_copy_neighbor_at_degree(
    const navgrid_t* navgrid, int x, int y, double degree) {
    if (!navgrid) return nullptr;
    static const int dx8[] = {1,  1, 0, -1, -1, -1,  0, 1};
    static const int dy8[] = {0, -1, -1, -1,  0,  1,  1, 1};
    int count = (navgrid->mode == NAVGRID_DIR_8) ? 8 : 4;
    int best_index = -1;
    double min_diff = 360.0;

    coord_t origin;
    coord_init_full(&origin, x, y);
    for (int i = 0; i < count; ++i) {
        int nx = x + dx8[i];
        int ny = y + dy8[i];
        if (!navgrid_is_inside(navgrid, nx, ny)) continue;
        coord_t* target = coord_create_full(nx, ny);
        double deg = coord_degree(&origin, target);
        double diff = fabs(degree - deg);
        if (diff > 180.0) diff = 360.0 - diff;
        if (diff < min_diff) {
            min_diff = diff;
            best_index = i;
        }
        coord_destroy(target);
    }

    if (best_index == -1) return nullptr;
    return coord_create_full(x + dx8[best_index], y + dy8[best_index]);
}

coord_t* navgrid_copy_neighbor_at_goal(
    const navgrid_t* navgrid, const coord_t* center, const coord_t* goal) {
    if (!navgrid || !center || !goal) return nullptr;
    int x = coord_get_x(center);
    int y = coord_get_y(center);
    coord_list_t* neighbors = navgrid_copy_neighbors_all(navgrid, x, y);
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

coord_list_t* navgrid_copy_neighbors_at_degree_range(
    const navgrid_t* navgrid,
    const coord_t* center, const coord_t* goal,
    double start_deg, double end_deg,
    int range
) {
    if (!navgrid || !center || !goal || range < 0) return nullptr;
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
            if (!navgrid_is_inside(navgrid, nx, ny)) continue;
            coord_t target;
            coord_init_full(&target, nx, ny);
            double deg = coord_degree(center, &target);
            bool in_range = (!wraps) ? (deg >= deg_min && deg <= deg_max)
                                     : (deg >= deg_min || deg <= deg_max);
            if (in_range && !coord_hash_contains(seen, &target)) {
                coord_hash_replace(seen, &target, nullptr);
            }
        }
    }
    coord_list_t* result = coord_hash_to_list(seen);
    coord_hash_destroy(seen);
    return result;
}

