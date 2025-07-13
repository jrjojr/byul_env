// route.cpp - GLib 제거, 구조체 은닉, 내부 std::vector 기반

#include "internal/route.h"
#include "coord.hpp"
#include "internal/core.h"
#include "internal/coord_list.h"

#include <vector>
#include <cmath>
#include <cstdio>
#include <algorithm>

static const int ROUTE_DIRECTION_VECTORS[9][2] = {
    {  0,  0 },  // UNKNOWN
    {  1,  0 },  // RIGHT
    {  1, -1 },  // UP_RIGHT
    {  0, -1 },  // UP
    { -1, -1 },  // UP_LEFT
    { -1,  0 },  // LEFT
    { -1,  1 },  // DOWN_LEFT
    {  0,  1 },  // DOWN
    {  1,  1 },  // DOWN_RIGHT
};

route_t* route_new(void) {
    return route_new_full(0.0);
}

route_t* route_new_full(float cost) {
    route_t* r = new route_t();
    r->coords = coord_list_new();
    r->visited_order = coord_list_new();
    r->visited_count = coord_hash_new();
    r->cost = cost;
    r->success = false;
    r->total_retry_count = 0;

    r->avg_vec_x = 0.0;
    r->avg_vec_y = 0.0;
    r->vec_count = 0;    
    return r;
}

void route_free(route_t* p) {
    if (!p) return;
    coord_list_free(p->coords);
    coord_list_free(p->visited_order);
    coord_hash_free(p->visited_count);
    delete p;
}


route_t* route_copy(const route_t* p) {
    if (!p) return nullptr;
    return new route_t(*p);  // 깊은 복사 (vector/map 모두 복사됨)
}

uintptr_t route_hash(const route_t* p) {
    return reinterpret_cast<uintptr_t>(p);
}

int route_equal(const route_t* a, const route_t* b) {
    return a == b;
}

void route_set_cost(route_t* p, float cost) {
    if (p) p->cost = cost;
}

float route_get_cost(const route_t* p) {
    return p ? p->cost : 0.0f;
}

void route_set_success(route_t* p, int success) {
    if (p) p->success = success;
}

int route_get_success(const route_t* p) {
    return p ? p->success : 0;
}

const coord_list_t* route_get_coords(const route_t* p) {
    return p ? p->coords : nullptr;
}

int route_add_coord(route_t* p, const coord_t* c) {
    if (!p) return 0;
    return coord_list_push_back(p->coords, c);
}

void route_clear_coords(route_t* p) {
    if (!p) return;
    coord_list_clear(p->coords);
}

int route_length(const route_t* p) {
    return p ? static_cast<int>(coord_list_length(p->coords)) : 0;
}

const coord_t* route_get_last(const route_t* p) {
    if (!p || coord_list_empty(p->coords)) return nullptr;
    return coord_list_back(p->coords);
}

const coord_t* route_get_coord_at(const route_t* p, int index) {
    if (!p || index < 0 || 
        index >= static_cast<int>(coord_list_length(p->coords)))

        return nullptr;
    return coord_list_get(p->coords, index);
}

const coord_list_t* route_get_visited_order(const route_t* p) {
    return p ? p->visited_order : nullptr;
}

const coord_hash_t* route_get_visited_count(const route_t* p) {
    return p ? p->visited_count : nullptr;
}

int route_get_total_retry_count(const route_t* p) {
    return p->total_retry_count;
}

void route_set_total_retry_count(route_t* p, int retry_count) {
    p->total_retry_count = retry_count;
}

int route_add_visited(route_t* p, const coord_t* c) {
    if (!p) return 0;
    coord_list_push_back(p->visited_order, c);

    int* old_val = static_cast<int*>(coord_hash_get(p->visited_count, c));
    int count = (old_val ? *old_val : 0) + 1;
    int* count_ptr = new int(count);
    coord_hash_replace(p->visited_count, c, count_ptr);
    delete count_ptr;

    return 1;
}

void route_clear_visited(route_t* p) {
    if (!p) return;
    coord_list_clear(p->visited_order);
    coord_hash_clear(p->visited_count);
}

void route_append(route_t* dest, const route_t* src) {
    if (!dest || !src) return;
    int n = coord_list_length(src->coords);
    for (int i = 0; i < n; ++i) {
        const coord_t* c = coord_list_get(src->coords, i);
        if (c) coord_list_push_back(dest->coords, c);
    }
}

void route_append_nodup(route_t* dest, const route_t* src) {
    if (!dest || !src) return;
    int src_len = coord_list_length(src->coords);
    if (src_len == 0) return;

    const coord_t* first = coord_list_get(src->coords, 0);
    const coord_t* last = coord_list_back(dest->coords);

    int start_index = 0;
    if (last && first && coord_get_x(last) == coord_get_x(first) && 
        coord_get_y(last) == coord_get_y(first))
        start_index = 1;

    for (int i = start_index; i < src_len; ++i) {
        const coord_t* c = coord_list_get(src->coords, i);
        if (c) coord_list_push_back(dest->coords, c);
    }
}

void route_insert(route_t* p, int index, const coord_t* c) {
    if (!p || !c || index < 0 || index > coord_list_length(p->coords)) return;
    coord_list_insert(p->coords, index, c);
}

void route_remove_at(route_t* p, int index) {
    if (!p || index < 0 || index >= coord_list_length(p->coords)) return;
    coord_list_remove_at(p->coords, index);
}

void route_remove_value(route_t* p, const coord_t* c) {
    if (!p || !c) return;
    coord_list_remove_value(p->coords, c);
}

int route_contains(const route_t* p, const coord_t* c) {
    if (!p || !c) return 0;
    return coord_list_contains(p->coords, c);
}

int route_find(const route_t* p, const coord_t* c) {
    if (!p || !c) return -1;
    return coord_list_find(p->coords, c);
}

route_t* route_slice(const route_t* p, int start, int end) {
    if (!p || start < 0 || end <= start) return NULL;

    int length = coord_list_length(p->coords);
    if (end > length) return NULL;

    coord_list_t* sliced_coords = coord_list_sublist(p->coords, start, end);
    if (!sliced_coords) return NULL;

    route_t* new_route = route_new();  // route_new() 함수가 있어야 함
    if (!new_route) {
        coord_list_free(sliced_coords);
        return NULL;
    }

    if (new_route->coords) coord_list_free(new_route->coords);
    new_route->coords = sliced_coords;
    return new_route;
}

void route_print(const route_t* p) {
    if (!p) return;
    printf("Route(len : %d): ", coord_list_length(route_get_coords(p)));
    int len = coord_list_length(p->coords);
    for (int i = 0; i < len; ++i) {
        const coord_t* c = coord_list_get(p->coords, i);
        if (!c) continue;
        if (i > 0) printf(" -> ");
        printf("(%d, %d)", coord_get_x(c), 
            coord_get_y(c));
    }
    printf("\n");
}

// coord_t* route_make_direction(route_t* p, int index) {
//     if (!p || coord_list_length(p->coords) < 2) return coord_new_full(0, 0);
//     int len = coord_list_length(p->coords);
//     if (index < 0 || index >= len) return coord_new_full(0, 0);

//     const coord_t* curr = coord_list_get(p->coords, index);
//     const coord_t* prev = (index >= 1) ? 
//         coord_list_get(p->coords, index - 1) : curr;

//     const coord_t* next = (index < len - 1) ? 
//         coord_list_get(p->coords, index + 1) : curr;

//     if (!curr || !prev || !next) return coord_new_full(0, 0);

//     int vx = 0, vy = 0;
//     vx += coord_get_x(curr) - coord_get_x(prev);
//     vy += coord_get_y(curr) - coord_get_y(prev);
//     vx += coord_get_x(next) - coord_get_x(curr);
//     vy += coord_get_y(next) - coord_get_y(curr);

//     if (vx > 1) vx = 1;
//     if (vx < -1) vx = -1;
//     if (vy > 1) vy = 1;
//     if (vy < -1) vy = -1;

//     return coord_new_full(vx, vy);
// }

coord_t* route_make_direction(route_t* p, int index) {
    if (!p || coord_list_length(p->coords) < 2) return coord_new_full(0, 0);

    int len = coord_list_length(p->coords);
    if (index < 0 || index >= len) return coord_new_full(0, 0);

    const coord_t* curr = coord_list_get(p->coords, index);

    // 끝점은 다음 좌표가 없기 때문에 이전 방향이 유지된다
    if (index == len - 1) {
        const coord_t* prev = coord_list_get(p->coords, index - 1);
        return coord_new_full(
            coord_get_x(curr) - coord_get_x(prev),
            coord_get_y(curr) - coord_get_y(prev)
        );
    }

    // 시작점과 중간 점은 현재좌표와 다음 좌표의 관계로 방향을 얻는다.
    const coord_t* next = coord_list_get(p->coords, index + 1);
    return coord_new_full(
        coord_get_x(next) - coord_get_x(curr),
        coord_get_y(next) - coord_get_y(curr)
    );
}


route_dir_t route_get_direction_by_dir_coord(const coord_t* dxdy) {
    if (coord_get_x(dxdy) == 0 && coord_get_y(dxdy) == 0)
        return ROUTE_DIR_UNKNOWN;

    int nx = (coord_get_x(dxdy) > 0) ? 1 : (coord_get_x(dxdy) < 0) ? -1 : 0;
    int ny = (coord_get_y(dxdy) > 0) ? 1 : (coord_get_y(dxdy) < 0) ? -1 : 0;

    for (int i = 1; i <= 8; ++i) {
        if (ROUTE_DIRECTION_VECTORS[i][0] == nx &&
            ROUTE_DIRECTION_VECTORS[i][1] == ny) {
            return static_cast<route_dir_t>(i);
        }
    }
    return ROUTE_DIR_UNKNOWN;
}

route_dir_t route_get_direction_by_index(route_t* p, int index) {
    route_dir_t dir;
    coord_t* vec = route_make_direction(p, index);
    dir = route_get_direction_by_dir_coord(vec);
    coord_free(vec);
    return dir;
}

route_dir_t route_calc_average_facing(route_t* p, int history) {
    if (!p || history < 1) return ROUTE_DIR_UNKNOWN;
    int len = static_cast<int>(coord_list_length(p->coords));
    if (len < 2) return ROUTE_DIR_UNKNOWN;

    int from = len - history - 1;
    if (from < 0) from = 0;

    const coord_t* c_from = coord_list_get(p->coords, from);
    const coord_t* c_to = coord_list_get(p->coords, len - 1);

    int dx = coord_get_x(c_to) - coord_get_x(c_from);
    int dy = coord_get_y(c_to) - coord_get_y(c_from);

    if (dx > 1) dx = 1;
    if (dx < -1) dx = -1;
    if (dy > 1) dy = 1;
    if (dy < -1) dy = -1;

    for (int i = 1; i <= 8; ++i) {
        if (dx == ROUTE_DIRECTION_VECTORS[i][0] && 
            dy == ROUTE_DIRECTION_VECTORS[i][1]) {
            return static_cast<route_dir_t>(i);
        }
    }
    return ROUTE_DIR_UNKNOWN;
}

float route_calc_average_dir(route_t* p, int history) {
    if (!p || history < 1) return 0.0f;
    int len = static_cast<int>(coord_list_length(p->coords));
    if (len < 2) return 0.0f;

    int from = len - history - 1;
    if (from < 0) from = 0;

    const coord_t* c_from = coord_list_get(p->coords, from);
    const coord_t* c_to = coord_list_get(p->coords, len - 1);

    int dx = coord_get_x(c_to) - coord_get_x(c_from);
    int dy = coord_get_y(c_to) - coord_get_y(c_from);

    if (dx == 0 && dy == 0) return 0.0f;
    return std::atan2((float)dy, (float)dx) * (180.0f / M_PI);
}

route_dir_t calc_direction(const coord_t* start, const coord_t* goal) {
    int dx = coord_get_x(goal) - coord_get_x(start);
    int dy = coord_get_y(goal) - coord_get_y(start);

    if (dx > 1) dx = 1;
    if (dx < -1) dx = -1;
    if (dy > 1) dy = 1;
    if (dy < -1) dy = -1;

    for (int i = 1; i <= 8; ++i) {
        if (ROUTE_DIRECTION_VECTORS[i][0] == dx && 
            ROUTE_DIRECTION_VECTORS[i][1] == dy) {
            return static_cast<route_dir_t>(i);
        }
    }
    return ROUTE_DIR_UNKNOWN;
}

coord_t* direction_to_coord(route_dir_t dir) {
    if (dir < ROUTE_DIR_UNKNOWN || dir > ROUTE_DIR_DOWN_RIGHT)
        return coord_new_full(0, 0);
    return coord_new_full(
        ROUTE_DIRECTION_VECTORS[dir][0],
        ROUTE_DIRECTION_VECTORS[dir][1]
    );
}

int route_has_changed(route_t* p, 
    const coord_t* from, const coord_t* to, float angle_threshold_deg) {

    if (!p) return 0;

    float dx = (float)(coord_get_x(to) - coord_get_x(from));
    float dy = (float)(coord_get_y(to) - coord_get_y(from));
    float len = std::sqrt(dx * dx + dy * dy);
    if (len < FLOAT_EPSILON) return 0;

    float curr_x = dx / len;
    float curr_y = dy / len;

    float avg_x = p->avg_vec_x;
    float avg_y = p->avg_vec_y;
    float dot = curr_x * avg_x + curr_y * avg_y;

    dot = std::max(-1.0f, std::min(1.0f, dot));
    float angle = std::acos(dot) * (180.0f / M_PI);

    return angle > angle_threshold_deg;
}

int route_has_changed_with_angle(route_t* p, 
    const coord_t* from, const coord_t* to, 
    float angle_threshold_deg, float* out_angle_deg) {

    if (!p || !out_angle_deg) return 0;

    float dx = (float)(coord_get_x(to) - coord_get_x(from));
    float dy = (float)(coord_get_y(to) - coord_get_y(from));
    float len = std::sqrt(dx * dx + dy * dy);
    if (len < FLOAT_EPSILON) {
        *out_angle_deg = 0.0f;
        return 0;
    }

    float curr_x = dx / len;
    float curr_y = dy / len;

    if (p->vec_count == 0) {
        p->avg_vec_x = curr_x;
        p->avg_vec_y = curr_y;
        p->vec_count = 1;
        *out_angle_deg = 0.0f;
        return 0;
    }

    float avg_len = std::sqrt(
        p->avg_vec_x * p->avg_vec_x + p->avg_vec_y * p->avg_vec_y);
        
    if (avg_len < FLOAT_EPSILON) {
        *out_angle_deg = 0.0f;
        return 0;
    }

    float avg_x = p->avg_vec_x / avg_len;
    float avg_y = p->avg_vec_y / avg_len;
    float dot = curr_x * avg_x + curr_y * avg_y;

    dot = std::max(-1.0f, std::min(1.0f, dot));
    float angle = std::acos(dot) * (180.0f / M_PI);
    *out_angle_deg = angle;

    p->avg_vec_x += curr_x;
    p->avg_vec_y += curr_y;
    p->vec_count += 1;

    return angle > angle_threshold_deg;
}

int route_has_changed_by_index(route_t* p, 
    int index_from, int index_to, float angle_threshold_deg) {
    float out_angle = 0.0f;
    return route_has_changed_with_angle_by_index(p, 
        index_from, index_to, 
        angle_threshold_deg, &out_angle);
}

int route_has_changed_with_angle_by_index(route_t* p, 
    int index_from, int index_to, 
    float angle_threshold_deg, float* out_angle_deg) {

    if (!p || !out_angle_deg) return 0;
    int len = coord_list_length(p->coords);
    if (index_from < 0 || index_to < 0 || 
        index_from >= len || index_to >= len) return 0;

    const coord_t* from = coord_list_get(p->coords, index_from);
    const coord_t* to   = coord_list_get(p->coords, index_to);
    if (!from || !to) return 0;

    return route_has_changed_with_angle(p, from, to, 
        angle_threshold_deg, out_angle_deg);
}

void route_update_average_vector(route_t* p, 
    const coord_t* from, const coord_t* to) {

    if (!p) return;
    float dx = (float)(coord_get_x(to) - coord_get_x(from));
    float dy = (float)(coord_get_y(to) - coord_get_y(from));
    float len = std::sqrt(dx * dx + dy * dy);
    if (len < FLOAT_EPSILON) return;

    p->avg_vec_x += dx / len;
    p->avg_vec_y += dy / len;
    p->vec_count++;
}

void route_update_average_vector_by_index(route_t* p, 
    int index_from, int index_to) {

    if (!p) return;
    int len = coord_list_length(p->coords);
    if (index_from < 0 || index_to < 0 || 
        index_from >= len || index_to >= len) return;

    const coord_t* from = coord_list_get(p->coords, index_from);
    const coord_t* to   = coord_list_get(p->coords, index_to);
    if (!from || !to) return;

    route_update_average_vector(p, from, to);
}

bool route_reconstruct_path(route_t* route, const coord_hash_t* came_from,
                            const coord_t* start, const coord_t* goal) {
    if (!route || !came_from || !start || !goal) return false;

    coord_list_t* reversed = coord_list_new();
    // coord_t* current = coord_copy(goal);
    const coord_t* current = goal;

    while (!coord_equal(current, start)) {
        coord_list_insert(reversed, 0, current);

        const coord_t* prev = (const coord_t*)coord_hash_get(came_from, current);
        // coord_free(current);

        if (!prev) {
            coord_list_free(reversed);
            return false;  // 복원 실패
        }

        // current = coord_copy(prev);
        current = prev;
    }

    coord_list_insert(reversed, 0, start);

    int len = coord_list_length(reversed);
    for (int i = 0; i < len; ++i) {
        route_add_coord(route, (coord_t*)coord_list_get(reversed, i));
    }

    coord_list_free(reversed);
    // coord_free(current);
    return true;
}
