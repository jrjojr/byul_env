#include "route.h"
#include "internal/route_internal.h"
#include "coord.h"
#include "scalar.h"
#include "coord_list.h"

#include <vector>
#include <cmath>
#include <cstdio>
#include <algorithm>
#include <cstddef>

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

route_t* route_create(void) {
    route_t* r = nullptr;
    try {
        r = new route_t{};
        r->coords = coord_list_create();
        r->visited_order = coord_list_create();
        r->visited_count = coord_hash_create();
        if (!r->coords || !r->visited_order || !r->visited_count) {
            route_destroy(r);
            return nullptr;
        }
        return r;
    } catch (...) {
        route_destroy(r);
        return nullptr;
    }
}

void route_destroy(route_t* p) {
    if (!p) return;
    coord_list_destroy(p->coords);
    coord_list_destroy(p->visited_order);
    coord_hash_destroy(p->visited_count);
    delete p;
}

route_t* route_copy(const route_t* p) {
    route_t* copied = nullptr;
    return route_clone_ex(p, &copied) == NAVSYS_STATUS_OK
        ? copied
        : nullptr;
}

navsys_status_t route_clone_ex(
    const route_t* source,
    route_t** out_route) {
    if (!source || !out_route)
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    if (!source->coords || !source->visited_order || !source->visited_count)
        return NAVSYS_STATUS_CORRUPT_STATE;

    route_t* copied = nullptr;
    try {
        copied = new route_t{};
    } catch (...) {
        return NAVSYS_STATUS_OUT_OF_MEMORY;
    }

    navsys_status_t status =
        coord_list_copy_ex(source->coords, &copied->coords);
    if (status == NAVSYS_STATUS_OK) {
        status = coord_list_copy_ex(
            source->visited_order, &copied->visited_order);
    }
    if (status == NAVSYS_STATUS_OK) {
        status = coord_hash_copy_ex(
            source->visited_count, &copied->visited_count);
    }
    if (status != NAVSYS_STATUS_OK) {
        route_destroy(copied);
        return status;
    }

    copied->cost = source->cost;
    copied->success = source->success;
    copied->total_retry_count = source->total_retry_count;
    copied->avg_vec_x = source->avg_vec_x;
    copied->avg_vec_y = source->avg_vec_y;
    copied->vec_count = source->vec_count;
    *out_route = copied;
    return NAVSYS_STATUS_OK;
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

size_t route_get_coord_count(const route_t* route) {
    return route ? static_cast<size_t>(route_length(route)) : 0;
}

navsys_status_t route_fetch_coord(
    const route_t* route,
    size_t index,
    coord_t* out_coord) {
    if (!route || !out_coord)
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    if (index >= route_get_coord_count(route))
        return NAVSYS_STATUS_NOT_FOUND;

    const coord_t* coord =
        route_get_coord_at(route, static_cast<int>(index));
    if (!coord)
        return NAVSYS_STATUS_CORRUPT_STATE;
    *out_coord = *coord;
    return NAVSYS_STATUS_OK;
}

navsys_status_t route_fetch_total_cost(
    const route_t* route,
    double* out_total_cost) {
    if (!route || !out_total_cost)
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    *out_total_cost = static_cast<double>(route_get_cost(route));
    return NAVSYS_STATUS_OK;
}

navsys_status_t route_fetch_completion(
    const route_t* route,
    route_completion_t* out_completion) {
    if (!route || !out_completion)
        return NAVSYS_STATUS_INVALID_ARGUMENT;

    route_completion_t completion = ROUTE_COMPLETION_NONE;
    if (route_get_success(route)) {
        completion = ROUTE_COMPLETION_COMPLETE;
    } else if (route_get_coord_count(route) > 0) {
        completion = ROUTE_COMPLETION_PARTIAL;
    }
    *out_completion = completion;
    return NAVSYS_STATUS_OK;
}

navsys_status_t route_export_coords(
    const route_t* route,
    coord_t* output,
    size_t capacity,
    size_t* out_required_count) {
    if (!route || !out_required_count || (!output && capacity != 0))
        return NAVSYS_STATUS_INVALID_ARGUMENT;

    const size_t required = static_cast<size_t>(route_length(route));
    if (!output) {
        *out_required_count = required;
        return NAVSYS_STATUS_OK;
    }

    *out_required_count = required;
    if (capacity < required)
        return NAVSYS_STATUS_INCOMPLETE;
    for (size_t i = 0; i < required; ++i) {
        output[i] = *route_get_coord_at(route, static_cast<int>(i));
    }
    return NAVSYS_STATUS_OK;
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

coord_hash_t* route_internal_get_visited_count_mutable(route_t* route) {
    return route ? route->visited_count : nullptr;
}

navsys_status_t route_internal_replace_visited_count(
    route_t* route,
    coord_hash_t* replacement) {
    if (!route || !replacement)
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    coord_hash_t* previous = route->visited_count;
    route->visited_count = replacement;
    coord_hash_destroy(previous);
    return NAVSYS_STATUS_OK;
}

int route_get_total_retry_count(const route_t* p) {
    return p ? p->total_retry_count : 0;
}

void route_set_total_retry_count(route_t* p, int retry_count) {
    if (p) p->total_retry_count = retry_count;
}

int route_add_visited(route_t* p, const coord_t* c) {
    if (!p || !c || !p->visited_order || !p->visited_count) return 0;

    int* old_val = static_cast<int*>(coord_hash_get(p->visited_count, c));
    int count = (old_val ? *old_val : 0) + 1;

    const size_t order_size = coord_list_size(p->visited_order);
    if (coord_list_push_back_ex(p->visited_order, c) != NAVSYS_STATUS_OK)
        return 0;

    if (coord_hash_upsert_copy(
            p->visited_count, c, &count, nullptr) != NAVSYS_STATUS_OK) {
        coord_t removed{};
        (void)coord_list_remove_at_ex(
            p->visited_order, order_size, &removed);
        return 0;
    }

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
    route_t* sliced = nullptr;
    return route_slice_ex(
        p,
        static_cast<size_t>(start),
        static_cast<size_t>(end),
        &sliced) == NAVSYS_STATUS_OK
        ? sliced
        : nullptr;
}

navsys_status_t route_slice_ex(
    const route_t* source,
    size_t begin,
    size_t end,
    route_t** out_route) {
    if (!source || !out_route || begin > end)
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    if (!source->coords)
        return NAVSYS_STATUS_CORRUPT_STATE;
    if (end > coord_list_size(source->coords))
        return NAVSYS_STATUS_INVALID_ARGUMENT;

    coord_list_t* sliced_coords = nullptr;
    navsys_status_t status = coord_list_create_slice(
        source->coords, begin, end, &sliced_coords);
    if (status != NAVSYS_STATUS_OK)
        return status;

    route_t* sliced = route_create();
    if (!sliced) {
        coord_list_destroy(sliced_coords);
        return NAVSYS_STATUS_OUT_OF_MEMORY;
    }

    coord_list_destroy(sliced->coords);
    sliced->coords = sliced_coords;
    *out_route = sliced;
    return NAVSYS_STATUS_OK;
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

coord_t* route_make_direction(route_t* p, int index) {
    if (!p || coord_list_length(p->coords) < 2) return coord_create_full(0, 0);

    int len = coord_list_length(p->coords);
    if (index < 0 || index >= len) return coord_create_full(0, 0);

    const coord_t* curr = coord_list_get(p->coords, index);
    if (!curr) return coord_create_full(0, 0);

    if (index == len - 1) {
        const coord_t* prev = coord_list_get(p->coords, index - 1);
        if (!prev) return coord_create_full(0, 0);
        return coord_create_full(
            coord_get_x(curr) - coord_get_x(prev),
            coord_get_y(curr) - coord_get_y(prev)
        );
    }

    const coord_t* next = coord_list_get(p->coords, index + 1);
    if (!next) return coord_create_full(0, 0);
    return coord_create_full(
        coord_get_x(next) - coord_get_x(curr),
        coord_get_y(next) - coord_get_y(curr)
    );
}


route_dir_t route_get_direction_by_dir_coord(const coord_t* dxdy) {
    if (!dxdy) return ROUTE_DIR_UNKNOWN;
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
    coord_destroy(vec);
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
    if (!c_from || !c_to) return ROUTE_DIR_UNKNOWN;

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
    if (!c_from || !c_to) return 0.0f;

    int dx = coord_get_x(c_to) - coord_get_x(c_from);
    int dy = coord_get_y(c_to) - coord_get_y(c_from);

    if (dx == 0 && dy == 0) return 0.0f;
    return std::atan2((float)dy, (float)dx) * (180.0f / M_PI);
}

route_dir_t calc_direction(const coord_t* start, const coord_t* goal) {
    if (!start || !goal) return ROUTE_DIR_UNKNOWN;
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
        return coord_create_full(0, 0);
    return coord_create_full(
        ROUTE_DIRECTION_VECTORS[dir][0],
        ROUTE_DIRECTION_VECTORS[dir][1]
    );
}

int route_has_changed(route_t* p, 
    const coord_t* from, const coord_t* to, float angle_threshold_deg) {

    if (!p || !from || !to) return 0;

    float dx = (float)(coord_get_x(to) - coord_get_x(from));
    float dy = (float)(coord_get_y(to) - coord_get_y(from));
    float len = std::sqrt(dx * dx + dy * dy);
    if (len < SCALAR_EPSILON) return 0;

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

    if (!p || !from || !to || !out_angle_deg) return 0;

    float dx = (float)(coord_get_x(to) - coord_get_x(from));
    float dy = (float)(coord_get_y(to) - coord_get_y(from));
    float len = std::sqrt(dx * dx + dy * dy);
    if (len < SCALAR_EPSILON) {
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
        
    if (avg_len < SCALAR_EPSILON) {
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

    if (!p || !from || !to) return;
    float dx = (float)(coord_get_x(to) - coord_get_x(from));
    float dy = (float)(coord_get_y(to) - coord_get_y(from));
    float len = std::sqrt(dx * dx + dy * dy);
    if (len < SCALAR_EPSILON) return;

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

bool route_reconstruct(route_t* route, const coord_hash_t* came_from,
                            const coord_t* start, const coord_t* goal) {
    return route_reconstruct_ex(route, came_from, start, goal)
        == NAVSYS_STATUS_OK;
}

navsys_status_t route_reconstruct_ex(
    route_t* route,
    const coord_hash_t* came_from,
    const coord_t* start,
    const coord_t* goal) {
    if (!route || !came_from || !start || !goal)
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    if (!route->coords)
        return NAVSYS_STATUS_CORRUPT_STATE;

    coord_list_t* reversed = nullptr;
    navsys_status_t status = coord_list_create_ex(&reversed);
    if (status != NAVSYS_STATUS_OK)
        return status;

    const coord_t* current = goal;
    const size_t predecessor_limit = coord_hash_size(came_from);
    size_t predecessor_count = 0;

    while (!coord_equal(current, start)) {
        status = coord_list_insert_ex(reversed, 0, current);
        if (status != NAVSYS_STATUS_OK) {
            coord_list_destroy(reversed);
            return status;
        }

        const coord_t* previous = static_cast<const coord_t*>(
            coord_hash_get(came_from, current));
        if (!previous) {
            coord_list_destroy(reversed);
            return NAVSYS_STATUS_NO_PATH;
        }
        current = previous;
        ++predecessor_count;
        if (predecessor_count > predecessor_limit) {
            coord_list_destroy(reversed);
            return NAVSYS_STATUS_CORRUPT_STATE;
        }
    }

    status = coord_list_insert_ex(reversed, 0, start);
    if (status != NAVSYS_STATUS_OK) {
        coord_list_destroy(reversed);
        return status;
    }

    coord_list_t* candidate = nullptr;
    status = coord_list_copy_ex(route->coords, &candidate);
    if (status != NAVSYS_STATUS_OK) {
        coord_list_destroy(reversed);
        return status;
    }

    const size_t path_size = coord_list_size(reversed);
    for (size_t index = 0; index < path_size; ++index) {
        coord_t coordinate{};
        status = coord_list_fetch(reversed, index, &coordinate);
        if (status == NAVSYS_STATUS_OK) {
            status = coord_list_push_back_ex(candidate, &coordinate);
        }
        if (status != NAVSYS_STATUS_OK) {
            coord_list_destroy(candidate);
            coord_list_destroy(reversed);
            return status;
        }
    }

    coord_list_t* previous_coords = route->coords;
    route->coords = candidate;
    coord_list_destroy(previous_coords);
    coord_list_destroy(reversed);
    return NAVSYS_STATUS_OK;
}
