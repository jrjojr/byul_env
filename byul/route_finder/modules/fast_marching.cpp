// fast_marching.cpp

#include "internal/fast_marching.h"
#include "internal/map.h"
#include "coord.hpp"
#include "internal/coord_list.h"
#include "internal/coord_hash.h"
#include "internal/cost_coord_pq.h"
#include <float.h>
#include <cmath>
#include <stdio.h>

void* fmm_cell_copy(const void* p) {
    if (!p) return nullptr;
    auto* in = static_cast<const fmm_cell_t*>(p);
    auto* out = new fmm_cell_t;
    *out = *in;
    return out;
}

void fmm_cell_free(void* p) {
    delete static_cast<fmm_cell_t*>(p);
}

fmm_cell_t* fmm_cell_new() {
    auto* cell = new fmm_cell_t;
    cell->state = FMM_FAR;
    cell->value = 0.0f;
    return cell;
}

fmm_cell_t* fmm_cell_new_full(fmm_state_t state, float value) {
    auto* cell = new fmm_cell_t;
    cell->state = state;
    cell->value = value;
    return cell;
}

fmm_grid_t* fmm_compute(const map_t* m, const coord_t* start, 
    cost_func cost_fn, float radius_limit, int max_retry) {
    if (!m || !start) return nullptr;

    if(!cost_fn) cost_fn = default_cost;

    if (radius_limit <= 0.0f || radius_limit > MAX_RADIUS)
        radius_limit = MAX_RADIUS;

    fmm_grid_t* grid = new fmm_grid_t();
    grid->width = m->width;
    grid->height = m->height;
    grid->cells = coord_hash_new_full(
        (coord_hash_copy_func) fmm_cell_copy,
        (coord_hash_free_func) fmm_cell_free
    );

    grid->visit_order = coord_list_new();

    cost_coord_pq_t* narrow_band = cost_coord_pq_new();

    fmm_cell_t* start_cell = new fmm_cell_t{FMM_NARROW, 0.0f};
    coord_hash_replace(grid->cells, start, start_cell);
    fmm_cell_free(start_cell);

    coord_list_push_back(grid->visit_order, start);
    cost_coord_pq_push(narrow_band, 0.0f, start);

    int retry = 0;
    while (!cost_coord_pq_is_empty(narrow_band)) {
        if (max_retry > 0 && retry++ >= max_retry) break;

        coord_t* current = cost_coord_pq_pop(narrow_band);

        fmm_cell_t* current_cell = (fmm_cell_t*)coord_hash_get(
            grid->cells, current);

        if (!current_cell) current_cell = new fmm_cell_t{FMM_KNOWN, FLT_MAX};
        current_cell->state = FMM_KNOWN;
        coord_hash_replace(grid->cells, current, current_cell);
        // fmm_cell_free(current_cell);

        if (current_cell->value > radius_limit) {
            coord_free(current);
            continue;
        }

        coord_list_push_back(grid->visit_order, current);

        coord_list_t* neighbors = map_make_neighbors(
            m, current->x, current->y);

        int len = coord_list_length(neighbors);

        for (int i = 0; i < len; ++i) {
            const coord_t* next = coord_list_get(neighbors, i);
            fmm_cell_t* next_cell = (fmm_cell_t*)coord_hash_get(
                grid->cells, next);

            if (next_cell && next_cell->state == FMM_KNOWN) continue;

            float cost = cost_fn(m, current, next, nullptr);

            float min_x = FLT_MAX;
            float min_y = FLT_MAX;

            coord_t px = { next->x - 1, next->y };
            coord_t nx = { next->x + 1, next->y };
            coord_t py = { next->x, next->y - 1 };
            coord_t ny = { next->x, next->y + 1 };

            float* d;
            if ((d = (float*)coord_hash_get(grid->cells, &px))) {
                min_x = std::min(min_x, ((fmm_cell_t*)d)->value);
            }
            if ((d = (float*)coord_hash_get(grid->cells, &nx))) {
                min_x = std::min(min_x, ((fmm_cell_t*)d)->value);
            }
            if ((d = (float*)coord_hash_get(grid->cells, &py))) {
                min_y = std::min(min_y, ((fmm_cell_t*)d)->value);
            }
            if ((d = (float*)coord_hash_get(grid->cells, &ny))) {
                min_y = std::min(min_y, ((fmm_cell_t*)d)->value);
            }

            float h = cost;
            float T = 0.0f;
            float a = std::min(min_x, FLT_MAX);
            float b = std::min(min_y, FLT_MAX);

            if (fabs(a - b) >= h)
                T = std::min(a, b) + h;
            else
                T = (a + b + sqrtf(2 * h * h - (a - b) * (a - b))) / 2.0f;

            if (T > radius_limit) continue;

            if (!next_cell || T < next_cell->value) {
                fmm_cell_t* new_cell = new fmm_cell_t{FMM_NARROW, T};
                coord_hash_replace(grid->cells, next, new_cell);
                fmm_cell_free(new_cell);

                cost_coord_pq_push(narrow_band, T, next);
            }
        }

        coord_list_free(neighbors);
        coord_free(current);
    }

    cost_coord_pq_free(narrow_band);
    grid->total_retry_count = retry;
    return grid;
}

void fmm_grid_free(fmm_grid_t* grid) {
    if (!grid) return;

    coord_hash_free(grid->cells);
    coord_list_free(grid->visit_order);
    delete grid;
}

void fmm_dump_ascii(const fmm_grid_t* grid) {
    if (!grid) return;

    for (int y = 0; y < grid->height; ++y) {
        for (int x = 0; x < grid->width; ++x) {
            coord_t c = { x, y };
            fmm_cell_t* cell = (fmm_cell_t*)coord_hash_get(grid->cells, &c);
            if (!cell) {
                printf(" .. ");
            } else {
                printf("%3d", (int)(cell->value));
            }
        }
        printf("\n");
    }
    printf("\n");
}

route_t* find_fast_marching(const map_t* m,
    const coord_t* start, const coord_t* goal,
    cost_func cost_fn, int max_retry, bool visited_logging) {

    float radius = coord_distance(start, goal) * 1.5f;
    fmm_grid_t* grid = fmm_compute(m, start, cost_fn, radius, max_retry);
    if (!grid) return nullptr;

    route_t* route = route_new();

    // 방문한 셀 순서대로 기록
    if (visited_logging && grid->visit_order) {
        int vlen = coord_list_length(grid->visit_order);
        for (int i = 0; i < vlen; ++i) {
            const coord_t* c = coord_list_get(grid->visit_order, i);
            route_add_visited(route, c);
        }
    }

    fmm_cell_t* goal_cell = (fmm_cell_t*)coord_hash_get(grid->cells, goal);

    const coord_t* fallback = nullptr;
    if (!goal_cell) {
        // visit_order에서 마지막 셀을 fallback으로 사용
        int len = coord_list_length(grid->visit_order);
        if (len > 0) {
            fallback = coord_list_get(grid->visit_order, len - 1);
            goal = fallback;
            goal_cell = (fmm_cell_t*)coord_hash_get(grid->cells, goal);
            route_set_success(route, false);  // fallback이므로 실패로 간주
        } else {
            route_set_success(route, false);
            fmm_grid_free(grid);
            return route;
        }
    }

    // 역추적 시작
    coord_t* current = coord_copy(goal);
    route_insert(route, 0, current);

    while (!coord_equal(current, start)) {
        coord_list_t* neighbors = map_make_neighbors(m, current->x, current->y);
        int len = coord_list_length(neighbors);

        float best_val = FLT_MAX;
        const coord_t* best_neighbor = nullptr;

        for (int i = 0; i < len; ++i) {
            const coord_t* n = coord_list_get(neighbors, i);
            fmm_cell_t* n_cell = (fmm_cell_t*)coord_hash_get(grid->cells, n);
            if (!n_cell) continue;

            if (n_cell->value < best_val) {
                best_val = n_cell->value;
                best_neighbor = n;
            }
        }

        if (!best_neighbor) {
            route_set_success(route, false);
            coord_free(current);
            coord_list_free(neighbors);
            fmm_grid_free(grid);
            return route;
        }

        route_insert(route, 0, best_neighbor);
        coord_free(current);
        current = coord_copy(best_neighbor);
        coord_list_free(neighbors);
    }

    route_set_total_retry_count(route, grid->total_retry_count);
    coord_free(current);
    fmm_grid_free(grid);
    if (!fallback) route_set_success(route, true);
    return route;
}
