#include "dstar_lite_tick.h"
#include "scalar.h"

#include <float.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <thread>
#include <climits>

static void dstar_lite_tick_proxy(void* context, float dt) {
    dstar_lite_tick_update((dstar_lite_tick_t*)context, dt);
}

dstar_lite_tick_t* dstar_lite_tick_create(dstar_lite_t* dsl){
    if (!dsl) return nullptr;

    dstar_lite_tick_t* dst = new dstar_lite_tick_t{};

    dst->base = dsl;

    dst->max_time = 10.0f;

    // 1m move config
    dst->unit_m = 1.0f;
    dst->speed_sec = 1.0f;

    dst->cur_time = 0.0f;
    dst->cur_elapsed_time = 0.0f;

    dst->s_last = {};
    dst->ticked = false;

	dst->max_elapsed_time = 0.0f;     // not used in this version

    return dst;
}

dstar_lite_tick_t* dstar_lite_tick_create_full(
    dstar_lite_t* dsl, float max_time){

    if (!dsl) return nullptr;
    
    dstar_lite_tick_t* dst = new dstar_lite_tick_t{};
    dst->max_time = max_time;
    return dst;
}

void dstar_lite_tick_destroy(dstar_lite_tick_t* dst) {
    if (!dst) return;

    delete dst;
}

dstar_lite_tick_t* dstar_lite_tick_copy(const dstar_lite_tick_t* src) {
    if (!src) return NULL;

    dstar_lite_tick_t* copy = new dstar_lite_tick_t();
    

    return copy;
}

void dstar_lite_tick_reset(dstar_lite_tick_t* dst) {
    if (!dst) return;

    dst->cur_time = 0.0f;
    dst->cur_elapsed_time = 0.0f;
    dst->s_last = {};
    dst->ticked = false;
    dstar_lite_reset(dst->base);
}

void dstar_lite_tick_prepare(dstar_lite_tick_t* dst, tick_t* tk) {
    if (!dst || !tk) return;

    dst->s_last = dst->base->start;

    dst->base->real_route = route_create();
    route_add_coord(dst->base->real_route, &dst->base->start);

    dst->ticked = true;
    dst->cur_time = 0.0f;

    tick_attach(tk, dstar_lite_tick_proxy, (void*)dst);

    if (dst->base->real_route->visited_count) {
        coord_hash_destroy(dst->base->real_route->visited_count);
    }
    dst->base->real_route->visited_count = coord_hash_create_full(
        (coord_hash_copy_func) int_copy,
        (coord_hash_destroy_func) int_destroy
    );
    // dst->base->cost_fn = dstar_lite_dynamic_cost;
    dst->base->cost_fn = dstar_lite_cost;
}

void dstar_lite_tick_prepare_full(
    dstar_lite_tick_t* dst,
    float unit_m,
    float speed_sec,
    float max_time,
    tick_t* tk)
{
    if (!dst || !tk || !dst->base) return;

    dst->s_last = dst->base->start;
    dst->ticked = true;
    dst->cur_time = 0.0f;
    dst->max_time = max_time;

    dst->unit_m = unit_m;
    dst->speed_sec = speed_sec;
    dst->max_elapsed_time = unit_m / speed_sec;
    dst->cur_elapsed_time = 0.0f;

    dst->base->interval_sec = unit_m / speed_sec;

    if (!dst->base->real_route)
        dst->base->real_route = route_create();

    route_add_coord(dst->base->real_route, &dst->base->start);

    if (dst->base->real_route->visited_count)
        coord_hash_destroy(dst->base->real_route->visited_count);

    dst->base->real_route->visited_count = coord_hash_create_full(
        (coord_hash_copy_func)int_copy,
        (coord_hash_destroy_func)int_destroy
    );

    tick_attach(tk, dstar_lite_tick_proxy, (void*)dst);
}

void dstar_lite_tick_update(dstar_lite_tick_t* dst, float dt) {
    if (!dst || !dst->base) return;

    dst->cur_time += dt;
    dst->cur_elapsed_time += dt;

    const coord_t* goal = &dst->base->goal;
    coord_t start = dst->base->start;
    coord_t next = start;

    if (coord_equal(&start, goal) || dst->cur_time >= dst->max_time || dst->base->force_quit) {
        dst->ticked = false;
        route_set_success(dst->base->real_route, coord_equal(&start, goal));
        return;
    }

    float required_time = dst->unit_m / dst->speed_sec;

	int max_step = 64; // move count limit at tick update
    int step_count = 0;

    while (dst->cur_elapsed_time >= required_time && step_count++ < max_step) {
        dst->cur_elapsed_time -= required_time;

        float* rhs_start_ptr = (float*)coord_hash_get(dst->base->rhs_table, &start);
        float rhs_start = rhs_start_ptr ? *rhs_start_ptr : FLT_MAX;
        if (scalar_equal(rhs_start, FLT_MAX)) {
            route_set_success(dst->base->real_route, false);
            dst->ticked = false;
            return;
        }

        bool found = dstar_lite_fetch_next(dst->base, &start, &next);
        if (!found || coord_equal(&next, &start)) {
            route_set_success(dst->base->real_route, false);
            dst->ticked = false;
            return;
        }

        dst->base->start = next;
        dstar_lite_update_vertex(dst->base, &next);
        route_add_coord(dst->base->real_route, &next);

        int visit_count = 0;
        if (coord_hash_contains(dst->base->real_route->visited_count, &next)) {
            visit_count = *(int*)coord_hash_get(dst->base->real_route->visited_count, &next);
            visit_count++;
        }
        int* visit_ptr = new int(visit_count);
        coord_hash_insert(dst->base->real_route->visited_count, &next, visit_ptr);
        delete visit_ptr;

        if (dst->base->move_fn)
            dst->base->move_fn(&next, dst->base->move_fn_userdata);

        if (dst->base->changed_coords_fn) {
            coord_list_t* changed = dst->base->changed_coords_fn(dst->base->changed_coords_fn_userdata);
            if (changed) {
                dst->base->km += dst->base->heuristic_fn(&dst->s_last, &start, NULL);
                dst->s_last = start;
                for (int i = 0; i < coord_list_length(changed); ++i) {
                    const coord_t* c = coord_list_get(changed, i);
                    dstar_lite_update_vertex(dst->base, c);
                }
                coord_list_destroy(changed);
            }
        }

        dstar_lite_compute_shortest_route(dst->base);

        if (coord_equal(&next, goal)) {
            route_set_success(dst->base->real_route, true);
            dst->ticked = false;
            return;
        }
    }

    if (dst->cur_time >= dst->max_time) {
        route_set_success(dst->base->real_route, coord_equal(&start, goal));
        dst->ticked = false;
    }
}

void dstar_lite_tick_complete(dstar_lite_tick_t* dst, tick_t* tk) {
    if (!dst || !tk) return;
    tick_request_detach(tk, dstar_lite_tick_proxy, dst);
    dst->ticked = false;
}
