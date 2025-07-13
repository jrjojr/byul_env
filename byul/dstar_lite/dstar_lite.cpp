// byul.h
//
// Copyright (c) 2025 별이아빠 (byuldev@outlook.kr)
// This file is part of the Byul World project.
// Licensed under the Byul World 공개 라이선스 v1.0
// See the LICENSE file for details.

#include "internal/dstar_lite.h"
#include "internal/map.h"
#include "internal/route.h"
#include "internal/dstar_lite_pqueue.h"
#include "internal/dstar_lite_key.h"

#include <float.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <thread>

static int auto_max_range(const coord_t* start, const coord_t* goal) {
    int dx = abs(goal->x - start->x);
    int dy = abs(goal->y - start->y);
    return dx + dy;
}

static int auto_compute_max_retry(const coord_t* start, const coord_t* goal) {
    int dx = abs(goal->x - start->x);
    int dy = abs(goal->y - start->y);
    int r = dx * dy;
    return r;
}

static int auto_reconstruct_max_retry(const coord_t* start, const coord_t* goal) {
    int dx = abs(goal->x - start->x);
    int dy = abs(goal->y - start->y);
    int r = dx * 2 + dy * 2;
    return r;
}

float dstar_lite_cost(
    const map_t* m, const coord_t* start, const coord_t* goal, void* userdata) {

    if (!m || !start || !goal)
        return FLT_MAX;

    if (m->is_coord_blocked_fn(m, goal->x, goal->y, nullptr))
        return FLT_MAX;

    float dx = (float)(start->x - goal->x);
    float dy = (float)(start->y - goal->y);
    return hypotf(dx, dy);  // ✅ 더 안전한 방식
}

cost_func dstar_lite_get_cost_func(const dstar_lite_t* dsl) {
    return dsl->cost_fn;
}

void dstar_lite_set_cost_func(dstar_lite_t* dsl, cost_func fn) {
    if (!dsl) return;
    dsl->cost_fn = fn;
}

void* dstar_lite_get_cost_func_userdata(const dstar_lite_t* dsl) {
    return dsl->cost_fn_userdata;
}

void dstar_lite_set_cost_func_userdata(dstar_lite_t* dsl, void* userdata) {
    if (!dsl) return;
    dsl->cost_fn_userdata = userdata;
}

bool dstar_lite_is_blocked(
    dstar_lite_t* dsl, int x, int y, void* userdata) {
        
    if (!dsl || !dsl->is_blocked_fn) return false;
    return dsl->m->is_coord_blocked_fn(dsl->m, x, y, nullptr);
}

is_coord_blocked_func dstar_lite_get_is_blocked_func(dstar_lite_t* dsl) {
    if (!dsl) return NULL;
    return dsl->is_blocked_fn;
}

void dstar_lite_set_is_blocked_func(
    dstar_lite_t* dsl, is_coord_blocked_func fn) {
    if (!dsl) return;
    dsl->is_blocked_fn = fn;
}

void* dstar_lite_get_is_blocked_func_userdata(dstar_lite_t* dsl) {
    if (!dsl) return NULL;
    return dsl->is_blocked_fn_userdata;
}

void dstar_lite_set_is_blocked_func_userdata(
    dstar_lite_t* dsl, void* userdata) {
    if (!dsl) return;
    dsl->is_blocked_fn_userdata = userdata;
}

float dstar_lite_heuristic(
    const coord_t* start, const coord_t* goal, void* userdata) {

    if (!start || !goal)
        return FLT_MAX;

    float dx = (float)(start->x - goal->x);
    float dy = (float)(start->y - goal->y);
    return hypotf(dx, dy);  // ✅ 더 정확하고 안정적
}

heuristic_func dstar_lite_get_heuristic_func(const dstar_lite_t* dsl) {
    return dsl->heuristic_fn;
}

void dstar_lite_set_heuristic_func(dstar_lite_t* dsl, heuristic_func func) {
    dsl->heuristic_fn = func;
}

void* dstar_lite_get_heuristic_func_userdata(const dstar_lite_t* dsl) {
    return dsl->heuristic_fn_userdata;
}

void dstar_lite_set_heuristic_func_userdata(dstar_lite_t* dsl, void* userdata) {
    dsl->heuristic_fn_userdata = userdata;
}

void move_to(const coord_t* c, void* userdata) {
    printf("move to (%d, %d) in finder.\n", c->x, c->y);
    // coord_free(c);
}

move_func dstar_lite_get_move_func(const dstar_lite_t* dsl) {
    if (!dsl) return NULL;
    return dsl->move_fn;
}

void dstar_lite_set_move_func(dstar_lite_t* dsl, move_func fn) {
    if (!dsl) return;
    dsl->move_fn = fn;    
}

void* dstar_lite_get_move_func_userdata(const dstar_lite_t* dsl) {
    if (!dsl) return NULL;
    return dsl->move_fn_userdata;
}

void dstar_lite_set_move_func_userdata(
    dstar_lite_t* dsl, void* userdata) {
    
    if (!dsl) return;
    dsl->move_fn_userdata = userdata;
}

coord_list_t* get_changed_coords(void* userdata) {
    if (!userdata) {
        // g_warning("changed_coords: userdata is NULL");
        printf("changed_coords: userdata is NULL\n");
        return NULL;
    }

    coord_list_t* original = (coord_list_t*)userdata;
    coord_list_t* copy = NULL;

    int len = coord_list_length(original);
    for (int i =0 ; i < len; i++){
        const coord_t* c = coord_list_get(original, i);
        coord_list_push_back(copy, c);
    }

    printf("changed_coords: %d changed coords copied and returned.\n",
        coord_list_length(copy));
    return copy;
}

changed_coords_func dstar_lite_get_changed_coords_func(
    const dstar_lite_t* dsl) {

    if (!dsl) return NULL;
    return dsl->changed_coords_fn;
}

void dstar_lite_set_changed_coords_func(
    dstar_lite_t* dsl, changed_coords_func fn) {

    if (!dsl) return;
    dsl->changed_coords_fn = fn;
}

void* dstar_lite_get_changed_coords_func_userdata(
    const dstar_lite_t* dsl) {

    if (!dsl) return NULL;
    return dsl->changed_coords_fn_userdata;
}

void dstar_lite_set_changed_coords_func_userdata(
    dstar_lite_t* dsl, void* userdata) {

    if (!dsl) return;
    dsl->changed_coords_fn_userdata = userdata;
}

dstar_lite_t* dstar_lite_new(map_t* m) {
    if (!m) return NULL;

    coord_t* c = coord_new();
    dstar_lite_t* dsl = dstar_lite_new_full(m, c,
        dstar_lite_cost, dstar_lite_heuristic,        
        false);
    coord_free(c);

    return dsl;
}

dstar_lite_t* dstar_lite_new_full(map_t* m, coord_t* start,
    cost_func cost_fn, heuristic_func heuristic_fn,
    bool debug_mode_enabled) {

    if (!m) return NULL;

    dstar_lite_t* dsl = new dstar_lite_t();
    dsl->m = m;
    // printf("[dsl->m assigned] %p\n", m);

    dsl->start = coord_copy(start);
    dsl->goal = coord_copy(start);
    
    dsl->km = 0.0f;
    dsl->max_range = 100;
    
    dsl->real_loop_max_retry = 3000;

    dsl->compute_max_retry = 3000;
    
    dsl->reconstruct_max_retry = 300;

    dsl->cost_fn = cost_fn ? cost_fn : dstar_lite_cost;
    dsl->heuristic_fn = heuristic_fn ? heuristic_fn : dstar_lite_heuristic;

    dsl->debug_mode_enabled = debug_mode_enabled;

    dsl->g_table = coord_hash_new_full(
        (coord_hash_copy_func) float_copy,
        (coord_hash_free_func) float_free
    );

    dsl->rhs_table = coord_hash_new_full(
        (coord_hash_copy_func) float_copy,
        (coord_hash_free_func) float_free        
    );

    dsl->update_count_table = coord_hash_new();

    dsl->frontier = dstar_lite_pqueue_new();

    dsl->interval_msec = 0;
    dsl->proto_route = NULL;
    dsl->real_route = NULL;

    dsl->move_fn = NULL;
    dsl->move_fn_userdata = NULL;

    dsl->changed_coords_fn = NULL;
    dsl->changed_coords_fn_userdata = NULL;

    dsl->force_quit = false;

    dsl->proto_compute_retry_count = 0;
    dsl->real_compute_retry_count = 0;

    dsl->reconstruct_retry_count = 0;

    dsl->real_loop_retry_count = 0;

    return dsl;
}

void dstar_lite_free(dstar_lite_t* dsl) {
    if (!dsl) return;
    coord_free(dsl->start);
    coord_free(dsl->goal);
    coord_hash_free(dsl->g_table);
    coord_hash_free(dsl->rhs_table);
    coord_hash_free(dsl->update_count_table);
    dstar_lite_pqueue_free(dsl->frontier);
    if (dsl->proto_route) route_free(dsl->proto_route);
    if (dsl->real_route) route_free(dsl->real_route);
    delete dsl;
}

dstar_lite_t* dstar_lite_copy(dstar_lite_t* src) {
    if (!src) return NULL;

    dstar_lite_t* copy = new dstar_lite_t();
    
    // 맵과 좌표 복사
    copy->m     = map_copy(src->m);
    copy->start = coord_copy(src->start);
    copy->goal  = coord_copy(src->goal);
    copy->km    = src->km;

    // 테이블 복사 (key: coord_t**, value: float*)
    copy->g_table = coord_hash_copy(src->g_table);

    copy->rhs_table = coord_hash_copy(src->rhs_table);

    // 우선순위 큐 복사 (주의: shallow copy로 충분한지에 따라 결정)
    copy->frontier = dstar_lite_pqueue_copy(src->frontier);

    // 콜백 함수와 유저 데이터는 그대로 유지
    copy->cost_fn               = src->cost_fn;
    copy->cost_fn_userdata      = src->cost_fn_userdata;
    copy->is_blocked_fn         = src->is_blocked_fn;
    copy->is_blocked_fn_userdata= src->is_blocked_fn_userdata;
    copy->heuristic_fn          = src->heuristic_fn;
    copy->heuristic_fn_userdata = src->heuristic_fn_userdata;
    copy->move_fn               = src->move_fn;
    copy->move_fn_userdata      = src->move_fn_userdata;
    copy->changed_coords_fn     = src->changed_coords_fn;
    copy->changed_coords_fn_userdata = src->changed_coords_fn_userdata;

    // 경로 복사
    copy->proto_route = route_copy(src->proto_route);
    copy->real_route  = route_copy(src->real_route);

    // 일반 설정 복사
    copy->interval_msec           = src->interval_msec;
    copy->real_loop_max_retry     = src->real_loop_max_retry;
    copy->compute_max_retry       = src->compute_max_retry;
    copy->reconstruct_max_retry   = src->reconstruct_max_retry;
    copy->proto_compute_retry_count = src->proto_compute_retry_count;
    copy->real_compute_retry_count  = src->real_compute_retry_count;
    copy->real_loop_retry_count     = src->real_loop_retry_count;
    copy->reconstruct_retry_count   = src->reconstruct_retry_count;
    copy->force_quit             = src->force_quit;
    copy->max_range              = src->max_range;
    copy->debug_mode_enabled     = src->debug_mode_enabled;

    // update_count_table 복사 (key: coord_t**, value: int*)
    copy->update_count_table = coord_hash_copy(src->update_count_table);

    return copy;
}

coord_t* dstar_lite_get_start(const dstar_lite_t* dsl) {
    return dsl->start;
}

void dstar_lite_set_start(dstar_lite_t* dsl, const coord_t* c) {
    // dsl->start = c;
    coord_set(dsl->start, c->x, c->y);
}

coord_t* dstar_lite_get_goal(const dstar_lite_t* dsl) {
    return dsl->goal;
}

void dstar_lite_set_goal(dstar_lite_t* dsl, const coord_t* c) {
    // dsl->goal = c;
    coord_set(dsl->goal, c->x, c->y);
}

coord_hash_t* dstar_lite_get_g_table(const dstar_lite_t* dsl) {
    return dsl->g_table;
}

coord_hash_t* dstar_lite_get_rhs_table(const dstar_lite_t* dsl) {
    return dsl->rhs_table;
}

dstar_lite_pqueue_t* dstar_lite_get_frontier(const dstar_lite_t* dsl) {
    return dsl->frontier;
}

void dstar_lite_set_frontier(dstar_lite_t* dsl, dstar_lite_pqueue_t* frontier) {
    if (dsl->frontier) dstar_lite_pqueue_free(dsl->frontier);
    dsl->frontier = frontier;
}

float dstar_lite_get_km(const dstar_lite_t* dsl) {
    return dsl->km;
}

void dstar_lite_set_km(dstar_lite_t* dsl, float km) {
    dsl->km = km;
}

int dstar_lite_get_max_range(const dstar_lite_t* dsl) {
    return dsl->max_range;
}

void dstar_lite_set_max_range(dstar_lite_t* dsl, int value) {
    dsl->max_range = value;
}


int dstar_lite_get_real_loop_max_retry(const dstar_lite_t* dsl) {
    return dsl->real_loop_max_retry;
}
void dstar_lite_set_real_loop_max_retry(dstar_lite_t* dsl, int value) {
    dsl->real_loop_max_retry = value;
}
int dstar_lite_real_loop_retry_count(const dstar_lite_t* dsl) {
    return dsl->real_loop_retry_count;
}

int dstar_lite_get_compute_max_retry(const dstar_lite_t* dsl) {
    return dsl->compute_max_retry;
}
void dstar_lite_set_compute_max_retry(
    dstar_lite_t* dsl, int v) {
        
    dsl->compute_max_retry = v;
}

int dstar_lite_proto_compute_retry_count(const dstar_lite_t* dsl) {
    return dsl->proto_compute_retry_count;
}
int dstar_lite_real_compute_retry_count(const dstar_lite_t* dsl) {
    return dsl->real_compute_retry_count;
}

int dstar_lite_get_reconstruct_max_retry(const dstar_lite_t* dsl) {
    return dsl->reconstruct_max_retry;
}

void dstar_lite_set_reconstruct_max_retry(
    dstar_lite_t* dsl, int v) {
    dsl->reconstruct_max_retry = v;
}

int dstar_lite_reconstruct_retry_count(const dstar_lite_t* dsl) {
    return dsl->reconstruct_retry_count;
}

bool dstar_lite_get_debug_mode_enabled(const dstar_lite_t* dsl) {
    return dsl->debug_mode_enabled;
}

void dstar_lite_set_debug_mode_enabled(dstar_lite_t* dsl, bool enabled) {
    dsl->debug_mode_enabled = enabled;
}

coord_hash_t* dstar_lite_get_update_count_table(const dstar_lite_t* dsl) {
    return dsl->update_count_table;
}

void dstar_lite_add_update_count(dstar_lite_t* dsl, const coord_t* c) {
    void* val = coord_hash_get(dsl->update_count_table, c);
    if (!val) {
        int* count =  new int(1);
        *count = 1;
        coord_hash_replace(dsl->update_count_table, c, count);
        delete count;
    } else {
        (*(int*)val)++;
    }
}

void dstar_lite_clear_update_count(dstar_lite_t* dsl) {
    coord_hash_remove_all(dsl->update_count_table);
}

int dstar_lite_get_update_count(dstar_lite_t* dsl, const coord_t* c) {
    void* val = coord_hash_get(dsl->update_count_table, c);
    return val ? *((int*)val) : 0;
}

const map_t* dstar_lite_get_map(const dstar_lite_t* dsl) {
    return dsl->m;
}

void    dstar_lite_set_map(dstar_lite_t* dsl, map_t* m) {
    dsl->m = m;
}

const route_t* dstar_lite_get_proto_route(const dstar_lite_t* dsl) {
    return dsl->proto_route;
}

const route_t* dstar_lite_get_real_route(const dstar_lite_t* dsl) {
    return dsl->real_route;
}

void dstar_lite_reset(dstar_lite_t* dsl) {
    // 🔥 해시테이블 완전 삭제 및 재생성
    coord_hash_free(dsl->g_table);
    coord_hash_free(dsl->rhs_table);
    coord_hash_free(dsl->update_count_table);

    dsl->g_table = coord_hash_new_full(
        (coord_hash_copy_func) float_copy,
        (coord_hash_free_func) float_free        
    );

    dsl->rhs_table = coord_hash_new_full(
        (coord_hash_copy_func) float_copy,
        (coord_hash_free_func) float_free        
    );

    dsl->update_count_table = coord_hash_new();

    if (dsl->proto_route) {
        route_free(dsl->proto_route);
        dsl->proto_route = NULL;
    }

    if (dsl->real_route) {
        route_free(dsl->real_route);
        dsl->real_route = NULL;
    }    
        
    // ♻️ 우선순위 큐도 완전 교체
    dstar_lite_pqueue_free(dsl->frontier);
    dsl->frontier = dstar_lite_pqueue_new();

    // 🎯 시작 / 목표 좌표 초기화
    // coord_set(dsl->start, 0, 0);
    // coord_set(dsl->goal, 0, 0);

    // ⚙️ 설정 값 초기화
    // dsl->km = 0.0f;
    // dsl->max_range = 0;
    // dsl->real_loop_max_retry = 0;
    // dsl->interval_msec = 0;

    dsl->proto_compute_retry_count = 0;
    dsl->real_compute_retry_count = 0;
    dsl->reconstruct_retry_count = 0;
    dsl->real_loop_retry_count = 0;    

    dstar_lite_init(dsl);
}

void dstar_lite_set_interval_msec(dstar_lite_t* dsl, int msec) {
    if (!dsl) return;
    dsl->interval_msec = msec;
}

int dstar_lite_get_interval_msec(const dstar_lite_t* dsl) {
    return dsl ? dsl->interval_msec : 0;
}

dstar_lite_key_t* dstar_lite_calculate_key(dstar_lite_t* dsl, const coord_t* s) {
    float g_val = FLT_MAX;
    float rhs_val = FLT_MAX;

    void* g_val_ptr = coord_hash_get(dsl->g_table, s);
    if (g_val_ptr != NULL) {
        g_val = *((float*)g_val_ptr);
    }

    void* rhs_val_ptr = coord_hash_get(dsl->rhs_table, s);
    if (rhs_val_ptr != NULL) {
        rhs_val = *((float*)rhs_val_ptr);
    }

    float k2 = fminf(g_val, rhs_val);
    float h = dsl->heuristic_fn(dsl->start, s, NULL);
    float k1 = k2 + h + dsl->km;

    dstar_lite_key_t* key = dstar_lite_key_new_full( k1, k2 );
    return key;
}

void dstar_lite_init(dstar_lite_t* dsl) {
    dsl->km = 0.0f;

    // for s in all_states:
    //     g[s] = float('inf')
    //     rhs[s] = float('inf')    
    // dstar_lite_cost()함수가 기본적으로 무한대를 할당하고 있다.

    // rhs[goal] = 0    
    float* rhs_goal_ptr = new float();
    *rhs_goal_ptr = 0.0f;
    coord_hash_replace(dsl->rhs_table, dsl->goal, rhs_goal_ptr);
    delete rhs_goal_ptr;
    
    // U.insert(goal, calculate_key(goal))
    dstar_lite_key_t* calc_key_goal = dstar_lite_calculate_key(dsl, dsl->start);
    dstar_lite_pqueue_push(dsl->frontier, calc_key_goal, dsl->goal);
    dstar_lite_key_free(calc_key_goal);
}

void dstar_lite_update_vertex(dstar_lite_t* dsl, const coord_t* u) {
    if (!dsl || !u) return;

    // ✅ 디버그용: update 횟수 기록
    if (dsl->debug_mode_enabled)
        dstar_lite_add_update_count(dsl, u);

    float min_rhs = FLT_MAX;
    coord_list_t* successors_s = NULL;
    
    float* g_s_ptr = NULL;
    float g_s = FLT_MAX;

    float cost =  FLT_MAX;

    float* g_u_ptr = NULL;
    float g_u = FLT_MAX;

    float* rhs_u_ptr = NULL;
    float rhs_u = FLT_MAX;

    if (!coord_equal(u, dsl->goal)) {
        successors_s = map_clone_neighbors_all(dsl->m, u->x, u->y);
        int len = coord_list_length(successors_s);
        for(int i=0; i<len; i++){
            const coord_t* s = coord_list_get(successors_s, i);

            g_s_ptr = (float*)coord_hash_get(dsl->g_table, s);
            if (g_s_ptr) {
                g_s = *g_s_ptr;
            } else {
                g_s = FLT_MAX;
            }

            cost = dsl->cost_fn(dsl->m, u, s, NULL) + g_s;

            if (cost < min_rhs)
                min_rhs = cost;
        }
        coord_list_free(successors_s);

        float* new_rhs_ptr = new float(1.0);        
        *new_rhs_ptr = min_rhs;
        coord_hash_replace(dsl->rhs_table, u, new_rhs_ptr);
        delete new_rhs_ptr;
    }

    if (dstar_lite_pqueue_contains(dsl->frontier, u)) 
        dstar_lite_pqueue_remove(dsl->frontier, u);

    g_u_ptr = (float*) coord_hash_get(dsl->g_table, u);
    if (g_u_ptr) {
        g_u = *g_u_ptr;
    } else {
        g_u = FLT_MAX;
    }

    rhs_u_ptr = (float*) coord_hash_get(dsl->rhs_table, u);
    if (rhs_u_ptr) {
        rhs_u = *(float*)rhs_u_ptr;
    } else {
        rhs_u = FLT_MAX;
    }

    if (!float_equal(g_u, rhs_u)) {
        dstar_lite_key_t* key = dstar_lite_calculate_key(dsl, u);
        dstar_lite_pqueue_push(dsl->frontier, key, u);
        dstar_lite_key_free(key);
    }
}        

void dstar_lite_update_vertex_range(dstar_lite_t* dsl, 
    const coord_t* s, int max_range) {

    if (!dsl) return;

    if (max_range < 1) {
        dstar_lite_update_vertex(dsl, s);
        return;
    }    

    coord_list_t* neighbors = map_clone_neighbors_all_range(dsl->m, 
        s->x, s->y, max_range);

    int len = coord_list_length(neighbors);
    for(int i=0; i<len; i++) {
        const coord_t* c = coord_list_get(neighbors, i);
        dstar_lite_update_vertex(dsl, c);
    }

    coord_list_free(neighbors);
}

void dstar_lite_update_vertex_auto_range(
    dstar_lite_t* dsl, const coord_t* s) {
    
    int max_range = dsl->max_range;
    return dstar_lite_update_vertex_range(dsl, s, max_range);
}

void dstar_lite_update_vertex_by_route(dstar_lite_t* dsl, const route_t* p) {
    if (!dsl || !p) return;

    const coord_list_t* coords = route_get_coords(p);
    int len = coord_list_length(coords);
    for(int i=0; i<len; i++){
        const coord_t* c = coord_list_get(coords, i);
        dstar_lite_update_vertex(dsl, c);
    }
}

void dstar_lite_compute_shortest_route(dstar_lite_t* dsl) {
    if (!dsl) return;

    coord_t* u = NULL;

    float* g_u_ptr = NULL;
    float g_u = FLT_MAX;
    
    float* rhs_u_ptr = NULL;
    float rhs_u = FLT_MAX;

    dstar_lite_key_t* calc_key = NULL;

    coord_list_t* predecessors_u = NULL;

    float g_old = FLT_MAX;

    float cost_s_u = FLT_MAX;
    float* rhs_s_ptr = NULL;
    float rhs_s = FLT_MAX;

    float min = FLT_MAX;   

    float min_old = FLT_MAX;
    float cost_s_s_prime = FLT_MAX;
    
    float* g_s_prime_ptr = NULL;
    float g_s_prime = FLT_MAX;    

    coord_list_t* successors_s = NULL;

    float g_start = FLT_MAX;
    float rhs_start = FLT_MAX;
    float* g_start_ptr = NULL;
    float* rhs_start_ptr = NULL;

    int loop = 0;

    dstar_lite_key_t* calc_key_start = NULL;
    dstar_lite_key_t* k_old = NULL;
    dstar_lite_key_t* top_key = dstar_lite_pqueue_top_key(dsl->frontier);
    do {
        loop++;
        // printf("dstar_lite_compute_shortest_route "
        //     "내부에서 루프 %d 시작.\n", loop);

        // print_all_dsl_internal(
        //     m, start, goal, km, g_table, rhs_table, frontier);

        if (k_old) {
            dstar_lite_key_free(k_old);
            k_old = NULL;
        }
        k_old = dstar_lite_key_copy(top_key);
        u = dstar_lite_pqueue_pop(dsl->frontier);
        if (!u) {
            // dstar_lite_key_free(k_old);
            break;
        }

        g_u_ptr = (float*) coord_hash_get(dsl->g_table, u);
        if (g_u_ptr) {
            g_u = *(float*)g_u_ptr;
        } else {
            g_u = FLT_MAX;
        }

        rhs_u_ptr = (float*) coord_hash_get(dsl->rhs_table, u);
        if (rhs_u_ptr) {
            rhs_u = *rhs_u_ptr;
        } else {
            rhs_u = FLT_MAX;
        }

        if(calc_key) {
            dstar_lite_key_free(calc_key);
            calc_key = NULL;
        }
        calc_key = dstar_lite_calculate_key(dsl, u);
        if (dstar_lite_key_compare(k_old, calc_key) < 0) {
            dstar_lite_pqueue_push(dsl->frontier, calc_key, u);
        } else if ( g_u > rhs_u) {
            // g_u = rhs_u;
            
            float* new_g = new float();
            *new_g = rhs_u;
            coord_hash_replace(dsl->g_table, u, new_g);
            delete new_g;

            // for s in predecessors(u):
            //     update_vertex(s)
            predecessors_u = map_clone_neighbors_all(
                dsl->m, u->x, u->y);

            int len = coord_list_length(predecessors_u);
            for (int i=0; i<len; i++){
                const coord_t* s = coord_list_get(predecessors_u, i);
                dstar_lite_update_vertex(dsl, s);
            }

            coord_list_free(predecessors_u);
        } else {
            float* g_old_ptr = (float*) coord_hash_get(dsl->g_table, u);
            if (g_old_ptr) {
                g_old = *g_old_ptr;
            } else {
                g_old = FLT_MAX;
            }

            float* new_g = new float();
            *new_g = FLT_MAX;
            coord_hash_replace(dsl->g_table, u, new_g);
            delete new_g;

            // for s in predecessors(u) | {u}:            
            predecessors_u = map_clone_neighbors_all(
                dsl->m, u->x, u->y);
            coord_list_push_back(predecessors_u, u);

            int len = coord_list_length(predecessors_u);
            for (int i=0; i<len; i++){
                const coord_t* s = coord_list_get(predecessors_u, i);

                cost_s_u = dsl->cost_fn(dsl->m, s, u, NULL);

                rhs_s_ptr = (float*) coord_hash_get(dsl->rhs_table, s);
                if (rhs_s_ptr) {
                    rhs_s = *rhs_s_ptr;
                } else {
                    rhs_s = FLT_MAX;
                }

                if (float_equal(rhs_s, cost_s_u + g_old)) {
                    if (!coord_equal(s, dsl->goal)) {

                        successors_s = map_clone_neighbors_all(
                            dsl->m, s->x, s->y);
                        
                        int len = coord_list_length(successors_s);
                        for (int i=0; i<len; i++){
                            const coord_t* s_prime = coord_list_get(successors_s, i);

                            cost_s_s_prime = dsl->cost_fn(dsl->m, s, s_prime, NULL);
                            g_s_prime_ptr = (float*) coord_hash_get(
                                dsl->g_table, s_prime);

                            if ( g_s_prime_ptr ) {
                                g_s_prime = *g_s_prime_ptr;
                            } else {
                                g_s_prime = FLT_MAX;
                            }

                            min = fminf(min, cost_s_s_prime + g_s_prime);

                            float* min_ptr = new float(1.0);
                            *min_ptr = min;
                            coord_hash_replace(dsl->rhs_table, s, min_ptr);
                            delete min_ptr;
                        }
                        coord_list_free(successors_s);
                    }
                    dstar_lite_update_vertex(dsl, s);
                }
            }
            coord_list_free(predecessors_u);
        }
        coord_free(u);

        if (top_key) dstar_lite_key_free(top_key);
        top_key = dstar_lite_pqueue_top_key(dsl->frontier);
        if (!top_key) break;

        if (calc_key_start) {
            dstar_lite_key_free(calc_key_start);
            calc_key_start = NULL;
        }
        calc_key_start = dstar_lite_calculate_key(dsl, dsl->start);
                    
        g_start_ptr = (float*)coord_hash_get(dsl->g_table, dsl->start);
        if ( g_start_ptr ) {
            g_start = *g_start_ptr;
        } else {
            g_start = FLT_MAX;
        }

        rhs_start_ptr = (float*)coord_hash_get(
            dsl->rhs_table, dsl->start);

        if ( rhs_start_ptr ) {
            rhs_start = *rhs_start_ptr;
        } else {
            rhs_start = FLT_MAX;
        }

    } while ((loop < dsl->compute_max_retry) && 
        ( (dstar_lite_key_compare(top_key, calc_key_start) < 0) || 
            (!float_equal(rhs_start, g_start) ) ) );

    if (calc_key_start) {
        dstar_lite_key_free(calc_key_start);
        calc_key_start = NULL;
    }            
    if (k_old) {
        dstar_lite_key_free(k_old);
        k_old = NULL;
    }
    if(calc_key) {
        dstar_lite_key_free(calc_key);
        calc_key = NULL;
    }
    if(top_key) dstar_lite_key_free(top_key);

    // if dsl->proto_route == NULL
    if (dsl->proto_route == NULL) {
        dsl->proto_compute_retry_count = loop;
    } else {
        dsl->real_compute_retry_count = loop;
    }
}

route_t* dstar_lite_reconstruct_route(dstar_lite_t* dsl) {
    if (!dsl) return NULL;

    route_t* p = route_new();
    route_add_coord(p, dsl->start);    

    float* g_start_ptr = (float*) coord_hash_get(dsl->g_table, dsl->start);
    if (!g_start_ptr || float_equal(*g_start_ptr, FLT_MAX)) {
        if (dsl->debug_mode_enabled) {
            // p->visited_count = coord_hash_copy(dsl->update_count_table);
            p->total_retry_count = dstar_lite_proto_compute_retry_count(dsl);
        }
        return p;
    }

    coord_t* current = coord_copy(dsl->start);
    // coord_t* current = start;

    int loop = 0;
    while (!coord_equal(current, dsl->goal) && 
        (loop < dsl->reconstruct_max_retry)) {

        loop++;
        // printf("dstar_lite_reconstruct_route "
        //     "내부에서 루프 %d 시작.\n", loop);

        coord_list_t* neighbors = map_clone_neighbors_all(
            dsl->m, current->x, current->y);

        int len = coord_list_length(neighbors);

        float min_cost = FLT_MAX;
        coord_t* next = NULL;

        for (int i=0; i<len; i++){
            const coord_t* s = coord_list_get(neighbors, i);

            float cost_current_s = dsl->cost_fn(dsl->m, current, s, NULL);
            float* g_s_ptr = (float*) coord_hash_get(dsl->g_table, s);
            float g_s = (g_s_ptr) ? *g_s_ptr : FLT_MAX;

            float total_cost = cost_current_s + g_s;
            if (total_cost < min_cost) {
                min_cost = total_cost;
                
                if (next) coord_free(next);  // 기존 next 해제
                next = coord_copy(s);
                // next = s;
            }
        }
        coord_list_free(neighbors);

        if (!next) {  // 더 이상 진행할 수 없음
            coord_free(current);
            // route_free(p);
            route_set_success(p, false);
            if (dsl->debug_mode_enabled){
                // p->visited_count = coord_hash_copy(dsl->update_count_table);
    p->total_retry_count = dstar_lite_proto_compute_retry_count(dsl);                
            }
            return p;
        }

        float* g_next_ptr = (float*) coord_hash_get(dsl->g_table, next);
        if (!g_next_ptr || float_equal(*g_next_ptr, FLT_MAX)) {
            coord_free(current);
            coord_free(next);
            // route_free(p);
            route_set_success(p, false);
            if (dsl->debug_mode_enabled){
                // p->visited_count = coord_hash_copy(dsl->update_count_table);  
p->total_retry_count = dstar_lite_proto_compute_retry_count(dsl);                
            }          
            return p;
        }

        route_add_coord(p, next);
        coord_free(current);
        current = next;
    }

    dsl->reconstruct_retry_count = loop;

    coord_free(current);
    route_set_success(p, true);    
    if (dsl->debug_mode_enabled){
        // p->visited_count = coord_hash_copy(dsl->update_count_table);    
p->total_retry_count = dstar_lite_proto_compute_retry_count(dsl);        
    }
    return p;
}

route_t* dstar_lite_find(dstar_lite_t* dsl) {
    if (!dsl) return NULL;

    dstar_lite_reset(dsl);

    dstar_lite_compute_shortest_route(dsl);
    return dstar_lite_reconstruct_route(dsl);
}

void dstar_lite_find_full(dstar_lite_t* dsl) {
    if (!dsl) return;
    dstar_lite_find_proto(dsl);
    dstar_lite_find_loop(dsl);
}

void dstar_lite_find_proto(dstar_lite_t* dsl) {
    if (!dsl) return;
    // dstar_lite_reset(dsl);
    dsl->proto_route = dstar_lite_find(dsl);
}
 
void dstar_lite_find_loop(dstar_lite_t* dsl) {
    if (!dsl) return;

    // if (!dsl->proto_route) dstar_lite_find_proto(dsl);

    coord_t* s_last = coord_copy(dsl->start);
    coord_t* start = coord_copy(dsl->start);

    dsl->real_route = route_new();
    route_add_coord(dsl->real_route, start);

    float* rhs_start_ptr = NULL;
    float rhs_start = FLT_MAX;

    coord_list_t* successors_start = NULL;
    coord_list_t* l = NULL;

    float min_cost = FLT_MAX;
    coord_t* next = NULL;

    coord_t* s = NULL;
    float* g_s_ptr = NULL;
    float g_s = FLT_MAX;
    float cost_start_s = FLT_MAX;
    float total_cost = FLT_MAX;

    coord_list_t* changed_coords = NULL;

    int loop = 0;
    while (!coord_equal(start, dsl->goal) && 
        (loop < dsl->real_loop_max_retry) && !dsl->force_quit) {

        loop++;
        // printf("dstar_lite_find_loop "
        //     "내부에서 루프 %d 시작.\n", loop);

        rhs_start_ptr = (float*) coord_hash_get(dsl->rhs_table, dsl->start);
        if (rhs_start_ptr) {
            rhs_start = *rhs_start_ptr;
        } else {
            rhs_start = FLT_MAX;
        }

        if (float_equal(rhs_start, FLT_MAX)) {
            route_set_success(dsl->real_route, false);
            dsl->real_loop_retry_count = loop;
            dsl->force_quit = false;
            return;
        }
        
        // 다음 이동 위치 선택
        min_cost = FLT_MAX;
        successors_start = map_clone_neighbors_all(
            dsl->m, start->x, start->y);

        int len = coord_list_length(successors_start);
        for (int i=0; i<len; i++){
            const coord_t* s = coord_list_get(successors_start, i);

            g_s_ptr = (float*)coord_hash_get(dsl->g_table, s);
            g_s = (g_s_ptr) ? *g_s_ptr : FLT_MAX;
            cost_start_s = dsl->cost_fn(dsl->m, start, s, NULL);
            total_cost = cost_start_s + g_s;

            if (total_cost < min_cost) {
                min_cost = total_cost;

                // next = s;
                if (next) coord_free(next);  // 기존 next 해제
                next = coord_copy(s);
            }
        }
        coord_list_free(successors_start);

        if (!next) {
            route_set_success(dsl->real_route, false);
            dsl->real_loop_retry_count = loop;
            dsl->force_quit = false;
            return;
        }
        route_add_coord(dsl->real_route, next);

        if (dsl->force_quit) break;
        // move callback을 실행한다.
        // if (dsl->move_fn) dsl->move_fn(coord_copy(next),
        //     dsl->move_fn_userdata);

        if (dsl->move_fn) dsl->move_fn(next,
            dsl->move_fn_userdata);            

        // 이동 간 대기 시간 적용
        // time.sleep(dsl->interval_msec / 1000.0);
        // g_usleep(dsl->interval_msec * 1000);  // 밀리초 → 마이크로초
        // if (dsl->interval_msec <= 0)
        //     //쓰레드에서도 안전하게
        //     g_thread_yield();   // 틱 양보
        // else
        //     g_usleep(dsl->interval_msec * 1000);

        if (dsl->interval_msec <= 0)
            std::this_thread::yield();  // ✅ C++ 안전한 쓰레드 양보
        else
            std::this_thread::sleep_for(std::chrono::milliseconds(dsl->interval_msec));            

        if (dsl->force_quit) break;


        if(start) coord_free(start);
        start = coord_copy(next);

        // 환경 변화 감지 및 반영
        if (dsl->changed_coords_fn) {
            changed_coords = dsl->changed_coords_fn(
                dsl->changed_coords_fn_userdata);

            if (changed_coords) {
                dsl->km += dsl->heuristic_fn(s_last, start, NULL);

                if (s_last) coord_free(s_last);
                s_last = coord_copy(start);

                // for u in changed_coords:
                    // update_vertex(u)
                int len = coord_list_length(changed_coords);
                for (int i=0; i<len; i++){
                    const coord_t* s = coord_list_get(changed_coords, i);
                    dstar_lite_update_vertex(dsl, s);
                }
            }
            // if (changed_coords) {
            //     g_list_free_full(changed_coords, (GDestroyNotify)coord_free);
            // }
            dstar_lite_compute_shortest_route(dsl);
        }
        loop++;
    }
    dsl->force_quit = false;
    if (start) coord_free(start);
    if (next) coord_free(next);

    if (loop >= dsl->real_loop_max_retry) {
        // 길을 못찾고 루프가 한계에 도달했다.
        if (s_last) {
            if (!coord_equal(s_last, dsl->goal)) {
                route_set_success(dsl->real_route, false);
                dsl->real_loop_retry_count = loop;
                coord_free(s_last);
                    dsl->force_quit = false;
                return;
            }
        }
    }
    route_set_success(dsl->real_route, true);
    dsl->real_loop_retry_count = loop;
    if (s_last) coord_free(s_last);    
        dsl->force_quit = false;
    return;    
}

void dstar_lite_force_quit(dstar_lite_t* dsl) {
    dsl->force_quit = true;
}

bool dstar_lite_is_quit_forced(dstar_lite_t* dsl) {
    return dsl->force_quit;
}

void dstar_lite_set_force_quit(dstar_lite_t* dsl, bool v) {
    dsl->force_quit = v;
}

