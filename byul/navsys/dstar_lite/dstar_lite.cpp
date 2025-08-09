#include "dstar_lite.h"
#include "navgrid.h"
#include "route.h"
#include "dstar_lite_pqueue.h"
#include "dstar_lite_key.h"

#include <float.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <thread>
#include <climits>

bool dstar_lite_fetch_next(const dstar_lite_t* dsl, 
    const coord_t* start, coord_t* out) {
    
    if(!dsl || !start || !out) return false;
        
    coord_list_t* successors 
    = navgrid_copy_neighbors_all(dsl->navgrid, start->x, start->y);

    int len = coord_list_length(successors);

    float min_cost = FLT_MAX;
    coord_t best = *start;  // fallback:

    for (int i = 0; i < len; i++) {
        coord_t s = *coord_list_get(successors, i);

        float g_s = FLT_MAX;
        float cost = FLT_MAX;
        float total = FLT_MAX;

        float* g_ptr = (float*)coord_hash_get(dsl->g_table, &s);
        if (g_ptr) g_s = *g_ptr;

        cost = dsl->cost_fn(
            dsl->navgrid, start, &s, dsl->proto_route->visited_count);
        total = g_s + cost;

        int visit = 0;
        if (coord_hash_contains(dsl->proto_route->visited_count, &s)) {
            visit = *(int*)coord_hash_get(dsl->proto_route->visited_count, &s);
        }

        if (total < min_cost) {
            min_cost = total;
            best = s;
        }
    }

    coord_list_destroy(successors);
    *out = best;
    return true;
}

static int auto_max_range(const coord_t* start, const coord_t* goal) {
    int dx = abs(goal->x - start->x);
    int dy = abs(goal->y - start->y);
    return dx + dy;
}

static int auto_max_retry(const coord_t* start, const coord_t* goal) {
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

float dstar_lite_cost(const navgrid_t* navgrid, 
    const coord_t* start, const coord_t* goal, void* userdata) {

    if (!navgrid || !start || !goal)
        return FLT_MAX;

    if (navgrid->is_coord_blocked_fn(navgrid, goal->x, goal->y, nullptr))
        return FLT_MAX;

    float dx = (float)(start->x - goal->x);
    float dy = (float)(start->y - goal->y);
    return hypotf(dx, dy);
}

float dstar_lite_dynamic_cost(const navgrid_t* navgrid, 
    const coord_t* start, const coord_t* goal, void* userdata) 
{
    if (!navgrid || !start || !goal)
        return FLT_MAX;

    if (navgrid->is_coord_blocked_fn &&
        navgrid->is_coord_blocked_fn(navgrid, goal->x, goal->y, NULL)) {
        return FLT_MAX;
    }

    float dx = (float)(start->x - goal->x);
    float dy = (float)(start->y - goal->y);
    float base_cost = hypotf(dx, dy);

    int visit_count = 0;
    if (userdata) {
        coord_hash_t* visit_table = (coord_hash_t*)userdata;
        int* count_ptr = (int*)coord_hash_get(visit_table, goal);
        if (count_ptr) visit_count = *count_ptr;
    }

    float penalty = 0.25f * (float)visit_count;

    return base_cost + penalty;
}


float dstar_lite_heuristic(
    const coord_t* start, const coord_t* goal, void* userdata) {

    if (!start || !goal)
        return FLT_MAX;

    float dx = (float)(start->x - goal->x);
    float dy = (float)(start->y - goal->y);
    return hypotf(dx, dy);
}

void move_to(const coord_t* c, void* userdata) {
    printf("move to (%d, %d) in finder.\n", c->x, c->y);
    // coord_destroy(c);
}

coord_list_t* get_changed_coord(void* userdata) {
    if (!userdata) {
        // g_warning("changed_coords: userdata is NULL");
        printf("changed_coord userdata(coord_t*) is NULL\n");
        return NULL;
    }

    coord_t* c = (coord_t*)userdata;

    coord_list_t* list = coord_list_create();
    coord_list_push_back(list, c);

    printf("changed_coord: changed coord is (%d, %d).\n",
        c->x, c->y);
    return list;
}

coord_list_t* get_changed_coords(void* userdata) {
    if (!userdata) {
        // g_warning("changed_coords: userdata is NULL");
        printf("changed_coords: userdata is NULL\n");
        return NULL;
    }

    coord_list_t* src = (coord_list_t*)userdata;
    coord_list_t* copy = coord_list_copy(src);

    printf("changed_coords: %d changed coords copied and returned.\n",
        coord_list_length(copy));
    return copy;
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

dstar_lite_t* dstar_lite_create(navgrid_t* navgrid) {
    if (!navgrid) return NULL;

    coord_t* c = coord_create();
    dstar_lite_t* dsl = dstar_lite_create_full(navgrid, c, c,
        dstar_lite_cost, dstar_lite_heuristic,        
        false);
    coord_destroy(c);

    return dsl;
}

dstar_lite_t* dstar_lite_create_full(navgrid_t* navgrid, 
    const coord_t* start,
    const coord_t* goal,
    cost_func cost_fn, heuristic_func heuristic_fn,
    bool debug_mode_enabled) {

    if (!navgrid) return NULL;

    dstar_lite_t* dsl = new dstar_lite_t();
    dsl->navgrid = navgrid;
    // printf("[dsl->navgrid assigned] %p\n", navgrid);

    dsl->start = *start;
    dsl->goal = *goal;
    
    dsl->km = 0.0f;
    dsl->max_range = 100;
    
    dsl->real_loop_max_retry = 3000;

    dsl->max_retry = 3000;
    
    dsl->reconstruct_max_retry = 300;

    dsl->cost_fn = cost_fn ? cost_fn : dstar_lite_cost;
    dsl->heuristic_fn = heuristic_fn ? heuristic_fn : dstar_lite_heuristic;

    dsl->debug_mode_enabled = debug_mode_enabled;

    dsl->g_table = coord_hash_create_full(
        (coord_hash_copy_func) float_copy,
        (coord_hash_destroy_func) float_destroy
    );

    dsl->rhs_table = coord_hash_create_full(
        (coord_hash_copy_func) float_copy,
        (coord_hash_destroy_func) float_destroy        
    );

    dsl->frontier = dstar_lite_pqueue_create();

    dsl->proto_route = route_create();
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

    dsl->interval_sec = 0.0f;

    return dsl;
}

void dstar_lite_destroy(dstar_lite_t* dsl) {
    if (!dsl) return;
    coord_hash_destroy(dsl->g_table);
    coord_hash_destroy(dsl->rhs_table);
    dstar_lite_pqueue_destroy(dsl->frontier);
    if (dsl->proto_route) route_destroy(dsl->proto_route);
    if (dsl->real_route) route_destroy(dsl->real_route);

    delete dsl;
}

dstar_lite_t* dstar_lite_copy(const dstar_lite_t* src) {
    if (!src) return NULL;

    dstar_lite_t* copy = new dstar_lite_t();
    
    copy->navgrid     = navgrid_copy(src->navgrid);
    copy->start = src->start;
    copy->goal  = src->goal;
    copy->km    = src->km;

    copy->g_table = coord_hash_copy(src->g_table);

    copy->rhs_table = coord_hash_copy(src->rhs_table);

    copy->frontier = dstar_lite_pqueue_copy(src->frontier);

    copy->cost_fn               = src->cost_fn;
    copy->cost_fn_userdata      = src->cost_fn_userdata;
    copy->heuristic_fn          = src->heuristic_fn;
    copy->heuristic_fn_userdata = src->heuristic_fn_userdata;
    copy->move_fn               = src->move_fn;
    copy->move_fn_userdata      = src->move_fn_userdata;
    copy->changed_coords_fn     = src->changed_coords_fn;
    copy->changed_coords_fn_userdata = src->changed_coords_fn_userdata;

    copy->proto_route = route_copy(src->proto_route);
    copy->real_route  = route_copy(src->real_route);

    copy->interval_sec           = src->interval_sec;
    copy->real_loop_max_retry     = src->real_loop_max_retry;
    copy->max_retry       = src->max_retry;
    copy->reconstruct_max_retry   = src->reconstruct_max_retry;
    copy->proto_compute_retry_count = src->proto_compute_retry_count;
    copy->real_compute_retry_count  = src->real_compute_retry_count;
    copy->real_loop_retry_count     = src->real_loop_retry_count;
    copy->reconstruct_retry_count   = src->reconstruct_retry_count;
    copy->force_quit             = src->force_quit;
    copy->max_range              = src->max_range;
    copy->debug_mode_enabled     = src->debug_mode_enabled;

    return copy;
}

int dstar_lite_fetch_start(const dstar_lite_t* dsl, coord_t* out) {
    if (!dsl) return -1;
    *out = dsl->start;
    return 0;
}

void dstar_lite_set_start(dstar_lite_t* dsl, const coord_t* c) {
    // dsl->start = c;
    coord_set(&dsl->start, c->x, c->y);
}

int dstar_lite_fetch_goal(const dstar_lite_t* dsl, coord_t* out) {
    if (!dsl) return -1;
    *out = dsl->goal;
    return 0;
}

void dstar_lite_set_goal(dstar_lite_t* dsl, const coord_t* c) {
    // dsl->goal = c;
    coord_set(&dsl->goal, c->x, c->y);
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
    if (dsl->frontier) dstar_lite_pqueue_destroy(dsl->frontier);
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

int dstar_lite_get_max_retry(const dstar_lite_t* dsl) {
    return dsl->max_retry;
}
void dstar_lite_set_max_retry(
    dstar_lite_t* dsl, int v) {
        
    dsl->max_retry = v;
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

bool dstar_lite_is_debug_mode_enabled(const dstar_lite_t* dsl) {
    return dsl->debug_mode_enabled;
}

void dstar_lite_enable_debug_mode(dstar_lite_t* dsl, bool enabled) {
    dsl->debug_mode_enabled = enabled;
}

const navgrid_t* dstar_lite_get_navgrid(const dstar_lite_t* dsl) {
    return dsl->navgrid;
}

void    dstar_lite_set_navgrid(dstar_lite_t* dsl, navgrid_t* navgrid) {
    dsl->navgrid = navgrid;
}

const route_t* dstar_lite_get_proto_route(const dstar_lite_t* dsl) {
    return dsl->proto_route;
}

const route_t* dstar_lite_get_real_route(const dstar_lite_t* dsl) {
    return dsl->real_route;
}

void dstar_lite_reset(dstar_lite_t* dsl) {
    coord_hash_destroy(dsl->g_table);
    coord_hash_destroy(dsl->rhs_table);

    dsl->g_table = coord_hash_create_full(
        (coord_hash_copy_func) float_copy,
        (coord_hash_destroy_func) float_destroy        
    );

    dsl->rhs_table = coord_hash_create_full(
        (coord_hash_copy_func) float_copy,
        (coord_hash_destroy_func) float_destroy        
    );

    if (dsl->proto_route) {
        route_destroy(dsl->proto_route);
        dsl->proto_route = route_create();
    }

    if (dsl->real_route) {
        route_destroy(dsl->real_route);
        dsl->real_route = NULL;
    }    
        
    dstar_lite_pqueue_destroy(dsl->frontier);
    dsl->frontier = dstar_lite_pqueue_create();

    dsl->proto_compute_retry_count = 0;
    dsl->real_compute_retry_count = 0;
    dsl->reconstruct_retry_count = 0;
    dsl->real_loop_retry_count = 0;    

    dstar_lite_init(dsl);
}

void dstar_lite_set_interval_sec(dstar_lite_t* dsl, float sec) {
    if (!dsl) return;
    dsl->interval_sec = sec;
}

float dstar_lite_get_interval_sec(const dstar_lite_t* dsl) {
    return dsl ? dsl->interval_sec : 0.0f;
}

dstar_lite_key_t* dstar_lite_calc_key(dstar_lite_t* dsl, const coord_t* s) {
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
    float h = dsl->heuristic_fn(&dsl->start, s, NULL);
    float k1 = k2 + h + dsl->km;

    dstar_lite_key_t* key = dstar_lite_key_create_full( k1, k2 );
    return key;
}

void dstar_lite_init(dstar_lite_t* dsl) {
    dsl->km = 0.0f;

    float* rhs_goal_ptr = new float();
    *rhs_goal_ptr = 0.0f;
    coord_hash_replace(dsl->rhs_table, &dsl->goal, rhs_goal_ptr);
    delete rhs_goal_ptr;
    
    // U.insert(goal, calc_key(goal))
    dstar_lite_key_t* calc_key_goal = dstar_lite_calc_key(dsl, &dsl->start);
    dstar_lite_pqueue_push(dsl->frontier, calc_key_goal, &dsl->goal);
    dstar_lite_key_destroy(calc_key_goal);
}

void dstar_lite_update_vertex(dstar_lite_t* dsl, const coord_t* u) {
    if (!dsl || !u) return;

    if (dsl->debug_mode_enabled){
        if (dsl->proto_route){
            route_add_visited(dsl->proto_route, u);
        }
    }

    float min_rhs = FLT_MAX;
    coord_list_t* successors_s = NULL;
    
    float* g_s_ptr = NULL;
    float g_s = FLT_MAX;

    float cost =  FLT_MAX;

    float* g_u_ptr = NULL;
    float g_u = FLT_MAX;

    float* rhs_u_ptr = NULL;
    float rhs_u = FLT_MAX;

    if (!coord_equal(u, &dsl->goal)) {
        successors_s = navgrid_copy_neighbors_all(dsl->navgrid, u->x, u->y);
        int len = coord_list_length(successors_s);
        for(int i=0; i<len; i++){
            const coord_t* s = coord_list_get(successors_s, i);

            g_s_ptr = (float*)coord_hash_get(dsl->g_table, s);
            if (g_s_ptr) {
                g_s = *g_s_ptr;
            } else {
                g_s = FLT_MAX;
            }

            cost = dsl->cost_fn(dsl->navgrid, u, s, NULL) + g_s;

            if (cost < min_rhs)
                min_rhs = cost;
        }
        coord_list_destroy(successors_s);

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
        dstar_lite_key_t* key = dstar_lite_calc_key(dsl, u);
        dstar_lite_pqueue_push(dsl->frontier, key, u);
        dstar_lite_key_destroy(key);
    }
}        

void dstar_lite_update_vertex_range(dstar_lite_t* dsl, 
    const coord_t* s, int max_range) {

    if (!dsl) return;

    if (max_range < 1) {
        dstar_lite_update_vertex(dsl, s);
        return;
    }    

    coord_list_t* neighbors = navgrid_copy_neighbors_all_range(dsl->navgrid, 
        s->x, s->y, max_range);

    int len = coord_list_length(neighbors);
    for(int i=0; i<len; i++) {
        const coord_t* c = coord_list_get(neighbors, i);
        dstar_lite_update_vertex(dsl, c);
    }

    coord_list_destroy(neighbors);
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

    int loop = 0;

    while (loop < dsl->max_retry) {
        ++loop;

        coord_t* u = dstar_lite_pqueue_pop(dsl->frontier);
        if (!u) break;

        float* g_u_ptr = (float*)coord_hash_get(dsl->g_table, u);
        float g_u = g_u_ptr ? *g_u_ptr : FLT_MAX;

        float* rhs_u_ptr = (float*)coord_hash_get(dsl->rhs_table, u);
        float rhs_u = rhs_u_ptr ? *rhs_u_ptr : FLT_MAX;

        dstar_lite_key_t* top_key = dstar_lite_calc_key(dsl, u);
        dstar_lite_key_t* start_key = dstar_lite_calc_key(dsl, &dsl->start);

        float* g_start_ptr = (float*)coord_hash_get(dsl->g_table, &dsl->start);
        float g_start = g_start_ptr ? *g_start_ptr : FLT_MAX;

        float* rhs_start_ptr = (float*)coord_hash_get(dsl->rhs_table, &dsl->start);
        float rhs_start = rhs_start_ptr ? *rhs_start_ptr : FLT_MAX;

        if (dstar_lite_key_compare(top_key, start_key) >= 0 &&
            float_equal(rhs_start, g_start)) {
            dstar_lite_key_destroy(top_key);
            dstar_lite_key_destroy(start_key);
            coord_destroy(u);
            break;
        }

        dstar_lite_key_destroy(start_key);

        if (g_u > rhs_u) {
            // Case 1: g(u) > rhs(u) -> g(u) <- rhs(u)
            float * rhs_u_ptr = new float{rhs_u};
            coord_hash_replace(dsl->g_table, u, rhs_u_ptr);
            delete rhs_u_ptr;
            
            coord_list_t* preds = navgrid_copy_neighbors_all(dsl->navgrid, u->x, u->y);
            for (int i = 0; i < coord_list_length(preds); ++i) {
                const coord_t* s = coord_list_get(preds, i);
                dstar_lite_update_vertex(dsl, s);
            }
            coord_list_destroy(preds);
        } else {
            // Case 2: g(u) <= rhs(u) -> g(u) <- INF, rhs(preds)
            float* inf_ptr = new float{FLT_MAX};
            coord_hash_replace(dsl->g_table, u, inf_ptr);
            delete inf_ptr;

            coord_list_t* preds = navgrid_copy_neighbors_all(dsl->navgrid, u->x, u->y);
            coord_list_push_back(preds, u); // preds U {u}

            for (int i = 0; i < coord_list_length(preds); ++i) {
                const coord_t* s = coord_list_get(preds, i);

                float* rhs_s_ptr = (float*)coord_hash_get(dsl->rhs_table, s);
                float rhs_s = rhs_s_ptr ? *rhs_s_ptr : FLT_MAX;

                float cost = dsl->cost_fn(dsl->navgrid, s, u, NULL);

                if (float_equal(rhs_s, cost + g_u)) {
                    if (!coord_equal(s, &dsl->goal)) {
                        float min_rhs = FLT_MAX;
                        coord_list_t* succs = navgrid_copy_neighbors_all(dsl->navgrid, s->x, s->y);
                        for (int j = 0; j < coord_list_length(succs); ++j) {
                            const coord_t* s2 = coord_list_get(succs, j);
                            float* g_s2_ptr = (float*)coord_hash_get(dsl->g_table, s2);
                            float g_s2 = g_s2_ptr ? *g_s2_ptr : FLT_MAX;
                            float c = dsl->cost_fn(dsl->navgrid, s, s2, NULL);
                            min_rhs = fminf(min_rhs, c + g_s2);
                        }
                        coord_list_destroy(succs);
                        float* min_rhs_ptr = new float{min_rhs};
                        coord_hash_replace(dsl->rhs_table, s, min_rhs_ptr);
                        delete min_rhs_ptr;
                    }
                    dstar_lite_update_vertex(dsl, s);
                }
            }

            coord_list_destroy(preds);
        }

        dstar_lite_key_destroy(top_key);
        coord_destroy(u);
    }

    if (dsl->proto_route == NULL)
        dsl->proto_compute_retry_count = loop;
    else
        dsl->real_compute_retry_count = loop;
}

bool dstar_lite_reconstruct_route(dstar_lite_t* dsl) {
    if (!dsl) return false;

    // route_t* p = route_create();
    route_t* p = dsl->proto_route;
    route_add_coord(p, &dsl->start);    

    float* g_start_ptr = (float*) coord_hash_get(dsl->g_table, &dsl->start);
    if (!g_start_ptr || float_equal(*g_start_ptr, FLT_MAX)) {
        if (dsl->debug_mode_enabled) {
            // p->visited_count = coord_hash_copy(dsl->update_count_table);
            p->total_retry_count = dstar_lite_proto_compute_retry_count(dsl);
        }
        return false;
    }

    coord_t current = dsl->start;
    // coord_t* current = start;

    int loop = 0;
    while (!coord_equal(&current, &dsl->goal) && 
        (loop < dsl->reconstruct_max_retry)) {

        loop++;

        coord_list_t* neighbors = navgrid_copy_neighbors_all(
            dsl->navgrid, current.x, current.y);

        int len = coord_list_length(neighbors);

        float min_cost = FLT_MAX;
        coord_t next = dsl->start;

        for (int i=0; i<len; i++){
            coord_t s = *coord_list_get(neighbors, i);

            float cost_current_s = dsl->cost_fn(dsl->navgrid, &current, &s, NULL);
            float* g_s_ptr = (float*) coord_hash_get(dsl->g_table, &s);
            float g_s = (g_s_ptr) ? *g_s_ptr : FLT_MAX;

            float total_cost = cost_current_s + g_s;
            if (total_cost < min_cost) {
                min_cost = total_cost;
                
                next = s;
                // next = s;
            }
        }
        coord_list_destroy(neighbors);

        if (coord_equal(&next, &dsl->start)) {
            // coord_destroy(current);
            // route_destroy(p);
            route_set_success(p, false);
            if (dsl->debug_mode_enabled){
                // p->visited_count = coord_hash_copy(dsl->update_count_table);
    p->total_retry_count = dstar_lite_proto_compute_retry_count(dsl);                
            }
            return false;
        }

        float* g_next_ptr = (float*) coord_hash_get(dsl->g_table, &next);
        if (!g_next_ptr || float_equal(*g_next_ptr, FLT_MAX)) {
            // coord_destroy(current);
            // coord_destroy(next);
            // route_destroy(p);
            route_set_success(p, false);
            if (dsl->debug_mode_enabled){
                // p->visited_count = coord_hash_copy(dsl->update_count_table);  
p->total_retry_count = dstar_lite_proto_compute_retry_count(dsl);                
            }          
            return false;
        }

        route_add_coord(p, &next);
        // coord_destroy(current);
        current = next;
    }

    dsl->reconstruct_retry_count = loop;

    // coord_destroy(current);
    route_set_success(p, true);    
    if (dsl->debug_mode_enabled){
        // p->visited_count = coord_hash_copy(dsl->update_count_table);    
p->total_retry_count = dstar_lite_proto_compute_retry_count(dsl);        
    }
    return true;
}

route_t* dstar_lite_find(dstar_lite_t* dsl) {
    if (!dsl) return NULL;

    dstar_lite_reset(dsl);

    dstar_lite_compute_shortest_route(dsl);
    bool result = dstar_lite_reconstruct_route(dsl);
    return route_copy(dsl->proto_route);
}

void dstar_lite_find_full(dstar_lite_t* dsl) {
    if (!dsl) return;
    dstar_lite_find_proto(dsl);
    dstar_lite_find_loop(dsl);
}

bool dstar_lite_find_proto(dstar_lite_t* dsl) {
    if (!dsl) return false;

    dstar_lite_reset(dsl);

    dstar_lite_compute_shortest_route(dsl);
    return dstar_lite_reconstruct_route(dsl);
}

void dstar_lite_find_loop(dstar_lite_t* dsl) {
    if (!dsl) return;

    coord_t s_last = dsl->start;
    coord_t current = dsl->start;
    coord_t next = {0};

    dsl->real_route = route_create();
    route_add_coord(dsl->real_route, &current);

    for (int loop = 0; loop < dsl->real_loop_max_retry && !dsl->force_quit; ++loop) {
        if (coord_equal(&current, &dsl->goal)) {
            route_set_success(dsl->real_route, true);
            dsl->real_loop_retry_count = loop;
            return;
        }

        float* rhs_ptr = (float*)coord_hash_get(dsl->rhs_table, &current);
        float rhs_val = rhs_ptr ? *rhs_ptr : FLT_MAX;
        if (float_equal(rhs_val, FLT_MAX)) {
            route_set_success(dsl->real_route, false);
            dsl->real_loop_retry_count = loop;
            return;
        }

        bool found = dstar_lite_fetch_next(dsl, &current, &next);

        if (!found || coord_equal(&next, &current)) {
            route_set_success(dsl->real_route, false);
            dsl->real_loop_retry_count = loop;
            return;
        }

        route_add_coord(dsl->real_route, &next);

        if (dsl->move_fn)
            dsl->move_fn(&next, dsl->move_fn_userdata);

        if (dsl->interval_sec <= 0.0f)
            std::this_thread::yield();
        else
            std::this_thread::sleep_for(std::chrono::duration<float>(dsl->interval_sec));

        if (dsl->changed_coords_fn) {
            coord_list_t* changed = dsl->changed_coords_fn(dsl->changed_coords_fn_userdata);
            if (changed && coord_list_length(changed) > 0) {
                dsl->km += dsl->heuristic_fn(&s_last, &next, NULL);
                s_last = next;

                for (int i = 0; i < coord_list_length(changed); ++i) {
                    const coord_t* c = coord_list_get(changed, i);
                    dstar_lite_update_vertex(dsl, c);
                }

                coord_list_destroy(changed);
                dstar_lite_compute_shortest_route(dsl);
            }
        }

        current = next;
    }

    route_set_success(dsl->real_route, false);
    dsl->real_loop_retry_count = dsl->real_loop_max_retry;
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
