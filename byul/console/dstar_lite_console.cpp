#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <unordered_set>
#include "dstar_lite_console.h"
#include "coord.h"
#include "dstar_lite_key.h"
#include "route.h"
#include "navgrid.h"
#include "coord_hash.h"
#include "dstar_lite_pqueue.h"

#include "console.h"

void dsl_debug_print_g_table(const navgrid_t* m, coord_hash_t* g_table) {
    if (!g_table) return;
    printf("\ng_table (g-values):\n");

    coord_hash_iter_t* it = coord_hash_iter_create(g_table);
    coord_t c;
    float* val;
    while (coord_hash_iter_next(it, &c, (void**)&val)) {
        printf("  (%d, %d) -> g = %.3f\n", c.x, c.y, *val);
    }
}

void dsl_debug_print_rhs_table(const navgrid_t* m, coord_hash_t* rhs_table) {
    if (!rhs_table) return;
    printf("\nrhs_table:\n");

    coord_hash_iter_t* it = coord_hash_iter_create(rhs_table);
    coord_t c;
    float* val;
    while (coord_hash_iter_next(it, &c, (void**)&val)) {
        printf("  (%d, %d) -> rhs = %.3f\n", c.x, c.y, *val);
    }
}

void dsl_debug_print_state(
    const dstar_lite_t* dsl,
    const coord_t* goal,
    float km,
    coord_hash_t* g_table,
    coord_hash_t* rhs_table,
    dstar_lite_pqueue_t* frontier) {
    if (!dsl) return;
    printf("\nD* Lite state:\n");
    if (goal) printf("  goal = (%d, %d)\n", goal->x, goal->y);
    printf("  km = %.3f\n", km);
    printf("  frontier size = %zu\n",
        frontier ? dstar_lite_pqueue_size(frontier) : 0u);
    dsl_debug_print_g_table(dsl->navgrid, g_table);
    dsl_debug_print_rhs_table(dsl->navgrid, rhs_table);
}

void dsl_debug_print_full_state(
    const dstar_lite_t* dsl,
    const coord_t* goal,
    float km,
    coord_hash_t* g_table,
    coord_hash_t* rhs_table,
    dstar_lite_pqueue_t* frontier,
    int max_range,
    int retry_limit,
    bool debug_mode,
    coord_hash_t* update_counter) {
    if (!dsl) return;
    dsl_debug_print_state(
        dsl, goal, km, g_table, rhs_table, frontier);
    printf("  max range = %d\n", max_range);
    printf("  retry limit = %d\n", retry_limit);
    printf("  debug mode = %s\n", debug_mode ? "true" : "false");
    if (update_counter) {
        printf("\nupdate counts:\n");
        coord_hash_iter_t* it = coord_hash_iter_create(update_counter);
        if (!it) return;
        coord_t coord;
        int* count = nullptr;
        while (coord_hash_iter_next(it, &coord, (void**)&count)) {
            printf("  (%d, %d) -> %d\n",
                coord.x, coord.y, count ? *count : 0);
        }
        coord_hash_iter_destroy(it);
    }
}

void dsl_print_info(const dstar_lite_t* dsl){
    printf("print dsl info\n");
    printf("dsl->max_range : %d\n", dsl->max_range);
    printf("dsl->max_retry : %d\n", dsl->max_retry);
    printf("dsl->real_loop_max_retry : %d\n", dsl->real_loop_max_retry);
    printf("dsl->reconstruct_max_retry : %d\n", dsl->reconstruct_max_retry);

    printf("dsl->proto_compute_retry_count : %d\n", dsl->proto_compute_retry_count);
    printf("dsl->real_compute_retry_count : %d\n", dsl->real_compute_retry_count);
    printf("dsl->reconstruct_retry_count : %d\n", dsl->reconstruct_retry_count);
    printf("dsl->real_loop_retry_count : %d\n", dsl->real_loop_retry_count);
}

void dsl_print_ascii_only_navgrid(const dstar_lite_t* dsl){
    if(!dsl) return;
    navgrid_print_ascii(dsl->navgrid);
}

void dsl_print_ascii_route(
    const dstar_lite_t* dsl, const route_t* p, int margin) {

    if (!dsl || !dsl->navgrid) return;

    navgrid_print_ascii_with_route(dsl->navgrid, p, margin);
}

void dsl_print_ascii_update_count(
    const dstar_lite_t* dsl, route_t* p, int margin) {

    if (!dsl || !p) return;
    
    // route_set_total_retry_count(p, dstar_lite_proto_compute_retry_count(dsl));

    navgrid_print_ascii_with_visited_count(dsl->navgrid, p, margin);
}
