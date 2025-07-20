#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <unordered_set>
#include "internal/common.h"
#include "internal/dstar_lite_utils.h"
#include "internal/coord.h"
#include "internal/dstar_lite_key.h"
#include "internal/route.h"
#include "internal/navgrid.h"
#include "internal/coord_hash.h"
#include "internal/dstar_lite_pqueue.h"

#include "internal/console.h"

void dsl_debug_print_g_table(const navgrid_t* m, coord_hash_t* g_table) {
    if (!g_table) return;
    printf("\n📊 g_table (g-values):\n");

    coord_hash_iter_t* it = coord_hash_iter_create(g_table);
    coord_t* c;
    float* val;
    while (coord_hash_iter_next(it, &c, (void**)&val)) {
        printf("  (%d, %d) → g = %.3f\n", c->x, c->y, *val);
    }
}

void dsl_debug_print_rhs_table(const navgrid_t* m, coord_hash_t* rhs_table) {
    if (!rhs_table) return;
    printf("\n📊 rhs_table:\n");

    coord_hash_iter_t* it = coord_hash_iter_create(rhs_table);
    coord_t* c;
    float* val;
    while (coord_hash_iter_next(it, &c, (void**)&val)) {
        printf("  (%d, %d) → rhs = %.3f\n", c->x, c->y, *val);
    }
}

void dsl_print_info(const dstar_lite_t* dsl){
    printf("print dsl info\n");
    printf("dsl->max_range : %d\n", dsl->max_range);
    printf("dsl->compute_max_retry : %d\n", dsl->compute_max_retry);
    printf("dsl->real_loop_max_retry : %d\n", dsl->real_loop_max_retry);
    printf("dsl->reconstruct_max_retry : %d\n", dsl->reconstruct_max_retry);

    printf("dsl->proto_compute_retry_count : %d\n", dsl->proto_compute_retry_count);
    printf("dsl->real_compute_retry_count : %d\n", dsl->real_compute_retry_count);
    printf("dsl->reconstruct_retry_count : %d\n", dsl->reconstruct_retry_count);
    printf("dsl->real_loop_retry_count : %d\n", dsl->real_loop_retry_count);
}

void dsl_print_ascii_only_navgrid(const dstar_lite_t* dsl){
    if(!dsl) return;
    navgrid_print_ascii(dsl->m);
}

void dsl_print_ascii_route(
    const dstar_lite_t* dsl, const route_t* p, int margin) {

    if (!dsl || !dsl->m) return;

    navgrid_print_ascii_with_route(dsl->m, p, margin);
}

void dsl_print_ascii_update_count(
    const dstar_lite_t* dsl, route_t* p, int margin) {

    if (!dsl || !p) return;
    
    route_set_total_retry_count(p, dstar_lite_proto_compute_retry_count(dsl));

    navgrid_print_ascii_with_visited_count(dsl->m, p, margin);
}
