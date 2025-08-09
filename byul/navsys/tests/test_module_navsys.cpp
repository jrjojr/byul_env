#include "doctest.h"
#include "navsys.h"
#include "console.h"

TEST_CASE("navsys: find astar simple") {
    navgrid_t* m = navgrid_create();

    coord_t start = {0, 0};
    coord_t goal = {9, 9};

    route_finder_t* rf = route_finder_create(m);

    // obstacle input  (vertical blocking)
    for (int y = 1; y < 10; ++y)
        navgrid_block_coord(m, 5, y);

    route_t* p = navsys_find_astar(m, &start, &goal);

    CHECK(route_get_success(p) == true);

    route_print(p);
    navgrid_print_ascii_with_route(m, p, 2);

    route_destroy(p);
    navgrid_destroy(m);
    route_finder_destroy(rf);
}

TEST_CASE("navsys: find astar") {
    navgrid_t* m = navgrid_create_full(
        10, 10, NAVGRID_DIR_8, is_coord_blocked_navgrid);

    coord_t start;
    coord_init_full(&start, 0, 0);

    coord_t goal; 
    coord_init_full(&goal, 9, 9);

    route_finder_t* rf = route_finder_create(m);

    // obstacle input  (vertical blocking)
    for (int y = 1; y < 10; ++y)
        navgrid_block_coord(m, 5, y);

    route_t* p = navsys_find_astar(m, &start, &goal);    

    CHECK(route_get_success(p) == true);

    route_print(p);
    navgrid_print_ascii_with_route(m, p, 2);

    route_destroy(p);
    navgrid_destroy(m);
    route_finder_destroy(rf);
}

TEST_CASE("navsys: find bfs") {
    navgrid_t* m = navgrid_create_full(
        10, 10, NAVGRID_DIR_8, is_coord_blocked_navgrid);

    coord_t start;
    coord_init_full(&start, 0, 0);

    coord_t goal; 
    coord_init_full(&goal, 9, 9);

    route_finder_t* rf = route_finder_create(m);

    // obstacle input  (vertical blocking)
    for (int y = 1; y < 10; ++y)
        navgrid_block_coord(m, 5, y);

    route_t* p = navsys_find_bfs(m, &start, &goal);    

    CHECK(route_get_success(p) == true);

    route_print(p);
    navgrid_print_ascii_with_route(m, p, 2);

    route_destroy(p);
    navgrid_destroy(m);
    route_finder_destroy(rf);
}

TEST_CASE("navsys: find dfs") {
    navgrid_t* m = navgrid_create_full(
        10, 10, NAVGRID_DIR_8, is_coord_blocked_navgrid);

    coord_t start;
    coord_init_full(&start, 0, 0);

    coord_t goal; 
    coord_init_full(&goal, 9, 9);

    route_finder_t* rf = route_finder_create(m);

    // obstacle input  (vertical blocking)
    for (int y = 1; y < 10; ++y)
        navgrid_block_coord(m, 5, y);

    route_t* p = navsys_find_dfs(m, &start, &goal);    

    CHECK(route_get_success(p) == true);

    route_print(p);
    // navgrid_print_ascii_with_route(m, p, 2);

    route_destroy(p);
    navgrid_destroy(m);
    route_finder_destroy(rf);
}

TEST_CASE("navsys: find dijkstra") {
    navgrid_t* m = navgrid_create_full(
        10, 10, NAVGRID_DIR_8, is_coord_blocked_navgrid);

    coord_t start;
    coord_init_full(&start, 0, 0);

    coord_t goal; 
    coord_init_full(&goal, 9, 9);

    route_finder_t* rf = route_finder_create(m);

    // obstacle input  (vertical blocking)
    for (int y = 1; y < 10; ++y)
        navgrid_block_coord(m, 5, y);

    route_t* p = navsys_find_dijkstra(m, &start, &goal);    

    CHECK(route_get_success(p) == true);

    route_print(p);
    navgrid_print_ascii_with_route(m, p, 2);

    route_destroy(p);
    navgrid_destroy(m);
    route_finder_destroy(rf);
}

TEST_CASE("navsys: find greedy_best_first") {
    navgrid_t* m = navgrid_create_full(
        10, 10, NAVGRID_DIR_8, is_coord_blocked_navgrid);

    coord_t start;
    coord_init_full(&start, 0, 0);

    coord_t goal; 
    coord_init_full(&goal, 9, 9);

    route_finder_t* rf = route_finder_create(m);

    // obstacle input  (vertical blocking)
    for (int y = 1; y < 10; ++y)
        navgrid_block_coord(m, 5, y);

    route_t* p = navsys_find_greedy_best_first(m, &start, &goal);    

    CHECK(route_get_success(p) == true);

    route_print(p);
    navgrid_print_ascii_with_route(m, p, 2);

    route_destroy(p);
    navgrid_destroy(m);
    route_finder_destroy(rf);
}

TEST_CASE("navsys: find fast_marching") {
    navgrid_t* m = navgrid_create_full(
        10, 10, NAVGRID_DIR_8, is_coord_blocked_navgrid);

    coord_t start;
    coord_init_full(&start, 0, 0);

    coord_t goal; 
    coord_init_full(&goal, 9, 9);

    route_finder_t* rf = route_finder_create(m);

    // obstacle input  (vertical blocking)
    for (int y = 1; y < 10; ++y)
        navgrid_block_coord(m, 5, y);

    route_t* p = navsys_find_fast_marching(m, &start, &goal);    

    CHECK(route_get_success(p) == true);

    route_print(p);
    navgrid_print_ascii_with_route(m, p, 2);

    route_destroy(p);
    navgrid_destroy(m);
    route_finder_destroy(rf);
}

TEST_CASE("navsys: find ida_star") {
    navgrid_t* m = navgrid_create_full(
        10, 10, NAVGRID_DIR_8, is_coord_blocked_navgrid);

    coord_t start;
    coord_init_full(&start, 0, 0);

    coord_t goal; 
    coord_init_full(&goal, 9, 9);

    route_finder_t* rf = route_finder_create(m);

    // obstacle input  (vertical blocking)
    for (int y = 1; y < 10; ++y)
        navgrid_block_coord(m, 5, y);

    route_t* p = navsys_find_ida_star(m, &start, &goal);    

    CHECK(route_get_success(p) == true);

    route_print(p);
    navgrid_print_ascii_with_route(m, p, 2);

    route_destroy(p);
    navgrid_destroy(m);
    route_finder_destroy(rf);
}

TEST_CASE("navsys: find fringe_search") {
    navgrid_t* m = navgrid_create_full(
        10, 10, NAVGRID_DIR_8, is_coord_blocked_navgrid);

    coord_t start;
    coord_init_full(&start, 0, 0);

    coord_t goal; 
    coord_init_full(&goal, 9, 9);

    route_finder_t* rf = route_finder_create(m);

    // obstacle input  (vertical blocking)
    for (int y = 1; y < 10; ++y)
        navgrid_block_coord(m, 5, y);

    route_t* p = navsys_find_fringe_search(m, &start, &goal, 0.3f);    

    CHECK(route_get_success(p) == true);

    route_print(p);
    navgrid_print_ascii_with_route(m, p, 2);

    route_destroy(p);
    navgrid_destroy(m);
    route_finder_destroy(rf);
}

TEST_CASE("navsys: find rta_star") {
    navgrid_t* m = navgrid_create_full(
        10, 10, NAVGRID_DIR_8, is_coord_blocked_navgrid);

    coord_t start;
    coord_init_full(&start, 0, 0);

    coord_t goal; 
    coord_init_full(&goal, 9, 9);

    route_finder_t* rf = route_finder_create(m);

    // obstacle input  (vertical blocking)
    for (int y = 1; y < 10; ++y)
        navgrid_block_coord(m, 5, y);

    route_t* p = navsys_find_rta_star(m, &start, &goal, 7);    

    CHECK(route_get_success(p) == true);

    route_print(p);
    navgrid_print_ascii_with_route(m, p, 2);

    route_destroy(p);
    navgrid_destroy(m);
    route_finder_destroy(rf);
}

TEST_CASE("navsys: find sma_star") {
    navgrid_t* m = navgrid_create_full(
        10, 10, NAVGRID_DIR_8, is_coord_blocked_navgrid);

    coord_t start;
    coord_init_full(&start, 0, 0);

    coord_t goal; 
    coord_init_full(&goal, 9, 9);

    route_finder_t* rf = route_finder_create(m);

    // obstacle input  (vertical blocking)
    for (int y = 1; y < 10; ++y)
        navgrid_block_coord(m, 5, y);

    route_t* p = navsys_find_sma_star(m, &start, &goal, 30);    

    CHECK(route_get_success(p) == true);

    route_print(p);
    navgrid_print_ascii_with_route(m, p, 2);

    route_destroy(p);
    navgrid_destroy(m);
    route_finder_destroy(rf);
}

TEST_CASE("navsys: find weighted_astar") {
    navgrid_t* m = navgrid_create_full(
        10, 10, NAVGRID_DIR_8, is_coord_blocked_navgrid);

    coord_t start;
    coord_init_full(&start, 0, 0);

    coord_t goal; 
    coord_init_full(&goal, 9, 9);

    route_finder_t* rf = route_finder_create(m);

    // obstacle input  (vertical blocking)
    for (int y = 1; y < 10; ++y)
        navgrid_block_coord(m, 5, y);

    route_t* p = navsys_find_weighted_astar(m, &start, &goal, 1.0f);    

    CHECK(route_get_success(p) == true);

    route_print(p);
    navgrid_print_ascii_with_route(m, p, 2);

    route_destroy(p);
    navgrid_destroy(m);
    route_finder_destroy(rf);
}
