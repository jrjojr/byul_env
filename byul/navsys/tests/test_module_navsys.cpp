#include "doctest.h"
#include "navsys.h"
#include "console.h"

#include <iostream>

TEST_CASE("navsys: find astar") {
    navgrid_t* navgrid = navgrid_create();

    coord_t start = {0, 0};
    coord_t goal = {9, 9};

    // obstacle input  (vertical blocking)
    for (int y = 1; y < 10; ++y)
        navgrid_block_coord(navgrid, 5, y);

    route_t* p = navsys_find_astar(navgrid, &start, &goal);

    CHECK(route_get_success(p) == true);

    route_print(p);
    navgrid_print_ascii_with_route(navgrid, p, 2);

    route_destroy(p);
    navgrid_destroy(navgrid);
}

TEST_CASE("navsys: find astar generic") {
    navgrid_t* navgrid = navgrid_create_full(
        10, 10, NAVGRID_DIR_8, is_coord_blocked_navgrid);

    coord_t start;
    coord_init_full(&start, 0, 0);

    coord_t goal; 
    coord_init_full(&goal, 9, 9);

    route_finder_t* rf = route_finder_create(navgrid);

    // obstacle input  (vertical blocking)
    for (int y = 1; y < 10; ++y)
        navgrid_block_coord(navgrid, 5, y);

    route_t* p = navsys_find_astar(navgrid, &start, &goal);    

    CHECK(route_get_success(p) == true);

    route_print(p);
    navgrid_print_ascii_with_route(navgrid, p, 2);

    route_destroy(p);
    navgrid_destroy(navgrid);
    route_finder_destroy(rf);
}

TEST_CASE("navsys: find bfs") {
    navgrid_t* navgrid = navgrid_create_full(
        10, 10, NAVGRID_DIR_8, is_coord_blocked_navgrid);

    coord_t start;
    coord_init_full(&start, 0, 0);

    coord_t goal; 
    coord_init_full(&goal, 9, 9);

    route_finder_t* rf = route_finder_create(navgrid);

    // obstacle input  (vertical blocking)
    for (int y = 1; y < 10; ++y)
        navgrid_block_coord(navgrid, 5, y);

    route_t* p = navsys_find_bfs(navgrid, &start, &goal);    

    CHECK(route_get_success(p) == true);

    route_print(p);
    navgrid_print_ascii_with_route(navgrid, p, 2);

    route_destroy(p);
    navgrid_destroy(navgrid);
    route_finder_destroy(rf);
}

TEST_CASE("navsys: find dfs") {
    navgrid_t* navgrid = navgrid_create_full(
        10, 10, NAVGRID_DIR_8, is_coord_blocked_navgrid);

    coord_t start;
    coord_init_full(&start, 0, 0);

    coord_t goal; 
    coord_init_full(&goal, 9, 9);

    route_finder_t* rf = route_finder_create(navgrid);

    // obstacle input  (vertical blocking)
    for (int y = 1; y < 10; ++y)
        navgrid_block_coord(navgrid, 5, y);

    route_t* p = navsys_find_dfs(navgrid, &start, &goal);    

    CHECK(route_get_success(p) == true);

    route_print(p);
    // navgrid_print_ascii_with_route(navgrid, p, 2);

    route_destroy(p);
    navgrid_destroy(navgrid);
    route_finder_destroy(rf);
}

TEST_CASE("navsys: find dijkstra") {
    navgrid_t* navgrid = navgrid_create_full(
        10, 10, NAVGRID_DIR_8, is_coord_blocked_navgrid);

    coord_t start;
    coord_init_full(&start, 0, 0);

    coord_t goal; 
    coord_init_full(&goal, 9, 9);

    route_finder_t* rf = route_finder_create(navgrid);

    // obstacle input  (vertical blocking)
    for (int y = 1; y < 10; ++y)
        navgrid_block_coord(navgrid, 5, y);

    route_t* p = navsys_find_dijkstra(navgrid, &start, &goal);    

    CHECK(route_get_success(p) == true);

    route_print(p);
    navgrid_print_ascii_with_route(navgrid, p, 2);

    route_destroy(p);
    navgrid_destroy(navgrid);
    route_finder_destroy(rf);
}

TEST_CASE("navsys: find greedy_best_first") {
    navgrid_t* navgrid = navgrid_create_full(
        10, 10, NAVGRID_DIR_8, is_coord_blocked_navgrid);

    coord_t start;
    coord_init_full(&start, 0, 0);

    coord_t goal; 
    coord_init_full(&goal, 9, 9);

    route_finder_t* rf = route_finder_create(navgrid);

    // obstacle input  (vertical blocking)
    for (int y = 1; y < 10; ++y)
        navgrid_block_coord(navgrid, 5, y);

    route_t* p = navsys_find_greedy_best_first(navgrid, &start, &goal);    

    CHECK(route_get_success(p) == true);

    route_print(p);
    navgrid_print_ascii_with_route(navgrid, p, 2);

    route_destroy(p);
    navgrid_destroy(navgrid);
    route_finder_destroy(rf);
}

TEST_CASE("navsys: find fast_marching") {
    navgrid_t* navgrid = navgrid_create_full(
        10, 10, NAVGRID_DIR_8, is_coord_blocked_navgrid);

    coord_t start;
    coord_init_full(&start, 0, 0);

    coord_t goal; 
    coord_init_full(&goal, 9, 9);

    route_finder_t* rf = route_finder_create(navgrid);

    // obstacle input  (vertical blocking)
    for (int y = 1; y < 10; ++y)
        navgrid_block_coord(navgrid, 5, y);

    route_t* p = navsys_find_fast_marching(navgrid, &start, &goal);    

    CHECK(route_get_success(p) == true);

    route_print(p);
    navgrid_print_ascii_with_route(navgrid, p, 2);

    route_destroy(p);
    navgrid_destroy(navgrid);
    route_finder_destroy(rf);
}

TEST_CASE("navsys: find ida_star") {
    navgrid_t* navgrid = navgrid_create_full(
        10, 10, NAVGRID_DIR_8, is_coord_blocked_navgrid);

    coord_t start;
    coord_init_full(&start, 0, 0);

    coord_t goal; 
    coord_init_full(&goal, 9, 9);

    route_finder_t* rf = route_finder_create(navgrid);

    // obstacle input  (vertical blocking)
    for (int y = 1; y < 10; ++y)
        navgrid_block_coord(navgrid, 5, y);

    route_t* p = navsys_find_ida_star(navgrid, &start, &goal);    

    CHECK(route_get_success(p) == true);

    route_print(p);
    navgrid_print_ascii_with_route(navgrid, p, 2);

    route_destroy(p);
    navgrid_destroy(navgrid);
    route_finder_destroy(rf);
}

TEST_CASE("navsys: find fringe_search") {
    navgrid_t* navgrid = navgrid_create_full(
        10, 10, NAVGRID_DIR_8, is_coord_blocked_navgrid);

    coord_t start;
    coord_init_full(&start, 0, 0);

    coord_t goal; 
    coord_init_full(&goal, 9, 9);

    route_finder_t* rf = route_finder_create(navgrid);

    // obstacle input  (vertical blocking)
    for (int y = 1; y < 10; ++y)
        navgrid_block_coord(navgrid, 5, y);

    route_t* p = navsys_find_fringe_search(navgrid, &start, &goal, 0.3f);    

    CHECK(route_get_success(p) == true);

    route_print(p);
    navgrid_print_ascii_with_route(navgrid, p, 2);

    route_destroy(p);
    navgrid_destroy(navgrid);
    route_finder_destroy(rf);
}

TEST_CASE("navsys: find rta_star") {
    navgrid_t* navgrid = navgrid_create_full(
        10, 10, NAVGRID_DIR_8, is_coord_blocked_navgrid);

    coord_t start;
    coord_init_full(&start, 0, 0);

    coord_t goal; 
    coord_init_full(&goal, 9, 9);

    route_finder_t* rf = route_finder_create(navgrid);

    // obstacle input  (vertical blocking)
    for (int y = 1; y < 10; ++y)
        navgrid_block_coord(navgrid, 5, y);

    route_t* p = navsys_find_rta_star(navgrid, &start, &goal, 7);    

    CHECK(route_get_success(p) == true);

    route_print(p);
    navgrid_print_ascii_with_route(navgrid, p, 2);

    route_destroy(p);
    navgrid_destroy(navgrid);
    route_finder_destroy(rf);
}

TEST_CASE("navsys: find sma_star") {
    navgrid_t* navgrid = navgrid_create_full(
        10, 10, NAVGRID_DIR_8, is_coord_blocked_navgrid);

    coord_t start;
    coord_init_full(&start, 0, 0);

    coord_t goal; 
    coord_init_full(&goal, 9, 9);

    route_finder_t* rf = route_finder_create(navgrid);

    // obstacle input  (vertical blocking)
    for (int y = 1; y < 10; ++y)
        navgrid_block_coord(navgrid, 5, y);

    route_t* p = navsys_find_sma_star(navgrid, &start, &goal, 30);    

    CHECK(route_get_success(p) == true);

    route_print(p);
    navgrid_print_ascii_with_route(navgrid, p, 2);

    route_destroy(p);
    navgrid_destroy(navgrid);
    route_finder_destroy(rf);
}

TEST_CASE("navsys: find weighted_astar") {
    navgrid_t* navgrid = navgrid_create_full(
        10, 10, NAVGRID_DIR_8, is_coord_blocked_navgrid);

    coord_t start;
    coord_init_full(&start, 0, 0);

    coord_t goal; 
    coord_init_full(&goal, 9, 9);

    route_finder_t* rf = route_finder_create(navgrid);

    // obstacle input  (vertical blocking)
    for (int y = 1; y < 10; ++y)
        navgrid_block_coord(navgrid, 5, y);

    route_t* p = navsys_find_weighted_astar(navgrid, &start, &goal, 1.0f);    

    CHECK(route_get_success(p) == true);

    route_print(p);
    navgrid_print_ascii_with_route(navgrid, p, 2);

    route_destroy(p);
    navgrid_destroy(navgrid);
    route_finder_destroy(rf);
}

TEST_CASE("navsys: find dstar_lite") {
    navgrid_t* navgrid = navgrid_create_full(
        10, 10, NAVGRID_DIR_8, is_coord_blocked_navgrid);

    coord_t start;
    coord_init_full(&start, 0, 0);

    coord_t goal; 
    coord_init_full(&goal, 9, 9);

    // obstacle input  (vertical blocking)
    for (int y = 1; y < 10; ++y)
        navgrid_block_coord(navgrid, 5, y);

    route_t* p = navsys_find_dstar_lite(navgrid, &start, &goal);    

    CHECK(route_get_success(p) == true);

    std::cout << "dstar lite find.\n";
    route_print(p);
    navgrid_print_ascii_with_route(navgrid, p, 2);

    route_destroy(p);
    navgrid_destroy(navgrid);
}

