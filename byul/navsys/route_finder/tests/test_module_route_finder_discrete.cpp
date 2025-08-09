#include "doctest.h"

// extern "C" {
    #include "navgrid.h"
    #include "coord.h"
    #include "route.h"

    #include "console.h"
// }

#include <iostream>
#include <bfs.h>
#include <dfs.h>
#include <dijkstra.h>
#include <astar.h>
#include <fast_marching.h>
#include <greedy_best_first.h>
#include <ida_star.h>
#include <fringe_search.h>
#include <weighted_astar.h>
#include <rta_star.h>
#include <sma_star.h>

TEST_CASE("BFS: simple route") {
    // navgrid_t* m = navgrid_create();
    std::cout << "BFS: simple route\n";
    navgrid_t* m = navgrid_create_full(
        10, 10, NAVGRID_DIR_8, is_coord_blocked_navgrid);
    coord_t* start = coord_create_full(0, 0);
    coord_t* goal = coord_create_full(9, 9);

    route_t* p = find_bfs(m, start, goal, 100, true);

    REQUIRE(p != nullptr);
    CHECK(route_get_success(p) == true);

    route_print(p);
    navgrid_print_ascii_with_visited_count(m, p, 5);

    route_destroy(p);
    coord_destroy(start);
    coord_destroy(goal);
    navgrid_destroy(m);
}

TEST_CASE("BFS: blocked route") {
    std::cout << "BFS: blocked route\n";
    // navgrid_t* m = navgrid_create();
    navgrid_t* m = navgrid_create_full(
        10, 10, NAVGRID_DIR_8, is_coord_blocked_navgrid);
    coord_t* start = coord_create_full(0, 0);
    coord_t* goal = coord_create_full(9, 9);

    REQUIRE_FALSE(coord_equal(start, goal));

    // obstacle input  (vertical blocking)
    for (int y = 1; y < 10; ++y)
        navgrid_block_coord(m, 5, y);

    route_t* p = find_bfs(m, start, goal, 100, true);

    REQUIRE(p != nullptr);
    CHECK(route_get_success(p) == true);

    route_print(p);
    navgrid_print_ascii_with_route(m, p, 5);
    navgrid_print_ascii_with_visited_count(m, p, 5);

    route_destroy(p);
    coord_destroy(start);
    coord_destroy(goal);
    navgrid_destroy(m);
}

TEST_CASE("BFS: blocked route force failed") {
    std::cout << "BFS: blocked route force failed\n";
    // navgrid_t* m = navgrid_create();
    navgrid_t* m = navgrid_create_full(
        10, 10, NAVGRID_DIR_8, is_coord_blocked_navgrid);
    coord_t* start = coord_create_full(0, 0);
    coord_t* goal = coord_create_full(9, 9);

    REQUIRE_FALSE(coord_equal(start, goal));

    // obstacle input  (vertical blocking)
    for (int y = 1; y < 10; ++y)
        navgrid_block_coord(m, 5, y);

    route_t* p = find_bfs(m, start, goal, 25, true);

    REQUIRE(p != nullptr);
    CHECK(route_get_success(p) == false);

    route_print(p);
    navgrid_print_ascii_with_route(m, p, 5);
    navgrid_print_ascii_with_visited_count(m, p, 5);

    route_destroy(p);
    coord_destroy(start);
    coord_destroy(goal);
    navgrid_destroy(m);
}

TEST_CASE("DFS: simple route") {
    // navgrid_t* m = navgrid_create();
    navgrid_t* m = navgrid_create_full(
        10, 10, NAVGRID_DIR_8, is_coord_blocked_navgrid);
    coord_t* start = coord_create_full(0, 0);
    coord_t* goal = coord_create_full(9, 9);

    route_t* p = find_dfs(m, start, goal, 100, true);

    REQUIRE(p != nullptr);
    CHECK(route_get_success(p) == true);

    route_print(p);
    navgrid_print_ascii_with_visited_count(m, p, 5);

    route_destroy(p);
    coord_destroy(start);
    coord_destroy(goal);
    navgrid_destroy(m);
}

TEST_CASE("DFS: blocked route") {
    // navgrid_t* m = navgrid_create();
    navgrid_t* m = navgrid_create_full(
        10, 10, NAVGRID_DIR_8, is_coord_blocked_navgrid);
    coord_t* start = coord_create_full(0, 0);
    coord_t* goal = coord_create_full(9, 9);

    REQUIRE_FALSE(coord_equal(start, goal));

    // obstacle input  (vertical blocking)
    for (int y = 1; y < 10; ++y)
        navgrid_block_coord(m, 5, y);

    route_t* p = find_dfs(m, start, goal, 100, true);

    REQUIRE(p != nullptr);
    CHECK(route_get_success(p) == true);

    route_print(p);
    navgrid_print_ascii_with_visited_count(m, p, 5);

    route_destroy(p);
    coord_destroy(start);
    coord_destroy(goal);
    navgrid_destroy(m);
}

TEST_CASE("DFS: blocked route force failed") {
    // navgrid_t* m = navgrid_create();
    navgrid_t* m = navgrid_create_full(
        10, 10, NAVGRID_DIR_8, is_coord_blocked_navgrid);
    coord_t* start = coord_create_full(0, 0);
    coord_t* goal = coord_create_full(9, 9);

    REQUIRE_FALSE(coord_equal(start, goal));

    // obstacle input  (vertical blocking)
    for (int y = 1; y < 10; ++y)
        navgrid_block_coord(m, 5, y);

    route_t* p = find_dfs(m, start, goal, 20, true);

    REQUIRE(p != nullptr);
    CHECK(route_get_success(p) == false);

    route_print(p);
    navgrid_print_ascii_with_visited_count(m, p, 5);

    route_destroy(p);
    coord_destroy(start);
    coord_destroy(goal);
    navgrid_destroy(m);
}

TEST_CASE("dijkstra: simple route") {
    // navgrid_t* m = navgrid_create();
    navgrid_t* m = navgrid_create_full(
        10, 10, NAVGRID_DIR_8, is_coord_blocked_navgrid);
    coord_t* start = coord_create_full(0, 0);
    coord_t* goal = coord_create_full(9, 9);

    route_t* p = find_dijkstra(m, start, goal, default_cost, 1000, true);

    REQUIRE(p != nullptr);
    CHECK(route_get_success(p) == true);

    route_print(p);
    navgrid_print_ascii_with_visited_count(m, p, 5);

    route_destroy(p);
    coord_destroy(start);
    coord_destroy(goal);
    navgrid_destroy(m);
}

TEST_CASE("dijkstra: blocked route") {
    // navgrid_t* m = navgrid_create();
    navgrid_t* m = navgrid_create_full(
        10, 10, NAVGRID_DIR_8, is_coord_blocked_navgrid);
    coord_t* start = coord_create_full(0, 0);
    coord_t* goal = coord_create_full(9, 9);

    REQUIRE_FALSE(coord_equal(start, goal));

    // obstacle input  (vertical blocking)
    for (int y = 1; y < 10; ++y)
        navgrid_block_coord(m, 5, y);

    route_t* p = find_dijkstra(m, start, goal, default_cost, 1000, true);

    REQUIRE(p != nullptr);
    CHECK(route_get_success(p) == true);

    route_print(p);
    navgrid_print_ascii_with_visited_count(m, p, 5);

    route_destroy(p);
    coord_destroy(start);
    coord_destroy(goal);
    navgrid_destroy(m);
}

TEST_CASE("dijkstra: blocked route force failed") {
    // navgrid_t* m = navgrid_create();
    navgrid_t* m = navgrid_create_full(
        10, 10, NAVGRID_DIR_8, is_coord_blocked_navgrid);
    coord_t* start = coord_create_full(0, 0);
    coord_t* goal = coord_create_full(9, 9);

    REQUIRE_FALSE(coord_equal(start, goal));

    // obstacle input  (vertical blocking)
    for (int y = 1; y < 10; ++y)
        navgrid_block_coord(m, 5, y);

    route_t* p = find_dijkstra(m, start, goal, default_cost, 20, true);

    REQUIRE(p != nullptr);
    CHECK(route_get_success(p) == false);

    route_print(p);
    navgrid_print_ascii_with_visited_count(m, p, 5);

    route_destroy(p);
    coord_destroy(start);
    coord_destroy(goal);
    navgrid_destroy(m);
}

TEST_CASE("astar: simple route") {
    // navgrid_t* m = navgrid_create();
    navgrid_t* m = navgrid_create_full(
        10, 10, NAVGRID_DIR_8, is_coord_blocked_navgrid);
    coord_t* start = coord_create_full(0, 0);
    coord_t* goal = coord_create_full(9, 9);

    route_t* p = find_astar(m, start, goal, default_cost, default_heuristic, 
        200, true);

    REQUIRE(p != nullptr);
    CHECK(route_get_success(p) == true);

    route_print(p);
    navgrid_print_ascii_with_visited_count(m, p, 5);

    route_destroy(p);
    coord_destroy(start);
    coord_destroy(goal);
    navgrid_destroy(m);
}

TEST_CASE("astar: blocked route") {
    // navgrid_t* m = navgrid_create();
    navgrid_t* m = navgrid_create_full(10, 10, NAVGRID_DIR_8, is_coord_blocked_navgrid);
    coord_t* start = coord_create_full(0, 0);
    coord_t* goal = coord_create_full(9, 9);

    REQUIRE_FALSE(coord_equal(start, goal));

    // obstacle input  (vertical blocking)
    for (int y = 1; y < 10; ++y)
        navgrid_block_coord(m, 5, y);

    route_t* p = find_astar(m, start, goal, default_cost, default_heuristic, 
        200, true);

    REQUIRE(p != nullptr);
    CHECK(route_get_success(p) == true);

    route_print(p);
    navgrid_print_ascii_with_visited_count(m, p, 5);

    route_destroy(p);
    coord_destroy(start);
    coord_destroy(goal);
    navgrid_destroy(m);
}

TEST_CASE("astar: blocked route force failed ") {
    // navgrid_t* m = navgrid_create();
    navgrid_t* m = navgrid_create_full(
        10, 10, NAVGRID_DIR_8, is_coord_blocked_navgrid);
    coord_t* start = coord_create_full(0, 0);
    coord_t* goal = coord_create_full(9, 9);

    REQUIRE_FALSE(coord_equal(start, goal));

    // obstacle input  (vertical blocking)
    for (int y = 1; y < 10; ++y)
        navgrid_block_coord(m, 5, y);

    route_t* p = find_astar(m, start, goal, default_cost, default_heuristic, 
        20, true);

    REQUIRE(p != nullptr);
    CHECK(route_get_success(p) == false);

    route_print(p);
    navgrid_print_ascii_with_visited_count(m, p, 5);

    route_destroy(p);
    coord_destroy(start);
    coord_destroy(goal);
    navgrid_destroy(m);
}

TEST_CASE("fast_marching: simple route") {
    std::cout << "fast_marching: simple route\n";
    // navgrid_t* m = navgrid_create();
    navgrid_t* m = navgrid_create_full(
        10, 10, NAVGRID_DIR_8, is_coord_blocked_navgrid);
    coord_t* start = coord_create_full(0, 0);
    coord_t* goal = coord_create_full(9, 9);

    route_t* p = find_fast_marching(m, start, goal, 
        default_cost, 1000, true);

    REQUIRE(p != nullptr);
    CHECK(route_get_success(p) == true);

    route_print(p);
    navgrid_print_ascii_with_visited_count(m, p, 5);

    route_destroy(p);
    coord_destroy(start);
    coord_destroy(goal);
    navgrid_destroy(m);
}

TEST_CASE("fast_marching: blocked route") {
    // navgrid_t* m = navgrid_create();
    std::cout << "fast_marching: blocked route\n";

    navgrid_t* m = navgrid_create_full(
        10, 10, NAVGRID_DIR_8, is_coord_blocked_navgrid);
    coord_t* start = coord_create_full(0, 0);
    coord_t* goal = coord_create_full(9, 9);

    REQUIRE_FALSE(coord_equal(start, goal));

    // obstacle input  (vertical blocking)
    for (int y = 1; y < 10; ++y)
        navgrid_block_coord(m, 5, y);

    route_t* p = find_fast_marching(m, start, goal, 
        default_cost, 1200, true);

    REQUIRE(p != nullptr);
    CHECK(route_get_success(p) == true);

    route_print(p);
    navgrid_print_ascii_with_visited_count(m, p, 5);

    route_destroy(p);
    coord_destroy(start);
    coord_destroy(goal);
    navgrid_destroy(m);
}

TEST_CASE("fast_marching: blocked route force failed ") {
    std::cout << "fast_marching: blocked route force failed\n";
    // navgrid_t* m = navgrid_create();
    navgrid_t* m = navgrid_create_full(
        10, 10, NAVGRID_DIR_8, is_coord_blocked_navgrid);
    coord_t* start = coord_create_full(0, 0);
    coord_t* goal = coord_create_full(9, 9);

    REQUIRE_FALSE(coord_equal(start, goal));

    // obstacle input  (vertical blocking)
    for (int y = 1; y < 10; ++y)
        navgrid_block_coord(m, 5, y);

    route_t* p = find_fast_marching(m, start, goal, 
        default_cost, 50, true);

    REQUIRE(p != nullptr);
    CHECK(route_get_success(p) == false);

    route_print(p);
    navgrid_print_ascii_with_visited_count(m, p, 5);

    route_destroy(p);
    coord_destroy(start);
    coord_destroy(goal);
    navgrid_destroy(m);
}

TEST_CASE("greedy_best_first: simple route") {
    std::cout << "greedy_best_first: simple route\n";

    // navgrid_t* m = navgrid_create();
    navgrid_t* m = navgrid_create_full(
        10, 10, NAVGRID_DIR_8, is_coord_blocked_navgrid);
    coord_t* start = coord_create_full(0, 0);
    coord_t* goal = coord_create_full(9, 9);

    route_t* p = find_greedy_best_first(m, start, goal, 
        default_heuristic, 1000, true);

    REQUIRE(p != nullptr);
    CHECK(route_get_success(p) == true);

    route_print(p);
    navgrid_print_ascii_with_visited_count(m, p, 5);

    route_destroy(p);
    coord_destroy(start);
    coord_destroy(goal);
    navgrid_destroy(m);
}

TEST_CASE("greedy_best_first: blocked route") {
    std::cout << "greedy_best_first: blocked route\n";

    // navgrid_t* m = navgrid_create();
    navgrid_t* m = navgrid_create_full(
        10, 10, NAVGRID_DIR_8, is_coord_blocked_navgrid);
    coord_t* start = coord_create_full(0, 0);
    coord_t* goal = coord_create_full(9, 9);

    REQUIRE_FALSE(coord_equal(start, goal));

    // obstacle input  (vertical blocking)
    for (int y = 1; y < 10; ++y)
        navgrid_block_coord(m, 5, y);

    route_t* p = find_greedy_best_first(m, start, goal, 
        default_heuristic, 1200, true);

    REQUIRE(p != nullptr);
    CHECK(route_get_success(p) == true);

    route_print(p);
    navgrid_print_ascii_with_visited_count(m, p, 5);

    route_destroy(p);
    coord_destroy(start);
    coord_destroy(goal);
    navgrid_destroy(m);
}

TEST_CASE("greedy_best_first: blocked route force failed ") {
    std::cout << "greedy_best_first: blocked route force failed\n";

    // navgrid_t* m = navgrid_create();
    navgrid_t* m = navgrid_create_full(
        10, 10, NAVGRID_DIR_8, is_coord_blocked_navgrid);
    coord_t* start = coord_create_full(0, 0);
    coord_t* goal = coord_create_full(9, 9);

    REQUIRE_FALSE(coord_equal(start, goal));

    // obstacle input  (vertical blocking)
    for (int y = 1; y < 10; ++y)
        navgrid_block_coord(m, 5, y);

    route_t* p = find_greedy_best_first(m, start, goal, 
        default_heuristic, 25, true);

    REQUIRE(p != nullptr);
    CHECK(route_get_success(p) == false);

    route_print(p);
    navgrid_print_ascii_with_visited_count(m, p, 5);

    route_destroy(p);
    coord_destroy(start);
    coord_destroy(goal);
    navgrid_destroy(m);
}

TEST_CASE("ida_star: simple route") {
    std::cout << "ida_star: simple route\n";

    // navgrid_t* m = navgrid_create();
    navgrid_t* m = navgrid_create_full(
        10, 10, NAVGRID_DIR_8, is_coord_blocked_navgrid);
    coord_t* start = coord_create_full(0, 0);
    coord_t* goal = coord_create_full(9, 9);

    route_t* p = find_ida_star(m, start, goal, default_cost, nullptr, 
        200, true);

    REQUIRE(p != nullptr);
    CHECK(route_get_success(p) == true);

    route_print(p);
    navgrid_print_ascii_with_visited_count(m, p, 5);

    route_destroy(p);
    coord_destroy(start);
    coord_destroy(goal);
    navgrid_destroy(m);
}

TEST_CASE("ida_star: blocked route") {
    std::cout << "ida_star: blocked route\n";

    // navgrid_t* m = navgrid_create();
    navgrid_t* m = navgrid_create_full(
        10, 10, NAVGRID_DIR_8, is_coord_blocked_navgrid);
    coord_t* start = coord_create_full(0, 0);
    coord_t* goal = coord_create_full(9, 9);

    REQUIRE_FALSE(coord_equal(start, goal));

    // obstacle input  (vertical blocking)
    for (int y = 1; y < 10; ++y)
        navgrid_block_coord(m, 5, y);

    route_t* p = find_ida_star(m, start, goal, default_cost, nullptr, 
        2000, true);

    REQUIRE(p != nullptr);
    CHECK(route_get_success(p) == true);

    route_print(p);
    navgrid_print_ascii_with_visited_count(m, p, 5);

    route_destroy(p);
    coord_destroy(start);
    coord_destroy(goal);
    navgrid_destroy(m);
}

TEST_CASE("ida_star: blocked route force failed ") {
    std::cout << "ida_star: blocked route force failed\n";

    // navgrid_t* m = navgrid_create();
    navgrid_t* m = navgrid_create_full(
        10, 10, NAVGRID_DIR_8, is_coord_blocked_navgrid);
    coord_t* start = coord_create_full(0, 0);
    coord_t* goal = coord_create_full(9, 9);

    REQUIRE_FALSE(coord_equal(start, goal));

    // obstacle input  (vertical blocking)
    for (int y = 1; y < 10; ++y)
        navgrid_block_coord(m, 5, y);

    route_t* p = find_ida_star(m, start, goal, default_cost, nullptr, 
        50, true);

    REQUIRE(p != nullptr);
    CHECK(route_get_success(p) == false);

    route_print(p);
    navgrid_print_ascii_with_visited_count(m, p, 5);

    route_destroy(p);
    coord_destroy(start);
    coord_destroy(goal);
    navgrid_destroy(m);
}

TEST_CASE("fringe_search: simple route") {
    std::cout << "fringe_search: simple route\n";

    // navgrid_t* m = navgrid_create();
    navgrid_t* m = navgrid_create_full(
        10, 10, NAVGRID_DIR_8, is_coord_blocked_navgrid);
    coord_t* start = coord_create_full(0, 0);
    coord_t* goal = coord_create_full(9, 9);

    route_t* p = find_fringe_search(m, start, goal, 
        default_cost, default_heuristic, 1.0,
        200, true);

    REQUIRE(p != nullptr);
    CHECK(route_get_success(p) == true);

    route_print(p);
    navgrid_print_ascii_with_visited_count(m, p, 5);

    route_destroy(p);
    coord_destroy(start);
    coord_destroy(goal);
    navgrid_destroy(m);
}

TEST_CASE("fringe_search: blocked route") {
    std::cout << "fringe_search: blocked route\n";

    // navgrid_t* m = navgrid_create();
    navgrid_t* m = navgrid_create_full(
        10, 10, NAVGRID_DIR_8, is_coord_blocked_navgrid);
    coord_t* start = coord_create_full(0, 0);
    coord_t* goal = coord_create_full(9, 9);

    REQUIRE_FALSE(coord_equal(start, goal));

    // obstacle input  (vertical blocking)
    for (int y = 1; y < 10; ++y)
        navgrid_block_coord(m, 5, y);

    route_t* p = find_fringe_search(m, start, goal, 
        default_cost, 
        default_heuristic, 1.0,
        200, true);

    REQUIRE(p != nullptr);
    CHECK(route_get_success(p) == true);

    route_print(p);
    navgrid_print_ascii_with_visited_count(m, p, 5);

    route_destroy(p);
    coord_destroy(start);
    coord_destroy(goal);
    navgrid_destroy(m);
}

TEST_CASE("fringe_search: blocked route force failed ") {
    std::cout << "fringe_search: blocked route force failed\n";

    // navgrid_t* m = navgrid_create();
    navgrid_t* m = navgrid_create_full(
        10, 10, NAVGRID_DIR_8, is_coord_blocked_navgrid);
    coord_t* start = coord_create_full(0, 0);
    coord_t* goal = coord_create_full(9, 9);

    REQUIRE_FALSE(coord_equal(start, goal));

    // obstacle input  (vertical blocking)
    for (int y = 1; y < 10; ++y)
        navgrid_block_coord(m, 5, y);

    route_t* p = find_fringe_search(m, start, goal, 
        default_cost, 
        default_heuristic, 1.0, 
        20, true);

    REQUIRE(p != nullptr);
    CHECK(route_get_success(p) == false);

    route_print(p);
    navgrid_print_ascii_with_visited_count(m, p, 5);

    route_destroy(p);
    coord_destroy(start);
    coord_destroy(goal);
    navgrid_destroy(m);
}

TEST_CASE("weighted_astar: simple route") {
    std::cout << "weighted_astar: simple route\n";

    // navgrid_t* m = navgrid_create();
    navgrid_t* m = navgrid_create_full(
        10, 10, NAVGRID_DIR_8, is_coord_blocked_navgrid);
    coord_t* start = coord_create_full(0, 0);
    coord_t* goal = coord_create_full(9, 9);

    route_t* p = find_weighted_astar(m, start, goal, 
        default_cost, default_heuristic, 1.0,
        200, true);

    REQUIRE(p != nullptr);
    CHECK(route_get_success(p) == true);

    route_print(p);
    navgrid_print_ascii_with_visited_count(m, p, 5);

    route_destroy(p);
    coord_destroy(start);
    coord_destroy(goal);
    navgrid_destroy(m);
}

TEST_CASE("weighted_astar: blocked route") {
    std::cout << "weighted_astar: blocked route\n";

    // navgrid_t* m = navgrid_create();
    navgrid_t* m = navgrid_create_full(
        10, 10, NAVGRID_DIR_8, is_coord_blocked_navgrid);
    coord_t* start = coord_create_full(0, 0);
    coord_t* goal = coord_create_full(9, 9);

    REQUIRE_FALSE(coord_equal(start, goal));

    // obstacle input  (vertical blocking)
    for (int y = 1; y < 10; ++y)
        navgrid_block_coord(m, 5, y);

    route_t* p = find_weighted_astar(m, start, goal, 
        default_cost, 
        default_heuristic, 1.0,
        200, true);

    REQUIRE(p != nullptr);
    CHECK(route_get_success(p) == true);

    route_print(p);
    navgrid_print_ascii_with_visited_count(m, p, 5);

    route_destroy(p);
    coord_destroy(start);
    coord_destroy(goal);
    navgrid_destroy(m);
}

TEST_CASE("weighted_astar: blocked route force failed ") {
    std::cout << "weighted_astar: blocked route force failed\n";

    // navgrid_t* m = navgrid_create();
    navgrid_t* m = navgrid_create_full(
        10, 10, NAVGRID_DIR_8, is_coord_blocked_navgrid);
    coord_t* start = coord_create_full(0, 0);
    coord_t* goal = coord_create_full(9, 9);

    REQUIRE_FALSE(coord_equal(start, goal));

    // obstacle input  (vertical blocking)
    for (int y = 1; y < 10; ++y)
        navgrid_block_coord(m, 5, y);

    route_t* p = find_weighted_astar(m, start, goal, 
        default_cost, 
        default_heuristic, 1.0, 
        20, true);

    REQUIRE(p != nullptr);
    CHECK(route_get_success(p) == false);

    route_print(p);
    navgrid_print_ascii_with_visited_count(m, p, 5);

    route_destroy(p);
    coord_destroy(start);
    coord_destroy(goal);
    navgrid_destroy(m);
}

TEST_CASE("rta_star: simple route") {
    std::cout << "rta_star: simple route\n";

    // navgrid_t* m = navgrid_create();
    navgrid_t* m = navgrid_create_full(
        10, 10, NAVGRID_DIR_8, is_coord_blocked_navgrid);
    coord_t* start = coord_create_full(0, 0);
    coord_t* goal = coord_create_full(9, 9);

    route_t* p = find_rta_star(m, start, goal, 
        default_cost, default_heuristic, 9,
        200, true);

    REQUIRE(p != nullptr);
    CHECK(route_get_success(p) == true);

    route_print(p);
    navgrid_print_ascii_with_visited_count(m, p, 5);

    route_destroy(p);
    coord_destroy(start);
    coord_destroy(goal);
    navgrid_destroy(m);
}

TEST_CASE("rta_star: blocked route") {
    std::cout << "rta_star: blocked route\n";

    // navgrid_t* m = navgrid_create();
    navgrid_t* m = navgrid_create_full(
        10, 10, NAVGRID_DIR_8, is_coord_blocked_navgrid);
    coord_t* start = coord_create_full(0, 0);
    coord_t* goal = coord_create_full(9, 9);

    REQUIRE_FALSE(coord_equal(start, goal));

    // obstacle input  (vertical blocking)
    for (int y = 1; y < 10; ++y)
        navgrid_block_coord(m, 5, y);

    route_t* p = find_rta_star(m, start, goal, 
        default_cost, 
        default_heuristic, 9,
        200, true);

    REQUIRE(p != nullptr);
    CHECK(route_get_success(p) == true);

    route_print(p);
    navgrid_print_ascii_with_visited_count(m, p, 5);

    route_destroy(p);
    coord_destroy(start);
    coord_destroy(goal);
    navgrid_destroy(m);
}

TEST_CASE("rta_star: blocked route force failed ") {
    std::cout << "rta_star: blocked route force failed\n";

    // navgrid_t* m = navgrid_create();
    navgrid_t* m = navgrid_create_full(
        10, 10, NAVGRID_DIR_8, is_coord_blocked_navgrid);
    coord_t* start = coord_create_full(0, 0);
    coord_t* goal = coord_create_full(9, 9);

    REQUIRE_FALSE(coord_equal(start, goal));

    // obstacle input  (vertical blocking)
    for (int y = 1; y < 10; ++y)
        navgrid_block_coord(m, 5, y);

    route_t* p = find_rta_star(m, start, goal, 
        default_cost, 
        default_heuristic, 9, 
        10, true);

    REQUIRE(p != nullptr);
    CHECK(route_get_success(p) == false);

    route_print(p);
    navgrid_print_ascii_with_visited_count(m, p, 5);

    route_destroy(p);
    coord_destroy(start);
    coord_destroy(goal);
    navgrid_destroy(m);
}

TEST_CASE("sma_star: simple route") {
    std::cout << "sma_star: simple route\n";

    // navgrid_t* m = navgrid_create();
    navgrid_t* m = navgrid_create_full(
        10, 10, NAVGRID_DIR_8, is_coord_blocked_navgrid);
    coord_t* start = coord_create_full(0, 0);
    coord_t* goal = coord_create_full(9, 9);

    route_t* p = find_sma_star(m, start, goal, 
        default_cost, default_heuristic, 1000,
        200, true);

    REQUIRE(p != nullptr);
    CHECK(route_get_success(p) == true);

    route_print(p);
    navgrid_print_ascii_with_visited_count(m, p, 5);

    route_destroy(p);
    coord_destroy(start);
    coord_destroy(goal);
    navgrid_destroy(m);
}

TEST_CASE("sma_star: blocked route") {
    std::cout << "sma_star: blocked route\n";

    // navgrid_t* m = navgrid_create();
    navgrid_t* m = navgrid_create_full(
        10, 10, NAVGRID_DIR_8, is_coord_blocked_navgrid);
    coord_t* start = coord_create_full(0, 0);
    coord_t* goal = coord_create_full(9, 9);

    REQUIRE_FALSE(coord_equal(start, goal));

    // obstacle input  (vertical blocking)
    for (int y = 1; y < 10; ++y)
        navgrid_block_coord(m, 5, y);

    route_t* p = find_sma_star(m, start, goal, 
        default_cost, 
        default_heuristic, 1000,
        200, true);

    REQUIRE(p != nullptr);
    CHECK(route_get_success(p) == true);

    route_print(p);
    navgrid_print_ascii_with_visited_count(m, p, 5);

    route_destroy(p);
    coord_destroy(start);
    coord_destroy(goal);
    navgrid_destroy(m);
}

TEST_CASE("sma_star: blocked route force failed ") {
    std::cout << "sma_star: blocked route force failed\n";

    // navgrid_t* m = navgrid_create();
    navgrid_t* m = navgrid_create_full(
        10, 10, NAVGRID_DIR_8, is_coord_blocked_navgrid);
    coord_t* start = coord_create_full(0, 0);
    coord_t* goal = coord_create_full(9, 9);

    REQUIRE_FALSE(coord_equal(start, goal));

    // obstacle input  (vertical blocking)
    for (int y = 1; y < 10; ++y)
        navgrid_block_coord(m, 5, y);

    route_t* p = find_sma_star(m, start, goal, 
        default_cost, 
        default_heuristic, 1000, 
        10, true);

    REQUIRE(p != nullptr);
    CHECK(route_get_success(p) == false);

    route_print(p);
    navgrid_print_ascii_with_visited_count(m, p, 5);

    route_destroy(p);
    coord_destroy(start);
    coord_destroy(goal);
    navgrid_destroy(m);
}
