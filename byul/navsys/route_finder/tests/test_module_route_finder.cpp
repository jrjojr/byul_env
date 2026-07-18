#include "doctest.h"

// extern "C" {
    #include "navgrid.h"
    #include "coord.h"
    #include "route.h"
    #include "route_finder.h"
    #include "console.h"
// }

#include <algorithm>
#include <iostream>
#include <limits>

TEST_CASE("route finder capability query matches the dispatcher") {
    const route_finder_type_t supported[] = {
        ROUTE_FINDER_ASTAR,
        ROUTE_FINDER_BFS,
        ROUTE_FINDER_DFS,
        ROUTE_FINDER_DIJKSTRA,
        ROUTE_FINDER_FAST_MARCHING,
        ROUTE_FINDER_FRINGE_SEARCH,
        ROUTE_FINDER_GREEDY_BEST_FIRST,
        ROUTE_FINDER_IDA_STAR,
        ROUTE_FINDER_RTA_STAR,
        ROUTE_FINDER_SMA_STAR,
        ROUTE_FINDER_WEIGHTED_ASTAR,
    };

    for (int value = ROUTE_FINDER_UNKNOWN;
         value <= ROUTE_FINDER_MCTS;
         ++value) {
        const auto type = static_cast<route_finder_type_t>(value);
        const bool expected =
            std::find(std::begin(supported), std::end(supported), type) !=
            std::end(supported);
        CHECK(route_finder_is_supported(type) == expected);
    }

    CHECK_FALSE(route_finder_is_supported(
        static_cast<route_finder_type_t>(-1)));
    CHECK_FALSE(route_finder_is_supported(
        static_cast<route_finder_type_t>(ROUTE_FINDER_MCTS + 1)));
}

TEST_CASE("typed algorithm configs bind atomically and unbind safely") {
    navgrid_t* navgrid = navgrid_create();
    REQUIRE(navgrid != nullptr);
    route_finder_t* finder = route_finder_create(navgrid);
    REQUIRE(finder != nullptr);

    route_finder_fringe_search_config_t fringe = {0.5f};
    route_finder_rta_star_config_t rta = {7};
    route_finder_sma_star_config_t sma = {64};
    route_finder_weighted_astar_config_t weighted = {2.0f};

    CHECK(route_finder_bind_fringe_search_config(finder, &fringe)
        == NAVSYS_STATUS_OK);
    CHECK(route_finder_get_type(finder) == ROUTE_FINDER_FRINGE_SEARCH);
    CHECK(route_finder_get_typedata(finder) == &fringe);

    CHECK(route_finder_bind_rta_star_config(finder, &rta)
        == NAVSYS_STATUS_OK);
    CHECK(route_finder_get_type(finder) == ROUTE_FINDER_RTA_STAR);
    CHECK(route_finder_get_typedata(finder) == &rta);

    CHECK(route_finder_bind_sma_star_config(finder, &sma)
        == NAVSYS_STATUS_OK);
    CHECK(route_finder_get_type(finder) == ROUTE_FINDER_SMA_STAR);
    CHECK(route_finder_get_typedata(finder) == &sma);

    CHECK(route_finder_bind_weighted_astar_config(finder, &weighted)
        == NAVSYS_STATUS_OK);
    CHECK(route_finder_get_type(finder) == ROUTE_FINDER_WEIGHTED_ASTAR);
    CHECK(route_finder_get_typedata(finder) == &weighted);

    route_finder_weighted_astar_config_t invalid = {10.1f};
    route_finder_fringe_search_config_t nan_fringe = {
        std::numeric_limits<float>::quiet_NaN()
    };
    route_finder_weighted_astar_config_t infinite_weight = {
        std::numeric_limits<float>::infinity()
    };
    CHECK(route_finder_bind_weighted_astar_config(finder, &invalid)
        == NAVSYS_STATUS_INVALID_ARGUMENT);
    CHECK(route_finder_bind_fringe_search_config(finder, &nan_fringe)
        == NAVSYS_STATUS_INVALID_ARGUMENT);
    CHECK(route_finder_bind_weighted_astar_config(finder, &infinite_weight)
        == NAVSYS_STATUS_INVALID_ARGUMENT);
    CHECK(route_finder_bind_rta_star_config(finder, nullptr)
        == NAVSYS_STATUS_INVALID_ARGUMENT);
    CHECK(route_finder_get_type(finder) == ROUTE_FINDER_WEIGHTED_ASTAR);
    CHECK(route_finder_get_typedata(finder) == &weighted);

    CHECK(route_finder_unbind_algorithm_config(finder) == NAVSYS_STATUS_OK);
    CHECK(route_finder_get_type(finder) == ROUTE_FINDER_WEIGHTED_ASTAR);
    CHECK(route_finder_get_typedata(finder) == nullptr);
    CHECK(route_finder_unbind_algorithm_config(nullptr)
        == NAVSYS_STATUS_INVALID_ARGUMENT);

    route_finder_destroy(finder);
    navgrid_destroy(navgrid);
}

TEST_CASE("default: route finder") {
    navgrid_t* m = navgrid_create();

    coord_t start;
    coord_init_full(&start, 0, 0);

    coord_t goal; 
    coord_init_full(&goal, 9, 9);

    route_finder_t* rf = route_finder_create(m);
    route_finder_set_start(rf, &start);
    route_finder_set_goal(rf, &goal);

    // obstacle input  (vertical blocking)
    for (int y = 1; y < 10; ++y)
        navgrid_block_coord(m, 5, y);

    route_t* p = route_finder_run(rf);

    CHECK(route_get_success(p) == true);

    route_print(p);
    navgrid_print_ascii_with_visited_count(m, p, 5);

    route_destroy(p);
    route_finder_destroy(rf);
    navgrid_destroy(m);
}

TEST_CASE("route_finder_all: blocked route") {
    coord_t* start = coord_create_full(0, 0);
    coord_t* goal = coord_create_full(9, 9);

    REQUIRE_FALSE(coord_equal(start, goal));

    std::cout << "default\n";
    navgrid_t* m = navgrid_create();
    // obstacle input  (vertical blocking)
    for (int y = 1; y < 10; ++y)
        navgrid_block_coord(m, 5, y);

    route_finder_t* a = route_finder_create(m);
    route_finder_set_goal(a, goal);
    route_finder_set_start(a, start);
    route_finder_enable_debug_mode(a, true);
    route_t* p = nullptr;

    p = route_finder_run(a);
    REQUIRE(p != nullptr);
    CHECK(route_get_success(p) == true);
    route_print(p);
    navgrid_print_ascii_with_visited_count(m, p, 5);
    route_destroy(p);    
    route_finder_destroy(a);
    navgrid_destroy(m);

    m = navgrid_create_full(10, 10, NAVGRID_DIR_8, is_coord_blocked_navgrid);
    // obstacle input  (vertical blocking)
    for (int y = 1; y < 10; ++y)
        navgrid_block_coord(m, 5, y);

    REQUIRE_FALSE(coord_equal(start, goal));

    a = route_finder_create(m);
    route_finder_set_goal(a, goal);
    route_finder_set_start(a, start);
    route_finder_enable_debug_mode(a, true);
    p = nullptr;



    // #include "astar.h"
    std::cout << "astar.h\n";
    route_finder_set_type(a, ROUTE_FINDER_ASTAR);
    p = route_finder_run(a);
    REQUIRE(p != nullptr);
    CHECK(route_get_success(p) == true);
    route_print(p);
    navgrid_print_ascii_with_visited_count(m, p, 5);
    route_destroy(p);

    // #include "bfs.h"
    std::cout << "bfs.h\n";
    route_finder_set_type(a, ROUTE_FINDER_BFS);
    p = route_finder_run(a);
    REQUIRE(p != nullptr);
    CHECK(route_get_success(p) == true);
    route_print(p);
    navgrid_print_ascii_with_visited_count(m, p, 5);
    route_destroy(p);

    // #include "dfs.h"
    std::cout << "dfs.h\n";
    route_finder_set_type(a, ROUTE_FINDER_DFS);
    p = route_finder_run(a);
    REQUIRE(p != nullptr);
    CHECK(route_get_success(p) == true);
    route_print(p);
    navgrid_print_ascii_with_visited_count(m, p, 5);
    route_destroy(p);

    // #include "dijkstra.h"
    std::cout << "dijkstra.h\n";
    route_finder_set_type(a, ROUTE_FINDER_DIJKSTRA);
    p = route_finder_run(a);
    REQUIRE(p != nullptr);
    CHECK(route_get_success(p) == true);
    route_print(p);
    navgrid_print_ascii_with_visited_count(m, p, 5);
    route_destroy(p);

    // #include "fast_marching.h"
    std::cout << "fast_marching.h\n";
    route_finder_set_type(a, ROUTE_FINDER_FAST_MARCHING);
    p = route_finder_run(a);
    REQUIRE(p != nullptr);
    CHECK(route_get_success(p) == true);
    route_print(p);
    navgrid_print_ascii_with_visited_count(m, p, 5);
    route_destroy(p);

    // #include "fringe_search.h"
    std::cout << "fringe_search.h\n";
    route_finder_set_type(a, ROUTE_FINDER_FRINGE_SEARCH);
    p = route_finder_run(a);
    REQUIRE(p != nullptr);
    CHECK(route_get_success(p) == true);
    route_print(p);
    navgrid_print_ascii_with_visited_count(m, p, 5);
    route_destroy(p);

    // #include "greedy_best_first.h"
    std::cout << "greedy_best_first.h\n";
    route_finder_set_type(a, ROUTE_FINDER_GREEDY_BEST_FIRST);
    p = route_finder_run(a);
    REQUIRE(p != nullptr);
    CHECK(route_get_success(p) == true);
    route_print(p);
    navgrid_print_ascii_with_visited_count(m, p, 5);
    route_destroy(p);

    // #include "ida_star.h"
    std::cout << "ida_star.h\n";
    route_finder_set_type(a, ROUTE_FINDER_IDA_STAR);
    p = route_finder_run(a);
    REQUIRE(p != nullptr);
    CHECK(route_get_success(p) == true);
    route_print(p);
    navgrid_print_ascii_with_visited_count(m, p, 5);
    route_destroy(p);

    // #include "rta_star.h"
    std::cout << "rta_star.h\n";
    route_finder_set_type(a, ROUTE_FINDER_RTA_STAR);
    p = route_finder_run(a);
    REQUIRE(p != nullptr);
    CHECK(route_get_success(p) == true);
    route_print(p);
    navgrid_print_ascii_with_visited_count(m, p, 5);
    route_destroy(p);

    // #include "sma_star.h"
    std::cout << "sma_star.h\n";
    route_finder_set_type(a, ROUTE_FINDER_SMA_STAR);
    p = route_finder_run(a);
    REQUIRE(p != nullptr);
    CHECK(route_get_success(p) == true);
    route_print(p);
    navgrid_print_ascii_with_visited_count(m, p, 5);
    route_destroy(p);

    // #include "weighted_astar.h"        
    std::cout << "weighted_astar.h\n";
    route_finder_set_type(a, ROUTE_FINDER_WEIGHTED_ASTAR);
    p = route_finder_run(a);
    REQUIRE(p != nullptr);
    CHECK(route_get_success(p) == true);
    route_print(p);
    navgrid_print_ascii_with_visited_count(m, p, 5);    
    route_destroy(p);

    coord_destroy(start);
    coord_destroy(goal);
    navgrid_destroy(m);
    route_finder_destroy(a);
}
