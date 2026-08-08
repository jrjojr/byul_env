#include "doctest.h"

// extern "C" {
    #include "navgrid.h"
    #include "coord.h"
    #include "route.h"
    #include "route_finder.h"
    #include "route_finder_evaluation.h"
    #include "rta_star.h"
    #include "console.h"
// }

#include <algorithm>
#include <iostream>
#include <limits>
#include <stdexcept>

extern "C" int route_finder_c_abi_reports_type_supported(int type_value);
extern "C" int route_finder_c_abi_exercises_stage4_symbols(void);

struct cancel_fixture {
    int calls;
    int cancel_after;
};

static bool cancel_after_n_polls(void* userdata) {
    cancel_fixture* fixture = static_cast<cancel_fixture*>(userdata);
    ++fixture->calls;
    return fixture->calls >= fixture->cancel_after;
}

static bool throwing_cancel(void*) {
    throw std::runtime_error("cancel callback failure");
}

static float unit_cost_with_userdata(
    const navgrid_t*, const coord_t*, const coord_t*, void* userdata) {
    return userdata ? *static_cast<const float*>(userdata) : -1.0f;
}

static float zero_heuristic_with_userdata(
    const coord_t*, const coord_t*, void* userdata) {
    return userdata ? *static_cast<const float*>(userdata) : -1.0f;
}

struct evaluation_fixture {
    int cost_calls;
    int heuristic_calls;
    float cost;
    float heuristic;
    navsys_status_t status;
};

static navsys_status_t checked_cost(
    const navgrid_t*, const coord_t*, const coord_t*, float* out_cost,
    void* userdata) {
    evaluation_fixture* fixture =
        static_cast<evaluation_fixture*>(userdata);
    ++fixture->cost_calls;
    *out_cost = fixture->cost;
    return fixture->status;
}

static navsys_status_t checked_heuristic(
    const coord_t*, const coord_t*, float* out_estimate, void* userdata) {
    evaluation_fixture* fixture =
        static_cast<evaluation_fixture*>(userdata);
    ++fixture->heuristic_calls;
    *out_estimate = fixture->heuristic;
    return fixture->status;
}

static void check_route_stats_contract(
    const route_t* route,
    const route_finder_run_stats_t& stats) {
    REQUIRE(route != nullptr);

    double route_cost = -1.0;
    route_completion_t completion = ROUTE_COMPLETION_NONE;
    REQUIRE(route_fetch_total_cost(route, &route_cost) == NAVSYS_STATUS_OK);
    REQUIRE(route_fetch_completion(route, &completion) == NAVSYS_STATUS_OK);

    CHECK(route_get_coord_count(route)
        == static_cast<size_t>(stats.route_length));
    CHECK(route_cost == doctest::Approx(stats.route_cost));
    if (stats.complete) {
        CHECK(completion == ROUTE_COMPLETION_COMPLETE);
        CHECK_FALSE(stats.partial);
    } else if (stats.partial) {
        CHECK(completion == ROUTE_COMPLETION_PARTIAL);
    } else {
        CHECK(completion == ROUTE_COMPLETION_NONE);
    }
}

TEST_CASE("[ROUTE-RESULT-001] start equals goal is one complete canonical value") {
    navgrid_t* navgrid = navgrid_create();
    REQUIRE(navgrid != nullptr);
    route_finder_t* finder = route_finder_create(navgrid);
    REQUIRE(finder != nullptr);

    const coord_t endpoint = {4, -3};
    route_finder_set_start(finder, &endpoint);
    route_finder_set_goal(finder, &endpoint);

    route_t* route = nullptr;
    route_finder_run_stats_t stats = {};
    REQUIRE(route_finder_run_ex(finder, &route, &stats)
        == NAVSYS_STATUS_OK);
    REQUIRE(route != nullptr);
    check_route_stats_contract(route, stats);

    CHECK(route_get_coord_count(route) == 1);
    coord_t observed = {};
    CHECK(route_fetch_coord(route, 0, &observed) == NAVSYS_STATUS_OK);
    CHECK(observed.x == endpoint.x);
    CHECK(observed.y == endpoint.y);
    double cost = -1.0;
    route_completion_t completion = ROUTE_COMPLETION_NONE;
    CHECK(route_fetch_total_cost(route, &cost) == NAVSYS_STATUS_OK);
    CHECK(cost == doctest::Approx(0.0));
    CHECK(route_fetch_completion(route, &completion) == NAVSYS_STATUS_OK);
    CHECK(completion == ROUTE_COMPLETION_COMPLETE);

    route_destroy(route);
    route_finder_destroy(finder);
    navgrid_destroy(navgrid);
}

TEST_CASE("route finder capability query matches the dispatcher") {
    const route_finder_type_t supported[] = {
        ROUTE_FINDER_ASTAR,
        ROUTE_FINDER_BFS,
        ROUTE_FINDER_DFS,
        ROUTE_FINDER_DIJKSTRA,
        ROUTE_FINDER_GREEDY_BEST_FIRST,
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
        CHECK(route_finder_is_type_supported(type) == expected);
        CHECK(route_finder_type_get_name(type) != nullptr);
        if (type != ROUTE_FINDER_UNKNOWN)
            CHECK(std::string(route_finder_type_get_name(type)) != "unknown");
        CHECK(std::string(get_route_finder_name(type)) ==
            route_finder_type_get_name(type));
    }

    CHECK(route_finder_c_abi_reports_type_supported(-1) == 0);
    CHECK(route_finder_c_abi_exercises_stage4_symbols() == 1);
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

TEST_CASE("checked settings and run_ex preserve outputs on errors") {
    navgrid_t* navgrid = navgrid_create();
    REQUIRE(navgrid != nullptr);
    route_finder_t* finder = route_finder_create(navgrid);
    REQUIRE(finder != nullptr);

    CHECK(route_finder_set_type_checked(finder, ROUTE_FINDER_BFS)
        == NAVSYS_STATUS_OK);
    CHECK(route_finder_set_type_checked(
        finder, ROUTE_FINDER_BELLMAN_FORD)
        == NAVSYS_STATUS_UNSUPPORTED);
    CHECK(route_finder_get_type(finder) == ROUTE_FINDER_BFS);

    CHECK(route_finder_set_max_retry_checked(finder, 25)
        == NAVSYS_STATUS_OK);
    CHECK(route_finder_set_max_retry_checked(finder, 0)
        == NAVSYS_STATUS_INVALID_ARGUMENT);
    CHECK(route_finder_get_max_retry(finder) == 25);
    route_finder_set_max_retry(finder, 30);
    CHECK(route_finder_get_max_retry(finder) == 30);

    route_t* route = reinterpret_cast<route_t*>(1);
    route_finder_run_stats_t stats = {91, 92, 93.0f, true, true};
    const route_finder_run_stats_t unchanged = stats;
    CHECK(route_finder_run_ex(nullptr, &route, &stats)
        == NAVSYS_STATUS_INVALID_ARGUMENT);
    CHECK(route == reinterpret_cast<route_t*>(1));
    CHECK(stats.total_retry_count == unchanged.total_retry_count);
    CHECK(stats.route_length == unchanged.route_length);
    CHECK(stats.route_cost == unchanged.route_cost);
    CHECK(stats.complete == unchanged.complete);
    CHECK(stats.partial == unchanged.partial);

    route_finder_set_type(finder, ROUTE_FINDER_BELLMAN_FORD);
    CHECK(route_finder_run_ex(finder, &route, &stats)
        == NAVSYS_STATUS_UNSUPPORTED);
    CHECK(route == reinterpret_cast<route_t*>(1));
    CHECK(stats.total_retry_count == unchanged.total_retry_count);

    route_finder_destroy(finder);
    navgrid_destroy(navgrid);
}

TEST_CASE("RTA star legacy config constructors provide their promised ABI") {
    rta_star_config_t* defaults = rta_star_config_create();
    REQUIRE(defaults != nullptr);
    CHECK(defaults->depth_limit == 5);

    rta_star_config_t* configured = rta_star_config_create_full(7);
    REQUIRE(configured != nullptr);
    CHECK(configured->depth_limit == 7);
    CHECK(rta_star_config_create_full(0) == nullptr);

    rta_star_config_destroy(defaults);
    rta_star_config_destroy(configured);
    rta_star_config_destroy(nullptr);
}

TEST_CASE("status evaluation bindings retain userdata and reject bad values") {
    navgrid_t* navgrid = navgrid_create();
    REQUIRE(navgrid != nullptr);
    route_finder_t* finder = route_finder_create(navgrid);
    REQUIRE(finder != nullptr);
    const coord_t goal = {3, 3};
    route_finder_set_goal(finder, &goal);

    evaluation_fixture fixture = {0, 0, 1.0f, 0.0f, NAVSYS_STATUS_OK};
    REQUIRE(route_finder_bind_cost_func_ex(
        finder, checked_cost, &fixture) == NAVSYS_STATUS_OK);
    REQUIRE(route_finder_bind_heuristic_func_ex(
        finder, checked_heuristic, &fixture) == NAVSYS_STATUS_OK);

    route_t* route = nullptr;
    route_finder_run_stats_t stats = {};
    REQUIRE(route_finder_run_ex(finder, &route, &stats) == NAVSYS_STATUS_OK);
    CHECK(fixture.cost_calls > 0);
    CHECK(fixture.heuristic_calls > 0);
    route_destroy(route);

    route = reinterpret_cast<route_t*>(1);
    stats = {91, 92, 93.0f, true, true};
    fixture.cost = std::numeric_limits<float>::quiet_NaN();
    CHECK(route_finder_run_ex(finder, &route, &stats)
        == NAVSYS_STATUS_CALLBACK_FAILED);
    CHECK(route == reinterpret_cast<route_t*>(1));
    CHECK(stats.total_retry_count == 91);

    fixture.cost = 1.0f;
    fixture.status = NAVSYS_STATUS_INVALID_ARGUMENT;
    CHECK(route_finder_run_ex(finder, &route, &stats)
        == NAVSYS_STATUS_CALLBACK_FAILED);
    CHECK(route == reinterpret_cast<route_t*>(1));

    REQUIRE(route_finder_unbind_cost_func(finder) == NAVSYS_STATUS_OK);
    REQUIRE(route_finder_unbind_heuristic_func(finder) == NAVSYS_STATUS_OK);
    fixture.status = NAVSYS_STATUS_OK;
    fixture.cost_calls = 0;
    fixture.heuristic_calls = 0;
    route = nullptr;
    stats = {};
    REQUIRE(route_finder_run_ex(finder, &route, &stats) == NAVSYS_STATUS_OK);
    CHECK(fixture.cost_calls == 0);
    CHECK(fixture.heuristic_calls == 0);
    route_destroy(route);

    route_finder_destroy(finder);
    navgrid_destroy(navgrid);
}

TEST_CASE("init_full retains both callback userdata values") {
    navgrid_t* navgrid = navgrid_create();
    REQUIRE(navgrid != nullptr);
    const coord_t start = {0, 0};
    const coord_t goal = {2, 2};
    float cost_userdata = 1.0f;
    float heuristic_userdata = 0.0f;
    route_finder_t finder = {};

    REQUIRE(route_finder_init_full(
        &finder, navgrid, &start, &goal, ROUTE_FINDER_ASTAR, nullptr,
        100, false, unit_cost_with_userdata, &cost_userdata,
        zero_heuristic_with_userdata, &heuristic_userdata) == 0);
    CHECK(route_finder_get_cost_fn_userdata(&finder) == &cost_userdata);
    CHECK(route_finder_get_heuristic_fn_userdata(&finder)
        == &heuristic_userdata);

    route_finder_free(&finder);
    navgrid_destroy(navgrid);
}

TEST_CASE("canonical evaluation functions validate inputs and units") {
    navgrid_t* navgrid = navgrid_create_full(
        4, 4, NAVGRID_DIR_8, is_coord_blocked_navgrid);
    REQUIRE(navgrid != nullptr);
    const coord_t origin = {0, 0};
    const coord_t diagonal = {1, 1};
    const coord_t goal = {3, 4};
    float value = -7.0f;

    CHECK(route_finder_cost_unit(
        navgrid, &origin, &diagonal, &value, nullptr) == NAVSYS_STATUS_OK);
    CHECK(value == doctest::Approx(1.0f));
    CHECK(route_finder_cost_diagonal(
        navgrid, &origin, &diagonal, &value, nullptr) == NAVSYS_STATUS_OK);
    CHECK(value == doctest::Approx(1.41421356237f));

    value = -7.0f;
    CHECK(route_finder_heuristic_euclidean(
        &origin, &goal, &value, nullptr) == NAVSYS_STATUS_OK);
    CHECK(value == doctest::Approx(5.0f));
    CHECK(route_finder_heuristic_manhattan(
        &origin, &goal, &value, nullptr) == NAVSYS_STATUS_OK);
    CHECK(value == doctest::Approx(7.0f));
    CHECK(route_finder_heuristic_chebyshev(
        &origin, &goal, &value, nullptr) == NAVSYS_STATUS_OK);
    CHECK(value == doctest::Approx(4.0f));
    CHECK(route_finder_heuristic_default(
        &origin, &goal, &value, nullptr) == NAVSYS_STATUS_OK);
    CHECK(value == doctest::Approx(0.0f));

    value = 13.0f;
    CHECK(route_finder_cost_unit(
        nullptr, &origin, &diagonal, &value, nullptr)
        == NAVSYS_STATUS_INVALID_ARGUMENT);
    CHECK(value == doctest::Approx(13.0f));
    CHECK(route_finder_heuristic_euclidean(
        nullptr, &goal, &value, nullptr)
        == NAVSYS_STATUS_INVALID_ARGUMENT);
    CHECK(value == doctest::Approx(13.0f));

    navgrid_destroy(navgrid);
}

TEST_CASE("IDA star dispatch does not mutate the bound heuristic") {
    navgrid_t* navgrid = navgrid_create();
    REQUIRE(navgrid != nullptr);
    route_finder_t* finder = route_finder_create(navgrid);
    REQUIRE(finder != nullptr);
    const coord_t goal = {3, 3};
    float zero = 0.0f;
    route_finder_set_goal(finder, &goal);
    REQUIRE(route_finder_bind_heuristic_func(
        finder, zero_heuristic_with_userdata, &zero) == NAVSYS_STATUS_OK);
    route_finder_set_type(finder, ROUTE_FINDER_IDA_STAR);

    route_t* route = route_finder_run(finder);
    REQUIRE(route != nullptr);
    CHECK(route_finder_get_heuristic_func(finder)
        == zero_heuristic_with_userdata);

    route_destroy(route);
    route_finder_destroy(finder);
    navgrid_destroy(navgrid);
}

TEST_CASE("run_ex separates success no-path and limit termination") {
    navgrid_t* navgrid = navgrid_create();
    REQUIRE(navgrid != nullptr);
    route_finder_t* finder = route_finder_create(navgrid);
    REQUIRE(finder != nullptr);
    coord_t goal = {9, 9};
    route_finder_set_goal(finder, &goal);

    route_t* route = nullptr;
    route_finder_run_stats_t stats = {};
    CHECK(route_finder_run_ex(finder, &route, &stats)
        == NAVSYS_STATUS_OK);
    REQUIRE(route != nullptr);
    CHECK(stats.complete);
    CHECK_FALSE(stats.partial);
    CHECK(stats.route_length == route_length(route));
    CHECK(stats.total_retry_count == route_get_total_retry_count(route));
    check_route_stats_contract(route, stats);
    route_destroy(route);

    navgrid_t* bounded = navgrid_create_full(
        10, 10, NAVGRID_DIR_8, is_coord_blocked_navgrid);
    REQUIRE(bounded != nullptr);
    route_finder_set_navgrid(finder, bounded);
    navgrid_destroy(navgrid);
    navgrid = bounded;
    for (int y = 0; y < 10; ++y)
        navgrid_block_coord(navgrid, 5, y);
    route = nullptr;
    stats = {};
    CHECK(route_finder_set_max_retry_checked(finder, 1000)
        == NAVSYS_STATUS_OK);
    CHECK(route_finder_run_ex(finder, &route, &stats)
        == NAVSYS_STATUS_NO_PATH);
    REQUIRE(route != nullptr);
    CHECK_FALSE(stats.complete);
    CHECK(stats.total_retry_count < 1000);
    check_route_stats_contract(route, stats);
    route_destroy(route);

    navgrid_destroy(navgrid);
    navgrid = navgrid_create();
    REQUIRE(navgrid != nullptr);
    route_finder_set_navgrid(finder, navgrid);
    route = nullptr;
    stats = {};
    CHECK(route_finder_set_max_retry_checked(finder, 1)
        == NAVSYS_STATUS_OK);
    CHECK(route_finder_run_ex(finder, &route, &stats)
        == NAVSYS_STATUS_LIMIT_REACHED);
    REQUIRE(route != nullptr);
    CHECK_FALSE(stats.complete);
    CHECK(stats.total_retry_count >= 1);
    check_route_stats_contract(route, stats);
    route_destroy(route);

    route_finder_destroy(finder);
    navgrid_destroy(navgrid);
}

TEST_CASE("run options cooperatively cancel every dispatcher algorithm") {
    const route_finder_type_t supported[] = {
        ROUTE_FINDER_ASTAR,
        ROUTE_FINDER_BFS,
        ROUTE_FINDER_DFS,
        ROUTE_FINDER_DIJKSTRA,
        ROUTE_FINDER_GREEDY_BEST_FIRST,
        ROUTE_FINDER_WEIGHTED_ASTAR,
    };
    navgrid_t* navgrid = navgrid_create();
    REQUIRE(navgrid != nullptr);
    route_finder_t* finder = route_finder_create(navgrid);
    REQUIRE(finder != nullptr);
    coord_t goal = {9, 9};
    route_finder_set_goal(finder, &goal);

    for (route_finder_type_t type : supported) {
        CAPTURE(type);
        REQUIRE(route_finder_set_type_checked(finder, type)
            == NAVSYS_STATUS_OK);
        cancel_fixture fixture = {0, 1};
        route_finder_run_options_t options = {
            static_cast<uint32_t>(sizeof(route_finder_run_options_t)),
            cancel_after_n_polls,
            &fixture
        };
        route_t* route = nullptr;
        route_finder_run_stats_t stats = {};
        CHECK(route_finder_run_with_options(
            finder, &options, &route, &stats) == NAVSYS_STATUS_CANCELLED);
        CHECK(fixture.calls == 1);
        REQUIRE(route != nullptr);
        CHECK_FALSE(stats.complete);
        CHECK(stats.route_length == route_length(route));
        check_route_stats_contract(route, stats);
        route_destroy(route);
    }

    route_t* route = reinterpret_cast<route_t*>(1);
    route_finder_run_stats_t stats = {91, 92, 93.0f, true, true};
    route_finder_run_options_t invalid = {};
    CHECK(route_finder_run_with_options(
        finder, &invalid, &route, &stats) == NAVSYS_STATUS_INVALID_ARGUMENT);
    CHECK(route == reinterpret_cast<route_t*>(1));
    CHECK(stats.total_retry_count == 91);

    route_finder_run_options_t throwing = {
        static_cast<uint32_t>(sizeof(route_finder_run_options_t)),
        throwing_cancel,
        nullptr
    };
    CHECK(route_finder_run_with_options(
        finder, &throwing, &route, &stats) == NAVSYS_STATUS_CALLBACK_FAILED);
    CHECK(route == reinterpret_cast<route_t*>(1));
    CHECK(stats.total_retry_count == 91);

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
