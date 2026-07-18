#include "doctest.h"
#include "navsys.h"
#include "console.h"

#include <iostream>

static float bound_cost(
    const navgrid_t*, const coord_t*, const coord_t*, void* userdata) {
    int* calls = static_cast<int*>(userdata);
    if (calls) ++*calls;
    return 1.0f;
}

static float bound_heuristic(
    const coord_t*, const coord_t*, void* userdata) {
    return userdata ? *static_cast<float*>(userdata) : 0.0f;
}

static void bound_move(const coord_t*, void*) {}

static coord_list_t* bound_changed_coords(void*) {
    return nullptr;
}

static bool bound_is_blocked(
    const void*, int x, int y, void* userdata) {
    const coord_t* blocked = static_cast<const coord_t*>(userdata);
    return blocked && blocked->x == x && blocked->y == y;
}

struct reentrant_route_finder_context {
    route_finder_t* finder;
    int calls;
    navsys_status_t bind_status;
    navsys_status_t unbind_status;
    route_t* nested_route;
    int destroy_result;
};

struct reentrant_navgrid_context {
    navgrid_t* navgrid;
    int calls;
    navsys_status_t bind_status;
    navsys_status_t unbind_status;
    bool destroy_attempted;
};

static bool reentrant_navgrid_is_blocked(
    const void*, int, int, void* userdata) {
    reentrant_navgrid_context* context =
        static_cast<reentrant_navgrid_context*>(userdata);
    ++context->calls;
    if (context->calls == 1) {
        context->bind_status = navgrid_bind_is_coord_blocked_func(
            context->navgrid, bound_is_blocked, nullptr);
        context->unbind_status =
            navgrid_unbind_is_coord_blocked_func(context->navgrid);
        navgrid_destroy(context->navgrid);
        context->destroy_attempted = true;
    }
    return false;
}

static float reentrant_route_finder_cost(
    const navgrid_t*, const coord_t*, const coord_t*, void* userdata) {
    reentrant_route_finder_context* context =
        static_cast<reentrant_route_finder_context*>(userdata);
    ++context->calls;
    if (context->calls == 1) {
        context->bind_status = route_finder_bind_cost_func(
            context->finder, bound_cost, nullptr);
        context->unbind_status =
            route_finder_unbind_cost_func(context->finder);
        context->nested_route = route_finder_run(context->finder);
        context->destroy_result =
            route_finder_destroy(context->finder);
    }
    return 1.0f;
}

TEST_CASE("navsys: public status numeric ABI") {
    CHECK(static_cast<int>(NAVSYS_STATUS_OK) == 0);
    CHECK(static_cast<int>(NAVSYS_STATUS_INVALID_ARGUMENT) == -1);
    CHECK(static_cast<int>(NAVSYS_STATUS_OUT_OF_MEMORY) == -2);
    CHECK(static_cast<int>(NAVSYS_STATUS_UNSUPPORTED) == -3);
    CHECK(static_cast<int>(NAVSYS_STATUS_CALLBACK_FAILED) == -4);
    CHECK(static_cast<int>(NAVSYS_STATUS_CORRUPT_STATE) == -5);
    CHECK(static_cast<int>(NAVSYS_STATUS_NOT_FOUND) == -6);
    CHECK(static_cast<int>(NAVSYS_STATUS_INVALIDATED) == -7);
    CHECK(static_cast<int>(NAVSYS_STATUS_NO_PATH) == -8);
    CHECK(static_cast<int>(NAVSYS_STATUS_CANCELLED) == -9);
    CHECK(static_cast<int>(NAVSYS_STATUS_LIMIT_REACHED) == -10);
    CHECK(static_cast<int>(NAVSYS_STATUS_INCOMPLETE) == -11);
    CHECK(static_cast<int>(NAVSYS_STATUS_IN_PROGRESS) == -12);
}

TEST_CASE("navsys: callback bindings commit and unbind as pairs") {
    navgrid_t* navgrid = navgrid_create();
    REQUIRE(navgrid != nullptr);

    coord_t blocked = {1, 0};
    CHECK(navgrid_bind_is_coord_blocked_func(
        navgrid, bound_is_blocked, &blocked) == NAVSYS_STATUS_OK);
    CHECK(navgrid->is_coord_blocked_fn == bound_is_blocked);
    CHECK(navgrid->is_coord_blocked_fn_userdata == &blocked);

    coord_t origin = {0, 0};
    coord_list_t* neighbors =
        navgrid_copy_neighbors(navgrid, origin.x, origin.y);
    REQUIRE(neighbors != nullptr);
    CHECK(coord_list_find(neighbors, &blocked) == -1);
    coord_list_destroy(neighbors);

    CHECK(navgrid_bind_is_coord_blocked_func(
        navgrid, nullptr, nullptr) == NAVSYS_STATUS_INVALID_ARGUMENT);
    CHECK(navgrid->is_coord_blocked_fn == bound_is_blocked);
    CHECK(navgrid->is_coord_blocked_fn_userdata == &blocked);

    route_finder_t* finder = route_finder_create(navgrid);
    REQUIRE(finder != nullptr);
    int cost_calls = 0;
    float heuristic_value = 2.0f;
    CHECK(route_finder_bind_cost_func(
        finder, bound_cost, &cost_calls) == NAVSYS_STATUS_OK);
    CHECK(route_finder_bind_heuristic_func(
        finder, bound_heuristic, &heuristic_value) == NAVSYS_STATUS_OK);
    CHECK(route_finder_get_cost_func(finder) == bound_cost);
    CHECK(route_finder_get_cost_fn_userdata(finder) == &cost_calls);
    CHECK(route_finder_get_heuristic_func(finder) == bound_heuristic);
    CHECK(route_finder_get_heuristic_fn_userdata(finder)
        == &heuristic_value);
    coord_t finder_goal = {2, 2};
    route_finder_set_goal(finder, &finder_goal);
    route_t* bound_route = route_finder_run(finder);
    REQUIRE(bound_route != nullptr);
    CHECK(cost_calls > 0);
    route_destroy(bound_route);

    dstar_lite_t* dsl = dstar_lite_create(navgrid);
    REQUIRE(dsl != nullptr);
    CHECK(dstar_lite_get_cost_func_userdata(dsl) == nullptr);
    CHECK(dstar_lite_get_heuristic_func_userdata(dsl) == nullptr);
    CHECK(dstar_lite_bind_cost_func(
        dsl, bound_cost, &cost_calls) == NAVSYS_STATUS_OK);
    CHECK(dstar_lite_bind_heuristic_func(
        dsl, bound_heuristic, &heuristic_value) == NAVSYS_STATUS_OK);
    CHECK(dstar_lite_bind_move_func(
        dsl, bound_move, &blocked) == NAVSYS_STATUS_OK);
    CHECK(dstar_lite_bind_changed_coords_func(
        dsl, bound_changed_coords, &blocked) == NAVSYS_STATUS_OK);
    CHECK(dstar_lite_get_cost_func_userdata(dsl) == &cost_calls);
    CHECK(dstar_lite_get_heuristic_func_userdata(dsl) == &heuristic_value);
    CHECK(dstar_lite_get_move_func_userdata(dsl) == &blocked);
    CHECK(dstar_lite_get_changed_coords_func_userdata(dsl) == &blocked);

    CHECK(dstar_lite_unbind_changed_coords_func(dsl) == NAVSYS_STATUS_OK);
    CHECK(dstar_lite_unbind_move_func(dsl) == NAVSYS_STATUS_OK);
    CHECK(dstar_lite_unbind_heuristic_func(dsl) == NAVSYS_STATUS_OK);
    CHECK(dstar_lite_unbind_cost_func(dsl) == NAVSYS_STATUS_OK);
    CHECK(dstar_lite_get_changed_coords_func(dsl) == nullptr);
    CHECK(dstar_lite_get_changed_coords_func_userdata(dsl) == nullptr);
    CHECK(dstar_lite_get_move_func(dsl) == nullptr);
    CHECK(dstar_lite_get_move_func_userdata(dsl) == nullptr);
    CHECK(dstar_lite_get_heuristic_func(dsl) == dstar_lite_heuristic);
    CHECK(dstar_lite_get_heuristic_func_userdata(dsl) == nullptr);
    CHECK(dstar_lite_get_cost_func(dsl) == dstar_lite_cost);
    CHECK(dstar_lite_get_cost_func_userdata(dsl) == nullptr);

    CHECK(route_finder_unbind_heuristic_func(finder) == NAVSYS_STATUS_OK);
    CHECK(route_finder_unbind_cost_func(finder) == NAVSYS_STATUS_OK);
    CHECK(route_finder_get_heuristic_func(finder) == euclidean_heuristic);
    CHECK(route_finder_get_heuristic_fn_userdata(finder) == nullptr);
    CHECK(route_finder_get_cost_func(finder) == default_cost);
    CHECK(route_finder_get_cost_fn_userdata(finder) == nullptr);

    CHECK(navgrid_unbind_is_coord_blocked_func(navgrid) == NAVSYS_STATUS_OK);
    CHECK(navgrid->is_coord_blocked_fn == nullptr);
    CHECK(navgrid->is_coord_blocked_fn_userdata == nullptr);

    dstar_lite_destroy(dsl);
    route_finder_destroy(finder);
    navgrid_destroy(navgrid);
}

TEST_CASE("navsys: route finder rejects same-owner callback reentrancy") {
    navgrid_t* navgrid = navgrid_create();
    REQUIRE(navgrid != nullptr);
    route_finder_t* finder = route_finder_create(navgrid);
    REQUIRE(finder != nullptr);

    coord_t goal = {2, 2};
    route_finder_set_goal(finder, &goal);
    reentrant_route_finder_context context = {
        finder,
        0,
        NAVSYS_STATUS_OK,
        NAVSYS_STATUS_OK,
        reinterpret_cast<route_t*>(1),
        0
    };
    REQUIRE(route_finder_bind_cost_func(
        finder, reentrant_route_finder_cost, &context)
        == NAVSYS_STATUS_OK);

    route_t* route = route_finder_run(finder);
    REQUIRE(route != nullptr);
    CHECK(context.calls > 0);
    CHECK(context.bind_status == NAVSYS_STATUS_IN_PROGRESS);
    CHECK(context.unbind_status == NAVSYS_STATUS_IN_PROGRESS);
    CHECK(context.nested_route == nullptr);
    CHECK(context.destroy_result == -1);
    CHECK(route_finder_get_cost_func(finder)
        == reentrant_route_finder_cost);
    CHECK(route_finder_get_cost_fn_userdata(finder) == &context);

    route_destroy(route);
    CHECK(route_finder_destroy(finder) == 0);
    navgrid_destroy(navgrid);
}

TEST_CASE("navsys: navgrid rejects same-owner callback reentrancy") {
    navgrid_t* navgrid = navgrid_create();
    REQUIRE(navgrid != nullptr);
    reentrant_navgrid_context context = {
        navgrid,
        0,
        NAVSYS_STATUS_OK,
        NAVSYS_STATUS_OK,
        false
    };
    REQUIRE(navgrid_bind_is_coord_blocked_func(
        navgrid, reentrant_navgrid_is_blocked, &context)
        == NAVSYS_STATUS_OK);

    coord_list_t* neighbors = navgrid_copy_neighbors(navgrid, 0, 0);
    REQUIRE(neighbors != nullptr);
    CHECK(context.calls > 0);
    CHECK(context.bind_status == NAVSYS_STATUS_IN_PROGRESS);
    CHECK(context.unbind_status == NAVSYS_STATUS_IN_PROGRESS);
    CHECK(context.destroy_attempted);
    CHECK(navgrid_get_is_coord_blocked_fn(navgrid)
        == reentrant_navgrid_is_blocked);
    CHECK(navgrid->is_coord_blocked_fn_userdata == &context);
    CHECK(navgrid_get_width(navgrid) == 0);

    coord_list_destroy(neighbors);
    navgrid_destroy(navgrid);
}

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
