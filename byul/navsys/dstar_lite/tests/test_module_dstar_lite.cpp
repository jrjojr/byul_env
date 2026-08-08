#include <doctest.h>

#include "dstar_lite.h"

#include "dstar_lite_console.h"

#include "route.h"
#include <stdio.h>
#ifdef _WIN32  
#include <windows.h>  
#else  
#include <unistd.h>  
#endif

#include <thread>
#include <iostream>
#include <array>
#include <atomic>
#include <cstdint>
#include <limits>
#include <queue>

namespace {

navsys_status_t block_outgoing_cost(
    const navgrid_t*, const coord_t* from, const coord_t*,
    float* out_cost, void*) {
    if (!from || !out_cost) return NAVSYS_STATUS_INVALID_ARGUMENT;
    *out_cost = from->x == 0 && from->y == 0
        ? std::numeric_limits<float>::infinity() : 1.0f;
    return NAVSYS_STATUS_OK;
}

navsys_status_t failing_checked_cost(
    const navgrid_t*, const coord_t*, const coord_t*, float*, void*) {
    return NAVSYS_STATUS_CORRUPT_STATE;
}

struct blocking_cost_fixture final {
    std::atomic<bool> entered{false};
    std::atomic<bool> release{false};
};

navsys_status_t blocking_checked_cost(
    const navgrid_t*, const coord_t*, const coord_t*, float* out_cost,
    void* userdata) {
    if (!out_cost || !userdata) return NAVSYS_STATUS_INVALID_ARGUMENT;
    auto& fixture = *static_cast<blocking_cost_fixture*>(userdata);
    fixture.entered.store(true, std::memory_order_release);
    while (!fixture.release.load(std::memory_order_acquire))
        std::this_thread::yield();
    *out_cost = 1.0f;
    return NAVSYS_STATUS_OK;
}

struct blocked_fixture final {
    unsigned mask = 0;
};

constexpr std::array<coord_t, 6> reference_cells{{
    {1, 0}, {0, 1}, {1, 1}, {2, 1}, {1, 2}, {2, 2}}};

bool fixture_blocked(const blocked_fixture& fixture, const coord_t& coord) {
    for (size_t index = 0; index < reference_cells.size(); ++index) {
        if (coord_equal(&reference_cells[index], &coord))
            return (fixture.mask & (1u << index)) != 0;
    }
    return false;
}

navsys_status_t fixture_cost(
    const navgrid_t*, const coord_t* from, const coord_t* to,
    float* out_cost, void* userdata) {
    if (!from || !to || !out_cost || !userdata)
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    const auto& fixture = *static_cast<const blocked_fixture*>(userdata);
    *out_cost = fixture_blocked(fixture, *from)
            || fixture_blocked(fixture, *to)
        ? std::numeric_limits<float>::infinity() : 1.0f;
    return NAVSYS_STATUS_OK;
}

int reference_distance(const blocked_fixture& fixture) {
    std::array<int, 16> distance{};
    distance.fill(-1);
    std::queue<coord_t> pending;
    pending.push(coord_t{0, 0});
    distance[0] = 0;
    constexpr int dx[4]{1, -1, 0, 0};
    constexpr int dy[4]{0, 0, 1, -1};
    while (!pending.empty()) {
        const coord_t current = pending.front();
        pending.pop();
        if (current.x == 3 && current.y == 3)
            return distance[static_cast<size_t>(current.y * 4 + current.x)];
        for (int direction = 0; direction < 4; ++direction) {
            const coord_t next{current.x + dx[direction], current.y + dy[direction]};
            if (next.x < 0 || next.y < 0 || next.x >= 4 || next.y >= 4
                || fixture_blocked(fixture, next)) continue;
            const size_t offset = static_cast<size_t>(next.y * 4 + next.x);
            if (distance[offset] >= 0) continue;
            distance[offset] = distance[static_cast<size_t>(
                current.y * 4 + current.x)] + 1;
            pending.push(next);
        }
    }
    return -1;
}

} // namespace

TEST_CASE("D* Lite canonical planner matches exhaustive reference grids") {
    navgrid_t* grid = navgrid_create_full(
        4, 4, NAVGRID_DIR_4, is_coord_blocked_navgrid);
    REQUIRE(grid);
    const coord_t start{0, 0};
    const coord_t goal{3, 3};
    for (unsigned mask = 0; mask < (1u << reference_cells.size()); ++mask) {
        blocked_fixture fixture{mask};
        dstar_lite_create_info_t info{};
        REQUIRE(dstar_lite_create_info_init(&info, grid, &start, &goal)
            == NAVSYS_STATUS_OK);
        info.cost_callback = fixture_cost;
        info.cost_userdata = &fixture;
        dstar_lite_t* planner = nullptr;
        REQUIRE(dstar_lite_create_ex(&info, &planner) == NAVSYS_STATUS_OK);
        route_t* route = nullptr;
        const navsys_status_t status = dstar_lite_replan(
            planner, nullptr, &route, nullptr);
        const int expected = reference_distance(fixture);
        if (expected < 0) {
            CHECK(status == NAVSYS_STATUS_NO_PATH);
            CHECK(route == nullptr);
        } else {
            REQUIRE(status == NAVSYS_STATUS_OK);
            REQUIRE(route);
            CHECK(route_get_coord_count(route)
                == static_cast<size_t>(expected + 1));
        }
        route_destroy(route);
        dstar_lite_destroy(planner);
    }
    navgrid_destroy(grid);
}

TEST_CASE("D* Lite canonical incremental planner lifecycle") {
    navgrid_t* grid = navgrid_create_full(
        6, 6, NAVGRID_DIR_8, is_coord_blocked_navgrid);
    REQUIRE(grid);
    const coord_t start{0, 0};
    const coord_t goal{5, 5};
    dstar_lite_create_info_t info{};
    REQUIRE(dstar_lite_create_info_init(&info, grid, &start, &goal)
        == NAVSYS_STATUS_OK);
    dstar_lite_t* planner = nullptr;
    REQUIRE(dstar_lite_create_ex(&info, &planner) == NAVSYS_STATUS_OK);

    route_t* first = nullptr;
    dstar_lite_stats_t first_stats{};
    REQUIRE(dstar_lite_replan(planner, nullptr, &first, &first_stats)
        == NAVSYS_STATUS_OK);
    REQUIRE(first);
    CHECK(route_get_coord_count(first) >= 2);
    coord_t first_step{};
    REQUIRE(route_fetch_coord(first, 1, &first_step) == NAVSYS_STATUS_OK);

    const dstar_lite_edge_update_t update{
        start, first_step, 1.0f, std::numeric_limits<float>::infinity()};
    REQUIRE(dstar_lite_notify_edge_changes(planner, &update, 1)
        == NAVSYS_STATUS_OK);
    route_t* repaired = nullptr;
    dstar_lite_stats_t repaired_stats{};
    REQUIRE(dstar_lite_replan(planner, nullptr, &repaired, &repaired_stats)
        == NAVSYS_STATUS_OK);
    coord_t repaired_step{};
    REQUIRE(route_fetch_coord(repaired, 1, &repaired_step)
        == NAVSYS_STATUS_OK);
    CHECK_FALSE(coord_equal(&first_step, &repaired_step));
    CHECK(repaired_stats.edge_changes == 1);
    CHECK(repaired_stats.replans == 2);

    REQUIRE(dstar_lite_set_current_start(planner, &repaired_step)
        == NAVSYS_STATUS_OK);
    dstar_lite_stats_t moved_stats{};
    REQUIRE(dstar_lite_get_stats(planner, &moved_stats) == NAVSYS_STATUS_OK);
    CHECK(moved_stats.km > 0.0f);

    route_destroy(first);
    route_destroy(repaired);
    dstar_lite_destroy(planner);
    navgrid_destroy(grid);
}

TEST_CASE("D* Lite canonical statuses preserve route output on failure") {
    navgrid_t* grid = navgrid_create_full(
        4, 4, NAVGRID_DIR_8, is_coord_blocked_navgrid);
    REQUIRE(grid);
    const coord_t start{0, 0};
    const coord_t goal{3, 3};
    dstar_lite_create_info_t info{};
    REQUIRE(dstar_lite_create_info_init(&info, grid, &start, &goal)
        == NAVSYS_STATUS_OK);
    dstar_lite_t* planner = nullptr;
    REQUIRE(dstar_lite_create_ex(&info, &planner) == NAVSYS_STATUS_OK);

    dstar_lite_replan_options_t options{};
    REQUIRE(dstar_lite_replan_options_init(&options) == NAVSYS_STATUS_OK);
    options.max_expansions = 1;
    route_t* sentinel = reinterpret_cast<route_t*>(static_cast<uintptr_t>(1));
    CHECK(dstar_lite_replan(planner, &options, &sentinel, nullptr)
        == NAVSYS_STATUS_LIMIT_REACHED);
    CHECK(sentinel == reinterpret_cast<route_t*>(static_cast<uintptr_t>(1)));

    REQUIRE(dstar_lite_reset_ex(planner, &start, &goal) == NAVSYS_STATUS_OK);
    REQUIRE(dstar_lite_request_cancel(planner) == NAVSYS_STATUS_OK);
    CHECK(dstar_lite_is_cancel_requested(planner));
    CHECK(dstar_lite_replan(planner, nullptr, &sentinel, nullptr)
        == NAVSYS_STATUS_CANCELLED);
    REQUIRE(dstar_lite_reset_ex(planner, &start, &goal) == NAVSYS_STATUS_OK);
    CHECK_FALSE(dstar_lite_is_cancel_requested(planner));

    REQUIRE(dstar_lite_bind_cost_callback(
        planner, failing_checked_cost, nullptr) == NAVSYS_STATUS_OK);
    CHECK(dstar_lite_replan(planner, nullptr, &sentinel, nullptr)
        == NAVSYS_STATUS_CALLBACK_FAILED);
    REQUIRE(dstar_lite_reset_ex(planner, &start, &goal) == NAVSYS_STATUS_OK);
    REQUIRE(dstar_lite_bind_cost_callback(
        planner, block_outgoing_cost, nullptr) == NAVSYS_STATUS_OK);
    CHECK(dstar_lite_replan(planner, nullptr, &sentinel, nullptr)
        == NAVSYS_STATUS_NO_PATH);

    dstar_lite_destroy(planner);
    navgrid_destroy(grid);
}

TEST_CASE("D* Lite canonical concurrent cancellation is cooperative") {
    navgrid_t* grid = navgrid_create_full(
        20, 20, NAVGRID_DIR_8, is_coord_blocked_navgrid);
    REQUIRE(grid);
    const coord_t start{0, 0};
    const coord_t goal{19, 19};
    blocking_cost_fixture fixture;
    dstar_lite_create_info_t info{};
    REQUIRE(dstar_lite_create_info_init(&info, grid, &start, &goal)
        == NAVSYS_STATUS_OK);
    info.cost_callback = blocking_checked_cost;
    info.cost_userdata = &fixture;
    dstar_lite_t* planner = nullptr;
    REQUIRE(dstar_lite_create_ex(&info, &planner) == NAVSYS_STATUS_OK);

    navsys_status_t result = NAVSYS_STATUS_OK;
    route_t* route = nullptr;
    std::thread worker([&] {
        result = dstar_lite_replan(planner, nullptr, &route, nullptr);
    });
    bool callback_entered = false;
    for (size_t attempt = 0; attempt < 100000; ++attempt) {
        if (fixture.entered.load(std::memory_order_acquire)) {
            callback_entered = true;
            break;
        }
        std::this_thread::yield();
    }
    if (callback_entered)
        CHECK(dstar_lite_request_cancel(planner) == NAVSYS_STATUS_OK);
    fixture.release.store(true, std::memory_order_release);
    worker.join();

    CHECK(callback_entered);
    CHECK(result == NAVSYS_STATUS_CANCELLED);
    CHECK(route == nullptr);
    CHECK(dstar_lite_is_cancel_requested(planner));

    dstar_lite_destroy(planner);
    navgrid_destroy(grid);
}

TEST_CASE("test_dstar_lite_basic") {
        coord_t* start = coord_create_full(0, 0);
        coord_t* goal = coord_create_full(9, 9);

        // navgrid_t* m = navgrid_create();
        navgrid_t* m = navgrid_create_full(
            10, 10, NAVGRID_DIR_8, is_coord_blocked_navgrid);

        dstar_lite_t* dsl = dstar_lite_create_full(m,start, goal,
            dstar_lite_cost, dstar_lite_heuristic, true);

        dstar_lite_set_start(dsl, start);
        dstar_lite_set_goal(dsl, goal);

        route_t* p = dstar_lite_find(dsl);

        CHECK(route_get_success(p));
        printf("[BASIC] route_t* length = %d\n", route_length(p));

        route_print(p);

        dsl_print_ascii_update_count(dsl, p, 5);
        dsl_print_ascii_route(dsl, p, 5);

        route_destroy(p);
        coord_destroy(start);
        coord_destroy(goal);
        dstar_lite_destroy(dsl);   
        navgrid_destroy(m);
}

TEST_CASE("test_dstar_lite_blocked_route") {
    coord_t* start = coord_create_full(0, 0);
    coord_t* goal = coord_create_full(9, 9);

    navgrid_t* m = navgrid_create_full(
        10, 10, NAVGRID_DIR_8, is_coord_blocked_navgrid);

    dstar_lite_t* dsl = dstar_lite_create_full(m, start, goal,
        dstar_lite_cost, dstar_lite_heuristic, true);

        dstar_lite_set_start(dsl, start);
        dstar_lite_set_goal(dsl, goal);

    for (int y = 1; y < 10; y++) navgrid_block_coord(m, 5, y);    

    route_t* p = dstar_lite_find(dsl);

    CHECK(p);
    CHECK(route_get_success(p));

    route_print(p);
    dsl_print_ascii_update_count(dsl, p, 5);

    route_destroy(p);
    coord_destroy(start);
    coord_destroy(goal);
    dstar_lite_destroy(dsl);
    navgrid_destroy(m);
     
}

TEST_CASE("test_dstar_lite_blocked_route_refind_ub1") {
    coord_t* start = coord_create_full(0, 0);
    coord_t* goal = coord_create_full(9, 9);

    navgrid_t* m = navgrid_create_full(
        10, 10, NAVGRID_DIR_8, is_coord_blocked_navgrid);
    dstar_lite_t* dsl = dstar_lite_create_full(m, start, goal,
        dstar_lite_cost, dstar_lite_heuristic, true);

                dstar_lite_set_start(dsl, start);
        dstar_lite_set_goal(dsl, goal);

    for (int y = 1; y < 10; y++) navgrid_block_coord(m, 5, y);    

    route_t* p = dstar_lite_find(dsl);

    CHECK(p);
    CHECK(route_get_success(p));

    route_print(p);
    dsl_print_ascii_update_count(dsl, p, 5);

    coord_t* c0 = coord_create_full(5,0);
    coord_t* c1 = coord_create_full(5,1);

    navgrid_block_coord(m, c0->x, c0->y);
    navgrid_unblock_coord(m, c1->x, c1->y);

    dstar_lite_update_vertex_range(dsl, c0, 0);
    dstar_lite_update_vertex_range(dsl, c1, 0);    
    
    // dstar_lite_update_vertex_by_route(dsl, p);

    route_t* p1 = dstar_lite_find(dsl);

    route_print(p);        
    dsl_print_ascii_update_count(dsl, p1, 5);

    coord_destroy(c0);
    coord_destroy(c1);

    route_destroy(p1);
    route_destroy(p);

    coord_destroy(start);
    coord_destroy(goal);
    dstar_lite_destroy(dsl);
    navgrid_destroy(m);
     
}

TEST_CASE("test_dstar_lite_blocked_route_refind_ub2") {
    coord_t* start = coord_create_full(0, 0);
    coord_t* goal = coord_create_full(9, 9);

    navgrid_t* m = navgrid_create_full(
        10, 10, NAVGRID_DIR_8, is_coord_blocked_navgrid);
    dstar_lite_t* dsl = dstar_lite_create_full(m, start, goal,
        dstar_lite_cost, dstar_lite_heuristic, true);

        dstar_lite_set_start(dsl, start);
        dstar_lite_set_goal(dsl, goal);

    for (int y = 1; y < 10; y++) navgrid_block_coord(m, 5, y);    

    route_t* p = dstar_lite_find(dsl);

    CHECK(p);
    CHECK(route_get_success(p));

    route_print(p);
    dsl_print_ascii_update_count(dsl, p, 5);

    coord_t* c0 = coord_create_full(5,0);
    coord_t* c2 = coord_create_full(5,2);

    navgrid_block_coord(dsl->navgrid, c0->x, c0->y);
    navgrid_unblock_coord(dsl->navgrid, c2->x, c2->y);

    dstar_lite_update_vertex_range(dsl, c0, 1);
    dstar_lite_update_vertex_range(dsl, c2, 1);    
    
    route_t* p1 = dstar_lite_find(dsl);

    route_print(p);        
    dsl_print_ascii_update_count(dsl, p1, 5);

    coord_destroy(c0);
    coord_destroy(c2);

    route_destroy(p1);
    route_destroy(p);

    coord_destroy(start);
    coord_destroy(goal);
    dstar_lite_destroy(dsl);
    navgrid_destroy(m);
     
}

TEST_CASE("test_dstar_lite_blocked_route_refind_ub3") {
    coord_t* start = coord_create_full(0, 0);
    coord_t* goal = coord_create_full(9, 9);

    navgrid_t* m = navgrid_create_full(
        10, 10, NAVGRID_DIR_8, is_coord_blocked_navgrid);
    dstar_lite_t* dsl = dstar_lite_create_full(m, start, goal,
        dstar_lite_cost, dstar_lite_heuristic, true);

        dstar_lite_set_start(dsl, start);
        dstar_lite_set_goal(dsl, goal);

    for (int y = 1; y < 10; y++) navgrid_block_coord(dsl->navgrid, 5, y);    

    route_t* p = dstar_lite_find(dsl);

    CHECK(p);
    CHECK(route_get_success(p));

    route_print(p);    
    dsl_print_ascii_update_count(dsl, p, 5);

    coord_t* c0 = coord_create_full(5,0);
    coord_t* c3 = coord_create_full(5,3);

    navgrid_block_coord(dsl->navgrid, c0->x, c0->y);
    navgrid_unblock_coord(dsl->navgrid, c3->x, c3->y);

    dstar_lite_update_vertex_range(dsl, c0, 1);
    dstar_lite_update_vertex_range(dsl, c3, 1);    
    
    route_t* p1 = dstar_lite_find(dsl);

    route_print(p);        
    dsl_print_ascii_update_count(dsl, p1, 5);

    coord_destroy(c0);
    coord_destroy(c3);

    route_destroy(p1);
    route_destroy(p);

    coord_destroy(start);
    coord_destroy(goal);
    dstar_lite_destroy(dsl);
    navgrid_destroy(m);
     
}

TEST_CASE("test_dstar_lite_blocked_route_refind_ub4") {
    coord_t* start = coord_create_full(0, 0);
    coord_t* goal = coord_create_full(9, 9);

    navgrid_t* m = navgrid_create_full(
        10, 10, NAVGRID_DIR_8, is_coord_blocked_navgrid);
    dstar_lite_t* dsl = dstar_lite_create_full(m, start, goal,
        dstar_lite_cost, dstar_lite_heuristic, true);

        dstar_lite_set_start(dsl, start);
        dstar_lite_set_goal(dsl, goal);

    for (int y = 1; y < 10; y++) navgrid_block_coord(dsl->navgrid, 5, y);    

    route_t* p = dstar_lite_find(dsl);

    CHECK(p);
    CHECK(route_get_success(p));

    dsl_print_ascii_update_count(dsl, p, 5);

    coord_t* c0 = coord_create_full(5,0);
    coord_t* c4 = coord_create_full(5,4);

    navgrid_block_coord(dsl->navgrid, c0->x, c0->y);
    navgrid_unblock_coord(dsl->navgrid, c4->x, c4->y);

    dstar_lite_update_vertex_range(dsl, c0, 1);
    dstar_lite_update_vertex_range(dsl, c4, 1);    
    
    route_t* p1 = dstar_lite_find(dsl);

    route_print(p);        
    dsl_print_ascii_update_count(dsl, p1, 5);

    coord_destroy(c0);
    coord_destroy(c4);

    route_destroy(p1);
    route_destroy(p);

    coord_destroy(start);
    coord_destroy(goal);
    dstar_lite_destroy(dsl);
    navgrid_destroy(m);
     
}

TEST_CASE("test_dstar_lite_blocked_route_refind_ub5") {
    coord_t* start = coord_create_full(0, 0);
    coord_t* goal = coord_create_full(9, 9);

    navgrid_t* m = navgrid_create_full(
        10, 10, NAVGRID_DIR_8, is_coord_blocked_navgrid);
    dstar_lite_t* dsl = dstar_lite_create_full(m, start, goal,
        dstar_lite_cost, dstar_lite_heuristic, true);

        dstar_lite_set_start(dsl, start);
        dstar_lite_set_goal(dsl, goal);

    for (int y = 1; y < 10; y++) navgrid_block_coord(dsl->navgrid, 5, y);    

    route_t* p = dstar_lite_find(dsl);

    CHECK(p);
    CHECK(route_get_success(p));
        
    route_print(p);
    dsl_print_ascii_update_count(dsl, p, 5);

    coord_t* c0 = coord_create_full(5,0);
    coord_t* c5 = coord_create_full(5,5);

    navgrid_block_coord(dsl->navgrid, c0->x, c0->y);
    navgrid_unblock_coord(dsl->navgrid, c5->x, c5->y);

    dstar_lite_update_vertex_range(dsl, c0, 1);
    dstar_lite_update_vertex_range(dsl, c5, 1);
    
    route_t* p1 = dstar_lite_find(dsl);

    route_print(p1);
    dsl_print_ascii_update_count(dsl, p1, 5);

    coord_destroy(c0);
    coord_destroy(c5);

    route_destroy(p1);
    route_destroy(p);

    coord_destroy(start);
    coord_destroy(goal);
    dstar_lite_destroy(dsl);
    navgrid_destroy(m);
     
}

TEST_CASE("test_dstar_lite_blocked_route_default") {
    coord_t* start = coord_create_full(5, 5);
    coord_t* goal = coord_create_full(5, 5);

    navgrid_t* m = navgrid_create_full(
        10, 10, NAVGRID_DIR_8, is_coord_blocked_navgrid);
    dstar_lite_t* dsl = dstar_lite_create(m);
    dstar_lite_set_real_loop_max_retry(dsl, 20);

    printf("Running find_route with default constructor\n");
    route_t* p = dstar_lite_find(dsl);
    CHECK(p);
    // g_assert_true(route_get_success(p));
    route_print(p);
    dsl_print_ascii_update_count(dsl, p, 5);
    route_destroy(p);
    p = NULL;    


    printf("Setting goal to (%d, %d)\n", goal->x, goal->y);
    dstar_lite_reset(dsl);
    dstar_lite_set_goal(dsl, goal);
    p = dstar_lite_find(dsl);
    CHECK(p);
    // g_assert_true(route_get_success(p));
    route_print(p);
    dsl_print_ascii_update_count(dsl, p, 5);
    route_destroy(p);

    coord_destroy(start);
    coord_destroy(goal);
    dstar_lite_destroy(dsl);
    navgrid_destroy(m);
}

TEST_CASE("test_dstar_lite_block_unblock_recover") {
    coord_t* start = coord_create_full(0, 0);
    coord_t* goal = coord_create_full(9, 9);

    navgrid_t* m = navgrid_create_full(
        10, 10, NAVGRID_DIR_8, is_coord_blocked_navgrid);
    dstar_lite_t* dsl = dstar_lite_create_full(m, start, goal,
        dstar_lite_cost, dstar_lite_heuristic, true);

        dstar_lite_set_start(dsl, start);
        dstar_lite_set_goal(dsl, goal);

    route_t* p = dstar_lite_find(dsl);
    CHECK(p);
    CHECK(route_get_success(p));

    route_print(p);
    dsl_print_ascii_update_count(dsl, p, 5);
    route_destroy(p);
    p = NULL;

    coord_t* c = coord_create_full(4, 4);
    coord_t* c0 = coord_create_full(3, 3);
    coord_t* c1 = coord_create_full(4, 3);

    navgrid_block_coord(dsl->navgrid, c->x, c->y);
    navgrid_block_coord(dsl->navgrid, c0->x, c0->y);
    navgrid_block_coord(dsl->navgrid, c1->x, c1->y);

    p = dstar_lite_find(dsl);
    CHECK(p);
    // g_assert_true(route_get_success(p));

    route_print(p);
    dsl_print_ascii_update_count(dsl, p, 5);
    route_destroy(p);
    p = NULL;

    navgrid_unblock_coord(dsl->navgrid, c->x, c->y);
    dstar_lite_update_vertex_range(dsl, c, 1);

    p = dstar_lite_find(dsl);
    CHECK(p);
    // g_assert_true(route_get_success(p));

    route_print(p);
    dsl_print_ascii_update_count(dsl, p, 5);
    route_destroy(p);
    p = NULL;

    coord_set(goal, 7, 6);
    dstar_lite_set_goal(dsl, goal);

    p = dstar_lite_find(dsl);
    CHECK(p);
    // g_assert_true(route_get_success(p));

    route_print(p);
    dsl_print_ascii_update_count(dsl, p, 5);
    route_destroy(p);
    p = NULL;

    coord_destroy(c);
    coord_destroy(c0);
    coord_destroy(c1);

    coord_destroy(start);
    coord_destroy(goal);
    dstar_lite_destroy(dsl);
    navgrid_destroy(m);
}    

TEST_CASE("test_dstar_lite_find_loop") {
    coord_t* start = coord_create_full(0, 0);
    coord_t* goal = coord_create_full(9, 9);

    navgrid_t* m = navgrid_create_full(
        10, 10, NAVGRID_DIR_8, is_coord_blocked_navgrid);
    dstar_lite_t* dsl = dstar_lite_create_full(m, start, goal,
        dstar_lite_cost, dstar_lite_heuristic, true);

        dstar_lite_set_start(dsl, start);
        dstar_lite_set_goal(dsl, goal);

    printf("Generating the first static path using dstar_lite_find()\n");
    route_t* p = dstar_lite_find(dsl);
    CHECK(p);
    CHECK(route_get_success(p));

    route_print(p);
    dsl_print_ascii_update_count(dsl, p, 5);
    route_destroy(p);
    p = NULL;
    dstar_lite_reset(dsl);

    coord_t* c = coord_create_full(4, 4);
    coord_t* c0 = coord_create_full(3, 3);
    coord_t* c1 = coord_create_full(4, 3);
    coord_t* c2 = coord_create_full(5, 3);


    printf("Generating static path after adding obstacles using dstar_lite_find()\n");

    navgrid_block_coord(dsl->navgrid, c->x, c->y);
    navgrid_block_coord(dsl->navgrid, c0->x, c0->y);
    navgrid_block_coord(dsl->navgrid, c1->x, c1->y);

    p = dstar_lite_find(dsl);
    CHECK(p);
    // g_assert_true(route_get_success(p));

    route_print(p);
    dsl_print_ascii_update_count(dsl, p, 5);
    route_destroy(p);
    p = NULL;
    dstar_lite_reset(dsl);

    printf("Generating static path after removing obstacles using dstar_lite_find()\n");

    navgrid_unblock_coord(dsl->navgrid, c->x, c->y);
    dstar_lite_update_vertex_range(dsl, c, 1);

    dstar_lite_find_proto(dsl);
    CHECK(dsl->proto_route);
    // g_assert_true(route_get_success(p));

    route_print(dsl->proto_route);
    dsl_print_ascii_update_count(dsl, dsl->proto_route, 5);

    dstar_lite_reset(dsl);


printf("Changing goal to (7, 6) and generating initial route with dstar_lite_find_proto()\n");

    coord_set(goal, 7, 6);
    dstar_lite_set_goal(dsl, goal);

    dstar_lite_find_proto(dsl);
    
    CHECK(dsl->proto_route);
    // g_assert_true(route_get_success(p));

    route_print(dsl->proto_route);
    dsl_print_ascii_update_count(dsl, dsl->proto_route, 5);
    // route_destroy(p);
    // p = NULL;


    // coord_list_t* route_list = NULL;
coord_list_t* changed_coords = NULL;
float interval_sec = 0.1;

dstar_lite_set_interval_sec(dsl, interval_sec);

dstar_lite_find_loop(dsl);

for (int i = 0; i < 5; i++) {
    printf("interval sec : %.3f, dstar_lite_find_loop() cretes dynamic routes.\n", interval_sec);

    coord_t* coord_i = coord_create_full(i + 4, 5);
    printf("blocked (%d, %d)\n", coord_get_x(coord_i), coord_get_y(coord_i));


    navgrid_block_coord(dsl->navgrid, coord_get_x(coord_i), coord_get_y(coord_i));

    if (i == 2) {
        coord_list_push_back(changed_coords, coord_i);
        dsl->changed_coords_fn = get_changed_coords;
        dsl->changed_coords_fn_userdata = changed_coords;
    }

    CHECK(dsl->real_route);

    route_print(p);
    dsl_print_ascii_update_count(dsl, p, 5);

    coord_destroy(coord_i);
}

coord_list_destroy((coord_list_t*)dsl->changed_coords_fn_userdata);
// g_list_destroy_full(route_list, (GDestroyNotify)route_destroy);

    coord_destroy(c);
    coord_destroy(c0);
    coord_destroy(c1);
    coord_destroy(c2);

    coord_destroy(start);
    coord_destroy(goal);
    dstar_lite_destroy(dsl);
    navgrid_destroy(m);
}    

TEST_CASE("test_dstar_lite_find_static") {
    coord_t* start = coord_create_full(0, 0);
    coord_t* goal = coord_create_full(9, 9);

    navgrid_t* m = navgrid_create_full(10, 10, NAVGRID_DIR_8, is_coord_blocked_navgrid);
    dstar_lite_t* dsl = dstar_lite_create_full(m, start, goal,
        dstar_lite_cost, dstar_lite_heuristic, true);

        dstar_lite_set_start(dsl, start);
        dstar_lite_set_goal(dsl, goal);

printf("Generating the initial static path with dstar_lite_find()\n");

    route_t* p = dstar_lite_find(dsl);
    CHECK(p);
    CHECK(route_get_success(p));
    route_print(p);
    dsl_print_ascii_update_count(dsl, p, 5);
    route_destroy(p);
    dstar_lite_reset(dsl);

    coord_t* c = coord_create_full(4, 4);
    coord_t* c0 = coord_create_full(3, 3);
    coord_t* c1 = coord_create_full(4, 3);
    coord_t* c2 = coord_create_full(5, 3);

printf("Generating path after adding obstacles using dstar_lite_find()\n");

    navgrid_block_coord(dsl->navgrid, c->x, c->y);
    navgrid_block_coord(dsl->navgrid, c0->x, c0->y);
    navgrid_block_coord(dsl->navgrid, c1->x, c1->y);

    p = dstar_lite_find(dsl);
    CHECK(p);
    route_print(p);
    dsl_print_ascii_update_count(dsl, p, 5);
    route_destroy(p);
    dstar_lite_reset(dsl);

printf("Generating path after removing obstacles using dstar_lite_find_proto()\n");

    navgrid_unblock_coord(dsl->navgrid, c->x, c->y);
    dstar_lite_update_vertex_range(dsl, c, 1);

    dstar_lite_find_proto(dsl);
    p = dsl->proto_route;
    CHECK(p);
    route_print(p);
    dsl_print_ascii_update_count(dsl, p, 5);
    dstar_lite_reset(dsl);

printf("Changing goal to (7, 6) and generating path using dstar_lite_find_proto()\n");

    coord_set(goal, 7, 6);
    dstar_lite_set_goal(dsl, goal);
    dstar_lite_find_proto(dsl);
    
    CHECK(dsl->proto_route);
    route_print(dsl->proto_route);
    dsl_print_ascii_update_count(dsl, dsl->proto_route, 5);


printf("Changing goal to (7, 6) and generating real-time path using dstar_lite_find_loop()\n");

    dstar_lite_find_loop(dsl);
    CHECK(dsl->real_route);
    route_print(dsl->real_route);
    dsl_print_ascii_update_count(dsl, dsl->real_route, 5);    

    coord_destroy(c); coord_destroy(c0); coord_destroy(c1); coord_destroy(c2);
    coord_destroy(start); coord_destroy(goal);
    dstar_lite_destroy(dsl);
    navgrid_destroy(m);
}

static void* run_find_loop(void* data) {
    dstar_lite_t* dsl = (dstar_lite_t*)data;
    dstar_lite_find_loop(dsl);
    return NULL;
}

TEST_CASE("test_dstar_lite_find_dynamic") {
    coord_t* start = coord_create_full(0, 0);
    coord_t* goal = coord_create_full(7, 6);

    navgrid_t* m = navgrid_create_full(
        10, 10, NAVGRID_DIR_8, is_coord_blocked_navgrid);

    dstar_lite_t* dsl = dstar_lite_create_full(m, start, goal,
        dstar_lite_cost, dstar_lite_heuristic, true);

        dstar_lite_set_start(dsl, start);
        dstar_lite_set_goal(dsl, goal);

    dstar_lite_find_proto(dsl);
    CHECK(dsl->proto_route);
    route_print(dsl->proto_route);
    dsl_print_ascii_update_count(dsl, dsl->proto_route, 5);

    float interval_sec = 0.1;
    dstar_lite_set_interval_sec(dsl, interval_sec);

    dsl->move_fn = move_to;
    dsl->changed_coords_fn = get_changed_coords;

    std::thread loop_thread([&]() {
        run_find_loop((void*)dsl);
    });    

    coord_t coord_i = {};
    coord_list_t* changed_coords = NULL;
    for (int i = 0; i < 50; i++) {

    std::this_thread::sleep_for(std::chrono::duration<float>(interval_sec * 0.03));


printf("%.3fms passed : checking for dynamic changes\n", i * interval_sec);

        if (i == 2) {
            coord_init_full(&coord_i, i + 1, i);
            printf("blocked (%d, %d)\n", coord_get_x(&coord_i), coord_get_y(&coord_i));

            navgrid_block_coord(dsl->navgrid, coord_get_x(&coord_i), coord_get_y(&coord_i));

            if (changed_coords != NULL) {
                coord_list_destroy(changed_coords);
                // g_list_destroy(changed_coords);
                changed_coords = NULL;
            }
            coord_list_push_back(changed_coords, &coord_i);
            dsl->changed_coords_fn_userdata = changed_coords;
        }

        if (dsl->real_route) {
            route_print(dsl->real_route);
            dsl_print_ascii_update_count(dsl, dsl->real_route, 5);
        }
        if (dsl->real_route && route_get_success(dsl->real_route)) {
            printf("Pathfinding successful\n");
            break;
        }
    }

    loop_thread.join();

    dsl_print_ascii_only_navgrid(dsl);

    route_print(dsl->real_route);
    dsl_print_ascii_update_count(dsl, dsl->real_route, 5);

    coord_list_destroy(changed_coords);

    coord_destroy(start); 
    coord_destroy(goal);
    dstar_lite_destroy(dsl);
    navgrid_destroy(m);
}

TEST_CASE("test_dstar_lite_block_all_around_start") {
    coord_t* start = coord_create_full(0, 0);
    coord_t* goal = coord_create_full(-9, -9);

    navgrid_t* m = navgrid_create_full(
        0, 0, NAVGRID_DIR_8, is_coord_blocked_navgrid);
    dstar_lite_t* dsl = dstar_lite_create_full(m, start, goal,
        dstar_lite_cost, dstar_lite_heuristic, true);

        dstar_lite_set_start(dsl, start);
        dstar_lite_set_goal(dsl, goal);

    navgrid_block_coord(dsl->navgrid, 1, 0);
    navgrid_block_coord(dsl->navgrid, 1, -1);
    navgrid_block_coord(dsl->navgrid, 0, -1);
    navgrid_block_coord(dsl->navgrid, -1, -1);

    route_t* p = dstar_lite_find(dsl);
    CHECK(p);
    CHECK(route_get_success(p));
    route_print(p);
    dsl_print_ascii_update_count(dsl, p, 5);
    route_destroy(p);
    p = NULL;

        
    navgrid_block_coord(dsl->navgrid, -1, 0);
    navgrid_block_coord(dsl->navgrid, -1, 1);
    navgrid_block_coord(dsl->navgrid, 0, 1);

    dstar_lite_set_max_retry(dsl, 200);

    p = dstar_lite_find(dsl);
    CHECK(p);

    CHECK(route_get_success(p));
    route_print(p);
    dsl_print_ascii_update_count(dsl, p, 5);
    route_destroy(p);
    p = NULL;    

    navgrid_block_coord(dsl->navgrid, 1, 1);    

    p = dstar_lite_find(dsl);
    CHECK(p);
    CHECK(!route_get_success(p));
    route_print(p);
    dsl_print_ascii_update_count(dsl, p, 5);
    route_destroy(p);
    p = NULL;        

    route_destroy(p);
    coord_destroy(start);
    coord_destroy(goal);
    dstar_lite_destroy(dsl);
    navgrid_destroy(m);
     
}

TEST_CASE("test_dstar_lite_find_proto") {
    printf("test_dstar_lite_find_proto\n.");
    coord_t* start = coord_create_full(0, 0);
    coord_t* goal = coord_create_full(9, 9);

    // navgrid_t* m = navgrid_create_full(0, 0, NAVGRID_DIR_8);
    navgrid_t* m = navgrid_create();
    dstar_lite_t* dsl = dstar_lite_create_full(m, start, goal,
        dstar_lite_cost, dstar_lite_heuristic, true);

        dstar_lite_set_start(dsl, start);
        dstar_lite_set_goal(dsl, goal);

    for (int y = 1; y < 10; y++) {
        navgrid_block_coord(dsl->navgrid, 5, y);    
    }

    dstar_lite_find_proto(dsl);
    CHECK(dsl->proto_route);
    CHECK(route_get_success(dsl->proto_route));
    route_print(dsl->proto_route);
    dsl_print_ascii_update_count(dsl, dsl->proto_route, 5);

    coord_destroy(start);
    coord_destroy(goal);
    dstar_lite_destroy(dsl);
    navgrid_destroy(m);
}

TEST_CASE("test_dstar_lite_find_proto_reverse") {
    printf("test_dstar_lite_find_proto_reverse\n.");
    coord_t* start = coord_create_full(9, 9);
    coord_t* goal = coord_create_full(0, 0);

    // navgrid_t* m = navgrid_create_full(0, 0, NAVGRID_DIR_8);
    navgrid_t* m = navgrid_create();
    dstar_lite_t* dsl = dstar_lite_create_full(m, start, goal,
        dstar_lite_cost, dstar_lite_heuristic, true);

        dstar_lite_set_start(dsl, start);
        dstar_lite_set_goal(dsl, goal);

    for (int y = 1; y < 10; y++) {
        navgrid_block_coord(dsl->navgrid, 5, y);    
    }

    dstar_lite_find_proto(dsl);
    CHECK(dsl->proto_route);
    CHECK(route_get_success(dsl->proto_route));
    route_print(dsl->proto_route);
    dsl_print_ascii_update_count(dsl, dsl->proto_route, 5);

    coord_destroy(start);
    coord_destroy(goal);
    dstar_lite_destroy(dsl);
    navgrid_destroy(m);
}

TEST_CASE("test_dstar_lite_find_proto_minus_start") {
    printf("test_dstar_lite_find_proto_minus_start\n.");

    coord_t* start = coord_create_full(-9, -9);
    coord_t* goal = coord_create_full(0, 0);

    // navgrid_t* m = navgrid_create_full(0, 0, NAVGRID_DIR_8);
    navgrid_t* m = navgrid_create();
    dstar_lite_t* dsl = dstar_lite_create_full(m, start, goal,
        dstar_lite_cost, dstar_lite_heuristic, true);

        dstar_lite_set_start(dsl, start);
        dstar_lite_set_goal(dsl, goal);

    for (int y = 1; y < 10; y++) {
        navgrid_block_coord(dsl->navgrid, 5, y);    
    }

    dstar_lite_find_proto(dsl);
    CHECK(dsl->proto_route);
    CHECK(route_get_success(dsl->proto_route));
    route_print(dsl->proto_route);
    dsl_print_ascii_update_count(dsl, dsl->proto_route, 5);

    coord_destroy(start);
    coord_destroy(goal);
    dstar_lite_destroy(dsl);
    navgrid_destroy(m);
}

TEST_CASE("test_dstar_lite_find_proto_minus_goal") {
    printf("test_dstar_lite_find_proto_minus_goal\n.");
    coord_t* start = coord_create_full(0, 0);
    coord_t* goal = coord_create_full(-9, -9);

    // navgrid_t* m = navgrid_create_full(0, 0, NAVGRID_DIR_8);
    navgrid_t* m = navgrid_create();
    dstar_lite_t* dsl = dstar_lite_create_full(m, start, goal,
        dstar_lite_cost, dstar_lite_heuristic, true);

        dstar_lite_set_start(dsl, start);
        dstar_lite_set_goal(dsl, goal);

    for (int y = 1; y < 10; y++) {
        navgrid_block_coord(dsl->navgrid, 5, y);    
    }

    dstar_lite_find_proto(dsl);
    CHECK(dsl->proto_route);
    CHECK(route_get_success(dsl->proto_route));
    route_print(dsl->proto_route);
    dsl_print_ascii_update_count(dsl, dsl->proto_route, 5);

    coord_destroy(start);
    coord_destroy(goal);
    dstar_lite_destroy(dsl);
    navgrid_destroy(m);
}

TEST_CASE("test_dstar_lite_find_proto_plus_start_minus_goal") {
    printf("test_dstar_lite_find_proto_plus_start_minus_goal\n.");
    coord_t* start = coord_create_full(7, 7);
    coord_t* goal = coord_create_full(-3, -3);

    // navgrid_t* m = navgrid_create_full(0, 0, NAVGRID_DIR_8);
    navgrid_t* m = navgrid_create();
    dstar_lite_t* dsl = dstar_lite_create_full(m, start, goal,
        dstar_lite_cost, dstar_lite_heuristic, true);

        dstar_lite_set_start(dsl, start);
        dstar_lite_set_goal(dsl, goal);

    for (int y = 1; y < 10; y++) {
        navgrid_block_coord(dsl->navgrid, 5, y);    
    }

    dstar_lite_find_proto(dsl);
    CHECK(dsl->proto_route);
    CHECK(route_get_success(dsl->proto_route));
    route_print(dsl->proto_route);
    dsl_print_ascii_update_count(dsl, dsl->proto_route, 5);

    coord_destroy(start);
    coord_destroy(goal);
    dstar_lite_destroy(dsl);
    navgrid_destroy(m);
}

TEST_CASE("test_dstar_lite_find_proto_minus_start_plus_goal") {
    printf("test_dstar_lite_find_proto_minus_start_plus_goal\n.");
    coord_t* start = coord_create_full(-3, -3);
    coord_t* goal = coord_create_full(7, 7);

    // navgrid_t* m = navgrid_create_full(30, 30, NAVGRID_DIR_8);
    navgrid_t* m = navgrid_create();
    dstar_lite_t* dsl = dstar_lite_create_full(m, start, goal,
        dstar_lite_cost, dstar_lite_heuristic, true);

        dstar_lite_set_start(dsl, start);
        dstar_lite_set_goal(dsl, goal);

    for (int y = 1; y < 10; y++) {
        navgrid_block_coord(dsl->navgrid, 5, y);    
    }

    dstar_lite_find_proto(dsl);
    CHECK(dsl->proto_route);
    CHECK(route_get_success(dsl->proto_route));
    route_print(dsl->proto_route);
    dsl_print_ascii_update_count(dsl, dsl->proto_route, 5);

    coord_t* goal1 = coord_create_full(3, 3);
    dstar_lite_set_goal(dsl, goal1);
    dstar_lite_find_proto(dsl);
    CHECK(dsl->proto_route);
    CHECK(route_get_success(dsl->proto_route));
    route_print(dsl->proto_route);
    dsl_print_ascii_update_count(dsl, dsl->proto_route, 5);    

    coord_destroy(start);
    coord_destroy(goal);
    coord_destroy(goal1);
    dstar_lite_destroy(dsl);
    navgrid_destroy(m);
}
