#include <doctest.h>

#include "dstar_lite_tick.h"

#include "dstar_lite_console.h"

#include <iostream>

TEST_CASE("D* Lite canonical tick is deterministic and lifecycle-safe") {
    navgrid_t* grid_a = navgrid_create_full(5, 5, NAVGRID_DIR_8, nullptr);
    navgrid_t* grid_b = navgrid_create_full(5, 5, NAVGRID_DIR_8, nullptr);
    REQUIRE(grid_a);
    REQUIRE(grid_b);
    const coord_t start{0, 0};
    const coord_t goal{4, 4};
    dstar_lite_create_info_t planner_info_a{};
    dstar_lite_create_info_t planner_info_b{};
    REQUIRE(dstar_lite_create_info_init(
        &planner_info_a, grid_a, &start, &goal) == NAVSYS_STATUS_OK);
    REQUIRE(dstar_lite_create_info_init(
        &planner_info_b, grid_b, &start, &goal) == NAVSYS_STATUS_OK);
    dstar_lite_t* planner_a = nullptr;
    dstar_lite_t* planner_b = nullptr;
    REQUIRE(dstar_lite_create_ex(&planner_info_a, &planner_a)
        == NAVSYS_STATUS_OK);
    REQUIRE(dstar_lite_create_ex(&planner_info_b, &planner_b)
        == NAVSYS_STATUS_OK);

    dstar_lite_tick_create_info_t tick_info_a{};
    dstar_lite_tick_create_info_t tick_info_b{};
    REQUIRE(dstar_lite_tick_create_info_init(&tick_info_a, planner_a)
        == NAVSYS_STATUS_OK);
    REQUIRE(dstar_lite_tick_create_info_init(&tick_info_b, planner_b)
        == NAVSYS_STATUS_OK);
    dstar_lite_tick_t* controller_a = nullptr;
    dstar_lite_tick_t* controller_b = nullptr;
    REQUIRE(dstar_lite_tick_create_ex(&tick_info_a, &controller_a)
        == NAVSYS_STATUS_OK);
    REQUIRE(dstar_lite_tick_create_ex(&tick_info_b, &controller_b)
        == NAVSYS_STATUS_OK);
    CHECK(dstar_lite_tick_get_state(controller_a)
        == DSTAR_LITE_TICK_STATE_DETACHED);

    uint32_t steps = 99;
    REQUIRE(dstar_lite_tick_advance(controller_a, 0.5f, &steps)
        == NAVSYS_STATUS_OK);
    CHECK(steps == 0);
    REQUIRE(dstar_lite_tick_advance(controller_a, 0.5f, &steps)
        == NAVSYS_STATUS_OK);
    CHECK(steps == 1);
    REQUIRE(dstar_lite_tick_advance(controller_b, 1.0f, &steps)
        == NAVSYS_STATUS_OK);
    CHECK(steps == 1);
    coord_t position_a{};
    coord_t position_b{};
    REQUIRE(dstar_lite_tick_fetch_position(controller_a, &position_a)
        == NAVSYS_STATUS_OK);
    REQUIRE(dstar_lite_tick_fetch_position(controller_b, &position_b)
        == NAVSYS_STATUS_OK);
    CHECK(coord_equal(&position_a, &position_b));

    tick_t* tick = tick_create();
    REQUIRE(tick);
    CHECK(dstar_lite_tick_start(controller_a, tick) == NAVSYS_STATUS_OK);
    CHECK(dstar_lite_tick_start(controller_a, tick)
        == NAVSYS_STATUS_IN_PROGRESS);
    dstar_lite_tick_destroy(controller_a);
    CHECK(tick_list_attached(tick, nullptr, 0) == 0);

    tick_info_b.speed_m_per_sec = 0.0f;
    dstar_lite_tick_t* untouched = controller_b;
    CHECK(dstar_lite_tick_create_ex(&tick_info_b, &untouched)
        == NAVSYS_STATUS_INVALID_ARGUMENT);
    CHECK(untouched == controller_b);

    dstar_lite_tick_destroy(controller_b);
    tick_destroy(tick);
    dstar_lite_destroy(planner_a);
    dstar_lite_destroy(planner_b);
    navgrid_destroy(grid_a);
    navgrid_destroy(grid_b);
}

static dstar_lite_t* create_dummy_dsl(navgrid_t* ng, int width, int height) {
    coord_t start = {0, 0};
    coord_t goal = {width, height};

    dstar_lite_t* dsl = dstar_lite_create_full(ng, &start, &goal,
        dstar_lite_cost, dstar_lite_heuristic, true);
    
    for (int y = 1; y < 10; y++) navgrid_block_coord(ng, 5, y);    
     
    return dsl;
}

TEST_CASE("D* Lite Tick Prepare attaches correctly") {
    tick_t* tk = tick_create();
    navgrid_t* ng = navgrid_create_full(10, 10, NAVGRID_DIR_8, nullptr);

    dstar_lite_t* dsl = create_dummy_dsl(ng, 10, 10);
    dstar_lite_tick_t* dst = dstar_lite_tick_create(dsl);

    dstar_lite_tick_prepare(dst, tk);
    CHECK(dst->ticked == true);
    CHECK(dst->base->force_quit == false);
    CHECK(dst->base->real_route != nullptr);
    CHECK(route_get_coord_count(dst->base->real_route) == 1);

    tick_entry_t funcs[10] = {};
    int count = tick_list_attached(tk, funcs, 10);
    bool found = false;
    for (int i = 0; i < count; ++i) {
        if (funcs[i].func && funcs[i].context == dst) {
            found = true;
            break;
        }
    }
    CHECK(found);

    dstar_lite_tick_complete(dst, tk);
    tick_destroy(tk);
    dstar_lite_destroy(dsl);
    dstar_lite_tick_destroy(dst);
    navgrid_destroy(ng);
}

TEST_CASE("D* Lite Tick Update progresses time and halts on goal") {
    tick_t* tk = tick_create();
    navgrid_t* ng = navgrid_create_full(10, 10, NAVGRID_DIR_8, nullptr);

    dstar_lite_t* dsl = create_dummy_dsl(ng, 10, 10);
    dstar_lite_tick_t* dst = dstar_lite_tick_create(dsl);

    dstar_lite_tick_prepare(dst, tk);

    // 강제로 goal = start로 설정 (즉시 완료됨)
    dst->base->goal = dst->base->start;

    dstar_lite_tick_update(dst, 0.1f);
    CHECK(dst->ticked == false);
    CHECK(route_get_success(dst->base->real_route));

    dstar_lite_tick_complete(dst, tk);
    tick_destroy(tk);
    dstar_lite_destroy(dsl);
    dstar_lite_tick_destroy(dst);
    navgrid_destroy(ng);
}

TEST_CASE("D* Lite Tick halts after max_time") {
    tick_t* tk = tick_create();
    navgrid_t* ng = navgrid_create_full(10, 10, NAVGRID_DIR_8, nullptr);

    dstar_lite_t* dsl = create_dummy_dsl(ng, 10, 10);
    dstar_lite_tick_t* dst = dstar_lite_tick_create(dsl);    

    dstar_lite_tick_prepare(dst, tk);

    // 경로는 멀지만 시간은 매우 짧음 → 실패
    dst->max_time = 0.01f;
    dstar_lite_tick_update(dst, 0.1f);
    CHECK(dst->ticked == false);
    CHECK(!route_get_success(dst->base->real_route));

    dstar_lite_tick_complete(dst, tk);
    tick_destroy(tk);
    dstar_lite_destroy(dsl);
    dstar_lite_tick_destroy(dst);
    navgrid_destroy(ng);
}

TEST_CASE("D* Lite Tick max_time") {
    tick_t* tk = tick_create();
    navgrid_t* ng = navgrid_create_full(10, 10, NAVGRID_DIR_8, nullptr);

    coord_t start = {0, 0};
    coord_t goal = {9, 9};

    dstar_lite_t* dsl = dstar_lite_create_full(ng, &start, &goal,
        dstar_lite_cost, dstar_lite_heuristic, true);

    dstar_lite_tick_t* dst = dstar_lite_tick_create(dsl);            
    
    dstar_lite_enable_debug_mode(dsl, true);

    // dstar_lite_find_proto(dst);
    dstar_lite_find_proto(dsl);
    std::cout << "Tick max_time proto route : \n";
    route_print(dst->base->proto_route);
    dsl_print_ascii_update_count(dsl, dst->base->proto_route, 2);
    dst->base->changed_coords_fn = get_changed_coords;
    coord_list_t* list = coord_list_create();

    dstar_lite_tick_prepare(dst, tk);

    dst->max_time = 1000.0f;

    float dt = 1.0f;

    for (int i = 0; i < 50; i++) {
        // printf("dt : %f,\n", dt * i);

        // dstar_lite_tick_update(dst, dt);
        printf("current time : %f\n", dst->cur_time);
        navgrid_block_coord(ng, 5, i+1);
        coord_t c = {5, i+4};
        coord_list_push_back(list, &c);
        dst->base->changed_coords_fn_userdata = (void*)list;

        tick_update(tk, dt);        

        route_print(dst->base->real_route);
        dsl_print_ascii_update_count(dsl, dst->base->real_route, 2);
        // std::this_thread::sleep_for(
        //     std::chrono::duration<float>(dt));
    }    

    // int count;
    // void** values = coord_hash_values(dst->base->real_route->visited_count, &count);
    // for ( int i=0; i<count; i++){
    //     int val = *(int*)values[i];
    //     std::cout << "value : " << i << ", " << val << std::endl;
    // }

    CHECK(route_get_success(dst->base->real_route));
    
    dstar_lite_tick_complete(dst, tk);
    tick_destroy(tk);
    dstar_lite_destroy(dsl);
    navgrid_destroy(ng);
    coord_list_destroy(list);
    dstar_lite_tick_destroy(dst);
    // free(values);
}

TEST_CASE("D* Lite Tick Prepare Full sets all parameters correctly and executes 1m movement") {
    tick_t* tk = tick_create();
    navgrid_t* ng = navgrid_create_full(10, 10, NAVGRID_DIR_8, nullptr);

    coord_t start = {0, 0};
    coord_t goal = {9, 9};

    dstar_lite_t* dsl = dstar_lite_create_full(ng, &start, &goal,
        dstar_lite_cost, dstar_lite_heuristic, true);
    dstar_lite_tick_t* dst = dstar_lite_tick_create(dsl);

    // 초기 경로 생성
    dstar_lite_find_proto(dsl);

    // 동적 장애물 변화 등록 (없어도 문제 없음)
    dst->base->changed_coords_fn = get_changed_coords;
    coord_list_t* list = coord_list_create();
    coord_t c = {5, 1};
    coord_list_push_back(list, &c);
    dst->base->changed_coords_fn_userdata = (void*)list;

    // 1m 이동 설정
    float unit_m = 1.0f;
    float speed_sec = 1.0f;
    float max_time = 10.0f;

    dstar_lite_tick_prepare_full(dst, unit_m, speed_sec, max_time, tk);

    CHECK(dst->unit_m == doctest::Approx(unit_m));
    CHECK(dst->speed_sec == doctest::Approx(speed_sec));
    CHECK(dst->max_time == doctest::Approx(max_time));
    CHECK(dst->cur_elapsed_time == doctest::Approx(0.0f));
    CHECK(dst->base->real_route != nullptr);
    CHECK(route_get_coord_count(dst->base->real_route) == 1);

    // 1초간 tick → 1m 이동되어야 함
    tick_update(tk, 0.3f);
	route_print(dst->base->real_route);
    tick_update(tk, 0.3f);
    route_print(dst->base->real_route);
    tick_update(tk, 0.3f);
    route_print(dst->base->real_route);
    tick_update(tk, 0.3f);
    route_print(dst->base->real_route);
    tick_update(tk, 0.3f);
    route_print(dst->base->real_route);
    tick_update(tk, 0.3f);
    route_print(dst->base->real_route);
    CHECK(route_get_coord_count(dst->base->real_route) >= 2);

    dstar_lite_tick_complete(dst, tk);
    tick_destroy(tk);
    dstar_lite_destroy(dsl);
    navgrid_destroy(ng);
    coord_list_destroy(list);
    dstar_lite_tick_destroy(dst);
}
