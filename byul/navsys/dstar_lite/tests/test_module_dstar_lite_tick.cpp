#include <doctest.h>

#include "dstar_lite_tick.h"

#include "dstar_lite_console.h"

#include <iostream>

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
    CHECK(coord_list_length(dst->base->real_route->coords) == 1);

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
    CHECK(dst->base->real_route->success);

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
    CHECK(!dst->base->real_route->success);

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

    CHECK(dst->base->real_route->success);
    
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
    CHECK(coord_list_length(dst->base->real_route->coords) == 1);

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
    CHECK(coord_list_length(dst->base->real_route->coords) >= 2);

    dstar_lite_tick_complete(dst, tk);
    tick_destroy(tk);
    dstar_lite_destroy(dsl);
    navgrid_destroy(ng);
    coord_list_destroy(list);
    dstar_lite_tick_destroy(dst);
}
