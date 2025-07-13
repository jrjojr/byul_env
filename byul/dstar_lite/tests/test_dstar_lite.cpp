#include <doctest.h>

#include "internal/dstar_lite.h"

#include "internal/dstar_lite_utils.h"

#include "internal/route.h"
#include <stdio.h>
#include <unistd.h>
#include <thread>

TEST_CASE("test_dstar_lite_basic") {
        coord_t* start = coord_new_full(0, 0);
        coord_t* goal = coord_new_full(9, 9);

        // map_t* m = map_new();
        map_t* m = map_new_full(10, 10, MAP_NEIGHBOR_8, is_coord_blocked_map);

        dstar_lite_t* dsl = dstar_lite_new_full(m,start,
            dstar_lite_cost, dstar_lite_heuristic, true);

        dstar_lite_set_start(dsl, start);
        dstar_lite_set_goal(dsl, goal);

        route_t* p = dstar_lite_find(dsl);

        CHECK(route_get_success(p));
        printf("[BASIC] route_t* length = %d\n", route_length(p));

        route_print(p);

        dsl_print_ascii_update_count(dsl, p, 5);
        dsl_print_ascii_route(dsl, p, 5);

        route_free(p);
        coord_free(start);
        coord_free(goal);
        dstar_lite_free(dsl);   
        map_free(m);
}

TEST_CASE("test_dstar_lite_blocked_route") {
    coord_t* start = coord_new_full(0, 0);
    coord_t* goal = coord_new_full(9, 9);

    map_t* m = map_new_full(10, 10, MAP_NEIGHBOR_8, is_coord_blocked_map);

    dstar_lite_t* dsl = dstar_lite_new_full(m, start,
        dstar_lite_cost, dstar_lite_heuristic, true);

        dstar_lite_set_start(dsl, start);
        dstar_lite_set_goal(dsl, goal);

    for (int y = 1; y < 10; y++) map_block_coord(m, 5, y);    

    route_t* p = dstar_lite_find(dsl);

    CHECK(p);
    CHECK(route_get_success(p));

    route_print(p);
    dsl_print_ascii_update_count(dsl, p, 5);

    route_free(p);
    coord_free(start);
    coord_free(goal);
    dstar_lite_free(dsl);
    map_free(m);
     
}

TEST_CASE("test_dstar_lite_blocked_route_refind_ub1") {
    coord_t* start = coord_new_full(0, 0);
    coord_t* goal = coord_new_full(9, 9);

    map_t* m = map_new_full(10, 10, MAP_NEIGHBOR_8, is_coord_blocked_map);
    dstar_lite_t* dsl = dstar_lite_new_full(m, start,
        dstar_lite_cost, dstar_lite_heuristic, true);

                dstar_lite_set_start(dsl, start);
        dstar_lite_set_goal(dsl, goal);

    for (int y = 1; y < 10; y++) map_block_coord(m, 5, y);    

    route_t* p = dstar_lite_find(dsl);

    CHECK(p);
    CHECK(route_get_success(p));

    route_print(p);
    dsl_print_ascii_update_count(dsl, p, 5);

    coord_t* c0 = coord_new_full(5,0);
    coord_t* c1 = coord_new_full(5,1);

    map_block_coord(m, c0->x, c0->y);
    map_unblock_coord(m, c1->x, c1->y);

    dstar_lite_update_vertex_range(dsl, c0, 0);
    dstar_lite_update_vertex_range(dsl, c1, 0);    
    
    // dstar_lite_update_vertex_by_route(dsl, p);

    route_t* p1 = dstar_lite_find(dsl);

    route_print(p);        
    dsl_print_ascii_update_count(dsl, p1, 5);

    coord_free(c0);
    coord_free(c1);

    route_free(p1);
    route_free(p);

    coord_free(start);
    coord_free(goal);
    dstar_lite_free(dsl);
    map_free(m);
     
}

TEST_CASE("test_dstar_lite_blocked_route_refind_ub2") {
    coord_t* start = coord_new_full(0, 0);
    coord_t* goal = coord_new_full(9, 9);

    map_t* m = map_new_full(10, 10, MAP_NEIGHBOR_8, is_coord_blocked_map);
    dstar_lite_t* dsl = dstar_lite_new_full(m, start,
        dstar_lite_cost, dstar_lite_heuristic, true);

        dstar_lite_set_start(dsl, start);
        dstar_lite_set_goal(dsl, goal);

    for (int y = 1; y < 10; y++) map_block_coord(m, 5, y);    

    route_t* p = dstar_lite_find(dsl);

    CHECK(p);
    CHECK(route_get_success(p));

    route_print(p);
    dsl_print_ascii_update_count(dsl, p, 5);

    coord_t* c0 = coord_new_full(5,0);
    coord_t* c2 = coord_new_full(5,2);

    map_block_coord(dsl->m, c0->x, c0->y);
    map_unblock_coord(dsl->m, c2->x, c2->y);

    dstar_lite_update_vertex_range(dsl, c0, 1);
    dstar_lite_update_vertex_range(dsl, c2, 1);    
    
    // 기존 경로 기반 update
    // dstar_lite_update_vertex_by_route(dsl, p);

    route_t* p1 = dstar_lite_find(dsl);

    route_print(p);        
    dsl_print_ascii_update_count(dsl, p1, 5);

    coord_free(c0);
    coord_free(c2);

    route_free(p1);
    route_free(p);

    coord_free(start);
    coord_free(goal);
    dstar_lite_free(dsl);
    map_free(m);
     
}

TEST_CASE("test_dstar_lite_blocked_route_refind_ub3") {
    coord_t* start = coord_new_full(0, 0);
    coord_t* goal = coord_new_full(9, 9);

    map_t* m = map_new_full(10, 10, MAP_NEIGHBOR_8, is_coord_blocked_map);
    dstar_lite_t* dsl = dstar_lite_new_full(m, start,
        dstar_lite_cost, dstar_lite_heuristic, true);

        dstar_lite_set_start(dsl, start);
        dstar_lite_set_goal(dsl, goal);

    for (int y = 1; y < 10; y++) map_block_coord(dsl->m, 5, y);    

    route_t* p = dstar_lite_find(dsl);

    CHECK(p);
    CHECK(route_get_success(p));

    route_print(p);    
    dsl_print_ascii_update_count(dsl, p, 5);

    coord_t* c0 = coord_new_full(5,0);
    coord_t* c3 = coord_new_full(5,3);

    map_block_coord(dsl->m, c0->x, c0->y);
    map_unblock_coord(dsl->m, c3->x, c3->y);

    dstar_lite_update_vertex_range(dsl, c0, 1);
    dstar_lite_update_vertex_range(dsl, c3, 1);    
    
    // 기존 경로 기반 update
    // dstar_lite_update_vertex_by_route(dsl, p);

    route_t* p1 = dstar_lite_find(dsl);

    route_print(p);        
    dsl_print_ascii_update_count(dsl, p1, 5);

    coord_free(c0);
    coord_free(c3);

    route_free(p1);
    route_free(p);

    coord_free(start);
    coord_free(goal);
    dstar_lite_free(dsl);
    map_free(m);
     
}

TEST_CASE("test_dstar_lite_blocked_route_refind_ub4") {
    coord_t* start = coord_new_full(0, 0);
    coord_t* goal = coord_new_full(9, 9);

    map_t* m = map_new_full(10, 10, MAP_NEIGHBOR_8, is_coord_blocked_map);
    dstar_lite_t* dsl = dstar_lite_new_full(m, start,
        dstar_lite_cost, dstar_lite_heuristic, true);

        dstar_lite_set_start(dsl, start);
        dstar_lite_set_goal(dsl, goal);

    for (int y = 1; y < 10; y++) map_block_coord(dsl->m, 5, y);    

    route_t* p = dstar_lite_find(dsl);

    CHECK(p);
    CHECK(route_get_success(p));

    dsl_print_ascii_update_count(dsl, p, 5);

    coord_t* c0 = coord_new_full(5,0);
    coord_t* c4 = coord_new_full(5,4);

    map_block_coord(dsl->m, c0->x, c0->y);
    map_unblock_coord(dsl->m, c4->x, c4->y);

    dstar_lite_update_vertex_range(dsl, c0, 1);
    dstar_lite_update_vertex_range(dsl, c4, 1);    
    
    route_t* p1 = dstar_lite_find(dsl);

    route_print(p);        
    dsl_print_ascii_update_count(dsl, p1, 5);

    coord_free(c0);
    coord_free(c4);

    route_free(p1);
    route_free(p);

    coord_free(start);
    coord_free(goal);
    dstar_lite_free(dsl);
    map_free(m);
     
}

TEST_CASE("test_dstar_lite_blocked_route_refind_ub5") {
    coord_t* start = coord_new_full(0, 0);
    coord_t* goal = coord_new_full(9, 9);

    map_t* m = map_new_full(10, 10, MAP_NEIGHBOR_8, is_coord_blocked_map);
    dstar_lite_t* dsl = dstar_lite_new_full(m, start,
        dstar_lite_cost, dstar_lite_heuristic, true);

        dstar_lite_set_start(dsl, start);
        dstar_lite_set_goal(dsl, goal);

    for (int y = 1; y < 10; y++) map_block_coord(dsl->m, 5, y);    

    route_t* p = dstar_lite_find(dsl);

    CHECK(p);
    CHECK(route_get_success(p));
        
    route_print(p);
    dsl_print_ascii_update_count(dsl, p, 5);

    coord_t* c0 = coord_new_full(5,0);
    coord_t* c5 = coord_new_full(5,5);

    map_block_coord(dsl->m, c0->x, c0->y);
    map_unblock_coord(dsl->m, c5->x, c5->y);

    dstar_lite_update_vertex_range(dsl, c0, 1);
    dstar_lite_update_vertex_range(dsl, c5, 1);
    
    route_t* p1 = dstar_lite_find(dsl);

    route_print(p1);
    dsl_print_ascii_update_count(dsl, p1, 5);

    coord_free(c0);
    coord_free(c5);

    route_free(p1);
    route_free(p);

    coord_free(start);
    coord_free(goal);
    dstar_lite_free(dsl);
    map_free(m);
     
}

TEST_CASE("test_dstar_lite_blocked_route_default") {
    coord_t* start = coord_new_full(5, 5);
    coord_t* goal = coord_new_full(5, 5);

    map_t* m = map_new_full(10, 10, MAP_NEIGHBOR_8, is_coord_blocked_map);
    dstar_lite_t* dsl = dstar_lite_new(m);
    dstar_lite_set_real_loop_max_retry(dsl, 20);

    printf("Running find_route with default constructor\n");
    route_t* p = dstar_lite_find(dsl);
    CHECK(p);
    // g_assert_true(route_get_success(p));
    route_print(p);
    dsl_print_ascii_update_count(dsl, p, 5);
    route_free(p);
    p = NULL;    


    printf("Setting goal to (%d, %d)\n", goal->x, goal->y);
    dstar_lite_reset(dsl);
    dstar_lite_set_goal(dsl, goal);
    p = dstar_lite_find(dsl);
    CHECK(p);
    // g_assert_true(route_get_success(p));
    route_print(p);
    dsl_print_ascii_update_count(dsl, p, 5);
    route_free(p);

    coord_free(start);
    coord_free(goal);
    dstar_lite_free(dsl);
    map_free(m);
}

TEST_CASE("test_dstar_lite_block_unblock_recover") {
    coord_t* start = coord_new_full(0, 0);
    coord_t* goal = coord_new_full(9, 9);

    map_t* m = map_new_full(10, 10, MAP_NEIGHBOR_8, is_coord_blocked_map);
    dstar_lite_t* dsl = dstar_lite_new_full(m, start,
        dstar_lite_cost, dstar_lite_heuristic, true);

        dstar_lite_set_start(dsl, start);
        dstar_lite_set_goal(dsl, goal);

    // 🔹 1. 최초 경로
    route_t* p = dstar_lite_find(dsl);
    CHECK(p);
    CHECK(route_get_success(p));

    route_print(p);
    dsl_print_ascii_update_count(dsl, p, 5);
    route_free(p);
    p = NULL;

    coord_t* c = coord_new_full(4, 4);
    coord_t* c0 = coord_new_full(3, 3);
    coord_t* c1 = coord_new_full(4, 3);

    // 🔹 2. 장애물 추가
    map_block_coord(dsl->m, c->x, c->y);
    map_block_coord(dsl->m, c0->x, c0->y);
    map_block_coord(dsl->m, c1->x, c1->y);

    p = dstar_lite_find(dsl);
    CHECK(p);
    // g_assert_true(route_get_success(p));

    route_print(p);
    dsl_print_ascii_update_count(dsl, p, 5);
    route_free(p);
    p = NULL;

    // 🔹 3. 장애물 제거
    map_unblock_coord(dsl->m, c->x, c->y);
    dstar_lite_update_vertex_range(dsl, c, 1);

    p = dstar_lite_find(dsl);
    CHECK(p);
    // g_assert_true(route_get_success(p));

    route_print(p);
    dsl_print_ascii_update_count(dsl, p, 5);
    route_free(p);
    p = NULL;

    coord_set(goal, 7, 6);
    dstar_lite_set_goal(dsl, goal);

    p = dstar_lite_find(dsl);
    CHECK(p);
    // g_assert_true(route_get_success(p));

    route_print(p);
    dsl_print_ascii_update_count(dsl, p, 5);
    route_free(p);
    p = NULL;

    coord_free(c);
    coord_free(c0);
    coord_free(c1);

    coord_free(start);
    coord_free(goal);
    dstar_lite_free(dsl);
    map_free(m);
}    

TEST_CASE("test_dstar_lite_find_loop") {
    coord_t* start = coord_new_full(0, 0);
    coord_t* goal = coord_new_full(9, 9);

    map_t* m = map_new_full(10, 10, MAP_NEIGHBOR_8, is_coord_blocked_map);
    dstar_lite_t* dsl = dstar_lite_new_full(m, start,
        dstar_lite_cost, dstar_lite_heuristic, true);

        dstar_lite_set_start(dsl, start);
        dstar_lite_set_goal(dsl, goal);

    printf("Generating the first static path using dstar_lite_find()\n");
    route_t* p = dstar_lite_find(dsl);
    CHECK(p);
    CHECK(route_get_success(p));

    route_print(p);
    dsl_print_ascii_update_count(dsl, p, 5);
    route_free(p);
    p = NULL;
    dstar_lite_reset(dsl);

    coord_t* c = coord_new_full(4, 4);
    coord_t* c0 = coord_new_full(3, 3);
    coord_t* c1 = coord_new_full(4, 3);
    coord_t* c2 = coord_new_full(5, 3);


    printf("Generating static path after adding obstacles using dstar_lite_find()\n");

    map_block_coord(dsl->m, c->x, c->y);
    map_block_coord(dsl->m, c0->x, c0->y);
    map_block_coord(dsl->m, c1->x, c1->y);

    p = dstar_lite_find(dsl);
    CHECK(p);
    // g_assert_true(route_get_success(p));

    route_print(p);
    dsl_print_ascii_update_count(dsl, p, 5);
    route_free(p);
    p = NULL;
    dstar_lite_reset(dsl);

    printf("Generating static path after removing obstacles using dstar_lite_find()\n");

    map_unblock_coord(dsl->m, c->x, c->y);
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
    // route_free(p);
    // p = NULL;


    // coord_list_t* route_list = NULL;
coord_list_t* changed_coords = NULL;
int interval_msec = 100;

dstar_lite_set_interval_msec(dsl, interval_msec);

dstar_lite_find_loop(dsl);

for (int i = 0; i < 5; i++) {
    printf("interval msec : %d, dstar_lite_find_loop() cretes dynamic routes.\n", interval_msec);

    coord_t* coord_i = coord_new_full(i + 4, 5);
    printf("blocked (%d, %d)\n", coord_get_x(coord_i), coord_get_y(coord_i));


    map_block_coord(dsl->m, coord_get_x(coord_i), coord_get_y(coord_i));

    if (i == 2) {
        coord_list_push_back(changed_coords, coord_i);
        dsl->changed_coords_fn = get_changed_coords;
        dsl->changed_coords_fn_userdata = changed_coords;
    }

    CHECK(dsl->real_route);

    route_print(p);
    dsl_print_ascii_update_count(dsl, p, 5);

    coord_free(coord_i);  // 원본 해제
}

// 정리
coord_list_free((coord_list_t*)dsl->changed_coords_fn_userdata);
// g_list_free_full(route_list, (GDestroyNotify)route_free);



    coord_free(c);
    coord_free(c0);
    coord_free(c1);
    coord_free(c2);

    coord_free(start);
    coord_free(goal);
    dstar_lite_free(dsl);
    map_free(m);
}    

TEST_CASE("test_dstar_lite_find_static") {
    coord_t* start = coord_new_full(0, 0);
    coord_t* goal = coord_new_full(9, 9);

    map_t* m = map_new_full(10, 10, MAP_NEIGHBOR_8, is_coord_blocked_map);
    dstar_lite_t* dsl = dstar_lite_new_full(m, start,
        dstar_lite_cost, dstar_lite_heuristic, true);

        dstar_lite_set_start(dsl, start);
        dstar_lite_set_goal(dsl, goal);

printf("Generating the initial static path with dstar_lite_find()\n");

    route_t* p = dstar_lite_find(dsl);
    CHECK(p);
    CHECK(route_get_success(p));
    route_print(p);
    dsl_print_ascii_update_count(dsl, p, 5);
    route_free(p);
    dstar_lite_reset(dsl);

    // 🔹 장애물 추가 후 경로 재계산
    coord_t* c = coord_new_full(4, 4);
    coord_t* c0 = coord_new_full(3, 3);
    coord_t* c1 = coord_new_full(4, 3);
    coord_t* c2 = coord_new_full(5, 3);

printf("Generating path after adding obstacles using dstar_lite_find()\n");

    map_block_coord(dsl->m, c->x, c->y);
    map_block_coord(dsl->m, c0->x, c0->y);
    map_block_coord(dsl->m, c1->x, c1->y);

    p = dstar_lite_find(dsl);
    CHECK(p);
    route_print(p);
    dsl_print_ascii_update_count(dsl, p, 5);
    route_free(p);
    dstar_lite_reset(dsl);

printf("Generating path after removing obstacles using dstar_lite_find_proto()\n");

    map_unblock_coord(dsl->m, c->x, c->y);
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

    // 정리
    coord_free(c); coord_free(c0); coord_free(c1); coord_free(c2);
    coord_free(start); coord_free(goal);
    dstar_lite_free(dsl);
    map_free(m);
}

static void* run_find_loop(void* data) {
    dstar_lite_t* dsl = (dstar_lite_t*)data;
    dstar_lite_find_loop(dsl);  // 무한 루프
    return NULL;
}

TEST_CASE("test_dstar_lite_find_dynamic") {
    coord_t* start = coord_new_full(0, 0);
    coord_t* goal = coord_new_full(7, 6);

    map_t* m = map_new_full(10, 10, MAP_NEIGHBOR_8, is_coord_blocked_map);
    dstar_lite_t* dsl = dstar_lite_new_full(m, start,
        dstar_lite_cost, dstar_lite_heuristic, true);

        dstar_lite_set_start(dsl, start);
        dstar_lite_set_goal(dsl, goal);

    // 초기 경로
    dstar_lite_find_proto(dsl);
    CHECK(dsl->proto_route);
    route_print(dsl->proto_route);
    dsl_print_ascii_update_count(dsl, dsl->proto_route, 5);

    // 설정
    int interval_msec = 100;
    dstar_lite_set_interval_msec(dsl, interval_msec);

    dsl->move_fn = move_to;
    dsl->changed_coords_fn = get_changed_coords;

    // 🔹 별도 쓰레드에서 loop 시작

    std::thread loop_thread([&]() {
        run_find_loop((void*)dsl);
    });    


    coord_t* coord_i = NULL;
    coord_list_t* changed_coords = NULL;
    for (int i = 0; i < 50; i++) {

    std::this_thread::sleep_for(std::chrono::milliseconds(interval_msec * 30));        


printf("%dms passed — checking for dynamic changes\n", i * interval_msec);

        if (i == 2) {
            coord_i = coord_new_full(i + 1, i);
            printf("blocked (%d, %d)\n", coord_get_x(coord_i), coord_get_y(coord_i));

            map_block_coord(dsl->m, coord_get_x(coord_i), coord_get_y(coord_i));

            if (changed_coords != NULL) {
                coord_list_free(changed_coords);
                // g_list_free(changed_coords);
                changed_coords = NULL;
            }
            coord_list_push_back(changed_coords, coord_copy(coord_i));
            dsl->changed_coords_fn_userdata = changed_coords;

            coord_free(coord_i);
            coord_i = NULL;
        }

        // 실시간 경로 출력
        if (dsl->real_route) {
            route_print(dsl->real_route);
            dsl_print_ascii_update_count(dsl, dsl->real_route, 5);
        }
        if (dsl->real_route->success) {
            printf("Pathfinding successful\n");

            break;
        }
    }

        // 종료 대기
    loop_thread.join();

    dsl_print_ascii_only_map(dsl);

    route_print(dsl->real_route);
    dsl_print_ascii_update_count(dsl, dsl->real_route, 5);

    coord_list_free(changed_coords);

    coord_free(start); 
    coord_free(goal);
    dstar_lite_free(dsl);
    map_free(m);

    coord_free(coord_i);    
}

TEST_CASE("test_dstar_lite_block_all_around_start") {
    coord_t* start = coord_new_full(0, 0);
    coord_t* goal = coord_new_full(-9, -9);

    map_t* m = map_new_full(0, 0, MAP_NEIGHBOR_8, is_coord_blocked_map);
    dstar_lite_t* dsl = dstar_lite_new_full(m, start,
        dstar_lite_cost, dstar_lite_heuristic, true);

        dstar_lite_set_start(dsl, start);
        dstar_lite_set_goal(dsl, goal);

    map_block_coord(dsl->m, 1, 0);
    map_block_coord(dsl->m, 1, -1);
    map_block_coord(dsl->m, 0, -1);
    map_block_coord(dsl->m, -1, -1);

    route_t* p = dstar_lite_find(dsl);
    CHECK(p);
    CHECK(route_get_success(p));
    route_print(p);
    dsl_print_ascii_update_count(dsl, p, 5);
    route_free(p);
    p = NULL;

        
    map_block_coord(dsl->m, -1, 0);
    map_block_coord(dsl->m, -1, 1);
    map_block_coord(dsl->m, 0, 1);

    dstar_lite_set_compute_max_retry(dsl, 200);

    p = dstar_lite_find(dsl);
    CHECK(p);
    CHECK(route_get_success(p));
    route_print(p);
    dsl_print_ascii_update_count(dsl, p, 5);
    route_free(p);
    p = NULL;    

    map_block_coord(dsl->m, 1, 1);    

    p = dstar_lite_find(dsl);
    CHECK(p);
    CHECK(!route_get_success(p));
    route_print(p);
    dsl_print_ascii_update_count(dsl, p, 5);
    route_free(p);
    p = NULL;        

    route_free(p);
    coord_free(start);
    coord_free(goal);
    dstar_lite_free(dsl);
    map_free(m);
     
}

TEST_CASE("test_dstar_lite_find_proto") {
    printf("test_dstar_lite_find_proto\n.");
    coord_t* start = coord_new_full(0, 0);
    coord_t* goal = coord_new_full(9, 9);

    // map_t* m = map_new_full(0, 0, MAP_NEIGHBOR_8);
    map_t* m = map_new();
    dstar_lite_t* dsl = dstar_lite_new_full(m, start,
        dstar_lite_cost, dstar_lite_heuristic, true);

        dstar_lite_set_start(dsl, start);
        dstar_lite_set_goal(dsl, goal);

    for (int y = 1; y < 10; y++) {
        map_block_coord(dsl->m, 5, y);    
    }

    dstar_lite_find_proto(dsl);
    CHECK(dsl->proto_route);
    CHECK(route_get_success(dsl->proto_route));
    route_print(dsl->proto_route);
    dsl_print_ascii_update_count(dsl, dsl->proto_route, 5);

    coord_free(start);
    coord_free(goal);
    dstar_lite_free(dsl);
    map_free(m);
}

TEST_CASE("test_dstar_lite_find_proto_reverse") {
    printf("test_dstar_lite_find_proto_reverse\n.");
    coord_t* start = coord_new_full(9, 9);
    coord_t* goal = coord_new_full(0, 0);

    // map_t* m = map_new_full(0, 0, MAP_NEIGHBOR_8);
    map_t* m = map_new();
    dstar_lite_t* dsl = dstar_lite_new_full(m, start,
        dstar_lite_cost, dstar_lite_heuristic, true);

        dstar_lite_set_start(dsl, start);
        dstar_lite_set_goal(dsl, goal);

    for (int y = 1; y < 10; y++) {
        map_block_coord(dsl->m, 5, y);    
    }

    dstar_lite_find_proto(dsl);
    CHECK(dsl->proto_route);
    CHECK(route_get_success(dsl->proto_route));
    route_print(dsl->proto_route);
    dsl_print_ascii_update_count(dsl, dsl->proto_route, 5);

    coord_free(start);
    coord_free(goal);
    dstar_lite_free(dsl);
    map_free(m);
}

TEST_CASE("test_dstar_lite_find_proto_minus_start") {
    printf("test_dstar_lite_find_proto_minus_start\n.");

    coord_t* start = coord_new_full(-9, -9);
    coord_t* goal = coord_new_full(0, 0);

    // map_t* m = map_new_full(0, 0, MAP_NEIGHBOR_8);
    map_t* m = map_new();
    dstar_lite_t* dsl = dstar_lite_new_full(m, start,
        dstar_lite_cost, dstar_lite_heuristic, true);

        dstar_lite_set_start(dsl, start);
        dstar_lite_set_goal(dsl, goal);

    for (int y = 1; y < 10; y++) {
        map_block_coord(dsl->m, 5, y);    
    }

    dstar_lite_find_proto(dsl);
    CHECK(dsl->proto_route);
    CHECK(route_get_success(dsl->proto_route));
    route_print(dsl->proto_route);
    dsl_print_ascii_update_count(dsl, dsl->proto_route, 5);

    coord_free(start);
    coord_free(goal);
    dstar_lite_free(dsl);
    map_free(m);
}

TEST_CASE("test_dstar_lite_find_proto_minus_goal") {
    printf("test_dstar_lite_find_proto_minus_goal\n.");
    coord_t* start = coord_new_full(0, 0);
    coord_t* goal = coord_new_full(-9, -9);

    // map_t* m = map_new_full(0, 0, MAP_NEIGHBOR_8);
    map_t* m = map_new();
    dstar_lite_t* dsl = dstar_lite_new_full(m, start,
        dstar_lite_cost, dstar_lite_heuristic, true);

        dstar_lite_set_start(dsl, start);
        dstar_lite_set_goal(dsl, goal);

    for (int y = 1; y < 10; y++) {
        map_block_coord(dsl->m, 5, y);    
    }

    dstar_lite_find_proto(dsl);
    CHECK(dsl->proto_route);
    CHECK(route_get_success(dsl->proto_route));
    route_print(dsl->proto_route);
    dsl_print_ascii_update_count(dsl, dsl->proto_route, 5);

    coord_free(start);
    coord_free(goal);
    dstar_lite_free(dsl);
    map_free(m);
}

TEST_CASE("test_dstar_lite_find_proto_plus_start_minus_goal") {
    printf("test_dstar_lite_find_proto_plus_start_minus_goal\n.");
    coord_t* start = coord_new_full(7, 7);
    coord_t* goal = coord_new_full(-3, -3);

    // map_t* m = map_new_full(0, 0, MAP_NEIGHBOR_8);
    map_t* m = map_new();
    dstar_lite_t* dsl = dstar_lite_new_full(m, start,
        dstar_lite_cost, dstar_lite_heuristic, true);

        dstar_lite_set_start(dsl, start);
        dstar_lite_set_goal(dsl, goal);

    for (int y = 1; y < 10; y++) {
        map_block_coord(dsl->m, 5, y);    
    }

    dstar_lite_find_proto(dsl);
    CHECK(dsl->proto_route);
    CHECK(route_get_success(dsl->proto_route));
    route_print(dsl->proto_route);
    dsl_print_ascii_update_count(dsl, dsl->proto_route, 5);

    coord_free(start);
    coord_free(goal);
    dstar_lite_free(dsl);
    map_free(m);
}

TEST_CASE("test_dstar_lite_find_proto_minus_start_plus_goal") {
    printf("test_dstar_lite_find_proto_minus_start_plus_goal\n.");
    coord_t* start = coord_new_full(-3, -3);
    coord_t* goal = coord_new_full(7, 7);

    // map_t* m = map_new_full(30, 30, MAP_NEIGHBOR_8);
    map_t* m = map_new();
    dstar_lite_t* dsl = dstar_lite_new_full(m, start,
        dstar_lite_cost, dstar_lite_heuristic, true);

        dstar_lite_set_start(dsl, start);
        dstar_lite_set_goal(dsl, goal);

    for (int y = 1; y < 10; y++) {
        map_block_coord(dsl->m, 5, y);    
    }

    dstar_lite_find_proto(dsl);
    CHECK(dsl->proto_route);
    CHECK(route_get_success(dsl->proto_route));
    route_print(dsl->proto_route);
    dsl_print_ascii_update_count(dsl, dsl->proto_route, 5);

    coord_t* goal1 = coord_new_full(3, 3);
    dstar_lite_set_goal(dsl, goal1);
    dstar_lite_find_proto(dsl);
    CHECK(dsl->proto_route);
    CHECK(route_get_success(dsl->proto_route));
    route_print(dsl->proto_route);
    dsl_print_ascii_update_count(dsl, dsl->proto_route, 5);    

    coord_free(start);
    coord_free(goal);
    coord_free(goal1);
    dstar_lite_free(dsl);
    map_free(m);
}
