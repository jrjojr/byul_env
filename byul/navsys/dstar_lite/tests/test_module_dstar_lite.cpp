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

    coord_t coord_i;
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
        if (dsl->real_route && dsl->real_route->success) {
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
 