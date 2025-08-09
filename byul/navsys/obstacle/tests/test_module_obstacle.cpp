//test_coord.cpp

#include "doctest.h"
#include <locale.h>
#include <iostream>

extern "C" {
#include "obstacle.h"

#include "console.h"
#include "navgrid.h"
}

TEST_CASE("obstacle_make_rect_all_blocked - full blocking") {
    obstacle_t* obs = obstacle_make_rect_all_blocked(10, 20, 5, 5);
    REQUIRE(obs != nullptr);
    CHECK(obstacle_get_width(obs) == 5);
    CHECK(obstacle_get_height(obs) == 5);

    const coord_hash_t* blocked = obstacle_get_blocked_coords(obs);
    CHECK(coord_hash_length(blocked) == 25);
    coord_hash_print(blocked);

    navgrid_t* navgrid = navgrid_create();
    obstacle_apply_to_navgrid(obs, navgrid);
    navgrid_print_ascii(navgrid);
    navgrid_destroy(navgrid);

    obstacle_destroy(obs);

}

TEST_CASE("obstacle_make_rect_random_blocked - ratio = 0.0") {
    obstacle_t* obs = obstacle_make_rect_random_blocked(0, 0, 5, 5, 0.0f);
    REQUIRE(obs == nullptr);
    CHECK(coord_hash_length(obstacle_get_blocked_coords(obs)) == 0);
    obstacle_destroy(obs);
}

TEST_CASE("obstacle_make_rect_random_blocked - ratio = 0.5") {
    obstacle_t* obs = obstacle_make_rect_random_blocked(0, 0, 5, 5, 0.5f);
    REQUIRE(obs != nullptr);

    int blocked = coord_hash_length(obstacle_get_blocked_coords(obs));
    CHECK(blocked >= 3);     // Too low means incorrect random values
    CHECK(blocked <= 22);    // Too high means ratio issue

    navgrid_t* navgrid = navgrid_create();
    obstacle_apply_to_navgrid(obs, navgrid);
    navgrid_print_ascii(navgrid);
    navgrid_destroy(navgrid);    

    obstacle_destroy(obs);
}

TEST_CASE("obstacle_make_rect_random_blocked - ratio = 1.0") {
    obstacle_t* obs = obstacle_make_rect_random_blocked(0, 0, 5, 5, 1.0f);
    REQUIRE(obs != nullptr);
    CHECK(coord_hash_length(obstacle_get_blocked_coords(obs)) == 25);

    navgrid_t* navgrid = navgrid_create();
    obstacle_apply_to_navgrid(obs, navgrid);
    navgrid_print_ascii(navgrid);
    navgrid_destroy(navgrid);

    obstacle_destroy(obs);
}

TEST_CASE("obstacle_make_beam") {
    coord_t start{10, 20};
    coord_t goal{30, 35};
    obstacle_t* obs = obstacle_make_beam(&start, &goal, 0);
    REQUIRE(obs != nullptr);

    const coord_hash_t* blocked = obstacle_get_blocked_coords(obs);
    coord_hash_print(blocked);

    navgrid_t* navgrid = navgrid_create();
    obstacle_apply_to_navgrid(obs, navgrid);
    navgrid_print_ascii(navgrid);
    navgrid_destroy(navgrid);

    obstacle_destroy(obs);
}

TEST_CASE("obstacle_make_beam power up") {
    coord_t start{10, 20};
    coord_t goal{30, 35};
    obstacle_t* obs = obstacle_make_beam(&start, &goal, 1);
    REQUIRE(obs != nullptr);

    const coord_hash_t* blocked = obstacle_get_blocked_coords(obs);
    coord_hash_print(blocked);

    navgrid_t* navgrid = navgrid_create();
    obstacle_apply_to_navgrid(obs, navgrid);
    navgrid_print_ascii(navgrid);
    navgrid_destroy(navgrid);

    obstacle_destroy(obs);
}

TEST_CASE("obstacle_make_torus minimum size") {
    coord_t start{0, 0};
    coord_t goal{6, 6}; // width = 7, height = 7
    int thickness = 2;

    obstacle_t* obs = obstacle_make_torus(&start, &goal, thickness);
    REQUIRE(obs != nullptr);

    const coord_hash_t* blocked = obstacle_get_blocked_coords(obs);
    coord_hash_print(blocked);

    navgrid_t* navgrid = navgrid_create();
    obstacle_apply_to_navgrid(obs, navgrid);
    navgrid_print_ascii(navgrid);
    navgrid_destroy(navgrid);

    obstacle_destroy(obs);
}

TEST_CASE("obstacle_make_torus too small should fail") {
    coord_t start{0, 0};
    coord_t goal{3, 3}; // width = 4, height = 4 < 2*thickness+1
    int thickness = 2;

    obstacle_t* obs = obstacle_make_torus(&start, &goal, thickness);
    REQUIRE(obs == nullptr);
}

TEST_CASE("obstacle_make_enclosure open LEFT") {
    coord_t start{0, 0};
    coord_t goal{6, 6};
    int thickness = 1;

    obstacle_t* obs = obstacle_make_enclosure(
        &start, &goal, thickness, ENCLOSURE_OPEN_LEFT);
    REQUIRE(obs != nullptr);

    const coord_hash_t* blocked = obstacle_get_blocked_coords(obs);
    coord_hash_print(blocked);

    navgrid_t* navgrid = navgrid_create();
    obstacle_apply_to_navgrid(obs, navgrid);
    navgrid_print_ascii(navgrid);
    navgrid_destroy(navgrid);

    obstacle_destroy(obs);
}

TEST_CASE("obstacle_make_enclosure fully closed") {
    coord_t start{0, 0};
    coord_t goal{6, 6};
    int thickness = 1;

    obstacle_t* obs = obstacle_make_enclosure(
        &start, &goal, thickness, ENCLOSURE_OPEN_UNKNOWN);
    REQUIRE(obs != nullptr);

    const coord_hash_t* blocked = obstacle_get_blocked_coords(obs);
    coord_hash_print(blocked);

    navgrid_t* navgrid = navgrid_create();
    obstacle_apply_to_navgrid(obs, navgrid);
    navgrid_print_ascii(navgrid);
    navgrid_destroy(navgrid);

    obstacle_destroy(obs);
}

TEST_CASE("obstacle_make_cross") {
    SUBCASE("center point only (length = 0, range = 0)") {
        coord_t center{10, 10};
        obstacle_t* obs = obstacle_make_cross(&center, 0, 0);
        REQUIRE(obs != nullptr);

        const coord_hash_t* blocked = obstacle_get_blocked_coords(obs);
        coord_hash_print(blocked);

        navgrid_t* navgrid = navgrid_create();
        obstacle_apply_to_navgrid(obs, navgrid);
        navgrid_print_ascii(navgrid);
        navgrid_destroy(navgrid);

        obstacle_destroy(obs);
    }

    SUBCASE("thin cross (length = 2, range = 0)") {
        coord_t center{10, 10};
        obstacle_t* obs = obstacle_make_cross(&center, 2, 0);
        REQUIRE(obs != nullptr);

        const coord_hash_t* blocked = obstacle_get_blocked_coords(obs);
        coord_hash_print(blocked);

        navgrid_t* navgrid = navgrid_create();
        obstacle_apply_to_navgrid(obs, navgrid);
        navgrid_print_ascii(navgrid);
        navgrid_destroy(navgrid);

        obstacle_destroy(obs);
    }

    SUBCASE("thick cross (length = 3, range = 1)") {
        coord_t center{10, 10};
        obstacle_t* obs = obstacle_make_cross(&center, 3, 1);
        REQUIRE(obs != nullptr);

        const coord_hash_t* blocked = obstacle_get_blocked_coords(obs);
        coord_hash_print(blocked);

        navgrid_t* navgrid = navgrid_create();
        obstacle_apply_to_navgrid(obs, navgrid);
        navgrid_print_ascii(navgrid);
        navgrid_destroy(navgrid);

        obstacle_destroy(obs);
    }

    SUBCASE("invalid input (null center)") {
        obstacle_t* obs = obstacle_make_cross(nullptr, 3, 1);
        REQUIRE(obs == nullptr);
    }

    SUBCASE("invalid input (negative range)") {
        coord_t center{10, 10};
        obstacle_t* obs = obstacle_make_cross(&center, 2, -1);
        REQUIRE(obs == nullptr);
    }

    SUBCASE("invalid input (negative length)") {
        coord_t center{10, 10};
        obstacle_t* obs = obstacle_make_cross(&center, -1, 1);
        REQUIRE(obs == nullptr);
    }
}

TEST_CASE("obstacle_make_spiral direction") {
    coord_t center = {20, 20};
    int radius = 5;
    int turns = 8;
    int range = 0;
    int gap = 2;

    SUBCASE("clockwise spiral") {
        obstacle_t* obs = obstacle_make_spiral(
            &center, radius, turns, range, gap, SPIRAL_CLOCKWISE);
        REQUIRE(obs != nullptr);

        const coord_hash_t* blocked = obstacle_get_blocked_coords(obs);
        coord_hash_print(blocked);

        navgrid_t* navgrid = navgrid_create();
        obstacle_apply_to_navgrid(obs, navgrid);
        navgrid_print_ascii(navgrid);
        navgrid_destroy(navgrid);

        obstacle_destroy(obs);
    }

    SUBCASE("counter-clockwise spiral") {
        obstacle_t* obs = obstacle_make_spiral(
            &center, radius, turns, range, gap, SPIRAL_COUNTER_CLOCKWISE);
        REQUIRE(obs != nullptr);

        const coord_hash_t* blocked = obstacle_get_blocked_coords(obs);
        coord_hash_print(blocked);

        navgrid_t* navgrid = navgrid_create();
        obstacle_apply_to_navgrid(obs, navgrid);
        navgrid_print_ascii(navgrid);
        navgrid_destroy(navgrid);

        obstacle_destroy(obs);
    }

    SUBCASE("clockwise with gap and range") {
        obstacle_t* obs = obstacle_make_spiral(
            &center, radius, turns, 0, 2, SPIRAL_CLOCKWISE);
        REQUIRE(obs != nullptr);

        const coord_hash_t* blocked = obstacle_get_blocked_coords(obs);
        coord_hash_print(blocked);

        navgrid_t* navgrid = navgrid_create();
        obstacle_apply_to_navgrid(obs, navgrid);
        navgrid_print_ascii(navgrid);
        navgrid_destroy(navgrid);

        obstacle_destroy(obs);
    }

    SUBCASE("counter-clockwise with gap and range") {
        obstacle_t* obs = obstacle_make_spiral(
            &center, radius, turns, 0, 2, SPIRAL_COUNTER_CLOCKWISE);
        REQUIRE(obs != nullptr);

        const coord_hash_t* blocked = obstacle_get_blocked_coords(obs);
        coord_hash_print(blocked);

        navgrid_t* navgrid = navgrid_create();
        obstacle_apply_to_navgrid(obs, navgrid);
        navgrid_print_ascii(navgrid);
        navgrid_destroy(navgrid);

        obstacle_destroy(obs);
    }
}

TEST_CASE("obstacle_make_triangle") {
    SUBCASE("basic triangle generate.") {
        coord_t a = {10, 10};
        coord_t b = {15, 10};
        coord_t c = {12, 15};

        obstacle_t* obs = obstacle_make_triangle(&a, &b, &c);
        REQUIRE(obs != nullptr);

        const coord_hash_t* blocked = obstacle_get_blocked_coords(obs);
        coord_hash_print(blocked);

        navgrid_t* navgrid = navgrid_create();
        obstacle_apply_to_navgrid(obs, navgrid);
        navgrid_print_ascii(navgrid);
        navgrid_destroy(navgrid);
        obstacle_destroy(obs);
    }

    SUBCASE("reverse triangllle") {
        coord_t a = {12, 10};
        coord_t b = {9, 15};
        coord_t c = {15, 15};

        obstacle_t* obs = obstacle_make_triangle(&a, &b, &c);
        REQUIRE(obs != nullptr);

        const coord_hash_t* blocked = obstacle_get_blocked_coords(obs);
        coord_hash_print(blocked);

        navgrid_t* navgrid = navgrid_create();
        obstacle_apply_to_navgrid(obs, navgrid);
        navgrid_print_ascii(navgrid);
        navgrid_destroy(navgrid);
        obstacle_destroy(obs);
    }

    SUBCASE("up_left diagonal triangle") {
        coord_t a = {10, 10};
        coord_t b = {15, 15};
        coord_t c = {10, 20};

        obstacle_t* obs = obstacle_make_triangle(&a, &b, &c);
        REQUIRE(obs != nullptr);

        const coord_hash_t* blocked = obstacle_get_blocked_coords(obs);
        coord_hash_print(blocked);

        navgrid_t* navgrid = navgrid_create();
        obstacle_apply_to_navgrid(obs, navgrid);
        navgrid_print_ascii(navgrid);
        navgrid_destroy(navgrid);
        obstacle_destroy(obs);
    }

    SUBCASE("input is null that fail") {
        coord_t a = {10, 10};
        obstacle_t* obs = obstacle_make_triangle(&a, nullptr, nullptr);
        CHECK(obs == nullptr);
    }
}

TEST_CASE("obstacle_make_triangle_torus") {
    SUBCASE("basic outline torus, thickness = 0") {
        coord_t a = {10, 10};
        coord_t b = {15, 10};
        coord_t c = {12, 15};

        obstacle_t* obs = obstacle_make_triangle_torus(&a, &b, &c, 0);
        REQUIRE(obs != nullptr);

        const coord_hash_t* blocked = obstacle_get_blocked_coords(obs);
        coord_hash_print(blocked);

        navgrid_t* navgrid = navgrid_create();
        obstacle_apply_to_navgrid(obs, navgrid);
        navgrid_print_ascii(navgrid);
        navgrid_destroy(navgrid);
        obstacle_destroy(obs);
    }

    SUBCASE("fat outline torus, thickness = 1") {
        coord_t a = {10, 10};
        coord_t b = {15, 10};
        coord_t c = {12, 15};

        obstacle_t* obs = obstacle_make_triangle_torus(&a, &b, &c, 1);
        REQUIRE(obs != nullptr);

        const coord_hash_t* blocked = obstacle_get_blocked_coords(obs);
        coord_hash_print(blocked);

        navgrid_t* navgrid = navgrid_create();
        obstacle_apply_to_navgrid(obs, navgrid);
        navgrid_print_ascii(navgrid);
        navgrid_destroy(navgrid);
        obstacle_destroy(obs);
    }

    SUBCASE("more fat torus, thickness = 2") {
        coord_t a = {8, 8};
        coord_t b = {16, 9};
        coord_t c = {12, 16};

        obstacle_t* obs = obstacle_make_triangle_torus(&a, &b, &c, 2);
        REQUIRE(obs != nullptr);

        const coord_hash_t* blocked = obstacle_get_blocked_coords(obs);
        coord_hash_print(blocked);

        navgrid_t* navgrid = navgrid_create();
        obstacle_apply_to_navgrid(obs, navgrid);
        navgrid_print_ascii(navgrid);
        navgrid_destroy(navgrid);
        obstacle_destroy(obs);
    }

    SUBCASE("fault input - thickness < 0") {
        coord_t a = {0, 0};
        coord_t b = {1, 0};
        coord_t c = {0, 1};

        obstacle_t* obs = obstacle_make_triangle_torus(&a, &b, &c, -1);
        CHECK(obs == nullptr);
    }

    SUBCASE("fault input - NULL coord") {
        coord_t a = {0, 0};
        obstacle_t* obs = obstacle_make_triangle_torus(&a, nullptr, nullptr, 1);
        CHECK(obs == nullptr);
    }
}

TEST_CASE("obstacle_make_polygon") {
    SUBCASE("generic pentagon polygon generate") {
        coord_list_t* list = coord_list_create();

        coord_t tmp = {10, 10};
        coord_list_push_back(list, &tmp);

        tmp = {15, 10};        
        coord_list_push_back(list, &tmp);

        tmp = {17, 15};        
        coord_list_push_back(list, &tmp);

        tmp = {12, 18};
        coord_list_push_back(list, &tmp);
        
        tmp = {8, 14};
        coord_list_push_back(list, &tmp);

        REQUIRE(coord_list_length(list) == 5);

        obstacle_t* obs = obstacle_make_polygon(list);
        REQUIRE(obs != nullptr);

        const coord_hash_t* blocked = obstacle_get_blocked_coords(obs);
        coord_hash_print(blocked);

        navgrid_t* navgrid = navgrid_create();
        obstacle_apply_to_navgrid(obs, navgrid);
        navgrid_print_ascii(navgrid);

        navgrid_destroy(navgrid);
        obstacle_destroy(obs);
        coord_list_destroy(list);
    }

    SUBCASE("lack of input (2 point )") {
        coord_list_t* list = coord_list_create();

        coord_t tmp = {0, 0};
        coord_list_push_back(list, &tmp);

        tmp = {1, 1};
        coord_list_push_back(list, &tmp);

        REQUIRE(coord_list_length(list) == 2);

        obstacle_t* obs = obstacle_make_polygon(list);
        CHECK(obs == nullptr);

        coord_list_destroy(list);
    }

    SUBCASE("input is NULL") {
        obstacle_t* obs = obstacle_make_polygon(nullptr);
        CHECK(obs == nullptr);
    }
}

TEST_CASE("obstacle_make_polygon_torus") {
    SUBCASE("basic pentagon torus generate - thickness = 0") {
        coord_list_t* list = coord_list_create();

        coord_t tmp = {10, 10};
        coord_list_push_back(list, &tmp);

        tmp = {15, 10};
        coord_list_push_back(list, &tmp);

        tmp = {17, 15};
        coord_list_push_back(list, &tmp);

        tmp = {12, 18};
        coord_list_push_back(list, &tmp);

        tmp = {8, 14};
        coord_list_push_back(list, &tmp);        

        REQUIRE(coord_list_length(list) == 5);

        obstacle_t* obs = obstacle_make_polygon_torus(list, 0);
        REQUIRE(obs != nullptr);

        const coord_hash_t* blocked = obstacle_get_blocked_coords(obs);
        coord_hash_print(blocked);

        navgrid_t* navgrid = navgrid_create();
        obstacle_apply_to_navgrid(obs, navgrid);
        navgrid_print_ascii(navgrid);

        navgrid_destroy(navgrid);
        obstacle_destroy(obs);
        coord_list_destroy(list);
    }

    SUBCASE("thickness = 1 more fat") {
        coord_list_t* list = coord_list_create();

        coord_t tmp = {5, 5};
        coord_list_push_back(list, &tmp);

        tmp = {10, 5};
        coord_list_push_back(list, &tmp);

        tmp = {12, 10};
        coord_list_push_back(list, &tmp);

        tmp = {7, 13};
        coord_list_push_back(list, &tmp);

        tmp = {3, 9};
        coord_list_push_back(list, &tmp);

        obstacle_t* obs = obstacle_make_polygon_torus(list, 1);
        REQUIRE(obs != nullptr);

        const coord_hash_t* blocked = obstacle_get_blocked_coords(obs);
        coord_hash_print(blocked);

        navgrid_t* navgrid = navgrid_create();
        obstacle_apply_to_navgrid(obs, navgrid);
        navgrid_print_ascii(navgrid);

        navgrid_destroy(navgrid);
        obstacle_destroy(obs);
        coord_list_destroy(list);
    }

    SUBCASE("lack of coord (2 point)") {
        coord_list_t* list = coord_list_create();

        coord_t tmp = {0, 0};
        coord_list_push_back(list, &tmp);

        tmp = {1, 1};
        coord_list_push_back(list, &tmp);

        obstacle_t* obs = obstacle_make_polygon_torus(list, 0);
        CHECK(obs == nullptr);
        coord_list_destroy(list);
    }

    SUBCASE("NULL list") {
        obstacle_t* obs = obstacle_make_polygon_torus(nullptr, 0);
        CHECK(obs == nullptr);
    }

    SUBCASE("minus thickness is fail") {
        coord_list_t* list = coord_list_create();
        
        coord_t tmp = {0, 0};
        coord_list_push_back(list, &tmp);
       
        tmp = {1, 0};
        coord_list_push_back(list, &tmp);
        
        tmp = {1, 1};
        coord_list_push_back(list, &tmp);        

        obstacle_t* obs = obstacle_make_polygon_torus(list, -1);
        CHECK(obs == nullptr);
        coord_list_destroy(list);
    }
}

TEST_CASE("obstacle_block_straight") {
    SUBCASE("range = 0, one line blocking") {
        obstacle_t* obs = obstacle_create_full(10, 10, 20, 20);
        REQUIRE(obs != nullptr);

        obstacle_block_straight(obs, 15, 15, 25, 20, 0);

        const coord_hash_t* blocked = obstacle_get_blocked_coords(obs);
        coord_hash_print(blocked);

        navgrid_t* navgrid = navgrid_create();
        obstacle_apply_to_navgrid(obs, navgrid);
        navgrid_print_ascii(navgrid);

        navgrid_destroy(navgrid);
        obstacle_destroy(obs);
    }

    SUBCASE("range = 1, block adjacent") {
        obstacle_t* obs = obstacle_create_full(0, 0, 30, 30);
        REQUIRE(obs != nullptr);

        obstacle_block_straight(obs, 5, 5, 20, 10, 1);

        const coord_hash_t* blocked = obstacle_get_blocked_coords(obs);
        coord_hash_print(blocked); // 디버깅용

        navgrid_t* navgrid = navgrid_create();
        obstacle_apply_to_navgrid(obs, navgrid);
        navgrid_print_ascii(navgrid); // 두꺼운 직선 확인

        navgrid_destroy(navgrid);
        obstacle_destroy(obs);
    }

    SUBCASE("range = 2, vertical blocking") {
        obstacle_t* obs = obstacle_create_full(0, 0, 30, 30);
        REQUIRE(obs != nullptr);

        obstacle_block_straight(obs, 10, 5, 10, 20, 2);

        const coord_hash_t* blocked = obstacle_get_blocked_coords(obs);
        coord_hash_print(blocked);

        navgrid_t* navgrid = navgrid_create();
        obstacle_apply_to_navgrid(obs, navgrid);
        navgrid_print_ascii(navgrid);

        navgrid_destroy(navgrid);
        obstacle_destroy(obs);
    }

    SUBCASE("range = 0, diagonal blocking") {
        obstacle_t* obs = obstacle_create_full(0, 0, 30, 30);
        REQUIRE(obs != nullptr);

        obstacle_block_straight(obs, 5, 5, 15, 15, 0);

        navgrid_t* navgrid = navgrid_create();
        obstacle_apply_to_navgrid(obs, navgrid);
        navgrid_print_ascii(navgrid);

        navgrid_destroy(navgrid);
        obstacle_destroy(obs);
    }
}