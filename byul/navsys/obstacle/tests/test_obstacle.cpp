//test_coord.cpp

#define DOCTEST_CONFIG_IMPLEMENT
#include "doctest.h"
#include <locale.h>
#include <iostream>

extern "C" {
#include "internal/obstacle.h"

#include "internal/console.h"
#include "internal/navgrid.h"
}

TEST_CASE("obstacle_make_rect_all_blocked - full blocking") {
    obstacle_t* obs = obstacle_make_rect_all_blocked(10, 20, 5, 5);
    REQUIRE(obs != nullptr);
    CHECK(obstacle_get_width(obs) == 5);
    CHECK(obstacle_get_height(obs) == 5);

    const coord_hash_t* blocked = obstacle_get_blocked_coords(obs);
    CHECK(coord_hash_length(blocked) == 25);
    coord_hash_print(blocked);

    navgrid_t* navgrid = navgrid_new();
    obstacle_apply_to_navgrid(obs, navgrid);
    navgrid_print_ascii(navgrid);
    navgrid_free(navgrid);

    obstacle_free(obs);

}

TEST_CASE("obstacle_make_rect_random_blocked - ratio = 0.0") {
    obstacle_t* obs = obstacle_make_rect_random_blocked(0, 0, 5, 5, 0.0f);
    REQUIRE(obs == nullptr);
    CHECK(coord_hash_length(obstacle_get_blocked_coords(obs)) == 0);
    obstacle_free(obs);
}

TEST_CASE("obstacle_make_rect_random_blocked - ratio = 0.5") {
    obstacle_t* obs = obstacle_make_rect_random_blocked(0, 0, 5, 5, 0.5f);
    REQUIRE(obs != nullptr);

    int blocked = coord_hash_length(obstacle_get_blocked_coords(obs));
    CHECK(blocked >= 3);     // 너무 낮으면 잘못된 난수
    CHECK(blocked <= 22);    // 너무 높으면 ratio 문제

    navgrid_t* navgrid = navgrid_new();
    obstacle_apply_to_navgrid(obs, navgrid);
    navgrid_print_ascii(navgrid);
    navgrid_free(navgrid);    

    obstacle_free(obs);
}

TEST_CASE("obstacle_make_rect_random_blocked - ratio = 1.0") {
    obstacle_t* obs = obstacle_make_rect_random_blocked(0, 0, 5, 5, 1.0f);
    REQUIRE(obs != nullptr);
    CHECK(coord_hash_length(obstacle_get_blocked_coords(obs)) == 25);

    navgrid_t* navgrid = navgrid_new();
    obstacle_apply_to_navgrid(obs, navgrid);
    navgrid_print_ascii(navgrid);
    navgrid_free(navgrid);

    obstacle_free(obs);
}

TEST_CASE("obstacle_make_beam") {
    coord_t start{10, 20};
    coord_t goal{30, 35};
    obstacle_t* obs = obstacle_make_beam(&start, &goal, 0);
    REQUIRE(obs != nullptr);

    const coord_hash_t* blocked = obstacle_get_blocked_coords(obs);
    coord_hash_print(blocked);

    navgrid_t* navgrid = navgrid_new();
    obstacle_apply_to_navgrid(obs, navgrid);
    navgrid_print_ascii(navgrid);
    navgrid_free(navgrid);

    obstacle_free(obs);
}

TEST_CASE("obstacle_make_beam power up") {
    coord_t start{10, 20};
    coord_t goal{30, 35};
    obstacle_t* obs = obstacle_make_beam(&start, &goal, 1);
    REQUIRE(obs != nullptr);

    const coord_hash_t* blocked = obstacle_get_blocked_coords(obs);
    coord_hash_print(blocked);

    navgrid_t* navgrid = navgrid_new();
    obstacle_apply_to_navgrid(obs, navgrid);
    navgrid_print_ascii(navgrid);
    navgrid_free(navgrid);

    obstacle_free(obs);
}

TEST_CASE("obstacle_make_torus minimum size") {
    coord_t start{0, 0};
    coord_t goal{6, 6}; // width = 7, height = 7
    int thickness = 2;

    obstacle_t* obs = obstacle_make_torus(&start, &goal, thickness);
    REQUIRE(obs != nullptr);

    const coord_hash_t* blocked = obstacle_get_blocked_coords(obs);
    coord_hash_print(blocked);

    navgrid_t* navgrid = navgrid_new();
    obstacle_apply_to_navgrid(obs, navgrid);
    navgrid_print_ascii(navgrid);
    navgrid_free(navgrid);

    obstacle_free(obs);
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

    obstacle_t* obs = obstacle_make_enclosure(&start, &goal, thickness, ENCLOSURE_OPEN_LEFT);
    REQUIRE(obs != nullptr);

    const coord_hash_t* blocked = obstacle_get_blocked_coords(obs);
    coord_hash_print(blocked);

    navgrid_t* navgrid = navgrid_new();
    obstacle_apply_to_navgrid(obs, navgrid);
    navgrid_print_ascii(navgrid);
    navgrid_free(navgrid);

    obstacle_free(obs);
}

TEST_CASE("obstacle_make_enclosure fully closed") {
    coord_t start{0, 0};
    coord_t goal{6, 6};
    int thickness = 1;

    obstacle_t* obs = obstacle_make_enclosure(&start, &goal, thickness, ENCLOSURE_OPEN_UNKNOWN);
    REQUIRE(obs != nullptr);

    const coord_hash_t* blocked = obstacle_get_blocked_coords(obs);
    coord_hash_print(blocked);

    navgrid_t* navgrid = navgrid_new();
    obstacle_apply_to_navgrid(obs, navgrid);
    navgrid_print_ascii(navgrid);
    navgrid_free(navgrid);

    obstacle_free(obs);
}

TEST_CASE("obstacle_make_cross") {
    SUBCASE("center point only (length = 0, range = 0)") {
        coord_t center{10, 10};
        obstacle_t* obs = obstacle_make_cross(&center, 0, 0);
        REQUIRE(obs != nullptr);

        const coord_hash_t* blocked = obstacle_get_blocked_coords(obs);
        coord_hash_print(blocked);

        navgrid_t* navgrid = navgrid_new();
        obstacle_apply_to_navgrid(obs, navgrid);
        navgrid_print_ascii(navgrid);
        navgrid_free(navgrid);

        obstacle_free(obs);
    }

    SUBCASE("thin cross (length = 2, range = 0)") {
        coord_t center{10, 10};
        obstacle_t* obs = obstacle_make_cross(&center, 2, 0);
        REQUIRE(obs != nullptr);

        const coord_hash_t* blocked = obstacle_get_blocked_coords(obs);
        coord_hash_print(blocked);

        navgrid_t* navgrid = navgrid_new();
        obstacle_apply_to_navgrid(obs, navgrid);
        navgrid_print_ascii(navgrid);
        navgrid_free(navgrid);

        obstacle_free(obs);
    }

    SUBCASE("thick cross (length = 3, range = 1)") {
        coord_t center{10, 10};
        obstacle_t* obs = obstacle_make_cross(&center, 3, 1);
        REQUIRE(obs != nullptr);

        const coord_hash_t* blocked = obstacle_get_blocked_coords(obs);
        coord_hash_print(blocked);

        navgrid_t* navgrid = navgrid_new();
        obstacle_apply_to_navgrid(obs, navgrid);
        navgrid_print_ascii(navgrid);
        navgrid_free(navgrid);

        obstacle_free(obs);
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

        navgrid_t* navgrid = navgrid_new();
        obstacle_apply_to_navgrid(obs, navgrid);
        navgrid_print_ascii(navgrid);
        navgrid_free(navgrid);

        obstacle_free(obs);
    }

    SUBCASE("counter-clockwise spiral") {
        obstacle_t* obs = obstacle_make_spiral(
            &center, radius, turns, range, gap, SPIRAL_COUNTER_CLOCKWISE);
        REQUIRE(obs != nullptr);

        const coord_hash_t* blocked = obstacle_get_blocked_coords(obs);
        coord_hash_print(blocked);

        navgrid_t* navgrid = navgrid_new();
        obstacle_apply_to_navgrid(obs, navgrid);
        navgrid_print_ascii(navgrid);
        navgrid_free(navgrid);

        obstacle_free(obs);
    }

    SUBCASE("clockwise with gap and range") {
        obstacle_t* obs = obstacle_make_spiral(
            &center, radius, turns, 0, 2, SPIRAL_CLOCKWISE);
        REQUIRE(obs != nullptr);

        const coord_hash_t* blocked = obstacle_get_blocked_coords(obs);
        coord_hash_print(blocked);

        navgrid_t* navgrid = navgrid_new();
        obstacle_apply_to_navgrid(obs, navgrid);
        navgrid_print_ascii(navgrid);
        navgrid_free(navgrid);

        obstacle_free(obs);
    }

    SUBCASE("counter-clockwise with gap and range") {
        obstacle_t* obs = obstacle_make_spiral(
            &center, radius, turns, 0, 2, SPIRAL_COUNTER_CLOCKWISE);
        REQUIRE(obs != nullptr);

        const coord_hash_t* blocked = obstacle_get_blocked_coords(obs);
        coord_hash_print(blocked);

        navgrid_t* navgrid = navgrid_new();
        obstacle_apply_to_navgrid(obs, navgrid);
        navgrid_print_ascii(navgrid);
        navgrid_free(navgrid);

        obstacle_free(obs);
    }
}

TEST_CASE("obstacle_make_triangle") {
    SUBCASE("기본 삼각형 생성") {
        coord_t a = {10, 10};
        coord_t b = {15, 10};
        coord_t c = {12, 15};

        obstacle_t* obs = obstacle_make_triangle(&a, &b, &c);
        REQUIRE(obs != nullptr);

        const coord_hash_t* blocked = obstacle_get_blocked_coords(obs);
        coord_hash_print(blocked);

        navgrid_t* navgrid = navgrid_new();
        obstacle_apply_to_navgrid(obs, navgrid);
        navgrid_print_ascii(navgrid);
        navgrid_free(navgrid);
        obstacle_free(obs);
    }

    SUBCASE("역삼각형 (아래쪽 뾰족)") {
        coord_t a = {12, 10};
        coord_t b = {9, 15};
        coord_t c = {15, 15};

        obstacle_t* obs = obstacle_make_triangle(&a, &b, &c);
        REQUIRE(obs != nullptr);

        const coord_hash_t* blocked = obstacle_get_blocked_coords(obs);
        coord_hash_print(blocked);

        navgrid_t* navgrid = navgrid_new();
        obstacle_apply_to_navgrid(obs, navgrid);
        navgrid_print_ascii(navgrid);
        navgrid_free(navgrid);
        obstacle_free(obs);
    }

    SUBCASE("좌상향 대각 삼각형") {
        coord_t a = {10, 10};
        coord_t b = {15, 15};
        coord_t c = {10, 20};

        obstacle_t* obs = obstacle_make_triangle(&a, &b, &c);
        REQUIRE(obs != nullptr);

        const coord_hash_t* blocked = obstacle_get_blocked_coords(obs);
        coord_hash_print(blocked);

        navgrid_t* navgrid = navgrid_new();
        obstacle_apply_to_navgrid(obs, navgrid);
        navgrid_print_ascii(navgrid);
        navgrid_free(navgrid);
        obstacle_free(obs);
    }

    SUBCASE("입력이 NULL이면 실패") {
        coord_t a = {10, 10};
        obstacle_t* obs = obstacle_make_triangle(&a, nullptr, nullptr);
        CHECK(obs == nullptr);
    }
}

TEST_CASE("obstacle_make_triangle_torus") {
    SUBCASE("기본 외곽선 torus, thickness = 0") {
        coord_t a = {10, 10};
        coord_t b = {15, 10};
        coord_t c = {12, 15};

        obstacle_t* obs = obstacle_make_triangle_torus(&a, &b, &c, 0);
        REQUIRE(obs != nullptr);

        const coord_hash_t* blocked = obstacle_get_blocked_coords(obs);
        coord_hash_print(blocked);

        navgrid_t* navgrid = navgrid_new();
        obstacle_apply_to_navgrid(obs, navgrid);
        navgrid_print_ascii(navgrid);
        navgrid_free(navgrid);
        obstacle_free(obs);
    }

    SUBCASE("두께 있는 외곽선 torus, thickness = 1") {
        coord_t a = {10, 10};
        coord_t b = {15, 10};
        coord_t c = {12, 15};

        obstacle_t* obs = obstacle_make_triangle_torus(&a, &b, &c, 1);
        REQUIRE(obs != nullptr);

        const coord_hash_t* blocked = obstacle_get_blocked_coords(obs);
        coord_hash_print(blocked);

        navgrid_t* navgrid = navgrid_new();
        obstacle_apply_to_navgrid(obs, navgrid);
        navgrid_print_ascii(navgrid);
        navgrid_free(navgrid);
        obstacle_free(obs);
    }

    SUBCASE("두께가 더 두꺼운 torus, thickness = 2") {
        coord_t a = {8, 8};
        coord_t b = {16, 9};
        coord_t c = {12, 16};

        obstacle_t* obs = obstacle_make_triangle_torus(&a, &b, &c, 2);
        REQUIRE(obs != nullptr);

        const coord_hash_t* blocked = obstacle_get_blocked_coords(obs);
        coord_hash_print(blocked);

        navgrid_t* navgrid = navgrid_new();
        obstacle_apply_to_navgrid(obs, navgrid);
        navgrid_print_ascii(navgrid);
        navgrid_free(navgrid);
        obstacle_free(obs);
    }

    SUBCASE("잘못된 입력 - thickness < 0") {
        coord_t a = {0, 0};
        coord_t b = {1, 0};
        coord_t c = {0, 1};

        obstacle_t* obs = obstacle_make_triangle_torus(&a, &b, &c, -1);
        CHECK(obs == nullptr);
    }

    SUBCASE("잘못된 입력 - NULL 좌표") {
        coord_t a = {0, 0};
        obstacle_t* obs = obstacle_make_triangle_torus(&a, nullptr, nullptr, 1);
        CHECK(obs == nullptr);
    }
}

TEST_CASE("obstacle_make_polygon") {
    SUBCASE("정상적인 오각형 polygon 생성") {
        coord_list_t* list = coord_list_new();
        coord_list_push_back(list, make_tmp_coord(10, 10));
        coord_list_push_back(list, make_tmp_coord(15, 10));
        coord_list_push_back(list, make_tmp_coord(17, 15));
        coord_list_push_back(list, make_tmp_coord(12, 18));
        coord_list_push_back(list, make_tmp_coord(8, 14));

        REQUIRE(coord_list_length(list) == 5);

        obstacle_t* obs = obstacle_make_polygon(list);
        REQUIRE(obs != nullptr);

        const coord_hash_t* blocked = obstacle_get_blocked_coords(obs);
        coord_hash_print(blocked);  // 디버깅용

        navgrid_t* navgrid = navgrid_new();
        obstacle_apply_to_navgrid(obs, navgrid);
        navgrid_print_ascii(navgrid);  // 시각 확인

        navgrid_free(navgrid);
        obstacle_free(obs);
        coord_list_free(list);
    }

    SUBCASE("입력이 부족한 경우 (2점)") {
        coord_list_t* list = coord_list_new();
        coord_list_push_back(list, make_tmp_coord(0, 0));
        coord_list_push_back(list, make_tmp_coord(1, 1));

        REQUIRE(coord_list_length(list) == 2);

        obstacle_t* obs = obstacle_make_polygon(list);
        CHECK(obs == nullptr);

        coord_list_free(list);
    }

    SUBCASE("입력이 NULL인 경우") {
        obstacle_t* obs = obstacle_make_polygon(nullptr);
        CHECK(obs == nullptr);
    }
}

TEST_CASE("obstacle_make_polygon_torus") {
    SUBCASE("기본적인 오각형 torus 생성 - thickness = 0") {
        coord_list_t* list = coord_list_new();
        coord_list_push_back(list, make_tmp_coord(10, 10));
        coord_list_push_back(list, make_tmp_coord(15, 10));
        coord_list_push_back(list, make_tmp_coord(17, 15));
        coord_list_push_back(list, make_tmp_coord(12, 18));
        coord_list_push_back(list, make_tmp_coord(8, 14));

        REQUIRE(coord_list_length(list) == 5);

        obstacle_t* obs = obstacle_make_polygon_torus(list, 0);
        REQUIRE(obs != nullptr);

        const coord_hash_t* blocked = obstacle_get_blocked_coords(obs);
        coord_hash_print(blocked);

        navgrid_t* navgrid = navgrid_new();
        obstacle_apply_to_navgrid(obs, navgrid);
        navgrid_print_ascii(navgrid);  // 외곽선만 그려졌는지 육안 확인

        navgrid_free(navgrid);
        obstacle_free(obs);
        coord_list_free(list);
    }

    SUBCASE("thickness = 1 로 선을 두껍게") {
        coord_list_t* list = coord_list_new();
        coord_list_push_back(list, make_tmp_coord(5, 5));
        coord_list_push_back(list, make_tmp_coord(10, 5));
        coord_list_push_back(list, make_tmp_coord(12, 10));
        coord_list_push_back(list, make_tmp_coord(7, 13));
        coord_list_push_back(list, make_tmp_coord(3, 9));

        obstacle_t* obs = obstacle_make_polygon_torus(list, 1);
        REQUIRE(obs != nullptr);

        const coord_hash_t* blocked = obstacle_get_blocked_coords(obs);
        coord_hash_print(blocked);

        navgrid_t* navgrid = navgrid_new();
        obstacle_apply_to_navgrid(obs, navgrid);
        navgrid_print_ascii(navgrid);

        navgrid_free(navgrid);
        obstacle_free(obs);
        coord_list_free(list);
    }

    SUBCASE("좌표 부족 시 실패 (2점)") {
        coord_list_t* list = coord_list_new();
        coord_list_push_back(list, make_tmp_coord(0, 0));
        coord_list_push_back(list, make_tmp_coord(1, 1));

        obstacle_t* obs = obstacle_make_polygon_torus(list, 0);
        CHECK(obs == nullptr);
        coord_list_free(list);
    }

    SUBCASE("NULL 리스트") {
        obstacle_t* obs = obstacle_make_polygon_torus(nullptr, 0);
        CHECK(obs == nullptr);
    }

    SUBCASE("음수 thickness는 실패") {
        coord_list_t* list = coord_list_new();
        coord_list_push_back(list, make_tmp_coord(0, 0));
        coord_list_push_back(list, make_tmp_coord(1, 0));
        coord_list_push_back(list, make_tmp_coord(1, 1));

        obstacle_t* obs = obstacle_make_polygon_torus(list, -1);
        CHECK(obs == nullptr);
        coord_list_free(list);
    }
}

int main(int argc, char** argv) {
#ifdef _WIN32
    UINT original_cp = GetConsoleOutputCP();
    SetConsoleOutputCP(65001);                          // UTF-8 출력용
    setlocale(LC_ALL, "ko_KR.UTF-8");                   // UTF-8 로케일
#else
    setlocale(LC_ALL, "ko_KR.UTF-8");                   // 리눅스/맥에서도 설정
#endif

    std::cout << u8"🌟 UTF-8 콘솔 코드페이지로 전환하고 테스트 시작!\n";

    doctest::Context context;
    context.applyCommandLine(argc, argv);
    int res = context.run();

    if (context.shouldExit()) {
        std::cout << u8"🌙 테스트 끝! 콘솔 코드페이지 원래대로 복구했습니다.\n";
#ifdef _WIN32
        SetConsoleOutputCP(original_cp);                // 원래 코드페이지 복원
        setlocale(LC_ALL, "");                          // 기본 로케일로 복귀
#endif
        return res;
    }

    std::cout << u8"🌙 테스트 종료. 콘솔 상태 복원 완료.\n";
#ifdef _WIN32
    SetConsoleOutputCP(original_cp);
    setlocale(LC_ALL, "");                              // 로케일 복원
#endif

    return res;
}


TEST_CASE("obstacle_block_straight") {
    SUBCASE("range = 0, 단일 선 블로킹") {
        obstacle_t* obs = obstacle_new_full(10, 10, 20, 20);
        REQUIRE(obs != nullptr);

        obstacle_block_straight(obs, 15, 15, 25, 20, 0);

        const coord_hash_t* blocked = obstacle_get_blocked_coords(obs);
        coord_hash_print(blocked); // 디버깅용

        navgrid_t* navgrid = navgrid_new();
        obstacle_apply_to_navgrid(obs, navgrid);
        navgrid_print_ascii(navgrid); // 시각 확인

        navgrid_free(navgrid);
        obstacle_free(obs);
    }

    SUBCASE("range = 1, 선 주변까지 블로킹") {
        obstacle_t* obs = obstacle_new_full(0, 0, 30, 30);
        REQUIRE(obs != nullptr);

        obstacle_block_straight(obs, 5, 5, 20, 10, 1);

        const coord_hash_t* blocked = obstacle_get_blocked_coords(obs);
        coord_hash_print(blocked); // 디버깅용

        navgrid_t* navgrid = navgrid_new();
        obstacle_apply_to_navgrid(obs, navgrid);
        navgrid_print_ascii(navgrid); // 두꺼운 직선 확인

        navgrid_free(navgrid);
        obstacle_free(obs);
    }

    SUBCASE("range = 2, 수직 블로킹") {
        obstacle_t* obs = obstacle_new_full(0, 0, 30, 30);
        REQUIRE(obs != nullptr);

        obstacle_block_straight(obs, 10, 5, 10, 20, 2);

        const coord_hash_t* blocked = obstacle_get_blocked_coords(obs);
        coord_hash_print(blocked);

        navgrid_t* navgrid = navgrid_new();
        obstacle_apply_to_navgrid(obs, navgrid);
        navgrid_print_ascii(navgrid);

        navgrid_free(navgrid);
        obstacle_free(obs);
    }

    SUBCASE("range = 0, 대각선 블로킹") {
        obstacle_t* obs = obstacle_new_full(0, 0, 30, 30);
        REQUIRE(obs != nullptr);

        obstacle_block_straight(obs, 5, 5, 15, 15, 0);

        navgrid_t* navgrid = navgrid_new();
        obstacle_apply_to_navgrid(obs, navgrid);
        navgrid_print_ascii(navgrid);

        navgrid_free(navgrid);
        obstacle_free(obs);
    }
}