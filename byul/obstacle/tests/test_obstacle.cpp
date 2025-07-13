//test_coord.cpp

#define DOCTEST_CONFIG_IMPLEMENT
#include "doctest.h"
#include <locale.h>
#include <iostream>

extern "C" {
#include "internal/obstacle.h"
#include "internal/obstacle_utils.h"
#include "internal/console.h"
#include "internal/map.h"
}

TEST_CASE("make_rect_all_blocked - full blocking") {
    obstacle_t* obs = make_rect_all_blocked(10, 20, 5, 5);
    REQUIRE(obs != nullptr);
    CHECK(obstacle_get_width(obs) == 5);
    CHECK(obstacle_get_height(obs) == 5);

    const coord_hash_t* blocked = obstacle_get_blocked_coords(obs);
    CHECK(coord_hash_length(blocked) == 25);
    coord_hash_print(blocked);

    map_t* map = map_new();
    obstacle_apply_to_map(obs, map);
    map_print_ascii(map);
    map_free(map);

    obstacle_free(obs);

}

TEST_CASE("make_rect_random_blocked - ratio = 0.0") {
    obstacle_t* obs = make_rect_random_blocked(0, 0, 5, 5, 0.0f);
    REQUIRE(obs == nullptr);
    CHECK(coord_hash_length(obstacle_get_blocked_coords(obs)) == 0);
    obstacle_free(obs);
}

TEST_CASE("make_rect_random_blocked - ratio = 0.5") {
    obstacle_t* obs = make_rect_random_blocked(0, 0, 5, 5, 0.5f);
    REQUIRE(obs != nullptr);

    int blocked = coord_hash_length(obstacle_get_blocked_coords(obs));
    CHECK(blocked >= 3);     // 너무 낮으면 잘못된 난수
    CHECK(blocked <= 22);    // 너무 높으면 ratio 문제

    map_t* map = map_new();
    obstacle_apply_to_map(obs, map);
    map_print_ascii(map);
    map_free(map);    

    obstacle_free(obs);
}

TEST_CASE("make_rect_random_blocked - ratio = 1.0") {
    obstacle_t* obs = make_rect_random_blocked(0, 0, 5, 5, 1.0f);
    REQUIRE(obs != nullptr);
    CHECK(coord_hash_length(obstacle_get_blocked_coords(obs)) == 25);

    map_t* map = map_new();
    obstacle_apply_to_map(obs, map);
    map_print_ascii(map);
    map_free(map);

    obstacle_free(obs);
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
