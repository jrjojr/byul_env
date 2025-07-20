#define DOCTEST_CONFIG_IMPLEMENT
#include "doctest.h"
#include "internal/route_carver.h"
#include "internal/coord.h"
#include <locale.h>

#include "internal/navgrid.h"
#include "internal/obstacle.h"
#include "internal/console.h"

TEST_CASE("route_carve_beam - straight line carve") {
    navgrid_t* navgrid = navgrid_create();
    obstacle_t* obs = obstacle_make_rect_all_blocked(0, 0, 10, 10);
    obstacle_apply_to_navgrid(obs, navgrid);

    coord_t start = { 1, 1 };
    coord_t goal = { 8, 8 };

    int removed = route_carve_beam(navgrid, &start, &goal, 0);
    CHECK(removed >= 1);

    navgrid_print_ascii(navgrid);

    obstacle_destroy(obs);
    navgrid_destroy(navgrid);
}

TEST_CASE("route_carve_beam - wide beam carve") {
    navgrid_t* navgrid = navgrid_create();
    obstacle_t* obs = obstacle_make_rect_all_blocked(0, 0, 10, 10);
    obstacle_apply_to_navgrid(obs, navgrid);

    coord_t start = { 2, 2 };
    coord_t goal = { 7, 7 };

    int removed = route_carve_beam(navgrid, &start, &goal, 1);  // 넓은 beam
    CHECK(removed >= 20);  // 충분한 영역 제거 예상

navgrid_print_ascii(navgrid);

    obstacle_destroy(obs);
    navgrid_destroy(navgrid);
}

TEST_CASE("route_carve_bomb - center explosion") {
    navgrid_t* navgrid = navgrid_create();
    obstacle_t* obs = obstacle_make_rect_all_blocked(0, 0, 10, 10);
    obstacle_apply_to_navgrid(obs, navgrid);

    coord_t center = { 5, 5 };
    int removed = route_carve_bomb(navgrid, &center, 2);  // 5x5 제거

    CHECK(removed == 25);  // 정확한 폭파 크기
navgrid_print_ascii(navgrid);

    obstacle_destroy(obs);
    navgrid_destroy(navgrid);
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
