//test_coord.cpp

#define DOCTEST_CONFIG_IMPLEMENT
#include "doctest.h"
#include <locale.h>
#include <iostream>

extern "C" {
#include "internal/maze.h"
#include "internal/console.h"
}

TEST_CASE("미로 생성 및 맵 적용") {
    maze_t* maze = maze_new_full(5, 5, 9, 9);
    CHECK(maze != nullptr);

    maze_make_recursive(maze);

    map_t* map = map_new_full(19, 19, MAP_NEIGHBOR_4, NULL);
    CHECK(map != nullptr);

    maze_apply_to_map(maze, map);

    const coord_hash_t* blocked = maze_get_blocked_coords(maze);
    CHECK(blocked != nullptr);
    CHECK(coord_hash_length(blocked) > 0);

    map_print_ascii(map);

    maze_free(maze);
    map_free(map);
}

TEST_CASE("Prim 미로 생성 테스트") {
    int x0 = 0, y0 = 0, width = 9, height = 9;
    maze_t* maze = maze_new_full(x0, y0, width, height);
    CHECK(maze != nullptr);

    maze_make_prim(maze);

    map_t* map = map_new_full(width, height, MAP_NEIGHBOR_4, nullptr);
    maze_apply_to_map(maze, map);

    // 미로 블럭이 너무 적거나 너무 많은지 체크
    const coord_hash_t* blocked = maze_get_blocked_coords(maze);
    int n_blocked = coord_hash_length(blocked);
    CHECK(n_blocked > (width * height / 3)); // 최소한 일부 벽은 있어야 함
    CHECK(n_blocked < (width * height));     // 전부 막혀 있으면 안됨

    map_print_ascii(map);

    map_free(map);
    maze_free(maze);
}

TEST_CASE("Binary Tree Maze Generation") {
    maze_t* maze = maze_new_full(0, 0, 9, 9);
    REQUIRE(maze != nullptr);

    maze_make_binary(maze);

    map_t* map = map_new();
    maze_apply_to_map(maze, map);

    map_print_ascii(map);

    map_free(map);
    maze_free(maze);
}

TEST_CASE("Eller's Algorithm Maze Generation") {
    maze_t* maze = maze_new_full(0, 0, 9, 9);
    REQUIRE(maze != nullptr);

    maze_make_eller(maze);

    map_t* map = map_new_full(9, 9, MAP_NEIGHBOR_4, NULL);
    REQUIRE(map != nullptr);

    maze_apply_to_map(maze, map);

    map_print_ascii(map);

    maze_free(maze);
    map_free(map);
}

TEST_CASE("Aldous-Broder Maze Generation") {
    int x0 = 0, y0 = 0, width = 9, height = 9;
    maze_t* maze = maze_new_full(x0, y0, width, height);
    REQUIRE(maze != nullptr);

    maze_make_aldous_broder(maze);

    map_t* map = map_new_full(width, height, MAP_NEIGHBOR_4, NULL);
    REQUIRE(map != nullptr);

    maze_apply_to_map(maze, map);

    const coord_hash_t* blocked = maze_get_blocked_coords(maze);
    int n_blocked = coord_hash_length(blocked);

    CHECK(n_blocked > (width * height / 3)); // 벽이 너무 적지 않음
    CHECK(n_blocked < (width * height));     // 전부 막혀 있지 않음

    map_print_ascii(map);

    map_free(map);
    maze_free(maze);
}

TEST_CASE("Wilson's Algorithm Maze Generation") {
    int x0 = 0, y0 = 0, width = 9, height = 9;
    maze_t* maze = maze_new_full(x0, y0, width, height);
    REQUIRE(maze != nullptr);

    maze_make_wilson(maze);

    map_t* map = map_new_full(width, height, MAP_NEIGHBOR_4, NULL);
    REQUIRE(map != nullptr);

    maze_apply_to_map(maze, map);

    const coord_hash_t* blocked = maze_get_blocked_coords(maze);
    int n_blocked = coord_hash_length(blocked);

    CHECK(n_blocked > (width * height / 3)); // 벽이 너무 적지 않음
    CHECK(n_blocked < (width * height));     // 전부 막혀 있지 않음

    map_print_ascii(map);

    map_free(map);
    maze_free(maze);
}

TEST_CASE("Hunt-and-Kill Maze Generation") {
    int x0 = 0, y0 = 0, width = 9, height = 9;
    maze_t* maze = maze_new_full(x0, y0, width, height);
    REQUIRE(maze != nullptr);

    maze_make_hunt_and_kill(maze);

    map_t* map = map_new_full(width, height, MAP_NEIGHBOR_4, NULL);
    REQUIRE(map != nullptr);

    maze_apply_to_map(maze, map);

    const coord_hash_t* blocked = maze_get_blocked_coords(maze);
    int n_blocked = coord_hash_length(blocked);

    CHECK(n_blocked > (width * height / 3)); // 벽이 너무 적지 않아야 함
    CHECK(n_blocked < (width * height));     // 전체가 벽으로만 되어있으면 안됨

    map_print_ascii(map);

    map_free(map);
    maze_free(maze);
}

TEST_CASE("Sidewinder Maze Generation") {
    int x0 = 0, y0 = 0, width = 9, height = 9;
    maze_t* maze = maze_new_full(x0, y0, width, height);
    REQUIRE(maze != nullptr);

    maze_make_sidewinder(maze);

    map_t* map = map_new_full(width, height, MAP_NEIGHBOR_4, NULL);
    REQUIRE(map != nullptr);

    maze_apply_to_map(maze, map);

    const coord_hash_t* blocked = maze_get_blocked_coords(maze);
    int n_blocked = coord_hash_length(blocked);

    CHECK(n_blocked > (width * height / 3)); // 벽이 너무 적지 않아야 함
    CHECK(n_blocked < (width * height));     // 전부 벽이면 안 됨

    map_print_ascii(map);

    map_free(map);
    maze_free(maze);
}

TEST_CASE("Recursive Division Maze Generation") {
    int x0 = 0, y0 = 0, width = 9, height = 9;
    maze_t* maze = maze_new_full(x0, y0, width, height);
    REQUIRE(maze != nullptr);

    maze_make_recursive_division(maze);

    map_t* map = map_new_full(width, height, MAP_NEIGHBOR_4, NULL);
    REQUIRE(map != nullptr);

    maze_apply_to_map(maze, map);

    const coord_hash_t* blocked = maze_get_blocked_coords(maze);
    int n_blocked = coord_hash_length(blocked);

    CHECK(n_blocked > (width * height / 3)); // 벽이 너무 적지 않음
    CHECK(n_blocked < (width * height));     // 전체가 벽이면 안 됨

    map_print_ascii(map);

    map_free(map);
    maze_free(maze);
}

TEST_CASE("Kruskal's Algorithm Maze Generation") {
    int width = 9, height = 9;

    maze_t* maze = maze_new_full(0, 0, width, height);
    REQUIRE(maze != nullptr);

    maze_make_kruskal(maze);

    // 맵 변환
    map_t* map = map_new_full(width, height, MAP_NEIGHBOR_4, nullptr);
    REQUIRE(map != nullptr);

    maze_apply_to_map(maze, map);

    // 벽 개수 확인 (너무 적거나 많으면 안 됨)
    const coord_hash_t* blocked = maze_get_blocked_coords(maze);
    int n_blocked = coord_hash_length(blocked);
    CHECK(n_blocked > (width * height / 4));
    CHECK(n_blocked < (width * height));

    // 출력 확인
    map_print_ascii(map);

    // 메모리 해제
    map_free(map);
    maze_free(maze);
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
