//test_coord.cpp

#include "doctest.h"
#include <locale.h>
#include <iostream>

extern "C" {
#include "internal/maze.h"
#include "internal/console.h"
}

TEST_CASE("미로 생성 및 맵 적용") {
    maze_t* maze = maze_create_full(5, 5, 9, 9);
    CHECK(maze != nullptr);

    maze_make_recursive(maze);

    navgrid_t* navgrid = navgrid_create_full(19, 19, NAVGRID_DIR_4, NULL);
    CHECK(navgrid != nullptr);

    maze_apply_to_navgrid(maze, navgrid);

    const coord_hash_t* blocked = maze_get_blocked_coords(maze);
    CHECK(blocked != nullptr);
    CHECK(coord_hash_length(blocked) > 0);

    navgrid_print_ascii(navgrid);

    maze_destroy(maze);
    navgrid_destroy(navgrid);
}

TEST_CASE("Prim 미로 생성 테스트") {
    int x0 = 0, y0 = 0, width = 9, height = 9;
    maze_t* maze = maze_create_full(x0, y0, width, height);
    CHECK(maze != nullptr);

    maze_make_prim(maze);

    navgrid_t* navgrid = navgrid_create_full(width, height, NAVGRID_DIR_4, nullptr);
    maze_apply_to_navgrid(maze, navgrid);

    // 미로 블럭이 너무 적거나 너무 많은지 체크
    const coord_hash_t* blocked = maze_get_blocked_coords(maze);
    int n_blocked = coord_hash_length(blocked);
    CHECK(n_blocked > (width * height / 3)); // 최소한 일부 벽은 있어야 함
    CHECK(n_blocked < (width * height));     // 전부 막혀 있으면 안됨

    navgrid_print_ascii(navgrid);

    navgrid_destroy(navgrid);
    maze_destroy(maze);
}

TEST_CASE("Binary Tree Maze Generation") {
    maze_t* maze = maze_create_full(0, 0, 9, 9);
    REQUIRE(maze != nullptr);

    maze_make_binary(maze);

    navgrid_t* navgrid = navgrid_create();
    maze_apply_to_navgrid(maze, navgrid);

    navgrid_print_ascii(navgrid);

    navgrid_destroy(navgrid);
    maze_destroy(maze);
}

TEST_CASE("Eller's Algorithm Maze Generation") {
    maze_t* maze = maze_create_full(0, 0, 9, 9);
    REQUIRE(maze != nullptr);

    maze_make_eller(maze);

    navgrid_t* navgrid = navgrid_create_full(9, 9, NAVGRID_DIR_4, NULL);
    REQUIRE(navgrid != nullptr);

    maze_apply_to_navgrid(maze, navgrid);

    navgrid_print_ascii(navgrid);

    maze_destroy(maze);
    navgrid_destroy(navgrid);
}

TEST_CASE("Aldous-Broder Maze Generation") {
    int x0 = 0, y0 = 0, width = 9, height = 9;
    maze_t* maze = maze_create_full(x0, y0, width, height);
    REQUIRE(maze != nullptr);

    maze_make_aldous_broder(maze);

    navgrid_t* navgrid = navgrid_create_full(width, height, NAVGRID_DIR_4, NULL);
    REQUIRE(navgrid != nullptr);

    maze_apply_to_navgrid(maze, navgrid);

    const coord_hash_t* blocked = maze_get_blocked_coords(maze);
    int n_blocked = coord_hash_length(blocked);

    CHECK(n_blocked > (width * height / 3)); // 벽이 너무 적지 않음
    CHECK(n_blocked < (width * height));     // 전부 막혀 있지 않음

    navgrid_print_ascii(navgrid);

    navgrid_destroy(navgrid);
    maze_destroy(maze);
}

TEST_CASE("Wilson's Algorithm Maze Generation") {
    int x0 = 0, y0 = 0, width = 9, height = 9;
    maze_t* maze = maze_create_full(x0, y0, width, height);
    REQUIRE(maze != nullptr);

    maze_make_wilson(maze);

    navgrid_t* navgrid = navgrid_create_full(width, height, NAVGRID_DIR_4, NULL);
    REQUIRE(navgrid != nullptr);

    maze_apply_to_navgrid(maze, navgrid);

    const coord_hash_t* blocked = maze_get_blocked_coords(maze);
    int n_blocked = coord_hash_length(blocked);

    CHECK(n_blocked > (width * height / 3)); // 벽이 너무 적지 않음
    CHECK(n_blocked < (width * height));     // 전부 막혀 있지 않음

    navgrid_print_ascii(navgrid);

    navgrid_destroy(navgrid);
    maze_destroy(maze);
}

TEST_CASE("Hunt-and-Kill Maze Generation") {
    int x0 = 0, y0 = 0, width = 9, height = 9;
    maze_t* maze = maze_create_full(x0, y0, width, height);
    REQUIRE(maze != nullptr);

    maze_make_hunt_and_kill(maze);

    navgrid_t* navgrid = navgrid_create_full(width, height, NAVGRID_DIR_4, NULL);
    REQUIRE(navgrid != nullptr);

    maze_apply_to_navgrid(maze, navgrid);

    const coord_hash_t* blocked = maze_get_blocked_coords(maze);
    int n_blocked = coord_hash_length(blocked);

    CHECK(n_blocked > (width * height / 3)); // 벽이 너무 적지 않아야 함
    CHECK(n_blocked < (width * height));     // 전체가 벽으로만 되어있으면 안됨

    navgrid_print_ascii(navgrid);

    navgrid_destroy(navgrid);
    maze_destroy(maze);
}

TEST_CASE("Sidewinder Maze Generation") {
    int x0 = 0, y0 = 0, width = 9, height = 9;
    maze_t* maze = maze_create_full(x0, y0, width, height);
    REQUIRE(maze != nullptr);

    maze_make_sidewinder(maze);

    navgrid_t* navgrid = navgrid_create_full(width, height, NAVGRID_DIR_4, NULL);
    REQUIRE(navgrid != nullptr);

    maze_apply_to_navgrid(maze, navgrid);

    const coord_hash_t* blocked = maze_get_blocked_coords(maze);
    int n_blocked = coord_hash_length(blocked);

    CHECK(n_blocked > (width * height / 3)); // 벽이 너무 적지 않아야 함
    CHECK(n_blocked < (width * height));     // 전부 벽이면 안 됨

    navgrid_print_ascii(navgrid);

    navgrid_destroy(navgrid);
    maze_destroy(maze);
}

TEST_CASE("Recursive Division Maze Generation") {
    int x0 = 0, y0 = 0, width = 9, height = 9;
    maze_t* maze = maze_create_full(x0, y0, width, height);
    REQUIRE(maze != nullptr);

    maze_make_recursive_division(maze);

    navgrid_t* navgrid = navgrid_create_full(width, height, NAVGRID_DIR_4, NULL);
    REQUIRE(navgrid != nullptr);

    maze_apply_to_navgrid(maze, navgrid);

    const coord_hash_t* blocked = maze_get_blocked_coords(maze);
    int n_blocked = coord_hash_length(blocked);

    CHECK(n_blocked > (width * height / 3)); // 벽이 너무 적지 않음
    CHECK(n_blocked < (width * height));     // 전체가 벽이면 안 됨

    navgrid_print_ascii(navgrid);

    navgrid_destroy(navgrid);
    maze_destroy(maze);
}

TEST_CASE("Kruskal's Algorithm Maze Generation") {
    int width = 9, height = 9;

    maze_t* maze = maze_create_full(0, 0, width, height);
    REQUIRE(maze != nullptr);

    maze_make_kruskal(maze);

    // 맵 변환
    navgrid_t* navgrid = navgrid_create_full(width, height, NAVGRID_DIR_4, nullptr);
    REQUIRE(navgrid != nullptr);

    maze_apply_to_navgrid(maze, navgrid);

    // 벽 개수 확인 (너무 적거나 많으면 안 됨)
    const coord_hash_t* blocked = maze_get_blocked_coords(maze);
    int n_blocked = coord_hash_length(blocked);
    CHECK(n_blocked > (width * height / 4));
    CHECK(n_blocked < (width * height));

    // 출력 확인
    navgrid_print_ascii(navgrid);

    // 메모리 해제
    navgrid_destroy(navgrid);
    maze_destroy(maze);
}
