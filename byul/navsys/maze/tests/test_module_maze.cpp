#include "doctest.h"
#include <locale.h>
#include <iostream>

extern "C" {
#include "maze.h"
#include "console.h"

}

TEST_CASE("maze_make: MAZE_TYPE_ALDOUS_BRODER") {
    maze_t* maze = nullptr;

    maze = maze_make(0, 0, 19, 19, MAZE_TYPE_ALDOUS_BRODER);
    CHECK(maze != nullptr);

    navgrid_t* navgrid = navgrid_create();
    CHECK(navgrid != nullptr);

    maze_apply_to_navgrid(maze, navgrid);

    std::cout << "maze_make: MAZE_TYPE_ALDOUS_BRODER.\n";
    navgrid_print_ascii(navgrid);

    maze_destroy(maze);
    navgrid_destroy(navgrid);
}

TEST_CASE("maze_make: MAZE_TYPE_BINARY") {
    maze_t* maze = nullptr;

    maze = maze_make(0, 0, 19, 19, MAZE_TYPE_BINARY);
    CHECK(maze != nullptr);

    navgrid_t* navgrid = navgrid_create();
    CHECK(navgrid != nullptr);

    maze_apply_to_navgrid(maze, navgrid);

    std::cout << "maze_make: MAZE_TYPE_BINARY.\n";
    navgrid_print_ascii(navgrid);

    maze_destroy(maze);
    navgrid_destroy(navgrid);
}

TEST_CASE("maze_make: MAZE_TYPE_ELLER") {
    maze_t* maze = nullptr;

    maze = maze_make(0, 0, 19, 19, MAZE_TYPE_ELLER);
    CHECK(maze != nullptr);

    navgrid_t* navgrid = navgrid_create();
    CHECK(navgrid != nullptr);

    maze_apply_to_navgrid(maze, navgrid);

    std::cout << "maze_make: MAZE_TYPE_ELLER.\n";
    navgrid_print_ascii(navgrid);

    maze_destroy(maze);
    navgrid_destroy(navgrid);
}

TEST_CASE("maze_make: MAZE_TYPE_HUNT_AND_KILL") {
    maze_t* maze = nullptr;

    maze = maze_make(0, 0, 19, 19, MAZE_TYPE_HUNT_AND_KILL);
    CHECK(maze != nullptr);

    navgrid_t* navgrid = navgrid_create();
    CHECK(navgrid != nullptr);

    maze_apply_to_navgrid(maze, navgrid);

    std::cout << "maze_make: MAZE_TYPE_HUNT_AND_KILL.\n";
    navgrid_print_ascii(navgrid);

    maze_destroy(maze);
    navgrid_destroy(navgrid);
}

TEST_CASE("maze_make: MAZE_TYPE_KRUSKAL") {
    maze_t* maze = nullptr;

    maze = maze_make(0, 0, 19, 19, MAZE_TYPE_KRUSKAL);
    CHECK(maze != nullptr);

    navgrid_t* navgrid = navgrid_create();
    CHECK(navgrid != nullptr);

    maze_apply_to_navgrid(maze, navgrid);

    std::cout << "maze_make: MAZE_TYPE_KRUSKAL.\n";
    navgrid_print_ascii(navgrid);

    maze_destroy(maze);
    navgrid_destroy(navgrid);
}

TEST_CASE("maze_make: MAZE_TYPE_PRIM") {
    maze_t* maze = nullptr;

    maze = maze_make(0, 0, 19, 19, MAZE_TYPE_PRIM);
    CHECK(maze != nullptr);

    navgrid_t* navgrid = navgrid_create();
    CHECK(navgrid != nullptr);

    maze_apply_to_navgrid(maze, navgrid);

    std::cout << "maze_make: MAZE_TYPE_PRIM.\n";
    navgrid_print_ascii(navgrid);

    maze_destroy(maze);
    navgrid_destroy(navgrid);
}

TEST_CASE("maze_make: MAZE_TYPE_RECURSIVE") {
    maze_t* maze = nullptr;

    maze = maze_make(0, 0, 19, 19, MAZE_TYPE_RECURSIVE);
    CHECK(maze != nullptr);

    navgrid_t* navgrid = navgrid_create();
    CHECK(navgrid != nullptr);

    maze_apply_to_navgrid(maze, navgrid);

    std::cout << "maze_make: MAZE_TYPE_RECURSIVE.\n";
    navgrid_print_ascii(navgrid);

    maze_destroy(maze);
    navgrid_destroy(navgrid);
}

TEST_CASE("maze_make: MAZE_TYPE_RECURSIVE_DIVISION") {
    maze_t* maze = nullptr;

    maze = maze_make(0, 0, 19, 19, MAZE_TYPE_RECURSIVE_DIVISION);
    CHECK(maze != nullptr);

    navgrid_t* navgrid = navgrid_create();
    CHECK(navgrid != nullptr);

    maze_apply_to_navgrid(maze, navgrid);

    std::cout << "maze_make: MAZE_TYPE_RECURSIVE_DIVISION.\n";
    navgrid_print_ascii(navgrid);

    maze_destroy(maze);
    navgrid_destroy(navgrid);
}

TEST_CASE("maze_make: MAZE_TYPE_ROOM_BLEND") {
    maze_t* maze = nullptr;

    maze = maze_make(0, 0, 19, 19, MAZE_TYPE_ROOM_BLEND);
    CHECK(maze != nullptr);

    navgrid_t* navgrid = navgrid_create();
    CHECK(navgrid != nullptr);

    maze_apply_to_navgrid(maze, navgrid);

    std::cout << "maze_make: MAZE_TYPE_ROOM_BLEND.\n";
    navgrid_print_ascii(navgrid);

    maze_destroy(maze);
    navgrid_destroy(navgrid);
}

TEST_CASE("maze_make: MAZE_TYPE_SIDEWINDER") {
    maze_t* maze = nullptr;

    maze = maze_make(0, 0, 19, 19, MAZE_TYPE_SIDEWINDER);
    CHECK(maze != nullptr);

    navgrid_t* navgrid = navgrid_create();
    CHECK(navgrid != nullptr);

    maze_apply_to_navgrid(maze, navgrid);

    std::cout << "maze_make: MAZE_TYPE_SIDEWINDER.\n";
    navgrid_print_ascii(navgrid);

    maze_destroy(maze);
    navgrid_destroy(navgrid);
}

TEST_CASE("maze_make: MAZE_TYPE_WILSON") {
    maze_t* maze = nullptr;

    maze = maze_make(0, 0, 19, 19, MAZE_TYPE_WILSON);
    CHECK(maze != nullptr);

    navgrid_t* navgrid = navgrid_create();
    CHECK(navgrid != nullptr);

    maze_apply_to_navgrid(maze, navgrid);

    std::cout << "maze_make: MAZE_TYPE_WILSON.\n";
    navgrid_print_ascii(navgrid);

    maze_destroy(maze);
    navgrid_destroy(navgrid);
}

TEST_CASE("maze overlays preserve overlapping sources") {
    maze_t* first = maze_create_full(0, 0, 8, 8);
    maze_t* second = maze_create_full(0, 0, 8, 8);
    navgrid_t* grid = navgrid_create_full(8, 8, NAVGRID_DIR_4, nullptr);
    REQUIRE(first != nullptr);
    REQUIRE(second != nullptr);
    REQUIRE(grid != nullptr);

    const coord_t blocked{2, 5};
    REQUIRE(coord_hash_upsert_copy(first->blocked, &blocked, nullptr, nullptr)
        == NAVSYS_STATUS_OK);
    REQUIRE(coord_hash_upsert_copy(second->blocked, &blocked, nullptr, nullptr)
        == NAVSYS_STATUS_OK);

    maze_apply_to_navgrid(first, grid);
    maze_apply_to_navgrid(second, grid);
    CHECK(is_coord_blocked_navgrid(grid, 2, 5, nullptr));
    maze_remove_from_navgrid(first, grid);
    CHECK(is_coord_blocked_navgrid(grid, 2, 5, nullptr));
    maze_remove_from_navgrid(second, grid);
    CHECK_FALSE(is_coord_blocked_navgrid(grid, 2, 5, nullptr));

    navgrid_destroy(grid);
    maze_destroy(second);
    maze_destroy(first);
}
