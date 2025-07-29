// test_coord.cpp

#include "doctest.h"
#include <locale.h>
#include <iostream>

extern "C" {
#include "maze.h"
#include "console.h"
}

TEST_CASE("Maze generation and map application") {
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

TEST_CASE("Prim Maze Generation Test") {
    int x0 = 0, y0 = 0, width = 9, height = 9;
    maze_t* maze = maze_create_full(x0, y0, width, height);
    CHECK(maze != nullptr);

    maze_make_prim(maze);

    navgrid_t* navgrid = navgrid_create_full(width, height, NAVGRID_DIR_4, nullptr);
    maze_apply_to_navgrid(maze, navgrid);

    // Check if the maze has a reasonable number of blocks
    const coord_hash_t* blocked = maze_get_blocked_coords(maze);
    int n_blocked = coord_hash_length(blocked);
    CHECK(n_blocked > (width * height / 3)); // Must have some walls
    CHECK(n_blocked < (width * height));     // Cannot be fully blocked

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

TEST_CASE("Eller Algorithm Maze Generation") {
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

    CHECK(n_blocked > (width * height / 3)); // Must not have too few walls
    CHECK(n_blocked < (width * height));     // Cannot be fully blocked

    navgrid_print_ascii(navgrid);

    navgrid_destroy(navgrid);
    maze_destroy(maze);
}

TEST_CASE("Wilson Algorithm Maze Generation") {
    int x0 = 0, y0 = 0, width = 9, height = 9;
    maze_t* maze = maze_create_full(x0, y0, width, height);
    REQUIRE(maze != nullptr);

    maze_make_wilson(maze);

    navgrid_t* navgrid = navgrid_create_full(width, height, NAVGRID_DIR_4, NULL);
    REQUIRE(navgrid != nullptr);

    maze_apply_to_navgrid(maze, navgrid);

    const coord_hash_t* blocked = maze_get_blocked_coords(maze);
    int n_blocked = coord_hash_length(blocked);

    CHECK(n_blocked > (width * height / 3)); // Must not have too few walls
    CHECK(n_blocked < (width * height));     // Cannot be fully blocked

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

    CHECK(n_blocked > (width * height / 3)); // Must not have too few walls
    CHECK(n_blocked < (width * height));     // Cannot be fully blocked

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

    CHECK(n_blocked > (width * height / 3)); // Must not have too few walls
    CHECK(n_blocked < (width * height));     // Cannot be fully blocked

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

    CHECK(n_blocked > (width * height / 3)); // Must not have too few walls
    CHECK(n_blocked < (width * height));     // Cannot be fully blocked

    navgrid_print_ascii(navgrid);

    navgrid_destroy(navgrid);
    maze_destroy(maze);
}

TEST_CASE("Kruskal Algorithm Maze Generation") {
    int width = 9, height = 9;

    maze_t* maze = maze_create_full(0, 0, width, height);
    REQUIRE(maze != nullptr);

    maze_make_kruskal(maze);

    // Map conversion
    navgrid_t* navgrid = navgrid_create_full(width, height, NAVGRID_DIR_4, nullptr);
    REQUIRE(navgrid != nullptr);

    maze_apply_to_navgrid(maze, navgrid);

    // Check wall count (must not be too few or too many)
    const coord_hash_t* blocked = maze_get_blocked_coords(maze);
    int n_blocked = coord_hash_length(blocked);
    CHECK(n_blocked > (width * height / 4));
    CHECK(n_blocked < (width * height));

    // Display output
    navgrid_print_ascii(navgrid);

    // Free memory
    navgrid_destroy(navgrid);
    maze_destroy(maze);
}
