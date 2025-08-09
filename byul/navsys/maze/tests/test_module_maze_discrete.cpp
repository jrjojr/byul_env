// test_coord.cpp

#include "doctest.h"
#include <locale.h>
#include <iostream>

extern "C" {
#include "maze.h"
#include "console.h"

#include "maze_recursive.h"
#include "maze_prim.h"
#include "maze_binary.h"
#include "maze_eller.h"

#include "maze_aldous_broder.h"
#include "maze_wilson.h"
#include "maze_hunt_and_kill.h"
#include "maze_sidewinder.h"

#include "maze_recursive_division.h"
#include "maze_kruskal.h"
#include "maze_room_blend.h"
}

TEST_CASE("Maze generation and map application") {
    maze_t* maze = nullptr;

    maze = maze_make_recursive(5, 5, 19, 19);

    navgrid_t* navgrid = navgrid_create_full(5, 5, NAVGRID_DIR_4, NULL);
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
    maze_t* maze =     maze_maze_prim(x0, y0, width, height);
    CHECK(maze != nullptr);

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
    maze_t* maze = maze_make_binary(0, 0, 9, 9);
    REQUIRE(maze != nullptr);

    navgrid_t* navgrid = navgrid_create();
    maze_apply_to_navgrid(maze, navgrid);

    navgrid_print_ascii(navgrid);

    navgrid_destroy(navgrid);
    maze_destroy(maze);
}

TEST_CASE("Eller Algorithm Maze Generation") {
    maze_t* maze = maze_make_eller(0, 0, 9, 9);
    REQUIRE(maze != nullptr);

    navgrid_t* navgrid = navgrid_create_full(9, 9, NAVGRID_DIR_4, NULL);
    REQUIRE(navgrid != nullptr);

    maze_apply_to_navgrid(maze, navgrid);

    navgrid_print_ascii(navgrid);

    maze_destroy(maze);
    navgrid_destroy(navgrid);
}

TEST_CASE("Aldous-Broder Maze Generation") {
    int x0 = 0, y0 = 0, width = 9, height = 9;
    maze_t* maze = maze_make_aldous_broder(x0, y0, width, height);
    REQUIRE(maze != nullptr);

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
    maze_t* maze = maze_make_wilson(x0, y0, width, height);
    REQUIRE(maze != nullptr);

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
    maze_t* maze = maze_make_hunt_and_kill(x0, y0, width, height);
    REQUIRE(maze != nullptr);

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
    maze_t* maze = maze_make_sidewinder(x0, y0, width, height);
    REQUIRE(maze != nullptr);

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
    maze_t* maze = maze_make_recursive_division(x0, y0, width, height);
    REQUIRE(maze != nullptr);

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
    maze_t* maze = nullptr;
    maze = maze_make_kruskal(0, 0, 19, 19);

    REQUIRE(maze != nullptr);

    navgrid_t* navgrid = navgrid_create();
    REQUIRE(navgrid != nullptr);

    maze_apply_to_navgrid(maze, navgrid);

    // Display output
    navgrid_print_ascii(navgrid);

    // Free memory
    navgrid_destroy(navgrid);
    maze_destroy(maze);
}
