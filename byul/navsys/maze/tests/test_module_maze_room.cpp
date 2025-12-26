#include "doctest.h"
#include "maze_core.h"
#include "maze_room_blend.h"
#include "navgrid.h"
#include "console.h"

TEST_CASE("Room + Maze Blending Algorithm") {
    int x0 = 0, y0 = 0, width = 31, height = 21;
    maze_t* maze = maze_make_room_blend(x0, y0, width, height);
    REQUIRE(maze != nullptr);

    const coord_hash_t* blocked = maze_get_blocked_coords(maze);
    int n_blocked = coord_hash_length(blocked);

    CHECK(n_blocked > (width * height / 4));
    CHECK(n_blocked < (width * height));

    navgrid_t* navgrid = navgrid_create_full(width, height, NAVGRID_DIR_4, nullptr);
    REQUIRE(navgrid != nullptr);

    maze_apply_to_navgrid(maze, navgrid);
    navgrid_print_ascii(navgrid);

    navgrid_destroy(navgrid);
    maze_destroy(maze);
}
