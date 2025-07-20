#include "doctest.h"
#include "internal/maze_common.h"
#include "internal/maze_room.h"
#include "internal/navgrid.h"
#include "internal/console.h"

TEST_CASE("Room + Maze Blending Algorithm") {
    int x0 = 0, y0 = 0, width = 31, height = 21;
    maze_t* maze = maze_create_full(x0, y0, width, height);
    REQUIRE(maze != nullptr);

    maze_make_room_blend(maze);

    const coord_hash_t* blocked = maze_get_blocked_coords(maze);
    int n_blocked = coord_hash_length(blocked);

    // 벽이 전혀 없는 경우는 잘못된 미로
    CHECK(n_blocked > (width * height / 4));
    CHECK(n_blocked < (width * height));

    // 맵에 적용 후 확인
    navgrid_t* navgrid = navgrid_create_full(width, height, NAVGRID_DIR_4, nullptr);
    REQUIRE(navgrid != nullptr);

    maze_apply_to_navgrid(maze, navgrid);  // 기존 maze_to_navgrid 이름 변경 기준
    navgrid_print_ascii(navgrid);

    navgrid_destroy(navgrid);
    maze_destroy(maze);
}
