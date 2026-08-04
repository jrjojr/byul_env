#include <cassert>
#include <cstddef>
#include <type_traits>

#include "navgrid_abi1.h"
#include "obstacle_abi1.h"
#include "maze_abi1.h"

int main() {
    static_assert(sizeof(void*) == 8);
    static_assert(std::is_standard_layout_v<navgrid_t>);
    static_assert(sizeof(navgrid_t) == 40);
    static_assert(alignof(navgrid_t) == 8);
    static_assert(offsetof(navgrid_t, width) == 0);
    static_assert(offsetof(navgrid_t, height) == 4);
    static_assert(offsetof(navgrid_t, mode) == 8);
    static_assert(offsetof(navgrid_t, cell_map) == 16);
    static_assert(offsetof(navgrid_t, is_coord_blocked_fn) == 24);
    static_assert(offsetof(navgrid_t, is_coord_blocked_fn_userdata) == 32);
    static_assert(std::is_standard_layout_v<obstacle_t>);
    static_assert(sizeof(obstacle_t) == 24);
    static_assert(alignof(obstacle_t) == 8);
    static_assert(offsetof(obstacle_t, x0) == 0);
    static_assert(offsetof(obstacle_t, y0) == 4);
    static_assert(offsetof(obstacle_t, width) == 8);
    static_assert(offsetof(obstacle_t, height) == 12);
    static_assert(offsetof(obstacle_t, blocked) == 16);
    static_assert(std::is_standard_layout_v<maze_t>);
    static_assert(sizeof(maze_t) == 24);
    static_assert(alignof(maze_t) == 8);
    static_assert(offsetof(maze_t, x0) == 0);
    static_assert(offsetof(maze_t, y0) == 4);
    static_assert(offsetof(maze_t, width) == 8);
    static_assert(offsetof(maze_t, height) == 12);
    static_assert(offsetof(maze_t, blocked) == 16);

    navgrid_abi_mismatch_t mismatch = NAVGRID_ABI_VERSION_MISMATCH;
    assert(navgrid_check_abi(
        BYUL_NAVGRID_ABI1_VERSION,
        BYUL_NAVGRID_ABI1_FINGERPRINT,
        &mismatch) == NAVSYS_STATUS_OK);
    assert(mismatch == NAVGRID_ABI_MATCH);
    navgrid_t* grid = navgrid_create_full(7, 9, NAVGRID_DIR_4, nullptr);
    assert(grid != nullptr);
    assert(grid->width == 7);
    assert(grid->height == 9);
    assert(grid->mode == NAVGRID_DIR_4);
    assert(grid->cell_map != nullptr);
    assert(grid->is_coord_blocked_fn == is_coord_blocked_navgrid);
    assert(grid->is_coord_blocked_fn_userdata == nullptr);
    obstacle_abi_mismatch_t obstacle_mismatch = OBSTACLE_ABI_VERSION_MISMATCH;
    assert(obstacle_check_abi(
        BYUL_OBSTACLE_ABI1_VERSION,
        BYUL_OBSTACLE_ABI1_FINGERPRINT,
        &obstacle_mismatch) == NAVSYS_STATUS_OK);
    assert(obstacle_mismatch == OBSTACLE_ABI_MATCH);
    obstacle_t* obstacle = obstacle_create_full(1, 2, 7, 9);
    assert(obstacle != nullptr);
    assert(obstacle->x0 == 1);
    assert(obstacle->y0 == 2);
    assert(obstacle->width == 7);
    assert(obstacle->height == 9);
    assert(obstacle->blocked != nullptr);
    byul_maze_abi_mismatch_t maze_mismatch = BYUL_MAZE_ABI_VERSION_MISMATCH;
    assert(byul_maze_check_abi(
        BYUL_MAZE_ABI1_VERSION,
        BYUL_MAZE_ABI1_FINGERPRINT,
        &maze_mismatch) == NAVSYS_STATUS_OK);
    assert(maze_mismatch == BYUL_MAZE_ABI_MATCH);
    maze_t* maze = maze_create_full(3, 4, 5, 7);
    assert(maze != nullptr);
    assert(maze->x0 == 3);
    assert(maze->y0 == 4);
    assert(maze->width == 5);
    assert(maze->height == 7);
    assert(maze->blocked != nullptr);
    maze_destroy(maze);
    obstacle_destroy(obstacle);
    navgrid_destroy(grid);
    return 0;
}
