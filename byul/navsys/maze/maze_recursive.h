#ifndef MAZE_RECURSIVE_H
#define MAZE_RECURSIVE_H

#include "maze_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Generate a maze using recursive backtracking (depth-first search).
 *
 * This function builds a perfect maze 
 * (no loops, single path between any two cells)
 * using recursive backtracking. It starts 
 * from an initial cell (1,1) and carves
 * random passages by visiting unvisited neighbors and removing walls.
 *
 * ---
 *
 * ### Features
 * - Creates a maze with no cycles (tree-like structure)
 * - Randomness provides unique layout on each execution
 * - Works only with odd-width and odd-height grids
 *
 * ---
 *
 * ### Input Constraints
 * - Both `width` and `height` must be **odd numbers**
 * - Minimum size: 3x3
 *
 * ---
 *
 * ### Usage Example
 * @code
 * maze_t* maze = maze_make_recursive(0, 0, 11, 11);
 * if (maze) {
 *     maze_apply_to_navgrid(maze, navgrid);
 *     navgrid_print_ascii(navgrid);
 * }
 * @endcode
 *
 * ---
 *
 * @param x0 Starting X coordinate of the maze
 * @param y0 Starting Y coordinate of the maze
 * @param width Maze width (must be odd and ≥ 3)
 * @param height Maze height (must be odd and ≥ 3)
 * @return Pointer to the generated maze (`maze_t*`), or `nullptr` on failure
 */
BYUL_API maze_t* maze_make_recursive(int x0, int y0, int width, int height);

#ifdef __cplusplus
}
#endif

#endif // MAZE_RECURSIVE_H
