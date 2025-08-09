#ifndef MAZE_ELLER_H
#define MAZE_ELLER_H

#include "maze_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Generate a maze using the Eller’s algorithm (line-by-line method).
 *
 * This function generates a maze row by row using Eller’s algorithm,
 * which balances complexity and connectivity. It maintains a disjoint-set structure
 * to manage connected regions and ensures that each row is properly joined
 * to the next to guarantee full connectivity without isolated sections.
 *
 * ---
 *
 * ### Features
 * - Line-by-line generation (row-at-a-time approach)
 * - Efficient and scalable for large mazes
 * - Ensures full connectivity from top to bottom
 * - Produces mazes with varied corridor patterns and some dead ends
 *
 * ---
 *
 * ### Input Constraints
 * - Both `width` and `height` must be **odd numbers** and at least **3**
 *
 * ---
 *
 * ### Usage Example
 * @code
 * maze_t* maze = maze_make_eller(0, 0, 21, 21);
 * if (maze) {
 *     maze_apply_to_navgrid(maze, navgrid);
 *     navgrid_print_ascii(navgrid);
 *     maze_destroy(maze);
 * }
 * @endcode
 *
 * ---
 *
 * @param x0 Starting X coordinate of the maze
 * @param y0 Starting Y coordinate of the maze
 * @param width Maze width (must be odd and ≥ 3)
 * @param height Maze height (must be odd and ≥ 3)
 * @return Pointer to the generated `maze_t` structure, or `nullptr` on failure
 */
BYUL_API maze_t* maze_make_eller(int x0, int y0, int width, int height);

#ifdef __cplusplus
}
#endif

#endif // MAZE_ELLER_H
