#ifndef MAZE_WILSON_H
#define MAZE_WILSON_H

#include "maze_core.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Generate a maze using Wilson's algorithm (loop-erased random walk).
 *
 * Wilson’s algorithm builds a uniform spanning tree by performing loop-erased
 * random walks from unvisited cells to the existing maze. 
 * It guarantees uniform
 * randomness and ensures every possible maze is equally likely.
 *
 * ---
 *
 * ### Features
 * - Produces a **perfect maze** (no cycles, full connectivity)
 * - Ensures **uniform probability** for all possible mazes
 * - Uses loop-erased random walks to avoid bias and revisits
 *
 * ---
 *
 * ### Constraints
 * - `width` and `height` must be **odd integers ≥ 3**
 *
 * ---
 *
 * ### Usage Example
 * @code
 * maze_t* maze = maze_make_wilson(0, 0, 21, 21);
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
 * @param width Width of the maze (must be odd and ≥ 3)
 * @param height Height of the maze (must be odd and ≥ 3)
 * @return Pointer to the generated `maze_t` structure, 
 * or `nullptr` on failure
 */
BYUL_API maze_t* maze_make_wilson(int x0, int y0, int width, int height);

#ifdef __cplusplus
}
#endif

#endif // MAZE_WILSON_H
