#ifndef MAZE_ALDOUS_BRODER_H
#define MAZE_ALDOUS_BRODER_H

#include "maze_core.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Generate a maze using the Aldous-Broder algorithm (random walk).
 *
 * This function creates a maze using the Aldous-Broder algorithm, which performs
 * a random walk over the grid, carving out passages only when visiting unvisited cells.
 * Though simple and unbiased, it can be slow due to frequent revisits to already visited cells.
 *
 * ---
 *
 * ### Features
 * - Produces a **perfect maze** (no loops, full connectivity)
 * - **Uniform randomness**: all mazes are equally probable
 * - Can be inefficient for large grids due to random walk nature
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
 * maze_t* maze = maze_make_aldous_broder(0, 0, 21, 21);
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
BYUL_API maze_t* maze_make_aldous_broder(int x0, int y0, int width, int height);

#ifdef __cplusplus
}
#endif

#endif // MAZE_ALDOUS_BRODER_H
