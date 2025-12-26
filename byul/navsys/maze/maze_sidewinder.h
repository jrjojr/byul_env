#ifndef MAZE_SIDEWINDER_H
#define MAZE_SIDEWINDER_H

#include "maze_core.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Generate a maze using the Sidewinder algorithm.
 *
 * The Sidewinder algorithm creates mazes by sweeping row-by-row from west to east,
 * carving eastward corridors and occasionally connecting them northward to
 * previous rows. This method produces mazes with long horizontal passages
 * and sparse vertical connectors.
 *
 * ---
 *
 * ### Features
 * - Generates a **perfect maze** (fully connected, no cycles)
 * - Produces **long horizontal corridors** and sparse vertical gaps
 * - Each row is processed independently, making the algorithm simple and fast
 *
 * ---
 *
 * ### Constraints
 * - Both `width` and `height` must be **odd integers ≥ 3**
 *
 * ---
 *
 * ### Usage Example
 * @code
 * maze_t* maze = maze_make_sidewinder(0, 0, 21, 21);
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
 * @return Pointer to the generated `maze_t` structure, or `nullptr` on failure
 */
BYUL_API maze_t* maze_make_sidewinder(int x0, int y0, int width, int height);

#ifdef __cplusplus
}
#endif

#endif // MAZE_SIDEWINDER_H
