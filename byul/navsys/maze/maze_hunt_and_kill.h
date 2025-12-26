#ifndef MAZE_HUNT_AND_KILL_H
#define MAZE_HUNT_AND_KILL_H

#include "maze_core.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Generate a maze using the Hunt-and-Kill algorithm.
 *
 * Hunt-and-Kill algorithm alternates between random walks (Kill phase) and
 * scanning for new starting points (Hunt phase). It creates mazes with a mix
 * of long corridors and sparse branching, 
 * often resulting in natural-looking paths.
 *
 * ---
 *
 * ### Features
 * - Generates a **perfect maze** (fully connected, no cycles)
 * - Tends to produce **long straight corridors**
 * - Alternates between two phases:
 *   - **Kill phase**: Random walk from current cell, carving passages
 *   - **Hunt phase**: Scan for unvisited cells adjacent to visited ones
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
 * maze_t* maze = maze_make_hunt_and_kill(0, 0, 21, 21);
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
BYUL_API maze_t* maze_make_hunt_and_kill(int x0, int y0, int width, int height);

#ifdef __cplusplus
}
#endif

#endif // MAZE_HUNT_AND_KILL_H
