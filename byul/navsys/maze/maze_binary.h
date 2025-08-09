#ifndef MAZE_BINARY_H
#define MAZE_BINARY_H

#include "maze_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Generate a maze using the Binary Tree algorithm.
 *
 * This function creates a simple and fast maze using the Binary Tree algorithm.
 * Each passage cell randomly opens either to the east or to the south, producing
 * a maze with a diagonal bias and many straight corridors.
 *
 * ---
 *
 * ### Features
 * - Very fast and simple algorithm
 * - Produces mazes with many dead ends and diagonal bias
 * - Easy to implement and ideal for quick maze generation
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
 * maze_t* maze = maze_make_binary(0, 0, 15, 15);
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
BYUL_API maze_t* maze_make_binary(int x0, int y0, int width, int height);

#ifdef __cplusplus
}
#endif

#endif // MAZE_BINARY_H
