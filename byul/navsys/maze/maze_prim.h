#ifndef MAZE_PRIM_H
#define MAZE_PRIM_H

#include "maze_core.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Generate a maze using Prim's algorithm.
 *
 * This function creates a fully connected, acyclic maze using
 * the randomized version of **Prim's algorithm**.
 * It starts from a random passage cell and incrementally
 * adds adjacent walls to a list, carving passages between unvisited regions.
 *
 * ---
 *
 * ### Features
 * - The resulting maze is **a single connected tree** (no loops).
 * - **Dead ends are common**, creating a classic dungeon-like layout.
 * - Randomized growth leads to organic, irregular shapes.
 *
 * ---
 *
 * ### Input Constraints
 * - Both `width` and `height` must be **odd numbers** and at least **3**.
 *   (This ensures alternating walls/passages structure.)
 *
 * ---
 *
 * ### Usage Example
 * @code
 * maze_t* maze = maze_maze_prim(0, 0, 21, 21);
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
BYUL_API maze_t* maze_maze_prim(int x0, int y0, int width, int height);

#ifdef __cplusplus
}
#endif

#endif // MAZE_PRIM_H
