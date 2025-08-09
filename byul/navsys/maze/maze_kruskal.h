#ifndef MAZE_KRUSKAL_H
#define MAZE_KRUSKAL_H

#include "maze_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Generate a maze using Kruskal’s algorithm (Minimum Spanning Tree).
 *
 * This function generates a fully connected, 
 * acyclic maze using Kruskal’s algorithm.
 * Each open cell is treated as a node, and walls 
 * between adjacent cells are considered edges.
 * All edges are randomly shuffled and processed one by one. 
 * If a wall connects two cells
 * from different sets, it is removed and the sets are merged.
 *
 * ---
 *
 * ### Features
 * - Fully connected: all passage cells are reachable from any other.
 * - No cycles: the resulting maze is a perfect tree.
 * - Many dead ends: typical of MST-based generation.
 *
 * ---
 *
 * ### Input Constraints
 * - Both `width` and `height` must be **odd numbers** 
 *  and **greater than or equal to 3**.
 *   This ensures a valid alternating layout of walls and paths.
 *
 * ---
 *
 * ### Usage Example
 * @code
 * maze_t* maze = maze_make_kruskal(0, 0, 15, 15);
 * if (maze) {
 *     // Automatically resizes navgrid if smaller than the maze
 *     maze_apply_to_navgrid(maze, navgrid);
 *     navgrid_print_ascii(navgrid);
 * }
 * @endcode
 *
 * ---
 *
 * @param x0 Starting X coordinate of the maze
 * @param y0 Starting Y coordinate of the maze
 * @param width Maze width (must be an odd number ≥ 3)
 * @param height Maze height (must be an odd number ≥ 3)
 * @return Pointer to the generated `maze_t` structure, or `nullptr` on failure
 */
BYUL_API maze_t* maze_make_kruskal(int x0, int y0, int width, int height);

#ifdef __cplusplus
}
#endif

#endif // MAZE_KRUSKAL_H
