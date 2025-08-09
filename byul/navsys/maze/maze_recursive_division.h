#ifndef MAZE_RECURSIVE_DIVISION_H
#define MAZE_RECURSIVE_DIVISION_H

#include "maze_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Create a maze using the Recursive Division algorithm.
 *
 * This function generates a maze by recursively dividing rectangular
 * regions with horizontal or vertical walls, leaving a single passage
 * through each wall to maintain partial connectivity.
 *
 * ### Algorithm Overview
 * - Starting from the full area, insert a wall either horizontally or vertically.
 * - Carve a single random passage through that wall.
 * - Recursively repeat the process for the resulting subregions.
 * - Stop dividing when the region is too small to continue.
 *
 * ### Characteristics
 * - **Not tree-based**: the resulting maze may have disconnected regions.
 * - Produces **long corridors**, **symmetrical layouts**, and **few dead-ends**.
 * - It is visually clean and architectural in style.
 *
 * ### When to Use
 * - If full connectivity is required, prefer tree-based algorithms such as
 *   `MAZE_TYPE_PRIM` or `MAZE_TYPE_KRUSKAL`.
 *
 * @param x0 Starting x-coordinate in world space.
 * @param y0 Starting y-coordinate in world space.
 * @param width Width of the maze (must be odd and ≥ 3).
 * @param height Height of the maze (must be odd and ≥ 3).
 * @return Pointer to a `maze_t` object representing the generated maze.
 *         Returns `nullptr` if the input dimensions are invalid.
 *
 * @see maze_t
 */
BYUL_API maze_t* maze_make_recursive_division(
    int x0, int y0, int width, int height);

#ifdef __cplusplus
}
#endif

#endif // MAZE_RECURSIVE_DIVISION_H
