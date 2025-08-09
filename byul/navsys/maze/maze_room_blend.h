#ifndef MAZE_ROOM_H
#define MAZE_ROOM_H

#include "coord_hash.h"
#include "maze.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Room structure definition
 *
 * Represents a rectangular room on the map.
 */
typedef struct {
    int x, y;   ///< Top-left coordinate of the room
    int w, h;   ///< Width and height of the room
} room_t;

/**
 * @brief Generate a maze using Room + Maze blending algorithm.
 *
 * This function first creates **rooms**,
 * connects them with **corridors**,  
 * and then fills the remaining space using a **backtracking maze algorithm**.
 * 
 * The result is a hybrid **RPG-style map** composed of rooms, corridors, 
 * and maze paths.
 *
 * ---
 *
 * ### Algorithm Overview
 * 1. **Room Placement**  
 *    - Randomly attempts to place rooms within the specified size range.  
 *    - Overlap checks are performed to prevent rooms from overlapping.  
 * 
 * 2. **Corridor Digging**  
 *    - Corridors are generated between the centers of consecutive rooms.  
 *    - The corridors are shaped like an "L" 
 * (horizontal-first or vertical-first, chosen randomly).
 * 
 * 3. **Maze Filling**  
 *    - Any remaining wall area not covered by rooms or corridors is filled
 *      using a backtracking-based maze generation algorithm.
 *    - This ensures connectivity and creates a varied, interesting structure.
 *
 * ---
 *
 * ### Features
 * - **Rooms are open areas**, **corridors are long and narrow**,  
 *   **mazes fill the remaining space with tight paths.**
 * - As maze generation is a later step, 
 * **the result is not strictly tree-like**.
 * - Dead-ends may exist but are limited,  
 *   making this ideal for **centralized level layouts**.
 *
 * ---
 *
 */
BYUL_API maze_t* maze_make_room_blend(int x0, int y0, int width, int height);

#ifdef __cplusplus
}
#endif

#endif // MAZE_ROOM_H
