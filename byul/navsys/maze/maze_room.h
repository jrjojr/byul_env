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
 * The result is a hybrid **RPG-style map** composed of rooms, corridors, and maze paths.
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
 *    - The corridors are shaped like an "L" (horizontal-first or vertical-first, chosen randomly).
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
 * - As maze generation is a later step, **the result is not strictly tree-like**.
 * - Dead-ends may exist but are limited,  
 *   making this ideal for **centralized level layouts**.
 *
 * ---
 *
 * ### Usage Requirements
 * - `maze_t` must be created using `maze_create_full()`.
 * - `width` and `height` must be **odd numbers** and **at least 9**.
 *
 * ---
 *
 * ### Example
 * @code
 * maze_t* maze = maze_create_full(0, 0, 31, 21, MAZE_TYPE_ROOM_BLEND);
 * maze_make_room_blend(maze);
 * navgrid_add_maze(navgrid, maze);
 * @endcode
 *
 * ---
 *
 * @param maze Pointer to the target maze structure to generate
 */
BYUL_API void maze_make_room_blend(maze_t* maze);

#ifdef __cplusplus
}
#endif

#endif // MAZE_ROOM_H
