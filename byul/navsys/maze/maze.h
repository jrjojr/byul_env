#ifndef MAZE_H
#define MAZE_H

#include "byul_common.h"
#include "coord.h"
#include "coord_hash.h"
#include "navgrid.h"
#include "maze_common.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MAZE_TYPE_RECURSIVE,
    MAZE_TYPE_PRIM,
    MAZE_TYPE_BINARY,
    MAZE_TYPE_ELLER,
    MAZE_TYPE_ALDOUS_BRODER,
    MAZE_TYPE_WILSON,
    MAZE_TYPE_HUNT_AND_KILL,
    MAZE_TYPE_SIDEWINDER,
    MAZE_TYPE_RECURSIVE_DIVISION,
    MAZE_TYPE_KRUSKAL,
    MAZE_TYPE_ROOM_BLEND
} maze_type_t;

BYUL_API void maze_make(maze_t* maze, maze_type_t type);

/**
 * @brief Generates a maze using the Recursive Division algorithm.
 *
 * This function creates a wall and passage layout within the area
 * defined by the `maze_t` structure (`x0`, `y0`, `width`, `height`)
 * using the classic **Recursive Division algorithm**.
 * It divides the grid into walls and passages in steps of 2 cells,
 * and it is recommended to use odd dimensions (e.g., 9x9).
 *
 * - The generated result is stored in `maze->blocked`,
 *   and can be inserted into a `navgrid_t` by using `maze_apply_to_navgrid()`.
 * - This function should only be called when the `type` field of `maze_t`
 *   is set to `MAZE_TYPE_RECURSIVE`.
 * - The maze coordinates are based on absolute positions,
 *   where `x0`, `y0` represent the top-left origin of the maze.
 *
 * @note The maze is divided using only the 4 cardinal directions (up, down, left, right),
 *       and no diagonal connections are created.
 * @note The maze is generated with closed outer walls,
 *       so entrances or exits need to be carved separately.
 *
 * @param maze Pointer to a `maze_t` structure where maze info and results will be stored.
 *
 * @see maze_create_full()
 * @see maze_apply_to_navgrid()
 */
BYUL_API void maze_make_recursive(maze_t* maze);

/**
 * @brief Generates a maze using the Prim algorithm.
 *
 * This function generates a maze inside the specified `maze_t` structure
 * based on the Prim algorithm. The generated maze is recorded in the internal
 * `blocked` coordinate set, and can be inserted into a `navgrid_t` using
 * the `maze_apply_to_navgrid()` function if needed.
 *
 * @details
 * The Prim algorithm is an application of the Minimum Spanning Tree (MST)
 * concept for maze generation. Starting from one cell surrounded by walls,
 * it randomly selects adjacent walls to connect cells, while ensuring that
 * only walls with exactly one visited cell are chosen. This results in a
 * single connected passage network.
 *
 * It is recommended to use odd values for `width` and `height` for the maze,
 * as all cells are located at odd coordinates, and even coordinates are
 * considered walls.
 *
 * @usage
 * ```c
 * maze_t* maze = maze_create_full(0, 0, 21, 21, MAZE_TYPE_PRIM);
 * maze_make_prim(maze);
 * navgrid_t* navgrid = navgrid_create_full(21, 21, NAVGRID_DIR_4, NULL);
 * maze_apply_to_navgrid(maze, navgrid);
 * // Use the navgrid here
 * maze_destroy(maze);
 * navgrid_destroy(navgrid);
 * ```
 *
 * @param maze Pointer to the `maze_t` structure where the maze will be generated (must not be NULL).
 */
BYUL_API void maze_make_prim(maze_t* maze);

/**
 * @brief Generates a maze using the Binary Tree algorithm.
 *
 * This function uses the Binary Tree maze generation algorithm to set the
 * `blocked` coordinate set inside the given `maze_t` structure.
 *
 * The Binary Tree algorithm is a very simple approach that removes walls
 * only towards the north or east from each cell.
 * While it is simple and fast to implement, the resulting maze tends
 * to be biased.
 *
 * - The maze will always have paths leaning toward the right or downward,
 *   resulting in low complexity.
 * - Due to its strong regularity, additional algorithms or post-processing
 *   are often needed in real game scenarios.
 *
 * @param maze A pointer to a `maze_t` structure with its coordinates,
 *      width, height, and type (MAZE_TYPE_BINARY) already set.
 *      This function modifies the `blocked` field within the structure
 *      to generate the maze.
 *
 * @note `maze` must be a valid pointer, and
 *       `width` and `height` must be odd values
 *       (to properly differentiate walls from passages).
 *
 * @see maze_create_full()
 * @see maze_apply_to_navgrid()
 *
 * @example
 * ```c
 * maze_t* maze = maze_create_full(0, 0, 9, 9, MAZE_TYPE_BINARY);
 * maze_make_binary(maze);
 * navgrid_t* navgrid = navgrid_create();
 * maze_apply_to_navgrid(maze, navgrid);
 * ```
 */
BYUL_API void maze_make_binary(maze_t* maze);

/**
 * @brief Generates a maze using the Eller algorithm.
 *
 * This function generates a maze pattern inside the specified `maze_t`
 * structure based on the classic **Eller algorithm**. The maze is created
 * **row by row**, where each row is managed with unique set IDs that are
 * merged and connected to form a fully connected structure.
 *
 * ### Algorithm Overview
 * - Assign a unique set ID to each odd cell.
 * - Randomly merge adjacent horizontal cells to create horizontal passages.
 * - Ensure that each set has at least one vertical passage to the next row.
 * - On the last row, merge all remaining sets to form a single connected structure.
 *
 * Note: This algorithm does not enforce outer walls,
 * and additional walls may need to be added as post-processing.
 * The implementation adheres closely to the original principle, and
 * whether the boundary is open or closed may vary due to randomness.
 *
 * ### Usage Requirements
 * This function must be used with a `maze_t` structure created using
 * `maze_create_full()`, and the following constraints must be satisfied:
 * - Width and height must be odd values (e.g., 9x9, 11x7).
 * - Both width and height must be at least 3.
 *
 * ### Example Usage
 * @code
 * maze_t* maze = maze_create_full(0, 0, 9, 9, MAZE_TYPE_ELLER);
 * maze_make_eller(maze);
 * navgrid_add_maze(navgrid, maze);
 * @endcode
 *
 * After execution, wall coordinates are stored in `maze->blocked`,
 * and `maze->type` is set to `MAZE_TYPE_ELLER`.
 *
 * @param maze Pointer to a `maze_t` structure where maze data will be generated
 */
BYUL_API void maze_make_eller(maze_t* maze);

/**
 * @brief Generates a maze using the Aldous-Broder algorithm.
 *
 * This function generates a maze in the specified `maze_t` structure  
 * using the **Aldous-Broder algorithm**, which is based on a random walk.  
 * All cells are traversed randomly, and **passages are opened only when an unvisited cell is reached**.
 *
 * ### Algorithm Overview
 * - Start from a random cell and continuously move to a neighboring cell.
 * - **When a new unvisited cell is visited for the first time**,  
 *   open a passage between the current cell and the next cell.
 * - If the cell has already been visited, simply move without creating a passage.
 * - Repeat until all cells are visited, resulting in a fully connected maze.
 *
 * This algorithm is very simple and  
 * **is one of the few algorithms that generates all possible mazes with uniform probability**.
 *
 * Note: It can be inefficient on average,  
 * especially when the number of cells is large.
 *
 * Note: Outer walls are not enforced by this algorithm.  
 * Add outer walls using `maze_make()` or post-processing if needed.
 *
 * ---
 *
 * ### Usage Requirements
 * - `maze_t` must be created using the `maze_create_full()` function.
 * - Both `width` and `height` must be **odd numbers** and **at least 3**.
 *
 * ### Example Usage
 * @code
 * maze_t* maze = maze_create_full(0, 0, 9, 9, MAZE_TYPE_ALDOUS_BRODER);
 * maze_make_aldous_broder(maze);
 * navgrid_add_maze(navgrid, maze);
 * @endcode
 *
 * After execution, wall coordinates are stored in `maze->blocked`,  
 * and `maze->type` is set to `MAZE_TYPE_ALDOUS_BRODER`.
 *
 * @param maze Pointer to the `maze_t` structure where the maze will be generated
 */
BYUL_API void maze_make_aldous_broder(maze_t* maze);

/**
 * @brief Generates a maze using the Wilson algorithm.
 *
 * This function generates a **uniform, unbiased** maze  
 * using **Wilson's Algorithm**.  
 * The key concept is **Loop-Erased Random Walk (LERW)**,  
 * which removes loops during random walks to create a pure tree structure.
 *
 * ### Algorithm Overview
 * - Choose one cell as the **initial cell** and mark it as visited.
 * - Pick any unvisited cell and start a **random walk**.
 * - Continue moving until reaching a visited cell,  
 *   and erase loops that form during the random walk (loop erase).
 * - Add the resulting path as **PASSAGES** and mark those cells as visited.
 * - Repeat this process **until all cells are included**,  
 *   forming a complete spanning tree.
 *
 * Note: This algorithm theoretically generates the **most fair (uniform distribution)** mazes.
 * The drawback is that it can be slow and is more complex to implement.
 *
 * ---
 *
 * ### Usage Requirements
 * - `maze_t` must be created using the `maze_create_full()` function.
 * - Both `width` and `height` must be **odd numbers** and at least **3**.
 *
 * ### Outer Wall Handling
 * - This algorithm does not enforce outer walls.  
 *   Use `maze_make()` or post-processing to add walls if needed.
 *
 * ### Example
 * @code
 * maze_t* maze = maze_create_full(0, 0, 9, 9, MAZE_TYPE_WILSON);
 * maze_make_wilson(maze);
 * navgrid_add_maze(navgrid, maze);
 * @endcode
 *
 * After execution, wall coordinates are stored in `maze->blocked`,  
 * and `maze->type` is set to `MAZE_TYPE_WILSON`.
 *
 * @param maze Pointer to the `maze_t` structure where the maze will be generated
 */
BYUL_API void maze_make_wilson(maze_t* maze);

/**
 * @brief Generates a maze using the Hunt-and-Kill algorithm.
 *
 * This function generates a maze within the specified `maze_t` structure
 * using the **Hunt-and-Kill algorithm**, which is a depthless random walk method.
 * It is simpler than DFS (Depth-First Search) and can quickly create
 * a natural **tree-shaped maze**.
 *
 * ### Algorithm Overview
 * - **Kill Phase**: Choose a random direction and move if there is
 *   an unvisited neighbor, carving a passage.
 * - If no unvisited neighbors are available, end the exploration
 *   and proceed to the next step.
 *
 * - **Hunt Phase**: Scan all cells to find a cell that has
 *   not been visited but is adjacent to a visited cell.  
 *   Carve a passage between them and re-enter the Kill Phase.
 *
 * This process is repeated **until all cells are visited**,  
 * resulting in a fully connected tree-like maze.
 *
 * Note: This algorithm is simple, fast, and generates
 * a typical tree maze with many dead-ends.
 *
 * Note: Outer walls are not enforced by this algorithm.  
 * They should be added with post-processing if needed.
 *
 * ---
 *
 * ### Usage Requirements
 * - `maze_t` must be created using the `maze_create_full()` function.
 * - Both `width` and `height` must be **odd numbers** and **at least 3**.
 *
 * ### Example Usage
 * @code
 * maze_t* maze = maze_create_full(0, 0, 9, 9, MAZE_TYPE_HUNT_AND_KILL);
 * maze_make_hunt_and_kill(maze);
 * navgrid_add_maze(navgrid, maze);
 * @endcode
 *
 * After execution, wall coordinates are recorded in `maze->blocked`,
 * and `maze->type` is set to `MAZE_TYPE_HUNT_AND_KILL`.
 *
 * @param maze Pointer to the `maze_t` structure where the maze will be generated
 */
BYUL_API void maze_make_hunt_and_kill(maze_t* maze);

/**
 * @brief Generates a maze using the Sidewinder algorithm.
 *
 * This function generates a maze inside the specified `maze_t` structure  
 * based on the **Sidewinder algorithm**, which creates passages with  
 * a rightward bias. It can be considered a horizontal extension of the  
 * Binary Tree algorithm, resulting in **open mazes with long horizontal  
 * passages and fewer vertical connections**.
 *
 * ### Algorithm Overview
 * - Process the maze **row by row from top to bottom**.
 * - Maintain a **run set** for each row and randomly decide whether to connect to the right.
 * - If not connecting to the right, select one cell from the run set and  
 *   create a connection upward, then reset the run set.
 * - Repeat this process for all rows.
 *
 * Note: This algorithm guarantees a cycle-free tree structure and  
 * creates **smooth mazes with very few dead-ends**.
 *
 * Note: Outer walls are not enforced by the algorithm and  
 * must be added separately if required.
 *
 * ---
 *
 * ### Usage Requirements
 * - `maze_t` must be created using the `maze_create_full()` function.
 * - Both `width` and `height` must be **odd values** and **at least 3**.
 *
 * ### Example
 * @code
 * maze_t* maze = maze_create_full(0, 0, 9, 9, MAZE_TYPE_SIDEWINDER);
 * maze_make_sidewinder(maze);
 * navgrid_add_maze(navgrid, maze);
 * @endcode
 *
 * After execution, wall coordinates are stored in `maze->blocked`,  
 * and `maze->type` is set to `MAZE_TYPE_SIDEWINDER`.
 *
 * @param maze Pointer to the `maze_t` structure where the maze will be generated
 */
BYUL_API void maze_make_sidewinder(maze_t* maze);

/**
 * @brief Generates a maze using the Recursive Division algorithm.
 *
 * This function uses the **Recursive Division** algorithm to divide the maze space
 * with walls, creating passages at random locations to build an
 * orderly, architectural-style maze.
 *
 * ### Algorithm Overview
 * - Starting with the entire maze area, place a wall either horizontally or vertically.
 * - Create **one random passage** through that wall.
 * - Recursively repeat the same process for the two subregions divided by the wall.
 * - Stop dividing when the region becomes too small.
 *
 * ### Important Characteristics
 * - This algorithm **is not tree-based** and produces  
 *   **open structures with very few dead-ends**.
 * - It **does not guarantee connectivity**.  
 *   -> Some rooms or corridors might remain disconnected,  
 *   requiring additional post-processing (e.g., `fix_disconnected_regions()`).
 *
 * Therefore, if a fully connected path network is required,  
 * tree-based algorithms like `MAZE_TYPE_PRIM` or `MAZE_TYPE_KRUSKAL` are recommended.
 *
 * ---
 *
 * ### Usage Requirements
 * - `maze_t` must be created using `maze_create_full()`.
 * - Both `width` and `height` must be **odd and at least 3**.
 *
 * ### Example
 * @code
 * maze_t* maze = maze_create_full(0, 0, 9, 9, MAZE_TYPE_RECURSIVE_DIVISION);
 * maze_make_recursive_division(maze);
 * navgrid_add_maze(navgrid, maze);
 * @endcode
 *
 * After execution:
 * - Wall coordinates are recorded in `maze->blocked`.
 * - `maze->type` is set to `MAZE_TYPE_RECURSIVE_DIVISION`.
 *
 * @param maze Pointer to the `maze_t` structure where the maze will be generated
 */
BYUL_API void maze_make_recursive_division(maze_t* maze);

/**
 * @brief Generates a fully connected maze using the Kruskal algorithm.
 *
 * This function uses **Kruskal's Algorithm** to build a Minimum Spanning Tree (MST),
 * creating a **fully connected maze without cycles**.
 * Each passage cell starts as a **separate set**, and walls between
 * adjacent cells are processed in random order.
 * **If the two cells belong to different sets, the wall is removed
 * and the sets are merged.**
 *
 * ### Algorithm Overview
 * - Initialize all odd-coordinate cells as PASSAGE and assign each to a unique set.
 * - Treat walls between adjacent cells as edges and store them in a list.
 * - Shuffle the wall list randomly, then process each wall:
 *   - If the cells on both sides belong to different sets -> remove the wall and union the sets.
 *   - If they belong to the same set -> skip.
 * - Stop when all cells are merged into one set.
 *
 * Note: This algorithm always **connects all cells into a single path network**,
 * guaranteeing a **fully connected maze**.
 *
 * Note: There are no cycles, and the result is a **tree structure with many dead-ends**.
 *
 * ---
 *
 * ### Usage Requirements
 * - `maze_t` must be created using `maze_create_full()`.
 * - Both `width` and `height` must be **odd and at least 3**.
 *
 * ### Example
 * @code
 * maze_t* maze = maze_create_full(0, 0, 9, 9, MAZE_TYPE_KRUSKAL);
 * maze_make_kruskal(maze);
 * navgrid_add_maze(navgrid, maze);
 * @endcode
 *
 * After execution:
 * - Wall coordinates are stored in `maze->blocked`.
 * - `maze->type` is set to `MAZE_TYPE_KRUSKAL`.
 *
 * @param maze Pointer to the `maze_t` structure where the maze will be generated
 */
BYUL_API void maze_make_kruskal(maze_t* maze);


#ifdef __cplusplus
}
#endif

#endif // MAZE_H
