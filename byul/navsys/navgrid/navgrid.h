#ifndef NAVGRID_H
#define NAVGRID_H

#include "byul_common.h"
#include "coord.h"
#include "coord_list.h"
#include "coord_hash.h"
#include "navcell.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Function pointer to check if a coordinate is blocked.
 *
 * This function determines whether a specific coordinate `(x, y)`
 * is an impassable cell for pathfinding or range operations.
 *
 * @param context External data required for coordinate checking (
 * e.g., navgrid, map)
 * @param x X coordinate to check
 * @param y Y coordinate to check
 * @param userdata Optional user-defined data
 * @return true - The coordinate is blocked  
 *         false - The coordinate is passable
 */
typedef bool (*is_coord_blocked_func)(
    const void* context, int x, int y, void* userdata);

/**
 * @brief Checks if a coordinate is blocked based on navgrid.
 *
 * Determines whether a coordinate is impassable due to walls or obstacles
 * using the internal cell information of the navgrid.
 *
 * @param context Pointer to navgrid object (const navgrid_t*)
 * @param x X coordinate to check
 * @param y Y coordinate to check
 * @param userdata Optional user-defined data (can be unused)
 * @return true - The coordinate is blocked  
 *         false - The coordinate is passable
 */
BYUL_API bool is_coord_blocked_navgrid(
    const void* context, int x, int y, void* userdata);

typedef enum {
    NAVGRID_DIR_4,
    NAVGRID_DIR_8
} navgrid_dir_mode_t;

struct s_navgrid {
    int width;
    int height;
    navgrid_dir_mode_t mode;

    coord_hash_t* cell_map;           // coord -> navcell_t*

    is_coord_blocked_func is_coord_blocked_fn;
    void* is_coord_blocked_fn_userdata;
};

typedef struct s_navgrid navgrid_t;

// Constructors and Destructors

/**
 * @brief Creates a navigation grid with default parameters.
 *
 * This function initializes a `navgrid_t` object with 
 * the following default settings:
 * - Grid size: `0 × 0` (interpreted as **infinite** width and height)
 * - Direction mode: `NAVGRID_DIR_8` (8-way movement)
 * - No obstacle-checking function (all coordinates considered traversable)
 *
 * A size of `0 × 0` indicates an **unbounded grid**, 
 * which may be useful for procedural or open-world environments.
 * However, infinite grids can lead to unbounded node expansion 
 * in some pathfinding algorithms.
 *
 * @warning Algorithms without heuristic guidance 
 * (e.g., BFS, DFS, Fringe Search) may enter infinite exploration
 *          on an unbounded grid unless a retry limit is enforced. 
 *          The system uses `MAX_RETRY` as a safeguard (default: 1000).
 *
 * @return Pointer to a newly allocated `navgrid_t` instance, 
 *          or `NULL` on failure.
 */
BYUL_API navgrid_t* navgrid_create();

/**
 * @brief Creates a new navigation grid with custom dimensions and settings.
 *
 * This function allocates and initializes a `navgrid_t` object 
 * using the specified
 * width, height, and direction mode. 
 * It also allows the caller to optionally provide
 * a custom function to determine whether a given coordinate is blocked.
 *
 * @param width               Grid width (number of columns)
 * @param height              Grid height (number of rows)
 * @param mode                Directional mode (NAVGRID_DIR_4 or NAVGRID_DIR_8)
 * @param is_coord_blocked_fn Optional user-defined function to determine 
 *      if a coordinate is blocked.
 *      If NULL, all coordinates are assumed walkable by default.
 *
 * @return A pointer to the newly created navgrid_t object, 
 * or NULL on allocation failure.
 */
BYUL_API navgrid_t* navgrid_create_full(int width, int height, 
    navgrid_dir_mode_t mode,
    is_coord_blocked_func is_coord_blocked_fn);

BYUL_API void navgrid_destroy(navgrid_t* navgrid);

// Copy and Comparison
BYUL_API navgrid_t* navgrid_copy(const navgrid_t* navgrid);
BYUL_API uint32_t navgrid_hash(const navgrid_t* navgrid);
BYUL_API bool navgrid_equal(const navgrid_t* a, const navgrid_t* b);

// Property Access
BYUL_API int navgrid_get_width(const navgrid_t* navgrid);
BYUL_API void navgrid_set_width(navgrid_t* navgrid, int width);

BYUL_API int navgrid_get_height(const navgrid_t* navgrid);
BYUL_API void navgrid_set_height(navgrid_t* navgrid, int height);

BYUL_API void navgrid_set_is_coord_blocked_func(
    navgrid_t* navgrid, is_coord_blocked_func fn);

BYUL_API is_coord_blocked_func navgrid_get_is_coord_blocked_fn(
    const navgrid_t* navgrid);

BYUL_API navgrid_dir_mode_t navgrid_get_mode(const navgrid_t* navgrid);
BYUL_API void navgrid_set_mode(navgrid_t* navgrid);

// Obstacle Management
BYUL_API bool navgrid_block_coord(navgrid_t* navgrid, int x, int y);
BYUL_API bool navgrid_unblock_coord(navgrid_t* navgrid, int x, int y);
BYUL_API bool navgrid_is_inside(const navgrid_t* navgrid, int x, int y);
// BYUL_API bool navgrid_is_blocked(const navgrid_t* navgrid, int x, int y);
BYUL_API void navgrid_clear(navgrid_t* navgrid);

// Cell Map Access
BYUL_API bool navgrid_set_cell(
    navgrid_t* navgrid, int x, int y, const navcell_t* cell);

BYUL_API int navgrid_fetch_cell(
    const navgrid_t* navgrid, int x, int y, navcell_t* out);

BYUL_API const coord_hash_t* navgrid_get_cell_map(const navgrid_t* navgrid);

// Neighbor Search
BYUL_API coord_list_t* navgrid_copy_neighbors(
    const navgrid_t* navgrid, int x, int y);

BYUL_API coord_list_t* navgrid_copy_neighbors_all(
    const navgrid_t* navgrid, int x, int y);

// If max_range is 0, 
// it behaves the same as navgrid_copy_neighbors_all (only checks neighbors)
BYUL_API coord_list_t* navgrid_copy_neighbors_all_range(
    navgrid_t* navgrid, int x, int y, int range);

BYUL_API coord_t* navgrid_copy_neighbor_at_degree(const navgrid_t* navgrid, 
    int x, int y, double degree);
    
BYUL_API coord_t* navgrid_copy_neighbor_at_goal(const navgrid_t* navgrid, 
    const coord_t* center, const coord_t* goal);

BYUL_API coord_list_t* navgrid_copy_neighbors_at_degree_range(
    const navgrid_t* navgrid,
    const coord_t* center, const coord_t* goal,
    double start_deg, double end_deg,
    int range);

#ifdef __cplusplus
}
#endif

#endif // NAVGRID_H
