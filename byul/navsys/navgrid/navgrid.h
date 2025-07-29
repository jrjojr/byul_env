#ifndef NAVGRID_H
#define NAVGRID_H

#include "byul_common.h"
#include "coord.h"
#include "coord_list.h"
#include "coord_hash.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Function pointer to check if a coordinate is blocked.
 *
 * This function determines whether a specific coordinate `(x, y)`
 * is an impassable cell for pathfinding or range operations.
 *
 * @param context External data required for coordinate checking (e.g., navgrid, map)
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
bool is_coord_blocked_navgrid(
    const void* context, int x, int y, void* userdata);

typedef enum {
    NAVGRID_DIR_4,
    NAVGRID_DIR_8
} navgrid_dir_mode_t;

struct s_navgrid {
    int width;
    int height;
    navgrid_dir_mode_t mode;

    coord_hash_t* blocked_coords;

    is_coord_blocked_func is_coord_blocked_fn;
};

typedef struct s_navgrid navgrid_t;

// Constructors and Destructors

// Default: 0 x 0 , NAVGRID_DIR_8
BYUL_API navgrid_t* navgrid_create();

BYUL_API navgrid_t* navgrid_create_full(int width, int height, navgrid_dir_mode_t mode,
    is_coord_blocked_func is_coord_blocked_fn);

BYUL_API void navgrid_destroy(navgrid_t* m);

// Copy and Comparison
BYUL_API navgrid_t* navgrid_copy(const navgrid_t* m);
BYUL_API uint32_t navgrid_hash(const navgrid_t* m);
BYUL_API bool navgrid_equal(const navgrid_t* a, const navgrid_t* b);

// Property Access
BYUL_API int navgrid_get_width(const navgrid_t* m);
BYUL_API void navgrid_set_width(navgrid_t* m, int width);

BYUL_API int navgrid_get_height(const navgrid_t* m);
BYUL_API void navgrid_set_height(navgrid_t* m, int height);

BYUL_API void navgrid_set_is_coord_blocked_func(navgrid_t* m, is_coord_blocked_func fn);
BYUL_API is_coord_blocked_func navgrid_get_is_coord_blocked_fn(const navgrid_t* m);

BYUL_API navgrid_dir_mode_t navgrid_get_mode(const navgrid_t* m);
BYUL_API void navgrid_set_mode(navgrid_t* m);

// Obstacle Management
BYUL_API bool navgrid_block_coord(navgrid_t* m, int x, int y);
BYUL_API bool navgrid_unblock_coord(navgrid_t* m, int x, int y);
BYUL_API bool navgrid_is_inside(const navgrid_t* m, int x, int y);
// BYUL_API bool navgrid_is_blocked(const navgrid_t* m, int x, int y);
BYUL_API void navgrid_clear(navgrid_t* m);

// Get blocked coordinates
BYUL_API const coord_hash_t* navgrid_get_blocked_coords(const navgrid_t* m);

// Neighbor Search
BYUL_API coord_list_t* navgrid_clone_adjacent(const navgrid_t* m, int x, int y);
BYUL_API coord_list_t* navgrid_clone_adjacent_all(const navgrid_t* m, int x, int y);

// If max_range is 0, it behaves the same as navgrid_clone_adjacent_all (only checks neighbors)
BYUL_API coord_list_t* navgrid_clone_adjacent_all_range(
    navgrid_t* m, int x, int y, int range);

BYUL_API coord_t* navgrid_clone_neighbor_at_degree(const navgrid_t* m, 
    int x, int y, double degree);
    
BYUL_API coord_t* navgrid_clone_neighbor_at_goal(const navgrid_t* m, 
    const coord_t* center, const coord_t* goal);

BYUL_API coord_list_t* navgrid_clone_adjacent_at_degree_range(
    const navgrid_t* m,
    const coord_t* center, const coord_t* goal,
    double start_deg, double end_deg,
    int range);

#ifdef __cplusplus
}
#endif

#endif // NAVGRID_H
