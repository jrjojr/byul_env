#ifndef OBSTACLE_CORE_H
#define OBSTACLE_CORE_H

#include "byul_config.h"
#include "coord.h"
#include "coord_hash.h"
#include "navgrid.h"

#ifdef __cplusplus
extern "C" {
#endif

// obstacle structure definition
typedef struct s_obstacle {
    int x0;
    int y0;
    int width;
    int height;
    coord_hash_t* blocked;
} obstacle_t;

// Basic constructors / destructors
BYUL_API obstacle_t* obstacle_create();
BYUL_API obstacle_t* obstacle_create_full(
    int x0, int y0, int width, int height);
BYUL_API void obstacle_destroy(obstacle_t* obstacle);

BYUL_API void obstacle_clear(obstacle_t* obstacle);

// Copy and comparison
BYUL_API obstacle_t* obstacle_copy(const obstacle_t* obstacle);
BYUL_API bool obstacle_equal(const obstacle_t* a, const obstacle_t* b);
BYUL_API uint32_t obstacle_hash(const obstacle_t* obstacle);

// Origin set / fetch
BYUL_API void obstacle_set_origin(obstacle_t* obstacle, int x0, int y0);
BYUL_API void obstacle_fetch_origin(
    const obstacle_t* obstacle, int* out_x0, int* out_y0);

// Property access
BYUL_API int obstacle_get_width(const obstacle_t* m);
BYUL_API void obstacle_set_width(obstacle_t* m, int width);

BYUL_API int obstacle_get_height(const obstacle_t* m);
BYUL_API void obstacle_set_height(obstacle_t* m, int height);

// Direct access to blocked coordinates (read-only)
BYUL_API const coord_hash_t* obstacle_get_blocked_coords(
    const obstacle_t* obstacle);

BYUL_API bool obstacle_block_coord(obstacle_t* m, int x, int y);
BYUL_API bool obstacle_unblock_coord(obstacle_t* m, int x, int y);
BYUL_API bool obstacle_is_inside(const obstacle_t* m, int x, int y);
// BYUL_API bool obstacle_is_blocked(const obstacle_t* m, int x, int y);
BYUL_API void obstacle_clear(obstacle_t* m);

// Neighbor search
BYUL_API coord_list_t* obstacle_clone_neighbors(
    const obstacle_t* m, int x, int y);

BYUL_API coord_list_t* obstacle_clone_neighbors_all(
    const obstacle_t* m, int x, int y);

// If max_range is 0, behaves the same as obstacle_clone_neighbors_all
BYUL_API coord_list_t* obstacle_clone_neighbors_all_range(
    obstacle_t* m, int x, int y, int range);

BYUL_API coord_t* obstacle_clone_neighbor_at_degree(const obstacle_t* m, 
    int x, int y, double degree);
    
BYUL_API coord_t* obstacle_clone_neighbor_at_goal(const obstacle_t* m, 
    const coord_t* center, const coord_t* goal);

BYUL_API coord_list_t* obstacle_clone_neighbors_at_degree_range(
    const obstacle_t* m,
    const coord_t* center, const coord_t* goal,
    double start_deg, double end_deg,
    int range);

bool obstacle_is_coord_blocked(const obstacle_t* obstacle, int x, int y);

BYUL_API void obstacle_apply_to_navgrid(const obstacle_t* obstacle, navgrid_t* navgrid);

BYUL_API void obstacle_remove_from_navgrid(
    const obstacle_t* obstacle, navgrid_t* navgrid);

/**
 * @brief Blocks all coordinates within the range around a given center coordinate.
 *
 * This function blocks all cells within a square region defined by `(x, y)`
 * as the center and `range` as the radius. A total of (2×range+1)^2 cells are blocked.
 *
 * @param obs    Obstacle object
 * @param x      Center x coordinate
 * @param y      Center y coordinate
 * @param range  Radius (0 means block only the center coordinate)
 */
BYUL_API void obstacle_block_range(obstacle_t* obs, int x, int y, int range);

// Blocks along a straight line to the target with a radius range.
// range == 0 means only the coordinate itself; >=1 means surrounding cells are also blocked.
BYUL_API void obstacle_block_straight(obstacle_t* obs, 
    int x0, int y0, int x1, int y1, int range);

#ifdef __cplusplus
}
#endif

#endif // OBSTACLE_CORE_H
