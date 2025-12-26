#ifndef OBSTACLE_H
#define OBSTACLE_H

#include "byul_config.h"
#include "obstacle_core.h"

#ifdef __cplusplus
extern "C" {
#endif

BYUL_API obstacle_t* obstacle_make_rect_all_blocked(
    int x0, int y0, int width, int height);

// ratio range is 0.0 ~ 1.0; 1.0 means fully blocked
BYUL_API obstacle_t* obstacle_make_rect_random_blocked(
    int x0, int y0, int width, int height, float ratio);

// range 0 : only the given coordinate, 
// 1 or more includes surrounding area
// Creates obstacles along the virtual line from start to goal
BYUL_API obstacle_t* obstacle_make_beam(
    const coord_t* start, const coord_t* goal, int range);

/**
 * @brief Creates a torus (donut)-shaped obstacle.
 *
 * This function creates a ring-shaped (torus) obstacle within a rectangular
 * area, leaving the inner part empty while blocking the outer boundary.
 * The inner and outer areas are completely separated with no path.
 * 
 * @note Minimum size:
 *       - width >= thickness × 2 + 1
 *       - height >= thickness × 2 + 1
 *       If smaller, there will be no inner space to form a donut shape.
 *
 * @note If `thickness` is too large and no inner space can be formed,
 *       the function returns NULL.
 *
 * @param start      One corner coordinate of the rectangular area
 * @param goal       Opposite corner coordinate
 * @param thickness  Thickness of the boundary wall (>=1)
 *                   A larger value creates a thicker outer ring
 *
 * @return A pointer to the torus-shaped obstacle on success, or NULL on failure
 */
BYUL_API obstacle_t* obstacle_make_torus(
    const coord_t* start, const coord_t* goal, int thickness);

typedef enum e_enclosure_open_dir {
    ENCLOSURE_OPEN_UNKNOWN,
    ENCLOSURE_OPEN_RIGHT,
    ENCLOSURE_OPEN_UP,
    ENCLOSURE_OPEN_LEFT,
    ENCLOSURE_OPEN_DOWN,
} enclosure_open_dir_t;

/**
 * @brief Creates a rectangular enclosure obstacle with one open side.
 *
 * This function creates a "pot" or "U-shaped" obstacle by leaving one of
 * the four sides of a rectangular boundary open. Useful for creating 
 * structures where a player or NPC can enter or exit.
 *
 * @note Minimum size:
 *       - width >= thickness × 2 + 1
 *       - height >= thickness × 2 + 1
 *       If there is no space for walls, creation may fail.
 * 
 * The `open` argument specifies the open direction.
 * If ENCLOSURE_OPEN_UNKNOWN is passed, all sides are closed.
 *
 * @param start      One corner coordinate of the rectangular area
 * @param goal       Opposite corner coordinate
 * @param thickness  Wall thickness (>=1)
 *                   Larger values create thicker walls
 * @param open       Direction to be left open (up, down, left, right)
 *
 * @return A pointer to the enclosure obstacle on success, or NULL on failure
 */
BYUL_API obstacle_t* obstacle_make_enclosure(
    const coord_t* start, const coord_t* goal, int thickness, 
    enclosure_open_dir_t open);

/**
 * @brief Creates a cross (+)-shaped obstacle centered at a given coordinate.
 *
 * This function creates a cross shape extending `length` units
 * from `center` in four directions. The thickness of each arm is 
 * defined by `range`.
 *
 * @param center    Center coordinate of the cross
 * @param length    Length of each arm (0 means only the center point)
 * @param range     Width of each arm (0 means only center, >=1 includes neighbors)
 *
 * @return A pointer to the created obstacle, or NULL on failure
 */
BYUL_API obstacle_t* obstacle_make_cross(
    const coord_t* center, int length, int range);

typedef enum e_spiral_dir {
    SPIRAL_CLOCKWISE,        ///< Clockwise (default)
    SPIRAL_COUNTER_CLOCKWISE ///< Counterclockwise
} spiral_dir_t;

/**
 * @brief Creates a spiral-shaped obstacle centered around a coordinate.
 *
 * This function creates a grid-based square spiral structure, blocking
 * cells along a path that rotates clockwise or counterclockwise.
 * The number of rotations (`turns`) determines the total spiral length.
 * Setting `gap` adds spacing between rotations to create open areas.
 *
 * `range` defines the thickness of blocked cells around the spiral path.
 *
 * @param center    Center coordinate of the spiral
 * @param radius    Maximum spiral radius (in grid distance)
 * @param turns     Total number of rotations (1 rotation = 4 directional turns)
 * @param range     Path radius (0 blocks only the path center, >=1 includes area)
 * @param gap       Spacing between rotations (0 means continuous)
 * @param direction Rotation direction (SPIRAL_CLOCKWISE or SPIRAL_COUNTER_CLOCKWISE)
 *
 * @return A pointer to the created obstacle, or NULL on failure
 *
 * @note Larger `gap` creates more spacing between rotations,
 *       larger `range` creates thicker obstacles.
 */
BYUL_API obstacle_t* obstacle_make_spiral(
    const coord_t* center,
    int radius,
    int turns,
    int range,
    int gap,
    spiral_dir_t direction
);

/**
 * @brief Creates a triangle-shaped obstacle that blocks the area inside.
 *
 * This function blocks the grid area defined by the three vertices `a`, `b`, `c`.
 *
 * @param a  First vertex
 * @param b  Second vertex
 * @param c  Third vertex
 *
 * @return A pointer to the created obstacle, or NULL on failure
 */
BYUL_API obstacle_t* obstacle_make_triangle(
    const coord_t* a,
    const coord_t* b,
    const coord_t* c);

/**
 * @brief Creates a triangle torus obstacle, blocking only the outer boundary.
 *
 * This function follows the edges of the triangle defined by `a`, `b`, `c`
 * and blocks the boundary line, leaving the interior unblocked.
 * If `thickness` >= 1, the boundary lines are thickened accordingly.
 *
 * @param a         Triangle vertex A
 * @param b         Triangle vertex B
 * @param c         Triangle vertex C
 * @param thickness Boundary line thickness (0 = line only, >=1 includes area)
 *
 * @return A pointer to the created obstacle, or NULL on failure
 */
BYUL_API obstacle_t* obstacle_make_triangle_torus(
    const coord_t* a,
    const coord_t* b,
    const coord_t* c,
    int thickness);

/**
 * @brief Creates a polygon-shaped obstacle that blocks the inside area.
 *
 * This function connects the coordinates in the list to form a closed shape
 * and blocks the inside area. At least 3 coordinates are required.
 * The polygon is considered closed automatically (last -> first).
 *
 * @param list  Polygon vertex list (coord_list_t*)
 * @return A pointer to the created obstacle, or NULL on failure
 */
BYUL_API obstacle_t* obstacle_make_polygon(coord_list_t* list);

/**
 * @brief Creates a polygon torus obstacle, blocking only the boundary.
 *
 * This function connects the coordinates in the list to form a closed shape,
 * and blocks along the boundary lines only, leaving the inside unblocked.
 * If `thickness` >= 1, the boundary thickness can be expanded.
 *
 * @param list       Polygon vertex list (at least 3 points)
 * @param thickness  Boundary thickness (0 = line only, >=1 includes area)
 * @return A pointer to the created obstacle, or NULL on failure
 */
BYUL_API obstacle_t* obstacle_make_polygon_torus(
    coord_list_t* list, int thickness);

#ifdef __cplusplus
}
#endif

#endif // OBSTACLE_H
