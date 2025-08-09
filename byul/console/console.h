#ifndef CONSOLE_H
#define CONSOLE_H

#include "byul_common.h"
#include "float_common.h"

#include "navgrid.h"
#include "coord.h"
#include "route.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Print the map in ASCII format.
 *
 * Blocked coordinates are printed as '#', and other cells as '.'.
 * Path, start, and goal points are not displayed.
 *
 * @param m Map to print.
 */
BYUL_API void navgrid_print_ascii(const navgrid_t* m);

/**
 * @brief Print the map in ASCII format including route information.
 *
 * Path cells are marked with '*', the start point with 'S', and the goal point with 'E'.
 * Blocked coordinates are printed as '#', and the rest as '.'.
 *
 * @param m Map object.
 * @param p Route object (only displayed if route_get_success(p) is TRUE).
 */
BYUL_API void navgrid_print_ascii_with_route(
    const navgrid_t* m, const route_t* p, int margin);

/**
 * @brief Print the visit count in ASCII map format.
 *
 * Based on the debug_mode_enabled hash table inside route_t,
 * each coordinate's visit count is printed as a 2-digit number between 1 and 99.
 *
 * Output format:
 * - Start coordinate: " S"
 * - Goal coordinate: " E"
 * - Obstacle: "#"
 * - Visited coordinate: "%2d" (two-digit number, capped at 99)
 * - Unvisited coordinate: " ."
 *
 * @param m Map object.
 * @param p Route result object (uses route_get_visited_count()).
 */
BYUL_API void navgrid_print_ascii_with_visited_count(
    const navgrid_t* m, const route_t* p, int margin);

#ifdef __cplusplus
}
#endif

#endif // CONSOLE_H
