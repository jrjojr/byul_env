#ifndef ROUTE_CARVER_H
#define ROUTE_CARVER_H

#include "byul_common.h"
#include "navgrid.h"     // navgrid_t definition required
#include "coord.h"       // coord_t definition required

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Removes obstacles in a straight line from start -> 
 * goal within the given radius (range),
 *        carving a direct passage through a wide area.
 *
 * @param navgrid Target map
 * @param start   Start coordinate
 * @param goal    Goal coordinate
 * @param range   Radius around the line (
 * 0 = only coordinates, 1 or more includes surrounding cells)
 * @return Number of obstacles removed
 */
BYUL_API int route_carve_beam(navgrid_t* navgrid, 
    const coord_t* start, const coord_t* goal, int range);

/**
 * @brief Bombards and clears block cells within the given radius around 
 * a specified center coordinate.
 *        Used to forcefully open scattered obstacles or to secure space.
 *
 * @param navgrid Target map
 * @param center  Center coordinate
 * @param range   Explosion radius (0 = only this coordinate,
 *  1 or more includes surrounding cells)
 * @return Number of obstacles removed
 */
BYUL_API int route_carve_bomb(
    navgrid_t* navgrid, const coord_t* center, int range);

#ifdef __cplusplus
}
#endif

#endif // ROUTE_CARVER_H
