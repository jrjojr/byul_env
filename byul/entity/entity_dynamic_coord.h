#ifndef ENTITY_DYNAMIC_COORD_H
#define ENTITY_DYNAMIC_COORD_H

#ifdef __cplusplus
extern "C" {
#endif

#include "entity_dynamic.h"

/**
 * @brief Calculate the absolute coordinate (coord_t) by adding
 *        the relative position of xform to the current coord.
 *
 * absolute_coord = base.coord + round(xform.translation)
 *
 * @param[in]  ed   Reference entity (contains coord and xform)
 * @param[out] out  Calculated absolute coordinate
 *
 * @note Wrap-around is automatically handled in coord operations.
 */
BYUL_API void entity_dynamic_get_world_coord(
    const entity_dynamic_t* ed, coord_t* out);

/**
 * @brief Commit xform translation to coord.
 *
 * If xform.translation accumulates by at least 1 cell, it is
 * reflected in coord, and xform.translation keeps the remaining fraction.
 *
 * @note If coord exceeds COORD_MAX or COORD_MIN, wrap-around is applied.
 */
BYUL_API void entity_dynamic_commit_coord(entity_dynamic_t* ed);

/**
 * @brief Calculate the distance between two entities based on their coords.
 *
 * @return Distance (float), returns INFINITY if the range is exceeded.
 * @note If coordinate difference exceeds COORD_MAX or COORD_MIN,
 *       calculation is aborted with wrap consideration.
 */
BYUL_API float entity_dynamic_coord_distance(
    const entity_dynamic_t* a, const entity_dynamic_t* b);

/**
 * @brief Check if the distance between two entity coords is within valid range.
 *
 * @return true = within range, false = out of range.
 */
BYUL_API bool entity_dynamic_coord_in_range(
    const entity_dynamic_t* a, const entity_dynamic_t* b);

#ifdef __cplusplus
}
#endif

#endif // ENTITY_DYNAMIC_COORD_H
