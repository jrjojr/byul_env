#ifndef ENTITY_ENCIRCLEMENT_H
#define ENTITY_ENCIRCLEMENT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "entity_dynamic.h"

// ---------------------------------------------------------
// Basic Utilities
// ---------------------------------------------------------

/**
 * @brief Generate candidate encirclement positions around an enemy.
 *
 * - Divide 360 degrees into ally_count segments to create circular points.
 * - Ignore Z axis (assume 2D plane).
 *
 * @param target        Position of the enemy.
 * @param ally_count    Number of allies.
 * @param L0            Encirclement radius.
 * @param out_positions Output array of positions (size ally_count).
 */
void entity_encirclement_generate_ring_positions(
    const vec3_t* target,
    int ally_count,
    float L0,
    vec3_t* out_positions);

/**
 * @brief Check if a given spot is already occupied by an ally.
 *
 * @param allies        Array of ally entities.
 * @param ally_count    Number of allies.
 * @param position      Encirclement position to check.
 * @param threshold     Distance threshold to consider occupied.
 * @return true         If a nearby ally is occupying this position.
 */
bool entity_encirclement_is_spot_occupied(
    const entity_dynamic_t* allies,
    int ally_count,
    const vec3_t* position,
    float threshold);

/**
 * @brief Assign the closest available target position to an ally.
 *
 * - Returns the index of the selected target position in out_index.
 *
 * @param ally_pos      Current position of the ally.
 * @param targets       Array of encirclement target positions.
 * @param target_count  Number of target positions.
 * @param used_flags    Usage flags array (0=unused, 1=used).
 * @return              Index of selected target (or -1 if none).
 */
int entity_encirclement_find_closest_available_target(
    const vec3_t* ally_pos,
    const vec3_t* targets,
    int target_count,
    int* used_flags);

// ---------------------------------------------------------
// Encirclement Target Management
// ---------------------------------------------------------

/**
 * @brief Compute final encirclement targets for allies around the enemy.
 *
 * - Generate ring positions.
 * - Exclude spots already occupied by allies.
 * - Assign remaining target positions to allies based on distance.
 *
 * @param allies        Array of ally entities.
 * @param ally_count    Number of allies.
 * @param target        Enemy position.
 * @param L0            Encirclement radius.
 * @param out_positions Output target positions for each ally.
 */
void entity_encirclement_find_targets(
    const entity_dynamic_t* allies,
    int ally_count,
    const vec3_t* target,
    float L0,
    vec3_t* out_positions);

// ---------------------------------------------------------
// Control Point Calculation
// ---------------------------------------------------------
/**
 * @brief Calculate curve control points (P1, P2) for encirclement movement.
 *
 * @param current_pos   Current position (P0).
 * @param target_pos    Target position (P3).
 * @param velocity      Current velocity vector.
 * @param out_p1        Computed control point P1.
 * @param out_p2        Computed control point P2.
 */
void entity_encirclement_calc_control_points(
    const vec3_t* current_pos,
    const vec3_t* target_pos,
    const vec3_t* velocity,
    vec3_t* out_p1,
    vec3_t* out_p2);

#ifdef __cplusplus
}
#endif

#endif // ENTITY_ENCIRCLEMENT_H
