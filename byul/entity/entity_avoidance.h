#ifndef ENTITY_AVOIDANCE_H
#define ENTITY_AVOIDANCE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "entity_dynamic.h"  // Reference to dynamic entity structure

// ---------------------------------------------------------
// Boundary structure for avoidance
// ---------------------------------------------------------
/**
 * @struct avoidance_boundary_t
 * @brief Defines a region that an entity should avoid.
 *
 * - center: The center position of the boundary
 * - radius: The boundary radius
 */
typedef struct s_avoidance_boundary {
    vec3_t center;
    float radius;
} avoidance_boundary_t;

// ---------------------------------------------------------
// Collision prediction and avoidance vector calculation
// ---------------------------------------------------------
/**
 * @brief Determine if avoidance is needed between two entities.
 */
bool entity_avoidance_need(
    const entity_dynamic_t* self,
    const entity_dynamic_t* other,
    float safe_dist);

/**
 * @brief Calculate the avoidance direction vector.
 */
vec3_t entity_avoidance_direction(
    const entity_dynamic_t* self,
    const entity_dynamic_t* other,
    float safe_dist,
    float k);

/**
 * @brief Predict potential collision between self and other and compute an avoidance vector.
 *
 * @param self      The entity to evaluate
 * @param other     The other entity
 * @param safe_dist Safe distance (if within this range, avoidance is required)
 * @param k         Avoidance intensity coefficient
 * @return          Avoidance vector (returns (0,0,0) if no avoidance is needed)
 */
vec3_t entity_avoidance_calc_single(
    const entity_dynamic_t* self,
    const entity_dynamic_t* other,
    float safe_dist,
    float k);

/**
 * @brief Compute the total avoidance vector from multiple entities.
 *
 * @param self      The target entity
 * @param others    Array of other entities
 * @param count     Number of entities in the others array
 * @param safe_dist Safe distance
 * @param k         Avoidance intensity coefficient
 * @return          Total avoidance vector
 */
vec3_t entity_avoidance_calc_multi(
    const entity_dynamic_t* self,
    const entity_dynamic_t* others,
    int count,
    float safe_dist,
    float k);

// ---------------------------------------------------------
// Trajectory record structure
// ---------------------------------------------------------
#define AVOIDANCE_MAX_STEP 256

/**
 * @struct entity_avoidance_traj_t
 * @brief Records the avoidance movement path.
 *
 * - path: Array of positions
 * - count: Number of recorded steps
 */
typedef struct s_entity_avoidance_traj {
    vec3_t path[AVOIDANCE_MAX_STEP];
    int count;
} entity_avoidance_traj_t;

// ---------------------------------------------------------
// Avoidance simulation
// ---------------------------------------------------------
/**
 * @brief Generate a path for an entity to move toward a target while avoiding other entities.
 *
 * @param e          Target entity
 * @param others     Array of other entities
 * @param count      Number of entities in the others array
 * @param traj       Optional trajectory record (can be NULL)
 * @param boundary   Optional avoidance boundary (NULL means unlimited)
 * @param dt         Time step (seconds)
 * @param safe_dist  Safe distance
 * @param k          Avoidance intensity coefficient
 * @param steps      Number of simulation steps
 */
void entity_avoidance_auto(
    entity_dynamic_t* e,
    const entity_dynamic_t* others,
    int count,
    entity_avoidance_traj_t* traj,
    const avoidance_boundary_t* boundary,
    float dt, float safe_dist, float k, int steps);

#ifdef __cplusplus
}
#endif

#endif // ENTITY_AVOIDANCE_H
