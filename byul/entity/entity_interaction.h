#ifndef ENTITY_INTERACTION_H
#define ENTITY_INTERACTION_H

#ifdef __cplusplus
extern "C" {
#endif

#include "entity_dynamic.h"
#include "vec3.h"
#include <stdbool.h>

/**
 * @file entity_interaction.h
 * @brief Defines interactions between dynamic entities:
 *        - Force application
 *        - Collision detection
 *        - State/Emotion transfer
 *        - Area of Effect (AoE) effects
 *        - Line of Sight and Field of View checks
 *        - Real-time interaction updates (polling-based)
 */

// ---------------------------------------------------------
// Force-Based Interactions
// ---------------------------------------------------------

/**
 * @brief Apply a force vector to a dynamic entity.
 *
 * This function simulates Newton's Second Law by applying a force over
 * a time interval `dt`, resulting in a change in velocity.
 *
 * @param target Target dynamic entity to receive the force
 * @param force  Force vector to apply (in world coordinates)
 * @param dt     Time step in seconds
 */
BYUL_API void entity_interact_apply_force(
    entity_dynamic_t* target,
    const vec3_t* force,
    float dt);

// ---------------------------------------------------------
// Collision Interactions
// ---------------------------------------------------------

/**
 * @brief Check if two dynamic entities collide.
 *
 * Collision occurs if the Euclidean distance between their positions
 * is less than the specified collision radius.
 *
 * @param a First entity
 * @param b Second entity
 * @param collision_radius Threshold distance for collision
 * @return true if collision occurs
 */
BYUL_API bool entity_interact_check_collision(
    const entity_dynamic_t* a,
    const entity_dynamic_t* b,
    float collision_radius);

/**
 * @brief Resolve elastic collision between two dynamic entities.
 *
 * Exchanges momentum between entities assuming elastic behavior.
 * Velocity is updated based on relative mass and directions.
 *
 * @param a First entity (modified)
 * @param b Second entity (modified)
 */
BYUL_API void entity_interact_resolve_bounce(
    entity_dynamic_t* a,
    entity_dynamic_t* b);

// ---------------------------------------------------------
// Area of Effect (AoE) Utilities
// ---------------------------------------------------------

/**
 * @brief Check if a dynamic entity is within a circular AoE range.
 *
 * Useful for explosion effects, buffs, or debuffs applied over area.
 *
 * @param target Entity to check
 * @param origin Center of AoE
 * @param radius Radius of effect (in meters)
 * @return true if target is within AoE
 */
BYUL_API bool entity_interact_within_aoe(
    const entity_dynamic_t* target,
    const vec3_t* origin,
    float radius);

// ---------------------------------------------------------
// Line of Sight / Field of View (FOV)
// ---------------------------------------------------------

/**
 * @brief Check if `target` is within the `observer`'s FOV cone.
 *
 * Computes angle between observer's forward vector and direction to target.
 * Useful for vision cones, stealth mechanics, or targeting logic.
 *
 * @param observer     Observer entity
 * @param target       Target entity
 * @param forward      Observer's forward unit vector
 * @param fov_angle_deg Half-angle of the FOV cone (degrees)
 * @return true if target is within field of view
 */
BYUL_API bool entity_interact_check_fov(
    const entity_dynamic_t* observer,
    const entity_dynamic_t* target,
    const vec3_t* forward,
    float fov_angle_deg);

// ---------------------------------------------------------
// Real-Time Interaction Update
// ---------------------------------------------------------

/**
 * @brief Real-time polling update function for interactions.
 *
 * Called every frame to simulate continuous interaction logic.
 * Processes proximity checks, FOV status, and decay over `dt`.
 *
 * @param self The current dynamic entity
 * @param others Array of other entities to check against
 * @param count Number of other entities
 * @param dt Time delta (in seconds)
 */
BYUL_API void entity_interact_update(
    entity_dynamic_t* self,
    entity_dynamic_t** others,
    int count,
    float dt);

/**
 * @brief Calculate Euclidean distance between two dynamic entities.
 *
 * @param a First dynamic entity
 * @param b Second dynamic entity
 * @return Distance in world space between entity positions
 */
BYUL_API float entity_dynamic_distance(
    const entity_dynamic_t* a, const entity_dynamic_t* b);

/**
 * @brief Check if two dynamic entities are in contact.
 *
 * Contact is determined by whether the distance between their centers
 * is less than the sum of their influence radii plus a given tolerance.
 *
 * @param a First entity
 * @param b Second entity
 * @param tolerance Margin of error (optional gap allowance)
 * @return true if entities are overlapping or touching
 */
BYUL_API bool entity_dynamic_in_contact(
    const entity_dynamic_t* a, const entity_dynamic_t* b, float tolerance);

/**
 * @brief Predict the time at which two dynamic entities will come closest.
 *
 * Assumes linear motion and computes the time `t` where their distance is minimized.
 * If relative velocity is zero or `t <= 0`, returns 0.
 *
 * @param a First entity
 * @param b Second entity
 * @return Estimated time in seconds until minimum separation (or 0)
 */
BYUL_API float entity_dynamic_predict_collision_time(
    const entity_dynamic_t* a,
    const entity_dynamic_t* b);

/**
 * @brief Estimate the collision point between two dynamic entities.
 *
 * Computes predicted positions at estimated collision time,
 * then returns their midpoint as an approximate collision location.
 *
 * @param[out] out Collision point (world coordinates)
 * @param[in]  a   First entity
 * @param[in]  b   Second entity
 * @return 1 if valid point computed, 0 otherwise
 */
BYUL_API int entity_dynamic_collision_point(
    vec3_t* out,
    const entity_dynamic_t* a,
    const entity_dynamic_t* b);

/**
 * @brief Predict collision time considering environmental acceleration.
 *
 * Incorporates gravity and wind from `env` into motion estimation.
 * Uses quadratic approximation if needed.
 *
 * @param a First entity
 * @param b Second entity
 * @param env Environment information (gravity, wind, etc.)
 * @return Estimated time until collision (0 if not predictable)
 */
BYUL_API float entity_dynamic_predict_collision_time_env(
    const entity_dynamic_t* a,
    const entity_dynamic_t* b,
    const environ_t* env);

/**
 * @brief Estimate the collision point under environment effects.
 *
 * Applies `a` and `b`'s predicted trajectories with environment acceleration,
 * then estimates the midpoint at closest point of approach.
 *
 * @param[out] out Collision point (world coordinates)
 * @param[in]  a   First entity
 * @param[in]  b   Second entity
 * @param[in]  env Environment info (gravity, wind)
 * @return 1 if collision point is estimated, 0 otherwise
 */
BYUL_API int entity_dynamic_collision_point_env(
    vec3_t* out,
    const entity_dynamic_t* a,
    const entity_dynamic_t* b,
    const environ_t* env);

#ifdef __cplusplus
}
#endif

#endif // ENTITY_INTERACTION_H
