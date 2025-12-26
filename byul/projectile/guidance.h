#ifndef GUIDANCE_H
#define GUIDANCE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "byul_config.h"
#include "vec3.h"
#include "entity_dynamic.h"
#include "trajectory.h"

/**
 * @typedef guidance_func
 * @brief Projectile guidance function type
 *
 * Based on the current state of the projectile, time step, and user data,
 * stores a **normalized direction vector (unit vector)** into `out` and returns its pointer.
 * If `out` is NULL, the function uses an internal static buffer.
 *
 * @param[in]  entdyn     Pointer to the current projectile
 * @param[in]  dt         Time interval (seconds)
 * @param[in]  userdata   User-defined data pointer (e.g., target info)
 * @param[out] out        Calculated unit vector (if NULL, an internal static buffer is used)
 * @return A pointer to `out` or to the static buffer
 */
typedef const vec3_t* (*guidance_func)(
    const entity_dynamic_t* entdyn, float dt, void* userdata, vec3_t* out);

// ---------------------------------------------------------
// No Guidance (None)
// ---------------------------------------------------------

/**
 * @brief No guidance
 *
 * The projectile maintains its current velocity and direction
 * without performing any guidance calculation.
 * Always returns the vector (0,0,0).
 *
 * @param[in]  entdyn     Pointer to the current projectile
 * @param[in]  dt         Time interval (seconds)
 * @param[in]  userdata   Not used (NULL)
 * @param[out] out        Stores (0,0,0) vector (uses static buffer if NULL)
 * @return A pointer to `out` or to the static buffer
 */
BYUL_API const vec3_t* guidance_none(
    const entity_dynamic_t* entdyn, float dt, void* userdata, vec3_t* out);

// ---------------------------------------------------------
// Linear Guidance
// ---------------------------------------------------------

/**
 * @brief Static target guidance
 *
 * Calculates a normalized direction vector pointing toward
 * a **fixed target position**.
 * `userdata` must be a pointer of type `const vec3_t*`.
 *
 * @param[in]  entdyn     Pointer to the current projectile
 * @param[in]  dt         Time interval (seconds)
 * @param[in]  userdata   `const vec3_t*` (target position)
 * @param[out] out        Calculated unit vector (uses static buffer if NULL)
 * @return A pointer to `out` or to the static buffer
 */
BYUL_API const vec3_t* guidance_point(
    const entity_dynamic_t* entdyn, float dt, void* userdata, vec3_t* out);

/**
 * @brief Moving target lead guidance
 *
 * Predicts a future intercept point based on the **target's current position and velocity**,
 * and calculates a normalized direction vector toward that point.
 * `userdata` must be a pointer of type `const entity_dynamic_t*`.
 *
 * @param[in]  entdyn     Pointer to the current projectile
 * @param[in]  dt         Time interval (seconds)
 * @param[in]  userdata   `const entity_dynamic_t*` (moving target info)
 * @param[out] out        Calculated unit vector (uses static buffer if NULL)
 * @return A pointer to `out` or to the static buffer
 */
BYUL_API const vec3_t* guidance_lead(
    const entity_dynamic_t* entdyn, float dt, void* userdata, vec3_t* out);

/**
 * @struct guidance_target_info_t
 * @brief Target information structure for guidance system
 *
 * This structure contains the target's state and environmental data
 * that are passed to the guidance function.
 * - Target's real-time dynamic state (`entity_dynamic_t`)
 * - External environment factors (`environ_t`)
 * - Reference prediction time (`current_time`)
 */
typedef struct s_guidance_target_info {
    entity_dynamic_t target;   /**< Target entity to track */
    environ_t env;            /**< Environmental information (gravity, wind, drag, etc.) */
    float current_time;        /**< Reference prediction time (seconds) */
} guidance_target_info_t;

// ---------------------------------------------------------
// Nonlinear Guidance
// ---------------------------------------------------------

/**
 * @brief Projectile guidance function (equation-based prediction)
 *
 * Calculates the intercept point based on the target's current position and velocity,
 * returning a unit direction vector that the projectile should follow.
 *
 * Internally, a simple lead time prediction is used to solve the condition:
 * `missile_position + missile_velocity * t = target_position + target_velocity * t`
 * to find the intersection direction.
 *
 * @param[in]  entdyn     Pointer to the current projectile (returns (0,0,0) if NULL)
 * @param[in]  dt         Time interval (seconds)
 * @param[in]  userdata   User data (expects `guidance_target_info_t*`)
 * @param[out] out        Calculated unit direction vector (uses static buffer if NULL)
 *
 * @return Pointer to the normalized direction vector (`out` or static)
 */
BYUL_API const vec3_t* guidance_predict(
    const entity_dynamic_t* entdyn,
    float dt,
    void* userdata,
    vec3_t* out);

/**
 * @brief Projectile guidance function (acceleration-based prediction, Cardano-based)
 *
 * Uses the target's position, velocity, and acceleration (`guidance_target_info_t`)
 * to solve a **cubic equation (Cardano's method)** to compute the intercept time,
 * and returns a unit vector toward the predicted intercept point.
 *
 * - If the target's acceleration is zero, it automatically falls back to a quadratic solution.
 * - Environmental effects such as gravity, wind, and drag (from `environ_t`)
 *   are added to `target_acc` to improve prediction accuracy.
 *
 * @param[in]  entdyn     Pointer to the current projectile
 * @param[in]  dt         Time interval (seconds)
 * @param[in]  userdata   User data (expects `guidance_target_info_t*`)
 * @param[out] out        Calculated unit direction vector (uses static buffer if NULL)
 *
 * @return Pointer to the normalized direction vector
 */
BYUL_API const vec3_t* guidance_predict_accel(
    const entity_dynamic_t* entdyn,
    float dt,
    void* userdata,
    vec3_t* out);

/**
 * @brief Projectile guidance function (acceleration + environment + entity state-based)
 *
 * This function retrieves the target's **accurate position/velocity** from `entity_dynamic_t`
 * and **external environmental effects (gravity, wind, drag, etc.)** from `environ_t`,
 * performs **Cardano-based intercept time calculation**, and returns the optimal intercept direction.
 *
 * Key features:
 * - Considers both target's linear motion (velocity) and external acceleration
 * - Can incorporate air resistance and wind effects using environment info and bodyprops
 * - Returns (0,0,0) for stability when the projectile is very close to the target
 *
 * @param[in]  entdyn     Pointer to the current projectile
 * @param[in]  dt         Time interval (seconds)
 * @param[in]  userdata   User data (expects `guidance_target_info_t*`)
 * @param[out] out        Calculated unit direction vector (uses static buffer if NULL)
 *
 * @return Pointer to the normalized direction vector
 */
BYUL_API const vec3_t* guidance_predict_accel_env(
    const entity_dynamic_t* entdyn,
    float dt,
    void* userdata,
    vec3_t* out);

#ifdef __cplusplus
}
#endif    

#endif // GUIDANCE_H
