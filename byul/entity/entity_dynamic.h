#ifndef ENTITY_DYNAMIC_H
#define ENTITY_DYNAMIC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "byul_common.h"
#include "entity.h"
#include "xform.h"
#include "bodyprops.h"
#include "environ.h"
#include "motion_state.h"

// Forward declaration
typedef struct s_motion_state motion_state_t;

// ---------------------------------------------------------
// Dynamic Entity Structure
// ---------------------------------------------------------
/**
 * @struct s_entity_dynamic
 * @brief Extended structure for moving entities.
 *
 * - Base entity info (entity_t).
 * - xform_t (precise position/rotation).
 * - Linear velocity, angular velocity.
 * - Physical properties (bodyprops_t).
 */
typedef struct s_entity_dynamic {
    entity_t base;            ///< Common entity properties
    xform_t xf;               ///< Precise position + rotation (Transform)
    bodyprops_t props;        ///< Physical properties (mass, friction, etc.)
    vec3_t velocity;          ///< Linear velocity (m/s)
    vec3_t angular_velocity;  ///< Angular velocity (rad/s)
    bool is_grounded;         ///< Grounded flag (true = Y-axis movement stopped)
} entity_dynamic_t;

// ---------------------------------------------------------
// Function Declarations
// ---------------------------------------------------------

/**
 * @brief Initialize entity_dynamic_t with default values.
 *
 * **Defaults**
 * - base = entity_init()
 * - xf = identity transform
 * - props = bodyprops_init()
 * - velocity = (0,0,0)
 * - angular_velocity = (0,0,0)
 */
BYUL_API void entity_dynamic_init(entity_dynamic_t* d);

/**
 * @brief Initialize entity_dynamic_t with user-specified values.
 *
 * @param[out] d         Dynamic entity to initialize
 * @param[in]  base      Base entity info (NULL = default)
 * @param[in]  xf        Initial position/rotation (NULL = default)
 * @param[in]  velocity  Initial linear velocity (NULL = (0,0,0))
 * @param[in]  angular   Initial angular velocity (NULL = (0,0,0))
 * @param[in]  props     Physical properties (NULL = default)
 */
BYUL_API void entity_dynamic_init_full(
    entity_dynamic_t* d,
    const entity_t* base,
    const xform_t* xf,
    const vec3_t* velocity,
    const vec3_t* angular,
    const bodyprops_t* props
);

/**
 * @brief Copy entity_dynamic_t data from src to dst.
 *
 * @param[out] dst Destination
 * @param[in]  src Source
 */
BYUL_API void entity_dynamic_assign(
    entity_dynamic_t* dst, const entity_dynamic_t* src);

/**
 * @brief Calculate average acceleration based on current and previous velocity.
 *
 * a = (v_curr - v_prev) / dt  
 * Returns (0,0,0) if dt <= 0.0f.
 *
 * @param curr      Current dynamic entity state
 * @param prev_vel  Previous velocity
 * @param dt        Time interval (seconds)
 * @param out_accel Calculated acceleration (m/s^2)
 */
BYUL_API void entity_dynamic_calc_accel(
    const entity_dynamic_t* curr,
    const vec3_t* prev_vel,
    float dt,
    vec3_t* out_accel
);

BYUL_API void entity_dynamic_calc_accel_env(
    const entity_dynamic_t* curr,
    const vec3_t* prev_vel,
    float dt,
    const environ_t* env,
    vec3_t* out_accel
);

/**
 * @brief Calculate drag acceleration for the entity.
 *
 * @param curr          Current dynamic entity state
 * @param prev_vel      Previous velocity
 * @param dt            Time interval (seconds)
 * @param env           Environment data (density, wind, etc.)
 * @param out_drag_accel Calculated drag acceleration (m/s^2)
 */
BYUL_API void entity_dynamic_calc_drag_accel(
    const entity_dynamic_t* curr,
    const vec3_t* prev_vel,
    float dt,
    const environ_t* env,
    vec3_t* out_drag_accel
);

/**
 * @brief Update position and rotation of a dynamic entity.
 *
 * - p = p + v * dt
 * - Apply rotation based on angular_velocity
 * - base.age += dt
 *
 * @param d   Dynamic entity 
 * @param dt  Time interval (seconds)
 */
BYUL_API void entity_dynamic_update(entity_dynamic_t* d, float dt);

/**
 * @brief Update position and rotation considering environment effects.
 *
 * - Position/velocity is integrated with gravity, drag, and wind.
 * - Rotation is updated based on angular_velocity.
 * - base.age += dt
 *
 * @param d    Dynamic entity
 * @param env  Environment data
 * @param dt   Time interval (seconds)
 */
BYUL_API void entity_dynamic_update_env(
    entity_dynamic_t* d,
    const environ_t* env,
    float dt);

// ---------------------------------------------------------
// Position Calculation: p(t) = p0 + v0 * dt
// ---------------------------------------------------------
/**
 * @brief Predict position after t seconds (constant velocity).
 *
 * Does NOT consider environment effects,  
 * applies only velocity (`d->velocity`) and friction (`d->props.friction`).
 * 
 * @note
 * - External forces like gravity or drag are not included.
 * - Suitable for basic NPC or object movement.
 *
 * @param d        Dynamic entity
 * @param dt       Time interval (seconds)
 * @param out_pos  Calculated position
 */
BYUL_API void entity_dynamic_calc_position(
    const entity_dynamic_t* d,
    float dt,
    vec3_t* out_pos);

// ---------------------------------------------------------
// Velocity Calculation: v(t) = v0 (no acceleration)
// ---------------------------------------------------------
/**
 * @brief Predict velocity after t seconds (constant velocity).
 *
 * Assumes constant velocity without acceleration,  
 * applies only velocity (`d->velocity`) and friction (`d->props.friction`).
 *
 * @note
 * - Suitable for entities without propulsion or external acceleration.
 * - Environment and gravity are not considered.
 *
 * @param d        Dynamic entity
 * @param dt       Time interval (seconds)
 * @param out_vel  Predicted velocity
 */
BYUL_API void entity_dynamic_calc_velocity(
    const entity_dynamic_t* d,
    float dt,
    vec3_t* out_vel);

// ---------------------------------------------------------
// State Prediction
// ---------------------------------------------------------
/**
 * @brief Predict entity position and velocity after t seconds (constant velocity).
 *
 * Combines `entity_dynamic_calc_position()` and `entity_dynamic_calc_velocity()`
 * to return position and velocity in a linear_state_t structure.
 *
 * @note
 * - Environment, gravity, and drag are not considered.
 * - Suitable for simple NPC or object motion.
 *
 * @param d         Dynamic entity
 * @param dt        Time interval (seconds)
 * @param out_state Predicted linear state (position + velocity)
 */
BYUL_API void entity_dynamic_calc_state(
    const entity_dynamic_t* d,
    float dt,
    linear_state_t* out_state);

// ---------------------------------------------------------
// Position Calculation with Environment
// ---------------------------------------------------------
/**
 * @brief Predict position after t seconds (with environment effects).
 *
 * Considers gravity (env->gravity), wind (env->wind_vel), and drag (air resistance)
 * using p = p0 + v0 * t + 0.5 * a * t^2.
 *
 * @note
 * - Suitable for projectiles or physics objects.
 * - Unlike entity_dynamic_calc_position(), acceleration is considered.
 *
 * @param d        Dynamic entity
 * @param env      Environment data
 * @param dt       Time interval (seconds)
 * @param out_pos  Predicted position
 */
BYUL_API void entity_dynamic_calc_position_env(
    const entity_dynamic_t* d,
    const environ_t* env,
    float dt,
    vec3_t* out_pos);

// ---------------------------------------------------------
// Velocity Calculation with Environment
// ---------------------------------------------------------
/**
 * @brief Predict velocity after t seconds (with environment effects).
 *
 * Considers gravity (env->gravity) and drag (air resistance).
 *
 * @param d        Dynamic entity
 * @param env      Environment data
 * @param dt       Time interval (seconds)
 * @param out_vel  Predicted velocity
 */
BYUL_API void entity_dynamic_calc_velocity_env(
    const entity_dynamic_t* d,
    const environ_t* env,
    float dt,
    vec3_t* out_vel);

// ---------------------------------------------------------
// Full State Prediction with Environment
// ---------------------------------------------------------
/**
 * @brief Predict full state (position + velocity) after t seconds (with environment).
 *
 * Combines `entity_dynamic_calc_position_env()` and `entity_dynamic_calc_velocity_env()`
 * into a single linear_state_t result.
 *
 * @param d         Dynamic entity
 * @param env       Environment data
 * @param dt        Time interval (seconds)
 * @param out_state Predicted dynamic entity state
 */
BYUL_API void entity_dynamic_calc_state_env(
    const entity_dynamic_t* d,
    const environ_t* env,
    float dt,
    linear_state_t* out_state);

/**
 * @brief Convert entity_dynamic_t to motion_state_t.
 *
 * @param ed      Source dynamic entity
 * @param out     Output motion_state
 * @param lin_acc External linear acceleration (optional, NULL allowed)
 * @param ang_acc External angular acceleration (optional, NULL allowed)
 */
BYUL_API void entity_dynamic_to_motion_state(
    const entity_dynamic_t* ed,
    motion_state_t* out,
    const vec3_t* lin_acc,
    const vec3_t* ang_acc);

/**
 * @brief Convert motion_state_t to entity_dynamic_t.
 */
BYUL_API void entity_dynamic_from_motion_state(
    entity_dynamic_t* ed, const motion_state_t* ms);

BYUL_API bool entity_dynamic_bounce(
    const entity_dynamic_t* d,
    const vec3_t* normal,
    vec3_t* out_velocity_out);

#ifdef __cplusplus
}
#endif

#endif // ENTITY_DYNAMIC_H
