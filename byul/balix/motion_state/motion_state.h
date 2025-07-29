#ifndef MOTION_STATE_H
#define MOTION_STATE_H

#include <stdbool.h>
#include "numal.h"
#include "byul_common.h"

#ifdef __cplusplus
extern "C" {
#endif

// ---------------------------------------------------------
// Motion State Structures (Linear + Angular)
// ---------------------------------------------------------

/**
 * @struct linear_state_t
 * @brief Represents the state of linear motion.
 */
typedef struct s_linear_state {
    vec3_t position;      /**< Current position */
    vec3_t velocity;      /**< Current velocity */
    vec3_t acceleration;  /**< Current acceleration */
} linear_state_t;

/**
 * @struct attitude_state_t
 * @brief Represents the state of rotational motion.
 */
typedef struct s_attitude_state {
    quat_t orientation;          /**< Current orientation (quaternion) */
    vec3_t angular_velocity;     /**< Current angular velocity */
    vec3_t angular_acceleration; /**< Current angular acceleration */
} attitude_state_t;

/**
 * @struct motion_state_t
 * @brief Represents both linear and rotational motion states.
 */
typedef struct s_motion_state {
    linear_state_t linear;    /**< Linear motion */
    attitude_state_t angular; /**< Rotational motion */
} motion_state_t;

/**
 * @brief Initialize a linear_state_t with zeros.
 * @param out Pointer to the structure to initialize.
 */
BYUL_API void linear_state_init(linear_state_t* out);

/**
 * @brief Initialize a linear_state_t with specified values.
 * @param out Pointer to the structure to initialize.
 * @param position Position vector.
 * @param velocity Velocity vector.
 * @param acceleration Acceleration vector.
 */
BYUL_API void linear_state_init_full(linear_state_t* out,
                                     const vec3_t* position,
                                     const vec3_t* velocity,
                                     const vec3_t* acceleration);

/**
 * @brief Copy a linear_state_t.
 * @param out Destination structure.
 * @param src Source structure.
 */
BYUL_API void linear_state_assign(
    linear_state_t* out, const linear_state_t* src);

/**
 * @brief Initialize an attitude_state_t with default values (identity quaternion).
 * @param out Pointer to the structure to initialize.
 */
BYUL_API void attitude_state_init(attitude_state_t* out);

/**
 * @brief Initialize an attitude_state_t with specified values.
 * @param out Pointer to the structure to initialize.
 * @param orientation Orientation quaternion.
 * @param angular_velocity Angular velocity vector.
 * @param angular_acceleration Angular acceleration vector.
 */
BYUL_API void attitude_state_init_full(attitude_state_t* out,
                                       const quat_t* orientation,
                                       const vec3_t* angular_velocity,
                                       const vec3_t* angular_acceleration);

/**
 * @brief Copy an attitude_state_t.
 * @param out Destination structure.
 * @param src Source structure.
 */
BYUL_API void attitude_state_assign(
    attitude_state_t* out, const attitude_state_t* src);

/**
 * @brief Initialize a motion_state_t with default values.
 * @param out Pointer to the structure to initialize.
 */
BYUL_API void motion_state_init(motion_state_t* out);

/**
 * @brief Initialize a motion_state_t with specified values.
 * @param out Pointer to the structure to initialize.
 * @param position Position vector.
 * @param velocity Velocity vector.
 * @param acceleration Acceleration vector.
 * @param orientation Orientation quaternion.
 * @param angular_velocity Angular velocity vector.
 * @param angular_acceleration Angular acceleration vector.
 */
BYUL_API void motion_state_init_full(motion_state_t* out,
                                     const vec3_t* position,
                                     const vec3_t* velocity,
                                     const vec3_t* acceleration,
                                     const quat_t* orientation,
                                     const vec3_t* angular_velocity,
                                     const vec3_t* angular_acceleration);

/**
 * @brief Copy a motion_state_t.
 * @param out Destination structure.
 * @param src Source structure.
 */
BYUL_API void motion_state_assign(
    motion_state_t* out, const motion_state_t* src);

/**
 * @brief Predict collision time between two motion_state_t objects.
 *
 * Assumes both objects are spheres (radius rA, rB) and calculates the time t > 0
 * when they collide based on their relative motion equations.
 *
 * @param a        First motion state.
 * @param b        Second motion state.
 * @param radius_a Radius of the first object.
 * @param radius_b Radius of the second object.
 * @param max_t    Maximum prediction time (seconds). The function searches for collisions within this time range only.
 * @param out_t    Collision time (seconds). Returns -1 if no collision is detected.
 * @return true    Collision occurs within max_t.
 * @return false   No collision.
 */
BYUL_API bool motion_state_calc_collision_time(
    const motion_state_t* a,
    const motion_state_t* b,
    float radius_a,
    float radius_b,
    float max_t,
    float* out_t);

/**
 * @brief Predict the collision point between two motion_state_t objects.
 *
 * First calculates collision time (t), then determines the collision coordinates at that t.
 *
 * @param a        First motion state.
 * @param b        Second motion state.
 * @param radius_a Radius of the first object.
 * @param radius_b Radius of the second object.
 * @param max_t    Maximum prediction time (seconds).
 * @param out_t    Collision time (seconds).
 * @param out_pos  Collision position (ignored if NULL).
 * @return true    Collision point successfully calculated.
 * @return false   No collision.
 */
BYUL_API bool motion_state_calc_collision_point(
    const motion_state_t* a,
    const motion_state_t* b,
    float radius_a,
    float radius_b,
    float max_t,
    float* out_t,
    vec3_t* out_pos);

#ifdef __cplusplus
}
#endif

#endif // MOTION_STATE_H
