/**
 * @file numeq_model_motion.h
 * @brief Full-body motion prediction using 
 * `motion_state_t` (translation + rotation).
 *
 * This module predicts future motion 
 * by fully utilizing both linear and angular state:
 * - Linear state: position, velocity, acceleration.
 * - Angular state: orientation, angular velocity, angular acceleration.
 *
 * Unlike linear-only models, 
 * this module considers rotational effects on future motion,
 * such as:
 * - Curved trajectories due to spin (e.g., Magnus effect).
 * - Induced lateral acceleration based on angular momentum.
 * - Orientation-based drag adjustments.
 *
 * @important
 * Even if the object moves linearly, 
 * its rotation affects acceleration and direction.
 * This is critical for simulating real-world behaviors like spinning balls, 
 * gyroscopic forces,
 * or asymmetric drag caused by orientation.
 *
 * @note
 * If angular effects are not yet implemented, 
 * the angular state must still be passed and
 * preserved for downstream usage (e.g., curved drag or collision modeling).
 *
 * This is a prediction system based on **the current state**, 
 * not a control system.
 */
#ifndef NUMEQ_MODEL_MOTION_H
#define NUMEQ_MODEL_MOTION_H

#include "motion_state.h"
#include "numeq_model.h"

#ifdef __cplusplus
extern "C" {
#endif

// ---------------------------------------------------------
// Full motion prediction: motion_state_t
// ---------------------------------------------------------

/**
 * @brief Predicts full motion state after time seconds.
 *
 * This function performs physical 
 * prediction for both translation and rotation:
 * - Linear state is integrated using environmental forces.
 * - Angular state is preserved (copied) 
 * as a placeholder for future integration.
 *
 * @param time          Time in seconds to predict forward.
 * @param state0     Initial full motion state.
 * @param env        Environment conditions (gravity, wind, etc.).
 * @param body       Physical properties (mass, drag, etc.).
 * @param[out] out_state Resulting predicted motion state.
 */
BYUL_API void numeq_model_motion_predict(
    float time,
    const motion_state_t* state0,
    const environ_t* env,
    const bodyprops_t* body,
    motion_state_t* out_state);

/**
 * @brief Predicts full motion state using RK4 integration.
 *
 * Linear state is numerically integrated over `steps` intervals.
 * Angular state is preserved as-is (not yet integrated).
 *
 * @param time          Total integration duration (seconds).
 * @param state0     Initial motion state.
 * @param env        Environment data.
 * @param body       Physical properties.
 * @param steps      Number of RK4 integration steps.
 * @param[out] out_state Resulting predicted motion state.
 */
BYUL_API void numeq_model_motion_predict_rk4(
    float time,
    const motion_state_t* state0,
    const environ_t* env,
    const bodyprops_t* body,
    int steps,
    motion_state_t* out_state);

/**
 * @brief Calculates total acceleration on a spinning projectile,
 *        including air drag and spin-induced effects (Magnus, Gyroscopic).
 *
 * @param[in]  state      Current motion state (linear + angular)
 * @param[in]  env        Environment data (air density, wind, etc.)
 * @param[in]  body       Physical body properties (mass, drag coefficient, etc.)
 * @param[in]  time         Time step (seconds)
 * @param[out] out_accel  Total resulting acceleration (m/s^2)
 */
BYUL_API void numeq_model_motion_accel(
    const motion_state_t* state,
    const environ_t* env,
    const bodyprops_t* body,
    float time,
    vec3_t* out_accel);

/**
 * @brief Computes acceleration induced by rotation, 
 * such as Magnus or gyroscopic effect.
 *
 * This function calculates additional acceleration 
 * components resulting from rotational
 * motion, which affect the linear trajectory. These effects include:
 * - **Magnus effect**: Lift-like acceleration caused by spin (ω × v).
 * - **Gyroscopic drift**: Acceleration caused by increasing spin (α × v).
 *
 * This is useful for simulating curved motion in 
 * spinning objects (e.g., balls, shells, drones).
 *
 * @param[out] out_accel
 *     Final acceleration vector induced by angular motion (in m/s²).
 * @param[in] process_dir_speed_sec
 *     Current linear velocity vector (m/s). 
 * This represents the direction and speed of motion.
 * @param[in] angular_velocity
 *     Angular velocity vector (rad/s), used to compute Magnus-like force.
 * @param[in] angular_accel
 *     Angular acceleration vector (rad/s²), used to compute gyroscopic drift.
 * @param[in]  time             Time step (seconds) — 
 * duration for which angular acceleration acts.
 * Typically between 0.01 and 0.1 for high-speed physics.
 * @param[in] k_magnus
 *     Coefficient for Magnus effect.
 *
 *     - **Reality-based range**: 0.05 ~ 0.3  
 *     - **Game-designed range**: 0.3 ~ 1.5  
 *     - **Maximum allowed**: up to 5.0  
 *
 *     Larger values exaggerate the curvature due to spin, 
 * useful for magical or non-physical projectiles.
 *
 * @param[in] k_gyro
 *     Coefficient for gyroscopic effect (due to angular acceleration).
 *
 *     - **Reality-based range**: 0.01 ~ 0.2  
 *     - **Game-designed range**: 0.2 ~ 1.0  
 *     - **Maximum allowed**: up to 4.0  
 *
 *     Higher values make projectiles curve more dramatically as 
 * they spin faster or decelerate rotationally.
 *
 * @note The final acceleration is computed as:
 *
 *       a = (k_magnus) * (angular_velocity × velocity)
 *         + (k_gyro) * time * (angular_acceleration × velocity)
 *
 *       Where:
 *       - angular_velocity × velocity     : represents Magnus-like force
 *       - angular_acceleration × velocity : represents gyroscopic deflection
 *
 * @warning Exceeding the recommended maximums 
 * (k_magnus > 5.0, k_gyro > 4.0) may cause unrealistic
 *          or unstable physics behaviors, 
 * including excessive curvature or simulation errors in RK4.
 */
BYUL_API void calc_spin_accel(
    vec3_t* out_accel,
    const vec3_t* process_dir_speed_sec,
    const vec3_t* angular_velocity,
    const vec3_t* angular_accel,
    float time,
    float k_magnus,
    float k_gyro);

BYUL_API float numeq_model_motion_drag_scale(
    const motion_state_t* state,
    const environ_t* env);

#ifdef __cplusplus
}
#endif

#endif // NUMEQ_MODEL_MOTION_H
