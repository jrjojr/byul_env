/**
 * @file numeq_model.h
 * @brief Numerical equation-based module for predicting physical states.
 *
 * This module, based on the given initial motion state (linear_state_t), 
 * environment (environ_t), and body physical properties (bodyprops_t), provides:
 *
 * - Prediction of **position p(t)**, **velocity v(t)**, **acceleration a(t)** (parabolic motion + drag).
 * - Computation of the complete linear state (linear_state_t) after t seconds.
 * - Calculation of air drag (F_drag = 0.5 * rho * v^2 * Cd * A).
 * - Determination of apex (highest point) and landing conditions.
 * - Collision bounce (reflection) interface.
 *
 * @note This module does not handle rotational motion (attitude_state_t),
 * and deals only with linear motion (position/velocity/acceleration).
 */
#ifndef NUMEQ_MODEL_H
#define NUMEQ_MODEL_H

#include "trajectory.h"
#include "vec3.h"
#include "environ.h"
#include "bodyprops.h"

#ifdef __cplusplus
extern "C" {
#endif

// ---------------------------------------------------------
// Air drag calculation (a = F / m)
// ---------------------------------------------------------

/**
 * @brief Calculates the air drag acceleration acting on a body.
 *
 * @param state0        Initial linear state (position, velocity).
 * @param env           Environment data (air density, wind).
 * @param body          Body properties (mass, drag coefficient, etc.).
 * @param[out] out_drag_accel Computed drag acceleration (m/s^2).
 *
 * @note Drag is calculated using the relative velocity (v - wind).
 */
BYUL_API void numeq_model_drag_accel(const linear_state_t* state0,
                        const environ_t* env,
                        const bodyprops_t* body,
                        vec3_t* out_drag_accel);

/**
 * @brief Calculates the total acceleration at the current time.
 *
 * @param state         Current linear state.
 * @param env           Environment data.
 * @param body          Body properties.
 * @param[out] out_accel Computed total acceleration (m/s^2).
 *
 * @note Total acceleration includes gravity, drag, and environmental adjustments.
 */
BYUL_API void numeq_model_accel(const linear_state_t* state,
                       const environ_t* env,
                       const bodyprops_t* body,
                       vec3_t* out_accel);

/**
 * @brief Calculates external acceleration excluding gravity (drag + wind + state.accel).
 *
 * Unlike numeq_model_accel(), which includes gravity (env->gravity),
 * this function sums all external forces excluding gravity.
 *
 * @param[in]  state     Linear state (velocity, current acceleration).
 * @param[in]  env       Environment data (wind, humidity, pressure, etc.).
 * @param[in]  body      Body properties (mass, drag coefficient, etc.).
 * @param[out] out_accel Computed external acceleration vector (gravity excluded).
 */
BYUL_API void numeq_model_accel_except_gravity(
    const linear_state_t* state,
    const environ_t* env,
    const bodyprops_t* body,
    vec3_t* out_accel);

// ---------------------------------------------------------
// Acceleration: a(t)
// ---------------------------------------------------------

/**
 * @brief Calculates the acceleration after t seconds.
 *
 * @param t             Prediction time (seconds).
 * @param state0        Initial state.
 * @param env           Environment data.
 * @param body          Body properties.
 * @param[out] out_accel Predicted acceleration after t seconds (m/s^2).
 *
 * @note Internally calls numeq_model_vel_at() to recalculate drag at time t.
 */
BYUL_API void numeq_model_accel_at(float t,
                          const linear_state_t* state0,
                          const environ_t* env,
                          const bodyprops_t* body,
                          vec3_t* out_accel);

// ---------------------------------------------------------
// Position: p(t)
// ---------------------------------------------------------

/**
 * @brief Calculates the position after t seconds (linear approximation).
 *
 * @param t             Prediction time (seconds).
 * @param state0        Initial state.
 * @param env           Environment data.
 * @param body          Body properties.
 * @param[out] out_position Predicted position after t seconds (m).
 *
 * @note Uses constant acceleration approximation: p(t) = p0 + v0*t + 0.5*a0*t^2.
 */
BYUL_API void numeq_model_pos_at(float t,
                        const linear_state_t* state0,
                        const environ_t* env,
                        const bodyprops_t* body,
                        vec3_t* out_position);

// ---------------------------------------------------------
// Velocity: v(t)
// ---------------------------------------------------------

/**
 * @brief Calculates the velocity after t seconds (linear approximation).
 *
 * @param t             Prediction time (seconds).
 * @param state0        Initial state.
 * @param env           Environment data.
 * @param body          Body properties.
 * @param[out] out_velocity Predicted velocity after t seconds (m/s).
 *
 * @note Uses constant acceleration approximation: v(t) = v0 + a0*t.
 */
BYUL_API void numeq_model_vel_at(float t,
                        const linear_state_t* state0,
                        const environ_t* env,
                        const bodyprops_t* body,
                        vec3_t* out_velocity);

// ---------------------------------------------------------
// Full state prediction: state(t)
// ---------------------------------------------------------

/**
 * @brief Calculates the linear state (position, velocity, acceleration) after t seconds.
 *
 * @param t             Prediction time (seconds).
 * @param state0        Initial state.
 * @param env           Environment data.
 * @param body          Body properties.
 * @param[out] out_state Predicted linear state after t seconds.
 */
BYUL_API void numeq_model_calc(float t,
                         const linear_state_t* state0,
                         const environ_t* env,
                         const bodyprops_t* body,
                         linear_state_t* out_state);

/**
 * @brief Predicts the linear state after t seconds using RK4 integration.
 *
 * @param t             Prediction time (seconds).
 * @param state0        Initial state.
 * @param env           Environment data.
 * @param body          Body properties.
 * @param steps         Number of integration steps (e.g., t=1 sec, steps=60 -> dt=1/60).
 * @param[out] out_state Predicted linear state after t seconds using RK4.
 *
 * @note Provides higher accuracy when drag, gravity, or environmental factors vary over time.
 */
BYUL_API void numeq_model_calc_rk4(
    float t,
    const linear_state_t* state0,
    const environ_t* env,
    const bodyprops_t* body,
    int steps,
    linear_state_t* out_state);

// ---------------------------------------------------------
// Basic collision bounce
// ---------------------------------------------------------

/**
 * @brief Computes the basic collision reflection velocity using vector reflection.
 *
 * @param velocity_in   Velocity before impact.
 * @param normal        Surface normal (must be normalized).
 * @param restitution   Restitution coefficient (0 to 1).
 * @param[out] out_velocity_out Velocity after bounce.
 *
 * @return true on success.
 */
bool numeq_model_bounce(const vec3_t* velocity_in,
                        const vec3_t* normal,
                        float restitution,
                        vec3_t* out_velocity_out);

/**
 * @brief Predicts the collision time between two objects.
 *
 * @param my_state      My linear state.
 * @param other_state   Other object's linear state.
 * @param radius_sum    Combined radius of both objects (collision threshold).
 * @param[out] out_time Predicted collision time (seconds).
 * @param[out] out_point Predicted collision point.
 *
 * @return true if a collision is predicted, false otherwise.
 *
 * @note The current implementation assumes constant or uniform acceleration.
 */
bool numeq_model_calc_collision(
    const linear_state_t* my_state,
    const linear_state_t* other_state,
    float radius_sum,
    float* out_time,
    vec3_t* out_point);

#ifdef __cplusplus
}
#endif

#endif // NUMEQ_MODEL_H
