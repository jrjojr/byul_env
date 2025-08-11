/**
 * @file numeq_integrator.h
 * @brief Numerical integration module for linear + rotational motion.
 *
 * This module predicts linear and rotational motion states based on
 * motion_state_t using various integration methods.
 *
 * Provided features:
 * - Linear motion integration 
 * (Euler, Semi-Implicit Euler, Verlet, Velocity Verlet, RK4)
 * - Rotational motion integration 
 * (Quaternion-based Euler, Semi-Implicit Euler, Verlet, Velocity Verlet, RK4)
 * - Combined linear + rotational integrators (Motion series)
 *
 * Integration method overview:
 * - **Euler**: Simplest, lowest accuracy, prone to instability at large dt. 
 *              Use only for quick prototypes or very small dt.
 * - **Semi-Implicit Euler**: 
 * Slightly more stable than Euler, good for real-time simulation at 60Hz.
 * - **Verlet**: Requires previous position, 
 * stable for oscillations and trail effects, but velocity is approximate.
 * - **Velocity Verlet**: No previous position needed, 
 * accurate velocity tracking, stable for most real-time simulations.
 * - **RK4**: Fourth-order Runge–Kutta, 
 * high accuracy for complex or fast-changing forces, 
 * best for offline trajectory calculation.
 *
 * Default choice:
 * - **Velocity Verlet** is recommended as the default integrator 
 * for general-purpose real-time physics.
 * - **RK4** should be used when maximum precision is required 
 * (e.g., offline trajectory generation or ballistic computation).
 *
 * Recommended usage scenarios:
 * - Real-time physics simulation 
 * (60Hz, Velocity Verlet or Semi-Implicit Euler)
 * - High-accuracy trajectory calculation (RK4)
 * - Special effects requiring historical states (Classic Verlet)
 *
 * @note Integration functions in this module perform 
 * only **single-step updates** using the given `dt`.
 *       To simulate longer periods, 
 * call the integrator repeatedly in a loop with small steps
 *       (e.g., 100 iterations with dt = 1.0f for a 100-second simulation).
 *
 * @attention This module provides only integrators, 
 * not high-level trajectory predictors.
 *            For long-term prediction, 
 * build your own loop using the integrators here,
 *            and include environment/body-dependent acceleration as needed.
 */
#ifndef NUMEQ_INTEGRATOR_H
#define NUMEQ_INTEGRATOR_H

#include "motion_state.h"
#include "environ.h"
#include "bodyprops.h"

#ifdef __cplusplus
extern "C" {
#endif

// ---------------------------------------------------------
// Integrator Types (simulation method selection)
// ---------------------------------------------------------

/**
 * @brief Types of numerical integrators.
 */
typedef enum e_integrator_type {
    INTEGRATOR_EULER,
    INTEGRATOR_SEMI_IMPLICIT,
    INTEGRATOR_VERLET,
    INTEGRATOR_RK4,

    INTEGRATOR_EULER_ENV,
    INTEGRATOR_SEMI_IMPLICIT_ENV,
    INTEGRATOR_VERLET_ENV,
    INTEGRATOR_VELOCITY_VERLET_ENV,
    INTEGRATOR_RK4_ENV,

    INTEGRATOR_MOTION_EULER,
    INTEGRATOR_MOTION_SEMI_IMPLICIT,
    INTEGRATOR_MOTION_VERLET,
    INTEGRATOR_MOTION_RK4,

    INTEGRATOR_MOTION_RK4_ENV
} integrator_type_t;

/**
 * @brief Configuration structure for single-step motion integration.
 *
 * This structure holds settings for an integrator instance, such as
 * method type, time step, environmental forces, and body properties.
 *
 * Usage:
 * - This is **not** a predictor. It only holds the configuration for one integration step.
 * - Use `integrator_step()` repeatedly to simulate over longer durations.
 *
 * Time Step (`dt`):
 * - Typically set to 0.016f for 60Hz simulation.
 * - Slight variations (±10~20%) are acceptable.
 * - Large dt reduces accuracy; small dt increases computational cost.
 */
typedef struct s_integrator {
    motion_state_t state;             ///< Current state to integrate
    integrator_type_t type;            ///< Integration method

    motion_state_t prev_state;       ///< Required for Verlet
    environ_t env;             ///< Gravity, wind, etc.
    bodyprops_t body;          ///< Physical properties (mass, drag)
} integrator_t;

/**
 * @brief Initializes the integrator with default settings.
 *
 * Sets the integration type to RK4.
 */
BYUL_API void integrator_init(integrator_t* intgr);

/**
 * @brief Fully initializes an integrator configuration.
 *
 * This function sets up an integrator_t structure for single-step motion integration.
 * It assigns the integration method, time step, and associated motion/environment properties.
 *
 * @param[out] intgr         Target integrator structure to initialize (must not be NULL).
 * @param[in]  type          Integration method to use (e.g., Euler, RK4, Verlet).
 * @param[in]  state         Pointer to current motion state (required, must not be NULL).
 * @param[in]  prev_state    Pointer to previous motion state (required for Verlet).
 *                           Can be NULL if not using Verlet.
 * @param[in]  env           Pointer to environment configuration (gravity, wind, etc.).
 *                           Can be NULL to assume zero environment.
 * @param[in]  body          Pointer to body physical properties (mass, drag, etc.).
 *                           Can be NULL to use default properties.
 *
 * @note This function does not allocate or deep-copy the provided pointers.
 *       The caller must ensure that all pointers remain valid during use.
 *       For Verlet integration, `prev_state` must be provided and non-NULL.
 */
BYUL_API void integrator_init_full(integrator_t* intgr,
                                   const integrator_type_t type,
                                   const motion_state_t* state,
                                   const motion_state_t* prev_state,
                                   const environ_t* env,
                                   const bodyprops_t* body);

// Frees all variables allocated internally and reinitializes them.
BYUL_API void integrator_clear(integrator_t* intgr);

// Frees all variables allocated internally and reinitializes them.
BYUL_API void integrator_free(integrator_t* intgr);

BYUL_API void integrator_assign(
    integrator_t* out, const integrator_t* src);

/**
 * @brief Performs a single integration step according to the configuration.
 *
 * This function dispatches to the appropriate integration method (Euler, RK4, etc.)
 * based on the `type` field inside `integrator_t`.
 *
 * @note This updates the state by one step of `dt`. To simulate over longer time spans,
 *       you must call this function repeatedly.
 */
BYUL_API void integrator_step(integrator_t* intgr, float dt);

// ---------------------------------------------------------
// Numerical Integration Methods
// ---------------------------------------------------------

/**
 * @brief Euler integration.
 *
 * v_{t+1} = v_t + a * dt
 * p_{t+1} = p_t + v_t * dt
 *
 * Simple but less accurate and can be unstable.
 */
BYUL_API void integrator_step_euler(motion_state_t* state, float dt);

BYUL_API void integrator_step_euler_env(
    motion_state_t* state,
    float dt,
    const environ_t* env,
    const bodyprops_t* body);

/**
 * @brief Semi-Implicit Euler integration.
 *
 * v_{t+1} = v_t + a * dt
 * p_{t+1} = p_t + v_{t+1} * dt
 *
 * More stable and recommended for most real-time simulations.
 */
BYUL_API void integrator_step_semi_implicit(
    motion_state_t* state, float dt);

// Semi-Implicit (Symplectic) Euler with environment-dependent acceleration
BYUL_API void integrator_step_semi_implicit_env(
    motion_state_t* state,
    float dt,
    const environ_t* env,
    const bodyprops_t* body);

/**
 * @brief Verlet integration (second-order accuracy).
 *
 * p_{t+1} = 2p_t - p_{t-1} + a * dt^2
 *
 * Requires previous position.
 * Useful for damping oscillations or trail effects.
 */
BYUL_API void integrator_step_verlet(motion_state_t* state,
    const motion_state_t* prev_state, float dt);

/**
 * @brief Classic Verlet integration with environment-dependent acceleration.
 *
 * Computes the next position using the classic Verlet formula:
 *    p(t+dt) = 2 * p(t) - p(t-dt) + a(t) * dt^2
 *
 * - Requires the previous position state (`prev_state`).
 * - Suitable for simulating motion with minimal numerical drift.
 * - Acceleration is computed from environment (`env`) and body properties (`body`),
 *   allowing effects such as gravity, wind, and drag to be applied.
 *
 * @param[in,out] state       Current motion state (updated in place).
 * @param[in]     prev_state  Previous motion state (required for position update).
 * @param[in]     dt          Time step (seconds).
 * @param[in]     env         Environment parameters (gravity, wind, etc.).
 * @param[in]     body        Body properties (mass, drag coefficient, etc.).
 */    
BYUL_API void integrator_step_verlet_env(
    motion_state_t* state,
    const motion_state_t* prev_state,
    float dt,
    const environ_t* env,
    const bodyprops_t* body);    

/**
 * @brief Velocity Verlet integration with environment-dependent acceleration.
 *
 * Computes position and velocity updates in two half-steps:
 *   v_half = v(t) + 0.5 * a(t) * dt
 *   p(t+dt) = p(t) + v_half * dt
 *   a(t+dt) = acceleration from env/body at p(t+dt)
 *   v(t+dt) = v_half + 0.5 * a(t+dt) * dt
 *
 * - Does not require a previous position state (unlike classic Verlet).
 * - More accurate velocity tracking compared to classic Verlet.
 * - Acceleration is recomputed at the new position to reflect environment and body forces.
 *
 * @param[in,out] state  Current motion state (updated in place).
 * @param[in]     dt     Time step (seconds).
 * @param[in]     env    Environment parameters (gravity, wind, etc.).
 * @param[in]     body   Body properties (mass, drag coefficient, etc.).
 */
BYUL_API void integrator_step_velocity_verlet_env(
    motion_state_t* state,
    float dt,
    const environ_t* env,
    const bodyprops_t* body);    

/**
 * @brief 4th-order Runge-Kutta integration (RK4).
 *
 * High accuracy for physics predictions.
 * Used in MPC, guided missiles, and complex dynamics.
 */
BYUL_API void integrator_step_rk4(motion_state_t* state, float dt);

BYUL_API void integrator_step_rk4_env(motion_state_t* state,
                                float dt,
                                const environ_t* env,
                                const bodyprops_t* body);

BYUL_API void integrator_step_attitude_euler(
    motion_state_t* state, float dt);

BYUL_API void integrator_step_attitude_euler_env(
    motion_state_t* state,
    float dt,
    const environ_t* env,
    const bodyprops_t* body);    

// ---------------------------------------------------------
// Rotational Integration (Semi-Implicit Euler)
// ---------------------------------------------------------
BYUL_API void integrator_step_attitude_semi_implicit(
    motion_state_t* state, float dt);

BYUL_API void integrator_step_attitude_semi_implicit_env(
    motion_state_t* state,
    float dt,
    const environ_t* env,
    const bodyprops_t* body);    

/**
 * @brief Attitude integration using Velocity-Verlet (no previous state needed).
 *
 * Update scheme:
 *   w_half = w(t) + 0.5 * alpha(t) * dt
 *   q(t+dt) = q(t) * exp( w_half * dt )    // implemented via quat_init_angular_velocity
 *   alpha(t+dt) ~ alpha(t)                  // if no environment re-evaluation is available
 *   w(t+dt) = w_half + 0.5 * alpha(t+dt) * dt
 *
 * Notes:
 * - This variant does NOT re-evaluate acceleration at t+dt; it uses alpha(t).
 *   For forces/torques that depend on orientation/omega, prefer the _env variant below.
 * - Quaternion is normalized after the update to avoid drift.
 *
 * @param[in,out] state  Motion state (orientation, angular velocity/accel updated in place).
 * @param[in]     dt     Time step (seconds).
 */
BYUL_API void integrator_step_attitude_velocity_verlet(
    motion_state_t* state, float dt);

/**
 * @brief Attitude integration using Velocity-Verlet with environment-dependent acceleration.
 *
 * Update scheme:
 *   1) alpha(t) from env/body/torques (may depend on q(t), w(t))
 *   2) w_half = w(t) + 0.5 * alpha(t) * dt
 *   3) q(t+dt) = q(t) * exp( w_half * dt ), then normalize
 *   4) Recompute alpha(t+dt) using updated state if your model depends on q or w
 *   5) w(t+dt) = w_half + 0.5 * alpha(t+dt) * dt
 *
 * Notes:
 * - The sample uses a simple angular-drag term to illustrate env/body coupling.
 *   Replace/extend it with your full torque model as needed.
 *
 * @param[in,out] state  Motion state (orientation, angular velocity/accel updated in place).
 * @param[in]     dt     Time step (seconds).
 * @param[in]     env    Environment (e.g., air density). Can be NULL.
 * @param[in]     body   Body properties (e.g., mass, drag coef). Can be NULL.
 */
BYUL_API void integrator_step_attitude_velocity_verlet_env(
    motion_state_t* state,
    float dt,
    const environ_t* env,
    const bodyprops_t* body);    

// ---------------------------------------------------------
// Rotational Integration (RK4)
// ---------------------------------------------------------
BYUL_API void integrator_step_attitude_rk4(
    motion_state_t* state, float dt);

BYUL_API void integrator_step_attitude_rk4_env(motion_state_t* state,
                                float dt,
                                const environ_t* env,
                                const bodyprops_t* body);


// Explicit Euler motion step using Dual Quaternion pose integration.
// - Pose D = qr + eps qd
// - Ddot = 0.5 * Omega * D,  Omega = [0, w] + eps [0, v]  (world-frame)
// - Use v(t), w(t) for pose update (explicit), then update velocities with a, alpha.
BYUL_API void integrator_step_motion_euler(
    motion_state_t* state, float dt);

// Combined linear + rotational Semi-Implicit Euler integrator
BYUL_API void integrator_step_motion_semi_implicit(
    motion_state_t* state, float dt);

// Combined linear + rotational Verlet integrator
BYUL_API void integrator_step_motion_verlet(motion_state_t* state,
    const motion_state_t* prev_state, float dt);

/**
 * @brief Combined linear + rotational RK4 integrator (ignores environment).
 *
 * @details
 * Uses 4th-order Runge-Kutta (RK4) method to integrate motion_state_t
 * for both linear and rotational states over a time step dt.
 *
 * @param state [in/out] Current motion state (position, velocity, orientation, etc.).
 *                       Updated to state after dt seconds.
 * @param dt    [in] Integration time step (in seconds).
 *
 * @note For external effects (gravity, drag, etc.), use
 *       integrator_step_motion_rk4_env instead.
 */
BYUL_API void integrator_step_motion_rk4(
    motion_state_t* state, float dt);

/**
 * @brief Combined linear + rotational RK4 integrator (with environment).
 *
 * @details
 * Uses 4th-order Runge-Kutta (RK4) integration to simulate linear and rotational
 * motion for motion_state_t over a time step dt.
 * This version takes environment data (@p env) and body properties (@p body)
 * into account to recalculate accelerations and angular accelerations at each step.
 *
 * @param state [in/out] Current motion state (motion_state_t).
 *                       Updated to state after dt seconds.
 * @param dt    [in] Integration time step (in seconds).
 * @param env   [in] Environment data (gravity, wind, air density, etc.).
 * @param body  [in] Physical body properties (mass, drag coefficient, friction, etc.).
 *
 * @note Provides stable prediction even for nonlinear forces like drag or external torque.
 */
BYUL_API void integrator_step_motion_rk4_env(motion_state_t* state,
                                float dt,
                                const environ_t* env,
                                const bodyprops_t* body);

#ifdef __cplusplus
}
#endif

#endif // NUMEQ_INTEGRATOR_H
