/**
 * @file numeq_integrator.h
 * @brief Numerical integration module for linear + rotational motion.
 *
 * This module predicts linear and rotational motion states based on
 * motion_state_t using various integration methods.
 *
 * Provided features:
 * - Linear motion integration (Euler, Semi-Implicit, Verlet, RK4)
 * - Rotational motion integration (Quaternion-based Euler, Semi-Implicit, Verlet, RK4)
 * - Combined linear + rotational integrators (Motion series)
 *
 * Recommended usage scenarios:
 * - Real-time physics simulation (60Hz, Semi-Implicit Euler)
 * - High-accuracy trajectory calculation (RK4)
 * - Special effects requiring historical states (Verlet)
 *
 * @note INTEGRATOR_EULER is the simplest but can be unstable.
 *       For general game physics, INTEGRATOR_SEMI_IMPLICIT or RK4 is recommended.
 *
 * @warning Integration functions in this module perform only **single-step updates** using the given `dt`.
 *          If you want to simulate motion over a longer time (e.g., 100 seconds), you must call the integrator
 *          repeatedly in a loop, accumulating small steps (e.g., 100 iterations with dt = 1.0f).
 *
 * @attention This module provides only integrators, not predictors.
 *            For long-term prediction or trajectory generation, you must build your own loop
 *            using the integrators provided here.
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
    INTEGRATOR_EULER,               ///< Simple Euler method
    INTEGRATOR_SEMI_IMPLICIT,       ///< Semi-Implicit Euler (velocity-first)
    INTEGRATOR_VERLET,              ///< Verlet method (requires previous position)
    INTEGRATOR_RK4,                 ///< 4th-order Runge-Kutta (high accuracy)
    INTEGRATOR_RK4_ENV,             ///< Linear + environment RK4
    INTEGRATOR_MOTION_EULER,        ///< Linear + rotational Euler
    INTEGRATOR_MOTION_SEMI_IMPLICIT,///< Linear + rotational Semi-Implicit Euler
    INTEGRATOR_MOTION_VERLET,       ///< Linear + rotational Verlet
    INTEGRATOR_MOTION_RK4,          ///< Linear + rotational RK4
    INTEGRATOR_MOTION_RK4_ENV       ///< Linear + rotational + environment RK4
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
    float dt;                          ///< Time step (single step only)

    motion_state_t prev_state;       ///< Required for Verlet
    environ_t env;             ///< Gravity, wind, etc.
    bodyprops_t body;          ///< Physical properties (mass, drag)
} integrator_t;

/**
 * @brief Initializes the integrator with default settings.
 *
 * Sets the integration type to RK4 and time step to 0.016f.
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
 * @param[in]  dt            Time step for a single integration step (e.g., 0.016f for 60Hz).
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
                                   const float dt,
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
BYUL_API void integrator_step(integrator_t* intgr);

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

// ---------------------------------------------------------
// Rotational Integration (Semi-Implicit Euler)
// ---------------------------------------------------------
BYUL_API void integrator_step_attitude_semi_implicit(
    motion_state_t* state, float dt);

// ---------------------------------------------------------
// Rotational Integration (RK4)
// ---------------------------------------------------------
BYUL_API void integrator_step_attitude_rk4(
    motion_state_t* state, float dt);

BYUL_API void integrator_step_attitude_rk4_env(motion_state_t* state,
                                float dt,
                                const environ_t* env,
                                const bodyprops_t* body);

// Rotational (attitude) Verlet integration
BYUL_API void integrator_step_attitude_verlet(motion_state_t* state,
    const motion_state_t* prev_state, float dt);

// Combined linear + rotational Verlet integrator
BYUL_API void integrator_step_motion_verlet(motion_state_t* state,
    const motion_state_t* prev_state, float dt);

// Combined linear + rotational Euler integrator
BYUL_API void integrator_step_motion_euler(
    motion_state_t* state, float dt);

// Combined linear + rotational Semi-Implicit Euler integrator
BYUL_API void integrator_step_motion_semi_implicit(
    motion_state_t* state, float dt);

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
