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
 */
#ifndef NUMEQ_INTEGRATOR_H
#define NUMEQ_INTEGRATOR_H

#include "motion_state.h"

#ifdef __cplusplus
extern "C" {
#endif

// Forward declarations
typedef struct s_bodyprops bodyprops_t;
typedef struct s_environ environ_t;

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
 * @brief Configuration structure for integration.
 *
 * This structure manages settings for various integrators (Euler, RK4, etc.)
 * including time step, environment, and physical body properties.
 *
 * ### Mean Range
 * - The `dt` is typically set to 0.016f (60Hz).
 *   It does not need to be perfectly fixed as long as it stays
 *   close to the average.
 * - Small variations (+/-10 to 20%) generally do not affect stability.
 * - If `dt` is too large, accuracy drops; if too small, computation
 *   costs increase.
 */
typedef struct s_integrator_config {
    integrator_type_t type;           ///< Integration method to use
    float dt;                  ///< Time step (dt), typically 0.016f for 60Hz
    motion_state_t* prev_state;       ///< Previous state for Verlet method
    const environ_t* env;             ///< Environment data (gravity, wind, etc.)
    const bodyprops_t* body;          ///< Physical body properties (mass, drag, etc.)
    void* userdata;                   ///< Optional user-defined data
} integrator_config_t;

/**
 * @brief Initialize integrator_config_t with default values.
 *
 * Details:
 * - `type` is initialized to INTEGRATOR_MOTION_RK4.
 * - `dt` is set to 0.016f (60Hz).
 * - `prev_state`, `env`, `body`, `userdata` are set to NULL.
 *
 * @param[out] cfg Pointer to integrator_config_t to initialize.
 */
BYUL_API void integrator_config_init(integrator_config_t* cfg);

/**
 * @brief Initialize integrator_config_t with specified values.
 *
 * @param[out] cfg         Configuration structure to initialize.
 * @param[in]  type        Integrator type to use.
 * @param[in]  dt   Time step (dt, typically 0.016f for 60Hz).
 * @param[in]  prev_state  Previous state for Verlet method (NULL if not used).
 * @param[in]  env         Environment data (gravity, wind, etc.).
 * @param[in]  body        Physical body properties (mass, drag, etc.).
 * @param[in]  userdata    User-defined data pointer.
 *
 * @note dt outside the typical range can still work, but too large values
 *       reduce accuracy while too small values increase computational cost.
 */
BYUL_API void integrator_config_init_full(integrator_config_t* cfg,
                                          integrator_type_t type,
                                          float dt,
                                          motion_state_t* prev_state,
                                          const environ_t* env,
                                          const bodyprops_t* body,
                                          void* userdata);

BYUL_API void integrator_config_assign(
    integrator_config_t* out, const integrator_config_t* src);

// ---------------------------------------------------------
// Common Interface
// ---------------------------------------------------------

/**
 * @brief Integrate motion state over time according to the configuration.
 *
 * @param state      [in/out] Motion state (position, velocity, acceleration).
 * @param config     [in] Integration configuration including type and dt.
 *
 * @note Internally dispatches to the selected integration method.
 */
BYUL_API void numeq_integrate(
    motion_state_t* state, const integrator_config_t* config);

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
BYUL_API void numeq_integrate_euler(motion_state_t* state, float dt);

/**
 * @brief Semi-Implicit Euler integration.
 *
 * v_{t+1} = v_t + a * dt
 * p_{t+1} = p_t + v_{t+1} * dt
 *
 * More stable and recommended for most real-time simulations.
 */
BYUL_API void numeq_integrate_semi_implicit(
    motion_state_t* state, float dt);

/**
 * @brief Verlet integration (second-order accuracy).
 *
 * p_{t+1} = 2p_t - p_{t-1} + a * dt^2
 *
 * Requires previous position.
 * Useful for damping oscillations or trail effects.
 */
BYUL_API void numeq_integrate_verlet(motion_state_t* state,
    const motion_state_t* prev_state, float dt);

/**
 * @brief 4th-order Runge-Kutta integration (RK4).
 *
 * High accuracy for physics predictions.
 * Used in MPC, guided missiles, and complex dynamics.
 */
BYUL_API void numeq_integrate_rk4(motion_state_t* state, float dt);

BYUL_API void numeq_integrate_rk4_env(motion_state_t* state,
                                float dt,
                                const environ_t* env,
                                const bodyprops_t* body);

BYUL_API void numeq_integrate_attitude_euler(
    motion_state_t* state, float dt);

// ---------------------------------------------------------
// Rotational Integration (Semi-Implicit Euler)
// ---------------------------------------------------------
BYUL_API void numeq_integrate_attitude_semi_implicit(
    motion_state_t* state, float dt);

// ---------------------------------------------------------
// Rotational Integration (RK4)
// ---------------------------------------------------------
BYUL_API void numeq_integrate_attitude_rk4(
    motion_state_t* state, float dt);

BYUL_API void numeq_integrate_attitude_rk4_env(motion_state_t* state,
                                float dt,
                                const environ_t* env,
                                const bodyprops_t* body);

// Rotational (attitude) Verlet integration
BYUL_API void numeq_integrate_attitude_verlet(motion_state_t* state,
    const motion_state_t* prev_state, float dt);

// Combined linear + rotational Verlet integrator
BYUL_API void numeq_integrate_motion_verlet(motion_state_t* state,
    const motion_state_t* prev_state, float dt);

// Combined linear + rotational Euler integrator
BYUL_API void numeq_integrate_motion_euler(
    motion_state_t* state, float dt);

// Combined linear + rotational Semi-Implicit Euler integrator
BYUL_API void numeq_integrate_motion_semi_implicit(
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
 *       numeq_integrate_motion_rk4_env instead.
 */
BYUL_API void numeq_integrate_motion_rk4(
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
BYUL_API void numeq_integrate_motion_rk4_env(motion_state_t* state,
                                float dt,
                                const environ_t* env,
                                const bodyprops_t* body);

#ifdef __cplusplus
}
#endif

#endif // NUMEQ_INTEGRATOR_H
