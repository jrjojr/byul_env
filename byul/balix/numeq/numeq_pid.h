/**
 * @file numeq_pid.h
 * @brief PID (Proportional-Integral-Derivative) controller module.
 *
 * This module provides a scalar PID controller.
 * PID control calculates control output based on the error between
 * the target value and the measured value to stably achieve the desired state.
 *
 * ---
 *
 * ## PID principle
 *
 * PID control calculates output using three terms:
 *
 * - **Proportional (P):** Output proportional to the current error.  
 *   P = Kp * e(t)
 *
 * - **Integral (I):** Output based on the accumulated past error.  
 *   I = Ki * Integral e(t) dt
 *
 * - **Derivative (D):** Output based on the rate of change of error.  
 *   D = Kd * de(t)/dt
 *
 * **Total output:**  
 *   u(t) = P + I + D
 *
 * ---
 *
 * ## Example Usage
 *
 * ### Scalar PID
 * @code
 * pid_controller_t pid;
 *
 * // 1. Initialize with default values
 * pid_init(&pid);
 *
 * // 2. Initialize with user-defined values
 * pid_init_full(&pid, 1.0f, 0.1f, 0.05f, 0.01f); // Kp, Ki, Kd, dt
 *
 * // 3. Set internal state (integral term, previous error)
 * pid_set_state(&pid, 0.0f, 0.0f);
 *
 * // 4. Main loop
 * while (running) {
 *     float control = pid_update(&pid, target_value, current_value);
 *     // Use 'control' to drive the system
 * }
 *
 * // 5. Reset state
 * pid_reset(&pid);
 *
 * // 6. Copy state
 * pid_controller_t pid_copy;
 * pid_assign(&pid_copy, &pid);
 *
 * // 7. Preview output without changing internal state
 * float preview_control = pid_preview(&pid, target_value, current_value);
 * @endcode
 *
 * ---
 *
 * ## Key Features
 * - Supports anti-windup (prevents integral term from accumulating excessively)
 * - Output limiting (output_limit) is supported
 * - Provides a preview function for non-state-changing output prediction
 */
#ifndef NUMEQ_PID_H
#define NUMEQ_PID_H

#include "trajectory.h"

#ifdef __cplusplus
extern "C" {
#endif

// ---------------------------------------------------------
// PID Controller (Scalar)
// ---------------------------------------------------------

/**
 * @brief Scalar PID controller.
 */
typedef struct s_pid_controller {
    float kp;             /**< Proportional gain */
    float ki;             /**< Integral gain */
    float kd;             /**< Derivative gain */

    float integral;       /**< Accumulated error */
    float prev_error;     /**< Previous error */

    float output_limit;   /**< Output limit (0 or less means no limit) */
    float dt;             /**< Time step */
    bool anti_windup;     /**< Anti-windup flag */
} pid_controller_t;

/**
 * @brief Initialize PID controller with specified values.
 *
 * The user sets Kp, Ki, Kd, and dt.  
 * Integral and prev_error are initialized to 0.
 *
 * ### Parameter Guidelines:
 * - **kp (Proportional):** 0.0 ~ 10.0  
 *   Higher values respond faster but may overshoot.
 *
 * - **ki (Integral):** 0.0 ~ 1.0  
 *   Corrects steady-state error. Too high causes instability.
 *
 * - **kd (Derivative):** 0.0 ~ 1.0  
 *   Dampens overshoot but sensitive to noise.
 *
 * - **dt (Time step):** 0.001 ~ 0.1s  
 *   Control loop period (e.g., 0.01s for 100Hz).
 *
 * ### Default initialization:
 * - integral = 0.0f
 * - prev_error = 0.0f
 * - output_limit = 0.0f (no limit)
 * - anti_windup = false
 */
BYUL_API void pid_init_full(pid_controller_t* pid, 
    float kp, float ki, float kd, float dt);

/**
 * @brief Initialize PID controller with default values.
 *
 * Default values:
 * - Kp = 1.0f
 * - Ki = 0.0f
 * - Kd = 0.0f
 * - dt = 0.01f
 * - integral = 0.0f
 * - prev_error = 0.0f
 * - output_limit = 0.0f (no limit)
 * - anti_windup = false
 */
BYUL_API void pid_init(pid_controller_t* pid);

/**
 * @brief Auto-tuning initialization of PID.
 *
 * Uses a heuristic approach based on Ziegler-Nichols rules to set Kp, Ki, Kd.
 * Useful for quick starting points, but manual fine-tuning may be required.
 *
 * Internal default values:
 * - Kp = 0.6
 * - Ki = Kp / (0.5 * dt)
 * - Kd = 0.125 * Kp * dt
 */
BYUL_API void pid_init_auto(pid_controller_t* pid, float dt);

/**
 * @brief Copy PID state.
 */
BYUL_API void pid_assign(pid_controller_t* dst, 
    const pid_controller_t* src);

/**
 * @brief Reset PID state (zero reset).
 */
BYUL_API void pid_reset(pid_controller_t* pid);    

/**
 * @brief Set integral and previous error manually.
 *
 * Useful when restoring state or resetting specific terms.
 */
BYUL_API void pid_set_state(pid_controller_t* pid, 
    float integral, float prev_error);

/**
 * @brief Update PID and compute control output.
 *
 * Formula:
 * ```
 * error = target - measured
 * P = Kp * error
 * I = I + Ki * error * dt
 * D = Kd * (error - prev_error) / dt
 * control = P + I + D
 * ```
 */
BYUL_API float pid_update(pid_controller_t* pid, 
    float target, float measured);

/**
 * @brief Preview PID output without updating internal state.
 */
BYUL_API float pid_preview(const pid_controller_t* pid, 
    float target, float measured);

#ifdef __cplusplus
}
#endif

#endif // NUMEQ_PID_H
