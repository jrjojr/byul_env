#ifndef NUMEQ_PID_VEC3_H
#define NUMEQ_PID_VEC3_H

#include "numeq_pid.h"
#include "trajectory.h"

#ifdef __cplusplus
extern "C" {
#endif

// ---------------------------------------------------------
// PID Controller (Vector Form)
// ---------------------------------------------------------

/**
 * @brief 3-axis vector PID controller.
 */
typedef struct s_pid_controller_vec3 {
    pid_controller_t x;
    pid_controller_t y;
    pid_controller_t z;
} pid_controller_vec3_t;

/**
 * @brief Initialize a vec3 PID controller with default values.
 */
BYUL_API void pid_vec3_init(pid_controller_vec3_t* pid);

/**
 * @brief Initialize a vec3 PID controller with user-defined values.
 */
BYUL_API void pid_vec3_init_full(pid_controller_vec3_t* pid,
                   float kp, float ki, float kd,
                   float dt);                   

/**
 * @brief Initialize a vec3 PID controller with auto-tuning.
 */
BYUL_API void pid_vec3_auto(pid_controller_vec3_t* pid, float dt);

/**
 * @brief Copy the state of a vec3 PID controller.
 */
BYUL_API void pid_vec3_assign(
    pid_controller_vec3_t* dst, const pid_controller_vec3_t* src);

/**
 * @brief Reset the state of a vec3 PID controller.
 */
BYUL_API void pid_vec3_reset(pid_controller_vec3_t* pid);

/**
 * @brief Set the internal state of a vec3 PID controller.
 *
 * This function sets the `integral` and `prev_error` values
 * of each axis (x, y, z) of the PID controller manually.
 *
 * @param pid        3-axis PID controller
 * @param integral   Integral term values (x, y, z)
 * @param prev_error Previous error values (x, y, z)
 *
 * @note Typically, `pid_vec3_reset()` is used to reset values to zero.
 *       Use this function only if you need to restore a previous state.
 */
BYUL_API void pid_vec3_set_state(pid_controller_vec3_t* pid,
                        const vec3_t* integral,
                        const vec3_t* prev_error);

/**
 * @brief Compute control output using vec3 PID.
 *
 * Compares the target (x, y, z) with the measured values and
 * computes the control vector (out_control) for each axis independently.
 *
 * **Calculation:**
 * - error = target - measured
 * - P = Kp * error
 * - I = I + Ki * error * dt
 * - D = Kd * (error - prev_error) / dt
 * - control = P + I + D
 *
 * @param pid         3-axis PID controller
 * @param target      Target vector (x, y, z)
 * @param measured    Measured vector (x, y, z)
 * @param out_control Computed control vector (x, y, z)
 */
BYUL_API void pid_vec3_update(pid_controller_vec3_t* pid,
                     const vec3_t* target,
                     const vec3_t* measured,
                     vec3_t* out_control);

/**
 * @brief Preview the control output of vec3 PID without state changes.
 *
 * Performs the same calculations as `pid_vec3_update()` but
 * does not update the internal state (`integral`, `prev_error`).
 *
 * **Usage:**
 * - Useful for simulations to check expected outputs.
 * - Evaluate control stability before applying it.
 *
 * @param pid         3-axis PID controller (state remains unchanged)
 * @param target      Target vector (x, y, z)
 * @param measured    Measured vector (x, y, z)
 * @param out_control Computed control vector (x, y, z)
 */
BYUL_API void pid_vec3_preview(const pid_controller_vec3_t* pid,
                      const vec3_t* target,
                      const vec3_t* measured,
                      vec3_t* out_control);

#ifdef __cplusplus
}
#endif

#endif // NUMEQ_PID_VEC3_H
