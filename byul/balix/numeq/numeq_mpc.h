/**
 * @file numeq_mpc.h
 * @brief Model Predictive Control (MPC) module header (based on motion_state_t).
 *
 * This header provides the Model Predictive Control (MPC) algorithm for
 * physics-based simulations, including **position + rotation prediction**, 
 * target tracking, and guidance control.
 *
 * ## Overview of MPC
 * Model Predictive Control works as follows:
 * 1. **Predict future states by applying multiple candidate accelerations/ang. accelerations 
 *    from the current motion_state_t.**
 * 2. **Calculate the cost between the predicted result and the target position/orientation.**
 * 3. **Select and apply the control input with the lowest cost.**
 * 4. **Repeat this process at the next frame.**
 *
 * MPC is suitable for the following:
 * - Projectile/missile trajectory and rotation control
 * - Handling environmental changes (wind, gravity)
 * - Target position + orientation tracking
 * - Constraints (max acceleration/angular acceleration, speed, etc.)
 *
 * This module supports:
 * - Single target point MPC (numeq_mpc_solve)
 * - Multi-waypoint path following (numeq_mpc_solve_route)
 * - Direction-holding target control (numeq_mpc_solve_directional)
 * - Custom cost functions (mpc_cost_func)
 * - Trajectory prediction and debugging
 */

#ifndef NUMEQ_MPC_H
#define NUMEQ_MPC_H

#include "trajectory.h"
#include "numeq_model.h"
#include "numal.h"

#ifdef __cplusplus
extern "C" {
#endif

// ---------------------------------------------------------
// Core Structures
// ---------------------------------------------------------

/**
 * @struct mpc_config_t
 * @brief MPC (Model Predictive Control) configuration parameters.
 *
 * This structure defines various parameters used in MPC-based path prediction
 * and control algorithms, including time horizon, speed/acceleration limits,
 * and cost weights.
 *
 * **Variable descriptions and defaults:**
 * - horizon_sec = 1.0f  
 *   Prediction time horizon (seconds). Example: horizon_sec = 1.0f predicts 1 second ahead.
 *
 * - step_dt = 0.05f  
 *   Simulation step interval. The total number of prediction steps is horizon_sec / step_dt.
 *
 * - max_accel = 10.0f  
 *   Maximum linear acceleration (m/s^2).
 *
 * - max_ang_accel = 5.0f  
 *   Maximum angular acceleration (rad/s^2).
 *
 * - max_speed = 50.0f  
 *   Maximum linear speed (m/s).
 *
 * - max_ang_speed = 10.0f  
 *   Maximum angular speed (rad/s).
 *
 * - weight_distance = 1.0f  
 *   Cost weight for target distance error.
 *
 * - weight_orientation = 0.5f  
 *   Cost weight for orientation error.
 *
 * - weight_velocity = 0.1f  
 *   Cost weight for velocity stability.
 *
 * - weight_accel = 0.1f  
 *   Cost weight for acceleration.
 *
 * - weight_ang_accel = 0.1f  
 *   Cost weight for angular acceleration.
 *
 * - max_iter = 10  
 *   Maximum internal optimization iterations.
 *
 * - output_trajectory = false  
 *   If true, the predicted trajectory is stored externally.
 *
 * - candidate_step = 0.5f  
 *   Step size between linear acceleration candidates.
 *
 * - ang_candidate_step = 0.1f  
 *   Step size between angular acceleration candidates.
 */
typedef struct s_mpc_config {
    float horizon_sec;
    float step_dt;
    float max_accel;
    float max_ang_accel;
    float max_speed;
    float max_ang_speed;
    float weight_distance;
    float weight_orientation;
    float weight_velocity;
    float weight_accel;
    float weight_ang_accel;
    int max_iter;
    bool output_trajectory;
    float candidate_step;
    float ang_candidate_step;
} mpc_config_t;

/**
 * @brief Initialize mpc_config_t with default values.
 */
BYUL_API void mpc_config_init(mpc_config_t* cfg);

/**
 * @brief Initialize mpc_config_t with custom values.
 */
BYUL_API void mpc_config_init_full(mpc_config_t* cfg,
                          float horizon_sec,
                          float step_dt,
                          float max_accel,
                          float max_ang_accel,
                          float max_speed,
                          float max_ang_speed,
                          float weight_distance,
                          float weight_orientation,
                          float weight_velocity,
                          float weight_accel,
                          float weight_ang_accel,
                          int max_iter,
                          bool output_trajectory,
                          float candidate_step,
                          float ang_candidate_step);

/**
 * @brief Copy mpc_config_t.
 */
BYUL_API void mpc_config_assign(mpc_config_t* out, const mpc_config_t* src);

/**
 * @brief Multi-waypoint target route.
 */
typedef struct s_mpc_target_route {
    const vec3_t* points;
    int count;
    bool loop;
} mpc_target_route_t;

BYUL_API void mpc_target_route_init(mpc_target_route_t* route);
BYUL_API void mpc_target_route_init_full(mpc_target_route_t* route,
                                const vec3_t* points,
                                int count,
                                bool loop);
BYUL_API void mpc_target_route_assign(mpc_target_route_t* out,
                           const mpc_target_route_t* src);

/**
 * @brief Direction-holding target.
 */
typedef struct s_mpc_direction_target {
    vec3_t direction;
    quat_t orientation;
    float weight_dir;
    float weight_rot;
    float duration;
} mpc_direction_target_t;

BYUL_API void mpc_direction_target_init(mpc_direction_target_t* target);
void mpc_direction_target_init_full(mpc_direction_target_t* target,
                                    const vec3_t* direction,
                                    const quat_t* orientation,
                                    float weight_dir,
                                    float weight_rot,
                                    float duration);
void mpc_direction_target_assign(mpc_direction_target_t* out,
                               const mpc_direction_target_t* src);

/**
 * @brief MPC output result structure.
 */
typedef struct s_mpc_output {
    vec3_t desired_accel;
    vec3_t desired_ang_accel;
    motion_state_t future_state;
    float cost;
} mpc_output_t;

/**
 * @brief Cost function type for MPC.
 */
typedef float (*mpc_cost_func)(
    const motion_state_t* sim_state,
    const motion_state_t* target,
    void* userdata);

BYUL_API float numeq_mpc_cost_default(
    const motion_state_t* sim_state,
    const motion_state_t* target,
    void* userdata);

BYUL_API float numeq_mpc_cost_speed(
    const motion_state_t* sim_state,
    const motion_state_t* target,
    void* userdata);

BYUL_API float numeq_mpc_cost_hybrid(
    const motion_state_t* sim_state,
    const motion_state_t* target,
    void* userdata);

// ---------------------------------------------------------
// Main MPC Functions
// ---------------------------------------------------------

BYUL_API bool numeq_mpc_solve(
    const motion_state_t* current_state,
    const motion_state_t* target_state,
    const environ_t* env,
    const bodyprops_t* body,
    const mpc_config_t* config,
    mpc_output_t* out_result,
    trajectory_t* out_traj,
    mpc_cost_func cost_fn,
    void* cost_userdata);

BYUL_API bool numeq_mpc_solve_fast(
    const motion_state_t* current_state,
    const motion_state_t* target_state,
    const environ_t* env,
    const bodyprops_t* body,
    const mpc_config_t* config,
    mpc_output_t* out_result,
    trajectory_t* out_traj,
    mpc_cost_func cost_fn,
    void* cost_userdata);

BYUL_API bool numeq_mpc_solve_coarse2fine(
    const motion_state_t* current_state,
    const motion_state_t* target_state,
    const environ_t* env,
    const bodyprops_t* body,
    const mpc_config_t* config,
    mpc_output_t* out_result,
    trajectory_t* out_traj,
    mpc_cost_func cost_fn,
    void* cost_userdata);

BYUL_API bool numeq_mpc_solve_route(
    const motion_state_t* current_state,
    const mpc_target_route_t* route,
    const environ_t* env,
    const bodyprops_t* body,
    const mpc_config_t* config,
    mpc_output_t* out_result,
    trajectory_t* out_traj,
    mpc_cost_func cost_fn,
    void* cost_userdata);

BYUL_API bool numeq_mpc_solve_directional(
    const motion_state_t* current_state,
    const mpc_direction_target_t* direction_target,
    const environ_t* env,
    const bodyprops_t* body,
    const mpc_config_t* config,
    mpc_output_t* out_result,
    trajectory_t* out_traj,
    mpc_cost_func cost_fn,
    void* cost_userdata);

#ifdef __cplusplus
}
#endif

#endif // NUMEQ_MPC_H
