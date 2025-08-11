#include "numeq_mpc.h"
#include "vec3.h"
#include "quat.h"
#include <cmath>
#include <cfloat>
#include <new>
#include <cstdio>
#include <cstring>
#include "numeq_integrator.h"

#include <float.h>
#include <math.h>
#include <stdio.h>

#include "numeq_mpc.h"
#include "numeq_integrator.h"
#include "numeq_solver.h"
#include "numeq_model_motion.h"
#include <Eigen/Dense>
#include <vector>
#include <limits>
#include <cmath>

static float quat_angle_diff(const quat_t* a, const quat_t* b) {
    quat_t inv_b;
    quat_inverse(&inv_b, b);

    quat_t rel;
    quat_mul(&rel, a, &inv_b);

    float angle = 2.0f * std::acos(fabsf(rel.w));
    return angle;
}

static void simulate_trajectory(
    const motion_state_t* start,
    const mpc_config_t* config,
    trajectory_t* out_traj)
{
    if (!out_traj || config->horizon_sec <= 0.0f || config->step_dt <= 0.0f)
        return;

    int steps = static_cast<int>(config->horizon_sec / config->step_dt);
    if (steps <= 0) return;

    trajectory_clear(out_traj);

    motion_state_t state = *start;

    for (int i = 0; i < steps; ++i) {

        vec3_t scaled_accel;
        vec3_scale(&scaled_accel, &state.linear.acceleration, config->step_dt);
        vec3_add(&state.linear.velocity, &state.linear.velocity, &scaled_accel);

        float speed = vec3_length(&state.linear.velocity);
        if (config->max_speed > 0.0f && speed > config->max_speed) {
            vec3_scale(&state.linear.velocity, &state.linear.velocity, config->max_speed / speed);
        }

        vec3_t scaled_vel;
        vec3_scale(&scaled_vel, &state.linear.velocity, config->step_dt);
        vec3_add(&state.linear.position, &state.linear.position, &scaled_vel);

        vec3_t scaled_ang_accel;
        vec3_scale(&scaled_ang_accel, &state.angular.angular_acceleration, config->step_dt);
        vec3_add(&state.angular.angular_velocity, &state.angular.angular_velocity, &scaled_ang_accel);

        float ang_speed = vec3_length(&state.angular.angular_velocity);
        if (config->max_ang_speed > 0.0f && ang_speed > config->max_ang_speed) {
            vec3_scale(&state.angular.angular_velocity, &state.angular.angular_velocity, config->max_ang_speed / ang_speed);
        }

        quat_t delta_rot;
        quat_init_angular_velocity(&delta_rot, &state.angular.angular_velocity, config->step_dt);
        quat_mul(&state.angular.orientation, &delta_rot, &state.angular.orientation);
        quat_unit(&state.angular.orientation, &state.angular.orientation);

        trajectory_add_sample(out_traj, i * config->step_dt, &state);
    }
}

void mpc_config_init(mpc_config_t* cfg) {
    if (!cfg) return;
    cfg->horizon_sec = 1.0f;
    cfg->step_dt = 0.05f;
    cfg->max_accel = 10.0f;
    cfg->max_ang_accel = 5.0f;
    cfg->max_speed = 50.0f;
    cfg->max_ang_speed = 10.0f;
    cfg->weight_distance = 1.0f;
    cfg->weight_orientation = 0.5f;
    cfg->weight_velocity = 0.1f;
    cfg->weight_accel = 0.1f;
    cfg->weight_ang_accel = 0.1f;
    cfg->max_iter = 10;
    cfg->output_trajectory = false;
    cfg->candidate_step = 0.5f;
    cfg->ang_candidate_step = 0.1f;
}

void mpc_config_init_full(mpc_config_t* cfg,
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
                          float ang_candidate_step) {
    if (!cfg) return;
    cfg->horizon_sec = horizon_sec;
    cfg->step_dt = step_dt;
    cfg->max_accel = max_accel;
    cfg->max_ang_accel = max_ang_accel;
    cfg->max_speed = max_speed;
    cfg->max_ang_speed = max_ang_speed;
    cfg->weight_distance = weight_distance;
    cfg->weight_orientation = weight_orientation;
    cfg->weight_velocity = weight_velocity;
    cfg->weight_accel = weight_accel;
    cfg->weight_ang_accel = weight_ang_accel;
    cfg->max_iter = max_iter;
    cfg->output_trajectory = output_trajectory;
    cfg->candidate_step = candidate_step;
    cfg->ang_candidate_step = ang_candidate_step;
}

void mpc_config_assign(mpc_config_t* out, const mpc_config_t* src) {
    if (!out || !src) return;
    *out = *src;
}

void mpc_target_route_init(mpc_target_route_t* route) {
    if (!route) return;
    route->points = NULL;
    route->count = 0;
    route->loop = false;
}

void mpc_target_route_init_full(mpc_target_route_t* route,
                                const vec3_t* points,
                                int count,
                                bool loop) {
    if (!route) return;
    route->points = points;
    route->count = count;
    route->loop = loop;
}

void mpc_target_route_assign(mpc_target_route_t* out,
                           const mpc_target_route_t* src) {
    if (!out || !src) return;
    *out = *src;
}

void mpc_direction_target_init(mpc_direction_target_t* target) {
    if (!target) return;
    target->direction = vec3_t{1.0f, 0.0f, 0.0f};
    quat_identity(&target->orientation);
    target->weight_dir = 1.0f;
    target->weight_rot = 0.5f;
    target->duration = 1.0f;
}

void mpc_direction_target_init_full(mpc_direction_target_t* target,
                                    const vec3_t* direction,
                                    const quat_t* orientation,
                                    float weight_dir,
                                    float weight_rot,
                                    float duration) {
    if (!target) return;
    target->direction = *direction;
    target->orientation = *orientation;
    target->weight_dir = weight_dir;
    target->weight_rot = weight_rot;
    target->duration = duration;
}

void mpc_direction_target_assign(mpc_direction_target_t* out,
                               const mpc_direction_target_t* src) {
    if (!out || !src) return;
    *out = *src;
}

float numeq_mpc_cost_default(
    const motion_state_t* sim_state,
    const motion_state_t* target,
    void* userdata)
{
    const mpc_config_t* cfg = static_cast<const mpc_config_t*>(userdata);

    vec3_t diff_pos;
    vec3_sub(&diff_pos, &sim_state->linear.position, &target->linear.position);

    float angle_diff = quat_angle_diff(
        &sim_state->angular.orientation, &target->angular.orientation);

    float w_dist  = cfg ? cfg->weight_distance : 1.0f;
    float w_rot   = cfg ? cfg->weight_orientation : 1.0f;
    float w_acc   = cfg ? cfg->weight_accel : 0.1f;
    float w_ang   = cfg ? cfg->weight_ang_accel : 0.1f;

  return w_dist * vec3_length_sq(&diff_pos)
         + w_rot  * (angle_diff * angle_diff)
         + w_acc  * vec3_length_sq(&sim_state->linear.acceleration)
         + w_ang  * vec3_length_sq(&sim_state->angular.angular_acceleration);
}

float numeq_mpc_cost_speed(
    const motion_state_t* sim_state,
    const motion_state_t* target,
    void* userdata)
{
    const mpc_config_t* cfg = static_cast<const mpc_config_t*>(userdata);
    float current_speed = vec3_length(&sim_state->linear.velocity);
    float target_speed  = target->linear.velocity.x;

    float dv = current_speed - target_speed;

    float w_speed = cfg ? cfg->weight_distance : 1.0f;
    float w_accel = cfg ? cfg->weight_accel : 0.1f;

    return w_speed * dv * dv + w_accel * 
        vec3_length_sq(&sim_state->linear.acceleration);
}

float numeq_mpc_cost_hybrid(
    const motion_state_t* sim_state,
    const motion_state_t* target,
    void* userdata)
{
    const mpc_config_t* cfg = static_cast<const mpc_config_t*>(userdata);

    vec3_t diff_pos;
    vec3_sub(&diff_pos, &sim_state->linear.position, &target->linear.position);

    vec3_t diff_vel;
    vec3_sub(&diff_vel, &sim_state->linear.velocity, &target->linear.velocity);

    float angle_diff = quat_angle_diff(&sim_state->angular.orientation, 
        &target->angular.orientation);

    float w_dist  = cfg ? cfg->weight_distance : 1.0f;
    float w_vel   = cfg ? cfg->weight_velocity : 1.0f;
    float w_rot   = cfg ? cfg->weight_orientation : 1.0f;
    float w_acc   = cfg ? cfg->weight_accel : 0.1f;
    float w_ang   = cfg ? cfg->weight_ang_accel : 0.1f;

    return w_dist * vec3_length_sq(&diff_pos)
         + w_vel  * vec3_length_sq(&diff_vel)
         + w_rot  * (angle_diff * angle_diff)
         + w_acc  * vec3_length_sq(&sim_state->linear.acceleration)
         + w_ang  * vec3_length_sq(&sim_state->angular.angular_acceleration);
}

bool numeq_mpc_solve(
    const motion_state_t* current_state,
    const motion_state_t* target_state,
    const environ_t* env,
    const bodyprops_t* body,
    const mpc_config_t* config,
    mpc_output_t* out_result,
    trajectory_t* out_traj,
    mpc_cost_func cost_fn,
    void* cost_userdata)
{
    if (!current_state || !target_state || !config || !out_result)
        return false;

    float accel_candidates[3] = {-config->max_accel, 0.0f, config->max_accel};
    float ang_candidates[3]   = {
        -config->max_ang_accel, 0.0f, config->max_ang_accel};

    const int horizon_sec = 
        (config->horizon_sec > 0) ? config->horizon_sec : 10;

    const float dt = (config->step_dt > 0.0f) ? config->step_dt : 0.016f;

    float best_cost = FLT_MAX;
    vec3_t best_accel = {0, 0, 0};
    vec3_t best_ang_accel = {0, 0, 0};

    for (int ix = 0; ix < 3; ix++) {
        for (int iy = 0; iy < 3; iy++) {
            for (int iz = 0; iz < 3; iz++) {
                for (int rx = 0; rx < 3; rx++) {
                    for (int ry = 0; ry < 3; ry++) {
                        for (int rz = 0; rz < 3; rz++) {

                            vec3_t accel = {accel_candidates[ix],
                                            accel_candidates[iy],
                                            accel_candidates[iz]};

                            vec3_t ang_accel = {ang_candidates[rx],
                                                ang_candidates[ry],
                                                ang_candidates[rz]};

                            motion_state_t sim_state = *current_state;

                            float total_cost = 0.0f;

                            for (int step = 0; step < horizon_sec; step++) {
                                vec3_t external_accel = {0, 0, 0};
                                if (env && body) {
                                    numeq_model_motion_accel(&sim_state,
                                        env, body, dt, &external_accel);
                                } else if (env) {
                                    environ_distort_accel_except_gravity(env, true, &external_accel);
                                }

                                vec3_t total_accel;
                                vec3_add(
                                    &total_accel, &accel, &external_accel);

                                sim_state.linear.acceleration = total_accel;
                                sim_state.angular.angular_acceleration = ang_accel;

                                integrator_step_motion_rk4(&sim_state, dt);

                                if (cost_fn) {
                                    total_cost += cost_fn(
                                        &sim_state, 
                                        target_state, cost_userdata);
                                }
                            }

                            if (total_cost < best_cost) {
                                best_cost = total_cost;
                                best_accel = accel;
                                best_ang_accel = ang_accel;
                            }
                        }
                    }
                }
            }
        }
    }

    out_result->desired_accel = best_accel;
    out_result->desired_ang_accel = best_ang_accel;
    out_result->cost = best_cost;

    if (out_traj && config->output_trajectory) {
        motion_state_t sim_state = *current_state;
        trajectory_clear(out_traj);

        for (int step = 0; step < horizon_sec; step++) {
            vec3_t external_accel = {0, 0, 0};
            if (env && body) {
                numeq_model_motion_accel(&sim_state,
                    env, body, dt, &external_accel);

            } else if (env) {
                environ_distort_accel_except_gravity(env, true, &external_accel);
            }

            vec3_t total_accel;
            vec3_add(&total_accel, &best_accel, &external_accel);
            sim_state.linear.acceleration = total_accel;
            sim_state.angular.angular_acceleration = best_ang_accel;

            integrator_step_motion_rk4(&sim_state, dt);
            trajectory_add_sample(out_traj, step * dt, &sim_state);
        }

        if (out_traj->count > 0) {
            out_result->future_state = 
                out_traj->samples[out_traj->count - 1].state;

        }
    } else {
        out_result->future_state = *target_state;
    }

    return true;
}

static inline float signf(float x) { return (x > 0.0f) - (x < 0.0f); }

bool numeq_mpc_solve_fast(
    const motion_state_t* current_state,
    const motion_state_t* target_state,
    const environ_t* env,
    const bodyprops_t* body,
    const mpc_config_t* config,
    mpc_output_t* out_result,
    trajectory_t* out_traj,
    mpc_cost_func cost_fn,
    void* cost_userdata)
{
    if (!current_state || !target_state || !config || !out_result)
        return false;

    vec3_t error;
    vec3_sub(&error, 
        &target_state->linear.position, &current_state->linear.position);

    float accel_candidates[2] = {0.0f, signf(error.x) * config->max_accel};
    float ang_candidates[2]   = {
        0.0f, signf(error.y) * config->max_ang_accel};

    const int horizon_sec = 
        (config->horizon_sec > 0) ? config->horizon_sec : 5;

    const float dt = (config->step_dt > 0.0f) ? config->step_dt : 0.016f;

    float best_cost = FLT_MAX;
    vec3_t best_accel = {0, 0, 0};
    vec3_t best_ang_accel = {0, 0, 0};

    for (int ix = 0; ix < 2; ix++) {
        for (int iy = 0; iy < 2; iy++) {
            for (int iz = 0; iz < 2; iz++) {
                for (int rx = 0; rx < 2; rx++) {
                    for (int ry = 0; ry < 2; ry++) {
                        for (int rz = 0; rz < 2; rz++) {

                            vec3_t accel = {accel_candidates[ix],
                                            accel_candidates[iy],
                                            accel_candidates[iz]};

                            vec3_t ang_accel = {ang_candidates[rx],
                                                ang_candidates[ry],
                                                ang_candidates[rz]};

                            motion_state_t sim_state = *current_state;
                            float total_cost = 0.0f;

                            for (int step = 0; step < horizon_sec; step++) {

                                vec3_t external_accel = {0, 0, 0};
                                if (env && body) {
                                    numeq_model_motion_accel(&sim_state,
                                        env, body, dt, &external_accel);

                                } else if (env) {
                                    environ_distort_accel_except_gravity(env, true, &external_accel);
                                }

                                vec3_t total_accel;
                                vec3_add(&total_accel, &accel, &external_accel);
                                sim_state.linear.acceleration = total_accel;
                                sim_state.angular.angular_acceleration = ang_accel;

                                integrator_step_motion_rk4(&sim_state, dt);

                                if (cost_fn) {
                                    total_cost += cost_fn(
                                        &sim_state, 
                                        target_state, cost_userdata);

                                    // Early Exit
                                    if (total_cost > best_cost) break;  
                                }
                            }

                            if (total_cost < best_cost) {
                                best_cost = total_cost;
                                best_accel = accel;
                                best_ang_accel = ang_accel;
                            }
                        }
                    }
                }
            }
        }
    }

    out_result->desired_accel = best_accel;
    out_result->desired_ang_accel = best_ang_accel;
    out_result->cost = best_cost;

    if (out_traj && config->output_trajectory) {
        motion_state_t sim_state = *current_state;
        trajectory_clear(out_traj);

        for (int step = 0; step < horizon_sec; step++) {
            vec3_t external_accel = {0, 0, 0};
            if (env && body) {
                numeq_model_motion_accel(&sim_state,
                    env, body, dt, &external_accel);

            } else if (env) {
                environ_distort_accel_except_gravity(env, true, &external_accel);
            }

            vec3_t total_accel;
            vec3_add(&total_accel, &best_accel, &external_accel);
            sim_state.linear.acceleration = total_accel;
            sim_state.angular.angular_acceleration = best_ang_accel;

            integrator_step_motion_rk4(&sim_state, dt);
            trajectory_add_sample(out_traj, step * dt, &sim_state);
        }

        if (out_traj->count > 0) {
            out_result->future_state = 
                out_traj->samples[out_traj->count - 1].state;
        }
    } else {
        out_result->future_state = *target_state;
    }

    return true;
}

static float evaluate_cost(
    const motion_state_t* current_state,
    const motion_state_t* target_state,
    const environ_t* env,
    const bodyprops_t* body,
    const vec3_t* accel,
    const vec3_t* ang_accel,
    int horizon_sec,
    float dt,
    mpc_cost_func cost_fn,
    void* cost_userdata)
{
    motion_state_t sim_state = *current_state;
    sim_state.angular.angular_acceleration = *ang_accel;

    float total_cost = 0.0f;
    for (int step = 0; step < horizon_sec; step++) {

        vec3_t external_accel = {0, 0, 0};
        if (env && body) {
            numeq_model_motion_accel(&sim_state,
                env, body, dt, &external_accel);

        } else if (env) {
            environ_distort_accel_except_gravity(env, true, &external_accel);
        }

        vec3_t total_accel;
        vec3_add(&total_accel, accel, &external_accel);
        sim_state.linear.acceleration = total_accel;

        integrator_step_motion_rk4(&sim_state, dt);

        if (cost_fn) {
            total_cost += cost_fn(&sim_state, target_state, cost_userdata);
        }
    }
    return total_cost;
}

bool numeq_mpc_solve_coarse2fine(
    const motion_state_t* current_state,
    const motion_state_t* target_state,
    const environ_t* env,
    const bodyprops_t* body,
    const mpc_config_t* config,
    mpc_output_t* out_result,
    trajectory_t* out_traj,
    mpc_cost_func cost_fn,
    void* cost_userdata)
{
    if (!current_state || !target_state || !config || !out_result)
        return false;

    const int horizon_sec = (config->horizon_sec > 0) ? config->horizon_sec : 5;
    const float dt = (config->step_dt > 0.0f) ? config->step_dt : 0.016f;

    float coarse_candidates[3] = {
        -config->max_accel, 0.0f, config->max_accel};

    float coarse_ang_candidates[3] = {
        -config->max_ang_accel, 0.0f, config->max_ang_accel};

    float best_cost = FLT_MAX;
    vec3_t best_accel = {0, 0, 0};
    vec3_t best_ang_accel = {0, 0, 0};

    for (int ix = 0; ix < 3; ix++) {
        for (int iy = 0; iy < 3; iy++) {
            for (int iz = 0; iz < 3; iz++) {
                for (int rx = 0; rx < 3; rx++) {
                    for (int ry = 0; ry < 3; ry++) {
                        for (int rz = 0; rz < 3; rz++) {
                            vec3_t accel = {coarse_candidates[ix],
                                            coarse_candidates[iy],
                                            coarse_candidates[iz]};
                            vec3_t ang_accel = {coarse_ang_candidates[rx],
                                                coarse_ang_candidates[ry],
                                                coarse_ang_candidates[rz]};

                            float cost = evaluate_cost(
                                current_state, target_state, env, body,
                                &accel, &ang_accel,
                                horizon_sec, dt, cost_fn, cost_userdata);

                            if (cost < best_cost) {
                                best_cost = cost;
                                best_accel = accel;
                                best_ang_accel = ang_accel;
                            }
                        }
                    }
                }
            }
        }
    }

    const float delta = config->max_accel * 0.25f;
    float fine_candidates[3] = {-delta, 0.0f, delta};

    for (int ix = 0; ix < 3; ix++) {
        for (int iy = 0; iy < 3; iy++) {
            for (int iz = 0; iz < 3; iz++) {
                vec3_t accel = {best_accel.x + fine_candidates[ix],
                                best_accel.y + fine_candidates[iy],
                                best_accel.z + fine_candidates[iz]};
                vec3_t ang_accel = best_ang_accel;

                float cost = evaluate_cost(
                    current_state, target_state, env, body,
                    &accel, &ang_accel,
                    horizon_sec, dt, cost_fn, cost_userdata);

                if (cost < best_cost) {
                    best_cost = cost;
                    best_accel = accel;
                }
            }
        }
    }

    out_result->desired_accel = best_accel;
    out_result->desired_ang_accel = best_ang_accel;
    out_result->cost = best_cost;

    if (out_traj && config->output_trajectory) {
        motion_state_t sim_state = *current_state;
        trajectory_clear(out_traj);

        for (int step = 0; step < horizon_sec; step++) {
            vec3_t external_accel = {0, 0, 0};
            if (env && body) {
                numeq_model_motion_accel(&sim_state,
                    env, body, dt, &external_accel);

            } else if (env) {
                environ_distort_accel_except_gravity(env, true, &external_accel);
            }

            vec3_t total_accel;
            vec3_add(&total_accel, &best_accel, &external_accel);
            sim_state.linear.acceleration = total_accel;
            sim_state.angular.angular_acceleration = best_ang_accel;

            integrator_step_motion_rk4(&sim_state, dt);
            trajectory_add_sample(out_traj, step * dt, &sim_state);
        }

        if (out_traj->count > 0) {
            out_result->future_state
            = out_traj->samples[out_traj->count - 1].state;
        }
    } else {
        out_result->future_state = *target_state;
    }

    return true;
}

bool numeq_mpc_solve_route(
    const motion_state_t* current_state,
    const mpc_target_route_t* route,
    const environ_t* env,
    const bodyprops_t* body,
    const mpc_config_t* config,
    mpc_output_t* out_result,
    trajectory_t* out_traj,
    mpc_cost_func cost_fn,
    void* cost_userdata)
{
    if (!route || route->count <= 0) return false;

    const vec3_t* next_target = &route->points[0];
    float best_dist = FLT_MAX;

    for (int i = 0; i < route->count; ++i) {
        vec3_t diff;
        vec3_sub(&diff, 
            &current_state->linear.position, &route->points[i]);

        float dist_sq = vec3_length_sq(&diff);
        if (dist_sq < best_dist) {
            best_dist = dist_sq;
            next_target = &route->points[i];
        }
    }

    motion_state_t target_state = *current_state;
    target_state.linear.position = *next_target;

    return numeq_mpc_solve(
        current_state,
        &target_state,
        env,
        body,
        config,
        out_result,
        out_traj,
        cost_fn,
        cost_userdata);
}

bool numeq_mpc_solve_directional(
    const motion_state_t* current_state,
    const mpc_direction_target_t* direction_target,
    const environ_t* env,
    const bodyprops_t* body,
    const mpc_config_t* config,
    mpc_output_t* out_result,
    trajectory_t* out_traj,
    mpc_cost_func cost_fn,
    void* cost_userdata)
{
    if (!direction_target) return false;

    vec3_t scaled_dir;
    vec3_scale(&scaled_dir, &direction_target->direction,
               direction_target->duration 
               * config->step_dt 
               * config->max_speed);

    motion_state_t target_state = *current_state;
    vec3_add(&target_state.linear.position, 
        &current_state->linear.position, &scaled_dir);
        
    target_state.angular.orientation = direction_target->orientation;

    return numeq_mpc_solve(
        current_state,
        &target_state,
        env,
        body,
        config,
        out_result,
        out_traj,
        cost_fn,
        cost_userdata);
}
