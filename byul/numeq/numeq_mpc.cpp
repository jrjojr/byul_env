#include "internal/numeq_mpc.h"
#include "internal/vec3.h"
#include "internal/quat.h"
#include <cmath>
#include <cfloat>
#include <new>
#include <cstdio>
#include <cstring>

// ---------------------------------------------------------
// 내부 유틸
// ---------------------------------------------------------

static float quat_angle_diff(const quat_t* a, const quat_t* b) {
    quat_t inv_b;
    quat_inverse(&inv_b, b);

    quat_t rel;
    quat_mul(&rel, a, &inv_b);

    float angle = 2.0f * std::acos(fabsf(rel.w));
    return angle;  // 라디안 값
}

// trajectory 샘플링 (선형 + 회전)
static void simulate_trajectory(
    const motion_state_t* start,
    const vec3_t& accel,
    const vec3_t& ang_accel,
    const mpc_config_t* config,
    trajectory_t* out_traj)
{
    if (!out_traj || config->horizon_sec <= 0.0f || config->step_dt <= 0.0f) return;

    int steps = static_cast<int>(config->horizon_sec / config->step_dt);
    if (steps <= 0) return;

    trajectory_clear(out_traj);

    motion_state_t state = *start;

    for (int i = 0; i < steps; ++i) {
        // 선형 속도 업데이트
        vec3_t scaled_accel;
        vec3_scale(&scaled_accel, &accel, config->step_dt);
        vec3_add(&state.linear.velocity, &state.linear.velocity, &scaled_accel);

        // 선형 속도 제한
        float speed = vec3_length(&state.linear.velocity);
        if (config->max_speed > 0.0f && speed > config->max_speed) {
            vec3_scale(&state.linear.velocity, &state.linear.velocity, config->max_speed / speed);
        }

        // 선형 위치 업데이트
        vec3_t scaled_vel;
        vec3_scale(&scaled_vel, &state.linear.velocity, config->step_dt);
        vec3_add(&state.linear.position, &state.linear.position, &scaled_vel);

        // 각속도 업데이트
        vec3_t scaled_ang_accel;
        vec3_scale(&scaled_ang_accel, &ang_accel, config->step_dt);
        vec3_add(&state.angular.angular_velocity, &state.angular.angular_velocity, &scaled_ang_accel);

        // 각속도 제한
        float ang_speed = vec3_length(&state.angular.angular_velocity);
        if (config->max_ang_speed > 0.0f && ang_speed > config->max_ang_speed) {
            vec3_scale(&state.angular.angular_velocity, &state.angular.angular_velocity, config->max_ang_speed / ang_speed);
        }

        // 회전 업데이트 (쿼터니언 적분)
        quat_t delta_rot;
        quat_from_angular_velocity(&delta_rot, &state.angular.angular_velocity, config->step_dt);
        quat_mul(&state.angular.orientation, &delta_rot, &state.angular.orientation);
        quat_normalize(&state.angular.orientation, &state.angular.orientation);

        trajectory_add_sample(out_traj, i * config->step_dt, &state);
    }
}

// 비용 함수 평가
static float evaluate_cost(
    const motion_state_t* current,
    const motion_state_t* target,
    const vec3_t* accel,
    const vec3_t* ang_accel,
    mpc_cost_func cost_fn,
    void* cost_userdata)
{
    if (cost_fn) return cost_fn(current, target, accel, ang_accel, cost_userdata);
    return numeq_mpc_cost_default(current, target, accel, ang_accel, cost_userdata);
}

// ---------------------------------------------------------
// 비용 함수 기본 구현
// ---------------------------------------------------------
float numeq_mpc_cost_default(
    const motion_state_t* sim_state,
    const motion_state_t* target,
    const vec3_t* accel,
    const vec3_t* ang_accel,
    void* userdata)
{
    const mpc_config_t* cfg = static_cast<const mpc_config_t*>(userdata);

    // 위치 오차
    vec3_t diff_pos;
    vec3_sub(&diff_pos, &sim_state->linear.position, &target->linear.position);

    // 회전 오차
    float angle_diff = quat_angle_diff(&sim_state->angular.orientation, &target->angular.orientation);

    float w_dist  = cfg ? cfg->weight_distance : 1.0f;
    float w_rot   = cfg ? cfg->weight_orientation : 1.0f;
    float w_acc   = cfg ? cfg->weight_accel : 0.1f;
    float w_ang   = cfg ? cfg->weight_ang_accel : 0.1f;

    return w_dist * vec3_length_sq(&diff_pos)
         + w_rot  * (angle_diff * angle_diff)
         + w_acc  * vec3_length_sq(accel)
         + w_ang  * vec3_length_sq(ang_accel);
}

float numeq_mpc_cost_speed(
    const motion_state_t* sim_state,
    const motion_state_t* target,
    const vec3_t* accel,
    const vec3_t* ang_accel,
    void* userdata)
{
    const mpc_config_t* cfg = static_cast<const mpc_config_t*>(userdata);
    float target_speed = target->linear.velocity.x; // target velocity in x-component

    float v = vec3_length(&sim_state->linear.velocity);
    float dv = v - target_speed;

    float w_acc = cfg ? cfg->weight_accel : 0.1f;
    return dv * dv + w_acc * vec3_length(accel) + vec3_length(ang_accel) * 0.05f;
}

// ---------------------------------------------------------
// 단일 목표 MPC
// ---------------------------------------------------------
bool numeq_mpc_solve(
    const motion_state_t* current_state,
    const motion_state_t* target_state,
    const environment_t* /*env*/,
    const body_properties_t* /*body*/,
    const mpc_config_t* config,
    mpc_output_t* out_result,
    trajectory_t* out_traj,
    mpc_cost_func cost_fn,
    void* cost_userdata)
{
    if (!current_state || !target_state || !config || !out_result) return false;

    float step = (config->candidate_step > 0.0f)
                 ? config->candidate_step
                 : config->max_accel / 2.0f;

    float ang_step = (config->ang_candidate_step > 0.0f)
                     ? config->ang_candidate_step
                     : config->max_ang_accel / 2.0f;

    float best_cost = FLT_MAX;
    vec3_t best_accel = {0, 0, 0};
    vec3_t best_ang_accel = {0, 0, 0};

    for (float ax = -config->max_accel; ax <= config->max_accel; ax += step) {
        for (float ay = -config->max_accel; ay <= config->max_accel; ay += step) {
            for (float az = -config->max_accel; az <= config->max_accel; az += step) {
                for (float rx = -config->max_ang_accel; rx <= config->max_ang_accel; rx += ang_step) {
                    for (float ry = -config->max_ang_accel; ry <= config->max_ang_accel; ry += ang_step) {
                        for (float rz = -config->max_ang_accel; rz <= config->max_ang_accel; rz += ang_step) {
                            vec3_t accel = {ax, ay, az};
                            vec3_t ang_accel = {rx, ry, rz};

                            float cost = evaluate_cost(current_state, target_state, &accel, &ang_accel, cost_fn, cost_userdata);
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

    out_result->desired_accel = best_accel;
    out_result->desired_ang_accel = best_ang_accel;
    out_result->cost = best_cost;

    if (out_traj && config->output_trajectory) {
        simulate_trajectory(current_state, best_accel, best_ang_accel, config, out_traj);
        if (out_traj->count > 0) {
            out_result->future_state = out_traj->samples[out_traj->count - 1].state;
        }
    } else {
        out_result->future_state = *target_state;
    }
    return true;
}

// ---------------------------------------------------------
// 경유점 기반 MPC
// ---------------------------------------------------------
bool numeq_mpc_solve_route(
    const motion_state_t* current_state,
    const mpc_target_route_t* route,
    const environment_t* env,
    const body_properties_t* body,
    const mpc_config_t* config,
    mpc_output_t* out_result,
    trajectory_t* out_traj,
    mpc_cost_func cost_fn,
    void* cost_userdata)
{
    if (!route || route->count <= 0) return false;

    // 가장 가까운 경유점을 단일 목표로 사용
    const vec3_t* next_target = &route->points[0];
    float best_dist = FLT_MAX;

    for (int i = 0; i < route->count; ++i) {
        vec3_t diff;
        vec3_sub(&diff, &current_state->linear.position, &route->points[i]);
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

// ---------------------------------------------------------
// 방향 유지형 MPC
// ---------------------------------------------------------
bool numeq_mpc_solve_directional(
    const motion_state_t* current_state,
    const mpc_direction_target_t* direction_target,
    const environment_t* env,
    const body_properties_t* body,
    const mpc_config_t* config,
    mpc_output_t* out_result,
    trajectory_t* out_traj,
    mpc_cost_func cost_fn,
    void* cost_userdata)
{
    if (!direction_target) return false;

    vec3_t scaled_dir;
    vec3_scale(&scaled_dir, &direction_target->direction,
               direction_target->duration * config->step_dt * config->max_speed);

    motion_state_t target_state = *current_state;
    vec3_add(&target_state.linear.position, &current_state->linear.position, &scaled_dir);
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
