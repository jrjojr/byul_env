#include "internal/numeq_mpc.h"
#include "internal/numeq_model.h"
#include "internal/numeq_integrator.h"
#include "internal/vec3.hpp"
#include <cmath>
#include <cstring>
#include <limits>
#include <vector>

// ---------------------------------------------------------
// 기본 비용 함수 (거리 오차 + 가속도 제어 비용)
// ---------------------------------------------------------
float numeq_mpc_cost_default(const state_vector_t* sim_state,
                             const vec3_t* target,
                             const vec3_t* accel,
                             void* userdata) {
    if (!sim_state || !target || !accel) return 1e9f;  // 입력 자체 오류 방지

    const mpc_config_t* cfg = (const mpc_config_t*)userdata;
    float weight_dist = 1.0f;
    float weight_acc = 1.0f;

    if (cfg) {
        weight_dist = cfg->weight_distance;
        weight_acc  = cfg->weight_accel;
    }

    float dist = vec3_distance(&sim_state->position, target);
    float accel_len = vec3_length(accel);

    return weight_dist * dist * dist +
           weight_acc * accel_len * accel_len;
}

// ---------------------------------------------------------
// MPC trajectory 초기화
// ---------------------------------------------------------
bool mpc_trajectory_init(mpc_trajectory_t* traj, int capacity) {
    if (capacity <= 0) return false;
    traj->samples = new trajectory_sample_t[capacity];
    traj->count = 0;
    traj->capacity = capacity;
    return true;
}

void mpc_trajectory_free(mpc_trajectory_t* traj) {
    if (traj->samples) {
        delete[] traj->samples;
        traj->samples = nullptr;
    }
    traj->count = 0;
    traj->capacity = 0;
}

// ---------------------------------------------------------
// 단일 목표 MPC 계산 함수
// ---------------------------------------------------------
bool numeq_mpc_solve(const state_vector_t* current_state,
                     const vec3_t* target,
                     const environment_t* env,
                     const body_properties_t* body,
                     const mpc_config_t* config,
                     mpc_output_t* out_result,
                     mpc_trajectory_t* out_traj,
                     mpc_cost_func cost_fn,
                     void* cost_userdata) {
    if (!current_state || !target || !env || !body || !config || !out_result) 
        return false;

    const int steps = static_cast<int>(config->horizon_sec / config->step_dt);
    if (steps <= 0) return false;

    std::vector<Vec3> accel_candidates;

    const float a = config->max_accel;
    accel_candidates.emplace_back( 0,  0,  0); // 정지
    accel_candidates.emplace_back( a,  0,  0);
    accel_candidates.emplace_back(-a,  0,  0);
    accel_candidates.emplace_back( 0,  a,  0);
    accel_candidates.emplace_back( 0, -a,  0);
    accel_candidates.emplace_back( 0,  0,  a);
    accel_candidates.emplace_back( 0,  0, -a);

    float best_cost = std::numeric_limits<float>::max();
    Vec3 best_accel;
    Vec3 best_future;

    mpc_trajectory_t* traj_out = config->output_trajectory ? out_traj : nullptr;

    for (const Vec3& candidate : accel_candidates) {
        state_vector_t sim = *current_state;

        mpc_trajectory_t local_traj;
        if (traj_out) {
            mpc_trajectory_init(&local_traj, steps);
        }

        float total_cost = 0.0f;
        for (int i = 0; i < steps; ++i) {
            integrator_config_t c = integrator_config_t{
                INTEGRATOR_SEMI_IMPLICIT, config->step_dt};
                
            numeq_integrate(&sim, &candidate.v, &c);

            float cost = cost_fn(&sim, target, &candidate.v, cost_userdata);
            total_cost += cost;

            if (traj_out && i < local_traj.capacity) {
                local_traj.samples[i].t = i * config->step_dt;
                local_traj.samples[i].state = sim;
                local_traj.count++;
            }

            if (config->max_speed > 0.0f &&
                vec3_length(&sim.velocity) > config->max_speed) {
                total_cost += 10000.0f; // 속도 초과 패널티
                break;
            }
        }

        if (total_cost < best_cost) {
            best_cost = total_cost;
            best_accel = candidate;
            best_future = Vec3(sim.position);

            if (traj_out) {
                *traj_out = local_traj;
            }
        } else {
            if (traj_out) {
                mpc_trajectory_free(&local_traj);
            }
        }
    }

    out_result->desired_accel = best_accel;
    out_result->future_target = best_future;
    out_result->cost = best_cost;
    return true;
}

bool numeq_mpc_solve_route(
    const state_vector_t* current_state,
    const mpc_target_route_t* route,
    const environment_t* env,
    const body_properties_t* body,
    const mpc_config_t* config,
    mpc_output_t* out_result,
    mpc_trajectory_t* out_traj,
    mpc_cost_func cost_fn,
    void* cost_userdata)
{
    if (!route || route->count <= 0) return false;

    float best_cost = std::numeric_limits<float>::max();
    mpc_output_t best_output = {};
    mpc_trajectory_t best_traj = {};

    for (int i = 0; i < route->count; ++i) {
        const vec3_t* waypoint = &route->points[i];

        mpc_output_t temp_output = {};
        mpc_trajectory_t temp_traj = {};

        bool ok = numeq_mpc_solve(current_state, waypoint,
                env, body, config,
                &temp_output,
                config->output_trajectory ? &temp_traj : nullptr,
                cost_fn, cost_userdata);

        if (!ok) continue;

        if (temp_output.cost < best_cost) {
            best_cost = temp_output.cost;
            best_output = temp_output;

            if (config->output_trajectory) {
                if (best_traj.samples) {
                    mpc_trajectory_free(&best_traj);
                }
                best_traj = temp_traj;
            }
        } else if (config->output_trajectory) {
            mpc_trajectory_free(&temp_traj);
        }

        if (!route->loop) break;  // 반복 안 하면 첫 번째만 사용
    }

    *out_result = best_output;
    if (config->output_trajectory && out_traj) {
        *out_traj = best_traj;
    }

    return true;
}

bool numeq_mpc_solve_directional(
    const state_vector_t* current_state,
    const mpc_direction_target_t* direction_target,
    const environment_t* env,
    const body_properties_t* body,
    const mpc_config_t* config,
    mpc_output_t* out_result,
    mpc_trajectory_t* out_traj,
    mpc_cost_func cost_fn,
    void* cost_userdata)
{
    if (!direction_target || vec3_length(&direction_target->direction) == 0.0f)
        return false;

    Vec3 norm_dir = Vec3(direction_target->direction).normalized();
    float total_time = direction_target->duration;

    // 목표 위치 = 현재 위치 + 정규화 방향 * 거리
    Vec3 p0(current_state->position);
    Vec3 future_pos = p0 + norm_dir * body->mass * total_time;

    return numeq_mpc_solve(current_state, &future_pos.v,
                           env, body, config,
                           out_result, out_traj,
                           cost_fn, cost_userdata);
}
