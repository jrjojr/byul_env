#include <cmath>
#include <cstdio>
#include "internal/projectile_predict.h"

// ---------------------------------------------------------
// projectile_result_create
// ---------------------------------------------------------
projectile_result_t* projectile_result_create() {
    projectile_result_t* res = new projectile_result_t;
    if (!res) return nullptr;

    res->impact_time = 0.0f;
    vec3_zero(&res->impact_pos);
    res->valid = false;

    res->trajectory = trajectory_create();  // 기본 capacity = 100
    if (!res->trajectory) {
        delete res;
        return nullptr;
    }
    return res;
}

// ---------------------------------------------------------
// projectile_result_create_full
// ---------------------------------------------------------
projectile_result_t* projectile_result_create_full(int capacity) {
    if (capacity <= 0) return nullptr;

    projectile_result_t* res = new projectile_result_t;
    if (!res) return nullptr;

    res->impact_time = 0.0f;
    vec3_zero(&res->impact_pos);
    res->valid = false;

    res->trajectory = trajectory_create_full(capacity);
    if (!res->trajectory) {
        delete res;
        return nullptr;
    }
    return res;
}

// ---------------------------------------------------------
// projectile_result_copy
// ---------------------------------------------------------
projectile_result_t* projectile_result_copy(const projectile_result_t* src) {
    if (!src) return nullptr;

    projectile_result_t* res = new projectile_result_t;
    if (!res) return nullptr;

    res->impact_time = src->impact_time;
    res->impact_pos = src->impact_pos;
    res->valid = src->valid;

    res->trajectory = trajectory_copy(src->trajectory);
    if (!res->trajectory) {
        delete res;
        return nullptr;
    }
    return res;
}

// ---------------------------------------------------------
// projectile_result_destroy
// ---------------------------------------------------------
void projectile_result_destroy(projectile_result_t* res) {
    if (!res) return;
    if (res->trajectory) {
        trajectory_destroy(res->trajectory);
        res->trajectory = nullptr;
    }
    delete res;
}

bool projectile_predict(
    projectile_result_t* out,
    const projectile_t* proj,
    const propulsion_t* propulsion,
    projectile_guidance_func guidance_fn,
    void* guidance_userdata,        // 유도 함수에 전달할 사용자 데이터
    target_info_t* target_info,     // 동적 타겟 정보
    float max_time,
    float time_step,
    environ_func env_fn,
    void* env_userdata)
{
    if (!proj || !out || time_step <= 0.0f) return false;
    trajectory_clear(out->trajectory);

    const int debug_mode = 1;
    float t = 0.0f;
    int step_count = 0;
    const float target_radius = 1.0f;  // 명중 판정 거리
    const float mass = (proj->base.props.mass > 0.0f) ?
        proj->base.props.mass : 1.0f;

    // 초기 발사체 상태
    motion_state_t state;
    entity_dynamic_to_motion_state(&proj->base, &state, nullptr, nullptr);

    projectile_t temp_proj = *proj;
    float fuel = (propulsion) ? propulsion->fuel_remaining : 0.0f;
    const int max_steps = static_cast<int>(ceilf(max_time / time_step));

    if (debug_mode) {
        printf("=== projectile_predict DEBUG START ===\n");
        printf("Start Pos: (%.2f, %.2f, %.2f)\n",
               state.linear.position.x,
               state.linear.position.y,
               state.linear.position.z);
        printf("Start Vel: (%.2f, %.2f, %.2f)\n",
               state.linear.velocity.x,
               state.linear.velocity.y,
               state.linear.velocity.z);
        if (propulsion) {
            printf("Fuel: %.2f, Max Thrust: %.2f\n",
                   propulsion->fuel_remaining, propulsion->max_thrust);
        }
    }

    // -----------------------------
    // 시뮬레이션 루프
    // -----------------------------
    for (; t <= max_time; t += time_step) {
        if (++step_count > max_steps) {
            fprintf(stderr,
                "[ERROR] projectile_predict: step overflow (t=%.2f)\n", t);
            break;
        }

        // 발사체 상태 갱신
        xform_set_position(&temp_proj.base.xf, &state.linear.position);
        temp_proj.base.velocity = state.linear.velocity;

        // -----------------------------
        // 타겟 위치 갱신
        // -----------------------------
        vec3_t target_pos = {0.0f, 0.0f, 0.0f};
        bool has_target = (target_info && target_info->target);
        if (has_target) {
            xform_get_position(&target_info->target->xf, &target_pos);
        }

        // -----------------------------
        // 유도 벡터 계산
        // -----------------------------
        vec3_t guidance = {0, 0, 0};
        if (guidance_fn) {
            const vec3_t* g = guidance_fn(
                &temp_proj, time_step,
                guidance_userdata ? guidance_userdata : (void*)target_info,
                nullptr);
            if (g) vec3_unit(&guidance, g);
            else guidance = {0, 1, 0}; // fallback
        }

        // -----------------------------
        // 추진력 계산
        // -----------------------------
        vec3_t thrust_accel = {0, 0, 0};
        if (propulsion && fuel > 0.0f) {
            float thrust = propulsion_get_thrust(propulsion);
            vec3_scale(&thrust_accel, &guidance, thrust / mass);
            fuel -= propulsion->burn_rate * time_step;
        }

        // -----------------------------
        // 환경 가속도
        // -----------------------------
        vec3_t env_accel = {0, 0, 0};
        if (env_fn) {
            env_fn(&env_accel, time_step, env_userdata);
        }

        // 총 가속도 = 추진 + 환경
        vec3_zero(&state.linear.acceleration);
        vec3_add(&state.linear.acceleration, &env_accel, &thrust_accel);

        // trajectory 기록
        trajectory_add_sample(out->trajectory, t, &state);

        // -----------------------------
        // 위치/속도 적분
        // -----------------------------
        vec3_t pos_prev = state.linear.position;
        integrator_config_t config;
        integrator_config_init(&config);
        config.time_step = time_step;
        
        numeq_integrate(&state, &config);

        // -----------------------------
        // 충돌 감지
        // -----------------------------
        bool hit_detected = false;
        vec3_t impact_pos = state.linear.position;
        float impact_time = t;

        if (has_target) {
            float distance = vec3_distance(&state.linear.position, &target_pos);
            if (distance <= target_radius) {
                hit_detected = true;
            }
        } else if (state.linear.position.y <= 0.0f) {
            float dy = state.linear.position.y - pos_prev.y;
            float alpha = (fabsf(dy) > 1e-6f) ? -pos_prev.y / dy : 0.0f;
            impact_time = t + alpha * time_step;
            vec3_lerp(&impact_pos, &pos_prev, &state.linear.position, alpha);
            impact_pos.y = 0.0f;
            hit_detected = true;
        }

        if (hit_detected) {
            out->impact_time = impact_time;
            out->impact_pos = impact_pos;
            out->valid = true;

            if (debug_mode) {
                printf("Impact detected at t=%.3f Pos:(%.2f,%.2f,%.2f)\n",
                       impact_time, impact_pos.x, impact_pos.y, impact_pos.z);
            }
            return true;
        }
    }

    // 충돌 없음
    out->valid = false;
    if (debug_mode) {
        printf("No impact detected (max_time=%.2f)\n", max_time);
        printf("=== projectile_predict DEBUG END ===\n");
    }
    return false;
}
