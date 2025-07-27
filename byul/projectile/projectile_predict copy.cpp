#include <cmath>
#include <cstdio>
#include "internal/projectile_predict.h"
#include "internal/guidance.h"
#include "internal/common.h"

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
    entity_dynamic_t* entdyn,
    float max_time,
    float time_step,
    const environ_t* env,
    const propulsion_t* propulsion,
    guidance_func guidance_fn
){
    if (!proj || !out || time_step <= 0.0f) return false;
    trajectory_clear(out->trajectory);

    // 나의 위치와 목표의 위치가 같으면 계산할 필요가 없다.
    // 이경우 추가 계산이 필요할수도 있다 데미지같은거...
    // 나의 생명력이 깍인다거나
    if(vec3_equal(&proj->base.xf.pos, &entdyn->xf.pos)){
        out->impact_time = 0.0f;
        out->impact_pos = entdyn->xf.pos;
        out->valid = true;
        return true;
    }

    // 나와 목표의 거리가 영향 범위내이면 계산할 필요 없다
    // 이경우 추가 계산이 필요할수도 있다 데미지같은거...
    // 나의 생명력이 깍인다거나
    float d = vec3_distance(&proj->base.xf.pos, &entdyn->xf.pos);
    if(float_equal(d, proj->radius)) {
        out->impact_time = 0.0f;
        out->impact_pos = entdyn->xf.pos;
        out->valid = true;
        return true;
    }

    const int debug_mode = 0;
    float t = 0.0f;
    int step_count = 0;
    const float target_radius = proj->radius;// 명중 판정 거리
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
        if (entdyn) {
            xform_get_position(&entdyn->xf, &target_pos);
        }

        // -----------------------------
        // 유도 벡터 계산
        // 유도 위치는 바라보는 방향을 의미한다
        // 최초의 바라보는 방향은 (target - self).normalize()
        // -----------------------------
        vec3_t guidance;
        vec3_sub(&guidance, &entdyn->xf.pos, &proj->base.xf.pos);
        vec3_normalize(&guidance);

        if (guidance_fn) {
            guidance_target_info_t info = {};
            if(env) info.env = *env;
            if(entdyn) info.target = *entdyn;
            vec3_t vec;
            const vec3_t* g = guidance_fn(
                &temp_proj.base, 
                time_step,
                &info,
                &vec);
            if (g) vec3_unit(&guidance, g);
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
        if (env) {
            environ_adjust_accel_gsplit(env, true, &env_accel);
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

        if (entdyn) {
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

bool projectile_compute_launch(
    launch_param_t* out,
    const projectile_t* proj,
    const vec3_t* target,
    float initial_force
) {
    if (!out || !proj || !target) return false;

    vec3_t start;
    xform_get_position(&proj->base.xf, &start);
    vec3_t diff;
    vec3_sub(&diff, target, &start);

    float g = 9.8f; // 중력 가속도 (m/s²)
    float R = sqrtf(diff.x * diff.x + diff.z * diff.z);
    if (R < 1e-6f) return false;

    float Dy = diff.y;

    // 초기 속도 크기 계산: F = m * a, v0 = sqrt(2 * a * R)
    float mass = proj->base.props.mass > 1e-6f ? proj->base.props.mass : 1.0f;
    float a0 = initial_force / mass;
    float v0 = sqrtf(2.0f * a0 * R);

    // 발사각 계산
    float under_sqrt = v0*v0*v0*v0 - g * (g*R*R + 2*Dy*v0*v0);
    if (under_sqrt < 0.0f) return false; // 도달 불가

    float theta = atanf((v0*v0 - sqrtf(under_sqrt)) / (g * R));

    // 수평 방향 단위 벡터
    vec3_t dir = { diff.x / R, 0, diff.z / R };

    // 초기 속도 벡터 계산
    out->vec.x = v0 * cosf(theta) * dir.x;
    out->vec.y = v0 * sinf(theta);
    out->vec.z = v0 * cosf(theta) * dir.z;

    // 예상 도달 시간
    out->dt = R / (v0 * cosf(theta));
    return true;
}

bool projectile_compute_launch_env(
    launch_param_t* out,
    const projectile_t* proj,
    const environ_t* env,
    const vec3_t* target,
    float initial_force
) {
    if (!out || !proj || !env || !target) return false;

    vec3_t start;
    xform_get_position(&proj->base.xf, &start);
    vec3_t diff;
    vec3_sub(&diff, target, &start);

    float R = sqrtf(diff.x * diff.x + diff.z * diff.z);
    if (R < 1e-6f) return false;

    float Dy = diff.y;
    float mass = proj->base.props.mass > 1e-6f ? proj->base.props.mass : 1.0f;
    float a0 = initial_force / mass;  // 초기 가속도
    float g = fabsf(env->gravity.y) > 1e-6f ? fabsf(env->gravity.y) : 9.8f;

    // 초기 속도 크기 (단순 모델)
    float v0 = sqrtf(2.0f * a0 * R);

    // 발사각 계산 (중력 반영)
    float under_sqrt = v0*v0*v0*v0 - g * (g * R * R + 2 * Dy * v0 * v0);
    if (under_sqrt < 0.0f) return false;

    float theta = atanf((v0*v0 - sqrtf(under_sqrt)) / (g * R));

    // 수평 단위 벡터
    vec3_t dir = { diff.x / R, 0, diff.z / R };

    // 초기 속도 벡터 + 바람 보정
    out->vec.x = v0 * cosf(theta) * dir.x + env->wind.x;
    out->vec.y = v0 * sinf(theta) + env->wind.y;
    out->vec.z = v0 * cosf(theta) * dir.z + env->wind.z;

    // 도착 시간
    float v_h = v0 * cosf(theta) + 
        sqrtf(env->wind.x * env->wind.x + env->wind.z * env->wind.z);

    out->dt = R / (v_h > 1e-3f ? v_h : 1e-3f);

    return true;
}

bool projectile_calc_force_for_time(
    launch_param_t* out,
    const projectile_t* proj,
    const vec3_t* target,
    float hit_time)
{
    if (!out || !proj || !target || hit_time <= 0.0f) return false;

    vec3_t delta;
    vec3_sub(&delta, target, &proj->position);

    vec3_t gravity_term;
    gravity_term.x = 0.0f;
    gravity_term.y = -0.5f * 9.81f * hit_time * hit_time; // y방향 중력만 가정
    gravity_term.z = 0.0f;

    vec3_t required_vel;
    required_vel.x = (delta.x - gravity_term.x) / hit_time;
    required_vel.y = (delta.y - gravity_term.y) / hit_time;
    required_vel.z = (delta.z - gravity_term.z) / hit_time;

    float required_force = proj->mass * vec3_length(&required_vel);

    out->vec = required_vel;
    out->dt = hit_time;

    // 필요하면 required_force를 out에 따로 저장할 수 있도록 launch_param_t를 확장할 수도 있음
    return true;
}
