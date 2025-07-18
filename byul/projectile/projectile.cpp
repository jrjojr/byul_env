#include "internal/projectile.h"
#include "internal/xform.h"
#include "internal/numeq.h"
#include <cmath>
#include <iostream>

// ---------------------------------------------------------
// 기본 콜백 함수
// ---------------------------------------------------------

void projectile_default_hit_cb(const projectile_t* proj, void* userdata) {
    (void)proj;
    (void)userdata;
    std::cout << "[projectile] default hit cb (no effect)\n";
}

const vec3_t* projectile_env_none(
    const projectile_t* proj, float dt, void* userdata) {

    static vec3_t zero = {0, 0, 0};
    (void)proj; (void)dt; (void)userdata;
    return &zero;
}

const vec3_t* projectile_env_constant(
    const projectile_t* proj, float dt, void* userdata) {

    static vec3_t wind = {1.0f, 0.0f, 0.0f};
    (void)proj; (void)dt; (void)userdata;
    return &wind;
}

const vec3_t* projectile_guidance_none(
    const projectile_t* proj, float dt, void* userdata) {

    static vec3_t zero = {0, 0, 0};
    (void)proj; (void)dt; (void)userdata;
    return &zero;
}

const vec3_t* projectile_guidance_to_target(
    const projectile_t* proj, float dt, void* userdata) {

    if (!userdata) return nullptr;
    const vec3_t* target = static_cast<const vec3_t*>(userdata);
    static vec3_t dir;

    vec3_t pos;
    xform_get_position(&proj->xf, &pos);
    vec3_sub(&dir, target, &pos);
    vec3_normalize(&dir, &dir);
    return &dir;
}

// ---------------------------------------------------------
// 공통 회전 적용
// ---------------------------------------------------------

void projectile_apply_rotation(projectile_t* proj, float dt) {
    if (!proj) return;

    float angle = vec3_length(&proj->angular_velocity) * dt;
    if (angle < 1e-5f) return;

    vec3_t axis;
    vec3_normalize(&axis, &proj->angular_velocity);
    xform_rotate_local_axis_angle(&proj->xf, &axis, angle);
}

// ---------------------------------------------------------
// Shell 업데이트
// ---------------------------------------------------------

void shell_update(shell_t* shell, float dt) {
    if (!shell) return;
    projectile_t* proj = &shell->base;

    // 환경 영향
    const vec3_t* env = shell->env_fn ? 
        shell->env_fn(proj, dt, shell->env_userdata) : nullptr;

    vec3_t total_accel = proj->acceleration;
    if (env) vec3_add(&total_accel, &total_accel, env);

    // 항력 계산
    vec3_t drag;
    vec3_scale(&drag, &proj->velocity, -shell->drag_coef);
    vec3_add(&total_accel, &total_accel, &drag);

    // 속도 업데이트
    vec3_t delta_v;
    vec3_scale(&delta_v, &total_accel, dt);
    vec3_add(&proj->velocity, &proj->velocity, &delta_v);

    // 위치 이동
    vec3_t delta_pos;
    vec3_scale(&delta_pos, &proj->velocity, dt);
    xform_translate(&proj->xf, &delta_pos);

    // 회전 적용
    projectile_apply_rotation(proj, dt);

    // 시간 갱신
    proj->age += dt;
    if (proj->age >= proj->lifetime && proj->on_hit) {
        proj->on_hit(proj, proj->hit_userdata);
    }
}

// ---------------------------------------------------------
// Missile 업데이트
// ---------------------------------------------------------

void missile_update(missile_t* missile, float dt) {
    if (!missile) return;
    projectile_t* proj = &missile->base;

    // 추진력 방향
    const vec3_t* guide_dir = missile->guidance_fn ? 
        missile->guidance_fn(proj, dt, missile->guidance_userdata) : nullptr;

    if (guide_dir && missile->fuel > 0.0f) {
        vec3_t thrust_vec;
        vec3_scale(&thrust_vec, guide_dir, vec3_length(&missile->thrust));
        vec3_add(&proj->acceleration, &proj->acceleration, &thrust_vec);
        missile->fuel -= dt;
    }

    // 환경 영향
    const vec3_t* env = missile->env_fn ? 
        missile->env_fn(proj, dt, missile->env_userdata) : nullptr;

    vec3_t total_accel = proj->acceleration;
    if (env) vec3_add(&total_accel, &total_accel, env);

    // 속도 업데이트
    vec3_t delta_v;
    vec3_scale(&delta_v, &total_accel, dt);
    vec3_add(&proj->velocity, &proj->velocity, &delta_v);

    // 위치 이동
    vec3_t delta_pos;
    vec3_scale(&delta_pos, &proj->velocity, dt);
    xform_translate(&proj->xf, &delta_pos);

    // 회전 적용
    projectile_apply_rotation(proj, dt);

    // 시간 갱신
    proj->age += dt;
    if (proj->age >= proj->lifetime && proj->on_hit) {
        proj->on_hit(proj, proj->hit_userdata);
    }
}

bool projectile_predict(const projectile_predictor_t* p,
                        projectile_result_t* out) {
    if (!p || !out) return false;

    vec3_t pos = p->start_pos;
    vec3_t vel = p->start_velocity;

    float t = 0.0f;
    for (; t < p->max_time; t += p->time_step) {
        // 외부 환경 영향
        vec3_t accel = p->gravity;
        if (p->env_fn) {
            const vec3_t* env = p->env_fn(NULL, p->time_step, p->env_userdata);
            if (env) vec3_add(&accel, &accel, env);
        }

        // 속도 업데이트
        vec3_t dv;
        vec3_scale(&dv, &accel, p->time_step);
        vec3_add(&vel, &vel, &dv);

        // 위치 업데이트
        vec3_t dp;
        vec3_scale(&dp, &vel, p->time_step);
        vec3_add(&pos, &pos, &dp);

        if (pos.y <= p->ground_height) {
            out->impact_time = t;
            out->impact_pos = pos;
            out->valid = true;
            return true;
        }
    }

    out->valid = false;
    return false;
}

#include "internal/numeq_integrator.h"
#include <math.h>
#include <string.h>

// bool projectile_predict_missile(const missile_predictor_t* p,
//                                  projectile_result_t* out) {
//     if (!p || !out) return false;

//     numeq_state_missile_t state;
//     state.pos = p->start_pos;
//     state.vel = p->start_velocity;
//     state.fuel = p->fuel;

//     float t = 0.0f;
//     vec3_t acc, guidance, thrust_accel, env;

//     for (; t <= p->max_time; t += p->time_step) {
//         // 초기화
//         vec3_zero(&acc);

//         // 외부 환경
//         if (p->env_fn) {
//             const vec3_t* env_ptr = p->env_fn(
//                 NULL, p->time_step, p->env_userdata);

//             if (env_ptr) env = *env_ptr;
//             else vec3_zero(&env);
//         } else {
//             vec3_zero(&env);
//         }

//         // 유도 가속도
//         vec3_zero(&guidance);
//         if (p->guidance_fn && state.fuel > 0.0f) {
//             const vec3_t* g = p->guidance_fn(
//                 NULL, p->time_step, p->guidance_userdata);

//             if (g) guidance = *g;
//             vec3_normalize(&guidance, &guidance);
//         }

//         // thrust 적용
//         vec3_zero(&thrust_accel);
//         if (state.fuel > 0.0f) {
//             vec3_scale(&thrust_accel, &guidance, vec3_length(&p->thrust));
//             state.fuel -= p->time_step;
//         }

//         // 가속도 = 중력 + 환경 + thrust
//         vec3_add(&acc, &p->gravity, &env);
//         vec3_add(&acc, &acc, &thrust_accel);

//         // Semi-implicit Euler 적분
//         vec3_t delta_v, delta_pos;
//         vec3_scale(&delta_v, &acc, p->time_step);
//         vec3_add(&state.vel, &state.vel, &delta_v);

//         vec3_scale(&delta_pos, &state.vel, p->time_step);
//         vec3_add(&state.pos, &state.pos, &delta_pos);

//         if (state.pos.y <= p->ground_height) {
//             out->impact_time = t;
//             out->impact_pos = state.pos;
//             out->valid = true;
//             return true;
//         }
//     }

//     out->valid = false;
//     return false;
// }

static void missile_solver_func(
    void* user_data,
    float t,
    const void* state,
    void* dst_out) // <- derivative 저장 위치
{
    const missile_predictor_t* p = (const missile_predictor_t*)user_data;
    const numeq_model_missile_t* s = (const numeq_model_missile_t*)state;
    numeq_model_missile_t* d = (numeq_model_missile_t*)dst_out;

    // 초기화
    vec3_zero(&d->pos);
    vec3_zero(&d->vel);
    d->fuel = 0;

    // 유도 방향
    vec3_t dir = {0, 0, 0};
    if (p->guidance_fn && s->fuel > 0.0f) {
        const vec3_t* g = p->guidance_fn(NULL, t, p->guidance_userdata);
        if (g) {
            dir = *g;
            vec3_normalize(&dir, &dir);
        }
    }

    // thrust 방향
    vec3_t thrust = {0, 0, 0};
    if (s->fuel > 0.0f)
        vec3_scale(&thrust, &dir, vec3_length(&p->thrust));

    // env
    vec3_t env = {0, 0, 0};
    if (p->env_fn) {
        const vec3_t* e = p->env_fn(NULL, t, p->env_userdata);
        if (e) env = *e;
    }

    // 총 가속도 = gravity + thrust + env
    vec3_add(&d->vel, &p->gravity, &thrust);
    vec3_add(&d->vel, &d->vel, &env);

    // 속도 미분 = 위치 변화
    d->pos = s->vel;

    // 연료 소모
    d->fuel = (s->fuel > 0) ? -1.0f : 0.0f;
}

#include "projectile.h"
#include "internal/numeq_integrator.h"
#include <math.h>

bool projectile_predict_missile(
    const missile_predictor_t* p,
    projectile_result_t* out)
{
    if (!p || !out) return false;

    state_vector_t state = {
        .position = p->start_pos,
        .velocity = p->start_velocity,
        .acceleration = {0, 0, 0} // 매 프레임 갱신됨
    };

    integrator_config_t config = {
        .type = p->integrator_type,
        .time_step = p->time_step
    };

    float t = 0.0f;
    float fuel = p->fuel;

    for (; t <= p->max_time; t += config.time_step) {
        // 🔹 유도 방향 계산
        vec3_t guidance = {0, 0, 0};
        if (fuel > 0.0f && p->guidance_fn) {
            const vec3_t* g = p->guidance_fn(NULL, t, p->guidance_userdata);
            if (g) {
                guidance = *g;
                vec3_normalize(&guidance, &guidance);
            }
        }

        // 🔹 thrust 계산
        vec3_t thrust_accel = {0, 0, 0};
        if (fuel > 0.0f) {
            vec3_scale(&thrust_accel, &guidance, vec3_length(&p->thrust));
            fuel -= config.time_step;
        }

        // 🔹 환경 영향
        vec3_t env = {0, 0, 0};
        if (p->env_fn) {
            const vec3_t* e = p->env_fn(NULL, t, p->env_userdata);
            if (e) env = *e;
        }

        // 🔹 총 가속도: gravity + env + thrust
        vec3_add(&state.acceleration, &p->gravity, &env);
        vec3_add(&state.acceleration, &state.acceleration, &thrust_accel);

        // ✅ 적분기 호출
        numeq_integrate(&state, &state.acceleration, &config);

        // 💥 지면 충돌 판단
        if (state.position.y <= p->ground_height) {
            out->impact_time = t;
            out->impact_pos = state.position;
            out->valid = true;
            return true;
        }
    }

    out->valid = false;
    return false;
}
