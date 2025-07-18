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
    vec3_unit(&dir, &dir);
    return &dir;
}

const vec3_t* projectile_guidance_lead(
    const projectile_t* proj, float dt, void* userdata) 
{
    if (!userdata) return nullptr;
    const target_info_t* target = (const target_info_t*)userdata;

    // 미사일 위치 가져오기
    vec3_t missile_pos;
    xform_get_position(&proj->xf, &missile_pos);

    // 미사일 속도 크기
    float missile_speed = vec3_length(&proj->velocity);
    if (missile_speed < 0.01f) missile_speed = 0.01f;

    // 목표와 미사일 간 벡터 및 거리
    vec3_t to_target;
    vec3_sub(&to_target, &target->position, &missile_pos);
    float distance = vec3_length(&to_target);

    // 예상 리드 타임 = 거리 / 미사일 속도
    float lead_time = distance / missile_speed;

    // 목표의 미래 위치
    vec3_t predicted_target = target->position;
    vec3_t future_offset;
    vec3_scale(&future_offset, &target->velocity, lead_time);
    vec3_add(&predicted_target, &predicted_target, &future_offset);

    // 방향 벡터 계산
    static vec3_t dir;
    vec3_sub(&dir, &predicted_target, &missile_pos);
    vec3_unit(&dir, &dir);
    return &dir;
}

const vec3_t* projectile_guidance_from_trajectory(
    const projectile_t* proj, float dt, void* userdata)
{
    static vec3_t dir;
    if (!userdata) return nullptr;
    const target_traj_info_t* target_info = (const target_traj_info_t*)userdata;

    // 미사일 위치
    vec3_t missile_pos;
    xform_get_position(&proj->xf, &missile_pos);

    // 미사일 속도 크기
    float missile_speed = vec3_length(&proj->velocity);
    if (missile_speed < 0.01f) missile_speed = 0.01f;

    // lead_time = 거리 / 미사일 속력
    vec3_t target_now;
    trajectory_sample_position(target_info->trajectory, target_info->current_time, &target_now);

    vec3_t diff;
    vec3_sub(&diff, &target_now, &missile_pos);
    float distance = vec3_length(&diff);
    float lead_time = distance / missile_speed;

    // 미래 위치 예측
    vec3_t predicted_target;
    trajectory_sample_position(target_info->trajectory,
                               target_info->current_time + lead_time,
                               &predicted_target);

    // 방향 계산
    vec3_sub(&dir, &predicted_target, &missile_pos);
    vec3_unit(&dir, &dir);
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
    vec3_unit(&axis, &proj->angular_velocity);
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

    trajectory_clear(&out->trajectory);

    vec3_t pos = p->start_pos;
    vec3_t vel = p->start_velocity;
    motion_state_t state = {0};

    float t = 0.0f;
    for (; t <= p->max_time; t += p->time_step) {
        vec3_t accel = p->gravity;
        if (p->env_fn) {
            const vec3_t* env = p->env_fn(NULL, p->time_step, p->env_userdata);
            if (env) vec3_add(&accel, &accel, env);
        }

        // 현재 상태 기록
        state.linear.position = pos;
        state.linear.velocity = vel;
        state.linear.acceleration = accel;
        trajectory_add_sample(&out->trajectory, t, &state);

        // 이전 위치 저장
        vec3_t pos_prev = pos;

        // 속도 업데이트
        vec3_t dv;
        vec3_scale(&dv, &accel, p->time_step);
        vec3_add(&vel, &vel, &dv);

        // 위치 업데이트
        vec3_t dp;
        vec3_scale(&dp, &vel, p->time_step);
        vec3_add(&pos, &pos, &dp);

        // 지면 충돌 체크 (보간)
        if (pos.y <= p->ground_height) {
            float dy = pos.y - pos_prev.y;
            float alpha = (p->ground_height - pos_prev.y) / dy;

            float impact_time = t + alpha * p->time_step;

            vec3_t impact_pos;
            vec3_lerp(&impact_pos, &pos_prev, &pos, alpha);
            impact_pos.y = p->ground_height;

            state.linear.position = impact_pos;
            trajectory_add_sample(&out->trajectory, impact_time, &state);

            out->impact_time = impact_time;
            out->impact_pos = impact_pos;
            out->valid = true;
            return true;
        }
    }

    out->valid = false;
    return false;
}

// bool projectile_predict_missile(const missile_predictor_t* p,
//                                 projectile_result_t* out) {
//     if (!p || !out) return false;

//     // trajectory 초기화
//     trajectory_clear(&out->trajectory);

//     // 초기 상태 설정
//     linear_state_t state = {
//         .position = p->start_pos,
//         .velocity = p->start_velocity,
//         .acceleration = {0, 0, 0}
//     };

//     // 임시 projectile_t (guidance_fn 호출용)
//     projectile_t temp_proj = {0};
//     xform_set_position(&temp_proj.xf, &p->start_pos);
//     temp_proj.velocity = p->start_velocity;

//     // 적분기 설정
//     integrator_config_t config = {
//         .type = p->integrator_type,
//         .time_step = p->time_step
//     };

//     // PID 제어기 초기화 (목표 속력 = 초기 속도 + thrust 크기)
//     pid_controller_t pid_speed;
//     pid_init(&pid_speed, 2.0f, 0.5f, 0.1f, p->time_step);  // (P, I, D, dt)
//     float target_speed = vec3_length(&p->start_velocity) + vec3_length(&p->thrust);

//     float t = 0.0f;
//     float fuel = p->fuel;

//     // 시뮬레이션 루프
//     for (; t <= p->max_time; t += config.time_step) {
//         // guidance_userdata에 시간 갱신
//         if (p->guidance_userdata) {
//             target_traj_info_t* ti = (target_traj_info_t*)p->guidance_userdata;
//             ti->current_time = t;
//         }

//         // temp_proj 업데이트
//         xform_set_position(&temp_proj.xf, &state.position);
//         temp_proj.velocity = state.velocity;

//         // -----------------------------
//         // 유도 방향 계산
//         // -----------------------------
//         vec3_t guidance = {0, 0, 0};
//         if (fuel > 0.0f) {
//             const vec3_t* g = (p->guidance_fn && p->guidance_fn != projectile_guidance_none)
//                                   ? p->guidance_fn(&temp_proj, t, p->guidance_userdata)
//                                   : NULL;

//             if (g) {
//                 guidance = *g;
//                 vec3_unit(&guidance, &guidance);
//             } else {
//                 guidance = p->thrust;
//                 vec3_unit(&guidance, &guidance);
//             }
//         }

//         // -----------------------------
//         // PID 기반 thrust 크기 조정
//         // -----------------------------
//         vec3_t thrust_accel = {0, 0, 0};
//         if (fuel > 0.0f) {
//             float current_speed = vec3_length(&state.velocity);
//             float thrust_mag = pid_update(&pid_speed, target_speed, current_speed);
//             thrust_mag = fmaxf(0.0f, fminf(thrust_mag, vec3_length(&p->thrust)));
//             vec3_scale(&thrust_accel, &guidance, thrust_mag);
//             fuel -= config.time_step;
//         }

//         // -----------------------------
//         // 환경 영향 계산
//         // -----------------------------
//         vec3_t env = {0, 0, 0};
//         if (p->env_fn) {
//             const vec3_t* e = p->env_fn(&temp_proj, t, p->env_userdata);
//             if (e) env = *e;
//         }

//         // 총 가속도 = 중력 + 환경 + thrust
//         vec3_add(&state.acceleration, &p->gravity, &env);
//         vec3_add(&state.acceleration, &state.acceleration, &thrust_accel);

//         // -----------------------------
//         // trajectory 샘플 기록
//         // -----------------------------
//         motion_state_t motion = {0};
//         motion.linear.position = state.position;
//         motion.linear.velocity = state.velocity;
//         motion.linear.acceleration = state.acceleration;
//         trajectory_add_sample(&out->trajectory, t, &motion);

//         // -----------------------------
//         // 적분 및 위치/속도 갱신
//         // -----------------------------
//         vec3_t pos_prev = state.position;
//         numeq_integrate(&state, &state.acceleration, &config);

//         // -----------------------------
//         // 충돌 체크
//         // -----------------------------
//         if (state.position.y <= p->ground_height) {
//             float dy = state.position.y - pos_prev.y;
//             float alpha = (p->ground_height - pos_prev.y) / dy;

//             float impact_time = t + alpha * config.time_step;
//             vec3_t impact_pos;
//             vec3_lerp(&impact_pos, &pos_prev, &state.position, alpha);
//             impact_pos.y = p->ground_height;

//             motion.linear.position = impact_pos;
//             trajectory_add_sample(&out->trajectory, impact_time, &motion);

//             out->impact_time = impact_time;
//             out->impact_pos = impact_pos;
//             out->valid = true;
//             return true;
//         }
//     }

//     out->valid = false;
//     return false;
// }

bool projectile_predict_missile(const missile_predictor_t* p,
                                projectile_result_t* out) {
    if (!p || !out) return false;

    // trajectory 초기화
    trajectory_clear(&out->trajectory);

    // 초기 상태 설정
    linear_state_t state = {
        .position = p->start_pos,
        .velocity = p->start_velocity,
        .acceleration = {0, 0, 0}
    };

    // 임시 projectile_t (guidance_fn 호출용)
    projectile_t temp_proj = {0};
    xform_set_position(&temp_proj.xf, &p->start_pos);
    temp_proj.velocity = p->start_velocity;

    // 적분기 설정
    integrator_config_t config = {
        .type = p->integrator_type,
        .time_step = p->time_step
    };

    // 제어기
    controller_t* ctrl = p->controller;  // PID, Bang-Bang, MPC 등
    float target_speed = vec3_length(&p->start_velocity) + vec3_length(&p->thrust);

    float t = 0.0f;
    float fuel = p->fuel;

    // 시뮬레이션 루프

    if (config.time_step <= 0.0f) {
    fprintf(stderr, "[ERROR] time_step is invalid (%.4f)\n", config.time_step);
    return false;
}
    for (; t <= p->max_time; t += config.time_step) {
        // guidance_userdata에 시간 갱신
        if (p->guidance_userdata) {
            target_traj_info_t* ti = (target_traj_info_t*)p->guidance_userdata;
            ti->current_time = t;
        }

        // temp_proj 업데이트
        xform_set_position(&temp_proj.xf, &state.position);
        temp_proj.velocity = state.velocity;

        // -----------------------------
        // 유도 방향 계산
        // -----------------------------
        vec3_t guidance = {0, 0, 0};
        if (fuel > 0.0f) {
            const vec3_t* g = (p->guidance_fn && p->guidance_fn != projectile_guidance_none)
                                  ? p->guidance_fn(&temp_proj, t, p->guidance_userdata)
                                  : NULL;

            if (g) {
                guidance = *g;
                vec3_unit(&guidance, &guidance);
            } else {
                guidance = p->thrust;
                vec3_unit(&guidance, &guidance);
            }
        }

        // -----------------------------
        // thrust 크기 제어 (controller 사용)
        // -----------------------------
        vec3_t thrust_accel = {0, 0, 0};
        if (fuel > 0.0f && ctrl) {
            float current_speed = vec3_length(&state.velocity);
            float thrust_mag = controller_compute(ctrl, target_speed, current_speed, config.time_step);
            thrust_mag = fmaxf(0.0f, fminf(thrust_mag, vec3_length(&p->thrust)));
            vec3_scale(&thrust_accel, &guidance, thrust_mag);
            fuel -= config.time_step;
        }

        // -----------------------------
        // 환경 영향 계산
        // -----------------------------
        vec3_t env = {0, 0, 0};
        if (p->env_fn) {
            const vec3_t* e = p->env_fn(&temp_proj, t, p->env_userdata);
            if (e) env = *e;
        }

        // 총 가속도 = 중력 + 환경 + thrust
        vec3_add(&state.acceleration, &p->gravity, &env);
        vec3_add(&state.acceleration, &state.acceleration, &thrust_accel);

        // -----------------------------
        // trajectory 샘플 기록
        // -----------------------------
        motion_state_t motion = {0};
        motion.linear.position = state.position;
        motion.linear.velocity = state.velocity;
        motion.linear.acceleration = state.acceleration;
        trajectory_add_sample(&out->trajectory, t, &motion);

        // -----------------------------
        // 적분 및 위치/속도 갱신
        // -----------------------------
        vec3_t pos_prev = state.position;
        numeq_integrate(&state, &state.acceleration, &config);

        // -----------------------------
        // 충돌 체크
        // -----------------------------
        if (state.position.y <= p->ground_height) {
            float dy = state.position.y - pos_prev.y;
            float alpha = (p->ground_height - pos_prev.y) / dy;

            float impact_time = t + alpha * config.time_step;
            vec3_t impact_pos;
            vec3_lerp(&impact_pos, &pos_prev, &state.position, alpha);
            impact_pos.y = p->ground_height;

            motion.linear.position = impact_pos;
            trajectory_add_sample(&out->trajectory, impact_time, &motion);

            out->impact_time = impact_time;
            out->impact_pos = impact_pos;
            out->valid = true;
            return true;
        }
    }

    out->valid = false;
    return false;
}
