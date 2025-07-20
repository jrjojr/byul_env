#include "internal/projectile.h"
#include "internal/xform.h"
#include "internal/numeq.h"
#include <cmath>
#include <iostream>

void projectile_init(projectile_t* out) {
    if (!out) return;

    // 위치/회전 초기화
    xform_init(&out->xf);

    // 속도/가속도/각속도 초기화
    out->velocity = (vec3_t){0.0f, 0.0f, 0.0f};
    out->acceleration = (vec3_t){0.0f, 0.0f, 0.0f};
    out->angular_velocity = (vec3_t){0.0f, 0.0f, 0.0f};

    // 기본값
    out->age = 0.0f;
    out->lifetime = 10.0f; // 기본 10초
    out->type = PROJECTILE_TYPE_SHELL;
    out->projectile_id = -1;
    out->owner = NULL;

    out->on_hit = projectile_default_hit_cb;
    out->hit_userdata = NULL;
}

void projectile_init_full(projectile_t* out, 
    projectile_type_t type, float lifetime) {

    if (!out) return;
    projectile_init(out);
    out->type = type;
    out->lifetime = lifetime > 0.0f ? lifetime : 10.0f; // 최소 0보다 커야 함
}

void projectile_assign(projectile_t* out, const projectile_t* src) {
    if (!out || !src) return;
    *out = *src;
}

void shell_init(shell_t* shell) {
    if (!shell) return;
    projectile_init(&shell->base);
    shell->base.type = PROJECTILE_TYPE_SHELL;
    shell->drag_coef = 0.0f;
    shell->env_fn = projectile_env_default;
    shell->env_userdata = NULL;
}

void shell_init_full(shell_t* shell, float drag_coef,
                     projectile_environ_func env_fn, void* env_userdata) {
    if (!shell) return;
    projectile_init(&shell->base);
    shell->base.type = PROJECTILE_TYPE_SHELL;
    shell->drag_coef = drag_coef;
    shell->env_fn = env_fn ? env_fn : projectile_env_default;
    shell->env_userdata = env_userdata;
}

void shell_assign(shell_t* out, const shell_t* src) {
    if (!out || !src) return;
    *out = *src;
}

void missile_init(missile_t* missile) {
    if (!missile) return;

    projectile_init(&missile->base);
    missile->base.type = PROJECTILE_TYPE_MISSILE;

    missile->thrust = (vec3_t){0.0f, 0.0f, 0.0f};
    missile->fuel = 0.0f;
    missile->controller = NULL;

    missile->guidance_fn = projectile_guidance_none;
    missile->guidance_userdata = NULL;

    missile->env_fn = projectile_env_default;
    missile->env_userdata = NULL;
}

void missile_init_full(missile_t* missile,
                       const vec3_t* thrust,
                       float fuel,
                       controller_t* controller,
                       projectile_guidance_func guidance_fn,
                       void* guidance_userdata,
                       projectile_environ_func env_fn,
                       void* env_userdata) {
    if (!missile) return;

    projectile_init(&missile->base);
    missile->base.type = PROJECTILE_TYPE_MISSILE;

    missile->thrust = thrust ? *thrust : (vec3_t){0.0f, 0.0f, 0.0f};
    missile->fuel = fuel;
    missile->controller = controller;

    missile->guidance_fn = guidance_fn ? guidance_fn : projectile_guidance_none;
    missile->guidance_userdata = guidance_userdata;

    missile->env_fn = env_fn ? env_fn : projectile_env_default;
    missile->env_userdata = env_userdata;
}

void missile_assign(missile_t* out, const missile_t* src) {
    if (!out || !src) return;
    *out = *src;
}

void projectile_predictor_init(projectile_predictor_t* out) {
    if (!out) return;

    out->start_pos = (vec3_t){0.0f, 0.0f, 0.0f};
    out->start_velocity = (vec3_t){0.0f, 0.0f, 0.0f};

    out->env_fn = projectile_env_default;
    out->env_userdata = NULL;

    out->ground_height = 0.0f;
    out->max_time = 10.0f;
    out->time_step = 0.01f;
}

void projectile_predictor_init_full(
    projectile_predictor_t* out,
    const vec3_t* start_pos,
    const vec3_t* start_velocity,
    float ground_height,
    float max_time,
    float time_step,
    projectile_environ_func env_fn,
    void* env_userdata
) {
    if (!out) return;

    projectile_predictor_init(out);

    if (start_pos) out->start_pos = *start_pos;
    if (start_velocity) out->start_velocity = *start_velocity;

    out->ground_height = ground_height;
    out->max_time = max_time > 0.0f ? max_time : 10.0f;
    out->time_step = time_step > 0.0f ? time_step : 0.01f;

    out->env_fn = env_fn ? env_fn : projectile_env_default;
    out->env_userdata = env_userdata;
}

void projectile_predictor_assign(projectile_predictor_t* out,
                               const projectile_predictor_t* src) {
    if (!out || !src) return;
    *out = *src;
}

void missile_predictor_init(missile_predictor_t* out) {
    if (!out) return;

    out->start_pos = (vec3_t){0, 0, 0};
    out->start_velocity = (vec3_t){0, 0, 0};
    out->thrust = (vec3_t){0, 0, 0};

    out->fuel = 0.0f;
    out->controller = NULL;

    out->guidance_fn = projectile_guidance_none;
    out->guidance_userdata = NULL;

    out->env_fn = projectile_env_default;
    out->env_userdata = NULL;

    out->ground_height = 0.0f;
    out->max_time = 10.0f;
    out->time_step = 0.01f;
    out->integrator_type = INTEGRATOR_EULER;
}

void missile_predictor_init_full(
    missile_predictor_t* out,
    const vec3_t* start_pos,
    const vec3_t* start_velocity,
    const vec3_t* thrust,
    float fuel,
    controller_t* controller,
    projectile_guidance_func guidance_fn,
    void* guidance_userdata,
    projectile_environ_func env_fn,
    void* env_userdata,
    float ground_height,
    float max_time,
    float time_step,
    integrator_type_t integrator_type
) {
    if (!out) return;

    missile_predictor_init(out);

    if (start_pos) out->start_pos = *start_pos;
    if (start_velocity) out->start_velocity = *start_velocity;
    if (thrust) out->thrust = *thrust;

    out->fuel = fuel;
    out->controller = controller;

    out->guidance_fn = guidance_fn ? guidance_fn : projectile_guidance_none;
    out->guidance_userdata = guidance_userdata;

    out->env_fn = env_fn ? env_fn : projectile_env_default;
    out->env_userdata = env_userdata;

    out->ground_height = ground_height;
    out->max_time = max_time > 0.0f ? max_time : 10.0f;
    out->time_step = time_step > 0.0f ? time_step : 0.01f;
    out->integrator_type = integrator_type;
}

void missile_predictor_assign(missile_predictor_t* out,
                            const missile_predictor_t* src) {
    if (!out || !src) return;
    *out = *src;
}

projectile_result_t* projectile_result_create() {
    projectile_result_t* res = new projectile_result_t;
    if (!res) return NULL;
    res->impact_time = 0.0f;
    res->impact_pos = (vec3_t){0, 0, 0};
    res->valid = false;
    res->trajectory = trajectory_create(); // 기본 샘플 개수 100
    return res;
}

projectile_result_t* projectile_result_create_full(int capacity) {
    projectile_result_t* res = new projectile_result_t;
    if (!res) return NULL;
    res->impact_time = 0.0f;
    res->impact_pos = (vec3_t){0, 0, 0};
    res->valid = false;
    res->trajectory = trajectory_create_full(capacity);
    return res;
}

projectile_result_t* projectile_result_assign(const projectile_result_t* src) {
    if (!src) return NULL;
    projectile_result_t* copy = nullptr;
    copy = projectile_result_create_full(src->trajectory->capacity);

    copy->impact_time = src->impact_time;
    copy->impact_pos = src->impact_pos;
    copy->valid = src->valid;
    copy->trajectory = trajectory_copy(src->trajectory);
    return copy;
}

void projectile_result_destroy(projectile_result_t* res) {
    if (!res) return;
    trajectory_destroy(res->trajectory);
    delete res;
}

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

const vec3_t* projectile_env_default(const projectile_t* proj, float dt, void* userdata) {
    static vec3_t accel = {0.0f, -9.81f, 0.0f};  // Y축을 중력 방향으로 가정
    return &accel;
}

const vec3_t* projectile_env_constant(
    const projectile_t* proj, float dt, void* userdata) {

    static vec3_t accel;
    vec3_t* wind = (vec3_t*)userdata;

    accel.x = (wind ? wind->x : 0.0f);
    accel.y = (wind ? wind->y : 0.0f) - 9.81f;
    accel.z = (wind ? wind->z : 0.0f);
    return &accel;
}

const vec3_t* projectile_env_dynamic(
    const projectile_t* proj, float dt, void* userdata) {

    static vec3_t accel = {0, 0, 0};
    env_dynamic_data_t* env = (env_dynamic_data_t*)userdata;

    if (env) {
        env->time += dt;
        float gust = sinf(env->time) * env->gust_strength;
        accel.x = env->base_wind.x + gust;
        accel.y = env->base_wind.y - 9.81f;
        accel.z = env->base_wind.z;
    } else {
        accel = (vec3_t){0.0f, -9.81f, 0.0f};
    }
    return &accel;
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

    // 미사일 위치
    vec3_t missile_pos;
    xform_get_position(&proj->xf, &missile_pos);

    // 미사일 속도 크기 (vec3_length_sq 사용)
    float missile_speed_sq = vec3_length_sq(&proj->velocity);
    float missile_speed = (missile_speed_sq > 1e-6f) ? 
        sqrtf(missile_speed_sq) : 0.01f;

    // 목표와 미사일 간 벡터 및 거리
    vec3_t to_target;
    vec3_sub(&to_target, &target->position, &missile_pos);
    float distance_sq = vec3_length_sq(&to_target);
    float distance = sqrtf(distance_sq);

    // 예상 리드 타임 = 거리 / 미사일 속도
    float lead_time = distance / missile_speed;

    // 목표의 미래 위치 예측
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
    float missile_speed = vec3_length_sq(&proj->velocity);
    if (missile_speed < 0.01f) missile_speed = 0.01f;

    // lead_time = 거리 / 미사일 속력
    vec3_t target_now;
    trajectory_sample_position(target_info->trajectory, 
        target_info->current_time, &target_now);

    vec3_t diff;
    vec3_sub(&diff, &target_now, &missile_pos);
    float distance = vec3_length_sq(&diff);
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

    float angle = vec3_length_sq(&proj->angular_velocity) * dt;
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

    trajectory_clear(out->trajectory);

    vec3_t pos = p->start_pos;
    vec3_t vel = p->start_velocity;
    motion_state_t state = {0};

    float t = 0.0f;
    for (; t <= p->max_time; t += p->time_step) {
        // 외부 가속도(중력 포함) 계산
        vec3_t accel = {0.0f, 0.0f, 0.0f};
        if (p->env_fn) {
            const vec3_t* env = p->env_fn(NULL, p->time_step, p->env_userdata);
            if (env) accel = *env;
        }

        // 현재 상태 기록
        state.linear.position = pos;
        state.linear.velocity = vel;
        state.linear.acceleration = accel;
        trajectory_add_sample(out->trajectory, t, &state);

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
            float alpha = (dy != 0.0f) 
                          ? (p->ground_height - pos_prev.y) / dy 
                          : 0.0f;

            float impact_time = t + alpha * p->time_step;

            vec3_t impact_pos;
            vec3_lerp(&impact_pos, &pos_prev, &pos, alpha);
            impact_pos.y = p->ground_height;

            state.linear.position = impact_pos;
            trajectory_add_sample(out->trajectory, impact_time, &state);

            out->impact_time = impact_time;
            out->impact_pos = impact_pos;
            out->valid = true;
            return true;
        }
    }

    out->valid = false;
    return false;
}

bool projectile_predict_missile(const missile_predictor_t* p,
                                projectile_result_t* out) {
    if (!p || !out) return false;
    trajectory_clear(out->trajectory);

    // 전역 또는 디버그 설정값
    const int debug_mode = 0;

    // -----------------------------
    // 초기 상태 설정
    // -----------------------------
    motion_state_t state;
    motion_state_init(&state);
    state.linear.position = p->start_pos;
    state.linear.velocity = p->start_velocity;
    state.linear.acceleration = (vec3_t){0, 0, 0};

    // 목표 속도 = 초기 속도 + 초기 thrust
    vec3_t target_velocity;
    vec3_add(&target_velocity, &p->start_velocity, &p->thrust);

    // 임시 projectile_t
    projectile_t temp_proj;
    projectile_init(&temp_proj);
    xform_set_position(&temp_proj.xf, &p->start_pos);
    temp_proj.velocity = p->start_velocity;

    // -----------------------------
    // 적분기 설정
    // -----------------------------
    integrator_config_t config;
    integrator_config_init(&config);
    config.time_step = (p->time_step > 0.0f) ? p->time_step : 0.01f;
    config.type = p->integrator_type;

    if (config.time_step <= 0.0f) {
        fprintf(stderr,
                "[ERROR] projectile_predict_missile: invalid time_step (%.4f)\n",
                config.time_step);
        return false;
    }

    controller_t* ctrl = p->controller;
    float t = 0.0f;
    float fuel = p->fuel;

    int max_steps = (int)ceilf(p->max_time / config.time_step);
    int step_count = 0;

    if (debug_mode) {
        printf("=== projectile_predict_missile DEBUG START ===\n");
        printf("Start Pos: (%.2f, %.2f, %.2f)\n",
               p->start_pos.x, p->start_pos.y, p->start_pos.z);
        printf("Start Vel: (%.2f, %.2f, %.2f)\n",
               p->start_velocity.x, p->start_velocity.y, p->start_velocity.z);
        printf("Thrust: (%.2f, %.2f, %.2f), Fuel: %.2f\n",
               p->thrust.x, p->thrust.y, p->thrust.z, p->fuel);
    }

    // -----------------------------
    // 시뮬레이션 루프
    // -----------------------------
    for (; t <= p->max_time; t += config.time_step) {
        if (++step_count > max_steps) {
            fprintf(stderr,
                    "[ERROR] projectile_predict_missile: step overflow (t=%.2f)\n",
                    t);
            break;
        }

        // guidance_userdata에 현재 시간 갱신
        if (p->guidance_userdata) {
            ((target_traj_info_t*)p->guidance_userdata)->current_time = t;
        }

        // temp_proj 위치 갱신
        xform_set_position(&temp_proj.xf, &state.linear.position);
        temp_proj.velocity = state.linear.velocity;

        // -----------------------------
        // 유도 벡터 계산
        // -----------------------------
        vec3_t guidance = {0, 0, 0};
        if (fuel > 0.0f) {
            const vec3_t* g = (
                p->guidance_fn && p->guidance_fn != projectile_guidance_none)
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
        // Thrust 계산 (연료 소진 시 0)
        // -----------------------------
        vec3_t thrust_accel = {0, 0, 0};
        if (fuel > 0.0f) {
            float max_thrust = vec3_length(&p->thrust);
            float thrust_mag = max_thrust;

            if (ctrl) {
                float current_speed = vec3_length(&state.linear.velocity);
                float target_speed = vec3_length(&target_velocity);

                thrust_mag = controller_compute(
                    ctrl, target_speed, current_speed, config.time_step);
                thrust_mag = fmaxf(fminf(thrust_mag, max_thrust), -max_thrust);
            }

            vec3_scale(&thrust_accel, &guidance, thrust_mag);
            fuel -= config.time_step;
        }

        // -----------------------------
        // 환경 가속도 (중력+바람)
        // -----------------------------
        vec3_t env = {0, 0, 0};
        if (p->env_fn) {
            const vec3_t* e = p->env_fn(&temp_proj, t, p->env_userdata);
            if (e) env = *e;
        }

        // 총 가속도 = 환경 + thrust
        vec3_add(&state.linear.acceleration, &env, &thrust_accel);

        if (debug_mode) {
            printf("[t=%.2f] Pos:(%.2f,%.2f,%.2f) Vel:(%.2f,%.2f,%.2f) "
                   "Acc:(%.2f,%.2f,%.2f) Fuel:%.2f\n",
                   t,
                   state.linear.position.x, state.linear.position.y, state.linear.position.z,
                   state.linear.velocity.x, state.linear.velocity.y, state.linear.velocity.z,
                   state.linear.acceleration.x, state.linear.acceleration.y, state.linear.acceleration.z,
                   fuel);
        }

        // trajectory 샘플 기록
        trajectory_add_sample(out->trajectory, t, &state);

        // 위치/속도 적분
        vec3_t pos_prev = state.linear.position;
        numeq_integrate(&state, &config);

        // -----------------------------
        // 충돌 감지
        // -----------------------------
        if (state.linear.position.y <= p->ground_height) {
            float dy = state.linear.position.y - pos_prev.y;
            float alpha = (fabsf(dy) > 1e-6f)
                            ? (p->ground_height - pos_prev.y) / dy
                            : 0.0f;

            float impact_time = t + alpha * config.time_step;
            vec3_t impact_pos;
            vec3_lerp(&impact_pos, &pos_prev, &state.linear.position, alpha);
            impact_pos.y = p->ground_height;

            state.linear.position = impact_pos;
            trajectory_add_sample(out->trajectory, impact_time, &state);

            out->impact_time = impact_time;
            out->impact_pos = impact_pos;
            out->valid = true;

            if (debug_mode) {
                printf("Impact at t=%.3f Pos:(%.2f,%.2f,%.2f)\n",
                       impact_time, impact_pos.x, impact_pos.y, impact_pos.z);
                printf("=== projectile_predict_missile DEBUG END ===\n");
            }
            return true;
        }
    }

    // 충돌이 발생하지 않음
    if (debug_mode) {
        printf("No impact detected (max_time=%.2f)\n", p->max_time);
        printf("=== projectile_predict_missile DEBUG END ===\n");
    }

    out->valid = false;
    return false;
}
