#include <cmath>
#include <cstring>
#include "internal/entity_dynamic.h"
#include "internal/vec3.h"
#include "internal/xform.h"
#include "internal/bodyprops.h"
#include "internal/trajectory.h"
#include "internal/numeq_model.h"

// ---------------------------------------------------------
// 기본 초기화
// ---------------------------------------------------------
void entity_dynamic_init(entity_dynamic_t* d)
{
    if (!d) return;
    entity_init(&d->base);
    xform_init(&d->xf);
    bodyprops_init(&d->props);
    vec3_zero(&d->velocity);
    vec3_zero(&d->angular_velocity);
}

// ---------------------------------------------------------
// 사용자 지정 초기화
// ---------------------------------------------------------
void entity_dynamic_init_full(
    entity_dynamic_t* d,
    const entity_t* base,
    const xform_t* xf,
    const vec3_t* velocity,
    const vec3_t* angular,
    const bodyprops_t* props
)
{
    if (!d) return;

    if (base) {
        entity_assign(&d->base, base);
    } else {
        entity_init(&d->base);
    }

    if (xf) {
        d->xf = *xf;
    } else {
        xform_init(&d->xf);
    }

    d->velocity = velocity ? *velocity : (vec3_t){0, 0, 0};
    d->angular_velocity = angular ? *angular : (vec3_t){0, 0, 0};
    d->props = props ? *props : (bodyprops_t){1.0f, 0.47f, 0.01f, 0.5f, 0.5f};
}

// ---------------------------------------------------------
// 복사
// ---------------------------------------------------------
void entity_dynamic_assign(entity_dynamic_t* dst, const entity_dynamic_t* src)
{
    if (!dst || !src) return;
    *dst = *src;
}

void entity_dynamic_calc_accel(
    const entity_dynamic_t* curr,
    const vec3_t* prev_vel,
    float dt,
    vec3_t* out_accel
) {
    if (!curr || !prev_vel || !out_accel || dt <= 0.0f) {
        if (out_accel) *out_accel = (vec3_t){0.0f, 0.0f, 0.0f};
        return;
    }

    // a = (v_curr - v_prev) / dt
    vec3_sub(out_accel, &curr->velocity, prev_vel);
    vec3_div_scalar(out_accel, out_accel, dt);
}

void entity_dynamic_calc_accel_env(
    const entity_dynamic_t* curr,
    const vec3_t* prev_vel,
    float dt,
    const environ_t* env,    
    vec3_t* out_accel
) {
    if (!curr || !prev_vel || !env || !out_accel) return;

    vec3_t accel;
    entity_dynamic_calc_accel(curr, prev_vel, dt, &accel);
    // linear_state_t로 변환
    linear_state_t state0;
    xform_get_position(&curr->xf, &state0.position);
    state0.velocity = curr->velocity;
    state0.acceleration = accel;

    numeq_model_accel_at(dt, &state0, env, &curr->props, out_accel);
}

void entity_dynamic_calc_drag_accel(
    const entity_dynamic_t* curr,
    const vec3_t* prev_vel,
    float dt,
    const environ_t* env,
    vec3_t* out_drag_accel
) {
    if (!curr || !prev_vel || !env || !out_drag_accel || dt <= 0.0f) {
        if (out_drag_accel) *out_drag_accel = (vec3_t){0.0f, 0.0f, 0.0f};
        return;
    }

    // 현재 가속도 계산 (a = Δv / dt)
    vec3_t accel;
    entity_dynamic_calc_accel_env(curr, prev_vel, dt, env, &accel);

    // linear_state_t 초기화
    linear_state_t state0;
    xform_get_position(&curr->xf, &state0.position);
    state0.velocity = curr->velocity;
    state0.acceleration = accel;

    // 항력 가속도 계산
    numeq_model_drag_accel(&state0, env, &curr->props, out_drag_accel);
}

void entity_dynamic_update(entity_dynamic_t* d, float dt)
{
    if (!d || dt <= 0.0f) return;

    // 위치 업데이트: p = p + v * dt
    if (!vec3_is_zero(&d->velocity)) {
        vec3_t pos;
        entity_dynamic_calc_position(d, dt, &pos);
        xform_set_position(&d->xf, &pos);
    }

    // 회전 업데이트
    if (!vec3_is_zero(&d->angular_velocity)) {
        float angle = vec3_length(&d->angular_velocity) * dt;
        if (angle > 1e-5f) {
            vec3_t axis;
            vec3_unit(&axis, &d->angular_velocity);
            xform_rotate_local_axis_angle(&d->xf, &axis, angle);
        }
    }

    // 시간 갱신
    d->base.age += dt;
}

void entity_dynamic_update_env(
    entity_dynamic_t* d,
    const environ_t* env,
    float dt)
{
    if (!d || !env || dt <= 0.0f) return;

    // linear_state_t로 변환
    linear_state_t state0;
    xform_get_position(&d->xf, &state0.position);
    state0.velocity = d->velocity;
    state0.acceleration = (vec3_t){0.0f, 0.0f, 0.0f};

    const bodyprops_t* body = &d->props;

    // 위치 업데이트 (가속도 포함)
    vec3_t new_pos;
    entity_dynamic_calc_position_env(d, env, dt, &new_pos);
    xform_set_position(&d->xf, &new_pos);

    // 회전 업데이트
    if (!vec3_is_zero(&d->angular_velocity)) {
        float angle = vec3_length(&d->angular_velocity) * dt;
        if (angle > 1e-5f) {
            vec3_t axis;
            vec3_unit(&axis, &d->angular_velocity);
            xform_rotate_local_axis_angle(&d->xf, &axis, angle);
        }
    }

    // 시간 갱신
    d->base.age += dt;
}

// ---------------------------------------------------------
// 위치 계산: p(t) = p0 + v0 * dt   (등속도 가정)
// ---------------------------------------------------------
BYUL_API void entity_dynamic_calc_position(
    const entity_dynamic_t* d,
    float dt,
    vec3_t* out_pos
) {
    if (!d || !out_pos || dt <= 0.0f) return;

    // 1. 현재 위치
    vec3_t current_pos;
    xform_get_position(&d->xf, &current_pos);

    // 2. 초기 속도 (마찰 포함)
    vec3_t v0 = d->velocity;
    bodyprops_apply_friction(&v0, &d->props, dt);

    // 3. 위치 변화 Δp = v0 * dt
    vec3_t delta;
    vec3_scale(&delta, &v0, dt);
    vec3_add(out_pos, &current_pos, &delta);
}


// ---------------------------------------------------------
// 속도 계산: v(t) = v0 (가속도 없음)
// ---------------------------------------------------------
BYUL_API void entity_dynamic_calc_velocity(
    const entity_dynamic_t* d,
    float dt,
    vec3_t* out_vel
) {
    if (!d || !out_vel) return;

    vec3_t vel = d->velocity;
    bodyprops_apply_friction(&vel, &d->props, dt);
    *out_vel = vel;
}

// ---------------------------------------------------------
// 전체 상태 예측
// ---------------------------------------------------------
BYUL_API void entity_dynamic_calc_state(
    const entity_dynamic_t* d,
    float dt,
    entity_dynamic_t* out_state
) {
    if (!d || !out_state) return;
    *out_state = *d; // 현재 상태 복사
    entity_dynamic_calc_position(d, dt, &out_state->xf.pos);
    out_state->velocity = d->velocity;
}

// ---------------------------------------------------------
// 위치 예측 (환경 영향 포함)
// p(t) = p0 + v0 * t + 0.5 * a * t²
// ---------------------------------------------------------
void entity_dynamic_calc_position_env(
    const entity_dynamic_t* d,
    const environ_t* env,
    float dt,
    vec3_t* out_pos
) {
    if (!d || !env || !out_pos || dt <= 0.0f) return;

    // 1. 현재 위치와 초기 속도
    vec3_t current_pos;
    xform_get_position(&d->xf, &current_pos);

    vec3_t v0 = d->velocity;
    bodyprops_apply_friction(&v0, &d->props, dt); // 마찰 적용

    // 2. 초기 가속도 계산 (중력 + 항력 + 외력)
    motion_state_t state0;
    entity_dynamic_to_motion_state(d, &state0, NULL, NULL);

    vec3_t a0 = {0, 0, 0};
    numeq_model_accel(&state0.linear, env, &d->props, &a0);

    // 3. 위치 변화량 계산: Δp = v0 * t + 0.5 * a0 * t²
    vec3_t term_v, term_a;
    vec3_scale(&term_v, &v0, dt);
    vec3_scale(&term_a, &a0, 0.5f * dt * dt);

    vec3_add(out_pos, &current_pos, &term_v);
    vec3_add(out_pos, out_pos, &term_a);
}


void entity_dynamic_calc_velocity_env(
    const entity_dynamic_t* d,
    const environ_t* env,
    float dt,
    vec3_t* out_vel
) {
    if (!d || !env || !out_vel) return;

    motion_state_t state0 = {};
    entity_dynamic_to_motion_state(d, &state0, nullptr, nullptr);

    numeq_model_vel_at(dt, &state0.linear, env, &d->props, out_vel);
}

void entity_dynamic_calc_state_env(
    const entity_dynamic_t* d,
    const environ_t* env,
    float dt,
    linear_state_t* out_state
) {
    if (!d || !env || !out_state) return;

    motion_state_t state0 = {};
    entity_dynamic_to_motion_state(d, &state0, nullptr, nullptr);
    entity_dynamic_calc_velocity_env(d, env, dt, &state0.linear.velocity);
    entity_dynamic_calc_position_env(d, env, dt, &state0.linear.position);

    *out_state = state0.linear;
}

void entity_dynamic_to_motion_state(
    const entity_dynamic_t* ed,
    motion_state_t* out,
    const vec3_t* lin_acc,
    const vec3_t* ang_acc)
{
    if (!ed || !out) return;

    // 선형 상태
    xform_get_position(&ed->xf, &out->linear.position);
    out->linear.velocity = ed->velocity;
    if (lin_acc) {
        out->linear.acceleration = *lin_acc;
    } else {
        vec3_zero(&out->linear.acceleration);
    }

    // 회전 상태
    quat_t orientation = ed->xf.rot;  // xform의 회전 직접 사용
    out->angular.orientation = orientation;
    out->angular.angular_velocity = ed->angular_velocity;
    if (ang_acc) {
        out->angular.angular_acceleration = *ang_acc;
    } else {
        vec3_zero(&out->angular.angular_acceleration);
    }
}

void entity_dynamic_from_motion_state(
    entity_dynamic_t* ed,
    const motion_state_t* ms)
{
    if (!ed || !ms) return;

    // 위치/회전
    xform_set_position(&ed->xf, &ms->linear.position);
    ed->xf.rot = ms->angular.orientation;

    // 선형/회전 속도
    ed->velocity = ms->linear.velocity;
    ed->angular_velocity = ms->angular.angular_velocity;
}

// ---------------------------------------------------------
// 충돌 반발 계산
// ---------------------------------------------------------
BYUL_API bool entity_dynamic_bounce(
    const entity_dynamic_t* d,
    const vec3_t* normal,
    vec3_t* out_velocity_out)
{
    if (!d || !normal || !out_velocity_out) return false;

    vec3_t n_unit = *normal;
    if (vec3_is_zero(&n_unit)) return false; // 잘못된 입력 방어
    vec3_normalize(&n_unit);

    float restitution = d->props.restitution;
    float dot = vec3_dot(&d->velocity, &n_unit);

    // v' = v - (1 + e)(v · n)n
    vec3_t scaled_normal;
    vec3_scale(&scaled_normal, &n_unit, (1.0f + restitution) * dot);
    vec3_sub(out_velocity_out, &d->velocity, &scaled_normal);

    return true;
}
