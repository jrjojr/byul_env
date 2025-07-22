#include <cmath>
#include <cstring>
#include "internal/entity_dynamic.h"
#include "internal/vec3.h"
#include "internal/xform.h"
#include "internal/bodyprops.h"
#include "internal/trajectory.h"

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

// ---------------------------------------------------------
// 동적 업데이트
// ---------------------------------------------------------
void entity_dynamic_update(entity_dynamic_t* d, float dt)
{
    if (!d || dt <= 0.0f) return;

    // 위치 업데이트: p = p + v * dt
    if (!vec3_is_zero(&d->velocity)) {
        vec3_t delta;
        vec3_scale(&delta, &d->velocity, dt);
        xform_translate(&d->xf, &delta);
    }

    // 회전 업데이트: angular_velocity를 기반으로 회전
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

void entity_dynamic_predict_position(
    const entity_dynamic_t* d,
    float dt,
    vec3_t* out_pos
) {
    if (!d || !out_pos || dt < 0.0f) return;

    // 현재 위치 가져오기
    vec3_t current_pos;
    xform_get_position(&d->xf, &current_pos);

    // delta_pos = velocity * dt
    vec3_t delta;
    vec3_scale(&delta, &d->velocity, dt);

    // out_pos = current_pos + delta_pos
    vec3_add(out_pos, &current_pos, &delta);
}

static entity_dynamic_bounce_func g_bounce_func = NULL;
static void* g_bounce_userdata = NULL;

// ---------------------------------------------------------
// 내부 헬퍼: 공기저항 가속도 계산
// ---------------------------------------------------------
static void _compute_drag_accel(
    const vec3_t* velocity,
    const bodyprops_t* body,
    float air_density,
    vec3_t* out_drag
) {
    vec3_zero(out_drag);
    float speed = vec3_length(velocity);
    if (speed < 1e-6f) return;

    float drag_force = 0.5f * air_density *
                       body->drag_coef *
                       body->cross_section *
                       speed * speed;
    float inv_mass = 1.0f / (body->mass > 1e-6f ? body->mass : 1.0f);
    vec3_scale(out_drag, velocity, -drag_force * inv_mass / speed);
}

// ---------------------------------------------------------
// 위치 계산: p(t)
// ---------------------------------------------------------
void entity_dynamic_predict_position_env(
    const entity_dynamic_t* d,
    const environ_t* env,
    float dt,
    vec3_t* out_pos
) {
    if (!d || !env || !out_pos) return;

    // 현재 위치
    vec3_t p0;
    xform_get_position(&d->xf, &p0);

    // 가속도 계산
    vec3_t accel;
    entity_dynamic_predict_accel_env(d, env, &accel);

    // p(t) = p0 + v * dt + 0.5 * a * dt²
    vec3_t v_dt, a_dt2;
    vec3_scale(&v_dt, &d->velocity, dt);
    vec3_scale(&a_dt2, &accel, 0.5f * dt * dt);
    vec3_add(out_pos, &p0, &v_dt);
    vec3_add(out_pos, out_pos, &a_dt2);
}

// ---------------------------------------------------------
// 속도 계산: v(t)
// ---------------------------------------------------------
void entity_dynamic_predict_velocity_env(
    const entity_dynamic_t* d,
    const environ_t* env,
    float dt,
    vec3_t* out_vel
) {
    if (!d || !env || !out_vel) return;

    vec3_t accel;
    entity_dynamic_predict_accel_env(d, env, &accel);

    // v(t) = v0 + a * dt
    vec3_scale(out_vel, &accel, dt);
    vec3_add(out_vel, &d->velocity, out_vel);
}

// ---------------------------------------------------------
// 가속도 계산: a(t)
// ---------------------------------------------------------
void entity_dynamic_predict_accel_env(
    const entity_dynamic_t* d,
    const environ_t* env,
    vec3_t* out_accel
) {
    if (!d || !env || !out_accel) return;

    // a = gravity + wind + drag
    *out_accel = env->gravity;
    vec3_add(out_accel, out_accel, &env->wind);

    vec3_t drag;
    _compute_drag_accel(&d->velocity, &d->props, env->air_density, &drag);
    vec3_add(out_accel, out_accel, &drag);
}

void entity_dynamic_predict_state_env(
    const entity_dynamic_t* d,
    const environ_t* env,
    float dt,
    entity_dynamic_t* out_state
) {
    if (!d || !env || !out_state) return;
    *out_state = *d; // 현재 상태를 복사

    // 새로운 위치 계산
    vec3_t new_pos;
    entity_dynamic_predict_position_env(d, env, dt, &new_pos);
    xform_set_position(&out_state->xf, &new_pos);

    // 새로운 속도 계산
    entity_dynamic_predict_velocity_env(d, env, dt, &out_state->velocity);
}

// ---------------------------------------------------------
// 공기 저항력 계산
// ---------------------------------------------------------
void entity_dynamic_drag_accel_env(
    const entity_dynamic_t* d,
    const environ_t* env,
    vec3_t* out_drag_accel
) {
    if (!d || !env || !out_drag_accel) return;
    _compute_drag_accel(&d->velocity, &d->props, env->air_density, out_drag_accel);
}

// ---------------------------------------------------------
// 최고점 여부 판단 (vy ≈ 0)
// ---------------------------------------------------------
bool entity_dynamic_is_apex(const entity_dynamic_t* d)
{
    if (!d) return false;
    return fabsf(d->velocity.y) < 1e-3f;
}

// ---------------------------------------------------------
// 착지 여부 판단
// ---------------------------------------------------------
bool entity_dynamic_is_grounded(
    const entity_dynamic_t* d,
    float ground_height
) {
    if (!d) return false;
    vec3_t p;
    xform_get_position(&d->xf, &p);
    return p.y <= ground_height;
}

// ---------------------------------------------------------
// 충돌 반발 계산
// ---------------------------------------------------------
void entity_dynamic_set_bounce_func(
    entity_dynamic_bounce_func func,
    void* userdata
) {
    g_bounce_func = func;
    g_bounce_userdata = userdata;
}

void entity_dynamic_get_bounce_func(
    entity_dynamic_bounce_func* out_func,
    void** out_userdata
) {
    if (out_func) *out_func = g_bounce_func;
    if (out_userdata) *out_userdata = g_bounce_userdata;
}

bool entity_dynamic_default_bounce(
    const vec3_t* velocity_in,
    const vec3_t* normal,
    float restitution,
    vec3_t* out_velocity_out
) {
    if (!velocity_in || !normal || !out_velocity_out) return false;

    // v' = v - (1 + e)(v · n)n
    float dot = vec3_dot(velocity_in, normal);
    vec3_t scaled_normal;
    vec3_scale(&scaled_normal, normal, (1.0f + restitution) * dot);
    vec3_sub(out_velocity_out, velocity_in, &scaled_normal);
    return true;
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
    quat_t orientation;
    vec3_t tmp;
    dualquat_to_quat_vec(&ed->xf.dq, &orientation, &tmp);
    out->angular.orientation = orientation;
    out->angular.angular_velocity = ed->angular_velocity;
    if (ang_acc) {
        out->angular.angular_acceleration = *ang_acc;
    } else {
        vec3_zero(&out->angular.angular_acceleration);
    }
}

void entity_dynamic_from_motion_state(
    entity_dynamic_t* ed, const motion_state_t* ms) {
    if (!ed || !ms) return;

    // 위치/회전
    dualquat_init_quat_vec(
        &ed->xf.dq, &ms->angular.orientation, &ms->linear.position);

    ed->velocity = ms->linear.velocity;
    ed->angular_velocity = ms->angular.angular_velocity;
}
