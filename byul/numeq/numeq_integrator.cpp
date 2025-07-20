// linear_state_t -> motion_state_t로 변경한 버전

#include "internal/numeq_integrator.h"
#include "internal/vec3.hpp"
#include <cassert>

// ---------------------------------------------------------
// integrator_config_t 유틸리티
// ---------------------------------------------------------
void integrator_config_init(integrator_config_t* cfg) {
    if (!cfg) return;
    cfg->type = INTEGRATOR_EULER;
    cfg->time_step = 0.016f; // 기본 60Hz
    cfg->prev_state = nullptr;
    cfg->userdata = nullptr;
}

void integrator_config_init_full(integrator_config_t* cfg,
                                 integrator_type_t type,
                                 float time_step,
                                 motion_state_t* prev_state,
                                 void* userdata) {
    if (!cfg) return;
    cfg->type = type;
    cfg->time_step = time_step;
    cfg->prev_state = prev_state;
    cfg->userdata = userdata;
}

void integrator_config_assign(
    integrator_config_t* out, const integrator_config_t* src) {

    if (!out || !src) return;
    *out = *src;
}

// ---------------------------------------------------------
// 오일러 방식 적분 (Euler)
// ---------------------------------------------------------
void numeq_integrate_euler(
    motion_state_t* state, float dt) {

    Vec3 v(state->linear.velocity);
    Vec3 p(state->linear.position);
    Vec3 a(state->linear.acceleration);

    Vec3 v_next = v + a * dt;
    Vec3 p_next = p + v * dt;

    state->linear.velocity = v_next;
    state->linear.position = p_next;
    state->linear.acceleration = a;
}

// ---------------------------------------------------------
// 세미-묵시적 오일러 (Semi-Implicit Euler)
// ---------------------------------------------------------
void numeq_integrate_semi_implicit(
    motion_state_t* state, float dt) {

    Vec3 a = Vec3(state->linear.acceleration);
    Vec3 v = Vec3(state->linear.velocity) + a * dt;
    Vec3 p = Vec3(state->linear.position) + v * dt;

    state->linear.velocity = v;
    state->linear.position = p;
    state->linear.acceleration = a;
}

// ---------------------------------------------------------
// Verlet 적분 (과거 위치 필요)
// ---------------------------------------------------------
void numeq_integrate_verlet(
    motion_state_t* state, const motion_state_t* prev_state, float dt) {
        
    assert(state && prev_state);

    Vec3 p(state->linear.position);
    Vec3 p_prev(prev_state->linear.position);
    Vec3 a(state->linear.acceleration);

    // 새 위치 = 2p - p_prev + a * dt^2
    Vec3 new_pos = p * 2.0f - p_prev + a * (dt * dt);

    // 속도는 중앙 차분 방식으로 추정
    Vec3 vel = (new_pos - p_prev) * (1.0f / (2.0f * dt));

    state->linear.position = new_pos;
    state->linear.velocity = vel;
}


// ---------------------------------------------------------
// 4차 Runge-Kutta 적분 (RK4)
// ---------------------------------------------------------
void numeq_integrate_rk4(
    motion_state_t* state, float dt) {
    Vec3 v0(state->linear.velocity);
    Vec3 a0(state->linear.acceleration);

    Vec3 k1_v = a0 * dt;
    Vec3 k1_p = v0 * dt;

    Vec3 k2_v = a0 * dt;
    Vec3 k2_p = (v0 + k1_v * 0.5f) * dt;

    Vec3 k3_v = a0 * dt;
    Vec3 k3_p = (v0 + k2_v * 0.5f) * dt;

    Vec3 k4_v = a0 * dt;
    Vec3 k4_p = (v0 + k3_v) * dt;

    Vec3 delta_v = (k1_v + (k2_v + k3_v) * 2.0f + k4_v) * (1.0f / 6.0f);
    Vec3 delta_p = (k1_p + (k2_p + k3_p) * 2.0f + k4_p) * (1.0f / 6.0f);

    state->linear.velocity = v0 + delta_v;
    state->linear.position = Vec3(state->linear.position) + delta_p;
    state->linear.acceleration = a0;
}

// ---------------------------------------------------------
// 회전 적분 (Angular Euler)
// ---------------------------------------------------------
void numeq_integrate_attitude_euler(motion_state_t* state, float dt) {
    if (!state) return;
    vec3_t* w = &state->angular.angular_velocity;
    vec3_t* a = &state->angular.angular_acceleration;

    // w = w + a * dt
    w->x += a->x * dt;
    w->y += a->y * dt;
    w->z += a->z * dt;

    // 각속도 → 쿼터니언 변환
    quat_t dq;
    quat_init_angular_velocity(&dq, w, dt);
    quat_mul(&state->angular.orientation, &state->angular.orientation, &dq);
    quat_normalize(&state->angular.orientation);
}


// ---------------------------------------------------------
// 회전 적분 (Semi-Implicit Euler)
// ---------------------------------------------------------
void numeq_integrate_attitude_semi_implicit(
    motion_state_t* state, float dt) {

    if (!state) return;
    vec3_t* w = &state->angular.angular_velocity;
    const vec3_t* a = &state->angular.angular_acceleration;

    // w = w + a * dt
    w->x += a->x * dt;
    w->y += a->y * dt;
    w->z += a->z * dt;

    // q = q * dq
    quat_t dq;
    quat_init_angular_velocity(&dq, w, dt);
    quat_mul(&state->angular.orientation, &state->angular.orientation, &dq);
    quat_normalize(&state->angular.orientation);
}


// ---------------------------------------------------------
// 회전 적분 (RK4)
// ---------------------------------------------------------
void numeq_integrate_attitude_rk4(
    motion_state_t* state, float dt) {

    if (!state) return;
    vec3_t w0 = state->angular.angular_velocity;
    const vec3_t a0 = state->angular.angular_acceleration;

    // RK4 각속도 업데이트
    vec3_t k1 = a0;
    vec3_t k2 = a0;
    vec3_t k3 = a0;
    vec3_t k4 = a0;

    w0.x += (k1.x + 2*k2.x + 2*k3.x + k4.x) * (dt / 6.0f);
    w0.y += (k1.y + 2*k2.y + 2*k3.y + k4.y) * (dt / 6.0f);
    w0.z += (k1.z + 2*k2.z + 2*k3.z + k4.z) * (dt / 6.0f);

    state->angular.angular_velocity = w0;

    quat_t dq;
    quat_init_angular_velocity(&dq, &w0, dt);
    quat_mul(&state->angular.orientation, &state->angular.orientation, &dq);
    quat_normalize(&state->angular.orientation);
}


// 회전(자세) Verlet 적분
void numeq_integrate_attitude_verlet(
    motion_state_t* state, const motion_state_t* prev_state, float dt) {

    assert(state && prev_state);

    Vec3 w(state->angular.angular_velocity);
    Vec3 w_prev(prev_state->angular.angular_velocity);
    Vec3 a(state->angular.angular_acceleration);

    Vec3 w_create = w * 2.0f - w_prev + a * (dt * dt);

    *const_cast<motion_state_t*>(prev_state) = *state;
    state->angular.angular_velocity = w_create;

    quat_t dq;
    quat_init_angular_velocity(&dq, &w_create.v, dt);
    quat_mul(&state->angular.orientation, &state->angular.orientation, &dq);
    quat_normalize(&state->angular.orientation);
}



// // 선형 + 회전 통합 Verlet 적분기
void numeq_integrate_motion_verlet(
    motion_state_t* state,
    motion_state_t* prev_state,
    float dt) {

    assert(state && prev_state);

    // ---- 선형 Verlet ----
    Vec3 p(state->linear.position);
    Vec3 p_prev(prev_state->linear.position);
    Vec3 a(state->linear.acceleration);

    Vec3 new_pos = p * 2.0f - p_prev + a * (dt * dt);

    // 이전 상태를 갱신
    *prev_state = *state;

    state->linear.position = new_pos;
    state->linear.velocity = (new_pos - p_prev) * (1.0f / (2.0f * dt));

    // ---- 회전 Verlet ----
    Vec3 w(state->angular.angular_velocity);
    Vec3 w_prev(prev_state->angular.angular_velocity);
    Vec3 ang_a(state->angular.angular_acceleration);

    Vec3 w_create = w * 2.0f - w_prev + ang_a * (dt * dt);

    state->angular.angular_velocity = w_create;

    // 쿼터니언 회전 업데이트
    quat_t dq;
    quat_init_angular_velocity(&dq, &w_create.v, dt);
    quat_mul(&state->angular.orientation, &state->angular.orientation, &dq);
    quat_normalize(&state->angular.orientation);
}

// 선형 + 회전 통합 Euler 적분기
void numeq_integrate_motion_euler(motion_state_t* state, float dt) {
    assert(state);

    // ---- 선형 Euler ----
    vec3_t* v = &state->linear.velocity;
    vec3_t* p = &state->linear.position;
    const vec3_t* a = &state->linear.acceleration;

    // v = v + a * dt
    v->x += a->x * dt;
    v->y += a->y * dt;
    v->z += a->z * dt;

    // p = p + v * dt
    p->x += v->x * dt;
    p->y += v->y * dt;
    p->z += v->z * dt;

    // ---- 회전 Euler ----
    vec3_t* w = &state->angular.angular_velocity;
    const vec3_t* ang_a = &state->angular.angular_acceleration;

    // w = w + ang_a * dt
    w->x += ang_a->x * dt;
    w->y += ang_a->y * dt;
    w->z += ang_a->z * dt;

    // q = q * dq
    quat_t dq;
    quat_init_angular_velocity(&dq, w, dt);
    quat_mul(&state->angular.orientation, &state->angular.orientation, &dq);
    quat_normalize(&state->angular.orientation);
}

// 선형 + 회전 통합 Semi-Implicit Euler 적분기 (accel 파라미터 제거)
void numeq_integrate_motion_semi_implicit(motion_state_t* state, float dt) {
    assert(state);

    // ---- 선형 Semi-Implicit Euler ----
    vec3_t* v = &state->linear.velocity;
    vec3_t* p = &state->linear.position;
    const vec3_t* a = &state->linear.acceleration;

    // v = v + a * dt
    v->x += a->x * dt;
    v->y += a->y * dt;
    v->z += a->z * dt;

    // p = p + v * dt
    p->x += v->x * dt;
    p->y += v->y * dt;
    p->z += v->z * dt;

    // ---- 회전 Semi-Implicit Euler ----
    vec3_t* w = &state->angular.angular_velocity;
    const vec3_t* ang_a = &state->angular.angular_acceleration;

    // w = w + ang_a * dt
    w->x += ang_a->x * dt;
    w->y += ang_a->y * dt;
    w->z += ang_a->z * dt;

    // q = q * dq
    quat_t dq;
    quat_init_angular_velocity(&dq, w, dt);
    quat_mul(&state->angular.orientation, &state->angular.orientation, &dq);
    quat_normalize(&state->angular.orientation);
}

// 선형 + 회전 통합 RK4 적분기 (accel 파라미터 제거)
void numeq_integrate_motion_rk4(motion_state_t* state, float dt) {
    assert(state);

    // ---- 선형 RK4 ----
    Vec3 v0(state->linear.velocity);
    Vec3 a0(state->linear.acceleration);

    Vec3 k1_v = a0 * dt;
    Vec3 k1_p = v0 * dt;

    Vec3 k2_v = a0 * dt;
    Vec3 k2_p = (v0 + k1_v * 0.5f) * dt;

    Vec3 k3_v = a0 * dt;
    Vec3 k3_p = (v0 + k2_v * 0.5f) * dt;

    Vec3 k4_v = a0 * dt;
    Vec3 k4_p = (v0 + k3_v) * dt;

    Vec3 delta_v = (k1_v + (k2_v + k3_v) * 2.0f + k4_v) * (1.0f / 6.0f);
    Vec3 delta_p = (k1_p + (k2_p + k3_p) * 2.0f + k4_p) * (1.0f / 6.0f);

    state->linear.velocity = v0 + delta_v;
    state->linear.position = Vec3(state->linear.position) + delta_p;

    // ---- 회전 RK4 ----
    Vec3 w0(state->angular.angular_velocity);
    Vec3 ang_a(state->angular.angular_acceleration);

    Vec3 k1_w = ang_a * dt;
    Vec3 k2_w = ang_a * dt;
    Vec3 k3_w = ang_a * dt;
    Vec3 k4_w = ang_a * dt;

    Vec3 delta_w = (k1_w + (k2_w + k3_w) * 2.0f + k4_w) * (1.0f / 6.0f);
    Vec3 w_create = w0 + delta_w;

    state->angular.angular_velocity = w_create;

    quat_t dq;
    quat_init_angular_velocity(&dq, &w_create.v, dt);
    quat_mul(&state->angular.orientation, &state->angular.orientation, &dq);
    quat_normalize(&state->angular.orientation);
}

// ---------------------------------------------------------
// 공통 적분기 인터페이스
// ---------------------------------------------------------
void numeq_integrate(
    motion_state_t* state, const integrator_config_t* config) {

    switch (config->type) {
        case INTEGRATOR_EULER:
            numeq_integrate_euler(state, config->time_step);
            break;
        case INTEGRATOR_SEMI_IMPLICIT:
            numeq_integrate_semi_implicit(state, config->time_step);
            break;
        case INTEGRATOR_RK4:
            numeq_integrate_rk4(state, config->time_step);
            break;
        case INTEGRATOR_VERLET:
            if (config->prev_state) {
                numeq_integrate_verlet(state, config->prev_state, 
                    config->time_step);
            } else {
                assert(false && "Verlet integration requires prev_state");
            }
            break;
        case INTEGRATOR_MOTION_EULER:
            numeq_integrate_motion_euler(state, config->time_step);
            break;
        case INTEGRATOR_MOTION_SEMI_IMPLICIT:
            numeq_integrate_motion_semi_implicit(state, config->time_step);
            break;
        case INTEGRATOR_MOTION_RK4:
            numeq_integrate_motion_rk4(state, config->time_step);
            break;
        case INTEGRATOR_MOTION_VERLET:
            if (config->prev_state) {
                numeq_integrate_motion_verlet(state, config->prev_state, 
                    config->time_step);
            } else {
                assert(
                    false && "Motion Verlet integration requires prev_state");
            }
            break;
        default:
            assert(false && "Unknown integration type");
            break;
    }
}
