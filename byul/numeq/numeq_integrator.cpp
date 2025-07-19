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
    vec3_zero(&cfg->linear_accel);
    vec3_zero(&cfg->angular_accel);
    cfg->prev_state = nullptr;
    cfg->userdata = nullptr;
}

void integrator_config_init_full(integrator_config_t* cfg,
                                 integrator_type_t type,
                                 float time_step,
                                 const vec3_t* linear_accel,
                                 const vec3_t* angular_accel,
                                 motion_state_t* prev_state,
                                 void* userdata) {
    if (!cfg) return;
    cfg->type = type;
    cfg->time_step = time_step;
    cfg->linear_accel = *linear_accel;
    cfg->angular_accel = *angular_accel;
    cfg->prev_state = prev_state;
    cfg->userdata = userdata;
}

void integrator_config_copy(
    integrator_config_t* out, const integrator_config_t* src) {

    if (!out || !src) return;
    *out = *src;
}

// ---------------------------------------------------------
// 오일러 방식 적분 (Euler)
// ---------------------------------------------------------
void numeq_integrate_euler(motion_state_t* state,
                           const vec3_t* accel,
                           float dt) {
    Vec3 v(state->linear.velocity);
    Vec3 p(state->linear.position);
    Vec3 a(*accel);

    Vec3 v_next = v + a * dt;
    Vec3 p_next = p + v * dt;

    state->linear.velocity = v_next;
    state->linear.position = p_next;
    state->linear.acceleration = a;
}

// ---------------------------------------------------------
// 세미-묵시적 오일러 (Semi-Implicit Euler)
// ---------------------------------------------------------
void numeq_integrate_semi_implicit(motion_state_t* state,
                                   const vec3_t* accel,
                                   float dt) {
    Vec3 a(*accel);
    Vec3 v = Vec3(state->linear.velocity) + a * dt;
    Vec3 p = Vec3(state->linear.position) + v * dt;

    state->linear.velocity = v;
    state->linear.position = p;
    state->linear.acceleration = a;
}

// ---------------------------------------------------------
// Verlet 적분 (과거 위치 필요)
// ---------------------------------------------------------
void numeq_integrate_verlet(motion_state_t* state,
                            const motion_state_t* prev_state,
                            const vec3_t* accel,
                            float dt) {
    assert(state && prev_state && accel);

    Vec3 p(state->linear.position);
    Vec3 p_prev(prev_state->linear.position);
    Vec3 a(*accel);

    Vec3 new_pos = p * 2.0f - p_prev + a * (dt * dt);

    // 이전 상태를 업데이트
    *const_cast<motion_state_t*>(prev_state) = *state;
    state->linear.position = new_pos;
    // 속도는 현재-이전 위치 차분으로 추정 가능
    state->linear.velocity = (new_pos - p_prev) * (1.0f / (2.0f * dt));
    state->linear.acceleration = a;
}

// ---------------------------------------------------------
// 4차 Runge-Kutta 적분 (RK4)
// ---------------------------------------------------------
void numeq_integrate_rk4(motion_state_t* state,
                         const vec3_t* accel,
                         float dt) {
    Vec3 v0(state->linear.velocity);
    Vec3 a0(*accel);

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
void numeq_integrate_attitude_euler(
    motion_state_t* state, const vec3_t* angular_accel, float dt) {

    if (!state || !angular_accel) return;
    vec3_t* w = &state->angular.angular_velocity;
    vec3_t* a = &state->angular.angular_acceleration;

    *a = *angular_accel;
    w->x += a->x * dt;
    w->y += a->y * dt;
    w->z += a->z * dt;

    quat_t dq;
    quat_init_angular_velocity(&dq, w, dt);
    quat_mul(&state->angular.orientation, &state->angular.orientation, &dq);
    quat_normalize(&state->angular.orientation);
}

// ---------------------------------------------------------
// 회전 적분 (Semi-Implicit Euler)
// ---------------------------------------------------------
void numeq_integrate_attitude_semi_implicit(
    motion_state_t* state, const vec3_t* angular_accel, float dt) {

    if (!state || !angular_accel) return;
    vec3_t* w = &state->angular.angular_velocity;
    vec3_t* a = &state->angular.angular_acceleration;

    w->x += angular_accel->x * dt;
    w->y += angular_accel->y * dt;
    w->z += angular_accel->z * dt;
    *a = *angular_accel;

    quat_t dq;
    quat_init_angular_velocity(&dq, w, dt);
    quat_mul(&state->angular.orientation, &state->angular.orientation, &dq);
    quat_normalize(&state->angular.orientation);
}

// ---------------------------------------------------------
// 회전 적분 (RK4)
// ---------------------------------------------------------
void numeq_integrate_attitude_rk4(
    motion_state_t* state, const vec3_t* angular_accel, float dt) {

    if (!state || !angular_accel) return;
    vec3_t w0 = state->angular.angular_velocity;
    vec3_t a0 = *angular_accel;

    // 오일러 단계 근사
    vec3_t k1 = a0;
    vec3_t w_temp = {w0.x + k1.x * 0.5f * dt, 
        w0.y + k1.y * 0.5f * dt, 
        w0.z + k1.z * 0.5f * dt};

    vec3_t k2 = *angular_accel;
    vec3_t k3 = *angular_accel;
    vec3_t k4 = *angular_accel;

    w0.x += (k1.x + 2*k2.x + 2*k3.x + k4.x) * (dt/6.0f);
    w0.y += (k1.y + 2*k2.y + 2*k3.y + k4.y) * (dt/6.0f);
    w0.z += (k1.z + 2*k2.z + 2*k3.z + k4.z) * (dt/6.0f);

    state->angular.angular_velocity = w0;
    state->angular.angular_acceleration = a0;

    quat_t dq;
    quat_init_angular_velocity(&dq, &w0, dt);
    quat_mul(&state->angular.orientation, &state->angular.orientation, &dq);
    quat_normalize(&state->angular.orientation);
}

// 회전(자세) Verlet 적분
void numeq_integrate_attitude_verlet(motion_state_t* state,
                                     const motion_state_t* prev_state,
                                     const vec3_t* angular_accel,
                                     float dt) {
    assert(state && prev_state && angular_accel);

    // 각속도와 각가속도를 업데이트
    Vec3 w(state->angular.angular_velocity);
    Vec3 w_prev(prev_state->angular.angular_velocity);
    Vec3 a(*angular_accel);

    Vec3 w_new = w * 2.0f - w_prev + a * (dt * dt);

    *const_cast<motion_state_t*>(prev_state) = *state;
    state->angular.angular_velocity = w_new;
    state->angular.angular_acceleration = a;

    // 쿼터니언 업데이트
    quat_t dq;
    quat_init_angular_velocity(&dq, &w_new.v, dt);
    quat_mul(&state->angular.orientation, &state->angular.orientation, &dq);
    quat_normalize(&state->angular.orientation);
}


// 선형 + 회전 통합 Verlet 적분기
void numeq_integrate_motion_verlet(motion_state_t* state,
                                   const motion_state_t* prev_state,
                                   const vec3_t* accel,
                                   const vec3_t* angular_accel,
                                   float dt) {
    assert(state && prev_state && accel && angular_accel);

    // ---- 선형 Verlet ----
    Vec3 p(state->linear.position);
    Vec3 p_prev(prev_state->linear.position);
    Vec3 a(*accel);

    Vec3 new_pos = p * 2.0f - p_prev + a * (dt * dt);

    // 이전 상태 업데이트
    *const_cast<motion_state_t*>(prev_state) = *state;

    state->linear.position = new_pos;
    state->linear.velocity = (new_pos - p_prev) * (1.0f / (2.0f * dt));
    state->linear.acceleration = a;

    // ---- 회전 Verlet ----
    Vec3 w(state->angular.angular_velocity);
    Vec3 w_prev(prev_state->angular.angular_velocity);
    Vec3 ang_a(*angular_accel);

    Vec3 w_new = w * 2.0f - w_prev + ang_a * (dt * dt);

    state->angular.angular_velocity = w_new;
    state->angular.angular_acceleration = ang_a;

    // 쿼터니언 회전 업데이트
    quat_t dq;
    quat_init_angular_velocity(&dq, &w_new.v, dt);
    quat_mul(&state->angular.orientation, &state->angular.orientation, &dq);
    quat_normalize(&state->angular.orientation);
}

// 선형 + 회전 통합 Euler 적분기
void numeq_integrate_motion_euler(motion_state_t* state,
                                  const vec3_t* accel,
                                  const vec3_t* angular_accel,
                                  float dt) {
    assert(state && accel && angular_accel);

    // ---- 선형 Euler ----
    Vec3 v(state->linear.velocity);
    Vec3 p(state->linear.position);
    Vec3 a(*accel);

    state->linear.velocity = v + a * dt;
    state->linear.position = p + v * dt;
    state->linear.acceleration = a;

    // ---- 회전 Euler ----
    Vec3 w(state->angular.angular_velocity);
    Vec3 ang_a(*angular_accel);
    w = w + ang_a * dt;

    state->angular.angular_velocity = w;
    state->angular.angular_acceleration = ang_a;

    quat_t dq;
    quat_init_angular_velocity(&dq, &w.v, dt);
    quat_mul(&state->angular.orientation, &state->angular.orientation, &dq);
    quat_normalize(&state->angular.orientation);
}

// 선형 + 회전 통합 Semi-Implicit Euler 적분기
void numeq_integrate_motion_semi_implicit(motion_state_t* state,
                                          const vec3_t* accel,
                                          const vec3_t* angular_accel,
                                          float dt) {
    assert(state && accel && angular_accel);

    // ---- 선형 Semi-Implicit Euler ----
    Vec3 a(*accel);
    Vec3 v = Vec3(state->linear.velocity) + a * dt;
    Vec3 p = Vec3(state->linear.position) + v * dt;

    state->linear.velocity = v;
    state->linear.position = p;
    state->linear.acceleration = a;

    // ---- 회전 Semi-Implicit Euler ----
    Vec3 w = Vec3(state->angular.angular_velocity) + Vec3(*angular_accel) * dt;
    state->angular.angular_velocity = w;
    state->angular.angular_acceleration = *angular_accel;

    quat_t dq;
    quat_init_angular_velocity(&dq, &w.v, dt);
    quat_mul(&state->angular.orientation, &state->angular.orientation, &dq);
    quat_normalize(&state->angular.orientation);
}

// 선형 + 회전 통합 RK4 적분기
void numeq_integrate_motion_rk4(motion_state_t* state,
                                const vec3_t* accel,
                                const vec3_t* angular_accel,
                                float dt) {
    assert(state && accel && angular_accel);

    // ---- 선형 RK4 ----
    Vec3 v0(state->linear.velocity);
    Vec3 a0(*accel);

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

    // ---- 회전 RK4 ----
    Vec3 w0(state->angular.angular_velocity);
    Vec3 ang_a(*angular_accel);

    Vec3 k1_w = ang_a * dt;
    Vec3 k2_w = ang_a * dt;
    Vec3 k3_w = ang_a * dt;
    Vec3 k4_w = ang_a * dt;

    Vec3 delta_w = (k1_w + (k2_w + k3_w) * 2.0f + k4_w) * (1.0f / 6.0f);
    Vec3 w_new = w0 + delta_w;

    state->angular.angular_velocity = w_new;
    state->angular.angular_acceleration = ang_a;

    quat_t dq;
    quat_init_angular_velocity(&dq, &w_new.v, dt);
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
            numeq_integrate_euler(
                state, &config->linear_accel, config->time_step);
            break;
        case INTEGRATOR_SEMI_IMPLICIT:
            numeq_integrate_semi_implicit(
                state, &config->linear_accel, config->time_step);
            break;
        case INTEGRATOR_RK4:
            numeq_integrate_rk4(
                state, &config->linear_accel, config->time_step);
            break;
        case INTEGRATOR_VERLET:
            if (config->prev_state) {
                numeq_integrate_verlet(state, config->prev_state, 
                    &config->linear_accel, config->time_step);
            } else {
                assert(false && "Verlet integration requires prev_state");
            }
            break;
        case INTEGRATOR_MOTION_EULER:
            numeq_integrate_motion_euler(state, &config->linear_accel, 
                &config->angular_accel, config->time_step);
            break;
        case INTEGRATOR_MOTION_SEMI_IMPLICIT:
            numeq_integrate_motion_semi_implicit(state, &config->linear_accel, 
                &config->angular_accel, config->time_step);
            break;
        case INTEGRATOR_MOTION_RK4:
            numeq_integrate_motion_rk4(state, &config->linear_accel, 
                &config->angular_accel, config->time_step);
            break;
        case INTEGRATOR_MOTION_VERLET:
            if (config->prev_state) {
                numeq_integrate_motion_verlet(state, config->prev_state, 
                    &config->linear_accel, &config->angular_accel, 
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
