// linear_state_t -> motion_state_t로 변경한 버전

#include "internal/numeq_integrator.h"
#include "internal/numeq_model.h"
#include "internal/vec3.hpp"
#include <cassert>

// ---------------------------------------------------------
// integrator_config_t 유틸리티
// ---------------------------------------------------------
void integrator_config_init(integrator_config_t* cfg) {
    if (!cfg) return;
    cfg->type = INTEGRATOR_RK4_ENV;
    cfg->time_step = 0.016f; // 기본 60Hz
    cfg->prev_state = nullptr;
    cfg->env = nullptr;
    cfg->body = nullptr;
    cfg->userdata = nullptr;
}

void integrator_config_init_full(integrator_config_t* cfg,
                                 integrator_type_t type,
                                 float time_step,
                                 motion_state_t* prev_state,
                                 const environ_t* env,
                                 const bodyprops_t* body,
                                 void* userdata)
{
    if (!cfg) return;

    cfg->type = type;
    cfg->time_step = time_step;
    cfg->prev_state = prev_state;
    cfg->env = env;
    cfg->body = body;
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

void numeq_integrate_rk4_env(motion_state_t* state,
                             float dt,
                             const environ_t* env,
                             const bodyprops_t* body)
{
    assert(state);

    if (!env && !body) {
        numeq_integrate_rk4(state, dt);
        return;
    }
    
    environ_t a_env = env ? *env : (environ_t){0};
    bodyprops_t a_body = body ? *body : (bodyprops_t){0};

    // 초기 상태
    Vec3 p0(state->linear.position);
    Vec3 v0(state->linear.velocity);

    // ===== RK4 선형 운동 =====
    // k1
    vec3_t a1;
    numeq_model_accel_at(0.0f, &state->linear, env, body, &a1);
    Vec3 k1_p = v0 * dt;
    Vec3 k1_v = Vec3(a1) * dt;

    // k2
    linear_state_t tmp2 = state->linear;
    tmp2.velocity = v0 + k1_v * 0.5f;

    vec3_t a2;
    numeq_model_accel_at(dt * 0.5f, &tmp2, env, body, &a2);
    Vec3 k2_p = (v0 + k1_v * 0.5f) * dt;
    Vec3 k2_v = Vec3(a2) * dt;

    // k3
    linear_state_t tmp3 = state->linear;
    vec3_t v_half3 = {v0.v.x + 0.5f * k2_v.v.x,
                      v0.v.y + 0.5f * k2_v.v.y,
                      v0.v.z + 0.5f * k2_v.v.z};
    tmp3.velocity = v_half3;
    vec3_t a3;
    numeq_model_accel_at(dt * 0.5f, &tmp3, env, body, &a3);
    Vec3 k3_p = (v0 + k2_v * 0.5f) * dt;
    Vec3 k3_v = Vec3(a3) * dt;

    // k4
    linear_state_t tmp4 = state->linear;
    vec3_t v_full = {v0.v.x + k3_v.v.x,
                     v0.v.y + k3_v.v.y,
                     v0.v.z + k3_v.v.z};
    tmp4.velocity = v_full;
    vec3_t a4;
    numeq_model_accel_at(dt, &tmp4, env, body, &a4);
    Vec3 k4_p = (v0 + k3_v) * dt;
    Vec3 k4_v = Vec3(a4) * dt;

    // 업데이트
    Vec3 dp = (k1_p + (k2_p + k3_p) * 2.0f + k4_p) * (1.0f / 6.0f);
    Vec3 dv = (k1_v + (k2_v + k3_v) * 2.0f + k4_v) * (1.0f / 6.0f);

    state->linear.position = p0 + dp;
    state->linear.velocity = v0 + dv;
    state->linear.acceleration = a4;  // 마지막 스텝 가속도로 갱신
}

// void numeq_integrate_rk4_env(motion_state_t* state,
//                              float dt,
//                              const environ_t* env,
//                              const bodyprops_t* body)
// {
//     assert(state);

//     if (!env && !body) {
//         numeq_integrate_rk4(state, dt);
//         return;
//     }

//     environ_t a_env = env ? *env : (environ_t){0};
//     bodyprops_t a_body = body ? *body : (bodyprops_t){0};

//     // 초기 상태
//     Vec3 p0(state->linear.position);
//     Vec3 v0(state->linear.velocity);
//     Vec3 g = env ? Vec3(env->gravity) : Vec3(0.0f, 0.0f, 0.0f);

//     // ===== RK4 선형 운동 =====
//     // k1
//     vec3_t a1_ext;
//     numeq_model_accel_except_gravity(&state->linear, &a_env, &a_body, &a1_ext);
//     Vec3 a1 = Vec3(a1_ext) + g;
//     Vec3 k1_p = v0 * dt;
//     Vec3 k1_v = a1 * dt;

//     // k2
//     linear_state_t tmp2 = state->linear;
//     tmp2.velocity = v0 + k1_v * 0.5f;
//     vec3_t a2_ext;
//     numeq_model_accel_except_gravity(&tmp2, &a_env, &a_body, &a2_ext);
//     Vec3 a2 = Vec3(a2_ext) + g;
//     Vec3 k2_p = (v0 + k1_v * 0.5f) * dt;
//     Vec3 k2_v = a2 * dt;

//     // k3
//     linear_state_t tmp3 = state->linear;
//     tmp3.velocity = v0 + k2_v * 0.5f;
//     vec3_t a3_ext;
//     numeq_model_accel_except_gravity(&tmp3, &a_env, &a_body, &a3_ext);
//     Vec3 a3 = Vec3(a3_ext) + g;
//     Vec3 k3_p = (v0 + k2_v * 0.5f) * dt;
//     Vec3 k3_v = a3 * dt;

//     // k4
//     linear_state_t tmp4 = state->linear;
//     tmp4.velocity = v0 + k3_v;
//     vec3_t a4_ext;
//     numeq_model_accel_except_gravity(&tmp4, &a_env, &a_body, &a4_ext);
//     Vec3 a4 = Vec3(a4_ext) + g;
//     Vec3 k4_p = (v0 + k3_v) * dt;
//     Vec3 k4_v = a4 * dt;

//     // 최종 업데이트
//     Vec3 dp = (k1_p + 2.0f * (k2_p + k3_p) + k4_p) / 6.0f;
//     Vec3 dv = (k1_v + 2.0f * (k2_v + k3_v) + k4_v) / 6.0f;

//     state->linear.position = p0 + dp;
//     state->linear.velocity = v0 + dv;
//     state->linear.acceleration = a4;  // 마지막 단계의 전체 가속도
// }

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

// ---------------------------------------------------------
// 회전 적분 (RK4 + 환경 반영)
// ---------------------------------------------------------
void numeq_integrate_attitude_rk4_env(motion_state_t* state,
                                      float dt,
                                      const environ_t* env,
                                      const bodyprops_t* body)
{
    if (!state) return;

    if(!env  || !body){
        numeq_integrate_attitude_rk4(state, dt);
        return;
    }

    // 초기 각속도 및 각가속도
    vec3_t w0 = state->angular.angular_velocity;
    vec3_t alpha0 = state->angular.angular_acceleration;

    // -----------------------------------------------------
    // 환경 영향: 예를 들어 공기 저항에 의한 회전 감쇠 토크를 추가할 수 있음.
    // 단순 모델: alpha0 -= k_drag * w0 (선형 감쇠 가정)
    // 여기서 k_drag는 공기 밀도, 단면적, 드래그 계수에 비례
    // -----------------------------------------------------
    float angular_drag_coeff = 0.5f * env->air_density 
    * body->drag_coef * body->cross_section / (body->mass + 1e-6f);
    
    vec3_t drag_torque = {
        -angular_drag_coeff * w0.x,
        -angular_drag_coeff * w0.y,
        -angular_drag_coeff * w0.z
    };
    alpha0.x += drag_torque.x;
    alpha0.y += drag_torque.y;
    alpha0.z += drag_torque.z;

    // -----------------------------------------------------
    // RK4 적분 (각속도)
    // -----------------------------------------------------
    vec3_t k1 = alpha0;
    vec3_t k2 = alpha0; // 선형 시스템이므로 동일
    vec3_t k3 = alpha0;
    vec3_t k4 = alpha0;

    w0.x += (k1.x + 2*k2.x + 2*k3.x + k4.x) * (dt / 6.0f);
    w0.y += (k1.y + 2*k2.y + 2*k3.y + k4.y) * (dt / 6.0f);
    w0.z += (k1.z + 2*k2.z + 2*k3.z + k4.z) * (dt / 6.0f);

    state->angular.angular_velocity = w0;

    // -----------------------------------------------------
    // 쿼터니언 업데이트
    // -----------------------------------------------------
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

void numeq_integrate_motion_rk4_env(motion_state_t* state,
                                    float dt,
                                    const environ_t* env,
                                    const bodyprops_t* body)
{
    assert(state);

    // 환경이나 물체 특성이 없으면 기본 RK4 호출
    if (!env || !body) {
        numeq_integrate_motion_rk4(state, dt);
        return;
    }

    environ_t a_env = env ? *env : (environ_t){0};
    bodyprops_t a_body = body ? *body : (bodyprops_t){0};

    // -----------------------------
    // 선형 운동 (Linear Motion)
    // -----------------------------
    Vec3 p0(state->linear.position);
    Vec3 v0(state->linear.velocity);

    // k1
    vec3_t a1;
    numeq_model_accel(&state->linear, &a_env, &a_body, &a1);
    Vec3 k1_p = v0 * dt;
    Vec3 k1_v = Vec3(a1) * dt;

    // k2
    linear_state_t tmp2 = state->linear;
    tmp2.velocity = v0 + k1_v * 0.5f;
    vec3_t a2;
    numeq_model_accel(&tmp2, &a_env, &a_body, &a2);
    Vec3 k2_p = (v0 + k1_v * 0.5f) * dt;
    Vec3 k2_v = Vec3(a2) * dt;

    // k3
    linear_state_t tmp3 = state->linear;
    tmp3.velocity = v0 + k2_v * 0.5f;
    vec3_t a3;
    numeq_model_accel(&tmp3, &a_env, &a_body, &a3);
    Vec3 k3_p = (v0 + k2_v * 0.5f) * dt;
    Vec3 k3_v = Vec3(a3) * dt;

    // k4
    linear_state_t tmp4 = state->linear;
    tmp4.velocity = v0 + k3_v;
    vec3_t a4;
    numeq_model_accel(&tmp4, &a_env, &a_body, &a4);
    Vec3 k4_p = (v0 + k3_v) * dt;
    Vec3 k4_v = Vec3(a4) * dt;

    // 최종 업데이트
    Vec3 dp = (k1_p + (k2_p + k3_p) * 2.0f + k4_p) * (1.0f / 6.0f);
    Vec3 dv = (k1_v + (k2_v + k3_v) * 2.0f + k4_v) * (1.0f / 6.0f);
    state->linear.position = p0 + dp;
    state->linear.velocity = v0 + dv;
    state->linear.acceleration = a4;

    // -----------------------------
    // 회전 운동 (Angular Motion)
    // -----------------------------
    Vec3 w0(state->angular.angular_velocity);
    Vec3 alpha0(state->angular.angular_acceleration);

    // 각속도 RK4 (상수 alpha 가정)
    Vec3 k1_w = alpha0 * dt;
    Vec3 k2_w = alpha0 * dt;
    Vec3 k3_w = alpha0 * dt;
    Vec3 k4_w = alpha0 * dt;
    Vec3 delta_w = (k1_w + (k2_w + k3_w) * 2.0f + k4_w) * (1.0f / 6.0f);
    Vec3 w_new = w0 + delta_w;
    state->angular.angular_velocity = w_new;

    // 쿼터니언 회전 적용
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
            numeq_integrate_euler(state, config->time_step);
            break;
        case INTEGRATOR_SEMI_IMPLICIT:
            numeq_integrate_semi_implicit(state, config->time_step);
            break;
        case INTEGRATOR_RK4:
            numeq_integrate_rk4(state, config->time_step);
            break;
        case INTEGRATOR_RK4_ENV:
            numeq_integrate_rk4_env(
                state, config->time_step, config->env, config->body);
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
        case INTEGRATOR_MOTION_RK4_ENV:
            numeq_integrate_motion_rk4_env(
                state, config->time_step, config->env, config->body);
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
