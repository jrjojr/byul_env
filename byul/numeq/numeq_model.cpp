#include "internal/numeq_model.h"
#include "internal/vec3.hpp"
#include <cmath>
#include <functional>
#include <mutex>

static std::mutex bounce_mutex;
static numeq_bounce_func g_bounce_func = nullptr;
static void* g_bounce_userdata = nullptr;

// ---------------------------------------------------------
// 공기 저항력 계산 (F_drag = 0.5 * ρ * v^2 * C_d * A) → a = F / m
// ---------------------------------------------------------
void numeq_model_drag_force(const vec3_t* velocity,
                            const body_properties_t* body,
                            float air_density,
                            vec3_t* out_drag_accel) {
    Vec3 v(*velocity);
    float v_mag = v.length();

    if (float_zero(v_mag)) {
        *out_drag_accel = Vec3(0, 0, 0);
        return;
    }

    Vec3 drag_dir = v * (-1.0f / v_mag); // 반대 방향 단위 벡터
    float drag_mag = 0.5f * air_density * v_mag * v_mag *
                     body->drag_coef * body->cross_section;
    float accel_mag = float_safe_div(drag_mag, body->mass, 0.0f);

    Vec3 result = drag_dir * accel_mag;
    *out_drag_accel = result;
}

// ---------------------------------------------------------
// 가속도 계산 (중력 + 바람 + 공기저항)
// ---------------------------------------------------------
void numeq_model_accel_at(float t,
                          const state_vector_t* state0,
                          const environment_t* env,
                          const body_properties_t* body,
                          vec3_t* out_accel) {
    Vec3 drag_accel;
    numeq_model_drag_force(&state0->velocity, body, env->air_density, &drag_accel.v);

    Vec3 gravity(env->gravity);
    Vec3 wind(env->wind);
    Vec3 result = gravity + wind + drag_accel;

    *out_accel = result;
}

// ---------------------------------------------------------
// 속도 계산 (v = v₀ + a * t)
// ---------------------------------------------------------
void numeq_model_vel_at(float t,
                        const state_vector_t* state0,
                        const environment_t* env,
                        const body_properties_t* body,
                        vec3_t* out_velocity) {
    Vec3 accel;
    numeq_model_accel_at(t, state0, env, body, &accel.v);
    Vec3 v0(state0->velocity);
    *out_velocity = v0 + accel * t;
}

// ---------------------------------------------------------
// 위치 계산 (p = p₀ + v₀·t + 0.5·a·t²)
// ---------------------------------------------------------
void numeq_model_pos_at(float t,
                        const state_vector_t* state0,
                        const environment_t* env,
                        const body_properties_t* body,
                        vec3_t* out_position) {
    Vec3 accel;
    numeq_model_accel_at(t, state0, env, body, &accel.v);

    Vec3 p0(state0->position);
    Vec3 v0(state0->velocity);

    Vec3 pos = p0 + v0 * t + accel * (0.5f * t * t);
    *out_position = pos;
}

// ---------------------------------------------------------
// 전체 상태 예측
// ---------------------------------------------------------
void numeq_model_predict(float t,
                         const state_vector_t* state0,
                         const environment_t* env,
                         const body_properties_t* body,
                         state_vector_t* out_state) {
    numeq_model_pos_at(t, state0, env, body, &out_state->position);
    numeq_model_vel_at(t, state0, env, body, &out_state->velocity);
    numeq_model_accel_at(t, state0, env, body, &out_state->acceleration);
}

// ---------------------------------------------------------
// 최고점 여부 판단: y 속도가 거의 0일 때
// ---------------------------------------------------------
bool numeq_model_is_apex(const state_vector_t* state) {
    return float_zero(state->velocity.y);
}

// ---------------------------------------------------------
// 착지 여부 판단: y 좌표가 지면보다 아래
// ---------------------------------------------------------
bool numeq_model_is_grounded(const state_vector_t* state, float ground_height) {
    return state->position.y <= ground_height;
}

// ---------------------------------------------------------
// 충돌 반발: 기본 내장 버전
// ---------------------------------------------------------
bool numeq_model_default_bounce(const vec3_t* velocity_in,
                                const vec3_t* normal,
                                float restitution,
                                vec3_t* out_velocity_out) {
    Vec3 v(*velocity_in);
    Vec3 n(*normal);

    Vec3 reflected = v - n * (2.0f * v.dot(n));
    *out_velocity_out = reflected * restitution;
    return true;
}

// ---------------------------------------------------------
// 외부 콜백 등록 / 조회
// ---------------------------------------------------------
void numeq_model_set_bounce_func(numeq_bounce_func func, void* userdata) {
    std::lock_guard<std::mutex> lock(bounce_mutex);
    g_bounce_func = func;
    g_bounce_userdata = userdata;
}

void numeq_model_get_bounce_func(numeq_bounce_func* out_func, void** out_userdata) {
    std::lock_guard<std::mutex> lock(bounce_mutex);
    if (out_func) *out_func = g_bounce_func;
    if (out_userdata) *out_userdata = g_bounce_userdata;
}
