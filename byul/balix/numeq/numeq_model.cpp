#include "internal/numeq_model.h"
#include "internal/vec3.hpp"
#include "internal/common.h"
#include "internal/numeq_integrator.h"
#include <cmath>
#include <functional>
#include <mutex>

static std::mutex bounce_mutex;
static numeq_bounce_func g_bounce_func = nullptr;
static void* g_bounce_userdata = nullptr;

// ---------------------------------------------------------
// 내부 유틸: drag_accel = -0.5 * ρ * v|v| * Cd * A / m
// ---------------------------------------------------------
static inline void compute_drag_accel(const vec3_t* velocity,
                                      const bodyprops_t* body,
                                      float air_density,
                                      vec3_t* out_drag_accel) {
    Vec3 v(*velocity);
    float v_mag = v.length();
    if (float_zero(v_mag)) {
        *out_drag_accel = {0, 0, 0};
        return;
    }
    Vec3 drag_dir = v * (-1.0f / v_mag); // 반대 방향
    float drag_mag = 0.5f * air_density * v_mag * v_mag *
                     body->drag_coef * body->cross_section;
    float accel_mag = float_safe_div(drag_mag, body->mass, 0.0f);
    *out_drag_accel = drag_dir * accel_mag;
}

// ---------------------------------------------------------
// 가속도 계산 (중력 + 항력 + 바람)
// ---------------------------------------------------------
void numeq_model_accel_at(float t,
                          const linear_state_t* state0,
                          const environ_t* env,
                          const bodyprops_t* body,
                          vec3_t* out_accel) {
    // t초 후 속도 예측
    vec3_t velocity_t;
    numeq_model_vel_at(t, state0, env, body, &velocity_t);

    vec3_t drag_accel;
    compute_drag_accel(&velocity_t, body, env->air_density, &drag_accel);

    // 바람은 속도와의 상대값으로 drag_accel 계산 시 고려 가능
    Vec3 result(env->gravity);
    result += Vec3(drag_accel);

    *out_accel = result;
}

// ---------------------------------------------------------
// 속도 계산 (선형 근사: v = v₀ + a(0) * t)
// ---------------------------------------------------------
void numeq_model_vel_at(float t,
                        const linear_state_t* state0,
                        const environ_t* env,
                        const bodyprops_t* body,
                        vec3_t* out_velocity) {
    // 초기 가속도 (a(0))
    vec3_t a0;
    compute_drag_accel(&state0->velocity, body, env->air_density, &a0);
    vec3_add(&a0, &a0, &env->gravity);

    Vec3 v0(state0->velocity);
    *out_velocity = v0 + Vec3(a0) * t;
}

// ---------------------------------------------------------
// 위치 계산 (p = p₀ + v₀ t + 0.5 a(0) t²)
// ---------------------------------------------------------
void numeq_model_pos_at(float t,
                        const linear_state_t* state0,
                        const environ_t* env,
                        const bodyprops_t* body,
                        vec3_t* out_position) {
    vec3_t a0;
    compute_drag_accel(&state0->velocity, body, env->air_density, &a0);
    vec3_add(&a0, &a0, &env->gravity);

    Vec3 p0(state0->position);
    Vec3 v0(state0->velocity);
    *out_position = p0 + v0 * t + Vec3(a0) * (0.5f * t * t);
}

// ---------------------------------------------------------
// 전체 상태 예측
// ---------------------------------------------------------
void numeq_model_predict(float t,
                         const linear_state_t* state0,
                         const environ_t* env,
                         const bodyprops_t* body,
                         linear_state_t* out_state) {
    numeq_model_pos_at(t, state0, env, body, &out_state->position);
    numeq_model_vel_at(t, state0, env, body, &out_state->velocity);
    numeq_model_accel_at(t, state0, env, body, &out_state->acceleration);
}

void numeq_model_predict_rk4(float t,
                             const linear_state_t* state0,
                             const environ_t* env,
                             const bodyprops_t* body,
                             int steps,
                             linear_state_t* out_state)
{
    if (!state0 || !out_state || steps <= 0 || t <= 0.0f) {
        if (out_state) {
            linear_state_assign(out_state, state0);
        }
        return;
    }

    // 1. 초기 상태를 motion_state_t로 변환 (회전은 무시)
    motion_state_t current;
    motion_state_init(&current);
    current.linear = *state0;

    // 2. 적분기 설정 (RK4)
    integrator_config_t cfg;
    integrator_config_init_full(&cfg, INTEGRATOR_MOTION_RK4_ENV,
                                        t / (float)steps,  // dt,
                                        nullptr, // prev_state (Verlet용)
                                        env, 
                                        body, 
                                        nullptr);// userdata (옵션)
    cfg.userdata = (void*)env;  // 환경 포인터 전달 가능 (필요 시)
    
    // 3. steps 횟수만큼 적분 수행
    for (int i = 0; i < steps; ++i) {
        numeq_integrate(&current, &cfg);
    }

    // 4. 최종 상태를 linear_state_t로 복사
    *out_state = current.linear;
}

// ---------------------------------------------------------
// 최고점 여부 (vy ≈ 0)
// ---------------------------------------------------------
bool numeq_model_is_apex(const linear_state_t* state) {
    return float_zero(state->velocity.y);
}

// ---------------------------------------------------------
// 착지 여부
// ---------------------------------------------------------
bool numeq_model_is_grounded(const linear_state_t* state,
                             float ground_height) {
    return state->position.y <= ground_height;
}

// ---------------------------------------------------------
// 기본 충돌 반발
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

void numeq_model_get_bounce_func(
    numeq_bounce_func* out_func, void** out_userdata) {
    std::lock_guard<std::mutex> lock(bounce_mutex);
    if (out_func) *out_func = g_bounce_func;
    if (out_userdata) *out_userdata = g_bounce_userdata;
}

bool numeq_model_predict_collision(
    const linear_state_t* my_state,
    const linear_state_t* other_state,
    const environ_t* env,
    const bodyprops_t* my_body,
    const bodyprops_t* other_body,
    float radius_sum,
    float max_time,
    float time_step,
    float* out_time,
    vec3_t* out_point)
{
    if (!my_state || !other_state || time_step <= 0.0f || max_time <= 0.0f)
        return false;

    vec3_t my_pos, other_pos;

    for (float t = 0.0f; t <= max_time; t += time_step) {
        numeq_model_pos_at(t, my_state, env, my_body, &my_pos);
        numeq_model_pos_at(t, other_state, env, other_body, &other_pos);

        float dist = vec3_distance(&my_pos, &other_pos);
        if (dist <= radius_sum) {
            if (out_time) *out_time = t;
            if (out_point) {
                vec3_add(out_point, &my_pos, &other_pos);
                vec3_scale(out_point, out_point, 0.5f);
            }
            return true;
        }
    }

    if (out_time) *out_time = -1.0f;
    if (out_point) vec3_zero(out_point);
    return false;
}
