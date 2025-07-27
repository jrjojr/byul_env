#include "internal/numeq_model.h"
#include "internal/vec3.hpp"
#include "internal/common.h"
#include "internal/numeq_integrator.h"
#include <cmath>
#include <functional>
#include <mutex>
#include "internal/numeq_solver.h"

// ---------------------------------------------------------
// drag_accel = -0.5 * ρ * v|v| * Cd * A / m
// ---------------------------------------------------------
static inline void compute_drag_accel(const vec3_t* velocity,
                                      const bodyprops_t* body,
                                      float air_density,
                                      vec3_t* out_drag_accel) 
{
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
// 항력 가속도 (즉시 계산)
// ---------------------------------------------------------
void numeq_model_drag_accel(const linear_state_t* state,
                            const environ_t* env,
                            const bodyprops_t* body,
                            vec3_t* out_drag_accel)
{
    if (!state || !body || !out_drag_accel) return;

    vec3_t rel_vel = state->velocity;
    if (env) vec3_sub(&rel_vel, &state->velocity, &env->wind);

    float air_density = env ? env->air_density : 1.225f;
    compute_drag_accel(&rel_vel, body, air_density, out_drag_accel);
}

// ---------------------------------------------------------
// 현재 또는 특정 velocity 기반 가속도 계산
// ---------------------------------------------------------


static inline void numeq_model_accel_internal(
    const vec3_t* vel,
    const environ_t* env,
    const bodyprops_t* body,
    vec3_t* out_accel)
{
    if (!vel || !out_accel) return;

    // 중력 (env가 있을 때만)
    *out_accel = env ? env->gravity : (vec3_t){0.0f, 0.0f, 0.0f};

    // 항력
    vec3_t drag_accel = {0, 0, 0};
    linear_state_t state;
    linear_state_init(&state);
    state.velocity = *vel;
    numeq_model_drag_accel(&state, env, body, &drag_accel);
    vec3_add(out_accel, out_accel, &drag_accel);

    // 중력 포함 외력들이 모두 보정되어서 최종 외력을 반환
    environ_adjust_accel(env, out_accel);
}

static inline void numeq_model_accel_except_gravity_internal(
    const vec3_t* vel,
    const environ_t* env,
    const bodyprops_t* body,
    vec3_t* out_accel)
{
    if (!vel || !out_accel) return;

    // 중력 (env가 있을 때만)
    *out_accel = env ? env->gravity : (vec3_t){0.0f, 0.0f, 0.0f};

    // 항력
    vec3_t drag_accel = {0, 0, 0};
    linear_state_t state;
    linear_state_init(&state);
    state.velocity = *vel;
    numeq_model_drag_accel(&state, env, body, &drag_accel);
    vec3_add(out_accel, out_accel, &drag_accel);

    // 중력 포함 외력들이 모두 보정되어서 최종 외력을 반환
    environ_adjust_accel_gsplit(env, true, out_accel);
}

// ---------------------------------------------------------
// 현재 시점 가속도 a = g + drag + environ + state 자체 가속도
// ---------------------------------------------------------
void numeq_model_accel(
    const linear_state_t* state,
    const environ_t* env,
    const bodyprops_t* body,
    vec3_t* out_accel)
{
    if (!state || !out_accel) return;

    // g + drag + environ
    numeq_model_accel_internal(&state->velocity, env, body, out_accel);

    // state의 자체 가속도 합산
    vec3_add(out_accel, out_accel, &state->acceleration);
}

void numeq_model_accel_except_gravity(
    const linear_state_t* state,
    const environ_t* env,
    const bodyprops_t* body,
    vec3_t* out_accel)
{
    if (!state || !out_accel) return;

    // 중력 제외: 내부 함수 호출 시 gravity 항은 0으로 강제
    numeq_model_accel_except_gravity_internal(&state->velocity, env, body, out_accel);

    // env->gravity 제거
    if (env) {
        vec3_t g = env->gravity;
        vec3_sub(out_accel, out_accel, &g);
    }

    // state의 자체 가속도 합산
    vec3_add(out_accel, out_accel, &state->acceleration);
}

// ---------------------------------------------------------
// t 시점 가속도 a(t) = g + drag + environ + state0.acceleration
// ---------------------------------------------------------
void numeq_model_accel_at(
    float t,
    const linear_state_t* state0,
    const environ_t* env,
    const bodyprops_t* body,
    vec3_t* out_accel)
{
    if (!state0 || !out_accel) return;

    // t=0일 때는 초기 가속도 사용
    if (t <= 0.0f) {
        numeq_model_accel(state0, env, body, out_accel);
        return;
    }

    // t초 후 속도 예측
    vec3_t vel;
    numeq_model_vel_at(t, state0, env, body, &vel);

    // 항력 및 환경 가속도
    numeq_model_accel_internal(&vel, env, body, out_accel);
}

// ---------------------------------------------------------
// 속도 v(t) = v0 + (a0 + state0.acceleration) * t
// ---------------------------------------------------------
void numeq_model_vel_at(
    float t,
    const linear_state_t* state0,
    const environ_t* env,
    const bodyprops_t* body,
    vec3_t* out_velocity) 
{
    if (!state0 || !out_velocity) return;

    vec3_t a0;
    numeq_model_accel(state0, env, body, &a0);

    vec3_t vel = state0->velocity;
    bodyprops_apply_friction(&vel, body, t);    // 마찰 적용

    Vec3 v0(vel);
    *out_velocity = v0 + Vec3(a0) * t;
}

// ---------------------------------------------------------
// 위치 p(t) = p0 + v0 * t + 0.5 * (a0 + state0.acceleration) * t²
// ---------------------------------------------------------
void numeq_model_pos_at(
    float t,
    const linear_state_t* state0,
    const environ_t* env,
    const bodyprops_t* body,
    vec3_t* out_position) 
{
    if (!state0 || !out_position) return;

    vec3_t a0;
    numeq_model_accel(state0, env, body, &a0);

    Vec3 p0(state0->position);
    Vec3 v0(state0->velocity);
    *out_position = p0 + v0 * t + Vec3(a0) * (0.5f * t * t);
}



// ---------------------------------------------------------
// 전체 상태 예측
// ---------------------------------------------------------
void numeq_model_calc(float t,
                         const linear_state_t* state0,
                         const environ_t* env,
                         const bodyprops_t* body,
                         linear_state_t* out_state) {
    numeq_model_pos_at(t, state0, env, body, &out_state->position);
    numeq_model_vel_at(t, state0, env, body, &out_state->velocity);
    numeq_model_accel_at(t, state0, env, body, &out_state->acceleration);
}

void numeq_model_calc_rk4(float t,
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
    integrator_config_init_full(&cfg, INTEGRATOR_RK4_ENV,
                                        t / (float)steps,  // dt,
                                        nullptr, // prev_state (Verlet용)
                                        env, 
                                        body, 
                                        nullptr);// userdata (옵션)
    // cfg.userdata = (void*)env;  // 환경 포인터 전달 가능 (필요 시)
    
    // 3. steps 횟수만큼 적분 수행
    for (int i = 0; i < steps; ++i) {
        numeq_integrate(&current, &cfg);
    }

    // 4. 최종 상태를 linear_state_t로 복사
    *out_state = current.linear;
}

bool numeq_model_bounce(const vec3_t* velocity_in,
                        const vec3_t* normal,
                        float restitution,
                        vec3_t* out_velocity_out)
{
    if (!velocity_in || !normal || !out_velocity_out)
        return false;

    // 반발계수 클램프 (0~1)
    float e = (restitution < 0.0f) ? 0.0f : (restitution > 1.0f) ? 1.0f : restitution;

    Vec3 v(*velocity_in);
    Vec3 n(*normal);
    float n_len = n.length();
    if (float_zero(n_len)) {
        *out_velocity_out = v;
        return false;
    }
    n /= n_len;

    // 속도를 법선/접선 성분으로 분리
    float vn = v.dot(n);
    Vec3 v_n = n * vn;
    Vec3 v_t = v - v_n;

    // 법선 성분만 반발계수 적용
    Vec3 v_new = v_t - v_n * e;
    *out_velocity_out = v_new;
    return true;
}

bool numeq_model_calc_collision(
    const linear_state_t* my_state,
    const linear_state_t* other_state,
    float radius_sum,
    float* out_time,
    vec3_t* out_point)
{
    if (!my_state || !other_state) return false;

    // 기본 초기화
    if (out_time) *out_time = -1.0f;
    if (out_point) vec3_zero(out_point);

    // 상대 위치/속도/가속도
    vec3_t p_rel, v_rel, a_rel;
    vec3_sub(&p_rel, &my_state->position, &other_state->position);
    vec3_sub(&v_rel, &my_state->velocity, &other_state->velocity);
    vec3_sub(&a_rel, &my_state->acceleration, &other_state->acceleration);

    // 초기 겹침 체크
    float initial_dist_sq = vec3_dot(&p_rel, &p_rel);
    if (initial_dist_sq <= radius_sum * radius_sum) {
        if (out_time) *out_time = 0.0f;
        if (out_point) {
            vec3_add(out_point, &my_state->position, &other_state->position);
            vec3_scale(out_point, out_point, 0.5f);
        }
        return true;
    }

    // 가속도 없는 경우 -> 2차 방정식
    if (vec3_is_zero(&a_rel)) {
        float A = vec3_dot(&v_rel, &v_rel);
        float B = 2.0f * vec3_dot(&p_rel, &v_rel);
        float C = initial_dist_sq - radius_sum * radius_sum;

        float x1, x2;
        if (!numeq_solve_quadratic(A, B, C, &x1, &x2))
            return false;

        // 미래 시점의 양수 해 선택
        float t = (x1 >= 0.0f) ? x1 : (x2 >= 0.0f ? x2 : -1.0f);
        if (t < 0.0f) return false;

        if (out_time) *out_time = t;
        if (out_point) {
            vec3_t pa, pb;
            vec3_scale(&pa, &my_state->velocity, t);
            vec3_scale(&pb, &other_state->velocity, t);
            vec3_add(&pa, &pa, &my_state->position);
            vec3_add(&pb, &pb, &other_state->position);
            vec3_add(out_point, &pa, &pb);
            vec3_scale(out_point, out_point, 0.5f);
        }
        return true;
    }

    // 가속도가 있는 경우 -> 4차 방정식 (추후 지원 예정)
    return false;
}
