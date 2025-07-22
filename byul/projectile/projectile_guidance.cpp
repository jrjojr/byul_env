#include "internal/projectile_guidance.h"
#include "internal/xform.h"
#include "internal/vec3.h"
#include <math.h>

#include "internal/environ.h"
#include "internal/numeq_integrator.h"
#include "internal/numeq_solver.h"
#include <float.h>

#include <stdio.h>


// ---------------------------------------------------------
// 유도 없음 (None)
// ---------------------------------------------------------
const vec3_t* projectile_guidance_none(
    const projectile_t* proj, float dt, void* userdata, vec3_t* out)
{
    static vec3_t s_zero = {0, 0, 0};
    vec3_t* result = out ? out : &s_zero;

    (void)proj; (void)dt; (void)userdata;
    vec3_zero(result);
    return result;
}

// ---------------------------------------------------------
// 정적 타겟 유도 (Point)
// ---------------------------------------------------------
const vec3_t* projectile_guidance_point(
    const projectile_t* proj, float dt, void* userdata, vec3_t* out)
{
    static vec3_t s_dir;
    vec3_t* result = out ? out : &s_dir;

    (void)dt;
    if (!proj || !userdata) {
        vec3_zero(result);
        return result;
    }

    const vec3_t* target_pos = (const vec3_t*)userdata;
    vec3_t proj_pos;
    xform_get_position(&proj->base.xf, &proj_pos);

    vec3_sub(result, target_pos, &proj_pos);
    vec3_unit(result, result);
    return result;
}

const vec3_t* projectile_guidance_lead(
    const projectile_t* proj, float dt, void* userdata, vec3_t* out)
{
    int debug_mode;

    debug_mode = 0;

    static vec3_t s_dir;
    vec3_t* result = out ? out : &s_dir;
    (void)dt;

    if (!proj || !userdata) {
        vec3_zero(result);
        return result;
    }

    const entity_dynamic_t* target = (const entity_dynamic_t*)userdata;

    vec3_t missile_pos;
    xform_get_position(&proj->base.xf, &missile_pos);
    if (debug_mode) {
        printf("[DEBUG] missile_pos: "); vec3_print(&missile_pos);
    }

    vec3_t target_pos;
    xform_get_position(&target->xf, &target_pos);
    if (debug_mode) {
        printf("[DEBUG] target_pos: "); vec3_print(&target_pos);
    }

    float missile_speed = vec3_length(&proj->base.velocity);
    if (debug_mode) {
        printf("[DEBUG] missile_speed = %f\n", missile_speed);
    }

    if (missile_speed < 1e-5f) {
        vec3_sub(result, &target_pos, &missile_pos);
        if (vec3_length(result) < 1e-5f) {
            vec3_zero(result);
        } else {
            vec3_unit(result, result);
        }
        return result;
    }

    vec3_t to_target;
    vec3_sub(&to_target, &target_pos, &missile_pos);
    float distance = vec3_length(&to_target);
    if (debug_mode) {
        printf("[DEBUG] distance = %f\n", distance);
    }

    float lead_time = distance / missile_speed;
    if (debug_mode) {
        printf("[DEBUG] lead_time = %f\n", lead_time);
    }

    vec3_t future_offset, predicted_target;
    vec3_scale(&future_offset, &target->velocity, lead_time);
    vec3_add(&predicted_target, &target_pos, &future_offset);
    if (debug_mode) {
        printf("[DEBUG] predicted_target: "); vec3_print(&predicted_target);
    }

    vec3_sub(result, &predicted_target, &missile_pos);
    if (vec3_length(result) < 1e-5f) {
        vec3_zero(result);
    } else {
        vec3_unit(result, result);
    }

    if (debug_mode) {
        printf("[DEBUG] result (dir): "); vec3_print(result);
    }

    return result;
}

/**
 * @brief 요격 시간을 계산 (수식 기반)
 *
 * 미사일 위치, 속도, 타겟 위치 및 속도를 사용해
 * |(target_pos + target_vel * t) - (missile_pos + missile_dir * missile_speed * t)| = 0
 * 조건을 근사적으로 만족하는 t를 구합니다.
 * @note 해가 없으면 거리/속도 비율로 fallback 합니다.
 */
static float compute_intercept_time(
    const vec3_t* missile_pos,
    float missile_speed,
    const vec3_t* target_pos,
    const vec3_t* target_vel)
{
    vec3_t rel_pos;
    vec3_sub(&rel_pos, target_pos, missile_pos);

    float a = vec3_dot(target_vel, target_vel) - missile_speed * missile_speed;
    float b = 2.0f * vec3_dot(&rel_pos, target_vel);
    float c = vec3_dot(&rel_pos, &rel_pos);

    float t1 = 0.0f, t2 = 0.0f;
    if (!numeq_solve_quadratic(a, b, c, &t1, &t2)) {
        // fallback: 거리 / 속도
        return vec3_length(&rel_pos) / missile_speed;
    }

    float t = (t1 > 0.0f) ? t1 : t2;
    if (t < 0.0f) t = vec3_length(&rel_pos) / missile_speed; // fallback
    return t;
}

/**
 * @brief 발사체 유도 함수 (수식 기반 예측)
 *
 * 타겟의 motion_state_t(위치, 속도)를 기반으로 요격 위치를 계산하여
 * 발사체가 향해야 할 단위 방향 벡터를 반환합니다.
 */
const vec3_t* projectile_guidance_predict(
    const projectile_t* proj,
    float dt,
    void* userdata,
    vec3_t* out)
{
    static vec3_t s_dir;
    vec3_t* result = out ? out : &s_dir;

    if (!proj || !userdata) {
        vec3_zero(result);
        return result;
    }

    const target_info_t* info = (const target_info_t*)userdata;
    const entity_dynamic_t* target = info->target;  // 타겟 엔티티

    // 발사체 위치 및 속도 크기
    vec3_t missile_pos;
    xform_get_position(&proj->base.xf, &missile_pos);
    float missile_speed = vec3_length(&proj->base.velocity);
    if (missile_speed < 0.01f) missile_speed = 0.01f;

    // -------------------------------
    // 타겟 상태 (현재 위치, 속도)
    // -------------------------------
    vec3_t target_pos;
    xform_get_position(&target->xf, &target_pos);
    vec3_t target_vel = target->velocity;    

    // 요격 시간 계산
    float intercept_time = compute_intercept_time(&missile_pos, missile_speed,
                                                  &target_pos, &target_vel);

    // 미래 타겟 위치 = target_pos + target_vel * intercept_time
    vec3_t predicted_target;
    vec3_scale(&predicted_target, &target_vel, intercept_time);
    vec3_add(&predicted_target, &target_pos, &predicted_target);

    // 방향 벡터 계산
    vec3_sub(result, &predicted_target, &missile_pos);
    vec3_unit(result, result);

    return result;
}


/**
 * @brief 요격 시간 계산 (가속도 + 환경 영향, Cardano 해법)
 *
 * 미사일과 타겟의 초기 상대 위치 p0, 타겟 속도 v, 타겟 가속도를 고려해
 * |p0 + v*t + 0.5*a*t²|² = (missile_speed² * t²) 조건을 풀어
 * 양수 실근 중 가장 작은 값을 요격 시간으로 반환합니다.
 */
static float compute_intercept_time_accel(
    const vec3_t* missile_pos,
    float missile_speed,
    const vec3_t* target_pos,
    const vec3_t* target_vel,
    const vec3_t* target_acc)
{
    vec3_t p0, v, a;
    vec3_sub(&p0, target_pos, missile_pos);
    v = *target_vel;
    a = *target_acc;

    // 타겟 위치(t) = p0 + v*t + 0.5*a*t²
    // 거리 제곱 = |p0 + v*t + 0.5*a*t²|² - (missile_speed² * t²) = 0

    float A = 0.25f * vec3_dot(&a, &a);
    float B = vec3_dot(&v, &a);
    float C = vec3_dot(&p0, &a) + vec3_dot(&v, &v) - missile_speed * missile_speed;
    float D = 2.0f * vec3_dot(&p0, &v);
    float E = vec3_dot(&p0, &p0);

    // A=0이면 2차 방정식
    if (fabsf(A) < 1e-6f) {
        float x1, x2;
        if (!numeq_solve_quadratic(C, D, E, &x1, &x2))
            return vec3_length(&p0) / missile_speed;
        float t = (x1 > 0.0f) ? x1 : x2;
        return (t > 0.0f) ? t : vec3_length(&p0) / missile_speed;
    }

    float roots[3];
    int count = 0;
    if (!numeq_solve_cubic(A, B, C, D, roots, &count)) {
        return vec3_length(&p0) / missile_speed; // fallback
    }

    float t_best = FLT_MAX;
    for (int i = 0; i < count; ++i) {
        if (roots[i] > 0.0f && roots[i] < t_best)
            t_best = roots[i];
    }

    return (t_best == FLT_MAX) ? vec3_length(&p0) / missile_speed : t_best;
}

/**
 * @brief 발사체 유도 함수 (가속도 반영 예측, Cardano 기반)
 *
 * target_info_t는 entity_dynamic_t + environ_t 기반으로 동작합니다.
 */
const vec3_t* projectile_guidance_predict_accel(
    const projectile_t* proj,
    float dt,
    void* userdata,
    vec3_t* out)
{
    static vec3_t s_dir;
    vec3_t* result = out ? out : &s_dir;

    if (!proj || !userdata) {
        vec3_zero(result);
        return result;
    }

    const target_info_t* info = (const target_info_t*)userdata;
    const entity_dynamic_t* target = info->target;  // 타겟 엔티티

    // -------------------------------
    // 발사체 위치 및 속도
    // -------------------------------
    vec3_t missile_pos;
    xform_get_position(&proj->base.xf, &missile_pos);
    float missile_speed = vec3_length(&proj->base.velocity);
    if (missile_speed < 0.01f) missile_speed = 0.01f;

    // -------------------------------
    // 타겟 상태 (현재 위치, 속도)
    // -------------------------------
    vec3_t target_pos;
    xform_get_position(&target->xf, &target_pos);
    vec3_t target_vel = target->velocity;

    // -------------------------------
    // 타겟 가속도 (환경 포함)
    // -------------------------------
    vec3_t target_acc = {0, 0, 0};
    if (info->env) {
        entity_dynamic_predict_accel_env(target, info->env, &target_acc);
    }

    // -------------------------------
    // 요격 시간 계산
    // -------------------------------
    float intercept_time = compute_intercept_time_accel(
        &missile_pos, missile_speed,
        &target_pos, &target_vel, &target_acc);

    // -------------------------------
    // 미래 타겟 위치 = p + v*t + 0.5*a*t²
    // -------------------------------
    vec3_t predicted_target = target_pos;
    vec3_t term_v, term_a;
    vec3_scale(&term_v, &target_vel, intercept_time);
    vec3_scale(&term_a, &target_acc, 0.5f * intercept_time * intercept_time);
    vec3_add(&predicted_target, &predicted_target, &term_v);
    vec3_add(&predicted_target, &predicted_target, &term_a);

    // -------------------------------
    // 방향 벡터
    // -------------------------------
    vec3_sub(result, &predicted_target, &missile_pos);
    vec3_unit(result, result);

    return result;
}

/**
 * @brief 발사체 유도 함수 (가속도 + 환경 영향 + 엔티티 상태 기반)
 *
 * - entity_dynamic_t (target)에서 위치, 속도, 가속도 정보를 얻습니다.
 * - environ_t를 이용해 중력, 바람, 드래그 등의 외부 힘을 반영합니다.
 * - Cardano 기반 요격 시간 계산으로 미래 타겟 위치를 예측하고 단위 방향 벡터를 반환합니다.
 */
const vec3_t* projectile_guidance_predict_accel_env(
    const projectile_t* proj,
    float dt,
    void* userdata,
    vec3_t* out)
{
    static vec3_t s_dir;
    vec3_t* result = out ? out : &s_dir;

    if (!proj || !userdata) {
        vec3_zero(result);
        return result;
    }

    const target_info_t* info = (const target_info_t*)userdata;
    const entity_dynamic_t* target = info->target;

    // --- 발사체 현재 위치 및 속도 ---
    vec3_t missile_pos;
    xform_get_position(&proj->base.xf, &missile_pos);
    float missile_speed = vec3_length(&proj->base.velocity);
    if (missile_speed < 0.01f) missile_speed = 0.01f;

    // --- 타겟의 현재 상태 ---
    vec3_t target_pos;
    xform_get_position(&target->xf, &target_pos);
    vec3_t target_vel = target->velocity;

    // --- 타겟 가속도 추정 (환경 영향 포함) ---
    vec3_t target_acc = {0, 0, 0};
    if (info->env) {
        entity_dynamic_predict_accel_env(target, info->env, &target_acc);
    }

    // --- 요격 시간 계산 ---
    float intercept_time = compute_intercept_time_accel(&missile_pos,
                                                        missile_speed,
                                                        &target_pos,
                                                        &target_vel,
                                                        &target_acc);

    // --- 미래 타겟 위치 = p + v*t + 0.5*a*t² ---
    vec3_t predicted_target = target_pos;
    vec3_t term_v, term_a;
    vec3_scale(&term_v, &target_vel, intercept_time);
    vec3_scale(&term_a, &target_acc, 0.5f * intercept_time * intercept_time);
    vec3_add(&predicted_target, &predicted_target, &term_v);
    vec3_add(&predicted_target, &predicted_target, &term_a);

    // --- 최종 방향 벡터 계산 ---
    vec3_sub(result, &predicted_target, &missile_pos);
    vec3_unit(result, result);
    return result;
}
