#include "guidance.h"
#include "xform.h"
#include "vec3.h"
#include <math.h>

#include "environ.h"
#include "numeq_integrator.h"
#include "numeq_solver.h"
#include <float.h>

#include <stdio.h>


const vec3_t* guidance_none(
    const entity_dynamic_t* entdyn, float dt, void* userdata, vec3_t* out)
{
    static vec3_t s_zero = {0, 0, 0};
    vec3_t* result = out ? out : &s_zero;

    (void)entdyn; (void)dt; (void)userdata;
    vec3_zero(result);
    return result;
}

const vec3_t* guidance_point(
    const entity_dynamic_t* entdyn, float dt, void* userdata, vec3_t* out)
{
    static vec3_t s_dir;
    vec3_t* result = out ? out : &s_dir;

    (void)dt;
    if (!entdyn || !userdata) {
        vec3_zero(result);
        return result;
    }

    const vec3_t* target_pos = (const vec3_t*)userdata;
    vec3_t proj_pos;
    xform_get_position(&entdyn->xf, &proj_pos);

    vec3_sub(result, target_pos, &proj_pos);
    vec3_unit(result, result);
    return result;
}

const vec3_t* guidance_lead(
    const entity_dynamic_t* entdyn, float dt, void* userdata, vec3_t* out)
{
    int debug_mode;

    debug_mode = 0;

    static vec3_t s_dir;
    vec3_t* result = out ? out : &s_dir;
    (void)dt;

    if (!entdyn || !userdata) {
        vec3_zero(result);
        return result;
    }

    const entity_dynamic_t* target = (const entity_dynamic_t*)userdata;

    vec3_t missile_pos;
    xform_get_position(&entdyn->xf, &missile_pos);
    if (debug_mode) {
        printf("[DEBUG] missile_pos: "); vec3_print(&missile_pos);
    }

    vec3_t target_pos;
    xform_get_position(&target->xf, &target_pos);
    if (debug_mode) {
        printf("[DEBUG] target_pos: "); vec3_print(&target_pos);
    }

    float missile_speed = vec3_length(&entdyn->velocity);
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

        return vec3_length(&rel_pos) / missile_speed;
    }

    float t = (t1 > 0.0f) ? t1 : t2;
    if (t < 0.0f) t = vec3_length(&rel_pos) / missile_speed; // fallback
    return t;
}

const vec3_t* guidance_predict(
    const entity_dynamic_t* entdyn,
    float dt,
    void* userdata,
    vec3_t* out)
{
    static vec3_t s_dir;
    vec3_t* result = out ? out : &s_dir;

    if (!entdyn || !userdata) {
        vec3_zero(result);
        return result;
    }

    const guidance_target_info_t* info 
    = (const guidance_target_info_t*)userdata;

    entity_dynamic_t target = info->target;

    vec3_t missile_pos;
    xform_get_position(&entdyn->xf, &missile_pos);
    float missile_speed = vec3_length(&entdyn->velocity);
    if (missile_speed < 0.01f) missile_speed = 0.01f;

    vec3_t target_pos;
    xform_get_position(&target.xf, &target_pos);
    vec3_t target_vel = target.velocity;    

    float intercept_time = compute_intercept_time(&missile_pos, missile_speed,
                                                  &target_pos, &target_vel);

    vec3_t predicted_target;
    vec3_scale(&predicted_target, &target_vel, intercept_time);
    vec3_add(&predicted_target, &target_pos, &predicted_target);

    vec3_sub(result, &predicted_target, &missile_pos);
    vec3_unit(result, result);

    return result;
}

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

    // target pos(t) = p0 + v*t + 0.5*a*t^2
    // distance * distance = |p0 + v*t + 0.5*a*t^2|^2 - (missile_speed^2 * t^2) = 0

    float A = 0.25f * vec3_dot(&a, &a);
    float B = vec3_dot(&v, &a);
    float C = vec3_dot(&p0, &a) + vec3_dot(&v, &v) - missile_speed * missile_speed;
    float D = 2.0f * vec3_dot(&p0, &v);
    float E = vec3_dot(&p0, &p0);

    // A=0 quadratic
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

const vec3_t* guidance_predict_accel(
    const entity_dynamic_t* entdyn,
    float dt,
    void* userdata,
    vec3_t* out)
{
    static vec3_t s_dir;
    vec3_t* result = out ? out : &s_dir;

    if (!entdyn || !userdata) {
        vec3_zero(result);
        return result;
    }

    const guidance_target_info_t* info = (const guidance_target_info_t*)userdata;
    entity_dynamic_t target = info->target;

    vec3_t missile_pos;
    xform_get_position(&entdyn->xf, &missile_pos);
    float missile_speed = vec3_length(&entdyn->velocity);
    if (missile_speed < 0.01f) missile_speed = 0.01f;

    vec3_t target_pos;
    xform_get_position(&target.xf, &target_pos);
    vec3_t target_vel = target.velocity;

    vec3_t target_acc = {0, 0, 0};
    vec3_t prev_vel = target_vel;

    entity_dynamic_calc_accel_env(&target, 
        &prev_vel, dt, &info->env, &target_acc);

    float intercept_time = compute_intercept_time_accel(
        &missile_pos, missile_speed,
        &target_pos, &target_vel, &target_acc);

    // -------------------------------
    // predict target pos = p + v*t + 0.5*a*t^2
    // -------------------------------
    vec3_t predicted_target = target_pos;
    vec3_t term_v, term_a;
    vec3_scale(&term_v, &target_vel, intercept_time);
    vec3_scale(&term_a, &target_acc, 0.5f * intercept_time * intercept_time);
    vec3_add(&predicted_target, &predicted_target, &term_v);
    vec3_add(&predicted_target, &predicted_target, &term_a);

    vec3_sub(result, &predicted_target, &missile_pos);
    vec3_unit(result, result);

    return result;
}

const vec3_t* guidance_predict_accel_env(
    const entity_dynamic_t* entdyn,
    float dt,
    void* userdata,
    vec3_t* out)
{
    static vec3_t s_dir;
    vec3_t* result = out ? out : &s_dir;

    if (!entdyn || !userdata) {
        vec3_zero(result);
        return result;
    }

    const guidance_target_info_t* info = (const guidance_target_info_t*)userdata;
    entity_dynamic_t target = info->target;

    vec3_t missile_pos;
    xform_get_position(&entdyn->xf, &missile_pos);
    float missile_speed = vec3_length(&entdyn->velocity);
    if (missile_speed < 0.01f) missile_speed = 0.01f;

    vec3_t target_pos;
    xform_get_position(&target.xf, &target_pos);
    vec3_t target_vel = target.velocity;

    vec3_t target_acc = {0, 0, 0};
    vec3_t prev_vel = target_vel;
    entity_dynamic_calc_accel_env(&target, &prev_vel, dt, 
        &info->env, &target_acc);

    float intercept_time = compute_intercept_time_accel(&missile_pos,
                                                        missile_speed,
                                                        &target_pos,
                                                        &target_vel,
                                                        &target_acc);

    // --- predict target pos = p + v*t + 0.5*a*t^2 ---
    vec3_t predicted_target = target_pos;
    vec3_t term_v, term_a;
    vec3_scale(&term_v, &target_vel, intercept_time);
    vec3_scale(&term_a, &target_acc, 0.5f * intercept_time * intercept_time);
    vec3_add(&predicted_target, &predicted_target, &term_v);
    vec3_add(&predicted_target, &predicted_target, &term_a);

    vec3_sub(result, &predicted_target, &missile_pos);
    vec3_unit(result, result);
    return result;
}
