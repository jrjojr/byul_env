#include "numeq_model.h"
#include "vec3.hpp"
#include "float_common.h"
#include "numeq_integrator.h"
#include <cmath>
#include <functional>
#include <mutex>
#include "numeq_solver.h"
#include "geom.h"

// ---------------------------------------------------------
// drag_accel = -0.5 * p * v|v| * Cd * A / m
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
    Vec3 drag_dir = v * (-1.0f / v_mag);
    float drag_mag = 0.5f * air_density * v_mag * v_mag *
                     body->drag_coef * body->cross_section;
    float accel_mag = float_safe_div(drag_mag, body->mass, 0.0f);
    *out_drag_accel = drag_dir * accel_mag;
}

void numeq_model_drag_accel(const linear_state_t* state,
                            const environ_t* env,
                            const bodyprops_t* body,
                            vec3_t* out_drag_accel)
{
    if (!state || !body || !out_drag_accel) return;

    vec3_t rel_vel = state->velocity;
    if (env) vec3_sub(&rel_vel, &state->velocity, &env->wind_vel);

    float air_density = env ? env->air_density : 1.225f;
    compute_drag_accel(&rel_vel, body, air_density, out_drag_accel);
}

static inline void numeq_model_accel_internal(
    const vec3_t* vel,
    const environ_t* env,
    const bodyprops_t* body,
    vec3_t* out_accel)
{
    if (!vel || !out_accel) return;

    *out_accel = env ? env->gravity : vec3_t{0.0f, 0.0f, 0.0f};

    vec3_t drag_accel = {0, 0, 0};
    linear_state_t state;
    linear_state_init(&state);
    state.velocity = *vel;
    numeq_model_drag_accel(&state, env, body, &drag_accel);
    vec3_add(out_accel, out_accel, &drag_accel);

    environ_distort_accel(env, out_accel);
}

static inline void numeq_model_accel_except_gravity_internal(
    const vec3_t* vel,
    const environ_t* env,
    const bodyprops_t* body,
    vec3_t* out_accel)
{
    if (!vel || !out_accel) return;

    *out_accel = env ? env->gravity : vec3_t{0.0f, 0.0f, 0.0f};

    vec3_t drag_accel = {0, 0, 0};
    linear_state_t state;
    linear_state_init(&state);
    state.velocity = *vel;
    numeq_model_drag_accel(&state, env, body, &drag_accel);
    vec3_add(out_accel, out_accel, &drag_accel);

    environ_distort_accel_except_gravity(env, true, out_accel);
}

void numeq_model_accel(
    const linear_state_t* state,
    const environ_t* env,
    const bodyprops_t* body,
    vec3_t* out_accel)
{
    if (!state || !out_accel) return;

    // g + drag + environ
    numeq_model_accel_internal(&state->velocity, env, body, out_accel);

    vec3_add(out_accel, out_accel, &state->acceleration);
}

void numeq_model_accel_except_gravity(
    const linear_state_t* state,
    const environ_t* env,
    const bodyprops_t* body,
    vec3_t* out_accel)
{
    if (!state || !out_accel) return;

    numeq_model_accel_except_gravity_internal(&state->velocity, env, body, out_accel);

    if (env) {
        vec3_t g = env->gravity;
        vec3_sub(out_accel, out_accel, &g);
    }

    vec3_add(out_accel, out_accel, &state->acceleration);
}

void numeq_model_accel_predict(
    float time,
    const linear_state_t* state0,
    const environ_t* env,
    const bodyprops_t* body,
    vec3_t* out_accel)
{
    if (!state0 || !out_accel) return;

    if (time <= 0.0f) {
        numeq_model_accel(state0, env, body, out_accel);
        return;
    }

    vec3_t vel;
    numeq_model_vel_predict(time, state0, env, body, &vel);

    numeq_model_accel_internal(&vel, env, body, out_accel);
}

// ---------------------------------------------------------
// velocity v(time) = v0 + (a0 + state0.acceleration) * time
// ---------------------------------------------------------
void numeq_model_vel_predict(
    float time,
    const linear_state_t* state0,
    const environ_t* env,
    const bodyprops_t* body,
    vec3_t* out_velocity) 
{
    if (!state0 || !out_velocity) return;

    vec3_t a0;
    numeq_model_accel(state0, env, body, &a0);

    vec3_t vel = state0->velocity;
    bodyprops_apply_friction(&vel, body, time);

    Vec3 v0(vel);
    *out_velocity = v0 + Vec3(a0) * time;
}

// ---------------------------------------------------------
// pos p(time) = p0 + v0 * time + 0.5 * (a0 + state0.acceleration) * t^2
// ---------------------------------------------------------
void numeq_model_pos_predict(
    float time,
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
    *out_position = p0 + v0 * time + Vec3(a0) * (0.5f * time * time);
}

void numeq_model_predict(float time,
                         const linear_state_t* state0,
                         const environ_t* env,
                         const bodyprops_t* body,
                         linear_state_t* out_state) {
    numeq_model_pos_predict(time, state0, env, body, &out_state->position);
    numeq_model_vel_predict(time, state0, env, body, &out_state->velocity);
    numeq_model_accel_predict(time, state0, env, body, &out_state->acceleration);
}

void numeq_model_predict_rk4(float time,
                             const linear_state_t* state0,
                             const environ_t* env,
                             const bodyprops_t* body,
                             int steps,
                             linear_state_t* out_state)
{
    if (!state0 || !out_state || steps <= 0 || time <= 0.0f) {
        if (out_state) {
            linear_state_assign(out_state, state0);
        }
        return;
    }

    motion_state_t current;
    motion_state_init(&current);
    current.linear = *state0;

    integrator_t intgr = {};
    integrator_init_full(&intgr, INTEGRATOR_RK4_ENV,
                                        &current,
                                        nullptr,
                                        env, 
                                        body);
    float dt = time / (float)steps;
    for (int i = 0; i < steps; ++i) {
        integrator_step(&intgr, dt);
    }

    current = intgr.state;
    *out_state = current.linear;
    integrator_free(&intgr);
}

bool numeq_model_bounce(const vec3_t* velocity_in,
                        const vec3_t* normal,
                        float restitution,
                        vec3_t* out_velocity_out)
{
    if (!velocity_in || !normal || !out_velocity_out)
        return false;

    float e = (restitution < 0.0f) ? 0.0f : (restitution > 1.0f) ? 1.0f : restitution;

    Vec3 v(*velocity_in);
    Vec3 n(*normal);
    float n_len = n.length();
    if (float_zero(n_len)) {
        *out_velocity_out = v;
        return false;
    }
    n /= n_len;

    float vn = v.dot(n);
    Vec3 v_n = n * vn;
    Vec3 v_t = v - v_n;

    Vec3 v_new = v_t - v_n * e;
    *out_velocity_out = v_new;
    return true;
}

bool numeq_model_predict_collision(
    const linear_state_t* my_state,
    const linear_state_t* other_state,
    float radius_sum,
    float* out_time,
    vec3_t* out_point)
{
    if (!my_state || !other_state) return false;

    if (out_time) *out_time = -1.0f;
    if (out_point) vec3_zero(out_point);

    vec3_t p_rel, v_rel, a_rel;
    vec3_sub(&p_rel, &my_state->position, &other_state->position);
    vec3_sub(&v_rel, &my_state->velocity, &other_state->velocity);
    vec3_sub(&a_rel, &my_state->acceleration, &other_state->acceleration);

    float initial_dist_sq = vec3_dot(&p_rel, &p_rel);
    if (initial_dist_sq <= radius_sum * radius_sum) {
        if (out_time) *out_time = 0.0f;
        if (out_point) {
            vec3_add(out_point, &my_state->position, &other_state->position);
            vec3_scale(out_point, out_point, 0.5f);
        }
        return true;
    }

    if (vec3_is_zero(&a_rel)) {
        float A = vec3_dot(&v_rel, &v_rel);
        float B = 2.0f * vec3_dot(&p_rel, &v_rel);
        float C = initial_dist_sq - radius_sum * radius_sum;

        float x1, x2;
        if (!numeq_solve_quadratic(A, B, C, &x1, &x2))
            return false;

        float time = (x1 >= 0.0f) ? x1 : (x2 >= 0.0f ? x2 : -1.0f);
        if (time < 0.0f) return false;

        if (out_time) *out_time = time;
        if (out_point) {
            vec3_t pa, pb;
            vec3_scale(&pa, &my_state->velocity, time);
            vec3_scale(&pb, &other_state->velocity, time);
            vec3_add(&pa, &pa, &my_state->position);
            vec3_add(&pb, &pb, &other_state->position);
            vec3_add(out_point, &pa, &pb);
            vec3_scale(out_point, out_point, 0.5f);
        }
        return true;
    }

    return false;
}

bool numeq_model_predict_collision_plane(
    const linear_state_t* my_state,
    const vec3_t* plane_point,
    const vec3_t* plane_normal,
    float radius_sum,
    float* out_time,
    vec3_t* out_point)
{
    if (!my_state || !plane_point || !plane_normal) return false;

    if (out_time) *out_time = -1.0f;
    if (out_point) vec3_zero(out_point);

    float dist = vec3_point_plane_distance(
        &my_state->position,
        plane_point,
        plane_normal);

    if (fabsf(dist) <= radius_sum)
    {
        if (out_time) *out_time = 0.0f;
        if (out_point) *out_point = my_state->position;
        return true;
    }

    if (vec3_is_zero(&my_state->velocity)) return false;

    float denom = vec3_dot(&my_state->velocity, plane_normal);
    if (fabsf(denom) < FLOAT_EPSILON) return false;

    vec3_t offset_plane_point = *plane_point;
    vec3_t scaled_normal;
    vec3_scale(&scaled_normal, plane_normal, -radius_sum);
    vec3_add(&offset_plane_point, &offset_plane_point, &scaled_normal);

    float time;
    vec3_t hit;
    bool intersect = vec3_ray_plane_intersect(
        &my_state->position,
        &my_state->velocity,
        &offset_plane_point,
        plane_normal,
        &time,
        &hit);

    if (!intersect || time < 0.0f) return false;

    if (out_time) *out_time = time;
    if (out_point) *out_point = hit;
    return true;
}
