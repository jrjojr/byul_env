#include "numeq_integrator.h"
#include "numeq_model.h"
#include "vec3.hpp"
#include <cassert>

void integrator_config_init(integrator_config_t* cfg) {
    if (!cfg) return;
    cfg->type = INTEGRATOR_RK4_ENV;
    cfg->time_step = 0.016f;
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

void numeq_integrate_semi_implicit(
    motion_state_t* state, float dt) {

    Vec3 a = Vec3(state->linear.acceleration);
    Vec3 v = Vec3(state->linear.velocity) + a * dt;
    Vec3 p = Vec3(state->linear.position) + v * dt;

    state->linear.velocity = v;
    state->linear.position = p;
    state->linear.acceleration = a;
}

void numeq_integrate_verlet(
    motion_state_t* state, const motion_state_t* prev_state, float dt) {
        
    assert(state && prev_state);

    Vec3 p(state->linear.position);
    Vec3 p_prev(prev_state->linear.position);
    Vec3 a(state->linear.acceleration);

    // new pos = 2p - p_prev + a * dt^2
    Vec3 new_pos = p * 2.0f - p_prev + a * (dt * dt);

    Vec3 vel = (new_pos - p_prev) * (1.0f / (2.0f * dt));

    state->linear.position = new_pos;
    state->linear.velocity = vel;
}

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
    
    environ_t a_env = env ? *env : environ_t{0};
    bodyprops_t a_body = body ? *body : bodyprops_t{0};

    Vec3 p0(state->linear.position);
    Vec3 v0(state->linear.velocity);

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

    Vec3 dp = (k1_p + (k2_p + k3_p) * 2.0f + k4_p) * (1.0f / 6.0f);
    Vec3 dv = (k1_v + (k2_v + k3_v) * 2.0f + k4_v) * (1.0f / 6.0f);

    state->linear.position = p0 + dp;
    state->linear.velocity = v0 + dv;
    state->linear.acceleration = a4;
}

void numeq_integrate_attitude_euler(motion_state_t* state, float dt) {
    if (!state) return;
    vec3_t* w = &state->angular.angular_velocity;
    vec3_t* a = &state->angular.angular_acceleration;

    // w = w + a * dt
    w->x += a->x * dt;
    w->y += a->y * dt;
    w->z += a->z * dt;

    quat_t dq;
    quat_init_angular_velocity(&dq, w, dt);
    quat_mul(&state->angular.orientation, &state->angular.orientation, &dq);
    quat_normalize(&state->angular.orientation);
}

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

void numeq_integrate_attitude_rk4(
    motion_state_t* state, float dt) {

    if (!state) return;
    vec3_t w0 = state->angular.angular_velocity;
    const vec3_t a0 = state->angular.angular_acceleration;

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

    vec3_t w0 = state->angular.angular_velocity;
    vec3_t alpha0 = state->angular.angular_acceleration;

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

    vec3_t k1 = alpha0;
    vec3_t k2 = alpha0;
    vec3_t k3 = alpha0;
    vec3_t k4 = alpha0;

    w0.x += (k1.x + 2*k2.x + 2*k3.x + k4.x) * (dt / 6.0f);
    w0.y += (k1.y + 2*k2.y + 2*k3.y + k4.y) * (dt / 6.0f);
    w0.z += (k1.z + 2*k2.z + 2*k3.z + k4.z) * (dt / 6.0f);

    state->angular.angular_velocity = w0;

    quat_t dq;
    quat_init_angular_velocity(&dq, &w0, dt);
    quat_mul(&state->angular.orientation, &state->angular.orientation, &dq);
    quat_normalize(&state->angular.orientation);
}

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

void numeq_integrate_motion_verlet(
    motion_state_t* state,
    motion_state_t* prev_state,
    float dt) {

    assert(state && prev_state);

    Vec3 p(state->linear.position);
    Vec3 p_prev(prev_state->linear.position);
    Vec3 a(state->linear.acceleration);

    Vec3 new_pos = p * 2.0f - p_prev + a * (dt * dt);

    *prev_state = *state;

    state->linear.position = new_pos;
    state->linear.velocity = (new_pos - p_prev) * (1.0f / (2.0f * dt));

    Vec3 w(state->angular.angular_velocity);
    Vec3 w_prev(prev_state->angular.angular_velocity);
    Vec3 ang_a(state->angular.angular_acceleration);

    Vec3 w_create = w * 2.0f - w_prev + ang_a * (dt * dt);

    state->angular.angular_velocity = w_create;

    quat_t dq;
    quat_init_angular_velocity(&dq, &w_create.v, dt);
    quat_mul(&state->angular.orientation, &state->angular.orientation, &dq);
    quat_normalize(&state->angular.orientation);
}

void numeq_integrate_motion_euler(motion_state_t* state, float dt) {
    assert(state);

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

void numeq_integrate_motion_semi_implicit(motion_state_t* state, float dt) {
    assert(state);

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

    if (!env || !body) {
        numeq_integrate_motion_rk4(state, dt);
        return;
    }

    environ_t a_env = env ? *env : environ_t{0};
    bodyprops_t a_body = body ? *body : bodyprops_t{0};

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

    Vec3 dp = (k1_p + (k2_p + k3_p) * 2.0f + k4_p) * (1.0f / 6.0f);
    Vec3 dv = (k1_v + (k2_v + k3_v) * 2.0f + k4_v) * (1.0f / 6.0f);
    state->linear.position = p0 + dp;
    state->linear.velocity = v0 + dv;
    state->linear.acceleration = a4;

    Vec3 w0(state->angular.angular_velocity);
    Vec3 alpha0(state->angular.angular_acceleration);

    Vec3 k1_w = alpha0 * dt;
    Vec3 k2_w = alpha0 * dt;
    Vec3 k3_w = alpha0 * dt;
    Vec3 k4_w = alpha0 * dt;
    Vec3 delta_w = (k1_w + (k2_w + k3_w) * 2.0f + k4_w) * (1.0f / 6.0f);
    Vec3 w_new = w0 + delta_w;
    state->angular.angular_velocity = w_new;

    quat_t dq;
    quat_init_angular_velocity(&dq, &w_new.v, dt);
    quat_mul(&state->angular.orientation, &state->angular.orientation, &dq);
    quat_normalize(&state->angular.orientation);
}

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
