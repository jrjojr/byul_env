#include "numeq_integrator.h"
#include "numeq_model.h"
#include "vec3.hpp"
#include <cassert>

void integrator_init(integrator_t* intgr) {
    if (!intgr) return;
    motion_state_t state;
    motion_state_init(&state);
    intgr->type = INTEGRATOR_RK4_ENV;
    intgr->state = state;    
    intgr->prev_state = {};
    intgr->env = {};
    intgr->body = {};
}

void integrator_init_full(integrator_t* intgr,
                                   const integrator_type_t type,
                                   const motion_state_t* state,
                                   const motion_state_t* prev_state,
                                   const environ_t* env,
                                   const bodyprops_t* body)
{
    if (!intgr) return;

    intgr->type = type;
    intgr->state = *state;    

    motion_state_assign(&intgr->prev_state, prev_state);
    environ_assign(&intgr->env, env);
    bodyprops_assign(&intgr->body, body);
}

void integrator_assign(
    integrator_t* out, const integrator_t* src) {

    if (!out || !src) return;
    *out = *src;
}

void integrator_clear(integrator_t* intgr){
    if(!intgr) return;

    intgr->prev_state = {};
    intgr->env = {};
    intgr->body = {};

    intgr->state = {};
    intgr->type = INTEGRATOR_RK4_ENV;
}

void integrator_free(integrator_t* intgr){
    integrator_clear(intgr);
}

void integrator_step_euler(
    motion_state_t* state, float dt) {

    Vec3 v(state->linear.velocity);
    Vec3 p(state->linear.position);
    Vec3 a(state->linear.acceleration);

    Vec3 v_next = v + a * dt;
    Vec3 p_next = p + v * dt;

    state->linear.velocity = v_next.v;
    state->linear.position = p_next.v;
    state->linear.acceleration = a.v;
}

// Euler with environment-dependent acceleration
void integrator_step_euler_env(
    motion_state_t* state,
    float dt,
    const environ_t* env,
    const bodyprops_t* body)
{
    if (!state) return;

    if (!env && !body) {
        integrator_step_euler(state, dt);
        return;
    }

    // a(t)
    vec3_t a0;
    numeq_model_accel_predict(0.0f, &state->linear, env, body, &a0);

    Vec3 v  = Vec3(state->linear.velocity);
    Vec3 p  = Vec3(state->linear.position);

    Vec3 v1 = v + Vec3(a0) * dt;
    Vec3 p1 = p + v * dt;

    state->linear.velocity     = v1;
    state->linear.position     = p1;
    state->linear.acceleration = a0; // bookkeeping
}


void integrator_step_semi_implicit(
    motion_state_t* state, float dt) {

    Vec3 a = Vec3(state->linear.acceleration);
    Vec3 v = Vec3(state->linear.velocity) + a * dt;
    Vec3 p = Vec3(state->linear.position) + v * dt;

    state->linear.velocity = v;
    state->linear.position = p;
    state->linear.acceleration = a;
}

// Semi-Implicit (Symplectic) Euler with environment-dependent acceleration
void integrator_step_semi_implicit_env(
    motion_state_t* state,
    float dt,
    const environ_t* env,
    const bodyprops_t* body)
{
    if (!state) return;

    if (!env && !body) {
        integrator_step_semi_implicit(state, dt);
        return;
    }

    // a(t)
    vec3_t a0;
    numeq_model_accel_predict(0.0f, &state->linear, env, body, &a0);

    Vec3 v0 = Vec3(state->linear.velocity);
    Vec3 p0 = Vec3(state->linear.position);

    // v(t+dt), then p(t+dt) with updated velocity
    Vec3 v1 = v0 + Vec3(a0) * dt;
    Vec3 p1 = p0 + v1 * dt;

    state->linear.velocity     = v1;
    state->linear.position     = p1;
    state->linear.acceleration = a0; // bookkeeping
}


void integrator_step_verlet(
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

// Optional: position-Verlet "_env" that mirrors your existing signature with prev_state.
// This variant is less robust when acceleration depends on velocity (drag).
void integrator_step_verlet_env(
    motion_state_t* state,
    const motion_state_t* prev_state,
    float dt,
    const environ_t* env,
    const bodyprops_t* body)
{
    if (!state || !prev_state) return;

    if (!env && !body) {
        integrator_step_verlet(state, prev_state, dt);
        return;
    }

    // a(t) using current state
    vec3_t a0;
    numeq_model_accel_predict(0.0f, &state->linear, env, body, &a0);

    Vec3 p  = Vec3(state->linear.position);
    Vec3 pp = Vec3(prev_state->linear.position);

    // new position
    Vec3 pn = p * 2.0f - pp + Vec3(a0) * (dt * dt);

    // velocity estimate
    Vec3 vn = (pn - pp) * (1.0f / (2.0f * dt));

    state->linear.position     = pn;
    state->linear.velocity     = vn;
    state->linear.acceleration = a0; // you may also re-evaluate at t+dt if needed
}

// Velocity Verlet with environment-dependent acceleration
// Requires previous state for classic position Verlet variant,
// but here we use the velocity-Verlet form which does not need prev_state.
void integrator_step_velocity_verlet_env(
    motion_state_t* state,
    float dt,
    const environ_t* env,
    const bodyprops_t* body)
{
    if (!state) return;

    if (!env && !body) {
        // Fallback: approximate with non-env Verlet if you have one,
        // or semi-implicit as a safe default.
        integrator_step_semi_implicit(state, dt);
        return;
    }

    // a(t)
    vec3_t a0;
    numeq_model_accel_predict(0.0f, &state->linear, env, body, &a0);

    Vec3 p0 = Vec3(state->linear.position);
    Vec3 v0 = Vec3(state->linear.velocity);

    // p(t+dt)
    Vec3 p1 = p0 + v0 * dt + Vec3(a0) * (0.5f * dt * dt);

    // Evaluate a(t+dt) at the new position (and predicted velocity if your model needs it)
    linear_state_t tmp = state->linear;
    tmp.position = p1.v;
    // If your acceleration depends on velocity (drag), you can predict v at half-step.
    // Here we pass v0 as-is; for stronger coupling, use v_half = v0 + 0.5*a0*dt.
    vec3_t a1;
    numeq_model_accel_predict(dt, &tmp, env, body, &a1);

    // v(t+dt)
    Vec3 v1 = v0 + (Vec3(a0) + Vec3(a1)) * (0.5f * dt);

    state->linear.position     = p1;
    state->linear.velocity     = v1;
    state->linear.acceleration = a1; // next-step hint
}

void integrator_step_rk4(
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

void integrator_step_rk4_env(motion_state_t* state,
                             float dt,
                             const environ_t* env,
                             const bodyprops_t* body)
{
    assert(state);

    if (!env && !body) {
        integrator_step_rk4(state, dt);
        return;
    }
    
    Vec3 p0(state->linear.position);
    Vec3 v0(state->linear.velocity);

    // k1
    vec3_t a1;
    numeq_model_accel_predict(0.0f, &state->linear, env, body, &a1);
    Vec3 k1_p = v0 * dt;
    Vec3 k1_v = Vec3(a1) * dt;

    // k2
    linear_state_t tmp2 = state->linear;
    tmp2.velocity = v0 + k1_v * 0.5f;

    vec3_t a2;
    numeq_model_accel_predict(dt * 0.5f, &tmp2, env, body, &a2);
    Vec3 k2_p = (v0 + k1_v * 0.5f) * dt;
    Vec3 k2_v = Vec3(a2) * dt;

    // k3
    linear_state_t tmp3 = state->linear;
    vec3_t v_half3 = {v0.v.x + 0.5f * k2_v.v.x,
                      v0.v.y + 0.5f * k2_v.v.y,
                      v0.v.z + 0.5f * k2_v.v.z};
    tmp3.velocity = v_half3;
    vec3_t a3;
    numeq_model_accel_predict(dt * 0.5f, &tmp3, env, body, &a3);
    Vec3 k3_p = (v0 + k2_v * 0.5f) * dt;
    Vec3 k3_v = Vec3(a3) * dt;

    // k4
    linear_state_t tmp4 = state->linear;
    vec3_t v_full = {v0.v.x + k3_v.v.x,
                     v0.v.y + k3_v.v.y,
                     v0.v.z + k3_v.v.z};
    tmp4.velocity = v_full;
    vec3_t a4;
    numeq_model_accel_predict(dt, &tmp4, env, body, &a4);
    Vec3 k4_p = (v0 + k3_v) * dt;
    Vec3 k4_v = Vec3(a4) * dt;

    Vec3 dp = (k1_p + (k2_p + k3_p) * 2.0f + k4_p) * (1.0f / 6.0f);
    Vec3 dv = (k1_v + (k2_v + k3_v) * 2.0f + k4_v) * (1.0f / 6.0f);

    state->linear.position = p0 + dp;
    state->linear.velocity = v0 + dv;
    state->linear.acceleration = a4;
}

void integrator_step_attitude_euler(motion_state_t* state, float dt) {
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

void integrator_step_attitude_euler_env(
    motion_state_t* state,
    float dt,
    const environ_t* env,
    const bodyprops_t* body)
{
    if (!state) return;
    if (!env || !body) {
        integrator_step_attitude_euler(state, dt);
        return;
    }

    vec3_t* w = &state->angular.angular_velocity;
    vec3_t a = state->angular.angular_acceleration;

    // Angular drag term
    float angular_drag_coeff = 0.5f * env->air_density 
        * body->drag_coef * body->cross_section / (body->mass + 1e-6f);

    a.x += -angular_drag_coeff * w->x;
    a.y += -angular_drag_coeff * w->y;
    a.z += -angular_drag_coeff * w->z;

    // w = w + a * dt
    w->x += a.x * dt;
    w->y += a.y * dt;
    w->z += a.z * dt;

    quat_t dq;
    quat_init_angular_velocity(&dq, w, dt);
    quat_mul(&state->angular.orientation, &state->angular.orientation, &dq);
    quat_normalize(&state->angular.orientation);

    state->angular.angular_acceleration = a;
}

void integrator_step_attitude_semi_implicit(
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

void integrator_step_attitude_semi_implicit_env(
    motion_state_t* state,
    float dt,
    const environ_t* env,
    const bodyprops_t* body)
{
    if (!state) return;
    if (!env || !body) {
        integrator_step_attitude_semi_implicit(state, dt);
        return;
    }

    vec3_t* w = &state->angular.angular_velocity;
    vec3_t a = state->angular.angular_acceleration;

    // Angular drag term
    float angular_drag_coeff = 0.5f * env->air_density 
        * body->drag_coef * body->cross_section / (body->mass + 1e-6f);

    a.x += -angular_drag_coeff * w->x;
    a.y += -angular_drag_coeff * w->y;
    a.z += -angular_drag_coeff * w->z;

    // Update angular velocity first (semi-implicit)
    w->x += a.x * dt;
    w->y += a.y * dt;
    w->z += a.z * dt;

    // Update orientation
    quat_t dq;
    quat_init_angular_velocity(&dq, w, dt);
    quat_mul(&state->angular.orientation, &state->angular.orientation, &dq);
    quat_normalize(&state->angular.orientation);

    state->angular.angular_acceleration = a;
}

void integrator_step_attitude_velocity_verlet(
    motion_state_t* state, float dt)
{
    if (!state) return;

    vec3_t w0 = state->angular.angular_velocity;
    vec3_t a0 = state->angular.angular_acceleration;

    // half-step angular velocity
    vec3_t w_half = {
        w0.x + 0.5f * a0.x * dt,
        w0.y + 0.5f * a0.y * dt,
        w0.z + 0.5f * a0.z * dt
    };

    // advance orientation with half-step omega
    quat_t dq;
    quat_init_angular_velocity(&dq, &w_half, dt);
    quat_mul(&state->angular.orientation, &state->angular.orientation, &dq);
    quat_normalize(&state->angular.orientation);

    // no re-evaluation in basic variant: alpha(t+dt) ~ alpha(t)
    vec3_t a1 = a0;

    // full-step angular velocity
    vec3_t w1 = {
        w_half.x + 0.5f * a1.x * dt,
        w_half.y + 0.5f * a1.y * dt,
        w_half.z + 0.5f * a1.z * dt
    };

    state->angular.angular_velocity     = w1;
    state->angular.angular_acceleration = a1;  // bookkeeping
}

void integrator_step_attitude_velocity_verlet_env(
    motion_state_t* state,
    float dt,
    const environ_t* env,
    const bodyprops_t* body)
{
    if (!state) return;

    // 1) alpha(t): start from current alpha and add simple angular drag if env/body exist
    vec3_t w0 = state->angular.angular_velocity;
    vec3_t a0 = state->angular.angular_acceleration;

    if (env && body) {
        // Example angular drag (replace with your torque model if available)
        float c = 0.5f * env->air_density
                * body->drag_coef * body->cross_section
                / (body->mass + 1e-6f);
        a0.x += -c * w0.x;
        a0.y += -c * w0.y;
        a0.z += -c * w0.z;
    }

    // 2) half-step omega
    vec3_t w_half = {
        w0.x + 0.5f * a0.x * dt,
        w0.y + 0.5f * a0.y * dt,
        w0.z + 0.5f * a0.z * dt
    };

    // 3) orientation update with w_half
    quat_t dq;
    quat_init_angular_velocity(&dq, &w_half, dt);
    quat_mul(&state->angular.orientation, &state->angular.orientation, &dq);
    quat_normalize(&state->angular.orientation);

    // 4) alpha(t+dt): re-evaluate if model depends on orientation or omega
    vec3_t a1 = a0;
    if (env && body) {
        float c = 0.5f * env->air_density
                * body->drag_coef * body->cross_section
                / (body->mass + 1e-6f);

        // If your alpha depends on q or w, build it here from updated state.
        // This demo keeps base alpha and adds drag from w_half.
        a1.x = state->angular.angular_acceleration.x + (-c * w_half.x);
        a1.y = state->angular.angular_acceleration.y + (-c * w_half.y);
        a1.z = state->angular.angular_acceleration.z + (-c * w_half.z);
    }

    // 5) full-step omega
    vec3_t w1 = {
        w_half.x + 0.5f * a1.x * dt,
        w_half.y + 0.5f * a1.y * dt,
        w_half.z + 0.5f * a1.z * dt
    };

    state->angular.angular_velocity     = w1;
    state->angular.angular_acceleration = a1;  // bookkeeping
}

void integrator_step_attitude_rk4(
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

// Helper: compute angular acceleration from base alpha + env/body drag (example).
// Replace or extend with your full torque model as needed.
static inline vec3_t attitude_alpha_eval(
    const vec3_t* base_alpha,  // state->angular.angular_acceleration
    const vec3_t* omega,       // current angular velocity
    const environ_t* env,
    const bodyprops_t* body)
{
    vec3_t a = *base_alpha;
    if (env && body) {
        float c = 0.5f * env->air_density
                * body->drag_coef * body->cross_section
                / (body->mass + 1e-6f);
        a.x += -c * omega->x;
        a.y += -c * omega->y;
        a.z += -c * omega->z;
    }
    return a;
}

void integrator_step_attitude_rk4_env(motion_state_t* state,
                                      float dt,
                                      const environ_t* env,
                                      const bodyprops_t* body)
{
    if (!state) return;

    // Base references
    const vec3_t w0 = state->angular.angular_velocity;
    const vec3_t a_base = state->angular.angular_acceleration;
    const quat_t q0 = state->angular.orientation;

    // RK4 stages for omega (angular velocity)
    // k1 = alpha(t, q0, w0)
    vec3_t k1 = attitude_alpha_eval(&a_base, &w0, env, body);

    // Predict half-step orientation with w0 (for torque models depending on q)
    quat_t q_half1 = q0;
    {
        quat_t dq;
        quat_init_angular_velocity(&dq, &w0, 0.5f * dt);
        quat_mul(&q_half1, &q_half1, &dq);
        quat_normalize(&q_half1);
    }
    // w at half-step using k1
    vec3_t w_half1 = { w0.x + 0.5f * k1.x * dt,
                       w0.y + 0.5f * k1.y * dt,
                       w0.z + 0.5f * k1.z * dt };
    // k2 = alpha(t+dt/2, q_half1, w_half1)
    vec3_t k2 = attitude_alpha_eval(&a_base, &w_half1, env, body);

    // Predict second half-step orientation with w_half1
    quat_t q_half2 = q0;
    {
        quat_t dq;
        quat_init_angular_velocity(&dq, &w_half1, 0.5f * dt);
        quat_mul(&q_half2, &q_half2, &dq);
        quat_normalize(&q_half2);
    }
    // w at half-step using k2
    vec3_t w_half2 = { w0.x + 0.5f * k2.x * dt,
                       w0.y + 0.5f * k2.y * dt,
                       w0.z + 0.5f * k2.z * dt };
    // k3 = alpha(t+dt/2, q_half2, w_half2)
    vec3_t k3 = attitude_alpha_eval(&a_base, &w_half2, env, body);

    // Predict full-step orientation using w_half2
    quat_t q_full = q0;
    {
        quat_t dq;
        quat_init_angular_velocity(&dq, &w_half2, dt);
        quat_mul(&q_full, &q_full, &dq);
        quat_normalize(&q_full);
    }
    // w at full-step using k3
    vec3_t w_full = { w0.x + k3.x * dt,
                      w0.y + k3.y * dt,
                      w0.z + k3.z * dt };
    // k4 = alpha(t+dt, q_full, w_full)
    vec3_t k4 = attitude_alpha_eval(&a_base, &w_full, env, body);

    // Omega update (RK4)
    vec3_t w1 = {
        w0.x + (k1.x + 2.0f*k2.x + 2.0f*k3.x + k4.x) * (dt / 6.0f),
        w0.y + (k1.y + 2.0f*k2.y + 2.0f*k3.y + k4.y) * (dt / 6.0f),
        w0.z + (k1.z + 2.0f*k2.z + 2.0f*k3.z + k4.z) * (dt / 6.0f)
    };

    // Orientation update
    // For better accuracy, use two half-steps with w0 and w1 (Strang-like splitting).
    {
        quat_t dq;
        quat_init_angular_velocity(&dq, &w0, 0.5f * dt);
        quat_mul(&state->angular.orientation, &state->angular.orientation, &dq);
        quat_init_angular_velocity(&dq, &w1, 0.5f * dt);
        quat_mul(&state->angular.orientation, &state->angular.orientation, &dq);
        quat_normalize(&state->angular.orientation);
    }

    state->angular.angular_velocity = w1;

    // Optional bookkeeping: a(t+dt) re-evaluated at final state (cheap estimate)
    // You can store this for the next step, or keep the previous a_base.
    vec3_t a1 = attitude_alpha_eval(&a_base, &w1, env, body);
    state->angular.angular_acceleration = a1;
}

// Explicit Euler motion step using Dual Quaternion pose integration.
// - Pose D = qr + eps qd
// - Ddot = 0.5 * Omega * D,  Omega = [0, w] + eps [0, v]  (world-frame)
// - Use v(t), w(t) for pose update (explicit), then update velocities with a, alpha.
void integrator_step_motion_euler(motion_state_t* state, float dt) {
    assert(state);

    // Read current state
    const vec3_t  p0   = state->linear.position;
    const vec3_t  v0   = state->linear.velocity;
    const vec3_t  a0   = state->linear.acceleration;

    const quat_t  q0   = state->angular.orientation;
    const vec3_t  w0   = state->angular.angular_velocity;
    const vec3_t  alpha0 = state->angular.angular_acceleration;

    // Build dual quaternion from pose
    dualquat_t D0;
    dualquat_init_quat_vec(&D0, &q0, &p0);

    // Build twist dual quaternion Omega = [0,w] + eps[0,v]
    dualquat_t Omega;
    Omega.real.w = 0.0f; 
    Omega.real.x = w0.x; 
    Omega.real.y = w0.y; 
    Omega.real.z = w0.z;

    Omega.dual.w = 0.0f; 
    Omega.dual.x = v0.x; 
    Omega.dual.y = v0.y; 
    Omega.dual.z = v0.z;

    // Ddot = 0.5 * Omega * D0
    dualquat_t Ddot;
    dualquat_mul(&Ddot, &Omega, &D0);
    dualquat_scale(&Ddot, &Ddot, 0.5f);

    // Explicit Euler pose update: D1 = D0 + Ddot * dt
    dualquat_t incr = Ddot;
    dualquat_scale(&incr, &incr, dt);
    dualquat_t D1;
    dualquat_add(&D1, &D0, &incr);

    // Normalize to unit dual quaternion
    dualquat_normalize(&D1);

    // Extract pose back to state
    quat_t q1; vec3_t p1;
    dualquat_to_quat_vec(&D1, &q1, &p1);
    state->angular.orientation = q1;
    quat_normalize(&state->angular.orientation); // keep unit rotation
    state->linear.position = p1;

    // Explicit Euler velocity updates (use accelerations at t)
    vec3_t v1 = { v0.x + a0.x * dt, v0.y + a0.y * dt, v0.z + a0.z * dt };
    vec3_t w1 = { w0.x + alpha0.x * dt, w0.y + alpha0.y * dt, w0.z + alpha0.z * dt };

    state->linear.velocity = v1;
    state->angular.angular_velocity = w1;
}

void integrator_step_motion_semi_implicit(motion_state_t* state, float dt) {
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

void integrator_step_motion_verlet(
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

void integrator_step_motion_rk4(motion_state_t* state, float dt) {
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

void integrator_step_motion_rk4_env(motion_state_t* state,
                                    float dt,
                                    const environ_t* env,
                                    const bodyprops_t* body)
{
    assert(state);

    if (!env || !body) {
        integrator_step_motion_rk4(state, dt);
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

void integrator_step(integrator_t* intgr, float dt) {

    switch (intgr->type) {
        case INTEGRATOR_EULER:
            integrator_step_euler(&intgr->state, dt);
            break;
        case INTEGRATOR_SEMI_IMPLICIT:
            integrator_step_semi_implicit(&intgr->state, dt);
            break;
        case INTEGRATOR_RK4:
            integrator_step_rk4(&intgr->state, dt);
            break;
        case INTEGRATOR_VERLET:
            integrator_step_verlet(&intgr->state, &intgr->prev_state, 
                    dt);
            break;

        case INTEGRATOR_EULER_ENV:
            integrator_step_euler_env(&intgr->state, dt, 
                &intgr->env, &intgr->body);
            break;
        case INTEGRATOR_SEMI_IMPLICIT_ENV:
            integrator_step_semi_implicit_env(&intgr->state, dt, 
                &intgr->env, &intgr->body);
            break;
        case INTEGRATOR_VERLET_ENV:
            integrator_step_verlet_env(&intgr->state, &intgr->prev_state, 
                    dt, &intgr->env, &intgr->body);
            break;            
        case INTEGRATOR_VELOCITY_VERLET_ENV:
            integrator_step_velocity_verlet_env(&intgr->state,
                    dt, &intgr->env, &intgr->body);
            break;                        
        case INTEGRATOR_RK4_ENV:
            integrator_step_rk4_env(
                &intgr->state, dt, &intgr->env, &intgr->body);
            break;                        

        case INTEGRATOR_MOTION_EULER:
            integrator_step_motion_euler(&intgr->state, dt);
            break;
        case INTEGRATOR_MOTION_SEMI_IMPLICIT:
            integrator_step_motion_semi_implicit(&intgr->state, dt);
            break;
        case INTEGRATOR_MOTION_VERLET:
            integrator_step_motion_verlet(&intgr->state, &intgr->prev_state, 
                dt);
            break;            
        case INTEGRATOR_MOTION_RK4:
            integrator_step_motion_rk4(&intgr->state, dt);
            break;

        case INTEGRATOR_MOTION_RK4_ENV:
            integrator_step_motion_rk4_env(
                &intgr->state, dt, &intgr->env, &intgr->body);
            break;

        default:
            assert(false && "Unknown integration type");
            break;
    }
}
