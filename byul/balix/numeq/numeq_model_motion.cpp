
#include "numeq_model_motion.h"
#include "numeq_model.h"
#include "vec3.h"
#include "quat.h"
#include <math.h>

void numeq_model_motion_predict(
    float time,
    const motion_state_t* state0,
    const environ_t* env,
    const bodyprops_t* body,
    motion_state_t* out_state)
{
    if (!state0 || !env || !body || !out_state) return;

    numeq_model_predict(time, &state0->linear, env, body, &out_state->linear);

    vec3_t spin_accel = {0};
    calc_spin_accel(
        &spin_accel,
        &state0->linear.velocity,
        &state0->angular.angular_velocity,
        &state0->angular.angular_acceleration,
        time,
        body->k_magnus,
        body->k_gyro
    );

    float drag_scale = numeq_model_motion_drag_scale(state0, env);

    vec3_iscale(&out_state->linear.acceleration, drag_scale);

    vec3_iadd(&out_state->linear.acceleration, &spin_accel);

    attitude_state_assign(&out_state->angular, &state0->angular);
}

void numeq_model_motion_predict_rk4(
    float time,
    const motion_state_t* state0,
    const environ_t* env,
    const bodyprops_t* body,
    int steps,
    motion_state_t* out_state)
{
    if (!state0 || !env || !body || !out_state || steps <= 0) return;

    linear_state_t temp = state0->linear;

    vec3_t spin_accel = {0};
    calc_spin_accel(
        &spin_accel,
        &temp.velocity,
        &state0->angular.angular_velocity,
        &state0->angular.angular_acceleration,
        time,
        body->k_magnus,
        body->k_gyro
    );

    float drag_scale = numeq_model_motion_drag_scale(state0, env);

    vec3_iscale(&out_state->linear.acceleration, drag_scale);

    vec3_iadd(&temp.acceleration, &spin_accel);

    numeq_model_predict_rk4(time, &temp, env, body, steps, &out_state->linear);

    attitude_state_assign(&out_state->angular, &state0->angular);
}

float numeq_model_motion_drag_scale(
    const motion_state_t* state,
    const environ_t* env)
{
    if (!state || !env) return 1.0f;

    vec3_t rel_vel;
    vec3_sub(&rel_vel, &state->linear.velocity, &env->wind_vel);

    float v_sq = vec3_length_sq(&rel_vel);
    float w_sq = vec3_length_sq(&state->angular.angular_velocity);

    if (v_sq <= 1e-6f || w_sq <= 1e-6f)
        return 1.0f;

    vec3_t v_dir, spin_dir;
    vec3_unit(&v_dir, &rel_vel);
    vec3_unit(&spin_dir, &state->angular.angular_velocity);

    float alignment = vec3_dot(&v_dir, &spin_dir);  // -1.0 ~ 1.0
    float alignment_abs = fabsf(alignment);         // 0.0 ~ 1.0

    float spin_mag = sqrtf(w_sq);
    const float max_effective_spin = 50.0f;
    float spin_factor = spin_mag / max_effective_spin;
    if (spin_factor > 1.0f) spin_factor = 1.0f;

    // --------------------------
    // DRAG REDUCTION: when aligned
    // --------------------------
    const float max_reduction = 0.30f;  // up to 30% reduction
    float reduction = 0.0f;

    if (alignment > 0.0f) {
        reduction = max_reduction * alignment_abs * spin_factor;
    }

    // --------------------------
    // DRAG INCREASE: when misaligned
    // --------------------------
    const float max_penalty = 0.15f;  // up to 15% drag increase
    float penalty = 0.0f;

    if (alignment < 0.5f) {  // less than ~60 degrees aligned
        float misalign_factor = (0.5f - alignment) / 1.5f;  // 0~1 scale
        penalty = max_penalty * misalign_factor * spin_factor;
    }

    // --------------------------
    // Final scale = 1.0 - reduction + penalty
    // Clamp to [0.0, 2.0]
    // --------------------------
    float scale = 1.0f - reduction + penalty;

    if (scale < 0.0f) scale = 0.0f;
    if (scale > 2.0f) scale = 2.0f;

    return scale;
}

void numeq_model_motion_accel(
    const motion_state_t* state,
    const environ_t* env,
    const bodyprops_t* body,
    float time,
    vec3_t* out_accel)
{
    if (!state || !env || !body || !out_accel || time <= 0.0f)
        return;

    vec3_t base_accel = {0};
    numeq_model_accel(&state->linear, env, body, &base_accel);

    vec3_t spin_accel = {0};
    calc_spin_accel(
        &spin_accel,
        &state->linear.velocity,
        &state->angular.angular_velocity,
        &state->angular.angular_acceleration,
        time,
        body->k_magnus,
        body->k_gyro
    );

    vec3_add(out_accel, &base_accel, &spin_accel);
}

void calc_spin_accel(
    vec3_t* out_accel,
    const vec3_t* process_dir_speed_sec,
    const vec3_t* angular_velocity,
    const vec3_t* angular_accel,
    float time,
    float k_magnus,
    float k_gyro)
{
    if (!out_accel || !process_dir_speed_sec || !angular_velocity || 
        !angular_accel || time <= 0.0f) {
        if (out_accel) vec3_zero(out_accel);
        return;
    }

    // -------------------------------
    // 1. Magnus-like effect: a_magnus = k_magnus * (omega × v)
    // -------------------------------
    vec3_t magnus_accel;
    vec3_cross(&magnus_accel, angular_velocity, process_dir_speed_sec);
    vec3_iscale(&magnus_accel, k_magnus);

    // -------------------------------
    // 2. Angular acceleration induced effect: a_gyro = k_gyro * time * (alpha × v)
    // -------------------------------
    vec3_t gyro_accel;
    vec3_cross(&gyro_accel, angular_accel, process_dir_speed_sec);
    vec3_iscale(&gyro_accel, k_gyro * time);

    // -------------------------------
    // 3. Combine both: a_total = a_magnus + a_gyro
    // -------------------------------
    vec3_add(out_accel, &magnus_accel, &gyro_accel);
}
