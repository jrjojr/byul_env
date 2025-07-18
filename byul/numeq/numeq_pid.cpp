#include "internal/numeq_pid.h"
#include "internal/vec3.hpp"
#include <cmath>
#include <cstring>

// ---------------------------------------------------------
// 내부 유틸: 출력 제한 및 windup 방지
// ---------------------------------------------------------
static float pid_clamp(float value, float limit) {
    if (limit <= 0.0f) return value;
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
}

// ---------------------------------------------------------
// 스칼라 PID
// ---------------------------------------------------------

void pid_init(pid_controller_t* pid, float kp, float ki, float kd, float dt) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->dt = dt;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->output_limit = 0.0f;
    pid->anti_windup = false;
}

void pid_set_state(pid_controller_t* pid, float integral, float prev_error) {
    pid->integral = integral;
    pid->prev_error = prev_error;
}

void pid_reset(pid_controller_t* pid) {
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}

float pid_update(pid_controller_t* pid, float target, float measured) {
    float error = target - measured;
    pid->integral += error * pid->dt;

    // 미분 항
    float derivative = (error - pid->prev_error) / pid->dt;

    // PID 합산
    float output = pid->kp * error +
                   pid->ki * pid->integral +
                   pid->kd * derivative;

    // 출력 제한 및 anti-windup 처리
    float limited = pid_clamp(output, pid->output_limit);

    if (pid->anti_windup && limited != output) {
        // 출력을 제한한 경우 → 적분 항 되돌리기
        pid->integral -= error * pid->dt;
    }

    pid->prev_error = error;
    return limited;
}

float pid_preview(const pid_controller_t* pid, float target, float measured) {
    float error = target - measured;
    float estimated_integral = pid->integral + error * pid->dt;
    float derivative = (error - pid->prev_error) / pid->dt;

    float output = pid->kp * error +
                   pid->ki * estimated_integral +
                   pid->kd * derivative;
    return pid_clamp(output, pid->output_limit);
}


void pid_copy(pid_controller_t* dst, const pid_controller_t* src) {
    std::memcpy(dst, src, sizeof(pid_controller_t));
}

// ---------------------------------------------------------
// 벡터 PID
// ---------------------------------------------------------

void pid_vec3_init(pid_controller_vec3_t* pid,
                   float kp, float ki, float kd,
                   float dt) {
    pid_init(&pid->x, kp, ki, kd, dt);
    pid_init(&pid->y, kp, ki, kd, dt);
    pid_init(&pid->z, kp, ki, kd, dt);
}

void pid_vec3_reset(pid_controller_vec3_t* pid) {
    pid_reset(&pid->x);
    pid_reset(&pid->y);
    pid_reset(&pid->z);
}

void pid_vec3_set_state(pid_controller_vec3_t* pid,
                        const vec3_t* integral,
                        const vec3_t* prev_error) {
    pid_set_state(&pid->x, integral->x, prev_error->x);
    pid_set_state(&pid->y, integral->y, prev_error->y);
    pid_set_state(&pid->z, integral->z, prev_error->z);
}

void pid_vec3_update(pid_controller_vec3_t* pid,
                     const vec3_t* target,
                     const vec3_t* measured,
                     vec3_t* out_control) {
    out_control->x = pid_update(&pid->x, target->x, measured->x);
    out_control->y = pid_update(&pid->y, target->y, measured->y);
    out_control->z = pid_update(&pid->z, target->z, measured->z);
}

void pid_vec3_preview(const pid_controller_vec3_t* pid,
                      const vec3_t* target,
                      const vec3_t* measured,
                      vec3_t* out_control) {
    out_control->x = pid_preview(&pid->x, target->x, measured->x);
    out_control->y = pid_preview(&pid->y, target->y, measured->y);
    out_control->z = pid_preview(&pid->z, target->z, measured->z);
}

void pid_vec3_copy(pid_controller_vec3_t* dst,
                   const pid_controller_vec3_t* src) {
    pid_copy(&dst->x, &src->x);
    pid_copy(&dst->y, &src->y);
    pid_copy(&dst->z, &src->z);
}
