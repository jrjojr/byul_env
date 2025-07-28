#include "numeq_pid.h"
#include "vec3.hpp"
#include <cmath>
#include <cstring>
#include <stddef.h>  // for NULL

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

void pid_init(pid_controller_t* pid) {
    if (!pid) return;
    pid->kp = 1.0f;
    pid->ki = 0.0f;
    pid->kd = 0.0f;
    pid->dt = 0.01f;          // 100Hz 제어 주기
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->output_limit = 0.0f; // 제한 없음
    pid->anti_windup = false;
}

void pid_init_full(pid_controller_t* pid, 
    float kp, float ki, float kd, float dt) {

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->dt = dt;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->output_limit = 0.0f;
    pid->anti_windup = false;
}

void pid_init_auto(pid_controller_t* pid, float dt) {
    if (!pid || dt <= 0.0f) return;

    // 안전 계수 기반 초기값
    float base_kp = 0.6f;
    float base_ki = base_kp / (0.5f * dt);
    float base_kd = 0.125f * base_kp * dt;

    pid->kp = base_kp;
    pid->ki = base_ki;
    pid->kd = base_kd;

    pid->dt = dt;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->output_limit = 0.0f; // 제한 없음
    pid->anti_windup = false;
}

void pid_assign(pid_controller_t* dst, const pid_controller_t* src) {
    std::memcpy(dst, src, sizeof(pid_controller_t));
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
