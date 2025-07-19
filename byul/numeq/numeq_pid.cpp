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

/**
 * @brief PID 기본값 초기화
 *
 * 이 함수는 PID 제어기를 다음 기본값으로 초기화합니다:
 * - Kp = 1.0f (비례 계수)
 * - Ki = 0.0f (적분 계수)
 * - Kd = 0.0f (미분 계수)
 * - dt = 0.01f (시간 간격, 100Hz 제어 기준)
 * - integral = 0.0f
 * - prev_error = 0.0f
 * - output_limit = 0.0f (제한 없음)
 * - anti_windup = false
 *
 * ### 권장 범위:
 * - **Kp:** 0.0 ~ 10.0 (너무 크면 발산, 너무 작으면 응답 느림)
 * - **Ki:** 0.0 ~ 1.0 (적분이 크면 오버슈트 증가 가능)
 * - **Kd:** 0.0 ~ 1.0 (노이즈 많은 환경에서는 낮게 설정)
 * - **dt:** 0.001 ~ 0.1초 (1ms~100ms)
 *
 * @param pid 초기화할 PID 구조체 포인터
 *
 * @note 이 초기값은 일반적인 제어 환경의 평균적인 값이며, 시스템 특성에 맞춰
 *       `pid_init_full()`로 조정하는 것을 권장합니다.
 */
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


void pid_init_full(pid_controller_t* pid, float kp, float ki, float kd, float dt) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->dt = dt;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->output_limit = 0.0f;
    pid->anti_windup = false;
}

#include <stddef.h>  // for NULL

/**
 * @brief PID 자동 튜닝 초기화
 *
 * 간단한 휴리스틱 방법(Ziegler–Nichols 기본값)을 사용하여
 * Kp, Ki, Kd를 대략적으로 추정합니다.
 *
 * @details
 * - Kp는 초기 비례 계수로 0.6을 사용.
 * - Ki는 Kp/(0.5*dt)를 사용 (Z-N 법칙 변형).
 * - Kd는 0.125*Kp*dt를 사용.
 * - integral, prev_error는 0으로 초기화됩니다.
 *
 * @param pid PID 구조체
 * @param dt 시간 간격 (초 단위)
 */
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
    pid_init_full(&pid->x, kp, ki, kd, dt);
    pid_init_full(&pid->y, kp, ki, kd, dt);
    pid_init_full(&pid->z, kp, ki, kd, dt);
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
