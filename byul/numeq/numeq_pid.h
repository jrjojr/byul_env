#ifndef NUMEQ_PID_H
#define NUMEQ_PID_H

#include "internal/trajectory.h"

#ifdef __cplusplus
extern "C" {
#endif

// ---------------------------------------------------------
// PID 컨트롤러 (스칼라형)
// ---------------------------------------------------------

/**
 * @brief 단일 축 PID 제어기
 */
typedef struct s_pid_controller {
    float kp;             /**< 비례 계수 */
    float ki;             /**< 적분 계수 */
    float kd;             /**< 미분 계수 */

    float integral;       /**< 누적 오차 */
    float prev_error;     /**< 이전 오차 */

    float output_limit;   /**< 출력 제한 (0 이하이면 제한 없음) */
    float dt;             /**< 시간 간격 */
    bool anti_windup;     /**< 출력 saturate 시 적분 제한 여부 */
} pid_controller_t;

/**
 * @brief PID 초기화
 */
BYUL_API void pid_init(pid_controller_t* pid, 
    float kp, float ki, float kd, float dt);

/**
 * @brief PID 내부 상태 설정
 */
BYUL_API void pid_set_state(pid_controller_t* pid, 
    float integral, float prev_error);

/**
 * @brief PID 상태 초기화 (zero reset)
 */
BYUL_API void pid_reset(pid_controller_t* pid);

/**
 * @brief PID 제어값 계산
 */
BYUL_API float pid_update(pid_controller_t* pid, 
    float target, float measured);

/**
 * @brief PID 출력 예측 (상태 변화 없음)
 */
BYUL_API float pid_preview(const pid_controller_t* pid, 
    float target, float measured);

/**
 * @brief PID 상태 복사
 */
BYUL_API void pid_copy(pid_controller_t* dst, 
    const pid_controller_t* src);

// ---------------------------------------------------------
// PID 컨트롤러 (벡터형)
// ---------------------------------------------------------

/**
 * @brief 3축 벡터용 PID 제어기
 */
typedef struct s_pid_controller_vec3 {
    pid_controller_t x;
    pid_controller_t y;
    pid_controller_t z;
} pid_controller_vec3_t;

/**
 * @brief vec3 PID 초기화
 */
BYUL_API void pid_vec3_init(pid_controller_vec3_t* pid,
                   float kp, float ki, float kd,
                   float dt);

/**
 * @brief vec3 PID 상태 초기화
 */
BYUL_API void pid_vec3_reset(pid_controller_vec3_t* pid);

/**
 * @brief vec3 PID 상태 설정
 */
BYUL_API void pid_vec3_set_state(pid_controller_vec3_t* pid,
                        const vec3_t* integral,
                        const vec3_t* prev_error);

/**
 * @brief vec3 PID 계산
 */
BYUL_API void pid_vec3_update(pid_controller_vec3_t* pid,
                     const vec3_t* target,
                     const vec3_t* measured,
                     vec3_t* out_control);

/**
 * @brief vec3 PID 예측 (상태 변화 없음)
 */
BYUL_API void pid_vec3_preview(const pid_controller_vec3_t* pid,
                      const vec3_t* target,
                      const vec3_t* measured,
                      vec3_t* out_control);

/**
 * @brief vec3 PID 상태 복사
 */
BYUL_API void pid_vec3_copy(
    pid_controller_vec3_t* dst, const pid_controller_vec3_t* src);

#ifdef __cplusplus
}
#endif

#endif // NUMEQ_PID_H
