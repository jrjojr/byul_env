#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <stdbool.h>
#include "internal/numeq.h"

#include "byul_config.h"

#ifdef __cplusplus
extern "C" {
#endif

// ---------------------------------------------------------
// 제어기 타입
// ---------------------------------------------------------
typedef enum e_controller_type {
    CONTROLLER_NONE = 0,   /**< 제어 없음 */
    CONTROLLER_PID,        /**< PID 제어 */
    CONTROLLER_BANGBANG,   /**< Bang-Bang 제어 */
    CONTROLLER_MPC         /**< Model Predictive Control */
} controller_type_t;

// ---------------------------------------------------------
// 제어기 공통 인터페이스
// ---------------------------------------------------------
typedef struct s_controller controller_t;

typedef float (*controller_compute_func)(
    controller_t* ctrl, float target, float measured, float dt);

typedef void (*controller_reset_func)(controller_t* ctrl);

// ---------------------------------------------------------
// 제어기 구조체
// ---------------------------------------------------------
struct s_controller {
    controller_type_t type;          /**< 제어기 종류 */
    void* impl;                      /**< 구체적 제어기 데이터 (PID, MPC 등) */
    controller_compute_func compute; /**< 제어 출력 계산 함수 */
    controller_reset_func reset;     /**< 내부 상태 초기화 */
    void* userdata;                  /**< 사용자 데이터 (옵션) */
};

// ---------------------------------------------------------
// 컨트롤러 생성 / 초기화
// ---------------------------------------------------------

/**
 * @brief PID 컨트롤러 생성
 */
BYUL_API controller_t* controller_create_pid(
    float kp, float ki, float kd, float dt, float output_limit);

/**
 * @brief Bang-Bang 컨트롤러 생성
 * @param max_output 최대 출력
 */
BYUL_API controller_t* controller_create_bangbang(float max_output);

/**
 * @brief MPC 컨트롤러 생성 (미래 예측 기반)
 */
BYUL_API controller_t* controller_create_mpc(
    const mpc_config_t*      config,
    const environment_t*     env,
    const body_properties_t* body);

/**
 * @brief 컨트롤러 해제
 */
BYUL_API void controller_destroy(controller_t* ctrl);

// ---------------------------------------------------------
// 제어기 공통 함수
// ---------------------------------------------------------

/**
 * @brief 제어기 출력 계산
 * @param ctrl 컨트롤러
 * @param target 목표 값
 * @param measured 현재 측정 값
 * @param dt 시간 간격
 * @return 제어 출력
 */
static inline float controller_compute(
    controller_t* ctrl, float target, float measured, float dt) {
    return (ctrl && ctrl->compute) ? ctrl->compute(ctrl, target, measured, dt) : 0.0f;
}

/**
 * @brief 컨트롤러 상태 리셋
 */
static inline void controller_reset(controller_t* ctrl) {
    if (ctrl && ctrl->reset) ctrl->reset(ctrl);
}

#ifdef __cplusplus
}
#endif

#endif // CONTROLLER_H
