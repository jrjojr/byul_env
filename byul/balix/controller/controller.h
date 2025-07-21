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
    CONTROLLER_BANGBANG,   /**< Bang-Bang 제어 */
    CONTROLLER_PID,        /**< PID 제어 */
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

// PID 구현체
typedef struct s_pid_impl {
    pid_controller_t pid;
    float output_limit;
} pid_impl_t;

/**
 * @brief pid_impl_t 기본 초기화
 *
 * PID 구현체(`pid_impl_t`)를 기본값으로 초기화합니다.
 *
 * @param impl 초기화할 pid_impl_t 구조체 포인터
 *
 * @note 내부적으로 `pid_init()`를 호출하여 PID 컨트롤러가 초기화됩니다.
 */
BYUL_API void pid_impl_init(pid_impl_t* impl);

/**
 * @brief pid_impl_t를 지정값으로 초기화
 *
 * 사용자가 지정한 PID 파라미터(Kp, Ki, Kd, dt)와
 * 출력 제한값(output_limit)을 기반으로 `pid_impl_t`를 초기화합니다.
 *
 * @param impl 초기화할 pid_impl_t 구조체 포인터
 * @param kp 비례 계수 (권장 범위: 0.0 ~ 10.0)
 * @param ki 적분 계수 (권장 범위: 0.0 ~ 1.0)
 * @param kd 미분 계수 (권장 범위: 0.0 ~ 1.0)
 * @param dt 제어 주기(초) (권장 범위: 0.001 ~ 0.1)
 * @param output_limit 출력 제한값 (0 이하이면 제한 없음)
 *
 * @note 내부적으로 `pid_init_full()`을 호출하여 PID 컨트롤러를 세팅합니다.
 */
BYUL_API void pid_impl_init_full(pid_impl_t* impl,
                                 float kp, float ki, float kd,
                                 float dt, float output_limit);

/**
 * @brief pid_impl_t 상태 복사
 *
 * 원본 `src`의 PID 파라미터와 내부 상태를
 * 대상 `dst`로 깊은 복사합니다.
 *
 * @param dst 복사 대상 pid_impl_t
 * @param src 원본 pid_impl_t
 *
 * @note `dst`와 `src`가 모두 유효한 포인터여야 합니다.
 */
BYUL_API void pid_impl_assign(pid_impl_t* dst, const pid_impl_t* src);

// Bang-Bang 구현체
typedef struct s_bangbang_impl {
    float max_output;
} bangbang_impl_t;

/**
 * @brief bangbang_impl_t 기본 초기화
 *
 * Bang-Bang 제어기를 기본값으로 초기화합니다.
 * - max_output = 1.0f
 *
 * @param impl 초기화할 bangbang_impl_t 구조체 포인터
 *
 * @note Bang-Bang 제어는 목표값보다 작으면 +max_output,
 *       크면 -max_output을 출력하는 단순 제어입니다.
 */
BYUL_API void bangbang_impl_init(bangbang_impl_t* impl);

/**
 * @brief bangbang_impl_t를 지정값으로 초기화
 *
 * 사용자가 지정한 최대 출력값(max_output)을
 * Bang-Bang 제어기 구조체에 설정합니다.
 *
 * @param impl 초기화할 bangbang_impl_t 구조체 포인터
 * @param max_output 제어기의 최대 출력값 (권장 범위: 0.0 이상)
 *
 * @note max_output은 절댓값 기준으로 제어기의 양방향 최대 출력으로 사용됩니다.
 */
BYUL_API void bangbang_impl_init_full(bangbang_impl_t* impl, float max_output);

/**
 * @brief bangbang_impl_t 상태 복사
 *
 * 원본 `src`의 max_output 값을 대상 `dst`로 복사합니다.
 *
 * @param dst 복사 대상 bangbang_impl_t
 * @param src 원본 bangbang_impl_t
 *
 * @note `dst`와 `src`는 모두 유효한 포인터여야 합니다.
 */
BYUL_API void bangbang_impl_assign(bangbang_impl_t* dst,
                                 const bangbang_impl_t* src);

// MPC 구현체
typedef struct s_mpc_impl {
    mpc_config_t config;   // MPC 설정
    motion_state_t target;         // 목표 속도/위치 (x방향 기준)
    environ_t env;     // 환경 정보 (필요 시 세팅)
    bodyprops_t body;// 물리 속성 (질량 등)
    mpc_cost_func cost_fn;
} mpc_impl_t;

/**
 * @brief mpc_impl_t 기본 초기화
 *
 * MPC(Model Predictive Control) 제어기를 기본값으로 초기화합니다.
 * - `mpc_config_t`는 `mpc_config_init()`로 초기화됩니다.
 * - `motion_state_t`는 `motion_state_init()`으로 초기화됩니다.
 * - `environ_t`는 `environment_init()`으로 초기화됩니다.
 * - `bodyprops_t`는 `body_properties_init()`으로 초기화됩니다.
 * - `cost_fn`은 numeq_mpc_cost_default 로 설정됩니다.
 *
 * @param impl 초기화할 mpc_impl_t 구조체 포인터
 *
 * @note 이 함수는 모든 내부 구조체를 기본값으로 초기화합니다.
 */
BYUL_API void mpc_impl_init(mpc_impl_t* impl);

/**
 * @brief mpc_impl_t를 지정값으로 초기화
 *
 * 사용자가 지정한 MPC 설정(config), 목표 상태(target),
 * 환경(env), 물체 속성(body), 비용 함수(cost_fn)를 복사하여
 * mpc_impl_t를 초기화합니다.
 *
 * @param impl 초기화할 mpc_impl_t 구조체 포인터
 * @param cfg   MPC 설정 (NULL이면 기본값으로 초기화)
 * @param target 목표 운동 상태 (NULL이면 기본값)
 * @param env 환경 정보 (NULL이면 기본값)
 * @param body 물체 속성 (NULL이면 기본값)
 * @param cost_fn 비용 함수 포인터 NULL 시 numeq_mpc_cost_default 가 설정
 *
 * @note 각 파라미터가 NULL이면 해당 항목은 기본 초기화 함수로 설정됩니다.
 */
BYUL_API void mpc_impl_init_full(mpc_impl_t* impl,
                                 const mpc_config_t* cfg,
                                 const motion_state_t* target,
                                 const environ_t* env,
                                 const bodyprops_t* body,
                                 mpc_cost_func cost_fn);

/**
 * @brief mpc_impl_t 상태 복사
 *
 * 원본 `src`의 MPC 설정, 목표 상태, 환경, 물체 속성, 비용 함수 포인터를
 * 대상 `dst`로 깊은 복사합니다.
 *
 * @param dst 복사 대상 mpc_impl_t
 * @param src 원본 mpc_impl_t
 *
 * @note `dst`와 `src`가 유효한 포인터여야 하며,
 *       내부 구조체(mpc_config_t, motion_state_t 등)는 값 복사됩니다.
 */
BYUL_API void mpc_impl_assign(mpc_impl_t* dst, const mpc_impl_t* src);

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
    const environ_t*     env,
    const bodyprops_t* body);

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
    return (ctrl && ctrl->compute) ? 
        ctrl->compute(ctrl, target, measured, dt) : 0.0f;
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
