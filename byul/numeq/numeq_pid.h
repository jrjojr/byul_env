/**
 * @file numeq_pid.h
 * @brief PID (Proportional-Integral-Derivative) 제어기 모듈
 *
 * 이 모듈은 스칼라 및 3축 벡터용 PID 제어기를 제공합니다.
 * PID 제어는 목표값(target)과 측정값(measured)의 오차(error)를 기반으로
 * 제어 출력(control)을 계산하여 시스템을 안정적으로 원하는 상태로 제어하는 데 사용됩니다.
 *
 * ---
 *
 * ## PID 원리
 *
 * PID 제어는 다음 세 가지 항목을 기반으로 출력값을 계산합니다:
 *
 * - **비례 (P):** 현재 오차에 비례하여 출력.  
 *   P = Kp * e(t)
 *
 * - **적분 (I):** 과거 오차의 누적값을 사용하여 출력.  
 *   I = Ki * ∫ e(t) dt
 *
 * - **미분 (D):** 오차의 변화율(속도)을 사용하여 출력.  
 *   D = Kd * de(t)/dt
 *
 * **총 출력:**  
 *   u(t) = P + I + D
 *
 * ---
 *
 * ## 대표적인 사용 예
 *
 * ### 스칼라 PID
 * @code
 * pid_controller_t pid;
 *
 * // 1. 기본값 초기화
 * pid_init(&pid);
 *
 * // 2. 사용자 지정값으로 초기화
 * pid_init_full(&pid, 1.0f, 0.1f, 0.05f, 0.01f); // Kp, Ki, Kd, dt
 *
 * // 3. 상태 설정 (적분항, 이전 오차)
 * pid_set_state(&pid, 0.0f, 0.0f);
 *
 * // 4. 메인 루프에서 PID 제어
 * while (running) {
 *     float control = pid_update(&pid, target_value, current_value);
 *     // control 값을 시스템 구동에 사용
 * }
 *
 * // 5. 상태 리셋
 * pid_reset(&pid);
 *
 * // 6. 상태 복사
 * pid_controller_t pid_assign_target;
 * pid_assign(&pid_assign_target, &pid);
 *
 * // 7. 상태 변화 없는 출력 예측
 * float preview_control = pid_preview(&pid, target_value, current_value);
 * @endcode
 *
 * ### 벡터 PID
 * @code
 * pid_controller_vec3_t pid3;
 * vec3_t target = {1, 0, 0};
 * vec3_t measured = {0, 0, 0};
 * vec3_t output;
 *
 * // 1. 초기화
 * pid_vec3_init(&pid3, 1.0f, 0.1f, 0.05f, 0.01f);
 *
 * // 2. 상태 설정
 * vec3_t integral = {0, 0, 0};
 * vec3_t prev_err = {0, 0, 0};
 * pid_vec3_set_state(&pid3, &integral, &prev_err);
 *
 * // 3. PID 제어 계산
 * pid_vec3_update(&pid3, &target, &measured, &output);
 *
 * // 4. 상태 리셋
 * pid_vec3_reset(&pid3);
 *
 * // 5. 상태 복사
 * pid_controller_vec3_t pid3_assign;
 * pid_vec3_assign(&pid3_assign, &pid3);
 *
 * // 6. 상태 변화 없는 출력 예측
 * pid_vec3_preview(&pid3, &target, &measured, &output);
 * @endcode
 *
 * ---
 *
 * ## 주요 특징
 * - anti_windup 옵션 지원 (적분항 과도 누적 방지)
 * - 출력 제한 (output_limit) 설정 가능
 * - preview 함수 제공 (상태 변화 없이 PID 출력 예측)
 */
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
 * @brief PID 제어기를 지정한 값으로 초기화
 *
 * 사용자가 Kp, Ki, Kd, dt를 직접 지정하여 PID 컨트롤러를 설정합니다.
 * integral(적분항)과 prev_error(이전 오차)는 0으로 초기화됩니다.
 *
 * ### 파라미터 설명 및 권장 범위:
 * - **kp (비례 계수):** 0.0 ~ 10.0  
 *   - 오차에 비례한 즉각적인 제어 출력.  
 *   - 값이 클수록 반응 속도가 빠르지만 진동(오버슈트)이 커질 수 있음.
 *
 * - **ki (적분 계수):** 0.0 ~ 1.0  
 *   - 과거 오차의 누적을 보상해 목표값에 도달하도록 도움.  
 *   - 너무 크면 과도한 적분 누적(Integral Windup)으로 불안정할 수 있음.
 *
 * - **kd (미분 계수):** 0.0 ~ 1.0  
 *   - 오차의 변화율을 기반으로 과도 진동을 줄이고 안정화에 도움.  
 *   - 노이즈가 많을 경우 낮은 값 권장.
 *
 * - **dt (시간 간격):** 0.001 ~ 0.1초 (1ms~100ms)  
 *   - 제어 루프 주기. 예: 100Hz 업데이트 시 0.01초.
 *
 * ### 기본 초기화:
 * - integral = 0.0f
 * - prev_error = 0.0f
 * - output_limit = 0.0f (제한 없음)
 * - anti_windup = false
 *
 * @param pid 초기화할 PID 구조체
 * @param kp 비례 계수
 * @param ki 적분 계수
 * @param kd 미분 계수
 * @param dt 시간 간격 (제어 루프 주기)
 *
 * @note 시스템 특성에 맞게 Kp, Ki, Kd를 조정해야 안정적 제어가 가능합니다.
 */
BYUL_API void pid_init_full(pid_controller_t* pid, 
    float kp, float ki, float kd, float dt);

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
BYUL_API void pid_init(pid_controller_t* pid);

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
BYUL_API void pid_init_auto(pid_controller_t* pid, float dt);

/**
 * @brief PID 상태 복사
 */
BYUL_API void pid_assign(pid_controller_t* dst, 
    const pid_controller_t* src);

/**
 * @brief PID 상태 초기화 (zero reset)
 */
BYUL_API void pid_reset(pid_controller_t* pid);    

/**
 * @brief PID 내부 상태 설정
 *
 * PID 컨트롤러의 적분항(`integral`)과 이전 오차(`prev_error`)를
 * 외부에서 직접 지정합니다.
 * 
 * **사용 예시:**
 * - 시뮬레이션 재시작 시 이전 상태를 복원.
 * - 특정 상황에서 적분항을 강제로 리셋하거나 원하는 값으로 조정.
 *
 * @param pid PID 컨트롤러 구조체 포인터
 * @param integral 적분항 초기값
 * @param prev_error 이전 오차값 (D항 계산 시 기준값)
 *
 * @note 보통은 `pid_reset()`으로 integral과 prev_error를 0으로 초기화하는 것이 기본입니다.
 */
BYUL_API void pid_set_state(pid_controller_t* pid, 
    float integral, float prev_error);

/**
 * @brief PID 제어값 계산
 *
 * 목표값(target)과 현재 측정값(measured)의 차이(오차)를 계산하여
 * PID 제어 출력을 반환합니다.
 *
 * **계산 공식:**
 * ```
 * error = target - measured
 * P = Kp * error
 * I = I + Ki * error * dt
 * D = Kd * (error - prev_error) / dt
 * control = P + I + D
 * ```
 *
 * @param pid PID 컨트롤러 구조체 포인터
 * @param target 목표값
 * @param measured 현재 측정값
 * @return PID 제어 출력 값
 *
 * @note `pid_update`는 내부 상태(integral, prev_error)를 갱신합니다.
 *       출력 제한(`output_limit`)이 설정되어 있으면 결과는 해당 범위로 클리핑됩니다.
 */
BYUL_API float pid_update(pid_controller_t* pid, 
    float target, float measured);

/**
 * @brief PID 출력 예측 (상태 변화 없음)
 *
 * `pid_update()`와 동일한 연산으로 제어 출력 값을 계산하지만,
 * 내부 상태(integral, prev_error)를 변경하지 않고 결과만 반환합니다.
 * 
 * **용도:**
 * - PID 파라미터 튜닝 시 특정 입력에 대한 예상 출력을 미리 확인.
 * - 상태를 변경하지 않는 안전한 테스트 시나리오.
 *
 * @param pid PID 컨트롤러 구조체 (상태 불변)
 * @param target 목표값
 * @param measured 현재 측정값
 * @return PID 제어 출력 값 (예상값)
 */
BYUL_API float pid_preview(const pid_controller_t* pid, 
    float target, float measured);

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
 * @brief vec3 PID 상태 복사
 */
BYUL_API void pid_vec3_assign(
    pid_controller_vec3_t* dst, const pid_controller_vec3_t* src);

/**
 * @brief vec3 PID 상태 초기화
 */
BYUL_API void pid_vec3_reset(pid_controller_vec3_t* pid);

/**
 * @brief vec3 PID 상태 설정
 *
 * 3축 PID 컨트롤러의 내부 상태(`integral`, `prev_error`)를 직접 지정합니다.
 * 이 함수는 외부에서 적분항 및 이전 오차를 초기화하거나 특정 값으로 설정해야
 * 하는 경우에 사용됩니다.
 *
 * @param pid 3축 PID 컨트롤러 구조체
 * @param integral 각 축(x, y, z)의 적분항 값
 * @param prev_error 각 축(x, y, z)의 이전 오차 값
 *
 * @note 일반적으로는 `pid_vec3_reset()`을 사용해 0으로 초기화하는 것이 기본이며,
 *       특별히 이전 상태를 복원할 필요가 있을 때만 사용합니다.
 */
BYUL_API void pid_vec3_set_state(pid_controller_vec3_t* pid,
                        const vec3_t* integral,
                        const vec3_t* prev_error);

/**
 * @brief vec3 PID 계산
 *
 * 3축 벡터 형태의 목표값(target)과 현재값(measured)을 비교해 오차를 계산하고,
 * 각 축별 PID 연산을 수행하여 제어 벡터(out_control)를 반환합니다.
 *
 * **계산 원리:**
 * - error = target - measured
 * - P = Kp * error
 * - I = I + Ki * error * dt
 * - D = Kd * (error - prev_error) / dt
 * - control = P + I + D
 *
 * @param pid 3축 PID 컨트롤러 구조체
 * @param target 목표값 (x, y, z)
 * @param measured 현재 측정값 (x, y, z)
 * @param out_control 계산된 제어 출력 (x, y, z)
 *
 * @note `pid_update()`와 달리 벡터 버전은 세 축을 독립적으로 계산하여
 *       통합된 제어 벡터를 제공합니다.
 */
BYUL_API void pid_vec3_update(pid_controller_vec3_t* pid,
                     const vec3_t* target,
                     const vec3_t* measured,
                     vec3_t* out_control);

/**
 * @brief vec3 PID 예측 (상태 변화 없음)
 *
 * `pid_vec3_update()`와 동일한 PID 연산을 수행하지만,
 * 내부 상태(integral, prev_error)를 갱신하지 않고
 * 현재 입력에 대한 제어 출력만 미리 확인할 수 있습니다.
 *
 * **용도:**
 * - 시뮬레이션에서 출력 예상값 확인
 * - 제어 입력을 적용하기 전 안전성 평가
 *
 * @param pid 3축 PID 컨트롤러 구조체 (상태 변경 없음)
 * @param target 목표값 (x, y, z)
 * @param measured 현재 측정값 (x, y, z)
 * @param out_control 계산된 제어 출력 (x, y, z)
 */
BYUL_API void pid_vec3_preview(const pid_controller_vec3_t* pid,
                      const vec3_t* target,
                      const vec3_t* measured,
                      vec3_t* out_control);

#ifdef __cplusplus
}
#endif

#endif // NUMEQ_PID_H
