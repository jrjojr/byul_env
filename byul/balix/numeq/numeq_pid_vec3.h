#ifndef NUMEQ_PID_VEC3_H
#define NUMEQ_PID_VEC3_H

#include "internal/numeq_pid.h"
#include "internal/trajectory.h"

#ifdef __cplusplus
extern "C" {
#endif

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
BYUL_API void pid_vec3_init(pid_controller_vec3_t* pid);

/**
 * @brief vec3 PID 초기화
 */
BYUL_API void pid_vec3_init_full(pid_controller_vec3_t* pid,
                   float kp, float ki, float kd,
                   float dt);                   

/**
 * @brief vec3 PID 초기화
 */
BYUL_API void pid_vec3_auto(pid_controller_vec3_t* pid, float dt);

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

#endif // NUMEQ_PID_VEC3_H
