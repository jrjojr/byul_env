/**
 * @file numeq_model.h
 * @brief 수치 방정식을 통한 물리 상태 예측 모듈
 *
 * 이 모듈은 주어진 초기 운동 상태(linear_state_t), 환경(environ_t), 
 * 물체의 물리 속성(bodyprops_t)을 기반으로 다음을 제공합니다:
 *
 * - **위치 p(t)**, **속도 v(t)**, **가속도 a(t)** 예측 (포물선 운동 + 항력 반영)
 * - t초 후의 전체 선형 상태(linear_state_t) 계산
 * - 공기 저항력(Drag) 계산 (F_drag = 0.5 * ρ * v² * Cd * A)
 * - 운동 상태의 최고점, 착지 여부 판단
 * - 충돌 반발(bounce) 계산 인터페이스 제공
 *
 * @note 이 모듈은 회전 운동(attitude_state_t)은 다루지 않으며, 
 * 선형 운동(위치/속도/가속도)만을 처리합니다.
 */
#ifndef NUMEQ_MODEL_H
#define NUMEQ_MODEL_H

#include "trajectory.h"
#include "vec3.h"
#include "environ.h"
#include "bodyprops.h"

#ifdef __cplusplus
extern "C" {
#endif

// ---------------------------------------------------------
// 공기 저항력 계산 (a = F / m)
// ---------------------------------------------------------

/**
 * @brief 물체에 작용하는 공기 저항 가속도를 계산합니다.
 *
 * @param state0       초기 선형 상태 (위치, 속도)
 * @param env          환경 정보 (공기 밀도, 바람)
 * @param body         물체의 물리 속성 (질량, 항력계수 등)
 * @param[out] out_drag_accel 계산된 항력 가속도 (m/s²)
 *
 * @note 드래그는 상대 속도 (v - wind)에 따라 계산됩니다.
 */
BYUL_API void numeq_model_drag_accel(const linear_state_t* state0,
                        const environ_t* env,
                        const bodyprops_t* body,
                        vec3_t* out_drag_accel);

/**
 * @brief 현재 시점의 총 가속도를 계산합니다.
 *
 * @param state        현재 선형 상태
 * @param env          환경 정보
 * @param body         물체의 물리 속성
 * @param[out] out_accel 계산된 총 가속도 (m/s²)
 *
 * @note 총 가속도는 중력, 항력 및 환경 보정을 포함합니다.
 */
BYUL_API void numeq_model_accel(const linear_state_t* state,
                       const environ_t* env,
                       const bodyprops_t* body,
                       vec3_t* out_accel);

/**
 * @brief 중력을 제외한 외력 가속도(drag + wind + state.accel)를 계산합니다.
 *
 * numeq_model_accel()이 중력(env->gravity)을 포함한 전체 가속도를 계산하는 것과 달리,
 * 이 함수는 drag, 바람, 물체 자체 가속도 등 중력을 제외한 모든 외력을 합산합니다.
 *
 * @param[in]  state      선형 상태 (속도, 현재 가속도)
 * @param[in]  env        환경 데이터 (바람, 습도, 기압 등)
 * @param[in]  body       물체 특성 (질량, 항력 계수 등)
 * @param[out] out_accel  계산된 외력 가속도 벡터 (중력 제외)
 */
BYUL_API void numeq_model_accel_except_gravity(
    const linear_state_t* state,
    const environ_t* env,
    const bodyprops_t* body,
    vec3_t* out_accel);

// ---------------------------------------------------------
// 가속도 계산: a(t)
// ---------------------------------------------------------

/**
 * @brief t초 후의 가속도를 계산합니다.
 *
 * @param t            미래 예측 시간 (초)
 * @param state0       초기 상태
 * @param env          환경 정보
 * @param body         물체의 물리 속성
 * @param[out] out_accel t초 후 예상 가속도 (m/s²)
 *
 * @note 내부적으로 `numeq_model_vel_at()`를 호출하여 
 * t 시점 속도를 기반으로 항력을 재계산합니다.
 */
BYUL_API void numeq_model_accel_at(float t,
                          const linear_state_t* state0,
                          const environ_t* env,
                          const bodyprops_t* body,
                          vec3_t* out_accel);

// ---------------------------------------------------------
// 위치 계산: p(t)
// ---------------------------------------------------------

/**
 * @brief t초 후의 위치를 계산합니다. (선형 근사)
 *
 * @param t            미래 예측 시간 (초)
 * @param state0       초기 상태
 * @param env          환경 정보
 * @param body         물체의 물리 속성
 * @param[out] out_position t초 후 예상 위치 (m)
 *
 * @note 등가속도 근사를 사용하며, p(t) = p₀ + v₀t + 0.5a₀t² 공식을 적용합니다.
 */
BYUL_API void numeq_model_pos_at(float t,
                        const linear_state_t* state0,
                        const environ_t* env,
                        const bodyprops_t* body,
                        vec3_t* out_position);

// ---------------------------------------------------------
// 속도 계산: v(t)
// ---------------------------------------------------------

/**
 * @brief t초 후의 속도를 계산합니다. (선형 근사)
 *
 * @param t            미래 예측 시간 (초)
 * @param state0       초기 상태
 * @param env          환경 정보
 * @param body         물체의 물리 속성
 * @param[out] out_velocity t초 후 예상 속도 (m/s)
 *
 * @note 등가속도 근사를 사용하며, v(t) = v₀ + a₀t 공식을 적용합니다.
 */
BYUL_API void numeq_model_vel_at(float t,
                        const linear_state_t* state0,
                        const environ_t* env,
                        const bodyprops_t* body,
                        vec3_t* out_velocity);

// ---------------------------------------------------------
// 전체 상태 예측: state(t)
// ---------------------------------------------------------

/**
 * @brief t초 후의 선형 상태 (위치, 속도, 가속도)를 계산합니다.
 *
 * @param t            미래 예측 시간 (초)
 * @param state0       초기 상태
 * @param env          환경 정보
 * @param body         물체의 물리 속성
 * @param[out] out_state t초 후 예상 선형 상태
 */
BYUL_API void numeq_model_calc(float t,
                         const linear_state_t* state0,
                         const environ_t* env,
                         const bodyprops_t* body,
                         linear_state_t* out_state);

/**
 * @brief RK4 적분 기반으로 t초 후 선형 상태를 예측합니다.
 *
 * @param t            미래 예측 시간 (초)
 * @param state0       초기 상태
 * @param env          환경 정보
 * @param body         물체의 물리 속성
 * @param steps        적분 스텝 수 (예: t=1초, steps=60 → dt=1/60)
 * @param[out] out_state t초 후 RK4 기반 선형 상태
 *
 * @note drag, 중력, 환경 영향이 시간에 따라 변할 때 더 높은 정확도를 제공합니다.
 */
BYUL_API void numeq_model_calc_rk4(
    float t,
    const linear_state_t* state0,
    const environ_t* env,
    const bodyprops_t* body,
    int steps,
    linear_state_t* out_state);

// ---------------------------------------------------------
// 기본 충돌 반발
// ---------------------------------------------------------

/**
 * @brief 벡터 반사를 이용한 기본 충돌 반발 속도를 계산합니다.
 *
 * @param velocity_in   충돌 전 속도 벡터
 * @param normal        충돌 표면의 법선 (정규화 필요)
 * @param restitution   반발 계수 (0~1)
 * @param[out] out_velocity_out 반발 후 속도 벡터
 *
 * @return 계산 성공 시 true
 */
bool numeq_model_bounce(const vec3_t* velocity_in,
                        const vec3_t* normal,
                        float restitution,
                        vec3_t* out_velocity_out);

/**
 * @brief 두 객체가 충돌할 시점을 예측합니다.
 *
 * @param my_state      나의 선형 상태
 * @param other_state   상대 선형 상태
 * @param radius_sum    두 객체의 반경 합 (충돌 임계값)
 * @param[out] out_time 충돌 예상 시간 (초)
 * @param[out] out_point 충돌 예상 지점
 *
 * @return 충돌이 예상되면 true, 아니면 false
 *
 * @note 현재 구현은 등속/등가속도를 가정한 근사 계산입니다.
 */
bool numeq_model_calc_collision(
    const linear_state_t* my_state,
    const linear_state_t* other_state,
    float radius_sum,
    float* out_time,
    vec3_t* out_point);

#ifdef __cplusplus
}
#endif

#endif // NUMEQ_MODEL_H
