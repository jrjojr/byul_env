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

#include "internal/trajectory.h"
#include "internal/vec3.h"
#include "internal/environ.h"
#include "internal/bodyprops.h"

#ifdef __cplusplus
extern "C" {
#endif

// ---------------------------------------------------------
// 위치 계산: p(t)
// ---------------------------------------------------------
BYUL_API void numeq_model_pos_at(float t,
                        const linear_state_t* state0,
                        const environ_t* env,
                        const bodyprops_t* body,
                        vec3_t* out_position);

// ---------------------------------------------------------
// 속도 계산: v(t)
// ---------------------------------------------------------
BYUL_API void numeq_model_vel_at(float t,
                        const linear_state_t* state0,
                        const environ_t* env,
                        const bodyprops_t* body,
                        vec3_t* out_velocity);

// ---------------------------------------------------------
// 가속도 계산: a(t)
// ---------------------------------------------------------
BYUL_API void numeq_model_accel_at(float t,
                          const linear_state_t* state0,
                          const environ_t* env,
                          const bodyprops_t* body,
                          vec3_t* out_accel);

// ---------------------------------------------------------
// 전체 상태 예측: state(t)
// ---------------------------------------------------------
BYUL_API void numeq_model_predict(float t,
                         const linear_state_t* state0,
                         const environ_t* env,
                         const bodyprops_t* body,
                         linear_state_t* out_state);

/**
 * @brief t초 후 전체 상태 예측 (RK4 적분 기반)
 *
 * @param t        예측할 시간 (초)
 * @param state0   초기 상태 (선형 운동)
 * @param env      환경 정보 (중력, 바람 등)
 * @param body     물체의 물리 속성 (질량, 항력계수 등)
 * @param steps    적분 스텝 수 (예: t=1초, steps=60 → dt=1/60)
 * @param out_state 예측된 최종 선형 상태
 */
BYUL_API void numeq_model_predict_rk4(
    float t,
    const linear_state_t* state0,
    const environ_t* env,
    const bodyprops_t* body,
    int steps,
    linear_state_t* out_state);                         

// ---------------------------------------------------------
// 공기 저항력 계산 (a = F / m)
// ---------------------------------------------------------
BYUL_API void numeq_model_drag_force(const vec3_t* velocity,
                            const bodyprops_t* body,
                            float air_density,
                            vec3_t* out_drag_accel);

// ---------------------------------------------------------
// 최고점 여부 판단 (vy ≈ 0)
// ---------------------------------------------------------
BYUL_API bool numeq_model_is_apex(const linear_state_t* state);

// ---------------------------------------------------------
// 착지 여부 판단
// ---------------------------------------------------------
BYUL_API bool numeq_model_is_grounded(const linear_state_t* state,
                             float ground_height);

// ---------------------------------------------------------
// 충돌 반발 계산 인터페이스 (외부 위임 가능)
// ---------------------------------------------------------

// 함수포인터 타입 정의
typedef bool (*numeq_bounce_func)(
    const vec3_t* velocity_in,
    const vec3_t* normal,
    float restitution,
    void* userdata,
    vec3_t* out_velocity_out
);

// 외부 콜백 등록 / 조회
BYUL_API void numeq_model_set_bounce_func(
    numeq_bounce_func func, void* userdata);

BYUL_API void numeq_model_get_bounce_func(
    numeq_bounce_func* out_func, void** out_userdata);

// 기본 반사 함수 (내장 버전, fallback 용)
BYUL_API bool numeq_model_default_bounce(const vec3_t* velocity_in,
                                const vec3_t* normal,
                                float restitution,
                                vec3_t* out_velocity_out);

/**
 * @brief 두 개체의 충돌 예측 시간과 충돌 지점을 계산합니다.
 *
 * @param my_state     나의 초기 상태 (선형 운동)
 * @param other_state  상대방 초기 상태 (선형 운동)
 * @param env          환경 정보 (중력, 바람 등)
 * @param my_body      나의 물리 속성 (질량, 항력 등)
 * @param other_body   상대방 물리 속성 (질량, 항력 등)
 * @param radius_sum   충돌 반경 (두 개체 반경 합)
 * @param max_time     예측할 최대 시간 (초)
 * @param time_step    적분 간격 (초)
 * @param out_time     충돌 발생 시간 (없으면 -1)
 * @param out_point    충돌 지점 좌표 (없으면 (0,0,0))
 * @return true        충돌이 발생하면 true
 * @return false       충돌이 발생하지 않으면 false
 */
BYUL_API bool numeq_model_predict_collision(
    const linear_state_t* my_state,
    const linear_state_t* other_state,
    const environ_t* env,
    const bodyprops_t* my_body,
    const bodyprops_t* other_body,
    float radius_sum,
    float max_time,
    float time_step,
    float* out_time,
    vec3_t* out_point);

#ifdef __cplusplus
}
#endif

#endif // NUMEQ_MODEL_H
