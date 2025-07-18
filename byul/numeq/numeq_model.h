#ifndef NUMEQ_MODEL_H
#define NUMEQ_MODEL_H

#include "internal/trajectory.h"

#ifdef __cplusplus
extern "C" {
#endif

// ---------------------------------------------------------
// 환경 구조체: 외부 조건
// ---------------------------------------------------------
typedef struct s_environment {
    vec3_t gravity;       // 중력 가속도 (기본: { 0, -9.8, 0 })
    vec3_t wind;          // 바람 가속도
    float air_density;    // 공기 밀도
    float humidity;       // 습도 [%]
    float temperature;    // 온도 [°C]
    float pressure;       // 기압 [Pa]
} environment_t;

// ---------------------------------------------------------
// 물체 구조체: 물체 자체 특성
// ---------------------------------------------------------
typedef struct s_body_properties {
    float mass;             // 질량 (kg)
    float drag_coef;        // 공기 저항 계수
    float cross_section;    // 단면적 (m²)
    float restitution;      // 반발 계수 (0: 흡수, 1: 완전 반사)
    float friction;         // 마찰 계수
} body_properties_t;

// ---------------------------------------------------------
// 위치 계산: p(t)
// ---------------------------------------------------------
BYUL_API void numeq_model_pos_at(float t,
                        const linear_state_t* state0,
                        const environment_t* env,
                        const body_properties_t* body,
                        vec3_t* out_position);

// ---------------------------------------------------------
// 속도 계산: v(t)
// ---------------------------------------------------------
BYUL_API void numeq_model_vel_at(float t,
                        const linear_state_t* state0,
                        const environment_t* env,
                        const body_properties_t* body,
                        vec3_t* out_velocity);

// ---------------------------------------------------------
// 가속도 계산: a(t)
// ---------------------------------------------------------
BYUL_API void numeq_model_accel_at(float t,
                          const linear_state_t* state0,
                          const environment_t* env,
                          const body_properties_t* body,
                          vec3_t* out_accel);

// ---------------------------------------------------------
// 전체 상태 예측: state(t)
// ---------------------------------------------------------
BYUL_API void numeq_model_predict(float t,
                         const linear_state_t* state0,
                         const environment_t* env,
                         const body_properties_t* body,
                         linear_state_t* out_state);

// ---------------------------------------------------------
// 공기 저항력 계산 (a = F / m)
// ---------------------------------------------------------
BYUL_API void numeq_model_drag_force(const vec3_t* velocity,
                            const body_properties_t* body,
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

#ifdef __cplusplus
}
#endif

#endif // NUMEQ_MODEL_H
