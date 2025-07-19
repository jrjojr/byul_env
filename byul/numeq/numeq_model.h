#ifndef NUMEQ_MODEL_H
#define NUMEQ_MODEL_H

#include "internal/trajectory.h"

#ifdef __cplusplus
extern "C" {
#endif

// ---------------------------------------------------------
// 환경 구조체: 외부 조건
// ---------------------------------------------------------
/**
 * @struct environment_t
 * @brief 시뮬레이션 환경 설정 값
 *
 * 이 구조체는 시뮬레이션에서 영향을 주는 기본적인 환경 요소를 나타냅니다.
 * 예를 들어 중력, 바람, 공기 밀도 등 외부 조건을 정의할 수 있습니다.
 */
typedef struct s_environment {
    vec3_t gravity;       /**< 중력 가속도 벡터 (기본: {0, -9.8, 0} m/s²) */
    vec3_t wind;          /**< 바람 가속도 벡터 (m/s²) */
    float air_density;    /**< 공기 밀도 (kg/m³) - 기본 1.225 kg/m³ (해수면 기준) */
    float humidity;       /**< 습도 (%) - 0~100 사이 값 */
    float temperature;    /**< 온도 (°C) - 기본 20°C */
    float pressure;       /**< 기압 (Pa) - 기본 101,325 Pa (해수면 기준) */
} environment_t;

/**
 * @brief environment_t 기본값 초기화
 *
 * 이 함수는 environment_t 구조체를 다음의 기본값으로 초기화합니다.
 * - gravity = {0.0f, -9.8f, 0.0f} (지구 표준 중력)
 * - wind = {0.0f, 0.0f, 0.0f} (바람 없음)
 * - air_density = 1.225f (해수면 기준 공기 밀도, kg/m³)
 * - humidity = 50.0f (상대 습도 50%)
 * - temperature = 20.0f (온도 20°C)
 * - pressure = 101325.0f (기압 101,325 Pa, 해수면 기준)
 *
 * @param env 초기화할 환경 구조체 (NULL이면 동작하지 않음)
 */
BYUL_API void environment_init(environment_t* env);

/**
 * @brief environment_t를 지정한 값으로 초기화
 *
 * @param env 초기화할 환경 구조체 (NULL이면 동작하지 않음)
 * @param gravity 중력 가속도 벡터 (예: {0, -9.8, 0})
 * @param wind 바람 가속도 벡터 (예: {0, 0, 0})
 * @param air_density 공기 밀도 (kg/m³, 기본 1.225f)
 * @param humidity 습도 [%] (0 ~ 100)
 * @param temperature 온도 [°C] (기본 20.0f)
 * @param pressure 기압 [Pa] (기본 101,325 Pa)
 */
BYUL_API void environment_init_full(environment_t* env,
                           const vec3_t* gravity,
                           const vec3_t* wind,
                           float air_density,
                           float humidity,
                           float temperature,
                           float pressure);

/**
 * @brief environment_t 복사
 * @param out 복사 대상
 * @param src 원본
 */
BYUL_API void environment_copy(environment_t* out, const environment_t* src);

/**
 * @struct body_properties_t
 * @brief 물체(물리 객체)의 고유 특성 값
 *
 * 물체의 질량, 마찰 계수, 공기 저항 계수 등을 포함합니다.
 * 이 값들은 물리 시뮬레이션에서 물체의 거동(운동, 반발, 마찰)에 영향을 줍니다.
 */
typedef struct s_body_properties {
    float mass;             /**< 질량 (kg) - 기본값 1.0 kg */
    float drag_coef;        /**< 공기 저항 계수 (Cd) */
    float cross_section;    /**< 단면적 (m²) - 공기 저항 계산에 사용 */
    float restitution;      /**< 반발 계수 (0: 흡수, 1: 완전 반사) */
    float friction;         /**< 마찰 계수 (0~1, 0: 마찰 없음) */
} body_properties_t;

// ---------------------------------------------------------
// body_properties_t 유틸리티
// ---------------------------------------------------------

/**
 * @brief body_properties_t 기본값 초기화
 *
 * 이 함수는 body_properties_t 구조체를 다음의 기본값으로 초기화합니다.
 * - mass = 1.0f (기본 질량 1 kg)
 * - drag_coef = 0.47f (구체의 전형적인 공기 저항 계수)
 * - cross_section = 0.01f (기본 단면적 0.01 m², 약 10cm²)
 * - restitution = 0.5f (중간 수준 반발 계수, 0: 흡수 ~ 1: 완전 반사)
 * - friction = 0.5f (중간 수준 마찰 계수, 0: 마찰 없음 ~ 1: 매우 높은 마찰)
 *
 * @param body 초기화할 물체 특성 구조체 (NULL이면 동작하지 않음)
 */
BYUL_API void body_properties_init(body_properties_t* body);

/**
 * @brief body_properties_t를 지정한 값으로 초기화
 *
 * @param body 초기화할 구조체 (NULL이면 동작하지 않음)
 * @param mass 질량 [kg] (예: 1.0f)
 * @param drag_coef 공기 저항 계수 (예: 0.47f, 구체 기준)
 * @param cross_section 단면적 [m²] (예: 0.01f)
 * @param restitution 반발 계수 (0: 흡수, 1: 완전 반사)
 * @param friction 마찰 계수 (0~1, 0: 없음, 1: 최대)
 */
BYUL_API void body_properties_init_full(body_properties_t* body,
                               float mass,
                               float drag_coef,
                               float cross_section,
                               float restitution,
                               float friction);

/**
 * @brief body_properties_t 복사
 * @param out 복사 대상
 * @param src 원본
 */
BYUL_API void body_properties_copy(
    body_properties_t* out, const body_properties_t* src);

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
