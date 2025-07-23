#ifndef ENVIRON_H
#define ENVIRON_H

#ifdef __cplusplus
extern "C" {
#endif

#include "byul_common.h"
#include "internal/vec3.h"

// 전방 선언
typedef struct s_bodyprops bodyprops_t;
typedef struct s_projectile projectile_t;

// ---------------------------------------------------------
// 환경 구조체: 외부 조건
// ---------------------------------------------------------
/**
 * @struct environ_t
 * @brief 시뮬레이션 환경 설정 값
 *
 * 이 구조체는 시뮬레이션에서 영향을 주는 기본적인 환경 요소를 나타냅니다.
 * 예를 들어 중력, 바람, 공기 밀도 등 외부 조건을 정의할 수 있습니다.
 */
typedef struct s_environ {
    vec3_t gravity;       /**< 중력 가속도 벡터 (기본: {0, -9.8, 0} m/s²) */
    vec3_t wind;          /**< 바람 가속도 벡터 (m/s²) */
    float air_density;    /**< 공기 밀도 (kg/m³) - 기본 1.225 kg/m³ (해수면 기준) */
    float humidity;       /**< 습도 (%) - 0~100 사이 값 */
    float temperature;    /**< 온도 (°C) - 기본 20°C */
    float pressure;       /**< 기압 (Pa) - 기본 101,325 Pa (해수면 기준) */
} environ_t;

/**
 * @brief environ_t 기본값 초기화
 *
 * 이 함수는 environ_t 구조체를 다음의 기본값으로 초기화합니다.
 * - gravity = {0.0f, -9.8f, 0.0f} (지구 표준 중력)
 * - wind = {0.0f, 0.0f, 0.0f} (바람 없음)
 * - air_density = 1.225f (해수면 기준 공기 밀도, kg/m³)
 * - humidity = 50.0f (상대 습도 50%)
 * - temperature = 20.0f (온도 20°C)
 * - pressure = 101325.0f (기압 101,325 Pa, 해수면 기준)
 *
 * @param env 초기화할 환경 구조체 (NULL이면 동작하지 않음)
 */
BYUL_API void environ_init(environ_t* env);

/**
 * @brief environ_t를 지정한 값으로 초기화
 *
 * @param env 초기화할 환경 구조체 (NULL이면 동작하지 않음)
 * @param gravity 중력 가속도 벡터 (예: {0, -9.8, 0})
 * @param wind 바람 가속도 벡터 (예: {0, 0, 0})
 * @param air_density 공기 밀도 (kg/m³, 기본 1.225f)
 * @param humidity 습도 [%] (0 ~ 100)
 * @param temperature 온도 [°C] (기본 20.0f)
 * @param pressure 기압 [Pa] (기본 101,325 Pa)
 */
BYUL_API void environ_init_full(environ_t* env,
                           const vec3_t* gravity,
                           const vec3_t* wind,
                           float air_density,
                           float humidity,
                           float temperature,
                           float pressure);

/**
 * @brief environ_t 복사
 * @param out 복사 대상
 * @param src 원본
 */
BYUL_API void environ_assign(
    environ_t* out, const environ_t* src);

/**
 * @brief environ_t에서 총 외부 가속도(중력 + 바람)를 계산합니다.
 *
 * @param[out] out 결과 가속도 벡터
 * @param[in] env 환경 데이터 (NULL이면 기본 중력 적용)
 */
BYUL_API void environ_get_accel(vec3_t* out, const environ_t* env);

/**
 * @brief environ_t와 bodyprops_t를 기반으로 총 가속도 계산
 *
 * 이 함수는 중력(gravity), 바람(wind), 공기 저항(drag)을 고려한
 * 총 외부 가속도를 계산하여 out에 저장합니다.
 *
 * @param[out] out       결과 가속도 벡터 (m/s²)
 * @param[in] env        환경 데이터 (NULL이면 기본 중력만 적용)
 * @param[in] props       물체 특성 데이터 (drag 계산에 필요, NULL이면 무시)
 * @param[in] velocity   현재 속도 벡터 (drag 계산에 필요, NULL이면 무시)
 */
BYUL_API void environ_get_accel_ext(vec3_t* out,
                       const environ_t* env,
                       const bodyprops_t* props,
                       const vec3_t* velocity);

// ---------------------------------------------------------
// 환경 함수 타입 (범용)
// ---------------------------------------------------------
/**
 * @brief 환경 함수 타입
 *
 * 외부 가속도(중력, 바람, 외력 등)를 계산하기 위한 콜백입니다.
 *
 * @param[out] out_accel 외부 가속도 결과 벡터 (NULL이면 내부 static 사용)
 * @param[in]  dt        시간 간격 (초)
 * @param[in]  userdata  환경 관련 사용자 데이터
 * @return 외부 가속도 벡터 포인터 (m/s²)
 *
 * @note
 * - out_accel이 NULL이면 static vec3_t 반환 (멀티스레드 환경에서는 제공 필수).
 */
typedef const vec3_t* (*environ_func)(
    vec3_t* out_accel,
    float dt,
    void* userdata
);


// ---------------------------------------------------------
// 기본 환경 함수
// ---------------------------------------------------------
/**
 * @brief 외부 힘 없음
 * @return 항상 (0, 0, 0)
 */
BYUL_API const vec3_t* environ_func_none(
    vec3_t* out_accel,
    float dt,
    void* userdata);

/**
 * @brief 표준 중력 적용
 * @return (0, -9.81, 0)
 */
BYUL_API const vec3_t* environ_func_default(
    vec3_t* out_accel,
    float dt,
    void* userdata);

/**
 * @brief 일정한 바람 + 중력
 * @param[in] userdata const vec3_t* (고정 바람 벡터)
 */
BYUL_API const vec3_t* environ_func_constant(
    vec3_t* out_accel,
    float dt,
    void* userdata);


// ---------------------------------------------------------
// 주기적 환경 데이터 구조체
// ---------------------------------------------------------
/**
 * @struct env_periodic_data_t
 * @brief 주기적 바람 및 외력 데이터를 담는 구조체
 */
typedef struct s_env_periodic_data {
    vec3_t base_wind;      ///< 기본 바람 벡터 (m/s)
    vec3_t gust_amplitude; ///< 바람 변동 진폭 (m/s)
    float gust_frequency;  ///< 바람 변동 주파수 (Hz)
    float time;            ///< 누적 시간 (s)
    vec3_t gravity;        ///< 중력 가속도 (기본 {0, -9.81, 0})
} env_periodic_data_t;

/**
 * @brief env_periodic_data_t 기본값 초기화
 */
BYUL_API void env_periodic_init(env_periodic_data_t* out);

/**
 * @brief env_periodic_data_t를 지정한 값으로 초기화
 */
BYUL_API void env_periodic_init_full(
    env_periodic_data_t* out,
    const vec3_t* base_wind,
    const vec3_t* gust_amp,
    float gust_freq,
    const vec3_t* gravity);

/**
 * @brief env_periodic_data_t 복사
 */
BYUL_API void env_periodic_assign(
    env_periodic_data_t* out, const env_periodic_data_t* src);

/**
 * @brief 주기적 바람 + 중력 환경 함수
 */
BYUL_API const vec3_t* environ_func_periodic(
    vec3_t* out_accel,
    float dt,
    void* userdata);

#ifdef __cplusplus
}
#endif

#endif // ENVIRON_H
