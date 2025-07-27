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

/**
 * @typedef environ_func
 * @brief 외부 가속도 계산 함수 포인터
 *
 * @param[in] env        환경 데이터 포인터
 * @param[in] dt         시간 간격 (초)
 * @param[in] userdata   사용자 정의 데이터
 * @param[out] out_accel 계산된 가속도 벡터
 * @return 계산된 가속도 벡터 포인터
 */
typedef const vec3_t* (*environ_func)(
    const struct s_environ* env,
    float dt,
    void* userdata,
    vec3_t* out_accel
);

// ---------------------------------------------------------
// 환경 구조체
// ---------------------------------------------------------
/**
 * @struct environ_t
 * @brief 시뮬레이션 환경 데이터와 외부 가속도 계산을 위한 함수 포인터
 *
 * 이 구조체는 발사체, 엔티티 등의 궤적 예측 시 고려되는 외부 환경 요소를 정의합니다.
 * - **중력(gravity)**, **바람(wind)**, **공기 밀도(air_density)** 등을 저장합니다.
 * - `environ_func`를 통해 동적 또는 사용자 정의 환경 가속도를 계산할 수 있습니다.
 */
typedef struct s_environ {
    vec3_t gravity;        /**< 중력 가속도 (m/s²), 기본 {0, -9.81, 0} */
    vec3_t wind;           /**< 바람 가속도 (m/s²) */
    float air_density;     /**< 공기 밀도 (kg/m³), 기본 1.225 */
    float humidity;        /**< 습도 [%] */
    float temperature;     /**< 온도 [°C] */
    float pressure;        /**< 기압 [Pa] */

    environ_func environ_fn;

    void* userdata;        /**< environ_func에 전달할 사용자 데이터 */
} environ_t;

// ---------------------------------------------------------
// 초기화 함수
// ---------------------------------------------------------
/**
 * @brief environ_t 구조체를 기본값으로 초기화합니다.
 *
 * - 중력: {0, -9.81, 0}
 * - 바람: {0, 0, 0}
 * - 공기 밀도: 1.225 kg/m³
 * - 습도: 50 %
 * - 온도: 20 °C
 * - 기압: 101,325 Pa
 * - environ_func: environ_calc_gravity
 *
 * @param[out] env 초기화할 환경 구조체
 */
BYUL_API void environ_init(environ_t* env);

/**
 * @brief environ_t 구조체를 지정 값으로 초기화합니다.
 *
 * @param[out] env        초기화할 환경 구조체
 * @param[in] gravity     중력 벡터 (NULL이면 기본값 사용)
 * @param[in] wind        바람 벡터 (NULL이면 기본값 사용)
 * @param[in] air_density 공기 밀도 (kg/m³)
 * @param[in] humidity    습도 [%]
 * @param[in] temperature 온도 [°C]
 * @param[in] pressure    기압 [Pa]
 * @param[in] environ_fn    외부 가속도 함수 포인터 (NULL이면 기본값 사용)
 * @param[in] userdata    가속도 함수에 전달할 사용자 데이터
 */
BYUL_API void environ_init_full(environ_t* env,
                                    const vec3_t* gravity,
                                    const vec3_t* wind,
                                    float air_density,
                                    float humidity,
                                    float temperature,
                                    float pressure,
                                    environ_func environ_fn,
                                    void* userdata);

/**
 * @brief 환경 데이터를 복사합니다.
 * @param[out] out 복사 대상
 * @param[in]  src 원본
 */
BYUL_API void environ_assign(environ_t* out, 
                                 const environ_t* src);

// ---------------------------------------------------------
// 외력 보정 (중력 포함 여부 상관 없음)
// ---------------------------------------------------------
/**
 * @brief 외력(accel)에 환경 보정을 일관되게 적용합니다.
 *
 * 입력된 가속도 벡터(accel)가 중력을 포함하고 있더라도, 
 * 보정 과정에서 중력 여부를 따지지 않고 drag, 바람 등 외력에 대한 
 * 환경 요인(습도, 온도, 기압)을 동일하게 적용합니다.
 *
 * @param[in]    env    환경 데이터 (바람, 습도, 온도, 기압 등)
 * @param[inout] accel  보정할 가속도 벡터 (중력 포함 여부 무관)
 */
BYUL_API void environ_adjust_accel(
    const environ_t* env, vec3_t* accel);

/**
 * @brief 가속도 벡터(accel)에서 중력 성분을 분리하여 외력만 환경 요인으로 보정하고,
 *        필요 시 중력을 다시 합산합니다.
 *
 * 이 함수는 입력된 가속도 벡터가 중력을 포함하거나 포함하지 않는 상황 모두를 지원합니다.
 * `has_gravity` 플래그로 입력 벡터에 중력이 포함되어 있는지 여부를 명시해야 하며,
 * 이 값에 따라 보정 로직이 달라집니다.
 *
 * 동작 원리:
 * - has_gravity = true:
 *    1) accel에서 env->gravity를 먼저 빼서 순수 외력만 추출
 *    2) 외력 성분에만 환경 보정(factor) 적용
 *    3) 보정이 끝난 외력에 다시 중력(env->gravity) 합산
 * - has_gravity = false:
 *    1) accel을 그대로 외력으로 간주
 *    2) 외력 성분에만 환경 보정(factor) 적용
 *    3) 중력은 추가하지 않음
 *
 * @param[in]    env            환경 데이터 (중력, 바람, 습도, 온도, 기압 등)
 * @param[in]    has_gravity    true이면 accel이 중력을 포함한 전체 가속도,
 *                              false이면 순수 외력만 포함된 벡터를 의미
 * @param[inout] accel          보정할 가속도 벡터 (중력 포함 여부는 has_gravity로 결정)
 */
BYUL_API void environ_adjust_accel_gsplit(
    const environ_t* env, bool has_gravity, vec3_t* accel);


// ---------------------------------------------------------
// 기본 환경 가속도 함수 (calc_accel 용)
// ---------------------------------------------------------
/**
 * @brief 외부 가속도가 항상 0인 환경 함수
 */
BYUL_API const vec3_t* environ_calc_none(
    const environ_t* env, float dt, void* userdata, vec3_t* out_accel);

/**
 * @brief 표준 중력만 적용하는 환경 함수
 */
BYUL_API const vec3_t* environ_calc_gravity(
    const environ_t* env, float dt, void* userdata, vec3_t* out_accel);

/**
 * @brief 표준 중력 + 고정 바람을 적용하는 환경 함수
 */
BYUL_API const vec3_t* environ_calc_gravity_wind(
    const environ_t* env, float dt, void* userdata, vec3_t* out_accel);

// ---------------------------------------------------------
// 주기적 환경
// ---------------------------------------------------------
/**
 * @struct environ_periodic_t
 * @brief 주기적으로 변하는 바람과 외력을 표현하는 환경 데이터
 */
typedef struct s_environ_periodic {
    vec3_t base_wind;      ///< 기본 바람 벡터
    vec3_t gust_amplitude; ///< 바람 변동 진폭
    float gust_frequency;  ///< 바람 변동 주파수 (Hz)
    float time;            ///< 누적 시간 (s)
    vec3_t gravity;        ///< 중력 가속도 (기본 {0, -9.81, 0})
} environ_periodic_t;

/**
 * @brief 주기적 환경 데이터를 기본값으로 초기화합니다.
 *
 * @param[out] out 초기화할 주기적 환경 구조체
 */
BYUL_API void environ_periodic_init(environ_periodic_t* out);

/**
 * @brief 주기적 환경 데이터를 지정 값으로 초기화합니다.
 *
 * @param[in]  base_wind  기본 바람 벡터
 * @param[in]  gust_amp   바람 진폭 벡터
 * @param[in]  gust_freq  바람 주파수 (Hz)
 * @param[in]  gravity    중력 벡터
 * @param[out] out        초기화할 주기적 환경 구조체
 */
BYUL_API void environ_periodic_init_full(
    const vec3_t* base_wind,
    const vec3_t* gust_amp,
    float gust_freq,
    const vec3_t* gravity,
    environ_periodic_t* out);

/**
 * @brief 주기적 환경 데이터를 복사합니다.
 * @param[out] out 복사 대상
 * @param[in]  src 원본
 */
BYUL_API void environ_periodic_assign(
    const environ_periodic_t* src, environ_periodic_t* out);

/**
 * @brief 주기적 바람과 중력을 적용하는 환경 함수
 */
BYUL_API const vec3_t* environ_calc_periodic(
    const environ_t* env, float dt, void* userdata, vec3_t* out_accel);

#ifdef __cplusplus
}
#endif

#endif // ENVIRON_H
