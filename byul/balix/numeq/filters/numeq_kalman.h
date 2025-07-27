/**
 * @file numeq_kalman.h
 * @brief 칼만 필터 (Kalman Filter) 모듈
 *
 * 1차원 스칼라 및 3차원 벡터 상태(위치, 속도)에 대한
 * **재귀적 상태 추정**을 수행하는 Kalman Filter 구현체입니다.
 *
 * 센서에서 얻는 노이즈가 있는 측정값을 기반으로 시스템의 상태를
 * **시간 업데이트(Time Update)** → **측정 업데이트(Measurement Update)** 하며,
 * 노이즈를 최소화하고 더 정확한 추정값을 제공합니다.
 */

#ifndef NUMEQ_KALMAN_H
#define NUMEQ_KALMAN_H

#include "internal/trajectory.h"

#ifdef __cplusplus
extern "C" {
#endif

// =========================================================
// 1. 스칼라 칼만 필터 (1D)
// =========================================================

/**
 * @struct kalman_filter_t
 * @brief 1차원 칼만 필터
 *
 * ### 기본 초기값:
 * - `x = 0` (초기 상태)
 * - `p = 1` (초기 오차 공분산)
 * - `q = 0.01` (프로세스 노이즈)
 * - `r = 1.0` (측정 노이즈)
 * - `k = 0` (칼만 이득)
 *
 * ### 표준 범위:
 * - `x`: 일반적으로 -1000 ~ +1000 (센서 값 범위에 따름)
 * - `p`: 0.001 ~ 100 (오차 공분산)
 * - `q`: 0.0001 ~ 0.1 (프로세스 노이즈)
 * - `r`: 0.01 ~ 10 (측정 노이즈)
 */
typedef struct s_kalman_filter {
    float x;   /**< 상태 추정값 */
    float p;   /**< 오차 공분산 */
    float q;   /**< 프로세스 노이즈 */
    float r;   /**< 측정 노이즈 */
    float k;   /**< 칼만 이득 */
} kalman_filter_t;

/**
 * @brief 1차원 칼만 필터 기본값 초기화
 *
 * ### 기본값:
 * - `x = 0`
 * - `p = 1`
 * - `q = 0.01`
 * - `r = 1`
 * - `k = 0`
 *
 * @param kf 초기화할 칼만 필터 포인터
 */
BYUL_API void kalman_init(kalman_filter_t* kf);

/**
 * @brief 1차원 칼만 필터 초기화 (사용자 지정)
 *
 * @param kf 필터 포인터
 * @param init_x 초기 상태값 (권장 범위: -1000 ~ +1000)
 * @param init_p 초기 오차 공분산 (권장 범위: 0.001 ~ 100)
 * @param process_noise 프로세스 노이즈 Q (권장 범위: 0.0001 ~ 0.1)
 * @param measurement_noise 측정 노이즈 R (권장 범위: 0.01 ~ 10)
 */
BYUL_API void kalman_init_full(kalman_filter_t* kf, float init_x, float init_p,
                               float process_noise, float measurement_noise);

                               /**
 * @brief kalman_filter_t 복사
 */
BYUL_API void kalman_assign(kalman_filter_t* dst, const kalman_filter_t* src);

/**
 * @brief 시간 업데이트 (Time Update)
 *
 * 예측 단계: 상태값 `x`는 유지되고 오차 공분산 `p`는 `p + q`로 증가합니다.
 *
 * @param kf 칼만 필터 포인터
 */
BYUL_API void kalman_time_update(kalman_filter_t* kf);

/**
 * @brief 측정 업데이트 (Measurement Update)
 *
 * 새로운 측정값 `measured`를 기반으로 보정된 상태값을 계산합니다.
 *
 * @param kf 칼만 필터 포인터
 * @param measured 새 측정값 (z)
 * @return 보정된 상태 추정값
 */
BYUL_API float kalman_measurement_update(kalman_filter_t* kf, float measured);


// =========================================================
// 2. 벡터 칼만 필터 (3D)
// =========================================================

/**
 * @struct kalman_filter_vec3_t
 * @brief 3차원 위치 및 속도 추정 칼만 필터
 *
 * ### 기본 초기값:
 * - `position = (0,0,0)`
 * - `velocity = (0,0,0)`
 * - `error_p = (1,1,1)`
 * - `q = 0.01`
 * - `r = 1.0`
 * - `dt = 0.1`
 *
 * ### 표준 범위:
 * - `position`: -1000 ~ +1000 (m 단위)
 * - `velocity`: -50 ~ +50 (m/s 단위)
 * - `q`: 0.0001 ~ 0.1 (프로세스 노이즈)
 * - `r`: 0.01 ~ 10 (측정 노이즈)
 * - `dt`: 0.01 ~ 0.1 (초)
 */
typedef struct s_kalman_filter_vec3 {
    vec3_t position;   /**< 위치 추정 */
    vec3_t velocity;   /**< 속도 추정 */

    vec3_t error_p;    /**< 오차 공분산 (x, y, z) */
    float q;           /**< 프로세스 노이즈 */
    float r;           /**< 측정 노이즈 */
    float dt;          /**< 시간 간격 (초) */
} kalman_filter_vec3_t;

/**
 * @brief 3D 칼만 필터 기본 초기화
 *
 * 기본값:
 * - `position = (0,0,0)`
 * - `velocity = (0,0,0)`
 * - `error_p = (1,1,1)`
 * - `q = 0.01`
 * - `r = 1.0`
 * - `dt = 0.1`
 *
 * @param kf 초기화할 칼만 필터 포인터
 */
BYUL_API void kalman_vec3_init(kalman_filter_vec3_t* kf);

/**
 * @brief 3D 칼만 필터 초기화 (사용자 지정)
 *
 * @param kf 초기화할 필터 포인터
 * @param init_pos 초기 위치 (권장 범위: -1000 ~ +1000)
 * @param init_vel 초기 속도 (권장 범위: -50 ~ +50)
 * @param process_noise 프로세스 노이즈 Q (권장 범위: 0.0001 ~ 0.1)
 * @param measurement_noise 측정 노이즈 R (권장 범위: 0.01 ~ 10)
 * @param dt 예측 스텝 간격 (초 단위, 권장: 0.01 ~ 0.1)
 */
BYUL_API void kalman_vec3_init_full(kalman_filter_vec3_t* kf,
                                    const vec3_t* init_pos,
                                    const vec3_t* init_vel,
                                    float process_noise,
                                    float measurement_noise,
                                    float dt);

/**
 * @brief kalman_filter_vec3_t 복사
 */
BYUL_API void kalman_vec3_assign(kalman_filter_vec3_t* dst,
                           const kalman_filter_vec3_t* src);
                                                               
/**
 * @brief 시간 업데이트 (Time Update)
 *
 * 상태를 `position = position + velocity * dt`로 외삽하고,
 * 오차 공분산을 `P = P + Q`로 증가시킵니다.
 *
 * @param kf 칼만 필터 포인터
 */
BYUL_API void kalman_vec3_time_update(kalman_filter_vec3_t* kf);

/**
 * @brief 측정 업데이트 (Measurement Update)
 *
 * 새로운 측정 위치 `measured_pos`를 사용하여 상태를 보정합니다.
 *
 * @param kf 칼만 필터 포인터
 * @param measured_pos 측정 위치
 */
BYUL_API void kalman_vec3_measurement_update(kalman_filter_vec3_t* kf,
                                             const vec3_t* measured_pos);

/**
 * @brief 미래 위치 예측 (선형 외삽)
 *
 * @param kf 칼만 필터 상태
 * @param future_dt 예측할 시간 (초 단위)
 * @param out_predicted_pos 예측된 위치 벡터
 */
BYUL_API void kalman_vec3_project(const kalman_filter_vec3_t* kf,
                                  float future_dt,
                                  vec3_t* out_predicted_pos);

#ifdef __cplusplus
}
#endif

#endif // NUMEQ_KALMAN_H
