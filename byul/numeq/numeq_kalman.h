#ifndef NUMEQ_KALMAN_H
#define NUMEQ_KALMAN_H

#include "internal/numeq_common.h"

#ifdef __cplusplus
extern "C" {
#endif

// ---------------------------------------------------------
// 1. 스칼라 Kalman 필터 (1차원)
// ---------------------------------------------------------

/**
 * @brief 1차원 칼만 필터 (단일 센서값 추정)
 */
typedef struct s_kalman_filter {
    float x;   /**< 상태 추정값 */
    float p;   /**< 오차 공분산 */
    float q;   /**< 프로세스 노이즈 */
    float r;   /**< 측정 노이즈 */
    float k;   /**< 칼만 이득 */
} kalman_filter_t;

/**
 * @brief 스칼라 필터 초기화
 */
BYUL_API void kalman_reset(kalman_filter_t* kf, float init_x, float init_p,
                  float process_noise, float measurement_noise);

/**
 * @brief 예측 단계 (상태 자체를 외부에서 조정할 경우 사용)
 */
BYUL_API void kalman_predict(kalman_filter_t* kf);

/**
 * @brief 보정 단계 (측정값 적용)
 */
BYUL_API float kalman_update(kalman_filter_t* kf, float measured);


// ---------------------------------------------------------
// 2. 벡터 Kalman 필터 (vec3 + 속도 예측)
// ---------------------------------------------------------

/**
 * @brief 3차원 위치 + 속도 추정을 위한 칼만 필터
 */
typedef struct s_kalman_filter_vec3 {
    vec3_t position;   /**< 위치 추정 */
    vec3_t velocity;   /**< 속도 추정 */

    vec3_t error_p;    /**< 상태 오차 공분산 (x, y, z) */
    float q;           /**< 프로세스 노이즈 공통값 */
    float r;           /**< 측정 노이즈 공통값 */

    float dt;          /**< 시간 간격 */
} kalman_filter_vec3_t;

/**
 * @brief 벡터 필터 초기화
 */
BYUL_API void kalman_vec3_reset(kalman_filter_vec3_t* kf,
                       const vec3_t* init_pos,
                       const vec3_t* init_vel,
                       float process_noise,
                       float measurement_noise,
                       float dt);

/**
 * @brief 벡터 필터 예측 (위치, 속도 외삽)
 */
BYUL_API void kalman_vec3_predict(kalman_filter_vec3_t* kf);

/**
 * @brief 벡터 필터 보정 (측정된 위치를 기준으로 오차 반영)
 */
BYUL_API void kalman_vec3_update(kalman_filter_vec3_t* kf,
                        const vec3_t* measured_pos);

/**
 * @brief 추정된 미래 위치 예측 (현재 위치 + 속도 * dt)
 */
BYUL_API void kalman_vec3_project(const kalman_filter_vec3_t* kf,
                         float future_dt,
                         vec3_t* out_predicted_pos);

/**
 * @brief 현재 상태 복사
 */
BYUL_API void kalman_vec3_copy(kalman_filter_vec3_t* dst,
                      const kalman_filter_vec3_t* src);

#ifdef __cplusplus
}
#endif

#endif // NUMEQ_KALMAN_H
