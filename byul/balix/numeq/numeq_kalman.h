/**
 * @file numeq_kalman.h
 * @brief 칼만 필터 (Kalman Filter) 모듈
 *
 * 이 모듈은 1차원 스칼라 및 3차원 벡터 상태(위치, 속도)를 추정하기 위해
 * **칼만 필터** 알고리즘을 제공합니다.
 * 센서로부터 얻은 노이즈가 있는 측정값을 이용하여 시스템의 상태를
 * 보다 정확하게 추정할 수 있습니다.
 *
 * ---
 *
 * ## 개요 (Kalman Filter 원리)
 * 칼만 필터는 **예측(Predict)** → **보정(Update)** 단계를 반복하면서
 * 현재 상태에 대한 추정값을 점진적으로 개선합니다.
 *
 * ### 예측 단계 (Predict)
 * - 이전 상태와 시스템 모델을 사용하여 다음 상태를 예측합니다.
 * - 불확실성(공분산 P)을 프로세스 노이즈 Q만큼 증가시킵니다.
 *   \f$ x' = x \f$  
 *   \f$ P' = P + Q \f$
 *
 * ### 보정 단계 (Update)
 * - 새로운 측정값 z가 도착하면 예측값과 비교하여 상태를 보정합니다.
 * - 칼만 이득 K는 예측값과 측정값을 가중 평균하는 비율입니다.
 *   \f$ K = P' / (P' + R) \f$  
 *   \f$ x = x' + K (z - x') \f$  
 *   \f$ P = (1 - K) P' \f$
 *
 * ---
 *
 * ## 대표적인 사용법
 *
 * ### 1차원 필터
 * @code
 * kalman_filter_t kf;
 * kalman_init_full(&kf, 0.0f, 1.0f, 0.01f, 1.0f);
 *
 * while (running) {
 *     kalman_predict(&kf);                // 상태 예측
 *     float filtered = kalman_update(&kf, sensor_value); // 측정값 보정
 *     printf("필터링 결과: %.2f\n", filtered);
 * }
 * @endcode
 *
 * ### 3차원 벡터 필터
 * @code
 * kalman_filter_vec3_t kf;
 * vec3_t init_pos = {0,0,0}, init_vel = {0,0,0};
 * kalman_vec3_init_full(&kf, &init_pos, &init_vel, 0.01f, 1.0f, 0.1f);
 *
 * while (running) {
 *     kalman_vec3_predict(&kf);                  // 예측
 *     kalman_vec3_update(&kf, &measured_pos);    // 보정
 * }
 *
 * vec3_t future_pos;
 * kalman_vec3_project(&kf, 0.5f, &future_pos);  // 0.5초 후 위치 예측
 * @endcode
 *
 * ---
 *
 * @note
 * - `predict()`는 루프 내에서 주기적으로 실행하여 불확실성을 추적합니다.
 * - `update()`는 센서 측정값이 들어올 때마다 호출하여 예측값을 보정합니다.
 * - `project()`는 미래 위치를 예측할 때 필요 시점에서만 호출합니다.
 */

#ifndef NUMEQ_KALMAN_H
#define NUMEQ_KALMAN_H

#include "internal/trajectory.h"

#ifdef __cplusplus
extern "C" {
#endif

// ---------------------------------------------------------
// 1. 스칼라 Kalman 필터 (1차원)
// ---------------------------------------------------------

/**
 * @struct kalman_filter_t
 * @brief 1차원 칼만 필터 (단일 센서값 추정)
 *
 * 칼만 필터(Kalman Filter)는 **노이즈가 있는 측정값**을 바탕으로 시스템의
 * **상태(state)**를 추정하기 위해 사용되는 재귀적 필터입니다.
 *
 * 1차원 칼만 필터는 단일 센서로부터 들어오는 값의 평균화 및 노이즈 제거에 적합합니다.
 * 내부적으로 예측(Predict) → 보정(Update) 단계를 반복하여 상태를 갱신합니다.
 *
 * ### 기본 동작 원리:
 * - **예측 단계 (Predict):**  
 *   기존 상태 추정값과 시스템 모델을 통해 다음 상태를 예측합니다.
 *   \f$ x' = x \f$, \f$ P' = P + Q \f$
 *   여기서 Q는 프로세스 노이즈로, 시스템 모델의 불확실성을 나타냅니다.
 *
 * - **보정 단계 (Update):**  
 *   새로운 측정값 z를 이용해 추정값을 보정합니다.
 *   칼만 이득 K는 측정값과 예측값을 얼마나 신뢰할지를 결정합니다.
 *   \f$ K = P' / (P' + R) \f$,  
 *   \f$ x = x' + K (z - x') \f$,  
 *   \f$ P = (1 - K) P' \f$
 *
 * ### 구조체 변수 설명 및 평균적 범위:
 * - `x` : 상태 추정값 (측정 대상 값).  
 *         일반적으로 -1000 ~ +1000 범위에서 사용되며, 센서 측정값 범위를 따름.
 * - `p` : 오차 공분산 (불확실성 정도).  
 *         0.0 이상, 보통 0.1 ~ 100 사이 값.
 * - `q` : 프로세스 노이즈 (시스템 모델 불확실성).  
 *         0.0001 ~ 0.1 권장.
 * - `r` : 측정 노이즈 (센서 측정값의 불확실성).  
 *         0.01 ~ 10.0 권장 (센서 특성에 따라 조정).
 * - `k` : 칼만 이득 (자동 계산되며 0.0 ~ 1.0 범위).
 *
 * ### 기본값:
 * - x = 0
 * - p = 1
 * - q = 0.01
 * - r = 1
 * - k = 0
 */
typedef struct s_kalman_filter {
    float x;   /**< 상태 추정값 */
    float p;   /**< 오차 공분산 */
    float q;   /**< 프로세스 노이즈 */
    float r;   /**< 측정 노이즈 */
    float k;   /**< 칼만 이득 */
} kalman_filter_t;

// ---------------------------------------------------------
// 1. 스칼라 Kalman 필터 (1차원)
// ---------------------------------------------------------

/**
 * @brief kalman_filter_t 기본값 초기화
 *
 * 기본값:
 * - x = 0
 * - p = 1
 * - q = 0.01
 * - r = 1
 * - k = 0
 *
 * @param kf 초기화할 필터 구조체
 */
BYUL_API void kalman_init(kalman_filter_t* kf);

/**
 * @brief kalman_filter_t 지정값 초기화
 *
 * 프로세스 노이즈(Q)와 측정 노이즈(R) 값을 지정하여 초기화할 수 있습니다.
 * 상태값(x)와 오차 공분산(p)도 초기값으로 설정됩니다.
 *
 * **기본값 예시:**
 * - init_x = 0.0f
 * - init_p = 1.0f
 * - process_noise = 0.01f
 * - measurement_noise = 1.0f
 *
 * @param kf 필터 포인터
 * @param init_x 초기 상태값
 * @param init_p 초기 오차 공분산
 * @param process_noise 프로세스 노이즈 (q)
 * @param measurement_noise 측정 노이즈 (r)
 */
BYUL_API void kalman_init_full(kalman_filter_t* kf, float init_x, float init_p,
                      float process_noise, float measurement_noise);

/**
 * @brief kalman_filter_t 복사
 */
BYUL_API void kalman_assign(kalman_filter_t* dst, const kalman_filter_t* src);

/**
 * @brief 스칼라 필터 초기화
 */
BYUL_API void kalman_reset(kalman_filter_t* kf, float init_x, float init_p,
                  float process_noise, float measurement_noise);

/**
 * @brief 예측 단계 (Predict)
 *
 * 칼만 필터의 예측 단계는 이전 상태 추정값을 기반으로 다음 상태를 예측합니다.
 * 이 함수는 상태값 `x`는 변경하지 않으며, 오차 공분산 `p`를 증가시켜
 * 시간 경과에 따른 불확실성을 반영합니다.
 *
 * ### 동작 원리:
 * - \f$ P' = P + Q \f$
 *   - `P`는 현재 오차 공분산
 *   - `Q`는 프로세스 노이즈 (시스템 모델 불확실성)
 *
 * @param kf 칼만 필터 구조체 포인터
 *
 * @note 이 함수는 측정값을 사용하지 않습니다. 센서 데이터가 도착하기 전,
 *       상태 예측만 진행할 때 호출됩니다.
 *
 * @code
 * kalman_filter_t kf;
 * kalman_init(&kf);
 * kalman_predict(&kf);
 * @endcode
 */
BYUL_API void kalman_predict(kalman_filter_t* kf);

/**
 * @brief 보정 단계 (Update)
 *
 * 새로운 측정값을 받아 이전 예측값을 보정하여 보다 정확한 상태 추정값을 계산합니다.
 * 칼만 이득 `K`를 사용해 예측값과 측정값을 가중 평균하여 노이즈를 줄입니다.
 *
 * ### 동작 원리:
 * - \f$ K = P' / (P' + R) \f$
 * - \f$ X = X' + K \cdot (Z - X') \f$
 * - \f$ P = (1 - K) \cdot P' \f$
 *   - `X'`는 예측된 상태값
 *   - `Z`는 새로운 측정값
 *   - `R`은 측정 노이즈
 *
 * @param kf 칼만 필터 구조체 포인터
 * @param measured 새로운 측정값
 * @return 보정된 상태 추정값 (x)
 *
 * @code
 * kalman_filter_t kf;
 * kalman_init_full(&kf, 0.0f, 1.0f, 0.01f, 1.0f);
 * kalman_predict(&kf);
 * float filtered = kalman_update(&kf, sensor_value);
 * @endcode
 */
BYUL_API float kalman_update(kalman_filter_t* kf, float measured);


// ---------------------------------------------------------
// 2. 벡터 Kalman 필터 (vec3 + 속도 예측)
// ---------------------------------------------------------

/**
 * @struct kalman_filter_vec3_t
 * @brief 3차원 위치 + 속도 추정을 위한 칼만 필터
 *
 * 이 필터는 3차원 위치(vec3_t)와 속도(vec3_t)를 동시에 추정하기 위해 사용됩니다.
 * 각 좌표축(x, y, z)에 대해 칼만 필터의 예측(Predict) 및 보정(Update) 단계를 수행합니다.
 *
 * ### 변수 범위 및 기본값:
 * - `position` : 추정 위치 (보통 -1000~+1000 m 범위)
 * - `velocity` : 추정 속도 (보통 -50~+50 m/s 범위)
 * - `error_p` : 상태 오차 공분산 (기본값 1.0, 보통 0~100)
 * - `q` : 프로세스 노이즈 (기본값 0.01, 권장 0.001~0.1)
 * - `r` : 측정 노이즈 (기본값 1.0, 권장 0.1~10.0)
 * - `dt` : 시간 간격 (기본값 0.1s, 보통 0.01~0.1)
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
 * @brief kalman_filter_vec3_t 기본값 초기화
 *
 * 기본값:
 * - position = (0,0,0)
 * - velocity = (0,0,0)
 * - error_p = (1,1,1)
 * - q = 0.01
 * - r = 1
 * - dt = 0.1
 */
BYUL_API void kalman_vec3_init(kalman_filter_vec3_t* kf);

/**
 * @brief kalman_filter_vec3_t 지정값 초기화
 *
 * 사용자가 초기 위치, 속도, 노이즈 및 시간 간격을 지정하여 필터를 초기화합니다.
 *
 * **기본값 예시:**
 * - process_noise = 0.01f
 * - measurement_noise = 1.0f
 * - dt = 0.1f
 *
 * @param kf 초기화할 칼만 필터 구조체
 * @param init_pos 초기 위치 (vec3)
 * @param init_vel 초기 속도 (vec3)
 * @param process_noise 프로세스 노이즈 (q)
 * @param measurement_noise 측정 노이즈 (r)
 * @param dt 예측 스텝 간격 (초 단위)
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
 * @brief 벡터 필터 초기화
 */
BYUL_API void kalman_vec3_reset(kalman_filter_vec3_t* kf,
                       const vec3_t* init_pos,
                       const vec3_t* init_vel,
                       float process_noise,
                       float measurement_noise,
                       float dt);

/**
 * @brief 벡터 필터 예측 (Predict 단계: 위치, 속도 외삽)
 *
 * 3차원 칼만 필터에서 예측 단계는 현재 추정된 위치와 속도를 기반으로
 * 다음 시점의 위치를 외삽(prediction)합니다.
 *
 * ### 동작 원리:
 * - 위치 예측: \f$ \mathbf{x}' = \mathbf{x} + \mathbf{v} \cdot \Delta t \f$
 * - 속도는 외부 가속도 모델이 없는 경우 그대로 유지됩니다.
 * - 오차 공분산: \f$ \mathbf{P}' = \mathbf{P} + Q \f$
 *
 * @param kf 예측을 수행할 kalman_filter_vec3_t 구조체 포인터
 *
 * @note 센서 데이터가 아직 도착하지 않았을 때, 시간 진행에 따른
 *       위치 변화를 예측하는 용도로 사용됩니다.
 *
 * @code
 * kalman_filter_vec3_t kf;
 * kalman_vec3_init(&kf);
 * kalman_vec3_predict(&kf);
 * @endcode
 */
BYUL_API void kalman_vec3_predict(kalman_filter_vec3_t* kf);

/**
 * @brief 벡터 필터 보정 (Update 단계: 측정된 위치를 반영)
 *
 * 예측된 위치와 새로운 측정값(measured_pos)을 비교하여
 * 3차원 위치 추정값과 속도를 보정합니다.
 * 각 축(x, y, z)에 대해 칼만 이득 K를 계산하고 예측값을 보정합니다.
 *
 * ### 동작 원리:
 * - 칼만 이득: \f$ K = P' / (P' + R) \f$
 * - 위치 보정: \f$ \mathbf{x} = \mathbf{x}' + K \cdot (\mathbf{z} - \mathbf{x}') \f$
 * - 공분산 갱신: \f$ P = (1 - K) P' \f$
 * - 속도는 보정된 위치 변화량을 기반으로 갱신될 수 있습니다.
 *
 * @param kf 보정을 수행할 kalman_filter_vec3_t 구조체 포인터
 * @param measured_pos 새로 측정된 위치 벡터 (vec3_t)
 * @return 없음
 *
 * @code
 * vec3_t measurement = {1.2f, 0.0f, -0.5f};
 * kalman_vec3_update(&kf, &measurement);
 * @endcode
 */
BYUL_API void kalman_vec3_update(kalman_filter_vec3_t* kf,
                        const vec3_t* measured_pos);

/**
 * @brief 추정된 미래 위치 예측 (현재 위치 + 속도 * future_dt)
 *
 * 현재 추정된 위치 `position`과 속도 `velocity`를 사용해,
 * 미래 `future_dt` 초 후의 위치를 선형 외삽(linear extrapolation) 방식으로 계산합니다.
 *
 * ### 동작 원리:
 * - \f$ \mathbf{x}_{future} = \mathbf{x}_{current} + \mathbf{v}_{current} \cdot \Delta t \f$
 *   - \f$ \mathbf{x}_{current} \f$ : 현재 위치 추정값 (kf->position)
 *   - \f$ \mathbf{v}_{current} \f$ : 현재 속도 추정값 (kf->velocity)
 *   - \f$ \Delta t \f$ : 미래 예측 시간 (future_dt)
 *
 * @param kf 현재 칼만 필터 상태 (위치 및 속도 정보 포함)
 * @param future_dt 예측할 시간 (초 단위)
 * @param out_predicted_pos 예측된 위치가 저장될 vec3_t 포인터
 *
 * @note 이 함수는 보정(Update) 단계 없이 단순 외삽을 수행합니다.
 *       예측값은 노이즈나 외부 가속도를 고려하지 않으므로 긴 future_dt에 대해
 *       정확도가 떨어질 수 있습니다.
 *
 * @code
 * vec3_t predicted;
 * kalman_vec3_project(&kf, 0.5f, &predicted); // 0.5초 후 위치 예측
 * @endcode
 */
BYUL_API void kalman_vec3_project(const kalman_filter_vec3_t* kf,
                         float future_dt,
                         vec3_t* out_predicted_pos);

#ifdef __cplusplus
}
#endif

#endif // NUMEQ_KALMAN_H
