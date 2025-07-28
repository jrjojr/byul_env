/**
 * @file numeq_filters.h
 * @brief 다양한 상태 추정 필터(Kalman, EKF, UKF 등)를 위한 공통 인터페이스
 *
 * 이 모듈은 Kalman Filter를 비롯한 여러 상태 추정 필터를
 * **통일된 인터페이스(filter_interface_t)**를 통해 제어할 수 있도록
 * 어댑터와 함수 포인터 기반 구조를 제공합니다.
 *
 * ---
 *
 * ## 주요 특징
 * - Kalman, EKF, UKF 등 다양한 필터를 **동일한 API**로 다룰 수 있음.
 * - `time_update`, `measurement_update`, `get_state` 세 가지 핵심 콜백 제공.
 * - `filter_state`에 필터별 내부 구조체를 바인딩.
 *
 * ---
 *
 * ## 사용 예시
 *
 * ### 1) Kalman Filter 초기화 및 인터페이스 생성
 * @code
 * kalman_filter_vec3_t kf;
 * vec3_t init_pos = {0,0,0}, init_vel = {0,0,0};
 * kalman_vec3_init_full(&kf, &init_pos, &init_vel, 0.01f, 1.0f, 0.1f);
 *
 * // Kalman 필터를 공통 인터페이스로 변환
 * filter_interface_t kalman_iface = make_kalman_vec3_interface(&kf);
 * @endcode
 *
 * ### 2) 업데이트 루프
 * @code
 * // 1. 시간 업데이트(Time Update)
 * kalman_iface.time_update(kalman_iface.filter_state);
 *
 * // 2. 측정 업데이트(Measurement Update)
 * kalman_iface.measurement_update(kalman_iface.filter_state, &measured_pos, NULL);
 *
 * // 3. 현재 상태 읽기
 * vec3_t pos, vel;
 * kalman_iface.get_state(kalman_iface.filter_state, &pos, &vel);
 * printf("필터링 결과: pos=(%.2f,%.2f,%.2f)\n", pos.x, pos.y, pos.z);
 * @endcode
 *
 * ---
 *
 * @note EKF, UKF 등의 다른 필터도 같은 방식으로 어댑터를 만들어 연결할 수 있습니다.
 */
#ifndef NUMEQ_FILTERS_H
#define NUMEQ_FILTERS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "byul_common.h"
#include "numeq_kalman.h"  // Kalman 기본 구현 포함

// =========================================================
// 필터 공통 인터페이스 정의
// =========================================================

/**
 * @typedef filter_time_update_func
 * @brief 시간 업데이트(Time Update) 함수 포인터
 */
typedef void (*filter_time_update_func)(void* filter);

/**
 * @typedef filter_measurement_update_func
 * @brief 측정 업데이트(Measurement Update) 함수 포인터
 */
typedef void (*filter_measurement_update_func)(void* filter,
                                               const vec3_t* measured_pos,
                                               const vec3_t* measured_vel);

/**
 * @typedef filter_get_state_func
 * @brief 필터의 현재 상태(위치, 속도) 반환 함수 포인터
 */
typedef void (*filter_get_state_func)(void* filter,
                                      vec3_t* out_pos,
                                      vec3_t* out_vel);

/**
 * @struct filter_interface_t
 * @brief 모든 필터를 통합 제어하기 위한 공통 인터페이스
 *
 * `filter_state`는 내부 필터 구조체(Kalman, EKF 등)를 가리키며,
 * 함수 포인터를 통해 시간 업데이트와 측정 업데이트를 호출할 수 있습니다.
 */
typedef struct s_filter_interface {
    void* filter_state;                       /**< 필터 상태 구조체 포인터 */
    filter_time_update_func time_update;      /**< 시간 업데이트 */
    filter_measurement_update_func measurement_update; /**< 측정 업데이트 */
    filter_get_state_func get_state;          /**< 현재 상태 반환 */
} filter_interface_t;

// =========================================================
// Kalman Filter 기본 어댑터
// =========================================================

/**
 * @brief kalman_filter_vec3_t → filter_interface_t 어댑터 생성
 * @param kf 칼만 필터 포인터
 * @return 필터 인터페이스 구조체
 */
BYUL_API filter_interface_t make_kalman_vec3_interface(
    kalman_filter_vec3_t* kf);

#ifdef __cplusplus
}
#endif
#endif // NUMEQ_FILTERS_H
