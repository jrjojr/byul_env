#ifndef MOTION_STATE_H
#define MOTION_STATE_H

#include <stdbool.h>
#include "internal/numal.h"
#include "byul_common.h"

#ifdef __cplusplus
extern "C" {
#endif

// ---------------------------------------------------------
// 운동 상태 구조체 (선형 + 회전)
// ---------------------------------------------------------

/**
 * @struct linear_state_t
 * @brief 선형(위치 기반) 운동 상태를 나타내는 구조체
 */
typedef struct s_linear_state {
    vec3_t position;      /**< 현재 위치 */
    vec3_t velocity;      /**< 현재 속도 */
    vec3_t acceleration;  /**< 현재 가속도 */
} linear_state_t;

/**
 * @struct attitude_state_t
 * @brief 회전(자세) 운동 상태를 나타내는 구조체
 */
typedef struct s_attitude_state {
    quat_t orientation;          /**< 현재 방향 (쿼터니언) */
    vec3_t angular_velocity;     /**< 현재 각속도 */
    vec3_t angular_acceleration; /**< 현재 각가속도 */
} attitude_state_t;

/**
 * @struct motion_state_t
 * @brief 선형 운동 + 회전 운동을 통합한 상태 구조체
 */
typedef struct s_motion_state {
    linear_state_t linear;    /**< 선형 운동 */
    attitude_state_t angular; /**< 회전 운동 */
} motion_state_t;

/**
 * @brief linear_state_t를 0으로 초기화
 * @param out 초기화할 구조체 포인터
 */
BYUL_API void linear_state_init(linear_state_t* out);

/**
 * @brief linear_state_t를 지정한 값으로 초기화
 * @param out 초기화할 구조체 포인터
 * @param position 위치 벡터
 * @param velocity 속도 벡터
 * @param acceleration 가속도 벡터
 */
BYUL_API void linear_state_init_full(linear_state_t* out,
                                     const vec3_t* position,
                                     const vec3_t* velocity,
                                     const vec3_t* acceleration);

/**
 * @brief linear_state_t 복사
 * @param out 복사 대상 구조체
 * @param src 복사할 원본 구조체
 */
BYUL_API void linear_state_assign(
    linear_state_t* out, const linear_state_t* src);

/**
 * @brief attitude_state_t를 기본값으로 초기화 (단위 쿼터니언)
 * @param out 초기화할 구조체 포인터
 */
BYUL_API void attitude_state_init(attitude_state_t* out);

/**
 * @brief attitude_state_t를 지정한 값으로 초기화
 * @param out 초기화할 구조체 포인터
 * @param orientation 쿼터니언 방향
 * @param angular_velocity 각속도 벡터
 * @param angular_acceleration 각가속도 벡터
 */
BYUL_API void attitude_state_init_full(attitude_state_t* out,
                                       const quat_t* orientation,
                                       const vec3_t* angular_velocity,
                                       const vec3_t* angular_acceleration);

/**
 * @brief attitude_state_t 복사
 * @param out 복사 대상 구조체
 * @param src 복사할 원본 구조체
 */
BYUL_API void attitude_state_assign(
    attitude_state_t* out, const attitude_state_t* src);

/**
 * @brief motion_state_t를 기본값으로 초기화
 * @param out 초기화할 구조체 포인터
 */
BYUL_API void motion_state_init(motion_state_t* out);

/**
 * @brief motion_state_t를 지정한 값으로 초기화
 * @param out 초기화할 구조체 포인터
 * @param position 위치 벡터
 * @param velocity 속도 벡터
 * @param acceleration 가속도 벡터
 * @param orientation 쿼터니언 방향
 * @param angular_velocity 각속도 벡터
 * @param angular_acceleration 각가속도 벡터
 */
BYUL_API void motion_state_init_full(motion_state_t* out,
                                     const vec3_t* position,
                                     const vec3_t* velocity,
                                     const vec3_t* acceleration,
                                     const quat_t* orientation,
                                     const vec3_t* angular_velocity,
                                     const vec3_t* angular_acceleration);

/**
 * @brief motion_state_t 복사
 * @param out 복사 대상 구조체
 * @param src 복사할 원본 구조체
 */
BYUL_API void motion_state_assign(
    motion_state_t* out, const motion_state_t* src);

/**
 * @brief 두 motion_state_t의 충돌 여부와 시간을 예측합니다.
 *
 * 두 개체를 구체로 가정하여 (반지름 rA, rB),
 * 상대 운동 방정식을 이용해 t>0에서의 충돌 시간을 계산합니다.
 *
 * @param a        첫 번째 상태
 * @param b        두 번째 상태
 * @param radius_a 첫 번째 반지름
 * @param radius_b 두 번째 반지름
 * @param max_t    최대 예측 시간 (초). 이 시간 범위 내에서만 충돌을 탐색합니다.
 * @param out_t    충돌 시점(초). 충돌이 없으면 -1을 반환.
 * @return true    max_t 내에서 충돌 발생
 * @return false   충돌 없음
 */
BYUL_API bool motion_state_predict_collision_time(
    const motion_state_t* a,
    const motion_state_t* b,
    float radius_a,
    float radius_b,
    float max_t,
    float* out_t);

/**
 * @brief 두 motion_state_t의 충돌 지점을 예측합니다.
 *
 * 충돌 시간(t)을 먼저 계산한 후, 해당 t에서의 위치를 계산하여 충돌 좌표를 반환합니다.
 * 
 * @param a        첫 번째 상태
 * @param b        두 번째 상태
 * @param radius_a 첫 번째 반지름
 * @param radius_b 두 번째 반지름
 * @param max_t    최대 예측 시간 (초)
 * @param out_t    충돌 시점 (초)
 * @param out_pos  충돌 지점 (NULL이면 무시)
 * @return true    충돌 지점 계산 성공
 * @return false   충돌 없음
 */
BYUL_API bool motion_state_predict_collision_point(
    const motion_state_t* a,
    const motion_state_t* b,
    float radius_a,
    float radius_b,
    float max_t,
    float* out_t,
    vec3_t* out_pos);



#ifdef __cplusplus
}
#endif

#endif // MOTION_STATE_H
