#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <stdbool.h>
#include "internal/numal.h"
#include "byul_config.h"

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
BYUL_API void linear_state_copy(
    linear_state_t* out, const linear_state_t* src);

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
BYUL_API void attitude_state_copy(
    attitude_state_t* out, const attitude_state_t* src);

/**
 * @struct motion_state_t
 * @brief 선형 운동 + 회전 운동을 통합한 상태 구조체
 */
typedef struct s_motion_state {
    linear_state_t linear;    /**< 선형 운동 */
    attitude_state_t angular; /**< 회전 운동 */
} motion_state_t;

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
BYUL_API void motion_state_copy(
    motion_state_t* out, const motion_state_t* src);

// ---------------------------------------------------------
// 비행 경로 샘플 (시간 + 상태)
// ---------------------------------------------------------

/**
 * @struct trajectory_sample_t
 * @brief 특정 시간의 물리 상태 샘플을 표현하는 구조체
 */
typedef struct s_trajectory_sample {
    float t;                 /**< 시간 (초 단위) */
    motion_state_t state;    /**< 해당 시점의 운동 상태 */
} trajectory_sample_t;

/**
 * @struct trajectory_t
 * @brief 시간 순서로 예측된 경로(trajectory) 데이터
 */
typedef struct s_trajectory {
    trajectory_sample_t* samples; /**< 예측된 경로 샘플 배열 */
    int count;                    /**< 유효한 샘플 수 */
    int capacity;                 /**< 할당된 샘플 수 */
} trajectory_t;

// ---------------------------------------------------------
// trajectory 메모리 관리 유틸리티
// ---------------------------------------------------------

/**
 * @brief trajectory를 초기화하고 메모리 할당
 * @param traj 대상 구조체
 * @param capacity 최대 샘플 수
 * @return true 초기화 성공 여부
 */
BYUL_API bool trajectory_init(trajectory_t* traj, int capacity);

/**
 * @brief trajectory 내부 메모리를 해제
 * @param traj 대상 trajectory
 */
BYUL_API void trajectory_free(trajectory_t* traj);

/**
 * @brief trajectory 내부 데이터를 모두 지움
 * @param traj 대상 trajectory
 */
BYUL_API void trajectory_clear(trajectory_t* traj);

/**
 * @brief trajectory에 샘플을 추가
 * @param traj 대상 trajectory
 * @param t 시간 값
 * @param state 운동 상태
 * @return 추가 성공 여부
 */
BYUL_API bool trajectory_add_sample(
    trajectory_t* traj, float t, const motion_state_t* state);

/**
 * @brief trajectory에 저장된 샘플 개수 반환
 * @param traj 대상 trajectory
 * @return 샘플 개수
 */
BYUL_API int trajectory_length(const trajectory_t* traj);

/**
 * @brief trajectory의 최대 capacity 반환
 * @param traj 대상 trajectory
 * @return capacity
 */
BYUL_API int trajectory_capacity(const trajectory_t* traj);

/**
 * @brief 특정 시간 t에서 위치를 보간
 * @param traj 대상 trajectory
 * @param t 시간 값
 * @param out_pos 보간 결과 위치 벡터
 * @return 보간 성공 여부
 */
BYUL_API bool trajectory_sample_position(
    const trajectory_t* traj, float t, vec3_t* out_pos);

// ---------------------------------------------------------
// trajectory 출력 유틸리티
// ---------------------------------------------------------

/**
 * @brief trajectory를 문자열로 변환
 * @param traj 대상 trajectory
 * @param buffer 결과를 담을 버퍼
 * @param size 버퍼 크기
 * @return buffer 포인터
 */
BYUL_API char* trajectory_to_string(
    const trajectory_t* traj, char* buffer, size_t size);

/**
 * @brief trajectory 내용을 콘솔에 출력
 * @param traj 대상 trajectory
 */
BYUL_API void trajectory_print(const trajectory_t* traj);

/**
 * @brief trajectory의 위치 리스트 추출
 * @param traj 대상 trajectory
 * @param out_list 결과를 저장할 vec3 배열
 * @param max 최대 추출 개수
 * @return 실제 추출한 개수
 */
BYUL_API int trajectory_get_positions(
    const trajectory_t* traj, vec3_t* out_list, int max);

/**
 * @brief trajectory의 속력 리스트 추출
 * @param traj 대상 trajectory
 * @param out_list 결과를 저장할 float 배열
 * @param max 최대 추출 개수
 * @return 실제 추출한 개수
 */
BYUL_API int trajectory_get_speeds(
    const trajectory_t* traj, float* out_list, int max);

#ifdef __cplusplus
}
#endif

#endif // TRAJECTORY_H
