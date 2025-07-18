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
 * @brief 1차 운동 상태 (위치, 속도, 가속도)
 */
typedef struct s_linear_state {
    vec3_t position;      /**< 현재 위치 */
    vec3_t velocity;      /**< 현재 속도 */
    vec3_t acceleration;  /**< 현재 가속도 */
} linear_state_t;

/**
 * @struct attitude_state_t
 * @brief 회전 상태 (자세, 각속도, 각가속도)
 */
typedef struct s_attitude_state {
    quat_t orientation;          /**< 현재 방향 (쿼터니언) */
    vec3_t angular_velocity;     /**< 현재 각속도 */
    vec3_t angular_acceleration; /**< 현재 각가속도 */
} attitude_state_t;

/**
 * @struct motion_state_t
 * @brief 선형 운동 + 회전 운동을 통합한 상태
 */
typedef struct s_motion_state {
    linear_state_t linear;    /**< 선형 운동 */
    attitude_state_t angular; /**< 회전 운동 */
} motion_state_t;

// ---------------------------------------------------------
// 비행 경로 샘플 (시간 + 상태)
// ---------------------------------------------------------

/**
 * @struct trajectory_sample_t
 * @brief 특정 시간의 물리 상태 샘플
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
// 🛠 trajectory 메모리 유틸리티
// ---------------------------------------------------------

/**
 * @brief trajectory를 초기화하고 메모리 할당
 * @param traj 대상 구조체
 * @param capacity 최대 샘플 수
 * @return true 성공 여부
 */
BYUL_API bool trajectory_init(trajectory_t* traj, int capacity);

/**
 * @brief trajectory 내부 메모리 해제
 * @param traj 대상 trajectory
 */
BYUL_API void trajectory_free(trajectory_t* traj);

// ---------------------------------------------------------
// trajectory 샘플 추가
// ---------------------------------------------------------
BYUL_API bool trajectory_add_sample(
    trajectory_t* traj, float t, const motion_state_t* state);

// ---------------------------------------------------------
// trajectory 초기화 (0으로 세팅)
// ---------------------------------------------------------
BYUL_API void trajectory_clear(trajectory_t* traj);


/**
 * @brief trajectory에 저장된 현재 샘플 수를 반환
 * @param traj 대상 trajectory
 * @return 샘플 개수
 */
BYUL_API int trajectory_length(const trajectory_t* traj);

/**
 * @brief trajectory에 저장 가능한 최대 샘플 수를 반환
 * @param traj 대상 trajectory
 * @return capacity
 */
BYUL_API int trajectory_capacity(const trajectory_t* traj);

// 특정 시간의 타겟 위치를 보간
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
 * @return buffer 포인터 (동일한 포인터 반환)
 */
BYUL_API char* trajectory_to_string(const trajectory_t* traj, char* buffer, size_t size);

/**
 * @brief trajectory 내용을 콘솔에 출력
 * @param traj 대상 trajectory
 */
BYUL_API void trajectory_print(const trajectory_t* traj);

/**
 * @brief trajectory의 위치 벡터 리스트 추출
 * @param traj 대상 trajectory
 * @param out_list 결과를 저장할 vec3 배열
 * @param max 최대 추출 개수
 * @return 실제 추출한 개수
 */
BYUL_API int trajectory_get_positions(const trajectory_t* traj, vec3_t* out_list, int max);

/**
 * @brief trajectory의 속력 리스트 추출
 * @param traj 대상 trajectory
 * @param out_list 결과를 저장할 float 배열
 * @param max 최대 추출 개수
 * @return 실제 추출한 개수
 */
BYUL_API int trajectory_get_speeds(const trajectory_t* traj, float* out_list, int max);


#ifdef __cplusplus
}
#endif

#endif // TRAJECTORY_H
