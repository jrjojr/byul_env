#ifndef NUMEQ_COMMON_H
#define NUMEQ_COMMON_H

#include <stdbool.h>
#include "internal/vec3.h"
#include "internal/dualnumber.h"
#include "internal/dualquat.h"
#include "internal/common.h"
#include "byul_config.h"

#ifdef __cplusplus
extern "C" {
#endif

// ---------------------------------------------------------
// 상태 벡터 (위치 + 속도 + 가속도)
// ---------------------------------------------------------

/**
 * @struct state_vector_t
 * @brief 1차 운동 상태를 나타내는 벡터 (위치, 속도, 가속도 포함)
 */
typedef struct s_state_vector {
    vec3_t position;      /**< 현재 위치 */
    vec3_t velocity;      /**< 현재 속도 */
    vec3_t acceleration;  /**< 현재 가속도 */
} state_vector_t;

// ---------------------------------------------------------
// 비행 경로 샘플 (시간 + 상태)
// ---------------------------------------------------------

/**
 * @struct trajectory_sample_t
 * @brief 특정 시간에 대한 상태 벡터 샘플
 */
typedef struct s_trajectory_sample {
    float t;               /**< 시간 (초 단위) */
    state_vector_t state;  /**< 그 시점의 물리 상태 */
} trajectory_sample_t;

#ifdef __cplusplus
}
#endif

#endif // NUMEQ_COMMON_H
