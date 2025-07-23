#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <stdbool.h>
#include "internal/numal.h"
#include "internal/motion_state.h"
#include "byul_common.h"

#ifdef __cplusplus
extern "C" {
#endif

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
 * @brief 새로운 trajectory를 생성하고 메모리를 할당합니다.
 *
 * trajectory_t 구조체와 내부 samples 배열을 동적 할당하며,
 * count를 0으로 초기화합니다.
 *
 * @param capacity trajectory가 저장할 수 있는 최대 샘플 개수
 * @return 성공 시 할당된 trajectory_t* 포인터, 실패 시 nullptr
 *
 * @note 사용 후 반드시 trajectory_destroy()를 호출하여 
 * 구조체와 내부 메모리를 해제해야 합니다.
 */
BYUL_API trajectory_t* trajectory_create_full(int capacity);

/**
 * @brief 기본 용량(100 샘플)을 갖는 trajectory를 생성합니다.
 *
 * @return capacity = 100으로 할당된 trajectory_t* 포인터
 *
 * @note 사용 후 반드시 trajectory_destroy()를 호출하여 
 * 구조체와 내부 메모리를 해제해야 합니다.
 */
BYUL_API trajectory_t* trajectory_create();

/**
 * @brief trajectory_t를 기본 용량(100)으로 초기화
 *
 * @param traj 초기화할 trajectory_t 포인터
 *
 * @note 기존 samples 배열이 존재하면 삭제 후 재할당합니다.
 */
BYUL_API void trajectory_init(trajectory_t* traj);

/**
 * @brief trajectory_t를 지정한 capacity로 초기화
 *
 * @param traj 초기화할 trajectory_t 포인터
 * @param capacity 샘플 배열의 최대 용량 (capacity > 0)
 *
 * @note 기존 samples 배열이 존재하면 삭제 후 재할당합니다.
 */
BYUL_API void trajectory_init_full(trajectory_t* traj, int capacity);

/**
 * @brief trajectory 내부 메모리를 해제합니다.
 *
 * trajectory_t 구조체 내의 samples 배열을 해제하고,
 * count와 capacity를 0으로 초기화합니다.
 *
 * @param traj 해제할 trajectory_t 포인터 (NULL 가능)
 *
 * @warning traj 자체는 해제하지 않습니다.
 *          trajectory_create_full()로 동적 생성한 구조체 전체를 해제하려면
 *  trajectory_destroy()를 사용하세요.
 */
BYUL_API void trajectory_free(trajectory_t* traj);

/**
 * @brief trajectory 구조체와 내부 메모리를 모두 해제합니다.
 *
 * trajectory_free()를 호출한 뒤 구조체 포인터 자체도 해제합니다.
 *
 * @param traj 해제할 trajectory_t 포인터 (NULL 가능)
 */
BYUL_API void trajectory_destroy(trajectory_t* traj);

/**
 * @brief trajectory 내용을 깊은 복사합니다.
 *
 * src의 샘플 데이터를 out에 깊은 복사합니다.
 * 필요 시 out->samples 메모리를 재할당합니다.
 *
 * @param out 대상 trajectory (이미 유효한 포인터여야 함)
 * @param src 원본 trajectory
 *
 * @note out->capacity < src->count인 경우 내부 메모리를 재할당합니다.
 */
BYUL_API void trajectory_assign(trajectory_t* out, const trajectory_t* src);

/**
 * @brief trajectory를 복제(클론)하여 새 인스턴스를 생성합니다.
 *
 * src 내용을 깊은 복사한 새로운 trajectory를 생성하여 반환합니다.
 *
 * @param src 복제할 원본 trajectory
 * @return 새로운 trajectory_t* (동적 할당), 실패 시 nullptr
 *
 * @note 사용 후 반드시 trajectory_destroy()를 호출하여 메모리를 해제해야 합니다.
 */
BYUL_API trajectory_t* trajectory_copy(const trajectory_t* src);

/**
 * @brief trajectory 내부 데이터를 모두 제거합니다.
 *
 * count 값을 0으로 초기화하여 모든 샘플을 제거합니다.
 * capacity 및 samples는 그대로 유지되므로 재사용이 가능합니다.
 *
 * @param traj 초기화할 trajectory 포인터 (NULL 가능)
 */
BYUL_API void trajectory_clear(trajectory_t* traj);

/**
 * @brief trajectory_t의 용량(capacity)을 새 크기로 조정합니다.
 *
 * @param traj     리사이즈할 trajectory_t 포인터
 * @param new_cap  새 용량 (new_cap > 0)
 *
 * @note 기존 samples 데이터를 보존하며, count가 new_cap을 초과하면
 *       count는 new_cap으로 잘립니다.
 *       메모리 재할당 후 기존 포인터는 delete[]로 안전하게 해제됩니다.
 */
BYUL_API void trajectory_resize(trajectory_t* traj, int new_cap);

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
 * @brief 주어진 시간 t에서 trajectory 상의 위치를 보간(interpolate)하여 계산합니다.
 *
 * 이 함수는 trajectory_t에 저장된 샘플들을 기반으로,
 * 시간 @p t 가 샘플 시간 범위 내에 있다면 
 * 두 인접 샘플 사이를 선형 보간(lerp)하여 위치를 계산합니다.
 * - @p t 가 첫 샘플 시간보다 작거나 같으면 첫 샘플 위치를 반환합니다.
 * - @p t 가 마지막 샘플 시간보다 크거나 같으면 마지막 샘플 위치를 반환합니다.
 *
 * @param[in] traj     보간에 사용할 trajectory 데이터
 * @param[in] t        보간할 시간 (초)
 * @param[out] out_pos 계산된 위치 벡터 (NULL이면 false 반환)
 * @return 보간 성공 시 true, 데이터가 유효하지 않으면 false
 */
BYUL_API bool trajectory_interpolate_position(
    const trajectory_t* traj, float t, vec3_t* out_pos);

/**
 * @brief 주어진 시간 t에서 타겟의 속도를 추정합니다.
 *
 * trajectory_t에 저장된 샘플들을 기반으로 t 시점의 속도를 계산합니다.
 * - 샘플이 2개 이상 있으면 인접 샘플의 위치 변화량을 사용하여 속도를 추정합니다.
 * - 샘플이 1개 이하이면 false를 반환합니다.
 *
 * @param[in] traj  속도 추정에 사용할 trajectory 데이터
 * @param[in] t     추정할 시간 (초)
 * @param[out] out_vel 계산된 속도 벡터 (NULL이면 false 반환)
 * @return 추정 성공 시 true, 실패 시 false
 */
BYUL_API bool trajectory_estimate_velocity(
    const trajectory_t* traj, float t, vec3_t* out_vel);

/**
 * @brief 주어진 시간 t에서 타겟의 가속도를 추정합니다.
 *
 * trajectory_t에 저장된 샘플들을 기반으로 t 시점의 가속도를 계산합니다.
 * - 속도를 두 번 미분하거나, 인접 샘플 간의 속도 변화량으로 근사 추정합니다.
 * - 샘플이 3개 이하이면 false를 반환합니다.
 *
 * @param[in] traj  가속도 추정에 사용할 trajectory 데이터
 * @param[in] t     추정할 시간 (초)
 * @param[out] out_acc 계산된 가속도 벡터 (NULL이면 false 반환)
 * @return 추정 성공 시 true, 실패 시 false
 */
BYUL_API bool trajectory_estimate_acceleration(
    const trajectory_t* traj, float t, vec3_t* out_acc);

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
