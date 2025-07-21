#ifndef PROJECTILE_PREDICT_H
#define PROJECTILE_PREDICT_H

#include "byul_config.h"
#include "internal/projectile.h"
#include "internal/trajectory.h"
#include "internal/projectile_propulsion.h"
#include "internal/projectile_guidance.h"
#include "internal/environ.h"
#include "internal/entity_dynamic.h"

/**
 * @struct projectile_result_t
 * @brief 발사체 궤적 예측 결과 구조체
 *
 * 이 구조체는 발사체가 이동하는 동안의 궤적(trajectory),
 * 예측 충돌 시각 및 충돌 위치를 포함합니다.
 */
typedef struct s_projectile_result {
    float impact_time;        /**< 예측 충돌 시각 (초 단위) */
    vec3_t impact_pos;        /**< 예측 충돌 위치 (월드 좌표) */
    bool valid;               /**< 예측 결과의 유효 여부 (true면 충돌 발생) */

    trajectory_t* trajectory; /**< 예측된 궤적 데이터 (동적 메모리 할당됨) */
} projectile_result_t;

// ---------------------------------------------------------
// projectile_result_t 관리 함수
// ---------------------------------------------------------

/**
 * @brief 기본 projectile_result_t 객체를 생성합니다.
 *
 * 기본 용량(capacity = 100)을 갖는 trajectory를 내부적으로 생성하고,
 * impact_time, impact_pos, valid 값을 초기화합니다.
 *
 * @return 생성된 projectile_result_t 포인터 (동적 메모리 할당됨)
 * @note 사용 후 반드시 projectile_result_destroy()로 해제해야 합니다.
 */
BYUL_API projectile_result_t* projectile_result_create();

/**
 * @brief 지정된 capacity로 trajectory를 생성하는 
 *      projectile_result_t 객체를 생성합니다.
 *
 * @param capacity trajectory에 할당할 최대 샘플 개수
 * @return 생성된 projectile_result_t 포인터 (동적 메모리 할당됨)
 * @note 사용 후 반드시 projectile_result_destroy()로 해제해야 합니다.
 */
BYUL_API projectile_result_t* projectile_result_create_full(int capacity);

/**
 * @brief 기존 projectile_result_t 객체를 깊은 복사(Deep Copy)합니다.
 *
 * @param src 원본 projectile_result_t (NULL 불가)
 * @return 복제된 projectile_result_t 포인터 (동적 메모리 할당됨)
 * @note 사용 후 반드시 projectile_result_destroy()로 해제해야 합니다.
 */
BYUL_API projectile_result_t* projectile_result_copy(
    const projectile_result_t* src);

/**
 * @brief projectile_result_t 객체와 내부 trajectory를 해제합니다.
 *
 * @param res 해제할 projectile_result_t 포인터 (NULL 가능)
 * @note trajectory_destroy()가 내부적으로 호출되며 res 자체도 free됩니다.
 */
BYUL_API void projectile_result_destroy(projectile_result_t* res);

// ---------------------------------------------------------
// 발사체 궤적 예측
// ---------------------------------------------------------

/**
 * @brief 발사체 궤적을 예측하여 충돌 여부와 궤적 데이터를 계산합니다.
 *
 * - 초기 발사체 상태(`projectile_t`)와 추진력(`propulsion_t`)을 기반으로,
 *   최대 시뮬레이션 시간(`max_time`) 동안 `time_step` 간격으로 궤적을 계산합니다.
 * - 유도 함수(`guidance_fn`)와 환경 함수(`env_fn`)를 
 *      이용해 발사체의 방향 및 가속도를 갱신합니다.
 * - 타겟 정보(`target_info_t`)가 주어지면 타겟과의 충돌을 체크하고,
 *   충돌 시점 및 충돌 위치를 `projectile_result_t`에 저장합니다.
 *
 * @param[out] out        예측 결과를 저장할 projectile_result_t (NULL 불가)
 * @param[in]  proj       초기 발사체 상태
 * @param[in]  propulsion 추진력 정보 (NULL이면 추진력 없음)
 * @param[in]  guidance_fn 유도 함수 포인터
 * @param[in]  guidance_userdata 유도 함수에 전달할 사용자 데이터
 * @param[in]  target_info 타겟 정보 구조체 (NULL이면 타겟 없음)
 * @param[in]  max_time   시뮬레이션 최대 시간 (초)
 * @param[in]  time_step  시뮬레이션 간격 (초)
 * @param[in]  env_fn     환경 함수 포인터 (NULL이면 환경 영향 없음)
 * @param[in]  env_userdata 환경 함수에 전달할 사용자 데이터
 *
 * @return true이면 충돌 발생, false이면 충돌 없음
 */
BYUL_API bool projectile_predict(
    projectile_result_t* out,
    const projectile_t* proj,
    const propulsion_t* propulsion,
    projectile_guidance_func guidance_fn,
    void* guidance_userdata,
    target_info_t* target_info,
    float max_time,
    float time_step,
    environ_func env_fn,
    void* env_userdata);

#endif // PROJECTILE_PREDICT_H
