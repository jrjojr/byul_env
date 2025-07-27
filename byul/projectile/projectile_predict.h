#ifndef PROJECTILE_PREDICT_H
#define PROJECTILE_PREDICT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "byul_common.h"
#include "internal/projectile_common.h"
#include "internal/trajectory.h"
#include "internal/propulsion.h"
#include "internal/guidance.h"
#include "internal/environ.h"
#include "internal/entity_dynamic.h"
#include "internal/numeq_filters.h"

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
 * @brief projectile_result_t 객체를 초기화 상태로 되돌립니다.
 *
 * - impact_time, impact_pos, valid 값을 초기화.
 * - trajectory는 유지하되 내부 데이터를 clear.
 *
 * @param res 초기화할 projectile_result_t 포인터 (NULL 가능)
 */
BYUL_API void projectile_result_reset(projectile_result_t* res);

/**
 * @brief trajectory의 메모리를 해제하고 새로운 capacity로 재할당합니다.
 *
 * @param res 변경할 projectile_result_t 포인터 (NULL 불가)
 * @param new_capacity 새로 할당할 샘플 최대 개수
 */
BYUL_API void projectile_result_resize(projectile_result_t* res, int new_capacity);

/**
 * @brief projectile_result_t 객체를 완전 해제하고 NULL로 초기화.
 *
 * - reset과 비슷하지만 trajectory도 완전 free.
 * - destroy와의 호환성을 위해 제공.
 *
 * @param res 해제할 projectile_result_t 포인터 (NULL 가능)
 */
BYUL_API void projectile_result_free(projectile_result_t* res);

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
 * @brief 발사체 궤적을 시뮬레이션하여 충돌 여부와 궤적 데이터를 계산합니다.
 *
 * 이 함수는 발사체의 초기 상태(`projectile_t`)와 주어진 환경 정보를 기반으로
 * 궤적을 예측하고, 타겟 충돌 여부를 확인합니다.
 * - **유도 함수(`guidance_func`)**를 이용해 발사체의 방향을 실시간 조정할 수 있습니다.
 * - **추진력(`propulsion_t`)**이 있으면 추진 가속도가 적용됩니다.
 * - **환경 정보(`environ_t`)**를 통해 중력, 바람, 공기 저항 등의 외부 힘이 반영됩니다.
 *
 * @param[out] out          발사체 궤적 및 충돌 결과를 저장할 구조체 (NULL 불가)
 * @param[in]  proj         초기 발사체 상태 (위치, 속도, 질량 등)
 * @param[in]  entdyn       타겟 동적 정보 (NULL이면 목표 없음)
 * @param[in]  max_time     시뮬레이션 최대 시간 (초)
 * @param[in]  time_step    시뮬레이션 간격 (초)
 * @param[in]  env          환경 정보 (NULL이면 환경 영향 없음)
 * @param[in]  propulsion   추진력 정보 (NULL이면 추진력 없음)
 * @param[in]  guidance_fn  유도 함수 포인터 (NULL이면 유도 없음)
 *
 * @retval true   충돌 발생 (타겟 또는 지면과의 충돌)
 * @retval false  충돌 없음 (최대 시뮬레이션 시간까지 도달하지 않음)
 */
BYUL_API bool projectile_predict(
    projectile_result_t* out,
    const projectile_t* proj,
    entity_dynamic_t* entdyn,
    float max_time,
    float time_step,
    const environ_t* env,
    propulsion_t* propulsion,
    guidance_func guidance_fn);

/**
 * @brief Kalman 필터를 적용한 발사체 궤적 예측.
 *
 * `projectile_predict()`의 기본 시뮬레이션 로직에
 * **3차원 칼만 필터(`kalman_filter_vec3_t`)**를 적용하여
 * 위치 및 속도를 보정하면서 궤적을 예측합니다.
 *
 * - 노이즈가 많은 환경(바람, 측정 오차)에서 궤적 예측 정확도를 향상.
 * - Kalman Filter의 `process_noise`, 
 *   `measurement_noise`는 함수 내부에서 기본값으로 초기화되며,
 *   필요시 커스터마이징할 수 있습니다.
 *
 * @warning
 * **실험적인 기능입니다.**
 * 이 함수는 칼만 필터를 궤적 예측 루프에 인위적으로 삽입하여 사용하지만,
 * 실제 필터링은 **동적인 실시간 `update()` 
 * 루프에서 측정 데이터를 기반으로 수행**하는 것이 권장됩니다.
 * 본 함수는 테스트 및 시뮬레이션 검증 목적으로만 사용하세요.
 *
 * @param[out] out          발사체 궤적 및 충돌 결과를 저장할 구조체
 * @param[in]  proj         초기 발사체 상태
 * @param[in]  entdyn       타겟 동적 정보
 * @param[in]  max_time     시뮬레이션 최대 시간 (초)
 * @param[in]  time_step    시뮬레이션 간격 (초)
 * @param[in]  env          환경 정보
 * @param[in]  propulsion   추진력 정보
 * @param[in]  guidance_fn  유도 함수 포인터
 *
 * @retval true   충돌 발생
 * @retval false  충돌 없음
 */
BYUL_API bool projectile_predict_with_kalman_filter(
    projectile_result_t* out,
    const projectile_t* proj,
    entity_dynamic_t* entdyn,
    float max_time,
    float time_step,
    const environ_t* env,
    const propulsion_t* propulsion,
    guidance_func guidance_fn);

/**
 * @brief 공통 필터 인터페이스(`filter_interface_t`)를 적용한 발사체 궤적 예측.
 *
 * `projectile_predict()`와 동일한 궤적 시뮬레이션을 수행하되,
 * Kalman, EKF, UKF 등 다양한 필터를 **`filter_interface_t`**를 통해 삽입하여
 * 위치 및 속도 데이터를 보정합니다.
 *
 * - 필터는 `filter_if->time_update()`와 
 *   `filter_if->measurement_update()`를 통해 갱신됩니다.
 * - 필터링된 상태(`get_state`)가 매 스텝에서 `state.linear`에 반영됩니다.
 *
 * @warning
 * **실험적인 기능입니다.**
 * 이 함수는 궤적 예측 루프 내에 필터링을 직접 포함하고 있으나,
 * 실제 사용 시에는 **동적 루프에서 실시간 측정 데이터와 함께 필터를 적용**하는 것이 
 * 더 올바른 접근입니다.
 * 본 함수는 테스트 및 알고리즘 비교 목적으로만 활용하세요.
 *
 * @param[out] out          발사체 궤적 및 충돌 결과를 저장할 구조체
 * @param[in]  proj         초기 발사체 상태
 * @param[in]  entdyn       타겟 동적 정보
 * @param[in]  max_time     시뮬레이션 최대 시간 (초)
 * @param[in]  time_step    시뮬레이션 간격 (초)
 * @param[in]  env          환경 정보
 * @param[in]  propulsion   추진력 정보
 * @param[in]  guidance_fn  유도 함수 포인터
 * @param[in]  filter_if    필터 인터페이스 (NULL이면 필터 미적용)
 *
 * @retval true   충돌 발생
 * @retval false  충돌 없음
 */
BYUL_API bool projectile_predict_with_filter(
    projectile_result_t* out,
    const projectile_t* proj,
    entity_dynamic_t* entdyn,
    float max_time,
    float time_step,
    const environ_t* env,
    const propulsion_t* propulsion,
    guidance_func guidance_fn,
    const filter_interface_t* filter_if);

/**
 * @struct launch_param_t
 * @brief 발사체를 목표로 발사하기 위해 필요한 초기 파라미터를 정의합니다.
 *
 * - **direction**: 목표를 향한 발사 방향 (단위 벡터)
 * - **force**: 발사 순간 적용되는 초기 힘 (뉴턴, N)
 *      - 1 N = 1 kg × 1 m/s² (뉴턴)
 *      - 예시 권장값:
 *      * 1 kg 발사체 → 10~100 N (10~30 m/s 초기 속도)
 *      * 10 kg 발사체 → 500~5000 N (20~100 m/s 초기 속도)* 
 * - **time_to_hit**: 목표에 도달할 것으로 예상되는 시간 (초)
 *
 * @note
 * - direction은 항상 단위 벡터로 보장됩니다.
 * - force 값은 질량 및 발사체 특성에 따라 실제 초기 속도 벡터로 환산됩니다.
 */
typedef struct s_launch_param {
    vec3_t direction;    ///< 발사 방향 (단위 벡터)
    float  force;        ///< 초기 발사 힘 (뉴턴, N)
    float  time_to_hit;  ///< 목표 도달 예상 시간 (초)
} launch_param_t;


/**
 * @brief 포탄이 주어진 target 위치에 도달하기 위한 발사 파라미터를 계산합니다.
 *
 * 이 함수는 **환경 요소(중력, 바람 등)를 전혀 고려하지 않고**,  
 * 발사체의 **자체 특성(질량, 마찰계수 등)**만을 기반으로 계산을 수행합니다.
 * 
 * - 발사 후 속도는 마찰계수에 의해 서서히 감소하며, 중력 효과가 없기 때문에
 *   바닥으로 떨어지지 않습니다.
 * - 속도가 0이 되면 발사체는 그 위치에 그대로 정지합니다.
 * - 초기 힘(`force`)과 방향(`direction`)을 계산하여 `out`에 저장하며,
 *   목표에 도달하기 위해 필요한 예상 도달 시간(`time_to_hit`)을 함께 제공합니다.
 *
 * @param[out] out           계산 결과 (발사 방향, 초기 힘, 도달 시간)
 * @param[in]  proj          발사체 정보 (시작 위치, 질량, 마찰계수 등)
 * @param[in]  target        목표 지점의 세계 좌표
 * @param[in]  initial_force 발사 순간 적용되는 초기 힘 (뉴턴, N)
 *                           - 1 N = 1 kg × 1 m/s² (뉴턴)
 *                           - 예시 권장값:
 *                             * 1 kg 발사체 → 10~100 N
 *                             * 10 kg 발사체 → 500~5000 N
 *
 * @return `true` = 계산 성공, `false` = 주어진 힘으로 목표에 도달 불가
 */
BYUL_API bool projectile_calc_launch_param(
    launch_param_t* out,
    const projectile_t* proj,
    const vec3_t* target,
    float initial_force
);

/**
 * @brief 환경 요소를 고려하여 목표에 도달하기 위한 발사 파라미터를 계산합니다.
 *
 * 중력, 바람, 공기 저항 등 환경 요소(`environ_t`)를 반영하여,
 * 초기 발사 방향(`direction`)과 힘(`force`), 도달 시간(`time_to_hit`)을 계산합니다.
 *
 * @param[out] out           계산 결과 (발사 방향, 힘, 도달 시간)
 * @param[in]  proj          발사체 정보 (시작 위치, 질량 등)
 * @param[in]  env           환경 정보 (중력, 바람, 공기 저항 등)
 * @param[in]  target        목표 지점의 세계 좌표
 * @param[in]  initial_force 발사 순간 적용되는 초기 힘 (뉴턴, N)
 *
 * @return `true` = 계산 성공, `false` = 해당 환경 조건에서 목표 도달 불가
 */
BYUL_API bool projectile_calc_launch_param_env(
    launch_param_t* out,
    const projectile_t* proj,
    const environ_t* env,
    const vec3_t* target,
    float initial_force
);


/**
 * @brief 목표 위치와 도달 시간(hit_time)이 주어졌을 때 필요한 발사 파라미터를 역으로 계산합니다.
 *
 * 이 함수는 **환경 요소(중력, 바람 등)를 고려하지 않고**,  
 * 발사체의 **시작 위치와 목표 위치** 및 **지정된 도달 시간(hit_time)**을 기반으로
 * 초기 발사 방향(`direction`)과 필요한 초기 힘(`force`)을 역산합니다.
 *
 * - 발사체는 자체 마찰계수로만 속도가 감소하며, 중력에 의해 바닥으로 떨어지지 않습니다.
 * - hit_time 동안 정확히 목표에 도달하기 위해 필요한 힘(force)을 계산하고,
 *   발사 방향은 시작 위치와 목표 위치를 연결하는 벡터를 단위화하여 설정됩니다.
 * - 계산된 도달 시간(`time_to_hit`)은 입력된 hit_time과 동일합니다.
 *
 * @param[out] out      계산 결과 (발사 방향, 초기 힘, 도달 시간)
 * @param[in]  proj     발사체 정보 (시작 위치, 질량, 마찰계수 등)
 * @param[in]  target   목표 지점의 세계 좌표
 * @param[in]  hit_time 목표 도달 시간 (초, 0 이하이면 실패)
 *
 * @return `true` = 계산 성공, `false` = 주어진 시간으로 목표 도달 불가
 */
BYUL_API bool projectile_calc_launch_param_inverse(
    launch_param_t* out,
    const projectile_t* proj,
    const vec3_t* target,
    float hit_time
);

/**
 * @brief 환경 요소를 고려하여 목표 위치와 도달 시간(hit_time)에 
 * 맞춘 발사 파라미터를 역으로 계산합니다.
 *
 * 중력, 바람, 공기 저항 등 환경 요소를 반영하여
 * 목표 지점과 도달 시간(hit_time)을 만족하는 
 * 초기 발사 방향(`direction`)과 힘(`force`)을 계산합니다.
 *
 * @param[out] out      계산 결과 (방향, 힘, 입력된 도달 시간)
 * @param[in]  proj     발사체 정보 (시작 위치, 질량 등)
 * @param[in]  env      환경 정보 (중력, 바람, 공기 저항 등)
 * @param[in]  target   목표 지점의 세계 좌표
 * @param[in]  hit_time 목표 도달 시간 (초)
 *
 * @return `true` = 계산 성공, `false` = 주어진 조건에서 목표 도달 불가
 */
BYUL_API bool projectile_calc_launch_param_inverse_env(
    launch_param_t* out,
    const projectile_t* proj,
    const environ_t* env,    
    const vec3_t* target,
    float hit_time
);

#ifdef __cplusplus
}
#endif

#endif // PROJECTILE_PREDICT_H
