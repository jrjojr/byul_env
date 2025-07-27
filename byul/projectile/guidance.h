#ifndef GUIDANCE_H
#define GUIDANCE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "byul_common.h"
#include "internal/vec3.h"
#include "internal/entity_dynamic.h"
#include "internal/trajectory.h"

/**
 * @typedef guidance_func
 * @brief 발사체 유도 함수 타입
 *
 * 발사체의 현재 상태와 시간 간격, 사용자 데이터를 바탕으로
 * **정규화된 방향 벡터(단위 벡터)**를 `out`에 저장하고 포인터를 반환합니다.
 * 만약 `out`이 NULL이면 함수 내부에서 static 버퍼를 사용합니다.
 *
 * @param[in]  entdyn     현재 발사체 포인터
 * @param[in]  dt       시간 간격 (초)
 * @param[in]  userdata 사용자 정의 데이터 포인터 (타겟 정보 등)
 * @param[out] out      계산된 단위 벡터 (NULL이면 내부 static 사용)
 * @return out 또는 static 버퍼의 포인터
 */
typedef const vec3_t* (*guidance_func)(
    const entity_dynamic_t* entdyn, float dt, void* userdata, vec3_t* out);

// ---------------------------------------------------------
// 유도 없음 (None)
// ---------------------------------------------------------

/**
 * @brief 유도 없음
 *
 * 발사체는 현재 속도와 방향을 유지하며 추가적인 유도 계산을 하지 않습니다.
 * 항상 (0,0,0) 벡터를 반환합니다.
 *
 * @param[in]  entdyn     현재 발사체 포인터
 * @param[in]  dt       시간 간격 (초)
 * @param[in]  userdata 사용하지 않음 (NULL)
 * @param[out] out      (0,0,0) 벡터를 저장 (NULL이면 static 버퍼 사용)
 * @return out 또는 static 버퍼의 포인터
 */
BYUL_API const vec3_t* guidance_none(
    const entity_dynamic_t* entdyn, float dt, void* userdata, vec3_t* out);

// ---------------------------------------------------------
// 선형 유도
// ---------------------------------------------------------

/**
 * @brief 정적 타겟 유도
 *
 * 지정된 **고정 타겟 위치**를 향한 정규화된 방향 벡터를 계산합니다.
 * `userdata`는 `const vec3_t*` 타입의 목표 위치 포인터여야 합니다.
 *
 * @param[in]  entdyn     현재 발사체 포인터
 * @param[in]  dt       시간 간격 (초)
 * @param[in]  userdata `const vec3_t*` (목표 위치)
 * @param[out] out      계산된 단위 벡터 (NULL이면 static 버퍼 사용)
 * @return out 또는 static 버퍼의 포인터
 */
BYUL_API const vec3_t* guidance_point(
    const entity_dynamic_t* entdyn, float dt, void* userdata, vec3_t* out);

/**
 * @brief 이동 타겟 리드 유도
 *
 * **이동 중인 타겟의 위치와 속도**를 기반으로 미래 교차점을 예측하고,
 * 해당 지점을 향한 정규화된 방향 벡터를 계산합니다.
 * `userdata`는 `const entity_dynamic_t*` 타입의 타겟 포인터여야 합니다.
 *
 * @param[in]  entdyn     현재 발사체 포인터
 * @param[in]  dt       시간 간격 (초)
 * @param[in]  userdata `const entity_dynamic_t*` (이동 타겟 정보)
 * @param[out] out      계산된 단위 벡터 (NULL이면 static 버퍼 사용)
 * @return out 또는 static 버퍼의 포인터
 */
BYUL_API const vec3_t* guidance_lead(
    const entity_dynamic_t* entdyn, float dt, void* userdata, vec3_t* out);

/**
 * @struct guidance_target_info_t
 * @brief 유도 시스템에서 사용하는 타겟 정보 구조체
 *
 * 이 구조체는 발사체 유도 함수에 전달되는 타겟의 상태와 환경 정보를 포함합니다.
 * - 타겟의 실시간 동적 상태(`entity_dynamic_t`)
 * - 외부 환경 요소(`environ_t`)
 * - 예측 기준 시각(`current_time`)
 */
typedef struct s_guidance_target_info {
    entity_dynamic_t target;   /**< 추적할 타겟 엔티티 포인터 */
    environ_t env;            /**< 환경 정보 (중력, 바람, 드래그 등) */
    float current_time;              /**< 예측 기준 시각 (초 단위) */
} guidance_target_info_t;


// ---------------------------------------------------------
// 비선형 유도
// ---------------------------------------------------------

/**
 * @brief 발사체 유도 함수 (수식 기반 예측)
 *
 * 이 함수는 타겟의 현재 위치와 속도를 기반으로 요격 위치를 계산하여,
 * 발사체가 향해야 할 단위 방향 벡터를 반환합니다.
 *
 * 내부적으로 단순한 리드타임(lead time) 예측을 사용하여
 * `미사일 위치 + 미사일 속도 * t = 타겟 위치 + 타겟 속도 * t` 조건을 풀어
 * 교차점 방향을 구합니다.
 *
 * @param[in]  entdyn     현재 발사체 포인터 (NULL이면 (0,0,0) 반환)
 * @param[in]  dt       시간 간격 (초)
 * @param[in]  userdata 유저 데이터 (guidance_target_info_t*를 기대)
 * @param[out] out      계산된 단위 방향 벡터 (NULL이면 내부 static 버퍼 사용)
 *
 * @return 단위화된 방향 벡터의 포인터 (out 또는 static)
 */
BYUL_API const vec3_t* guidance_predict(
    const entity_dynamic_t* entdyn,
    float dt,
    void* userdata,
    vec3_t* out);


/**
 * @brief 발사체 유도 함수 (가속도 반영 예측, Cardano 기반)
 *
 * 이 함수는 타겟의 위치, 속도, 가속도(`guidance_target_info_t`)를 기반으로
 * **3차 방정식(Cardano 해법)**을 풀어 요격 시점을 계산하고,
 * 그 시점의 타겟 위치를 향하는 단위 벡터를 반환합니다.
 *
 * - 타겟 가속도가 0이면 2차 방정식으로 자동 폴백합니다.
 * - 환경(`environ_t`)에서 제공되는 중력, 바람, 드래그 등이 있으면
 *   `target_acc`에 합산해 예측 정확도를 높입니다.
 *
 * @param[in]  entdyn     현재 발사체 포인터
 * @param[in]  dt       시간 간격 (초)
 * @param[in]  userdata 유저 데이터 (guidance_target_info_t*를 기대)
 * @param[out] out      계산된 단위 방향 벡터 (NULL이면 내부 static 버퍼 사용)
 *
 * @return 단위화된 방향 벡터의 포인터
 */
BYUL_API const vec3_t* guidance_predict_accel(
    const entity_dynamic_t* entdyn,
    float dt,
    void* userdata,
    vec3_t* out);

/**
 * @brief 발사체 유도 함수 (가속도 + 환경 영향 + 엔티티 상태 기반)
 *
 * 이 함수는 `entity_dynamic_t`에서 타겟의 **정확한 위치/속도**를,
 * `environ_t`에서 **외부 환경 영향(중력, 바람, 드래그 등)**을 가져와
 * **Cardano 기반 요격 시간 계산**을 수행하고 최적 요격 방향을 반환합니다.
 *
 * 주요 특징:
 * - 타겟의 선형 운동(velocity)과 외부 가속도(acceleration)를 모두 고려
 * - 공기 저항 및 바람 영향을 환경 정보와 bodyprops를 통해 반영 가능
 * - 발사체와 타겟이 근접한 경우에는 (0,0,0)으로 안정 처리
 *
 * @param[in]  entdyn     현재 발사체 포인터
 * @param[in]  dt       시간 간격 (초)
 * @param[in]  userdata 유저 데이터 (guidance_target_info_t*를 기대)
 * @param[out] out      계산된 단위 방향 벡터 (NULL이면 내부 static 버퍼 사용)
 *
 * @return 단위화된 방향 벡터의 포인터
 */
BYUL_API const vec3_t* guidance_predict_accel_env(
    const entity_dynamic_t* entdyn,
    float dt,
    void* userdata,
    vec3_t* out);

#ifdef __cplusplus
}
#endif    

#endif // GUIDANCE_H
