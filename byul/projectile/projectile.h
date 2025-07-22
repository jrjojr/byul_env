#ifndef PROJECTILE_H
#define PROJECTILE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include "byul_config.h"
#include "internal/entity_dynamic.h"

// ---------------------------------------------------------
// 탄도체 유형
// ---------------------------------------------------------
typedef enum e_projectile_type {
    PROJECTILE_TYPE_SHELL,    ///< 단순 포탄 (추진력 없음)
    PROJECTILE_TYPE_MISSILE   ///< 미사일 (추진력/유도 포함)
} projectile_type_t;

// ---------------------------------------------------------
// 구조체 및 콜백 타입 선언
// ---------------------------------------------------------
typedef struct s_projectile projectile_t;

/**
 * @brief 충돌 콜백 함수 타입
 * @param proj      충돌이 발생한 발사체
 * @param userdata  사용자 지정 데이터
 */
typedef void (*projectile_hit_cb)(const projectile_t* proj, void* userdata);

/**
 * @struct s_projectile
 * @brief 포탄/미사일 등 발사체의 공통 구조체
 */
struct s_projectile {
    entity_dynamic_t base;    ///< 동적 엔티티 기반 속성
    projectile_type_t type;   ///< 발사체 유형
    projectile_hit_cb on_hit; ///< 충돌 콜백 (NULL 가능)
    void* hit_userdata;       ///< 충돌 콜백 사용자 데이터
};

// ---------------------------------------------------------
// 초기화 함수
// ---------------------------------------------------------
/**
 * @brief projectile_t를 기본값으로 초기화합니다.
 *
 * **기본값**
 * - base: entity_dynamic_init()
 * - type = PROJECTILE_TYPE_SHELL
 * - on_hit = NULL
 * - hit_userdata = NULL
 */
BYUL_API void projectile_init(projectile_t* proj);

/**
 * @brief projectile_t를 사용자 지정 값으로 완전 초기화합니다.
 *
 * - base 동적 엔티티(entity_dynamic_t)를 사전에 설정해 전달하거나,
 *   NULL이면 내부에서 entity_dynamic_init()으로 초기화합니다.
 *
 * @param[out] proj         초기화할 발사체
 * @param[in]  base         동적 엔티티 정보 (NULL이면 기본값)
 * @param[in]  type         발사체 유형
 * @param[in]  on_hit       충돌 콜백 (NULL 가능)
 * @param[in]  hit_userdata 콜백에 전달할 사용자 데이터
 */
BYUL_API void projectile_init_full(
    projectile_t* proj,
    const entity_dynamic_t* base,
    projectile_type_t type,
    projectile_hit_cb on_hit,
    void* hit_userdata
);

/**
 * @brief projectile_t를 다른 projectile_t로 복사합니다.
 * @param[out] out 복사 대상
 * @param[in]  src 원본
 */
BYUL_API void projectile_assign(projectile_t* out, const projectile_t* src);

// ---------------------------------------------------------
// 업데이트 및 동작 함수
// ---------------------------------------------------------
/**
 * @brief 발사체 상태 갱신
 *
 * - 위치 = 위치 + 속도 * dt
 * - 회전 = angular_velocity * dt 적용
 * - 수명(lifetime) 체크 후 만료 시 on_hit 콜백 호출
 *
 * @param proj 발사체
 * @param dt   시간 간격 (초)
 */
BYUL_API void projectile_update(projectile_t* proj, float dt);

// ---------------------------------------------------------
// 기본 콜백
// ---------------------------------------------------------
/**
 * @brief 기본 충돌 콜백 (아무 작업도 하지 않음)
 * 아무 작업하지 않는다고 출력한다
 */
BYUL_API void projectile_default_hit_cb(
    const projectile_t* proj, void* userdata);

typedef struct s_comp_result {
    vec3_t vec;   ///< 타겟으로 보내기 위한 초기 속도 (방향+힘)
    float dt;     ///< 타겟 도달 예상 시간
} comp_result_t;

/**
 * @brief 포탄이 주어진 target 위치에 도달하기 위해 필요한 
 * 초기 발사 속도와 예상 비행 시간을 계산합니다.
 *
 * 이 함수는 단순 포물선 운동(중력만 고려)을 기반으로 하여 목표 지점에 도달하기 위한
 * 초기 속도 벡터(`out->vec`)와 예상 비행 시간(`out->dt`)을 계산합니다.
 *
 * @param[out] out           계산 결과 (속도 벡터 및 예상 도착 시간)
 * @param[in]  proj          발사체 정보 (시작 위치, 질량 등)
 * @param[in]  target        목표 지점의 세계 좌표
 * @param[in]  initial_force 발사 순간에 적용되는 힘 (뉴턴, N)
 *                           - 1 N = 1 kg × 1 m/s² (뉴턴의 정의)
 *                           - 평균 권장값:
 *                             * 1 kg 발사체 → 약 10~100 N (10~30 m/s 초기 속도)
 *                             * 10 kg 발사체 → 약 500~5000 N (20~100 m/s 초기 속도)
 *
 * @return `true`이면 계산 성공, `false`이면 해당 힘으로 목표에 도달할 수 없음
 */
BYUL_API bool projectile_compute_launch(
    comp_result_t* out,
    const projectile_t* proj,
    const vec3_t* target,
    float initial_force
);

/**
 * @brief 환경 정보를 고려하여 포탄이 target 위치에 도달하기 위한 
 * 초기 발사 속도와 예상 비행 시간을 계산합니다.
 *
 * 중력, 바람, 공기 저항 등 환경 요소를 반영하여 목표 지점에 도달하기 위한
 * 초기 속도 벡터(`out->vec`)와 예상 비행 시간(`out->dt`)을 계산합니다.
 *
 * @param[out] out           계산 결과 (속도 벡터 및 예상 도착 시간)
 * @param[in]  proj          발사체 정보 (시작 위치, 질량 등)
 * @param[in]  env           환경 정보 (중력, 바람, 공기 저항 등)
 * @param[in]  target        목표 지점의 세계 좌표
 * @param[in]  initial_force 발사 순간에 적용되는 힘 (뉴턴, N)
 *                           - 1 N = 1 kg × 1 m/s² (뉴턴의 정의)
 *                           - 평균 권장값:
 *                             * 1 kg 발사체 → 약 10~100 N (10~30 m/s 초기 속도)
 *                             * 10 kg 발사체 → 약 500~5000 N (20~100 m/s 초기 속도)
 *
 * @return `true`이면 계산 성공, `false`이면 주어진 힘과 환경으로 목표에 도달할 수 없음
 */
BYUL_API bool projectile_compute_launch_env(
    comp_result_t* out,
    const projectile_t* proj,
    const environ_t* env,
    const vec3_t* target,
    float initial_force
);

#ifdef __cplusplus
}
#endif

#endif // PROJECTILE_H
