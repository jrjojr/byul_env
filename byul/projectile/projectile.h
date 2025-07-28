#ifndef PROJECTILE_H
#define PROJECTILE_H

/**
 * @file projectile.h
 * @brief 무기 발사체 계층(Shell, Rocket, Missile, Patriot)의 초기화 및 궤적 생성 인터페이스
 *
 * 이 헤더는 총알, 포탄, 로켓, 미사일까지 포함한 모든 **무기 발사체**를
 * 공통적으로 다룰 수 있는 API를 제공합니다.
 *
 * ### 주요 개념
 * - **Projectile:** 
 *   - 돌멩이, 화살, 석궁볼트 같은 단순 발사체.
 * - **Shell:** 
 *   - 폭발 반경을 가진 포탄(총알, 포탄 등).
 * - **Rocket:** 
 *   - 추진기(propulsion)가 추가된 발사체, 유도 없음.
 * - **Missile:** 
 *   - 로켓 + 기본 유도(`guidance_point`, `guidance_lead`).
 * - **Patriot:** 
 *   - 미사일 + 고급 유도(`guidance_predict_accel`), 
 *     **타겟 엔티티(entity_dynamic_t)**를 추적.
 *
 * ### 핵심 함수
 * - `projectile_launch()`: 
 *   - 일반 발사체의 궤적 예측.
 * - `shell_launch()`, `rocket_launch()`, `missile_launch()`, `patriot_launch()`: 
 *   - 발사체 계층별 궤적 생성 함수.
 *
 * ### 사용 예시
 * ```c
 * projectile_result_t result;
 * shell_t shell;
 * shell_init(&shell);
 * shell_launch(&shell, &target_pos, 50.0f, &env, &result);
 * ```
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include "byul_common.h"
#include "projectile_common.h"
#include "propulsion.h"
#include "guidance.h"
#include "projectile_predict.h"
#include "vec3.h"
#include "entity_dynamic.h"
#include "environ.h"

// ---------------------------------------------------------
// 전방 선언
// ---------------------------------------------------------
typedef struct s_projectile projectile_t;
typedef struct s_projectile_result projectile_result_t;
typedef struct s_shell_projectile shell_projectile_t;
typedef struct s_rocket rocket_t;
typedef struct s_missile missile_t;
typedef struct s_patriot patriot_t;

typedef struct s_shell_projectile {
    projectile_t proj;
    float explosion_radius;
} shell_projectile_t;

typedef struct s_rocket {
    shell_projectile_t base;
    propulsion_t propulsion;       ///< 추진기 (유도 없음)
} rocket_t;

typedef struct s_missile {
    rocket_t base;
    guidance_func guidance;        ///< guidance_point / guidance_lead
    void* guidance_userdata;       ///< 벡터 타겟
} missile_t;

typedef struct s_patriot {
    missile_t base;
    guidance_func guidance;        ///< guidance_predict_accel / accel_env
    void* guidance_userdata;       ///< 엔티티 타겟
} patriot_t;

// ---------------------------------------------------------
// 공통 발사체 (Projectile)
// ---------------------------------------------------------

/**
 * @brief 일반 발사체(Shell 계열)의 궤적을 계산하고 충돌 여부를 예측합니다.
 *
 * 돌멩이, 화살, 석궁 볼트 등 **단순 발사체**에 적합합니다.
 *
 * @param[in]  projectile    발사체 정보
 * @param[in]  target        목표 좌표
 * @param[in]  initial_speed 초기 발사 속도
 * @param[in]  env           환경 정보 (중력, 바람 등, NULL이면 무시)
 * @param[out] out           궤적 및 충돌 결과
 * @retval true  목표에 도달 또는 충돌 발생
 * @retval false 시뮬레이션 시간 내 도달 불가
 */
BYUL_API bool projectile_launch(
    const projectile_t* projectile,
    const vec3_t* target,
    float initial_speed,
    const environ_t* env,
    projectile_result_t* out
);

// ---------------------------------------------------------
// 포탄 (Shell) - 총알, 포탄 기본 데미지 1.0, 폭발범위 10.0
// ---------------------------------------------------------
BYUL_API void shell_projectile_init(shell_projectile_t* shell);
BYUL_API void shell_projectile_init_full(
    shell_projectile_t* shell, float damage, float explosion_radius);

BYUL_API void shell_projectile_assign(
    shell_projectile_t* shell, const shell_projectile_t* src);

BYUL_API bool shell_projectile_launch(
    const shell_projectile_t* shell,
    const vec3_t* target,
    float initial_speed,
    const environ_t* env,
    projectile_result_t* out
);

// ---------------------------------------------------------
// 로켓 (Rocket) 기본 데미지 1.0, 폭발범위 10.0
// ---------------------------------------------------------
BYUL_API void rocket_init(rocket_t* rocket);
BYUL_API void rocket_init_full(
    rocket_t* rocket, float damage, float explosion_radius);

BYUL_API void rocket_assign(rocket_t* rocket, const rocket_t* src);

BYUL_API bool rocket_launch(
    const rocket_t* rocket,
    const vec3_t* target,
    float initial_speed,
    const environ_t* env,
    projectile_result_t* out
);

// ---------------------------------------------------------
// 미사일 (Missile) 기본 데미지 1.0, 폭발범위 10.0 선형 유도 장치
// ---------------------------------------------------------
BYUL_API void missile_init(missile_t* missile);
BYUL_API void missile_init_full(
    missile_t* missile, float damage, float explosion_radius);

BYUL_API void missile_assign(missile_t* missile, const missile_t* src);

BYUL_API bool missile_launch(
    const missile_t* missile,
    const vec3_t* target,
    float initial_speed,
    const environ_t* env,
    projectile_result_t* out
);

// ---------------------------------------------------------
// 패트리엇 (Patriot)  기본 데미지 1.0, 폭발범위 10.0 비선형 유도 장치
// ---------------------------------------------------------
BYUL_API void patriot_init(patriot_t* patriot);
BYUL_API void patriot_init_full(
    patriot_t* patriot, float damage, float explosion_radius);

BYUL_API void patriot_assign(patriot_t* patriot, const patriot_t* src);

BYUL_API bool patriot_launch(
    const patriot_t* patriot,
    const entity_dynamic_t* target,
    float initial_speed,
    const environ_t* env,
    projectile_result_t* out
);

// ---------------------------------------------------------
// 기본 포탄 충돌 콜백
// ---------------------------------------------------------
/**
 * @brief 기본 포탄 충돌 콜백
 * 충돌시 데미지를 출력한다.
 */
BYUL_API void shell_projectile_hit_cb(
    const void* projectile, void* userdata);

#ifdef __cplusplus
}
#endif

#endif // PROJECTILE_H
