#ifndef PROJECTILE_H
#define PROJECTILE_H

/**
 * @file projectile.h
 * @brief Initialization and trajectory generation interface for weapon projectiles 
 *        (Shell, Rocket, Missile, Patriot)
 *
 * This header provides an API that can handle all **weapon projectiles**,
 * including bullets, shells, rockets, and missiles, in a unified way.
 *
 * ### Key Concepts
 * - **Projectile:** 
 *   - Simple projectiles like stones, arrows, and crossbow bolts.
 * - **Shell:** 
 *   - Explosive shells (bullets, artillery shells) with a blast radius.
 * - **Rocket:** 
 *   - Projectile with a propulsion system (no guidance).
 * - **Missile:** 
 *   - Rocket + basic guidance (`guidance_point`, `guidance_lead`).
 * - **Patriot:** 
 *   - Missile + advanced guidance (`guidance_predict_accel`), 
 *     tracks a **target entity (entity_dynamic_t)**.
 *
 * ### Core Functions
 * - `projectile_launch()`: 
 *   - Predicts the trajectory of a general projectile.
 * - `shell_launch()`, `rocket_launch()`, `missile_launch()`, `patriot_launch()`: 
 *   - Functions to generate trajectories for each projectile type.
 *
 * ### Example Usage
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

/**
 * @file trajectory_config.h
 * @brief Trajectory Simulation Configuration (초고속 전략 발사체 대응 포함)
 *
 * 이 구성은 기본 궤적 시뮬레이션 해상도를 정의합니다.
 * 시뮬레이션 대상은 다양한 속도 범위를 포함하며, 
 * Mach 3 이상의 초음속부터 Mach 25 수준의 전략 ICBM까지 커버 가능합니다.
 *
 * - 궤적 대상 거리: 99999.0 meters
 * - 시뮬레이션 총 시간: 100.0초
 * - 샘플 수: 2048개 (dt ≈ 0.0488초)
 *
 * @section Velocity 기준 속도 정보
 * - 평균 속도: 999.99 m/s
 * - 시속 기준: 약 3600 km/h
 * - 음속 기준: Mach 2.92 (표준 해수면 속도 기준, 343 m/s)
 *
 * @section Velocity 등급 구분
 * - Mach 3 이상: 고속 미사일 수준
 * - Mach 5 이상: 극초음속 미사일 (예: DF-17, HGV)
 * - Mach 20 이상: 전략 ICBM 수준 (예: 트라이던트 II, 아방가르드)
 *
 * @section 확장 시뮬레이션 구성 예시
 * - Mach 5 대응 시   : 60초 / 2048샘플 / dt ≈ 0.0293초
 * - Mach 20 대응 시  : 15초 / 3000샘플 / dt ≈ 0.0050초
 * - Mach 25 대응 시  : 12초 / 4096샘플 / dt ≈ 0.00293초
 *
 * @note 고속일수록 시뮬레이션 시간은 짧고, 샘플 수는 많아야 정밀한 궤적 표현이 가능합니다.
 *
 * @section VisualPerception 별이의 세계 시각적 체감 기준
 * 현실적으로 별이의 세계에서 가장 자연스럽고 편안하게 인지되는 속도는 시속 3.6~4.0 km/h입니다.
 * 이는 1.0 m/s에 해당하며, 다음과 같은 픽셀 해상도에서 편안하게 시각적으로 추적할 수 있습니다:
 *
 * @subsection VisualScales 모니터 해상도 & 픽셀 매핑 기준
 * - 10 px = 1 m (1 px = 10 cm) → 1.0 m/s = 10 px/sec 이동
 * - 100 px = 1 m (1 px = 1 cm) → 1.0 m/s = 100 px/sec 이동
 * - 1000 px = 1 m (1 px = 1 mm) → 1.0 m/s = 1000 px/sec 이동
 *
 * @subsection PracticalView 실제 시야 기준
 * - 대부분의 모니터는 1920x1080 해상도 기준 약 50~100 px/sec의 이동 속도가 
 *   부드럽고 시각적으로 인식 가능함
 * - 따라서 "1.0 m/s (시속 3.6 km)"는 해상도 100 px/m 기준에서 약 100 px/sec로 이동하며,
 *   시각적으로 자연스러운 '걷기 속도'처럼 느껴짐
 *
 * @see projectile_profile.h
 */

#define MAX_SAMPLE_COUNT   2048
#define MIN_SIM_TIME                  1.0f
#define MAX_SIM_TIME       100.0f // 100 초
#define DELTA_TIME                 (MAX_SIM_TIME / MAX_SAMPLE_COUNT)   ///< ≈ 0.048828125초

// #define XFORM_MAX_DISTANCE   99999.0f   // 목표 예측 거리
#define XFORM_MAX_DISTANCE   XFORM_MAX_POS // 목표 예측 거리
#define MIN_DELTA_TIME                    0.002f     // 최대 정밀도 (500Hz)
#define MAX_DELTA_TIME                    0.1f       // 최소 정밀도 (10Hz)

// ---------------------------------------------------------
// Forward Declarations
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
    propulsion_t propulsion;       ///< Propulsion (no guidance)
} rocket_t;

typedef struct s_missile {
    rocket_t base;
    guidance_func guidance;        ///< guidance_point / guidance_lead
    void* guidance_userdata;       ///< Vector target
} missile_t;

typedef struct s_patriot {
    missile_t base;
    guidance_func guidance;        ///< guidance_predict_accel / accel_env
    void* guidance_userdata;       ///< Entity target
} patriot_t;

// ---------------------------------------------------------
// General Projectile
// ---------------------------------------------------------
BYUL_API bool projectile_launch(
    const projectile_t* proj,
    const vec3_t* dir,             // normalized
    float initial_force,               // 초기 힘 크기
    const environ_t* env,       // 환경 + update_func 포함 가능
    projectile_result_t* out
);

// ---------------------------------------------------------
// Shell - Default damage 1.0, explosion radius 10.0
// ---------------------------------------------------------
BYUL_API void shell_projectile_init(shell_projectile_t* shell);
BYUL_API void shell_projectile_init_full(
    shell_projectile_t* shell, float damage, float explosion_radius);

BYUL_API void shell_projectile_assign(
    shell_projectile_t* shell, const shell_projectile_t* src);

BYUL_API bool shell_projectile_launch(
    const shell_projectile_t* shell,
    const vec3_t* dir,             // normalized
    float initial_force,               // 초기 힘 크기
    const environ_t* env,       // 환경 + update_func 포함 가능
    projectile_result_t* out
);

// ---------------------------------------------------------
// Rocket - Default damage 1.0, explosion radius 10.0
// ---------------------------------------------------------
BYUL_API void rocket_init(rocket_t* rocket);
BYUL_API void rocket_init_full(
    rocket_t* rocket, float damage, float explosion_radius);

BYUL_API void rocket_assign(rocket_t* rocket, const rocket_t* src);

BYUL_API bool rocket_launch(
    rocket_t* rocket,
    const vec3_t* target,
    float initial_force,
    const environ_t* env,
    projectile_result_t* out
);

// ---------------------------------------------------------
// Missile - Default damage 1.0, explosion radius 10.0, linear guidance
// ---------------------------------------------------------
BYUL_API void missile_init(missile_t* missile);
BYUL_API void missile_init_full(
    missile_t* missile, float damage, float explosion_radius);

BYUL_API void missile_assign(missile_t* missile, const missile_t* src);

BYUL_API bool missile_launch(
    missile_t* missile,
    const vec3_t* target,
    float initial_force,
    const environ_t* env,
    projectile_result_t* out
);

// ---------------------------------------------------------
// Patriot - Default damage 1.0, explosion radius 10.0, nonlinear guidance
// ---------------------------------------------------------
BYUL_API void patriot_init(patriot_t* patriot);
BYUL_API void patriot_init_full(
    patriot_t* patriot, float damage, float explosion_radius);

BYUL_API void patriot_assign(patriot_t* patriot, const patriot_t* src);

BYUL_API bool patriot_launch(
    patriot_t* patriot,
    const entity_dynamic_t* target,
    float initial_force,
    const environ_t* env,
    projectile_result_t* out
);

// ---------------------------------------------------------
// Default Shell Collision Callback
// ---------------------------------------------------------
/**
 * @brief Default shell collision callback
 * Prints damage on collision.
 */
BYUL_API void shell_projectile_hit_cb(
    const void* projectile, void* userdata);

#ifdef __cplusplus
}
#endif

#endif // PROJECTILE_H
