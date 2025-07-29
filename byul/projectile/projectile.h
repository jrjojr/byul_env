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

/**
 * @brief Calculates the trajectory of a general projectile (Shell class) and predicts collision.
 *
 * Suitable for **simple projectiles** such as stones, arrows, or crossbow bolts.
 *
 * @param[in]  projectile    Projectile information
 * @param[in]  target        Target coordinates
 * @param[in]  initial_speed Initial launch speed
 * @param[in]  env           Environment data (gravity, wind, etc., ignored if NULL)
 * @param[out] out           Trajectory and collision result
 * @retval true  Reaches the target or collision occurred
 * @retval false Cannot reach the target within simulation time
 */
BYUL_API bool projectile_launch(
    const projectile_t* projectile,
    const vec3_t* target,
    float initial_speed,
    const environ_t* env,
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
    const vec3_t* target,
    float initial_speed,
    const environ_t* env,
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
    const rocket_t* rocket,
    const vec3_t* target,
    float initial_speed,
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
    const missile_t* missile,
    const vec3_t* target,
    float initial_speed,
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
    const patriot_t* patriot,
    const entity_dynamic_t* target,
    float initial_speed,
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
