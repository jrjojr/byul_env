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
* @brief High-speed strategic projectile simulation settings
*
* This configuration defines the base resolution for projectile trajectory simulation.
* It supports a wide range of speeds, from Mach 3 (supersonic missiles)
* up to Mach 25 (strategic ICBMs).
*
* - Target distance: 99999.0 meters
* - Total simulation time: 100.0 seconds
* - Sample count: 2048 (dt ~= 0.0488 seconds)
*
* @section VelocityReference Speed Reference
* - Average speed: 999.99 m/s
* - Speed in km/h: approximately 3600 km/h
* - In terms of Mach: Mach 2.92 (based on sea-level standard of 343 m/s)
*
* @section VelocityGrades Speed Grades
* - Mach 3 or higher: High-speed missile class
* - Mach 5 or higher: Hypersonic missile (e.g., DF-17, HGV)
* - Mach 20 or higher: Strategic ICBM class (e.g., Trident II, Avangard)
*
* @section ExtendedSimulations Examples of Extended Configurations
* - For Mach 5:   60 sec / 2048 samples / dt ~= 0.0293 sec
* - For Mach 20:  15 sec / 3000 samples / dt ~= 0.0050 sec
* - For Mach 25:  12 sec / 4096 samples / dt ~= 0.00293 sec
*
* @note The faster the object, the shorter the simulation time
*       and the higher the number of samples required
*       for precise trajectory resolution.
*
* @section VisualPerception Visual Perception in Byul's World
* In the context of Byul's World, the most natural and comfortable speed
* for human perception is around 3.6 to 4.0 km/h, which equals 1.0 m/s.
*
* @subsection VisualScales Monitor Resolution & Pixel Mapping
* - 10 px = 1 m (1 px = 10 cm) -> 1.0 m/s = 10 px/sec
* - 100 px = 1 m (1 px = 1 cm) -> 1.0 m/s = 100 px/sec
* - 1000 px = 1 m (1 px = 1 mm) -> 1.0 m/s = 1000 px/sec
*
* @subsection PracticalView Realistic Viewing Speed
* - On most monitors (e.g., 1920x1080), 50 ~ 100 px/sec is the ideal speed
*   for smooth and visible movement.
* - Therefore, "1.0 m/s (3.6 km/h)" equals about 100 px/sec with
*   100 px/m resolution and feels like natural walking speed.
*/
#define MAX_SAMPLE_COUNT     2048
#define MIN_SIM_TIME         1.0f
#define MAX_SIM_TIME         100.0f    // 100 seconds
#define DELTA_TIME           (MAX_SIM_TIME / MAX_SAMPLE_COUNT)   // ~= 0.048828125 seconds

     // #define XFORM_MAX_DISTANCE   99999.0f   // Target prediction distance
#define XFORM_MAX_DISTANCE   XFORM_MAX_POS  // Target prediction distance
#define MIN_DELTA_TIME       0.002f     // Highest precision (500 Hz)
#define MAX_DELTA_TIME       0.1f       // Lowest precision (10 Hz)

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
    const vec3_t* dir,
	float initial_force_scalar,
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
    const vec3_t* dir,
    float initial_force_scalar,
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
    rocket_t* rocket,
    const vec3_t* target,
    float initial_force_scalar,
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
    float initial_force_scalar,
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
    float initial_force_scalar,
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
