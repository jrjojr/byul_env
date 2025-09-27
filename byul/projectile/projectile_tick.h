#ifndef PROJECTILE_TICK_H
#define PROJECTILE_TICK_H

#ifdef __cplusplus
extern "C" {
#endif

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
#define MIN_DELTA_TIME       0.002f     // Highest precision (500 Hz)
#define MAX_DELTA_TIME       0.1f       // Lowest precision (10 Hz)

#define XFORM_MAX_DISTANCE   XFORM_MAX_POS  // Target prediction distance



#include "byul_common.h"
#include "projectile_common.h"
#include "trajectory.h"
#include "propulsion.h"
#include "guidance.h"
#include "environ.h"
#include "entity_dynamic.h"
#include "numeq_filters.h"
#include "byul_tick.h"
#include "ground.h"

/**
 * @brief Calculates an appropriate simulation time step (dt) based 
 *  on force and mass.
 *
 * This function estimates the suitable dt for 
 * trajectory simulation to balance
 * accuracy and performance. It uses the initial applied 
 * force and mass to compute
 * acceleration and velocity, then determines dt to 
 * ensure an adequate number of
 * samples across the expected travel distance.
 *
 * @param[out] dt_out   Pointer to the output dt value (in seconds)
 * @param[in]  dir      Direction vector (normalized, 
 *     not used internally but kept for future use)
 * @param[in]  force    Applied initial force (in Newtons)
 * @param[in]  mass     Mass of the projectile (in kilograms)
 */
BYUL_API void calc_suitable_dt(
    float* dt_out, const vec3_t* dir, float force, float mass);

/**
 * @brief Estimates a suitable simulation max time based on 
 * initial force and mass.
 *
 * This function computes the total expected simulation time 
 * (max_time) required
 * for a projectile to travel a predefined distance, 
 * based on its mass and the applied force.
 * It uses estimated velocity to calculate 
 * how long it would take to cover the full range.
 *
 * @param[out] max_time_out Pointer to output value (in seconds)
 * @param[in]  dir          Normalized direction vector (currently unused)
 * @param[in]  force        Applied initial force (Newtons)
 * @param[in]  mass         Mass of the projectile (kilograms)
 */
BYUL_API void calc_suitable_max_time(
    float* max_time_out, const vec3_t* dir, float force, float mass);    

typedef struct s_projectile_tick {
    projectile_t proj;
    entity_dynamic_t target;

    integrator_t intgr;

    environ_t* env;    
    ground_t* ground;
    propulsion_t* propulsion;
    guidance_func guidance_fn;

    bool bool_debug;    
    trajectory_t* trajectory;

    vec3_t impact_pos;
    float impact_time;
    bool bool_impacted;

    tick_t* tick;

} projectile_tick_t;

BYUL_API void projectile_tick_init(projectile_tick_t* prt);

/*
env, propulsion, guidance_fn nullptr
*/
BYUL_API void projectile_tick_init_full(projectile_tick_t* prt,
    const projectile_t* proj, 

    const entity_dynamic_t* target, 

    const environ_t* env,
    const ground_t* ground,
    const propulsion_t* propulsion,
    const guidance_func guidance_fn,

    bool bool_debug
);

BYUL_API void projectile_tick_free(projectile_tick_t* prt);

BYUL_API void projectile_tick_assign(
    projectile_tick_t* out, const projectile_tick_t* src);

BYUL_API bool projectile_tick_prepare(
    projectile_tick_t* prt,
    tick_t* tk);

BYUL_API bool projectile_tick_prepare_full(
    projectile_tick_t* prt,
    const entity_dynamic_t* target,
    tick_t* tk);    

BYUL_API bool projectile_tick(
    projectile_tick_t* prt,
    float dt
);

BYUL_API bool projectile_tick_complete(
    projectile_tick_t* prt,
    tick_t* tk);

#ifdef __cplusplus
}
#endif

#endif // PROJECTILE_TICK_H
