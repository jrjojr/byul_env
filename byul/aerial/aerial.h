#ifndef AERIAL_H
#define AERIAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include "byul_common.h"
#include "vec3.h"
#include "entity_dynamic.h"
#include "environ.h"
#include "projectile_common.h"
#include "propulsion.h"
#include "guidance.h"

// Forward declaration
typedef struct s_projectile_result projectile_result_t;

//
// Aerial vehicle structure
//
typedef struct s_aerial {
    /**
     * @brief Dynamic entity-based structure.
     * @details Contains physical information such as position, velocity, and rotation.
     */
    entity_dynamic_t base;
    
    propulsion_t propulsion;
    guidance_func guidance;
    void* guidance_userdata;

    float wing_area;         ///< Wing area (m²)
    float lift_coefficient;  ///< Lift coefficient
    float drag_coefficient;  ///< Drag coefficient
} aerial_t;


// ---------------------------------------------------------
// Aerial Vehicle (Aerial)
// ---------------------------------------------------------
BYUL_API void aerial_init(aerial_t* aerial);
BYUL_API void aerial_init_full(
    aerial_t* aerial,
    const vec3_t* initial_pos,     // Initial position
    const vec3_t* initial_velocity,// Initial velocity
    float wing_area,               // Wing area
    float lift_coeff,              // Lift coefficient
    float drag_coeff,              // Drag coefficient
    const propulsion_t* propulsion,// Initial propulsion data
    guidance_func guidance,        // Guidance function
    void* guidance_userdata        // Guidance function user data
);

BYUL_API void aerial_assign(aerial_t* patriot, const aerial_t* src);

BYUL_API bool aerial_launch(
    const aerial_t* aerial,
    const vec3_t* target,
    float initial_speed,
    const environ_t* env,
    projectile_result_t* out    
);

#ifdef __cplusplus
}
#endif

#endif // AERIAL_H
