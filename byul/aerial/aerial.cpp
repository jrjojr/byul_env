#include "projectile.h"
#include "projectile_predict.h"
#include "propulsion.h"
#include "guidance.h"
#include "entity_dynamic.h"
#include <math.h>    // sqrtf

typedef struct s_shell_projectile {
    projectile_t proj;
    float explosion_radius;
} shell_projectile_t;

typedef struct s_rocket {
    shell_projectile_t base;
    propulsion_t propulsion;
} rocket_t;

typedef struct s_missile {
    rocket_t base;
    guidance_func guidance;        ///< guidance_point / guidance_lead
    void* guidance_userdata;       ///< vector target
} missile_t;

typedef struct s_patriot {
    missile_t base;
    guidance_func guidance;        ///< guidance_predict_accel / accel_env
    void* guidance_userdata;       ///< entity target
} patriot_t;

typedef struct s_aerial_vehicle {
    entity_dynamic_t base;
    
    propulsion_t propulsion;
    guidance_func guidance;
    void* guidance_userdata;

    float wing_area;
    float lift_coefficient;
    float drag_coefficient;
} aerial_vehicle_t;

bool projectile_launch(
    const projectile_t* proj,
    const vec3_t* target,
    float initial_force,
    projectile_result_t* out)
{
    if (!proj || !target || !out) return false;

    entity_dynamic_t entdyn;
    entity_dynamic_init(&entdyn);
    entdyn.xf.pos = *target;

    return projectile_predict(out, proj, &entdyn, 5.0f, 0.01f, 
        NULL, NULL, NULL);
}
