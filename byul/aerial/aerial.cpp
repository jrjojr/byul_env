#include "projectile.h"
#include "projectile_predict.h"
#include "propulsion.h"
#include "guidance.h"
#include "entity_dynamic.h"
#include <math.h>    // sqrtf 등

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

typedef struct s_aerial_vehicle {
    /**
     * @brief 동적 엔티티 기반 구조체.
     * @details 위치, 속도, 회전 등 물리 정보를 포함합니다.
     */
    entity_dynamic_t base;
    
    propulsion_t propulsion;
    guidance_func guidance;
    void* guidance_userdata;

    float wing_area;
    float lift_coefficient;
    float drag_coefficient;
} aerial_vehicle_t;


// ---------------------------------------------------------
// 기본 발사체
// ---------------------------------------------------------
bool projectile_launch(
    const projectile_t* proj,
    const vec3_t* target,
    float initial_speed,
    projectile_result_t* out)
{
    if (!proj || !target || !out) return false;

    entity_dynamic_t entdyn;
    entity_dynamic_init(&entdyn);
    entdyn.xf.pos = *target;

    // 환경 영향 없음
    return projectile_predict(out, proj, &entdyn, 5.0f, 0.01f, 
        NULL, NULL, NULL);
}
