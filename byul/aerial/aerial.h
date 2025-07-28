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

// 전방 선언
typedef struct s_projectile_result projectile_result_t;

//
typedef struct s_aerial {
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
} aerial_t;


// ---------------------------------------------------------
// 항공기 (Aerial)
// ---------------------------------------------------------
BYUL_API void aerial_init(aerial_t* aerial);
BYUL_API void aerial_init_full(
    aerial_t* aerial,
    const vec3_t* initial_pos,     // 초기 위치
    const vec3_t* initial_velocity,// 초기 속도
    float wing_area,               // 날개 면적
    float lift_coeff,              // 양력 계수
    float drag_coeff,              // 항력 계수
    const propulsion_t* propulsion,// 추진기 초기값
    guidance_func guidance,        // 유도 함수
    void* guidance_userdata        // 유도 함수 데이터
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
