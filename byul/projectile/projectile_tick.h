#ifndef PROJECTILE_TICK_H
#define PROJECTILE_TICK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "byul_common.h"
#include "projectile_common.h"
#include "trajectory.h"
#include "propulsion.h"
#include "guidance.h"
#include "environ.h"
#include "entity_dynamic.h"
#include "numeq_filters.h"
#include "byul_tick.h"

typedef struct s_projectile_tick {
    projectile_t proj;
    // motion_state_t state;
    entity_dynamic_t target;

    integrator_t* intgr;

    environ_t* env;    
    propulsion_t* propulsion;
    guidance_func guidance_fn;

    trajectory_t* trajectory;   // ← 궤적 저장 위치 (optional)
    float elapsed;              // ← 누적 시간 (초)
    float max_time;             // 최대 시간 (초)

    vec3_t impact_pos; // 결과가 없으면 nullptr
    float impact_time;

} projectile_tick_t;

BYUL_API void projectile_tick_init(projectile_tick_t* prt);

/*
proj, state, target, intgr은 깊은복사,
env, propulsion, guidance_fn, 은 깊은 복사 이지만 nullptr 가능
*/
BYUL_API void projectile_tick_init_full(projectile_tick_t* prt,
    const projectile_t* proj, 
    // const motion_state_t* state,
    const entity_dynamic_t* target, integrator_t* intgr, 

    const environ_t* env,
    const propulsion_t* propulsion,
    const guidance_func guidance_fn);

// 내부에서 메모리 할당된 변수들만 해제한다.
// prt는 건들지 않는다 메모리 해제하지 않는다.
BYUL_API void projectile_tick_free(projectile_tick_t* prt);

// 깊은 복사한다.
BYUL_API void projectile_tick_assign(
    projectile_tick_t* out, const projectile_tick_t* src);

BYUL_API bool projectile_tick_prepare(
    const projectile_tick_t* prt,
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
