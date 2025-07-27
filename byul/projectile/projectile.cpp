#include "internal/projectile.h"
#include "internal/projectile_predict.h"
#include "internal/propulsion.h"
#include "internal/guidance.h"
#include "internal/entity_dynamic.h"
#include <math.h>    // sqrtf 등

// ---------------------------------------------------------
// 기본 발사체
// ---------------------------------------------------------
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

    // 환경 영향 없음
    return projectile_predict(out, proj, &entdyn, 5.0f, 0.01f, 
        NULL, NULL, NULL);
}
