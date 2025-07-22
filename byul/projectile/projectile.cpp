#include <iostream>
#include "internal/projectile.h"
#include <math.h>

// ---------------------------------------------------------
// 초기화 함수
// ---------------------------------------------------------
void projectile_init(projectile_t* proj)
{
    if (!proj) return;

    entity_dynamic_init(&proj->base);
    proj->type = PROJECTILE_TYPE_SHELL;
    proj->on_hit = NULL;
    proj->hit_userdata = NULL;
}

void projectile_init_full(
    projectile_t* proj,
    const entity_dynamic_t* base,
    projectile_type_t type,
    projectile_hit_cb on_hit,
    void* hit_userdata
)
{
    if (!proj) return;

    if (base) {
        entity_dynamic_assign(&proj->base, base);
    } else {
        entity_dynamic_init(&proj->base);
    }

    proj->type = type;
    proj->on_hit = on_hit;
    proj->hit_userdata = hit_userdata;
}

void projectile_assign(projectile_t* out, const projectile_t* src)
{
    if (!out || !src) return;
    *out = *src;
}

// ---------------------------------------------------------
// 업데이트 함수
// ---------------------------------------------------------
void projectile_update(projectile_t* proj, float dt)
{
    if (!proj || dt <= 0.0f) return;

    entity_dynamic_update(&proj->base, dt);

    // 수명 체크 후 on_hit 호출
    if (proj->base.base.lifetime > 0.0f &&
        proj->base.base.age >= proj->base.base.lifetime)
    {
        if (proj->on_hit) {
            proj->on_hit(proj, proj->hit_userdata);
        }
    }
}

void projectile_default_hit_cb(const projectile_t* proj, void* userdata)
{
    (void)proj;
    (void)userdata;
    std::cout << "[projectile] default hit cb (no effect)\n";
}

bool projectile_compute_launch(
    comp_result_t* out,
    const projectile_t* proj,
    const vec3_t* target,
    float initial_force
) {
    if (!out || !proj || !target) return false;

    vec3_t start;
    xform_get_position(&proj->base.xf, &start);
    vec3_t diff;
    vec3_sub(&diff, target, &start);

    float g = 9.8f; // 중력 가속도 (m/s²)
    float R = sqrtf(diff.x * diff.x + diff.z * diff.z);
    if (R < 1e-6f) return false;

    float Dy = diff.y;

    // 초기 속도 크기 계산: F = m * a, v0 = sqrt(2 * a * R)
    float mass = proj->base.props.mass > 1e-6f ? proj->base.props.mass : 1.0f;
    float a0 = initial_force / mass;
    float v0 = sqrtf(2.0f * a0 * R);

    // 발사각 계산
    float under_sqrt = v0*v0*v0*v0 - g * (g*R*R + 2*Dy*v0*v0);
    if (under_sqrt < 0.0f) return false; // 도달 불가

    float theta = atanf((v0*v0 - sqrtf(under_sqrt)) / (g * R));

    // 수평 방향 단위 벡터
    vec3_t dir = { diff.x / R, 0, diff.z / R };

    // 초기 속도 벡터 계산
    out->vec.x = v0 * cosf(theta) * dir.x;
    out->vec.y = v0 * sinf(theta);
    out->vec.z = v0 * cosf(theta) * dir.z;

    // 예상 도달 시간
    out->dt = R / (v0 * cosf(theta));
    return true;
}

bool projectile_compute_launch_env(
    comp_result_t* out,
    const projectile_t* proj,
    const environ_t* env,
    const vec3_t* target,
    float initial_force
) {
    if (!out || !proj || !env || !target) return false;

    vec3_t start;
    xform_get_position(&proj->base.xf, &start);
    vec3_t diff;
    vec3_sub(&diff, target, &start);

    float R = sqrtf(diff.x * diff.x + diff.z * diff.z);
    if (R < 1e-6f) return false;

    float Dy = diff.y;
    float mass = proj->base.props.mass > 1e-6f ? proj->base.props.mass : 1.0f;
    float a0 = initial_force / mass;  // 초기 가속도
    float g = fabsf(env->gravity.y) > 1e-6f ? fabsf(env->gravity.y) : 9.8f;

    // 초기 속도 크기 (단순 모델)
    float v0 = sqrtf(2.0f * a0 * R);

    // 발사각 계산 (중력 반영)
    float under_sqrt = v0*v0*v0*v0 - g * (g * R * R + 2 * Dy * v0 * v0);
    if (under_sqrt < 0.0f) return false;

    float theta = atanf((v0*v0 - sqrtf(under_sqrt)) / (g * R));

    // 수평 단위 벡터
    vec3_t dir = { diff.x / R, 0, diff.z / R };

    // 초기 속도 벡터 + 바람 보정
    out->vec.x = v0 * cosf(theta) * dir.x + env->wind.x;
    out->vec.y = v0 * sinf(theta) + env->wind.y;
    out->vec.z = v0 * cosf(theta) * dir.z + env->wind.z;

    // 도착 시간
    float v_h = v0 * cosf(theta) + 
        sqrtf(env->wind.x * env->wind.x + env->wind.z * env->wind.z);

    out->dt = R / (v_h > 1e-3f ? v_h : 1e-3f);

    return true;
}
