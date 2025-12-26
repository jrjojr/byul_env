#include <iostream>
#include "projectile_core.h"
#include <math.h>

void projectile_init(projectile_t* proj)
{
    if (!proj) return;

    entity_dynamic_init(&proj->base);
    proj->on_hit = projectile_default_hit_cb;
    proj->hit_userdata = NULL;
    proj->damage = 1.0f;
    proj->base.base.lifetime = 60.0f;
}

void projectile_init_full(
    projectile_t* proj,
    const entity_dynamic_t* base,
    projectile_attr_t attrs,
    float damage,
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

    proj->attrs = attrs;
    proj->damage = damage;
    proj->on_hit = on_hit;
    proj->hit_userdata = hit_userdata;
}

void projectile_assign(projectile_t* out, const projectile_t* src)
{
    if (!out || !src) return;
    *out = *src;
}

void projectile_update(projectile_t* proj, float dt)
{
    if (!proj || dt <= 0.0f) return;

    entity_dynamic_update(&proj->base, dt);

    if (proj->base.base.lifetime > 0.0f &&
        proj->base.base.age >= proj->base.base.lifetime)
    {
        if (proj->on_hit) {
            proj->on_hit(proj, proj->hit_userdata);
        }
    }
}

void projectile_default_hit_cb(const projectile_t* projectile, void* userdata)
{
    (void)userdata;

    const projectile_t* proj = projectile;
    if (!proj) {
        printf("[projectile] hit callback called with null projectile\n");
        return;
    }

    printf("[projectile] default hit cb damaged : %.2f\n", proj->damage);
}

void projectile_default_expire_cb(
    const projectile_t* projectile, void* userdata)
{
    (void)userdata;

    const projectile_t* proj = projectile;
    if (!proj) {
        printf("[projectile] expire callback called with null projectile\n");
        return;
    }

    printf("[projectile] lifetime expired without collision. damage : %.2f\n",
           proj->damage);
}

static inline float projectile_safe_mass(const projectile_t* proj) {
    return (proj && proj->base.props.mass > 1e-6f) 
    ? proj->base.props.mass : 1.0f;
}

static inline bool projectile_calc_horizontal(
    vec3_t* dir_out, 
    float* R_out, 
    const vec3_t* start, 
    const vec3_t* target) {

    vec3_t diff;
    vec3_sub(&diff, target, start);

    float R = sqrtf(diff.x * diff.x + diff.z * diff.z);
    if (R < 1e-6f) return false;

    if (dir_out) {
        dir_out->x = diff.x / R;
        dir_out->y = 0.0f;
        dir_out->z = diff.z / R;
    }
    if (R_out) *R_out = R;
    return true;
}

bool projectile_calc_launch_param(
    launch_param_t* out,
    const projectile_t* proj,
    const vec3_t* target,
    float initial_force_scalar)
{
    if (!out || !proj || !target) return false;

    vec3_t start;
    xform_get_position(&proj->base.xf, &start);

    vec3_t dir;
    float R;
    if (!projectile_calc_horizontal(&dir, &R, &start, target)) return false;

    float Dy = target->y - start.y;
    float mass = projectile_safe_mass(proj);
    float a0 = initial_force_scalar / mass;
    float v0 = sqrtf(2.0f * a0 * R);
    float g = 9.8f;

    float under_sqrt = v0 * v0 * v0 * v0 - g * (g * R * R + 2 * Dy * v0 * v0);
    if (under_sqrt < 0.0f) return false;

    float theta = atanf((v0 * v0 - sqrtf(under_sqrt)) / (g * R));

    out->direction.x = cosf(theta) * dir.x;
    out->direction.y = sinf(theta);
    out->direction.z = cosf(theta) * dir.z;
    vec3_normalize(&out->direction);

    out->force = initial_force_scalar;
    out->time_to_hit = R / (v0 * cosf(theta));
    return true;
}

bool projectile_calc_launch_param_env(
    launch_param_t* out,
    const projectile_t* proj,
    const environ_t* env,
    const vec3_t* target,
    float initial_force_scalar)
{
    if (!out || !proj || !env || !target) return false;

    vec3_t start;
    xform_get_position(&proj->base.xf, &start);

    vec3_t dir;
    float R;
    if (!projectile_calc_horizontal(&dir, &R, &start, target)) return false;

    float Dy = target->y - start.y;
    float mass = projectile_safe_mass(proj);
    float a0 = initial_force_scalar / mass;
    float g = fabsf(env->gravity.y) > 1e-6f ? fabsf(env->gravity.y) : 9.8f;

    float v0 = sqrtf(2.0f * a0 * R);
    float under_sqrt = v0 * v0 * v0 * v0 - g * (g * R * R + 2 * Dy * v0 * v0);
    if (under_sqrt < 0.0f) return false;

    float theta = atanf((v0 * v0 - sqrtf(under_sqrt)) / (g * R));

    out->direction.x = cosf(theta) * dir.x;
    out->direction.y = sinf(theta);
    out->direction.z = cosf(theta) * dir.z;
    vec3_normalize(&out->direction);

    float wind_h 
    = sqrtf(env->wind_vel.x * env->wind_vel.x + env->wind_vel.z * env->wind_vel.z);
    float v_h = v0 * cosf(theta) + wind_h;
    out->force = initial_force_scalar;
    out->time_to_hit = R / (v_h > 1e-3f ? v_h : 1e-3f);

    return true;
}

bool projectile_calc_launch_param_inverse(
    launch_param_t* out,
    const projectile_t* proj,
    const vec3_t* target,
    float hit_time)
{
    if (!out || !proj || !target || hit_time <= 0.0f) return false;

    vec3_t start;
    xform_get_position(&proj->base.xf, &start);

    vec3_t delta;
    vec3_sub(&delta, target, &start);

    vec3_t gravity_term = { 0.0f, -0.5f * 9.81f * hit_time * hit_time, 0.0f };

    vec3_t required_vel = {
        (delta.x - gravity_term.x) / hit_time,
        (delta.y - gravity_term.y) / hit_time,
        (delta.z - gravity_term.z) / hit_time
    };

    float mass = projectile_safe_mass(proj);
    float required_force = mass * vec3_length(&required_vel);

    vec3_unit(&out->direction, &required_vel);
    out->force = required_force;
    out->time_to_hit = hit_time;
    return true;
}

bool projectile_calc_launch_param_inverse_env(
    launch_param_t* out,
    const projectile_t* proj,
    const environ_t* env,
    const vec3_t* target,
    float hit_time)
{
    if (!out || !proj || !env || !target || hit_time <= 0.0f) return false;

    vec3_t start;
    xform_get_position(&proj->base.xf, &start);

    vec3_t delta;
    vec3_sub(&delta, target, &start);

    vec3_t gravity_term = {
        0.0f,
        -0.5f * fabsf(env->gravity.y) * hit_time * hit_time,
        0.0f
    };

    vec3_t required_vel = {
        (delta.x - gravity_term.x - env->wind_vel.x * hit_time) / hit_time,
        (delta.y - gravity_term.y - env->wind_vel.y * hit_time) / hit_time,
        (delta.z - gravity_term.z - env->wind_vel.z * hit_time) / hit_time
    };

    float mass = projectile_safe_mass(proj);
    float required_force = mass * vec3_length(&required_vel);

    vec3_unit(&out->direction, &required_vel);
    out->force = required_force;
    out->time_to_hit = hit_time;
    return true;
}
