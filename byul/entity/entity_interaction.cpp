#include "entity_interaction.h"
#include "float_common.h"
#include <math.h>

void entity_interact_apply_force(
    entity_dynamic_t* target,
    const vec3_t* force,
    float dt)
{
    if (!target || !force || dt <= 0.0f || target->props.mass <= 0.0f)
        return;

    vec3_t accel;
    vec3_assign(&accel, force);
    vec3_scale(&accel, &accel, 1.0f / target->props.mass);

    vec3_t delta_v;
    vec3_scale(&delta_v, &accel, dt);
    vec3_add(&target->velocity, &target->velocity, &delta_v);
}

bool entity_interact_check_collision(
    const entity_dynamic_t* a,
    const entity_dynamic_t* b,
    float collision_radius)
{
    if (!a || !b) return false;

    float dist_sq = vec3_distance_sq(&a->xf.pos, &b->xf.pos);
    return dist_sq < (collision_radius * collision_radius);
}

void entity_interact_resolve_bounce(
    entity_dynamic_t* a,
    entity_dynamic_t* b)
{
    if (!a || !b || float_equal(a->props.mass + b->props.mass, 0.0f))
        return;

    vec3_t v1 = a->velocity;
    vec3_t v2 = b->velocity;
    float m1 = a->props.mass;
    float m2 = b->props.mass;

    // Elastic velocity exchange (1D-style approximation)
    vec3_t new_v1, new_v2;
    vec3_scale(&new_v1, &v1, m1 - m2);
    vec3_t temp;
    vec3_scale(&temp, &v2, 2.0f * m2);
    vec3_add(&new_v1, &new_v1, &temp);
    vec3_scale(&new_v1, &new_v1, 1.0f / (m1 + m2));

    vec3_scale(&new_v2, &v2, m2 - m1);
    vec3_scale(&temp, &v1, 2.0f * m1);
    vec3_add(&new_v2, &new_v2, &temp);
    vec3_scale(&new_v2, &new_v2, 1.0f / (m1 + m2));

    a->velocity = new_v1;
    b->velocity = new_v2;
}

bool entity_interact_within_aoe(
    const entity_dynamic_t* target,
    const vec3_t* origin,
    float radius)
{
    if (!target || !origin || radius <= 0.0f)
        return false;

    float dist_sq = vec3_distance_sq(&target->xf.pos, origin);
    return dist_sq <= radius * radius;
}

bool entity_interact_check_fov(
    const entity_dynamic_t* observer,
    const entity_dynamic_t* target,
    const vec3_t* forward,
    float fov_angle_deg)
{
    if (!observer || !target || !forward || fov_angle_deg <= 0.0f)
        return false;

    vec3_t dir_to_target;
    vec3_sub(&dir_to_target, &target->xf.pos, &observer->xf.pos);
    vec3_normalize(&dir_to_target);

    float dot = vec3_dot(forward, &dir_to_target);
    float angle_rad = acosf(dot);
    float fov_rad = (fov_angle_deg * 3.14159265f) / 180.0f;

    return angle_rad <= fov_rad;
}

void entity_interact_update(
    entity_dynamic_t* self,
    entity_dynamic_t** others,
    int count,
    float dt)
{
    if (!self || !others || count <= 0 || dt <= 0.0f)
        return;

    for (int i = 0; i < count; ++i) {
        entity_dynamic_t* other = others[i];
        if (!other || other == self) continue;

        if (entity_interact_check_collision(self, other, 0.5f)) {
            entity_interact_resolve_bounce(self, other);
        }
    }
} 

float entity_dynamic_distance(
    const entity_dynamic_t* a, const entity_dynamic_t* b) {

    return vec3_distance(&a->xf.pos, &b->xf.pos);
}

bool entity_dynamic_in_contact(
    const entity_dynamic_t* a, const entity_dynamic_t* b, float tolerance) 
{
    float r1 = entity_size(&a->base);
    float r2 = entity_size(&b->base);
    float dist = vec3_distance(&a->xf.pos, &b->xf.pos);
    
    return dist <= (r1 + r2 + tolerance);
}

float entity_dynamic_predict_collision_time(
    const entity_dynamic_t* a,
    const entity_dynamic_t* b)
{
    vec3_t dp, dv;
    vec3_sub(&dp, &a->xf.pos, &b->xf.pos);
    vec3_sub(&dv, &a->velocity, &b->velocity);

    float a_coef = vec3_dot(&dv, &dv);
    if (a_coef < 1e-6f) return 0.0f; // relative velocity 0

    float b_coef = 2.0f * vec3_dot(&dp, &dv);
    float t = -b_coef / (2.0f * a_coef);
    return t < 0.0f ? 0.0f : t;
}

int entity_dynamic_collision_point(
    vec3_t* out,
    const entity_dynamic_t* a,
    const entity_dynamic_t* b)
{
    if (!out || !a || !b) return 0;

    float t = entity_dynamic_predict_collision_time(a, b);
    if (t <= 0.0f) return 0;

    vec3_t pa, pb, va_dt, vb_dt;

    // p1(t) = pos1 + v1 * t
    vec3_scale(&va_dt, &a->velocity, t);
    vec3_add(&pa, &a->xf.pos, &va_dt);

    // p2(t) = pos2 + v2 * t
    vec3_scale(&vb_dt, &b->velocity, t);
    vec3_add(&pb, &b->xf.pos, &vb_dt);

    vec3_lerp(out, &pa, &pb, 0.5f);  // out = 0.5 * (pa + pb)

    return 1;
}

float entity_dynamic_predict_collision_time_env(
    const entity_dynamic_t* a,
    const entity_dynamic_t* b,
    const environ_t* env)
{
    if (!a || !b || !env) return 0.0f;

    linear_state_t state_a, state_b;
    entity_dynamic_calc_state_env(a, env, 1.0f, &state_a);
    entity_dynamic_calc_state_env(b, env, 1.0f, &state_b);

    vec3_t dp, dv;
    vec3_sub(&dp, &state_a.position, &state_b.position);
    vec3_sub(&dv, &state_a.velocity, &state_b.velocity);

    float a_coef = vec3_dot(&dv, &dv);
    if (a_coef < 1e-6f) return 0.0f;

    float b_coef = 2.0f * vec3_dot(&dp, &dv);
    float t = -b_coef / (2.0f * a_coef);
    return t < 0.0f ? 0.0f : t;
}

int entity_dynamic_collision_point_env(
    vec3_t* out,
    const entity_dynamic_t* a,
    const entity_dynamic_t* b,
    const environ_t* env)
{
    if (!out || !a || !b || !env) return 0;

    float t = entity_dynamic_predict_collision_time_env(a, b, env);
    if (t <= 0.0f) return 0;

    vec3_t pa, pb;
    entity_dynamic_calc_position_env(a, env, t, &pa);
    entity_dynamic_calc_position_env(b, env, t, &pb);

    vec3_lerp(out, &pa, &pb, 0.5f);
    return 1;
}
