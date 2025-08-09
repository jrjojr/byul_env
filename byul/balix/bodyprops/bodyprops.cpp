#include "bodyprops.h"
#include "numeq_solver.h"

void bodyprops_init(bodyprops_t* body) {
    if (!body) return;

    body->mass = 1.0f;
    body->drag_coef = 0.47f;
    body->cross_section = 0.01f;
    body->restitution = 0.5f;
    body->friction = 0.1f;

    body->k_magnus = 0.2f;
    body->k_gyro = 0.05f;
}

void bodyprops_init_full(bodyprops_t* body,
                         float mass,
                         float drag_coef,
                         float cross_section,
                         float restitution,
                         float friction,
                         float k_magnus,
                         float k_gyro) {
    if (!body) return;

    body->mass = mass;
    body->drag_coef = drag_coef;
    body->cross_section = cross_section;
    body->restitution = restitution;
    body->friction = friction;
    body->k_magnus = k_magnus;
    body->k_gyro = k_gyro;
}

void bodyprops_assign(bodyprops_t* out, const bodyprops_t* src) {
    if (!out || !src) return;
    *out = *src;
}

void bodyprops_apply_friction(vec3_t* velocity,
                              const bodyprops_t* body,
                              float time)
{
    if (!velocity || !body || time <= 0.0f) return;

    float v0 = vec3_length(velocity);
    if (v0 <= 1e-5f) {
        *velocity = vec3_t{0, 0, 0};
        return;
    }

    float factor = 1.0f - body->friction * time;
    if (factor >= 0.0f) {
        vec3_scale(velocity, velocity, factor);
    } else {
        float t_stop = 0.0f;
        if (numeq_solve_linear(body->friction, -1.0f, &t_stop) && t_stop > 0.0f) {
            *velocity = vec3_t{0, 0, 0};
        }
    }
}

float bodyprops_apply_friction_heat(vec3_t* velocity,
                                    const bodyprops_t* body,
                                    float time) {
    if (!velocity || !body || time <= 0.0f) return 0.0f;

    float v_prev = vec3_length(velocity);

    float factor = 1.0f - body->friction * time;
    if (factor < 0.0f) factor = 0.0f;
    vec3_scale(velocity, velocity, factor);

    float v_new = vec3_length(velocity);
    float delta_ke = 0.5f * body->mass * (v_prev * v_prev - v_new * v_new);
    return (delta_ke > 0.0f) ? delta_ke : 0.0f;
}

float bodyprops_estimate_stop_time(const vec3_t* velocity,
                                   const bodyprops_t* body)
{
    if (!velocity || !body || body->friction <= 0.0f)
        return 0.0f;

    float v_mag = vec3_length(velocity);
    if (v_mag <= 1e-5f)
        return 0.0f;

    return 1.0f / body->friction;
}
