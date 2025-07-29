#include "entity_spring.h"
#include "vec3.h"
#include <math.h>
#include <stdlib.h>

static inline void record_trajectory(trajectory_t* traj,
                                     entity_dynamic_t* e,
                                     int count,
                                     float time)
{
    if (!traj) return;
    for (int i = 0; i < count; ++i) {
        motion_state_t ms;
        entity_dynamic_to_motion_state(&e[i], &ms, NULL, NULL);
        trajectory_add_sample(traj, time, &ms);
    }
}

void spring_force(vec3_t* out,
                  const entity_dynamic_t* a,
                  const entity_dynamic_t* b,
                  float k, float c, float L0)
{
    vec3_zero(out);
    if (!a || !b || k <= 0.0f) return;

    vec3_t dir;
    vec3_sub(&dir, &b->xf.pos, &a->xf.pos);
    float d = vec3_length(&dir);
    if (d <= 1e-6f) return;

    vec3_scale(&dir, &dir, 1.0f / d);

    // Hooke's Law
    float x = d - L0;
    vec3_scale(out, &dir, k * x);

    vec3_t v_rel;
    vec3_sub(&v_rel, &b->velocity, &a->velocity);
    float v_proj = vec3_dot(&v_rel, &dir);
    vec3_madd(out, out, &dir, -c * v_proj);
}

void spring_force_total(vec3_t* out,
                        const entity_dynamic_t* self,
                        const entity_dynamic_t* others,
                        int count,
                        float k, float c, float L0)
{
    vec3_zero(out);
    if (!self || !others || count <= 0) return;

    for (int i = 0; i < count; ++i) {
        if (&others[i] == self) continue;
        vec3_t f;
        spring_force(&f, self, &others[i], k, c, L0);
        vec3_iadd(out, &f);
    }
}

static inline void apply_damping(vec3_t* velocity, float c, float dt)
{
    if (!velocity) return;
    float damping = expf(-c * dt);
    vec3_iscale(velocity, damping);
}

// =========================================================
// (Forward Euler)
// =========================================================
void spring_simulate(trajectory_t* traj,
                     entity_dynamic_t* e, int count,
                     float dt, float k, float c, float L0,
                     int steps)
{
    if (!e || count <= 0 || dt <= 0.0f || steps <= 0) return;

    float time = 0.0f;

    for (int step = 0; step < steps; ++step) {
        for (int i = 0; i < count; ++i) {
            vec3_t force;
            spring_force_total(&force, &e[i], e, count, k, c, L0);

            float inv_mass = (e[i].props.mass > 0.0f) ? 1.0f / e[i].props.mass : 0.0f;

            // v = v + (F/m) * dt
            vec3_madd(&e[i].velocity, &e[i].velocity, &force, inv_mass * dt);

            apply_damping(&e[i].velocity, c, dt);

            // p = p + v * dt
            vec3_madd(&e[i].xf.pos, &e[i].xf.pos, &e[i].velocity, dt);
        }

        record_trajectory(traj, e, count, time);
        time += dt;
    }
}

// =========================================================
// Pairwise Spring (Semi-Implicit Euler)
// =========================================================
void spring_simulate_pairwise(trajectory_t* traj,
                              entity_dynamic_t* e, int count,
                              float dt, float k, float c, float L0,
                              int steps)
{
    if (!e || count <= 1 || dt <= 0.0f || steps <= 0) return;

    vec3_t* forces = (vec3_t*)calloc(count, sizeof(vec3_t));
    if (!forces) return;

    float time = 0.0f;

    for (int step = 0; step < steps; ++step) {

        for (int i = 0; i < count; ++i)
            vec3_zero(&forces[i]);

        for (int i = 0; i < count; ++i) {
            for (int j = i + 1; j < count; ++j) {
                vec3_t f;
                spring_force(&f, &e[i], &e[j], k, 0.0f, L0);
                vec3_iadd(&forces[i], &f);
                vec3_isub(&forces[j], &f);
            }
        }

        for (int i = 0; i < count; ++i) {
            float inv_mass = (e[i].props.mass > 0.0f) ? 1.0f / e[i].props.mass : 0.0f;
            vec3_madd(&e[i].velocity, &e[i].velocity, &forces[i], inv_mass * dt);
            apply_damping(&e[i].velocity, c, dt);
            vec3_madd(&e[i].xf.pos, &e[i].xf.pos, &e[i].velocity, dt);
        }

        record_trajectory(traj, e, count, time);
        time += dt;
    }

    free(forces);
}

static void compute_network_forces(entity_dynamic_t* e, int count,
                                   const spring_link_t* links, int link_count,
                                   vec3_t* forces)
{
    for (int i = 0; i < count; ++i)
        vec3_zero(&forces[i]);

    for (int l = 0; l < link_count; ++l) {
        int i = links[l].i;
        int j = links[l].j;
        if (i < 0 || i >= count || j < 0 || j >= count)
            continue;

        vec3_t f;
        spring_force(&f, &e[i], &e[j], links[l].k, links[l].c, links[l].L0);

        vec3_iadd(&forces[i], &f);
        vec3_isub(&forces[j], &f);
    }
}

// =========================================================
// (Velocity Verlet)
// =========================================================
void spring_simulate_network(trajectory_t* traj,
                             entity_dynamic_t* e, int count,
                             const spring_link_t* links, int link_count,
                             float dt, int steps)
{
    if (!e || count <= 1 || !links || link_count <= 0 || dt <= 0.0f || steps <= 0)
        return;

    vec3_t* forces = (vec3_t*)calloc(count, sizeof(vec3_t));
    if (!forces) return;

    float time = 0.0f;

    for (int step = 0; step < steps; ++step) {

        compute_network_forces(e, count, links, link_count, forces);

        // 2. v_half = v + 0.5 * a * dt
        for (int i = 0; i < count; ++i) {
            float inv_mass = (e[i].props.mass > 0.0f) ? 1.0f / e[i].props.mass : 0.0f;
            vec3_madd(&e[i].velocity, &e[i].velocity, &forces[i], 0.5f * inv_mass * dt);
        }

        for (int i = 0; i < count; ++i) {
            vec3_madd(&e[i].xf.pos, &e[i].xf.pos, &e[i].velocity, dt);
        }

        // 4. force(t + dt)
        compute_network_forces(e, count, links, link_count, forces);

        // 5. v(t + dt) = v_half + 0.5 * a(t+dt) * dt
        for (int i = 0; i < count; ++i) {
            float inv_mass = (e[i].props.mass > 0.0f) ? 1.0f / e[i].props.mass : 0.0f;
            vec3_madd(&e[i].velocity, &e[i].velocity, &forces[i], 0.5f * inv_mass * dt);

            vec3_iscale(&e[i].velocity, 0.99f);
        }

        record_trajectory(traj, e, count, time);
        time += dt;
    }

    free(forces);
}

// =========================================================
// Repulsion  ( Velocity Verlet)
// =========================================================
void repulsion_simulate_network(
    trajectory_t* traj,
    entity_dynamic_t* e, int count,
    const spring_link_t* links, int link_count,
    float dt, int steps)
{
    if (!e || count <= 1 || !links || link_count <= 0 || dt <= 0.0f || steps <= 0)
        return;

    vec3_t* forces = (vec3_t*)calloc(count, sizeof(vec3_t));
    if (!forces) return;

    for (int step = 0; step < steps; ++step) {

        for (int i = 0; i < count; ++i)
            vec3_zero(&forces[i]);

        for (int l = 0; l < link_count; ++l) {
            int i = links[l].i;
            int j = links[l].j;
            if (i < 0 || i >= count || j < 0 || j >= count) continue;

            vec3_t dir;
            vec3_sub(&dir, &e[i].xf.pos, &e[j].xf.pos);
            float dist = vec3_length(&dir);
            if (dist <= 1e-6f) continue;

            vec3_scale(&dir, &dir, 1.0f / dist);

            float diff = links[l].L0 - dist;
            if (diff > 0.0f) {
                float F = links[l].k * diff - links[l].c * vec3_dot(&dir, &e[i].velocity);
                if (F > 0.0f) { 
                    vec3_t f; vec3_scale(&f, &dir, F);
                    vec3_add(&forces[i], &forces[i], &f);
                    vec3_sub(&forces[j], &forces[j], &f);
                }
            }
        }

        // 3. Velocity Verlet
        for (int i = 0; i < count; ++i) {
            float inv_mass = (e[i].props.mass > 0.0f) ? 1.0f / e[i].props.mass : 0.0f;

            vec3_t accel;
            vec3_scale(&accel, &forces[i], inv_mass);

            // pos(t + dt) = pos(t) + v(t)*dt + 0.5*a*dt²
            vec3_t half_step;
            vec3_scale(&half_step, &accel, 0.5f * dt * dt);
            vec3_add(&e[i].xf.pos, &e[i].xf.pos, &half_step);
            vec3_madd(&e[i].xf.pos, &e[i].xf.pos, &e[i].velocity, dt);

            // v(t + dt/2)
            vec3_madd(&e[i].velocity, &e[i].velocity, &accel, 0.5f * dt);
        }

        // 4. (Verlet )
        for (int i = 0; i < count; ++i) {
            float inv_mass = (e[i].props.mass > 0.0f) ? 1.0f / e[i].props.mass : 0.0f;

            vec3_t accel;
            vec3_scale(&accel, &forces[i], inv_mass);
            vec3_madd(&e[i].velocity, &e[i].velocity, &accel, 0.5f * dt);
        }

        if (traj) {
            record_trajectory(traj, e, count, step * dt);
        }
    }

    free(forces);
}

void push_simulate_network(trajectory_t* traj,
                                    entity_dynamic_t* e, int count,
                                    const vec3_t* push_forces,
                                    float dt, int steps)
{
    if (!e || count <= 0 || !push_forces || dt <= 0.0f || steps <= 0)
        return;

    for (int step = 0; step < steps; ++step) {
        float time = step * dt;

        for (int i = 0; i < count; ++i) {
            if (e[i].props.mass <= 0.0f) continue;

            // a = F/m
            vec3_t accel;
            vec3_scale(&accel, &push_forces[i], 1.0f / e[i].props.mass);

            vec3_t delta_v;
            vec3_scale(&delta_v, &accel, dt);
            vec3_add(&e[i].velocity, &e[i].velocity, &delta_v);

            // (Verlet : x += v * dt)
            vec3_t delta_x;
            vec3_scale(&delta_x, &e[i].velocity, dt);
            vec3_add(&e[i].xf.pos, &e[i].xf.pos, &delta_x);

            if (traj) record_trajectory(traj, &e[i], steps, step * dt);
        }
    }
}
