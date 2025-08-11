#include <cmath>
#include <cstdio>
#include "projectile_predict.h"
#include "guidance.h"
#include "float_common.h"
#include "numeq_filters.h"
#include <numeq_model_motion.h>

#include "collision.h"

// ---------------------------------------------------------
// projectile_result_create
// ---------------------------------------------------------
projectile_result_t* projectile_result_create() {
    projectile_result_t* res = new projectile_result_t;
    if (!res) return nullptr;

    vec3_zero(&res->start_pos);
    vec3_zero(&res->target_pos);
    vec3_zero(&res->initial_velocity);

    res->impact_time = 0.0f;
    vec3_zero(&res->impact_pos);
    res->bool_impacted = false;

    res->trajectory = trajectory_create();  // default capacity = 100
    if (!res->trajectory) {
        delete res;
        return nullptr;
    }
    return res;
}

// ---------------------------------------------------------
// projectile_result_create_full
// ---------------------------------------------------------
projectile_result_t* projectile_result_create_full(int capacity) {
    if (capacity <= 0) return nullptr;

    projectile_result_t* res = new projectile_result_t;
    if (!res) return nullptr;

    vec3_zero(&res->start_pos);
    vec3_zero(&res->target_pos);
    vec3_zero(&res->initial_velocity);

    res->impact_time = 0.0f;
    vec3_zero(&res->impact_pos);
    res->bool_impacted = false;

    res->trajectory = trajectory_create_full(capacity);
    if (!res->trajectory) {
        delete res;
        return nullptr;
    }
    return res;
}

// ---------------------------------------------------------
// projectile_result_copy
// ---------------------------------------------------------
projectile_result_t* projectile_result_copy(const projectile_result_t* src) {
    if (!src) return nullptr;

    projectile_result_t* res = new projectile_result_t;
    if (!res) return nullptr;

    res->start_pos = src->start_pos;
    res->target_pos = src->target_pos;
    res->initial_velocity = src->initial_velocity;

    res->impact_time = src->impact_time;
    res->impact_pos = src->impact_pos;
    res->bool_impacted = src->bool_impacted;

    res->trajectory = trajectory_copy(src->trajectory);
    if (!res->trajectory) {
        delete res;
        return nullptr;
    }
    return res;
}

// ---------------------------------------------------------
// projectile_result_reset
// ---------------------------------------------------------
void projectile_result_reset(projectile_result_t* res)
{
    if (!res) return;

    vec3_zero(&res->start_pos);
    vec3_zero(&res->target_pos);
    vec3_zero(&res->initial_velocity);

    res->impact_time = 0.0f;
    vec3_zero(&res->impact_pos);
    res->bool_impacted = false;

    if (res->trajectory) {
        trajectory_clear(res->trajectory);
    }
}

// ---------------------------------------------------------
// projectile_result_resize
// ---------------------------------------------------------
void projectile_result_resize(projectile_result_t* res, int new_capacity)
{
    if (!res || new_capacity <= 0) return;

    if (res->trajectory) {
        trajectory_destroy(res->trajectory);
    }
    res->trajectory = trajectory_create_full(new_capacity);

    projectile_result_reset(res);
}

// ---------------------------------------------------------
// projectile_result_free
// ---------------------------------------------------------
void projectile_result_free(projectile_result_t* res)
{
    if (!res) return;

    if (res->trajectory) {
        trajectory_destroy(res->trajectory);
        res->trajectory = NULL;
    }

    vec3_zero(&res->start_pos);
    vec3_zero(&res->target_pos);
    vec3_zero(&res->initial_velocity);

    res->impact_time = 0.0f;
    vec3_zero(&res->impact_pos);
    res->bool_impacted = false;

    delete res;
}

// ---------------------------------------------------------
// projectile_result_destroy
// ---------------------------------------------------------
void projectile_result_destroy(projectile_result_t* res) {
    if (!res) return;
    if (res->trajectory) {
        trajectory_destroy(res->trajectory);
        res->trajectory = nullptr;
    }
    delete res;
}

static int projectile_result_format(
    const projectile_result_t* result,
    char* buffer,
    size_t buffer_size)
{
    if (!buffer || buffer_size == 0) return -1;

    if (!result) {
        return snprintf(buffer, buffer_size, "[Projectile Result] (null)");
    }

    return snprintf(buffer, buffer_size,
        "[Projectile Result]\n"
        "  Start Pos   : (%.3f, %.3f, %.3f)\n"
        "  Target Pos  : (%.3f, %.3f, %.3f)\n"
        "  Initial Vel : (%.3f, %.3f, %.3f)\n"
        "  Valid       : %s\n"
        "  Impact Time : %.3f sec\n"
        "  Impact Pos  : (%.3f, %.3f, %.3f)\n"
        "  Trajectory  : %s (%u points)",
        result->start_pos.x, result->start_pos.y, result->start_pos.z,
        result->target_pos.x, result->target_pos.y, result->target_pos.z,
        result->initial_velocity.x, result->initial_velocity.y, result->initial_velocity.z,
        result->bool_impacted ? "true" : "false",
        result->impact_time,
        result->impact_pos.x, result->impact_pos.y, result->impact_pos.z,
        result->trajectory ? "present" : "none",
        result->trajectory ? result->trajectory->count : 0
    );
}

void projectile_result_print(const projectile_result_t* result)
{
    char buffer[PROJECTILE_RESULT_STR_BUFSIZE];
    int written = projectile_result_format(result, buffer, sizeof(buffer));
    if (written > 0) {
        printf("%s\n", buffer);
    }
}

char* projectile_result_to_string(
    const projectile_result_t* result,
    char* buffer,
    size_t buffer_size)
{
    projectile_result_format(result, buffer, buffer_size);
    return buffer;
}

void projectile_result_print_detailed(const projectile_result_t* result)
{
    if (!result) {
        printf("[Projectile Result] (null)\n");
        return;
    }

    if (result->trajectory) {
        trajectory_print(result->trajectory);
    } else {
        printf("Trajectory: (none)\n");
    }

    projectile_result_print(result);
}

char* projectile_result_to_string_detailed(
    const projectile_result_t* result,
    size_t buffer_size, char* buffer)
{
    if (!buffer || buffer_size == 0) return NULL;

    if (!result) {
        snprintf(buffer, buffer_size, "[Projectile Result] (null)");
        return buffer;
    }

    size_t offset = 0;

    int written = projectile_result_format(result, buffer, buffer_size);
    if (written < 0) return buffer;
    if ((size_t)written >= buffer_size) return buffer;
    offset = (size_t)written;

    if (result->trajectory) {
        size_t remaining = buffer_size - offset;
        if (remaining > 1) {
            buffer[offset++] = '\n';
            trajectory_to_string(result->trajectory,
                                 remaining - 1,
                                buffer + offset);
        }
    } else {
        snprintf(buffer + offset, buffer_size - offset, "\nTrajectory: (none)\n");
    }

    return buffer;
}

float projectile_result_calc_initial_force_scalar(
    const projectile_result_t* result,
    float mass)
{
    if (!result || !result->trajectory || result->trajectory->count < 2) {
        return 0.0f;
    }

    const trajectory_sample_t* s0 = &result->trajectory->samples[0];
    const trajectory_sample_t* s1 = &result->trajectory->samples[1];

    float dt = s1->t - s0->t;
    if (dt <= 1e-6f) {
        return 0.0f;
    }

    vec3_t dv;
    vec3_sub(&dv, &s1->state.linear.velocity, &s0->state.linear.velocity);
    vec3_scale(&dv, &dv, 1.0f / dt);

    // FORCE = m * |a|
    return mass * vec3_length(&dv);
}

bool projectile_predict(    
    const projectile_t* proj,
    float time_step,
    const entity_dynamic_t* target,
    const environ_t* env,
    const ground_t* ground,
    propulsion_t* propulsion,
    guidance_func guidance_fn,
    projectile_result_t* out)
{
    if (!proj || !out || time_step <= 0.0f) return false;
    if (out->trajectory) trajectory_clear(out->trajectory);

    vec3_zero(&out->target_pos);
    if (target) out->target_pos = target->xf.pos;
    out->start_pos = proj->base.xf.pos;
    out->initial_velocity = proj->base.velocity;
    out->bool_impacted = false;

    motion_state_t state;
    entity_dynamic_to_motion_state(&proj->base, &state, NULL, NULL);

    const float mass = (proj->base.props.mass > 0.0f) ? proj->base.props.mass : 1.0f;
    const float lifetime = (proj->base.base.lifetime > 0.0f) ? proj->base.base.lifetime : 0.0f;
    const int   max_steps = (int)ceilf(lifetime / time_step);

    integrator_t intgr = {};
    integrator_init_full(&intgr, INTEGRATOR_RK4_ENV, 
        &state, NULL, env, &proj->base.props);

    float elapsed = 0.0f;

    for (int step = 0; step < max_steps; ++step) {
        vec3_t pos_prev = state.linear.position;
        vec3_t vel_prev = state.linear.velocity;

        // optional: env gusts hook (currently no-op)
        if (env) {
            vec3_t env_accel = {0, 0, 0};
            environ_apply_wind(&intgr.env, &env_accel, time_step);
            // If you want it effective: vec3_iadd(&state.linear.acceleration, &env_accel);
        }

        // propulsion guidance
        vec3_t thrust_accel = {0, 0, 0};
        if (propulsion && propulsion->fuel_remaining > 0.0f) {
            vec3_t guidance_dir = {0, 0, 0};
            if (guidance_fn) {
                guidance_target_info_t info = {};
                if (env)    info.env = *env;
                if (target) info.target = *target;
                vec3_t gtmp;
                const vec3_t* g = guidance_fn(&proj->base, time_step, &info, &gtmp);
                if (g) vec3_unit(&guidance_dir, g);
            } else if (target) {
                vec3_sub(&guidance_dir, &target->xf.pos, &state.linear.position);
                vec3_normalize(&guidance_dir);
            }
            propulsion_update(propulsion, time_step);
            float thrust = propulsion_get_thrust(propulsion);
            vec3_scale(&thrust_accel, &guidance_dir, thrust / mass);
            vec3_iadd(&state.linear.acceleration, &thrust_accel);
        }

        // integrate one step
        intgr.state = state;
        integrator_step(&intgr, time_step);
        state = intgr.state;
        elapsed += time_step;

        if (out->trajectory)
            trajectory_add_sample(out->trajectory, elapsed, &state);

        // sphere target continuous collision
        if (target) {
            const float target_radius = entity_size(&target->base);
            if (detect_sphere_collision(
                    &pos_prev, &vel_prev,
                    &state.linear.acceleration,
                    &target->xf.pos, target_radius,
                    elapsed - time_step, time_step,
                    &out->impact_pos, &out->impact_time))
            {
                out->bool_impacted = true;
                break;
            }
        }

        // ground collision: segment-level raycast (recommended)
        if (ground) {
            vec3_t step_vec;
            vec3_sub(&step_vec, &state.linear.position, &pos_prev);
            float seg_len = vec3_length(&step_vec);
            if (seg_len > VEC3_ABS_EPS_LEN) {
                vec3_t dir_step = step_vec;
                vec3_scale(&dir_step, &dir_step, 1.0f / seg_len);

                vec3_t hit_p, hit_n;
                float t_hit = 0.0f;
                if (ground_raycast(ground, &pos_prev, &dir_step, seg_len,
                                   &hit_p, &hit_n, NULL, &t_hit))
                {
                    out->impact_pos  = hit_p;
                    out->impact_time = (elapsed - time_step) + t_hit;
                    out->bool_impacted = true;
                    break;
                }
            }
        }
    }

    // annotate trajectory
    if (out->trajectory) {
        out->trajectory->impact_pos  = out->impact_pos;
        out->trajectory->impact_time = out->impact_time;
    }

    // callbacks
    if (out->bool_impacted) {
        if (proj->on_hit) proj->on_hit(proj, proj->hit_userdata);
    } else {
        // lifetime expired
        projectile_default_expire_cb(proj, proj->hit_userdata);
    }

    return out->bool_impacted;
}
