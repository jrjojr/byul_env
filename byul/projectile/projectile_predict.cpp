#include <cmath>
#include <cstdio>
#include "projectile_predict.h"
#include "guidance.h"
#include "float_common.h"
#include "numeq_filters.h"
#include <numeq_model_motion.h>

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
    res->valid = false;

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
    res->valid = false;

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
    res->valid = src->valid;

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
    res->valid = false;

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
    res->valid = false;

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
        result->valid ? "true" : "false",
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

static bool solve_ground_hit_time_interval(
    const vec3_t* pos_prev,
    const vec3_t* vel_prev,
    const vec3_t* accel,
    float dt,
    float* alpha) // 0 ~ 1
{
    float y0 = pos_prev->y;
    float vy = vel_prev->y;
    float ay = accel->y;

    // y(t) = y0 + vy*t + 0.5*ay*t^2
    if (fabsf(ay) < 1e-6f) {
        if (fabsf(vy) < 1e-6f) return false;
        *alpha = -y0 / (vy * dt);
        return (*alpha >= 0.0f && *alpha <= 1.0f);
    }

    // 0 = y0 + vy*t + 0.5*ay*t^2
    float a = 0.5f * ay;
    float b = vy;
    float c = y0;

    float disc = b*b - 4*a*c;
    if (disc < 0.0f) return false;

    float sqrt_disc = sqrtf(disc);
    float t1 = (-b - sqrt_disc) / (2*a);
    float t2 = (-b + sqrt_disc) / (2*a);

    float t_hit = (t1 >= 0 && t1 <= dt) 
    ? t1 : ((t2 >= 0 && t2 <= dt) ? t2 : -1.0f);
    if (t_hit < 0) return false;

    *alpha = t_hit / dt;
    return true;
}

bool detect_ground_collision_precise(
    const vec3_t* pos_prev,
    const vec3_t* pos_curr,
    const vec3_t* vel_prev,
    const vec3_t* accel,
    float t_prev,
    float dt,
    vec3_t* impact_pos,
    float* impact_time)
{
    if (pos_prev->y > 0.0f && pos_curr->y <= 0.0f) {
        float alpha;
        if (!solve_ground_hit_time_interval(
            pos_prev, vel_prev, accel, dt, &alpha))
            return false;

        *impact_time = t_prev + alpha * dt;
        vec3_lerp(impact_pos, pos_prev, pos_curr, alpha);
        impact_pos->y = 0.0f;
        return true;
    }
    return false;
}

static bool solve_entity_hit_time(
    const vec3_t* rel_p,
    const vec3_t* rel_v,
    const vec3_t* rel_a,
    float R,
    float dt,
    float* t_hit)
{
    // 1/2 a pre calc
    vec3_t half_a = *rel_a;
    vec3_scale(&half_a, &half_a, 0.5f);

    // s(t) = p + v t + 0.5 a t^2
    // |s(t)|^2 = A t^2 + B t + C = R^2
    float A = vec3_dot(rel_v, rel_v) +
              2.0f * vec3_dot(rel_v, &half_a) +
              vec3_dot(&half_a, &half_a);

    float B = 2.0f * (vec3_dot(rel_p, rel_v) + vec3_dot(rel_p, &half_a));

    float C = vec3_dot(rel_p, rel_p) - R * R;

    if (fabsf(A) < FLOAT_EPSILON) {
        float t;
        if (!numeq_solve_linear(B, C, &t)) return false;
        if (t >= 0.0f && t <= dt) { *t_hit = t; return true; }
        return false;
    }

    float x1, x2;
    if (!numeq_solve_quadratic(A, B, C, &x1, &x2)) return false;

    // Select the smallest positive solution within the range [0, dt]
    bool found = false;
    float best = dt + 1.0f;

    if (x1 >= 0.0f && x1 <= dt) { best = x1; found = true; }
    if (x2 >= 0.0f && x2 <= dt && (!found || x2 < best)) {
         best = x2; found = true; 
    }

    if (!found) return false;
    *t_hit = best;
    return true;
}

bool detect_entity_collision_precise(
    const vec3_t* proj_pos_prev,
    const vec3_t* proj_vel_prev,
    const vec3_t* proj_accel,
    const vec3_t* target_pos,
    float target_radius,
    float dt,
    float t_prev,
    vec3_t* impact_pos,
    float* impact_time)
{
    vec3_t rel_p = *proj_pos_prev;
    vec3_sub(&rel_p, &rel_p, target_pos);

    vec3_t rel_v = *proj_vel_prev;
    vec3_t rel_a = *proj_accel;

    linear_state_t state_prev;
    linear_state_init(&state_prev);
    state_prev.position = rel_p;
    state_prev.velocity = rel_v;
    state_prev.acceleration = rel_a;

    float d_prev = vec3_length(&rel_p);
    vec3_t rel_curr;
    numeq_model_pos_predict(dt, &state_prev, NULL, NULL, &rel_curr);
    float d_curr = vec3_length(&rel_curr);

    if (d_prev <= target_radius) {
        *impact_time = t_prev;
        *impact_pos = *proj_pos_prev;
        return true;
    }
    float t_local;
    if (solve_entity_hit_time(
        &rel_p, &rel_v, &rel_a, target_radius, dt, &t_local)) {
        if (t_local >= 0.0f && t_local <= dt) {
            *impact_time = t_prev + t_local;
            numeq_model_pos_predict(t_local, &state_prev, NULL, NULL, impact_pos);
            vec3_add(impact_pos, impact_pos, target_pos);
            return true;
        }
    }
    // --- Additional check based on distance reduction 
    // (for cases with large sample intervals) ---
    if (d_prev > target_radius && d_curr < target_radius) {
        // If interpolation fails, 
        // estimate collision time using linear interpolation
        float ratio = (d_prev - target_radius) / (d_prev - d_curr);
        float approx_t = ratio * dt;
        if (approx_t < 0.0f) approx_t = 0.0f;
        if (approx_t > dt) approx_t = dt;

        *impact_time = t_prev + approx_t;
        numeq_model_pos_predict(approx_t, &state_prev, NULL, NULL, impact_pos);
        vec3_add(impact_pos, impact_pos, target_pos);
        return true;
    }
    return false;
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

// ---------------------------------------------------------
// projectile_predict()
// ---------------------------------------------------------
bool projectile_predict(
    projectile_result_t* out,
    const projectile_t* proj,
    const entity_dynamic_t* target,
    float max_time,
    float dt,
    const environ_t* env,
    propulsion_t* propulsion,
    guidance_func guidance_fn)
{
    if (!proj || !out || dt <= 0.0f) return false;
    trajectory_clear(out->trajectory);

    vec3_t target_pos = target ? target->xf.pos : vec3_t{0, 0, 0};
    float target_radius = target ? entity_size(&target->base) : 0.0f;

    motion_state_t state;
    entity_dynamic_to_motion_state(&proj->base, &state, NULL, NULL);

    projectile_t temp_proj = *proj;
    float mass = (proj->base.props.mass > 0.0f) 
    ? proj->base.props.mass : 1.0f;

    float t = 0.0f;
    const int max_steps = (int)ceilf(max_time / dt);

    out->start_pos = proj->base.xf.pos;
    out->target_pos = target->xf.pos;
    out->initial_velocity = proj->base.velocity;
    out->valid = false;


    integrator_t intgr = {};
    integrator_init_full(
        &intgr, INTEGRATOR_RK4_ENV,
        dt, &state, nullptr, env, &proj->base.props);

    for (int step = 0; step < max_steps; ++step, t += dt) {
        vec3_t pos_prev = state.linear.position;
        vec3_t vel_prev = state.linear.velocity;

        vec3_t env_accel = {0, 0, 0};
        if (env) {
            // entity_dynamic_calc_accel_env(
            //     &proj->base, &vel_prev, dt, env, &env_accel);
            
            env->environ_fn(env, env->userdata, &env_accel);
        }

        vec3_t thrust_accel = {0, 0, 0};
        if (propulsion && propulsion->fuel_remaining > 0.0f) {
            vec3_t guidance = {0, 0, 0};
            if (target) {
                vec3_sub(&guidance, &target->xf.pos, &state.linear.position);
                vec3_normalize(&guidance);
            }
            if (guidance_fn) {
                guidance_target_info_t info = {};
                if (env) info.env = *env;
                if (target) info.target = *target;
                vec3_t vec;
                const vec3_t* g = guidance_fn(
                    &temp_proj.base, dt, &info, &vec);
                if (g) vec3_unit(&guidance, g);
            }
            propulsion_update(propulsion, dt);
            float thrust = propulsion_get_thrust(propulsion);
            vec3_scale(&thrust_accel, &guidance, thrust / mass);
        }

        vec3_add(&state.linear.acceleration, &env_accel, &thrust_accel);

        intgr.state = state;
        integrator_step(&intgr);
        state = intgr.state;

        bodyprops_apply_friction(
            &state.linear.velocity, &proj->base.props, dt);

        trajectory_add_sample(out->trajectory, t, &state);

        bool ground_hit = false;
        if (state.linear.position.y <= 0.0f){

            ground_hit = true;
        }

        if (target) {
            float dist_prev = vec3_distance(&pos_prev, &target->xf.pos);
            float dist_curr = vec3_distance(
                &state.linear.position, &target->xf.pos);

            if (detect_entity_collision_precise(
                    &pos_prev, &vel_prev, 
                    &state.linear.acceleration,
                    &target->xf.pos, target_radius,
                    dt, t-dt, &out->impact_pos, &out->impact_time))
            {
                out->valid = true;
                goto finalize;
            }
        }

        if(ground_hit){
            if(detect_ground_collision_precise(
                &pos_prev, &state.linear.position,
                &vel_prev, &state.linear.acceleration,
                t, dt, &out->impact_pos, &out->impact_time)){

                out->valid = true;
               goto finalize;
            }
        }
    }

    out->valid = false;

finalize:
    integrator_free(&intgr);
    return out->valid;
}

bool projectile_predict_with_kalman_filter(
    projectile_result_t* out,
    const projectile_t* proj,
    entity_dynamic_t* entdyn,
    float max_time,
    float dt,
    const environ_t* env,
    const propulsion_t* propulsion,
    guidance_func guidance_fn)
{
    if (!proj || !out || dt <= 0.0f) return false;
    trajectory_clear(out->trajectory);

    vec3_t target_pos = entdyn ? entdyn->xf.pos : vec3_t{0, 0, 0};
    float target_radius = entdyn ? entity_size(&entdyn->base) : 0.0f;

    motion_state_t state;
    entity_dynamic_to_motion_state(&proj->base, &state, NULL, NULL);

    projectile_t temp_proj = *proj;
    float mass = (proj->base.props.mass > 0.0f) ? proj->base.props.mass : 1.0f;

    kalman_filter_vec3_t kf;
    kalman_vec3_init_full(
        &kf,
        &state.linear.position,
        &state.linear.velocity,
        0.01f,
        1.0f,
        dt);

    float t = 0.0f;
    const int max_steps = (int)ceilf(max_time / dt);

    for (int step = 0; step < max_steps; ++step, t += dt) {
        vec3_t pos_prev = state.linear.position;
        vec3_t vel_prev = state.linear.velocity;

        vec3_t env_accel = {0, 0, 0};
        if (env) {
            numeq_model_motion_accel(&state, 
                env, &proj->base.props, dt, &env_accel);
        }

        vec3_t thrust_accel = {0, 0, 0};
        if (propulsion && propulsion->fuel_remaining > 0.0f) {
            vec3_t guidance = {0, 0, 0};
            if (entdyn) {
                vec3_sub(&guidance, &entdyn->xf.pos, &state.linear.position);
                vec3_normalize(&guidance);
            }
            if (guidance_fn) {
                guidance_target_info_t info = {};
                if (env) info.env = *env;
                if (entdyn) info.target = *entdyn;
                vec3_t vec;
                const vec3_t* g = guidance_fn(&temp_proj.base, dt, &info, &vec);
                if (g) vec3_unit(&guidance, g);
            }
            float thrust = propulsion_get_thrust(propulsion);
            vec3_scale(&thrust_accel, &guidance, thrust / mass);
        }

        vec3_add(&state.linear.acceleration, &env_accel, &thrust_accel);

        // --- Kalman Filter Predict & Measurement Update ---
        kalman_vec3_time_update(&kf);
        kalman_vec3_measurement_update(&kf, &state.linear.position);

        state.linear.position = kf.position;
        state.linear.velocity = kf.velocity;

        trajectory_add_sample(out->trajectory, t, &state);

        integrator_t intgr;
        integrator_init_full(
            &intgr, INTEGRATOR_MOTION_RK4_ENV,
            dt, &state, nullptr, env, &proj->base.props);

        integrator_step(&intgr);

        if (entdyn) {
            float dist_prev = vec3_distance(&pos_prev, &target_pos);
            float dist_curr = vec3_distance(&state.linear.position, &target_pos);

            if (dist_prev > target_radius && dist_curr <= target_radius) {
                if (detect_entity_collision_precise(
                        &pos_prev, &vel_prev, 
                        &state.linear.acceleration,
                        &target_pos, target_radius,
                        dt, t, &out->impact_pos, &out->impact_time))
                {
                    out->valid = true;
                    return true;
                }
            }
        }

        if (detect_ground_collision_precise(
                &pos_prev, &state.linear.position,
                &vel_prev, &state.linear.acceleration,
                t, dt, &out->impact_pos, &out->impact_time))
        {
            out->valid = true;
            return true;
        }
    }

    out->valid = false;
    return false;
}

bool projectile_predict_with_filter(
    projectile_result_t* out,
    const projectile_t* proj,
    entity_dynamic_t* entdyn,
    float max_time,
    float dt,
    const environ_t* env,
    const propulsion_t* propulsion,
    guidance_func guidance_fn,
    const filter_interface_t* filter_if)
{
    if (!proj || !out || dt <= 0.0f) return false;
    trajectory_clear(out->trajectory);

    vec3_t target_pos = entdyn ? entdyn->xf.pos : vec3_t{0, 0, 0};
    float target_radius = entdyn ? entity_size(&entdyn->base) : 0.0f;

    motion_state_t state;
    entity_dynamic_to_motion_state(&proj->base, &state, NULL, NULL);

    projectile_t temp_proj = *proj;
    float mass = (proj->base.props.mass > 0.0f) ? proj->base.props.mass : 1.0f;

    float t = 0.0f;
    const int max_steps = (int)ceilf(max_time / dt);

    for (int step = 0; step < max_steps; ++step, t += dt) {
        vec3_t pos_prev = state.linear.position;
        vec3_t vel_prev = state.linear.velocity;

        vec3_t env_accel = {0, 0, 0};
        if (env) {
            numeq_model_motion_accel(&state, env, 
                &proj->base.props, dt, &env_accel);
        }

        vec3_t thrust_accel = {0, 0, 0};
        if (propulsion && propulsion->fuel_remaining > 0.0f) {
            vec3_t guidance = {0, 0, 0};
            if (entdyn) {
                vec3_sub(&guidance, &entdyn->xf.pos, &state.linear.position);
                vec3_normalize(&guidance);
            }
            if (guidance_fn) {
                guidance_target_info_t info = {};
                if (env) info.env = *env;
                if (entdyn) info.target = *entdyn;
                vec3_t vec;
                const vec3_t* g = guidance_fn(&temp_proj.base, dt, &info, &vec);
                if (g) vec3_unit(&guidance, g);
            }
            float thrust = propulsion_get_thrust(propulsion);
            vec3_scale(&thrust_accel, &guidance, thrust / mass);
        }

        vec3_add(&state.linear.acceleration, &env_accel, &thrust_accel);

        if (filter_if && filter_if->time_update && filter_if->measurement_update) {
            filter_if->time_update(filter_if->filter_state);
            filter_if->measurement_update(filter_if->filter_state,
                                          &state.linear.position,
                                          &state.linear.velocity);
            if (filter_if->get_state) {
                vec3_t filtered_pos, filtered_vel;
                filter_if->get_state(filter_if->filter_state,
                                     &filtered_pos, &filtered_vel);
                state.linear.position = filtered_pos;
                state.linear.velocity = filtered_vel;
            }
        }

        trajectory_add_sample(out->trajectory, t, &state);

        integrator_t intgr;
        integrator_init_full(&intgr, INTEGRATOR_MOTION_RK4_ENV,
                                    dt, &state, nullptr, env, &proj->base.props);
        intgr.dt = dt;
        integrator_step(&intgr);

        if (entdyn) {
            float dist_prev = vec3_distance(&pos_prev, &target_pos);
            float dist_curr = vec3_distance(&state.linear.position, &target_pos);
            if (dist_prev > target_radius && dist_curr <= target_radius) {
                if (detect_entity_collision_precise(
                        &pos_prev, &vel_prev, 
                        &state.linear.acceleration,
                        &target_pos, target_radius,
                        dt, t, &out->impact_pos, &out->impact_time))
                {
                    out->valid = true;
                    return true;
                }
            }
        }
        if (detect_ground_collision_precise(&pos_prev, &state.linear.position,
                    &vel_prev, &state.linear.acceleration,
                    t, dt, &out->impact_pos, &out->impact_time))
        {
            out->valid = true;
            return true;
        }
    }

    out->valid = false;
    return false;
}

