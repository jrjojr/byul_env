#include "projectile_tick.h"
#include "projectile_predict.h"
#include "numeq_integrator.h"
#include "bodyprops.h"
#include "motion_state.h"
#include "vec3.h"
#include "collision.h"

void calc_suitable_dt(
    float* dt_out, const vec3_t* dir, float force, float mass)
{
    if (!dt_out || !dir || mass <= 0.0f) return;

    // Estimate acceleration and velocity
    float acceleration = force / mass;
    float velocity = acceleration * DELTA_TIME;

    // Estimate how far the projectile moves per frame
    float distance_per_step = velocity * DELTA_TIME;

    // Estimate appropriate sample count
    int sample_count = (int)(XFORM_MAX_DISTANCE / distance_per_step);
    if (sample_count < 32) sample_count = 32;
    if (sample_count > 4096) sample_count = 4096;

    // Compute dt = total_distance / (sample_count x velocity)
    float estimated_dt = XFORM_MAX_DISTANCE / (sample_count * velocity);

    // Clamp dt to acceptable range
    if (estimated_dt < MIN_DELTA_TIME) estimated_dt = MIN_DELTA_TIME;
    if (estimated_dt > MAX_DELTA_TIME) estimated_dt = MAX_DELTA_TIME;

    *dt_out = estimated_dt;
}

void calc_suitable_max_time(
    float* max_time_out, const vec3_t* dir, float force, float mass)
{
    if (!max_time_out || !dir || mass <= 0.0f) return;

    float acceleration = force / mass;
    float velocity = acceleration * DELTA_TIME;

    float estimated_time = XFORM_MAX_DISTANCE / velocity;

    if (estimated_time < MIN_SIM_TIME) estimated_time = MIN_SIM_TIME;
    if (estimated_time > MAX_SIM_TIME) estimated_time = MAX_SIM_TIME;

    *max_time_out = estimated_time;
}



static void projectile_tick_update_cb(void* context, float dt)
{
    if (!context || dt <= 0.0f) return;

    projectile_tick_t* prt = (projectile_tick_t*)context;

    if (!projectile_tick(prt, dt))
        return;
}

void projectile_tick_init(projectile_tick_t* prt) {
    if (!prt) return;

    projectile_init(&prt->proj);
    // motion_state_init(&prt->state);
    entity_dynamic_init(&prt->target);
    // integrator_init(&prt->intgr);

    prt->impact_pos = {};
    prt->trajectory = nullptr;
    prt->env = nullptr;
    prt->ground = nullptr;
    prt->propulsion = nullptr;
    prt->guidance_fn = nullptr;
    prt->proj.base.base.age = 0.0f;
    prt->impact_time = -1.0f;
    prt->bool_impacted = false;
    prt->bool_debug = false;
    prt->proj.base.base.lifetime = 60.0f;
    prt->tick = nullptr;
}

void projectile_tick_init_full(projectile_tick_t* prt,
    const projectile_t* proj, 
    const entity_dynamic_t* target, 

    const environ_t* env,
    const ground_t* ground,
    const propulsion_t* propulsion,
    const guidance_func guidance_fn,

    bool bool_debug)
{
    if (!prt || !proj || !target) return;

    projectile_tick_init(prt);
    projectile_assign(&prt->proj, proj);
    // motion_state_assign(&prt->state, state);
    entity_dynamic_assign(&prt->target, target);

    motion_state_t state;
    entity_dynamic_to_motion_state(&prt->proj.base, &state, nullptr, nullptr);

    integrator_init_full(&prt->intgr, INTEGRATOR_MOTION_RK4_ENV,
        &state, nullptr, env, &proj->base.props);

    prt->bool_debug = bool_debug;

    if (env) {
        prt->env = new environ_t;
        if (prt->env) environ_assign(prt->env, env);
    }

    if (ground) {
        prt->ground = new ground_t;
        if (prt->ground) ground_assign(prt->ground, ground);
    }

    if (propulsion) {
        prt->propulsion = new propulsion_t;
        if (prt->propulsion) propulsion_assign(prt->propulsion, propulsion);
    }

    if (guidance_fn) {
        prt->guidance_fn = guidance_fn;
    }

    if (bool_debug){
        prt->trajectory = trajectory_create_full(MAX_SAMPLE_COUNT);
    }

}

void projectile_tick_free(projectile_tick_t* prt) {
    if (!prt) return;

    if (prt->env) {
        delete prt->env;
        prt->env = nullptr;
    }

    if (prt->ground) {
        delete prt->ground;
        prt->ground = nullptr;
    }

    if (prt->propulsion) {
        delete prt->propulsion;
        prt->propulsion = nullptr;
    }

    if (prt->bool_debug && prt->trajectory) {
        trajectory_destroy(prt->trajectory);
        prt->trajectory = nullptr;
        prt->bool_debug = false;
    }

    if (prt->tick) {
        prt->tick = nullptr;
    }
}

void projectile_tick_assign(
    projectile_tick_t* out, const projectile_tick_t* src) {

    if (!out || !src) return;

    projectile_assign(&out->proj, &src->proj);
    // motion_state_assign(&out->state, &src->state);
    entity_dynamic_assign(&out->target, &src->target);
    // integrator_assign(&out->intgr, &src->intgr);
    out->intgr = src->intgr;

    out->impact_time = src->impact_time;

    if (src->env) {
        if (!out->env) out->env = new environ_t;
        if (out->env) environ_assign(out->env, src->env);

    } else {
        out->env = nullptr;
    }

    if (src->ground) {
        if (!out->ground) out->ground = new ground_t;
        if (out->ground) ground_assign(out->ground, src->ground);

    } else {
        out->ground = nullptr;
    }    

    if (src->propulsion) {
        if (!out->propulsion)  out->propulsion = new propulsion_t;
        if (out->propulsion) 
            propulsion_assign(out->propulsion, src->propulsion);
    } else {
        out->propulsion = nullptr;
    }

    out->guidance_fn = src->guidance_fn;

    if (src->trajectory) {
        out->trajectory = src->trajectory;
    }

    if (src->tick){
        out->tick = src->tick;
    }

    out->impact_pos = src->impact_pos;
    
}

bool projectile_tick_prepare(
    projectile_tick_t* prt,
    tick_t* tk)
{
    if (!prt || !tk) return false;
    prt->tick = tk;
    prt->bool_impacted = false;

        if (prt->trajectory) {
            motion_state_t state = {};
            entity_dynamic_to_motion_state(
                &prt->proj.base, &state, nullptr, nullptr);

            trajectory_add_sample(prt->trajectory, prt->proj.base.base.age, &state);
        }    

    return tick_attach(tk, projectile_tick_update_cb, (void*)prt) == 0;
}

bool projectile_tick_prepare_full(
    projectile_tick_t* prt,
    const entity_dynamic_t* target,
    tick_t* tk)
{
    if (!prt || !target || !tk) return false;

    motion_state_t state;
    entity_dynamic_to_motion_state(&prt->proj.base, &state, nullptr, nullptr);
    entity_dynamic_assign(&prt->target, target);

    prt->tick = tk;
    prt->bool_impacted = false;

        if (prt->trajectory) {
            motion_state_t state = {};
            entity_dynamic_to_motion_state(
                &prt->proj.base, &state, nullptr, nullptr);
                
            trajectory_add_sample(prt->trajectory, prt->proj.base.base.age, &state);
        }    

    return tick_attach(tk, projectile_tick_update_cb, (void*)prt) == 0;
}

// bool projectile_tick(
//     projectile_tick_t* prt,
//     float dt)
// {
//     if (!prt || dt <= 0.0f)
//         return false;
    
//     if (prt->bool_impacted){
//         projectile_tick_complete(prt, prt->tick);
//         return true;
//     }

//     motion_state_t state;
//     entity_dynamic_to_motion_state(&prt->proj.base, &state, nullptr, nullptr);

//     prt->proj.base.base.age += dt;
//     if (prt->proj.base.base.age >= prt->proj.base.base.lifetime){
//         // projectile_default_expire_cb
//         prt->proj.on_hit = projectile_default_expire_cb;
//         projectile_tick_complete(prt, prt->tick);
//         return false;        
//     }
//     if (prt->proj.base.base.age < prt->proj.base.base.lifetime) {
//         vec3_t pos_prev = state.linear.position;
//         vec3_t vel_prev = state.linear.velocity;

//         vec3_t env_accel = {0, 0, 0};
//         if (prt->env && prt->env->environ_fn) {
//             prt->env->environ_fn(prt->env, prt->env->userdata, &env_accel);
//         }

//         vec3_t thrust_accel = {0, 0, 0};
//         if (prt->propulsion && prt->propulsion->fuel_remaining > 0.0f) {
//             vec3_t guidance = {0, 0, 0};
//             vec3_sub(&guidance, &prt->target.xf.pos, &state.linear.position);
//             vec3_normalize(&guidance);

//             projectile_t temp_proj = prt->proj;
//             if (prt->guidance_fn) {
//                 guidance_target_info_t info = {};
//                 if (prt->env) info.env = *prt->env;
//                 info.target = prt->target;
//                 vec3_t vec;
//                 const vec3_t* g = prt->guidance_fn(
//                     &temp_proj.base, dt, &info, &vec);
//                 if (g) vec3_unit(&guidance, g);
//             }
//             propulsion_update(prt->propulsion, dt);
//             float thrust = propulsion_get_thrust(prt->propulsion);
//             vec3_scale(&thrust_accel, &guidance, 
//                 thrust / prt->proj.base.props.mass);
//         }

//         vec3_iadd(&state.linear.acceleration, &thrust_accel);

//         prt->intgr.state = state;
//         integrator_step(&prt->intgr, dt);
//         state = prt->intgr.state;
//         entity_dynamic_from_motion_state(&prt->proj.base, &state);

//         if (prt->bool_debug && prt->trajectory) {
//             trajectory_add_sample(prt->trajectory, prt->proj.base.base.age, &state);
//         }

//         float dist_prev = vec3_distance(&pos_prev, &prt->target.xf.pos);
//         float dist_curr = vec3_distance(
//             &state.linear.position, &prt->target.xf.pos);

//         float target_radius = entity_size(&prt->target.base);

//         if (detect_sphere_collision(
//                 &pos_prev, &vel_prev, 
//                 &state.linear.acceleration,
//                 &prt->target.xf.pos, target_radius,
//                 prt->proj.base.base.age - dt, dt, &prt->impact_pos, &prt->impact_time))
//         {
//             goto finalize;
//         }

//         bool ground_hit = false;
//         // Before loop: define ground plane once (if you are using a ground plane)
//         vec3_t plane_point  = {0.0f, 0.0f, 0.0f};
//         vec3_t plane_normal = {0.0f, 1.0f, 0.0f};
//         // vec3_normalize(&plane_normal); // ensure unit length

//         if(detect_plane_collision(
//             &pos_prev, &state.linear.position,
//             &vel_prev, &state.linear.acceleration,
//             &plane_point, &plane_normal,
//             prt->proj.base.base.age - dt, dt, &prt->impact_pos, &prt->impact_time)){

//             goto finalize;
//         }
//     }
//     return false;

// finalize:
//     if (prt->trajectory){
//         prt->trajectory->impact_pos = prt->impact_pos;
//         prt->trajectory->impact_time = prt->impact_time;
//     }
//     prt->bool_impacted = true;
//     if (prt->proj.on_hit) {
//         prt->proj.on_hit(&prt->proj, prt->proj.hit_userdata);
//     }
//     return true;
// }

// #include "ground.h"  /* ground_raycast */
// #include "vec3.h"
// #include "entity_dynamic.h"
// #include "integrator.h"
// #include "projectile.h"
// #include "environ.h"

bool projectile_tick(
    projectile_tick_t* prt,
    float dt)
{
    if (!prt || dt <= 0.0f) return false;

    /* already done this frame */
    if (prt->bool_impacted) {
        projectile_tick_complete(prt, prt->tick);
        return true;
    }

    /* current state snapshot */
    motion_state_t state;
    entity_dynamic_to_motion_state(&prt->proj.base, &state, nullptr, nullptr);

    /* age and expire check */
    prt->proj.base.base.age += dt;
    if (prt->proj.base.base.age >= prt->proj.base.base.lifetime) {
        /* lifetime expired: default expire behavior */
        projectile_default_expire_cb(&prt->proj, prt->proj.hit_userdata);
        projectile_tick_complete(prt, prt->tick);
        return false;
    }

    /* previous kinematics */
    vec3_t pos_prev = state.linear.position;
    vec3_t vel_prev = state.linear.velocity;

    /* environment acceleration hook */
    if (prt->env && prt->env->environ_fn) {
        vec3_t env_accel = {0, 0, 0};
        prt->env->environ_fn(prt->env, prt->env->userdata, &env_accel);
        vec3_iadd(&state.linear.acceleration, &env_accel);
    }

    /* propulsion and guidance */
    if (prt->propulsion && prt->propulsion->fuel_remaining > 0.0f) {
        vec3_t guidance_dir = {0, 0, 0};

        if (prt->guidance_fn) {
            guidance_target_info_t info = {};
            if (prt->env)    info.env = *prt->env;
            info.target = prt->target; /* if unset, zero-initialized struct */
            vec3_t tmp;
            const vec3_t* g = prt->guidance_fn(&prt->proj.base, dt, &info, &tmp);
            if (g) vec3_unit(&guidance_dir, g);
        } else {
            /* simple seek if target is meaningful */
            vec3_sub(&guidance_dir, &prt->target.xf.pos, &state.linear.position);
            vec3_normalize(&guidance_dir);
        }

        propulsion_update(prt->propulsion, dt);
        const float thrust = propulsion_get_thrust(prt->propulsion);
        const float mass   = (prt->proj.base.props.mass > 0.0f) ? prt->proj.base.props.mass : 1.0f;

        vec3_t thrust_accel = {0, 0, 0};
        vec3_scale(&thrust_accel, &guidance_dir, thrust / mass);
        vec3_iadd(&state.linear.acceleration, &thrust_accel);
    }

    /* integrate one fixed step */
    prt->intgr.state = state;
    integrator_step(&prt->intgr, dt);
    state = prt->intgr.state;
    entity_dynamic_from_motion_state(&prt->proj.base, &state);

    /* optional debug sampling */
    if (prt->bool_debug && prt->trajectory) {
        trajectory_add_sample(prt->trajectory, prt->proj.base.base.age, &state);
    }

    /* target collision: continuous sphere test over this step */
    {
        const float target_radius = entity_size(&prt->target.base);
        if (target_radius > 0.0f) {
            if (detect_sphere_collision(
                    &pos_prev, &vel_prev,
                    &state.linear.acceleration,
                    &prt->target.xf.pos, target_radius,
                    prt->proj.base.base.age - dt, dt,
                    &prt->impact_pos, &prt->impact_time))
            {
                goto finalize_hit;
            }
        }
    }

    /* ground collision: robust segment raycast for any ground mode */
    if (prt->ground) {
        vec3_t step_vec;
        vec3_sub(&step_vec, &state.linear.position, &pos_prev);
        const float seg_len = vec3_length(&step_vec);

        if (seg_len > VEC3_ABS_EPS_LEN) {
            vec3_t dir_step = step_vec;
            vec3_scale(&dir_step, &dir_step, 1.0f / seg_len);

            vec3_t hit_p, hit_n;
            float  t_hit = 0.0f;
            if (ground_raycast(prt->ground, &pos_prev, &dir_step, seg_len,
                               &hit_p, &hit_n, nullptr, &t_hit))
            {
                prt->impact_pos  = hit_p;
                prt->impact_time = (prt->proj.base.base.age - dt) + t_hit;
                goto finalize_hit;
            }
        }
    }

    /* no impact this frame */
    return false;

finalize_hit:
    if (prt->trajectory) {
        prt->trajectory->impact_pos  = prt->impact_pos;
        prt->trajectory->impact_time = prt->impact_time;
    }
    prt->bool_impacted = true;
    if (prt->proj.on_hit) {
        prt->proj.on_hit(&prt->proj, prt->proj.hit_userdata);
    }
    return true;
}


bool projectile_tick_complete(
    projectile_tick_t* prt,
    tick_t* tk)
{
    if (!prt) return false;

    prt->bool_impacted = false;

    if (tk)
    {
        tick_request_detach(tk, projectile_tick_update_cb, (void*)prt);
    }

    return true;
}
