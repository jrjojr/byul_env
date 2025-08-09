#include "projectile_tick.h"
#include "projectile_predict.h"
#include "numeq_integrator.h"
#include "bodyprops.h"
#include "motion_state.h"
#include "vec3.h"

// ─────────────────────────────────────────────────────────
// Tick update callback (등록용)
// ─────────────────────────────────────────────────────────

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
    prt->trajectory = NULL;
    prt->env = NULL;
    prt->propulsion = NULL;
    prt->guidance_fn = NULL;
    prt->elapsed = 0.0f;
    prt->impact_time = -1.0f;
    prt->max_time = 60.0f;
}

void projectile_tick_init_full(projectile_tick_t* prt,
    const projectile_t* proj, 
    // const motion_state_t* state,
    const entity_dynamic_t* target, integrator_t* intgr,
    const environ_t* env,
    const propulsion_t* propulsion,
    const guidance_func guidance_fn)
{
    if (!prt || !proj || !target || !intgr) return;

    // 기본값 초기화
    projectile_tick_init(prt);
    projectile_assign(&prt->proj, proj);
    // motion_state_assign(&prt->state, state);
    entity_dynamic_assign(&prt->target, target);
    // integrator_assign(&prt->intgr, intgr);
    prt->intgr = intgr;

    // 포인터 기반 인자들 복사 (존재할 경우)
    if (env) {
        prt->env = new environ_t;
        if (prt->env) environ_assign(prt->env, env);
    }

    if (propulsion) {
        prt->propulsion = new propulsion_t;
        if (prt->propulsion) propulsion_assign(prt->propulsion, propulsion);
    }

    if (guidance_fn) {
        prt->guidance_fn = guidance_fn;  // 함수 포인터 복사 (값 복사만으로 충분)
    }
}

void projectile_tick_free(projectile_tick_t* prt) {
    if (!prt) return;

    if (prt->env) {
        delete prt->env;
        prt->env = NULL;
    }

    if (prt->propulsion) {
        delete prt->propulsion;
        prt->propulsion = NULL;
    }

    if (prt->trajectory) {
        delete prt->trajectory;
        prt->trajectory = NULL;
    }

    // if (prt->impact_pos) {
    //     delete prt->impact_pos;
    //     prt->impact_pos = {};
    // }
}

void projectile_tick_assign(
    projectile_tick_t* out, const projectile_tick_t* src) {

    if (!out || !src) return;

    // 값 복사
    projectile_assign(&out->proj, &src->proj);
    // motion_state_assign(&out->state, &src->state);
    entity_dynamic_assign(&out->target, &src->target);
    // integrator_assign(&out->intgr, &src->intgr);
    out->intgr = src->intgr;

    out->elapsed = src->elapsed;
    out->max_time = src->max_time;
    out->impact_time = src->impact_time;

    // 포인터 복사 (깊은 복사 대상)
    if (src->env) {
        if (!out->env) out->env = new environ_t;
        if (out->env) environ_assign(out->env, src->env);

    } else {
        out->env = NULL;
    }

    if (src->propulsion) {
        if (!out->propulsion)  out->propulsion = new propulsion_t;
        if (out->propulsion) 
            propulsion_assign(out->propulsion, src->propulsion);
    } else {
        out->propulsion = NULL;
    }

    // 함수 포인터는 값 복사만으로 충분
    out->guidance_fn = src->guidance_fn;

    // trajectory, impact_pos는 소유하지 않음 → 얕은 복사
    if (out->trajectory) {
        trajectory_free(out->trajectory);
        out->trajectory = trajectory_copy(src->trajectory);
    }
    out->impact_pos = src->impact_pos;
}

// ─────────────────────────────────────────────────────────
// Tick prepare: tick_attach만 수행
// ─────────────────────────────────────────────────────────

bool projectile_tick_prepare(
    const projectile_tick_t* prt,
    tick_t* tk)
{
    if (!prt || !tk) return false;

    return tick_attach(tk, projectile_tick_update_cb, (void*)&prt->proj) == 0;
}

// ─────────────────────────────────────────────────────────
// Tick prepare_full: 유도방향 및 초기 속도 세팅 + attach
// ─────────────────────────────────────────────────────────

bool projectile_tick_prepare_full(
    projectile_tick_t* prt,
    const entity_dynamic_t* target,
    tick_t* tk)
{
    if (!prt || !target || !tk) return false;

    motion_state_t state;
    entity_dynamic_to_motion_state(&prt->proj.base, &state, NULL, NULL);

    // integrator_init_full(
    //     &prt->intgr, INTEGRATOR_RK4_ENV,
    //     0.016f, &state, nullptr, prt->env, &prt->proj.base.props);

    return tick_attach(tk, projectile_tick_update_cb, (void*)&prt->proj) == 0;
}

// ─────────────────────────────────────────────────────────
// Tick: 실시간 상태 적분
// ─────────────────────────────────────────────────────────

bool projectile_tick(
    projectile_tick_t* prt,
    float dt)
{
    if (!prt || dt <= 0.0f)
        return false;

    motion_state_t state;
    entity_dynamic_to_motion_state(&prt->proj.base, &state, NULL, NULL);

    prt->elapsed += dt;

    if (prt->elapsed < prt->max_time) {
        vec3_t pos_prev = state.linear.position;
        vec3_t vel_prev = state.linear.velocity;

        vec3_t env_accel = {0, 0, 0};
        if (prt->env && prt->env->environ_fn) {
            prt->env->environ_fn(prt->env, prt->env->userdata, &env_accel);
        }

        vec3_t thrust_accel = {0, 0, 0};
        if (prt->propulsion && prt->propulsion->fuel_remaining > 0.0f) {
            vec3_t guidance = {0, 0, 0};
            vec3_sub(&guidance, &prt->target.xf.pos, &state.linear.position);
            vec3_normalize(&guidance);

            projectile_t temp_proj = prt->proj;
            if (prt->guidance_fn) {
                guidance_target_info_t info = {};
                if (prt->env) info.env = *prt->env;
                info.target = prt->target;
                vec3_t vec;
                const vec3_t* g = prt->guidance_fn(
                    &temp_proj.base, dt, &info, &vec);
                if (g) vec3_unit(&guidance, g);
            }
            propulsion_update(prt->propulsion, dt);
            float thrust = propulsion_get_thrust(prt->propulsion);
            vec3_scale(&thrust_accel, &guidance, 
                thrust / prt->proj.base.props.mass);
        }

        vec3_add(&state.linear.acceleration, &env_accel, &thrust_accel);

        prt->intgr->state = state;
        prt->intgr->dt = dt;
        integrator_step(prt->intgr);
        state = prt->intgr->state;
        entity_dynamic_from_motion_state(&prt->proj.base, &state);

        bodyprops_apply_friction(
            &state.linear.velocity, &prt->proj.base.props, dt);

        trajectory_add_sample(prt->trajectory, prt->elapsed, &state);

        bool ground_hit = false;
        if (state.linear.position.y <= 0.0f){

            ground_hit = true;
        }

        float dist_prev = vec3_distance(&pos_prev, &prt->target.xf.pos);
        float dist_curr = vec3_distance(
            &state.linear.position, &prt->target.xf.pos);

        float target_radius = entity_size(&prt->target.base);

        if (detect_entity_collision_precise(
                &pos_prev, &vel_prev, 
                &state.linear.acceleration,
                &prt->target.xf.pos, target_radius,
                dt, prt->elapsed - dt, &prt->impact_pos, &prt->impact_time))
        {
            goto finalize;
        }

        if(ground_hit){

            if(detect_ground_collision_precise(
                &pos_prev, &state.linear.position,
                &vel_prev, &state.linear.acceleration,
                prt->elapsed, dt, &prt->impact_pos, &prt->impact_time)){

               goto finalize;
            }
        }
    }

    return false;

finalize:
    return true;
}


// ─────────────────────────────────────────────────────────
// Tick complete: trajectory 저장 + tick_detach
// ─────────────────────────────────────────────────────────

bool projectile_tick_complete(
    projectile_tick_t* prt,
    tick_t* tk)
{
    if (!prt) return false;

    if (tk)
    {
        // 등록된 tick_func를 제거
        tick_detach(tk, projectile_tick_update_cb, (void*)prt);
    }

    return true;
}
