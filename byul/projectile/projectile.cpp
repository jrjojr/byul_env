#include "projectile.h"
#include "projectile_predict.h"
#include "propulsion.h"
#include "guidance.h"
#include "entity_dynamic.h"
#include <math.h>    // sqrtf
#include <string.h>  // memset, memcpy
#include <iostream>

// ---------------------------------------------------------
// Shell Projectile
// ---------------------------------------------------------
void shell_projectile_init(shell_projectile_t* shell)
{
    if (!shell) return;
    memset(shell, 0, sizeof(shell_projectile_t));
    projectile_init(&shell->proj);
    shell->explosion_radius = 10.0f;
    shell->proj.on_hit = shell_projectile_hit_cb;
}

void shell_projectile_init_full(
    shell_projectile_t* shell,
    float damage,
    float explosion_radius)
{
    if (!shell) return;
    shell_projectile_init(shell);
    shell->proj.damage = damage;
    shell->explosion_radius = explosion_radius;
}

void shell_projectile_assign(
    shell_projectile_t* shell,
    const shell_projectile_t* src)
{
    if (!shell || !src) return;
    memcpy(shell, src, sizeof(shell_projectile_t));
}

// ---------------------------------------------------------
// Rocket
// ---------------------------------------------------------
void rocket_init(rocket_t* rocket)
{
    if (!rocket) return;
    memset(rocket, 0, sizeof(rocket_t));
    shell_projectile_init(&rocket->base);
    propulsion_init(&rocket->propulsion);
}

void rocket_init_full(
    rocket_t* rocket,
    float damage,
    float explosion_radius)
{
    if (!rocket) return;
    rocket_init(rocket);
    rocket->base.proj.damage = damage;
    rocket->base.explosion_radius = explosion_radius;
}

void rocket_assign(rocket_t* rocket, const rocket_t* src)
{
    if (!rocket || !src) return;
    memcpy(rocket, src, sizeof(rocket_t));
}

// ---------------------------------------------------------
// Missile
// ---------------------------------------------------------
void missile_init(missile_t* missile)
{
    if (!missile) return;
    memset(missile, 0, sizeof(missile_t));
    rocket_init(&missile->base);
    missile->guidance = guidance_lead; // linear guide
    missile->guidance_userdata = NULL;
}

void missile_init_full(
    missile_t* missile,
    float damage,
    float explosion_radius)
{
    if (!missile) return;
    missile_init(missile);
    missile->base.base.proj.damage = damage;
    missile->base.base.explosion_radius = explosion_radius;
}

void missile_assign(missile_t* missile, const missile_t* src)
{
    if (!missile || !src) return;
    memcpy(missile, src, sizeof(missile_t));
}

// ---------------------------------------------------------
// Patriot
// ---------------------------------------------------------
void patriot_init(patriot_t* patriot)
{
    if (!patriot) return;
    memset(patriot, 0, sizeof(patriot_t));
    missile_init(&patriot->base);
    patriot->guidance = guidance_predict_accel_env; // non linear
    patriot->guidance_userdata = NULL;
}

void patriot_init_full(
    patriot_t* patriot,
    float damage,
    float explosion_radius)
{
    if (!patriot) return;
    patriot_init(patriot);
    patriot->base.base.base.proj.damage = damage;
    patriot->base.base.base.explosion_radius = explosion_radius;
}

void patriot_assign(patriot_t* patriot, const patriot_t* src)
{
    if (!patriot || !src) return;
    memcpy(patriot, src, sizeof(patriot_t));
}



/**
 * @brief Calculates an appropriate simulation time step (dt) based 
 *  on force and mass.
 *
 * This function estimates the suitable dt for 
 * trajectory simulation to balance
 * accuracy and performance. It uses the initial applied 
 * force and mass to compute
 * acceleration and velocity, then determines dt to 
 * ensure an adequate number of
 * samples across the expected travel distance.
 *
 * @param[out] dt_out   Pointer to the output dt value (in seconds)
 * @param[in]  dir      Direction vector (normalized, 
 *     not used internally but kept for future use)
 * @param[in]  force    Applied initial force (in Newtons)
 * @param[in]  mass     Mass of the projectile (in kilograms)
 */
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

    // Compute dt = total_distance / (sample_count × velocity)
    float estimated_dt = XFORM_MAX_DISTANCE / (sample_count * velocity);

    // Clamp dt to acceptable range
    if (estimated_dt < MIN_DELTA_TIME) estimated_dt = MIN_DELTA_TIME;
    if (estimated_dt > MAX_DELTA_TIME) estimated_dt = MAX_DELTA_TIME;

    *dt_out = estimated_dt;
}

/**
 * @brief Estimates a suitable simulation max time based on 
 * initial force and mass.
 *
 * This function computes the total expected simulation time 
 * (max_time) required
 * for a projectile to travel a predefined distance, 
 * based on its mass and the applied force.
 * It uses estimated velocity to calculate 
 * how long it would take to cover the full range.
 *
 * @param[out] max_time_out Pointer to output value (in seconds)
 * @param[in]  dir          Normalized direction vector (currently unused)
 * @param[in]  force        Applied initial force (Newtons)
 * @param[in]  mass         Mass of the projectile (kilograms)
 */
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

bool projectile_launch(
    const projectile_t* proj,
    const vec3_t* dir,               // normalized
    float initial_force,             // 입력된 힘 (N)
    const environ_t* env,            // 환경 (update_func 포함 가능)
    projectile_result_t* out)
{
    if (!proj || !dir || !out) return false;

    // 기본 시뮬레이션 설정
    float max_time = MAX_SIM_TIME;  // 예: 100.0f
    float dt = DELTA_TIME;          // 예: 0.048828f

    calc_suitable_dt(&dt, dir, initial_force, proj->base.props.mass);

    calc_suitable_max_time(
        &max_time, dir, initial_force, proj->base.props.mass);    

    // 질량이 있어야 힘을 속도로 변환 가능
    float mass = proj->base.props.mass;
    if (mass <= 0.0f) return false;

    // a = F / m
    vec3_t accel;
    vec3_scale(&accel, dir, initial_force / mass);

    // v = a * dt
    vec3_t velocity;
    vec3_scale(&velocity, &accel, dt);

    // 발사체 복사 및 초기 속도 설정
    projectile_t self;
    projectile_assign(&self, proj);
    self.base.velocity = velocity;

    // 목표 엔티티: 궤적 계산 시 필요하므로 기본 위치만 지정
    entity_dynamic_t target;
    entity_dynamic_init(&target);
    // 필요시 target.xf.pos 설정 가능

    return projectile_predict(
        out,
        &self,
        &target,
        max_time,
        dt,
        env,
        NULL,  // propulsion
        NULL   // guidance
    );
}

// ---------------------------------------------------------
// Shell
// ---------------------------------------------------------
BYUL_API bool shell_projectile_launch(
    const shell_projectile_t* shell,
    const vec3_t* dir,             // normalized
    float initial_force,               // 초기 힘 크기
    const environ_t* env,       // 환경 + update_func 포함 가능
    projectile_result_t* out)
{
    if (!shell || !dir || !out) return false;

    // 기본 시뮬레이션 설정
    float max_time = MAX_SIM_TIME;  // 예: 100.0f
    float dt = DELTA_TIME;          // 예: 0.048828f

    calc_suitable_dt(&dt, dir, initial_force, shell->proj.base.props.mass);
    
    calc_suitable_max_time(
        &max_time, dir, initial_force, shell->proj.base.props.mass);    

    // 질량이 있어야 힘을 속도로 변환 가능
    float mass = shell->proj.base.props.mass;
    if (mass <= 0.0f) return false;

    // a = F / m
    vec3_t accel;
    vec3_scale(&accel, dir, initial_force / mass);

    // v = a * dt
    vec3_t velocity;
    vec3_scale(&velocity, &accel, dt);

    // 발사체 복사 및 초기 속도 설정
    projectile_t self;
    projectile_assign(&self, &shell->proj);
    self.base.velocity = velocity;

    // 목표 엔티티: 궤적 계산 시 필요하므로 기본 위치만 지정
    entity_dynamic_t target;
    entity_dynamic_init(&target);
    // 필요시 target.xf.pos 설정 가능

    return projectile_predict(
        out,
        &self,
        &target,
        max_time,
        dt,
        env,
        NULL,  // propulsion
        NULL   // guidance
    );
}

// ---------------------------------------------------------
// Rocket
// ---------------------------------------------------------
bool rocket_launch(
    rocket_t* rocket,
    const vec3_t* target,
    float initial_force,
    const environ_t* env,
    projectile_result_t* out)
{
    if (!rocket || !target || !out) return false;

    // 기본 시뮬레이션 설정
    float max_time = MAX_SIM_TIME;  // 예: 100.0f
    float dt = DELTA_TIME;          // 예: 0.048828f

    vec3_t dir;
    vec3_unit(&dir, target);
    calc_suitable_dt(&dt, &dir, initial_force, 
        rocket->base.proj.base.props.mass);
    
    calc_suitable_max_time(
        &max_time, &dir, initial_force, rocket->base.proj.base.props.mass);    

    // 질량이 있어야 힘을 속도로 변환 가능
    float mass = rocket->base.proj.base.props.mass;
    if (mass <= 0.0f) return false;

    // a = F / m
    vec3_t accel;
    vec3_scale(&accel, &dir, initial_force / mass);

    // v = a * dt
    vec3_t velocity;
    vec3_scale(&velocity, &accel, dt);

    // 발사체 복사 및 초기 속도 설정
    projectile_t self;
    projectile_assign(&self, &rocket->base.proj);
    self.base.velocity = velocity;

    entity_dynamic_t entdyn;
    entity_dynamic_init(&entdyn);
    entdyn.xf.pos = *target;

    return projectile_predict(
        out,
        &self,
        &entdyn,
        max_time,
        dt,
        env,
        &rocket->propulsion,  // propulsion
        NULL   // guidance
    );
}

// ---------------------------------------------------------
// Missile
// ---------------------------------------------------------
bool missile_launch(
    missile_t* missile,
    const vec3_t* target,
    float initial_force,
    const environ_t* env,
    projectile_result_t* out)
{
    if (!missile || !target || !out) return false;

    // 기본 시뮬레이션 설정
    float max_time = MAX_SIM_TIME;  // 예: 100.0f
    float dt = DELTA_TIME;          // 예: 0.048828f

    vec3_t dir;
    vec3_unit(&dir, target);
    calc_suitable_dt(&dt, &dir, initial_force, 
        missile->base.base.proj.base.props.mass);
    
    calc_suitable_max_time(
        &max_time, &dir, initial_force, 
        missile->base.base.proj.base.props.mass);    

    // 질량이 있어야 힘을 속도로 변환 가능
    float mass = missile->base.base.proj.base.props.mass;
    if (mass <= 0.0f) return false;

    // a = F / m
    vec3_t accel;
    vec3_scale(&accel, &dir, initial_force / mass);

    // v = a * dt
    vec3_t velocity;
    vec3_scale(&velocity, &accel, dt);

    // 발사체 복사 및 초기 속도 설정
    projectile_t self;
    projectile_assign(&self, &missile->base.base.proj);
    self.base.velocity = velocity;

    entity_dynamic_t entdyn;
    entity_dynamic_init(&entdyn);
    entdyn.xf.pos = *target;

    return projectile_predict(out, &self, &entdyn,
                              MAX_SIM_TIME, DELTA_TIME, env,
                              &missile->base.propulsion, missile->guidance);
}

// ---------------------------------------------------------
// Patriot
// ---------------------------------------------------------
bool patriot_launch(
    patriot_t* patriot,
    const entity_dynamic_t* target,
    float initial_force,
    const environ_t* env,
    projectile_result_t* out)
{
    if (!patriot || !target || !out) return false;

    // 기본 시뮬레이션 설정
    float max_time = MAX_SIM_TIME;  // 예: 100.0f
    float dt = DELTA_TIME;          // 예: 0.048828f

    vec3_t dir;
    vec3_sub(&dir, &target->xf.pos, &patriot->base.base.base.proj.base.xf.pos);
    vec3_normalize(&dir);
    calc_suitable_dt(&dt, &dir, initial_force, 
        patriot->base.base.base.proj.base.props.mass);
    
    calc_suitable_max_time(
        &max_time, &dir, initial_force, 
        patriot->base.base.base.proj.base.props.mass);    

    // 질량이 있어야 힘을 속도로 변환 가능
    float mass = patriot->base.base.base.proj.base.props.mass;
    if (mass <= 0.0f) return false;

    // a = F / m
    vec3_t accel;
    vec3_scale(&accel, &dir, initial_force / mass);

    // v = a * dt
    vec3_t velocity;
    vec3_scale(&velocity, &accel, dt);

    // 발사체 복사 및 초기 속도 설정
    projectile_t self;
    projectile_assign(&self, &patriot->base.base.base.proj);
    self.base.velocity = velocity;

    return projectile_predict(out, &self, target,
                              MAX_SIM_TIME, DELTA_TIME, env,
                              &patriot->base.base.propulsion, 
                              patriot->guidance);
}

void shell_projectile_hit_cb(const void* projectile, void* userdata)
{
    (void)userdata;

    const shell_projectile_t* proj = (const shell_projectile_t*)projectile;
    if (!proj) {
        printf("[shell projectile] hit callback called with null projectile\n");
        return;
    }

    printf(
    "[shell projectile] hit cb damaged : %.3f, explosion_radius : %.3f\n", 
    proj->proj.damage, proj->explosion_radius);
}
