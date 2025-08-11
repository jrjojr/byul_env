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
    shell->proj.hit_userdata = &shell->explosion_radius;
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



bool projectile_launch(
    const projectile_t* proj,
    const vec3_t* dir,
    float initial_force_scalar,
    const environ_t* env,
    const ground_t* ground,    
    projectile_result_t* out)
{
    if (!proj || !dir || !out) return false;

    float dt = DELTA_TIME;          // 0.048828f

    float mass = proj->base.props.mass;
    if (mass <= 0.0f) return false;

    // a = F / m
    vec3_t accel;
    vec3_scale(&accel, dir, initial_force_scalar / mass);

    // v = a * dt
    vec3_t velocity;
    vec3_scale(&velocity, &accel, dt);

    projectile_t self;
    projectile_assign(&self, proj);
    self.base.velocity = velocity;

    entity_dynamic_t target;
    entity_dynamic_init(&target);

    return projectile_predict(
        &self,
        dt,
        &target,
        env,
        ground,
        NULL,  // propulsion
        NULL,   // guidance
        out
    );
}

bool projectile_launch_tick(
    const projectile_t* proj,
    const vec3_t* dir,
	float initial_force_scalar,
    const environ_t* env,
    const ground_t* ground,    
    projectile_tick_t* prt)
{
    if (!proj || !dir || !prt) return false;

    // prt->bool_impacted = true;
    // prt->trajectory = trajectory_create_full(MAX_SAMPLE_COUNT);
    // prt->env = env;
    environ_assign(prt->env, env);

    // prt->proj = proj;
    projectile_assign(&prt->proj, proj);

    vec3_t target_vel = proj->base.xf.pos;
    // f = ma; a = f/m; vel = v0 + a(t)
    // a = initial_Force / mass
    float accel = initial_force_scalar / proj->base.props.mass;
    
    vec3_t a_dir = *dir;
    vec3_normalize(&a_dir);
    vec3_madd(&target_vel, &target_vel, &a_dir, accel);

    entity_dynamic_t target = {};
    target.velocity = target_vel;

    // prt->target = &target;
    entity_dynamic_assign(&prt->target, &target);

    return projectile_tick_prepare(prt, prt->tick);
}

// ---------------------------------------------------------
// Shell
// ---------------------------------------------------------
bool shell_projectile_launch(
    const shell_projectile_t* shell,
    const vec3_t* dir,
    float initial_force_scalar,
    const environ_t* env,
    const ground_t* ground,    
    projectile_result_t* out)
{
    if (!shell || !dir || !out) return false;

    float dt = DELTA_TIME;          // 0.048828f

    float mass = shell->proj.base.props.mass;
    if (mass <= 0.0f) return false;

    // a = F / m
    vec3_t accel;
    vec3_scale(&accel, dir, initial_force_scalar / mass);

    // v = a * dt
    vec3_t velocity = {};
    vec3_scale(&velocity, &accel, dt);

    shell_projectile_t self = {};
    shell_projectile_assign(&self, shell);
    self.proj.base.velocity = velocity;

    entity_dynamic_t target;
    entity_dynamic_init(&target);

    return projectile_predict(
        &self.proj,
        dt,

        &target,
        env,
        ground,
        NULL,  // propulsion
        NULL,   // guidance

        out
    );
}

// ---------------------------------------------------------
// Rocket
// ---------------------------------------------------------
bool rocket_launch(
    rocket_t* rocket,
    const vec3_t* target,
    float initial_force_scalar,
    const environ_t* env,
    const ground_t* ground,    
    projectile_result_t* out)
{
    if (!rocket || !target || !out) return false;

    float dt = DELTA_TIME;          // 0.048828f

    vec3_t dir;
    vec3_unit(&dir, target);

    float mass = rocket->base.proj.base.props.mass;
    if (mass <= 0.0f) return false;

    // a = F / m
    vec3_t accel;
    vec3_scale(&accel, &dir, initial_force_scalar / mass);

    // v = a * dt
    vec3_t velocity;
    vec3_scale(&velocity, &accel, dt);

    projectile_t self;
    projectile_assign(&self, &rocket->base.proj);
    self.base.velocity = velocity;

    entity_dynamic_t entdyn;
    entity_dynamic_init(&entdyn);
    entdyn.xf.pos = *target;

    return projectile_predict(
        &self,
        dt,
        &entdyn,
        env,
        ground,
        &rocket->propulsion,  // propulsion
        NULL,   // guidance
        out        
    );
}

// ---------------------------------------------------------
// Missile
// ---------------------------------------------------------
bool missile_launch(
    missile_t* missile,
    const vec3_t* target,
    float initial_force_scalar,
    const environ_t* env,
    const ground_t* ground,    
    projectile_result_t* out)
{
    if (!missile || !target || !out) return false;

    float dt = DELTA_TIME;          //  0.048828f

    vec3_t dir;
    vec3_unit(&dir, target);

    float mass = missile->base.base.proj.base.props.mass;
    if (mass <= 0.0f) return false;

    // a = F / m
    vec3_t accel;
    vec3_scale(&accel, &dir, initial_force_scalar / mass);

    // v = a * dt
    vec3_t velocity;
    vec3_scale(&velocity, &accel, dt);

    projectile_t self;
    projectile_assign(&self, &missile->base.base.proj);
    self.base.velocity = velocity;

    entity_dynamic_t entdyn;
    entity_dynamic_init(&entdyn);
    entdyn.xf.pos = *target;

    return projectile_predict(
        &self, 
        dt,
        &entdyn,
        env,
        ground,
        &missile->base.propulsion, 
        missile->guidance,
        out);
}

// ---------------------------------------------------------
// Patriot
// ---------------------------------------------------------
bool patriot_launch(
    patriot_t* patriot,
    const entity_dynamic_t* target,
    float initial_force_scalar,
    const environ_t* env,
    const ground_t* ground,    
    projectile_result_t* out)
{
    if (!patriot || !target || !out) return false;

    float dt = DELTA_TIME;          // 0.048828f

    vec3_t dir;
    vec3_sub(&dir, &target->xf.pos, &patriot->base.base.base.proj.base.xf.pos);
    vec3_normalize(&dir);

    float mass = patriot->base.base.base.proj.base.props.mass;
    if (mass <= 0.0f) return false;

    // a = F / m
    vec3_t accel;
    vec3_scale(&accel, &dir, initial_force_scalar / mass);

    // v = a * dt
    vec3_t velocity;
    vec3_scale(&velocity, &accel, dt);

    projectile_t self;
    projectile_assign(&self, &patriot->base.base.base.proj);
    self.base.velocity = velocity;

    return projectile_predict(
        &self, 
        dt,         
        target,
        env,
        ground,
        &patriot->base.base.propulsion, 
        patriot->guidance,
        out
        );
}

void shell_projectile_hit_cb(const projectile_t* projectile, void* userdata)
{
    (void)userdata;

    const projectile_t* proj = projectile;
    float explosion_radius = *(float*)userdata;

    if (!proj) {
        printf("[shell projectile] hit callback called with null projectile\n");
        return;
    }

    printf(
    "[shell projectile] hit cb damaged : %.3f, explosion_radius : %.3f\n", 
    proj->damage, explosion_radius);
}
