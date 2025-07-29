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

// 5 sec
#ifndef MAX_TIME
#define MAX_TIME 5.0f
#endif

// 0.01 = 100 hz
#ifndef TIME_STEP
#define TIME_STEP 0.01f 
#endif

bool projectile_launch(
    const projectile_t* proj,
    const vec3_t* target,
    float initial_speed,
    const environ_t* env,
    projectile_result_t* out)
{
    if (!proj || !target || !out) return false;

    float max_time = MAX_TIME;
    float time_step = TIME_STEP;

    vec3_t dir;
    vec3_sub(&dir, target, &proj->base.xf.pos);   // dir = target - my_pos
    vec3_normalize(&dir);
    vec3_scale(&dir, &dir, initial_speed);

    projectile_t self;
    projectile_assign(&self, proj);

    self.base.velocity = dir;

    entity_dynamic_t entdyn;
    entity_dynamic_init(&entdyn);
    entdyn.xf.pos = *target;

    return projectile_predict(out, &self, &entdyn,
                              max_time, time_step, env, NULL, NULL);
}

// ---------------------------------------------------------
// Shell
// ---------------------------------------------------------
bool shell_projectile_launch(
    const shell_projectile_t* shell,
    const vec3_t* target,
    float initial_speed,
    const environ_t* env,
    projectile_result_t* out)
{
    if (!shell || !target || !out) return false;

    vec3_t dir;
    vec3_sub(&dir, target, &shell->proj.base.xf.pos);
    vec3_normalize(&dir);
    vec3_scale(&dir, &dir, initial_speed);

    shell_projectile_t self;
    shell_projectile_assign(&self, shell);
    self.proj.base.velocity = dir;

    entity_dynamic_t entdyn;
    entity_dynamic_init(&entdyn);
    entdyn.xf.pos = *target;

    return projectile_predict(out, &self.proj, &entdyn,
                              MAX_TIME, TIME_STEP, env, NULL, NULL);
}

// ---------------------------------------------------------
// Rocket
// ---------------------------------------------------------
bool rocket_launch(
    const rocket_t* rocket,
    const vec3_t* target,
    float initial_speed,
    const environ_t* env,
    projectile_result_t* out)
{
    if (!rocket || !target || !out) return false;

    vec3_t dir;
    vec3_sub(&dir, target, &rocket->base.proj.base.xf.pos);
    vec3_normalize(&dir);
    vec3_scale(&dir, &dir, initial_speed);

    rocket_t self;
    rocket_assign(&self, rocket);
    self.base.proj.base.velocity = dir;

    entity_dynamic_t entdyn;
    entity_dynamic_init(&entdyn);
    entdyn.xf.pos = *target;

    return projectile_predict(out, &self.base.proj, &entdyn,
                              MAX_TIME, TIME_STEP, env, &self.propulsion, NULL);
}

// ---------------------------------------------------------
// Missile
// ---------------------------------------------------------
bool missile_launch(
    const missile_t* missile,
    const vec3_t* target,
    float initial_speed,
    const environ_t* env,
    projectile_result_t* out)
{
    if (!missile || !target || !out) return false;

    vec3_t dir;
    vec3_sub(&dir, target, &missile->base.base.proj.base.xf.pos);
    vec3_normalize(&dir);
    vec3_scale(&dir, &dir, initial_speed);

    missile_t self;
    missile_assign(&self, missile);
    self.base.base.proj.base.velocity = dir;

    entity_dynamic_t entdyn;
    entity_dynamic_init(&entdyn);
    entdyn.xf.pos = *target;

    return projectile_predict(out, &self.base.base.proj, &entdyn,
                              MAX_TIME, TIME_STEP, env,
                              &self.base.propulsion, self.guidance);
}

// ---------------------------------------------------------
// Patriot
// ---------------------------------------------------------
bool patriot_launch(
    const patriot_t* patriot,
    const entity_dynamic_t* target,
    float initial_speed,
    const environ_t* env,
    projectile_result_t* out)
{
    if (!patriot || !target || !out) return false;

    vec3_t dir;
    vec3_sub(&dir, &target->xf.pos, &patriot->base.base.base.proj.base.xf.pos);
    vec3_normalize(&dir);
    vec3_scale(&dir, &dir, initial_speed);

    patriot_t self;
    patriot_assign(&self, patriot);
    self.base.base.base.proj.base.velocity = dir;

    return projectile_predict(out, &self.base.base.base.proj, target,
                              MAX_TIME, TIME_STEP, env,
                              &self.base.base.propulsion, self.guidance);
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
