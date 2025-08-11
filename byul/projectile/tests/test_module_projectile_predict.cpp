#include "doctest.h"
#include "projectile_predict.h"
#include "guidance.h"
#include "propulsion.h"
#include "trajectory.h"
#include "xform.h"
#include "entity_dynamic.h"
#include <cmath>
#include <stdio.h>

TEST_CASE("projectile_predict - ground collision") {
    MESSAGE("\nprojectile_predict - ground collision");
    vec3_t start_pos = {0, 500, 0};
    vec3_t target_pos = {100, 100, 0};
    projectile_t proj;
    projectile_init(&proj);
    proj.base.xf.pos = start_pos;

    entity_dynamic_t entdyn;
    entity_dynamic_init(&entdyn);
    entdyn.xf.pos = target_pos;

    projectile_result_t* result = projectile_result_create();

    environ_t env;
    environ_init(&env);

    ground_t ground;
    ground_init(&ground);

    bool hit = projectile_predict(
        &proj,           // [in]  Projectile entity
        1.0f,            // [in]  dt: Simulation sampling interval (seconds)
        &entdyn,         // [in]  Target entity (for collision detection)

        &env,            // [in]  Environment data (gravity, wind, etc.)
        &ground,
        nullptr,         // [in]  Propulsion (nullptr if none)
        guidance_none,    // [in]  Guidance function pointer (guidance_none if none)
                result          // [out] Store projectile trajectory
    );

    CHECK(hit == true);
    CHECK(result->bool_impacted == true);
    CHECK(result->impact_time > 0.0f);
    CHECK(result->impact_pos.y == doctest::Approx(0.0f).epsilon(1.0f));

    trajectory_print(result->trajectory);
    char buf[64];
    printf("impact time : %f, impact pos : %s\n", result->impact_time, 
        vec3_to_string(&result->impact_pos, 64, buf));    
    projectile_result_destroy(result);
}

TEST_CASE("projectile_predict - static target hit") {
    MESSAGE("\nprojectile_predict - static target hit");
    vec3_t start_pos = {0, 500, 0};
    vec3_t target_pos = {0, 100, 0};
    projectile_t proj;
    projectile_init(&proj);
    proj.base.xf.pos = start_pos;

    entity_dynamic_t entdyn;
    entity_dynamic_init(&entdyn);
    entdyn.xf.pos = target_pos;

    projectile_result_t* result = projectile_result_create();

    environ_t env;
    environ_init(&env);

    bool hit = projectile_predict(
        &proj,           // [in]  Projectile entity
        1.0f,            // [in]  dt: Simulation sampling interval (seconds)
        &entdyn,         // [in]  Target entity (for collision detection)
        &env,            // [in]  Environment data (gravity, wind, etc.)
        nullptr,
        nullptr,         // [in]  Propulsion (nullptr if none)
        guidance_none,    // [in]  Guidance function pointer (guidance_none if none)
        result          // [out] Store projectile trajectory        
    );

    CHECK(hit == true);
    CHECK(result->bool_impacted == true);
    CHECK(result->impact_time > 0.0f);
    CHECK(result->impact_pos.y == doctest::Approx(0.0f).epsilon(1.0f));

    trajectory_print(result->trajectory);
    char buf[64];
    printf("impact time : %f, impact pos : %s\n", result->impact_time, 
        vec3_to_string(&result->impact_pos, 64, buf));    
    projectile_result_destroy(result);
}

TEST_CASE("projectile_predict - moving target with lead guidance") {
    MESSAGE("\nprojectile_predict - moving target with lead guidance");    
    vec3_t start_pos = {0, 10, 0};
    vec3_t start_vel = {12, 0, 0};
    vec3_t target_pos = {15, 10, 0};
    vec3_t target_vel = {-2, 0, 0};

    projectile_t proj;
    projectile_init(&proj);
    proj.base.xf.pos = start_pos;
    proj.base.velocity = start_vel;

    entity_dynamic_t target;
    entity_dynamic_init(&target);
    target.xf.pos = target_pos;
    target.velocity = target_vel;

    projectile_result_t* result = projectile_result_create();

    bool hit = projectile_predict(
        &proj,
        0.1f,
        &target,
        nullptr,
        nullptr,
        nullptr,
        guidance_lead,
        result
        );

    CHECK(hit == true);
    CHECK(result->bool_impacted == true);
    CHECK(result->impact_time > 0.0f);
    CHECK(result->impact_pos.x > 10.0f);
    trajectory_print(result->trajectory);

    char buf[64];
    printf("impact time : %f, impact pos : %s\n", result->impact_time, 
        vec3_to_string(&result->impact_pos, 64, buf));        

    projectile_result_destroy(result);
}

TEST_CASE("projectile_predict - moving target with lead guidance with propulsion") {
    MESSAGE("\nprojectile_predict - moving target with lead guidance with propulsion");    
    projectile_t proj;
    projectile_init(&proj);
    proj.base.xf.pos = {0.0f, 10.0f, 0.0f};
    proj.base.velocity = {12.0f, 0.0f, 0.0f};

    entity_dynamic_t target;
    entity_dynamic_init(&target);
    target.xf.pos = {15.0f, 10.0f, 0.0f};
    target.velocity = {-2.0f, 0.0f, 0.0f};

    projectile_result_t* result = projectile_result_create();

    propulsion_t propulsion;
    propulsion_init(&propulsion);
    bool hit = projectile_predict(
                                    &proj,
                                  0.1f,
                                    &target,
                                    NULL,
                                  NULL,
                                  &propulsion,
                                 guidance_lead,
                                 result
                                );

    CHECK(hit == true);
    CHECK(result->bool_impacted == true);
    CHECK(result->impact_time > 0.0f);
    CHECK(result->impact_pos.x > 10.0f);
    trajectory_print(result->trajectory);
    char buf[64];
    printf("impact time : %f, impact pos : %s\n", result->impact_time, 
        vec3_to_string(&result->impact_pos, 64, buf));        
    projectile_result_destroy(result);
}

TEST_CASE("projectile_predict - moving target lead vs propulsion") {
    MESSAGE("\nprojectile_predict - moving target lead vs propulsion");    
    vec3_t start_pos = {0, 10, 0};
    vec3_t start_vel = {6, 0, 0};

    vec3_t target_pos = {20, 10, 0};
    vec3_t target_vel = {-2, 0, 0};

    projectile_t proj;
    projectile_init(&proj);
    proj.base.xf.pos = start_pos;
    proj.base.velocity = start_vel;

    entity_dynamic_t target;
    entity_dynamic_init(&target);
    target.xf.pos = target_pos;
    target.velocity = target_vel;

    projectile_result_t* result_no_prop = projectile_result_create();
    bool hit_no_prop = projectile_predict(
        &proj,
        0.1f,
        &target,
        nullptr,
        nullptr,
        nullptr,
        guidance_lead,
        result_no_prop
    );

    CHECK(hit_no_prop == true);
    CHECK(result_no_prop->bool_impacted == true);
    float impact_time_no_prop = result_no_prop->impact_time;
    float impact_x_no_prop = result_no_prop->impact_pos.x;
    trajectory_print(result_no_prop->trajectory);

    char buf[64];
    printf("result_no_prop impact time : %f, impact pos : %s\n", 
        result_no_prop->impact_time, 
        vec3_to_string(&result_no_prop->impact_pos, 64, buf));            
    projectile_result_destroy(result_no_prop);

    projectile_result_t* result_prop = projectile_result_create();
    propulsion_t propulsion;
    propulsion_init(&propulsion);
    propulsion.current_thrust = 0.1f;
    propulsion.max_thrust = 30.0f;
    propulsion.fuel_remaining = 10.0f;
    propulsion.burn_rate = 0.1f;
    propulsion.active = true;

    bool hit_prop = projectile_predict(
        &proj,
        0.1f,
        &target,
        nullptr,
        nullptr,
        &propulsion,
        guidance_lead,
        result_prop
    );

    CHECK(hit_prop == true);
    CHECK(result_prop->bool_impacted == true);
    float impact_time_prop = result_prop->impact_time;
    float impact_x_prop = result_prop->impact_pos.x;
    trajectory_print(result_prop->trajectory);

    printf("result_prop impact time : %f, impact pos : %s\n", 
        result_prop->impact_time, 
        vec3_to_string(&result_prop->impact_pos, 64, buf));            
    projectile_result_destroy(result_prop);

    CHECK(impact_time_prop <= impact_time_no_prop);
}

