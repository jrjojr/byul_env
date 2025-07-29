#include "doctest.h"
#include "projectile_predict.h"
#include "guidance.h"
#include "propulsion.h"
#include "trajectory.h"
#include "xform.h"
#include "entity_dynamic.h"
#include <cmath>
#include <stdio.h>

static projectile_t create_test_projectile(
    const vec3_t& pos, const vec3_t& vel, float mass = 1.0f) {
    projectile_t proj{};
    projectile_init(&proj);
    xform_set_position(&proj.base.xf, &pos);
    proj.base.velocity = vel;
    proj.base.props.mass = mass;
    return proj;
}

static entity_dynamic_t create_test_target(
    const vec3_t& pos, const vec3_t& vel = {0, 0, 0}) {
    entity_dynamic_t target{};
    entity_dynamic_init(&target);
    xform_set_position(&target.xf, &pos);
    target.velocity = vel;
    return target;
}

TEST_CASE("projectile_predict - ground collision") {
    MESSAGE("\nprojectile_predict - ground collision");
    vec3_t start_pos = {0, 500, 0};
    vec3_t target_pos = {0, 0, 0};
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
        result,          // [out] Store projectile trajectory
        &proj,           // [in]  Projectile entity
        &entdyn,         // [in]  Target entity (for collision detection)
        500.0f,          // [in]  max_time: Maximum prediction time (seconds)
        1.0f,            // [in]  time_step: Simulation sampling interval (seconds)
        &env,            // [in]  Environment data (gravity, wind, etc.)
        nullptr,         // [in]  Propulsion (nullptr if none)
        guidance_none    // [in]  Guidance function pointer (guidance_none if none)
    );

    CHECK(hit == true);
    CHECK(result->valid == true);
    CHECK(result->impact_time > 0.0f);
    CHECK(result->impact_pos.y == doctest::Approx(0.0f).epsilon(1.0f));

    trajectory_print(result->trajectory);
    char buf[64];
    printf("impact time : %f, impact pos : %s\n", result->impact_time, 
        vec3_to_string(&result->impact_pos, buf, 64));    
    projectile_result_destroy(result);
}

TEST_CASE("projectile_predict - static target hit") {
    vec3_t start_pos = {0, 10, 0};
    vec3_t start_vel = {15, 0, 0};
    projectile_t proj = create_test_projectile(start_pos, start_vel);

    entity_dynamic_t target;
    entity_dynamic_init(&target);
    target.xf.pos = {100, 10, 0};

    projectile_result_t* result = projectile_result_create();

    bool hit = projectile_predict(result, 
                                    &proj,
                                  &target,
                                  50.0f,
                                  2.0f,
                                  nullptr,
                                  nullptr,
                                guidance_point);

    CHECK(hit == true);
    CHECK(result->valid == true);
    CHECK(result->impact_time > 0.0f);
    // CHECK(result->impact_pos.x == 100.0f).approx(1.0f);
    CHECK(result->impact_pos.x 
        == doctest::Approx(100.0f).epsilon(entity_size(&proj.base.base)));


    trajectory_print(result->trajectory);
    char buf[64];
    printf("impact time : %f, impact pos : %s\n", result->impact_time, 
        vec3_to_string(&result->impact_pos, buf, 64));    

    projectile_result_destroy(result);
}

TEST_CASE("projectile_predict - moving target with lead guidance") {
    vec3_t start_pos = {0, 0, 0};
    vec3_t start_vel = {12, 0, 0};
    projectile_t proj = create_test_projectile(start_pos, start_vel);

    entity_dynamic_t target = create_test_target({15, 0, 0}, {-2, 0, 0});

    projectile_result_t* result = projectile_result_create();

    bool hit = projectile_predict(result, &proj,
                                  &target,
                                  5.0f,
                                  0.1f,
                                  nullptr,
                                  nullptr,
                                  guidance_lead
                                  );

    CHECK(hit == true);
    CHECK(result->valid == true);
    CHECK(result->impact_time > 0.0f);
    CHECK(result->impact_pos.x > 10.0f);
    trajectory_print(result->trajectory);

    char buf[64];
    printf("impact time : %f, impact pos : %s\n", result->impact_time, 
        vec3_to_string(&result->impact_pos, buf, 64));        

    projectile_result_destroy(result);
}

TEST_CASE("projectile_predict - moving target with lead guidance with propulsion") {
    vec3_t start_pos = {0, 0, 0};
    vec3_t start_vel = {12, 0, 0};
    projectile_t proj = create_test_projectile(start_pos, start_vel);

    entity_dynamic_t target = create_test_target({15, 0, 0}, {-2, 0, 0});

    projectile_result_t* result = projectile_result_create();

    propulsion_t propulsion;
    propulsion_init(&propulsion);
    bool hit = projectile_predict(result, 
                                    &proj,
                                  &target,
                                  5.0f,
                                  0.1f,
                                  nullptr,
                                  &propulsion,
                                 guidance_lead

                                );

    CHECK(hit == true);
    CHECK(result->valid == true);
    CHECK(result->impact_time > 0.0f);
    CHECK(result->impact_pos.x > 10.0f);
    trajectory_print(result->trajectory);
    char buf[64];
    printf("impact time : %f, impact pos : %s\n", result->impact_time, 
        vec3_to_string(&result->impact_pos, buf, 64));        
    projectile_result_destroy(result);
}

TEST_CASE("projectile_predict - moving target lead vs propulsion") {
    vec3_t start_pos = {0, 0, 0};
    vec3_t start_vel = {6, 0, 0};
    projectile_t proj = create_test_projectile(start_pos, start_vel);

    entity_dynamic_t target = create_test_target({20, 0, 0}, {-2, 0, 0});


    projectile_result_t* result_no_prop = projectile_result_create();
    bool hit_no_prop = projectile_predict(result_no_prop, &proj,
                                          &target,
                                          5.0f,
                                          0.1f,
                                          nullptr,
                                          nullptr,
                                          guidance_lead
    );

    CHECK(hit_no_prop == true);
    CHECK(result_no_prop->valid == true);
    float impact_time_no_prop = result_no_prop->impact_time;
    float impact_x_no_prop = result_no_prop->impact_pos.x;
    trajectory_print(result_no_prop->trajectory);

    char buf[64];
    printf("result_no_prop impact time : %f, impact pos : %s\n", result_no_prop->impact_time, 
        vec3_to_string(&result_no_prop->impact_pos, buf, 64));            
    projectile_result_destroy(result_no_prop);

    projectile_result_t* result_prop = projectile_result_create();
    propulsion_t propulsion;
    propulsion_init(&propulsion);
    propulsion.current_thrust = 0.1f;
    propulsion.max_thrust = 30.0f;
    propulsion.fuel_remaining = 10.0f;
    propulsion.burn_rate = 0.1f;
    propulsion.active = true;

    bool hit_prop = projectile_predict(result_prop, &proj,
                                       &target,
                                       5.0f,
                                       0.1f,
                                       nullptr,
                                       &propulsion,

                                       guidance_lead

                                                    );

    CHECK(hit_prop == true);
    CHECK(result_prop->valid == true);
    float impact_time_prop = result_prop->impact_time;
    float impact_x_prop = result_prop->impact_pos.x;
    trajectory_print(result_prop->trajectory);

    printf("result_prop impact time : %f, impact pos : %s\n", 
        result_prop->impact_time, 
        vec3_to_string(&result_prop->impact_pos, buf, 64));            
    projectile_result_destroy(result_prop);

    CHECK(impact_time_prop <= impact_time_no_prop);
}

