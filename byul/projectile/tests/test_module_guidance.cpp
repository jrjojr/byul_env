#include "doctest.h"
#include "guidance.h"
#include "xform.h"
#include "entity_dynamic.h"
#include "environ.h"
#include "projectile.h"
#include <cmath>
#include <stdio.h>

static const vec3_t ZERO = {0, 0, 0};

TEST_CASE("guidance_none returns zero vector") {
    projectile_t proj{};
    vec3_t dir;

    const vec3_t* result 
    = guidance_none(&proj.base, 0.016f, nullptr, &dir);
    CHECK(result == &dir);

    vec3_t zero;
    vec3_init(&zero);
    CHECK(vec3_equal(&dir, &zero));
}

TEST_CASE("guidance_point points towards target") {
    projectile_t proj = {};
    vec3_t dir;

    xform_set_position(&proj.base.xf, &ZERO);

    vec3_t target = {10, 0, 0};

    const vec3_t* result 
    = guidance_point(&proj.base, 0.016f, &target, &dir);
    CHECK(result == &dir);

    vec3_t expected = {1, 0, 0};
    CHECK(vec3_equal(&dir, &expected));
}

TEST_CASE("guidance_lead predicts target") {
    projectile_t proj{};
    projectile_init(&proj);

    entity_dynamic_t target{};
    entity_dynamic_init(&target);

    vec3_t dir;

    xform_set_position(&proj.base.xf, &ZERO);
    proj.base.velocity = vec3_t{1, 0, 0};

    xform_set_position(&target.xf, &ZERO);
    vec3_t offset = {10, 0, 0};
    xform_translate(&target.xf, &offset);
    target.velocity = vec3_t{-0.5f, 0, 0};

    const vec3_t* result 
    = guidance_lead(&proj.base, 0.016f, &target, &dir);
    CHECK(result == &dir);
    float dir_len = vec3_length(&dir);
    CHECK(fabsf(dir_len - 1.0f) < 1e-5f);
}

static projectile_t create_test_projectile(
    const vec3_t& pos, const vec3_t& vel) {
    projectile_t proj{};
    projectile_init(&proj);
    xform_set_position(&proj.base.xf, &pos);
    proj.base.velocity = vel;
    return proj;
}

static entity_dynamic_t create_test_target(
    const vec3_t& pos, const vec3_t& vel) {
    entity_dynamic_t target{};
    entity_dynamic_init(&target);
    xform_set_position(&target.xf, &pos);
    target.velocity = vel;
    return target;
}

TEST_CASE("guidance_predict returns correct direction") {
    vec3_t dir;
    vec3_t proj_pos = {0, 0, 0};
    vec3_t proj_vel = {5, 0, 0};
    projectile_t proj = create_test_projectile(proj_pos, proj_vel);

    vec3_t target_pos = {10, 0, 0};
    vec3_t target_vel = {-1, 0, 0};
    entity_dynamic_t target = create_test_target(target_pos, target_vel);

    guidance_target_info_t info;
    info.target = target;
    info.current_time = 0.0f;

    const vec3_t* result 
    = guidance_predict(&proj.base, 0.016f, &info, &dir);
    CHECK(result == &dir);
    CHECK(fabsf(vec3_length(&dir) - 1.0f) < 1e-5f);
    CHECK(dir.x > 0.0f);
}

TEST_CASE("guidance_predict_accel handles acceleration") {
    vec3_t dir = {};
    vec3_t proj_pos = {0, 0, 0};
    vec3_t proj_vel = {5, 0, 0};
    projectile_t proj = create_test_projectile(proj_pos, proj_vel);

    vec3_t target_pos = {10, 0, 0};
    vec3_t target_vel = {-1, 0, 0};
    entity_dynamic_t target = create_test_target(target_pos, target_vel);

    guidance_target_info_t info = {};
    info.target = target;
    info.current_time = 0.0f;

    const vec3_t* result 
    = guidance_predict_accel(&proj.base, 0.016f, &info, &dir);
    CHECK(result == &dir);
    char buf[64];
    vec3_print(&dir);
    printf("guidance_predict_accel :%s\n", vec3_to_string(&dir, 64, buf));
    CHECK(fabsf(vec3_length(&dir) - 1.0f) <= 1e-5f);
    CHECK(dir.x >= 0.0f);
}

TEST_CASE("guidance_predict_accel_env with gravity and wind") {
    vec3_t dir;
    vec3_t proj_pos = {0, 0, 0};
    vec3_t proj_vel = {10, 0, 0};
    projectile_t proj = create_test_projectile(proj_pos, proj_vel);

    vec3_t target_pos = {20, 0, 0};
    vec3_t target_vel = {-2, 0, 0};
    entity_dynamic_t target = create_test_target(target_pos, target_vel);

    environ_t env;
    environ_init(&env);
    env.gravity = {0, -9.8f, 0};
    env.wind = {0.5f, 0, 0};

    guidance_target_info_t info;
    info.target =target;
    info.env = env;
    info.current_time = 0.0f;

    const vec3_t* result
    = guidance_predict_accel_env(&proj.base, 0.016f, &info, &dir);
    CHECK(result == &dir);
    CHECK(fabsf(vec3_length(&dir) - 1.0f) < 1e-5f);

    CHECK(dir.x > 0.0f);
}
