#include "doctest.h"
#include <iostream>
#include "math.h"

extern "C" {
#include "projectile.h"
#include "entity_dynamic.h"
#include "vec3.h"
#include "xform.h"
}

TEST_CASE("projectile_init") {
    projectile_t proj;
    projectile_init(&proj);

    CHECK(proj.base.base.id == -1);
    vec3_t zero = {0,0,0};
    CHECK(vec3_equal(&proj.base.velocity, &zero));
    CHECK(vec3_equal(&proj.base.angular_velocity, &zero));
}

TEST_CASE("projectile_init_full") {
    projectile_t proj;
    entity_dynamic_t base_dyn;
    entity_dynamic_init(&base_dyn);
    base_dyn.base.id = 77;
    base_dyn.velocity = {1.0f, 2.0f, 3.0f};

    auto on_hit_test = [](const void* p, void* ud) {
        (void)p;
        int* val = static_cast<int*>(ud);
        *val = 123;
    };

    int userdata_val = 0;
    projectile_attr_t attrs ={};
    
    projectile_init_full(&proj, &base_dyn, attrs, 1.0f 
        , on_hit_test, &userdata_val);

    CHECK(proj.on_hit != nullptr);
    CHECK(proj.hit_userdata == &userdata_val);
    CHECK(proj.base.base.id == 77);
    CHECK(vec3_equal(&proj.base.velocity, &base_dyn.velocity));
}

TEST_CASE("projectile_assign") {
    projectile_t src;
    projectile_init(&src);
    src.base.velocity = {2.0f, 3.0f, 4.0f};

    projectile_t dst;
    projectile_assign(&dst, &src);

    CHECK(vec3_equal(&dst.base.velocity, &src.base.velocity));
}

TEST_CASE("projectile_update") {
    projectile_t proj;
    projectile_init(&proj);

    proj.base.velocity = {1.0f, 0.0f, 0.0f};
    proj.base.angular_velocity = {0.0f, 0.0f, 3.1415f};
    proj.base.base.lifetime = 0.5f;

    int userdata_val = 0;
    proj.on_hit = [](const void* p, void* ud) {
        (void)p;
        int* val = static_cast<int*>(ud);
        *val = 999;
    };
    proj.hit_userdata = &userdata_val;

    // 0.3 sec after
    projectile_update(&proj, 0.3f);
    CHECK(proj.base.base.age == doctest::Approx(0.3f));
    CHECK(userdata_val == 0);

    // 0.3 sec add (total 0.6 sec > 0.5 sec life time)
    projectile_update(&proj, 0.3f);
    CHECK(proj.base.base.age == doctest::Approx(0.6f));
    CHECK(userdata_val == 999);
}

TEST_CASE("projectile_default_hit_cb") {
    projectile_t proj;
    projectile_init(&proj);

    projectile_default_hit_cb(&proj, nullptr);
    CHECK(true);
}

TEST_CASE("projectile_update on_hit") {
    projectile_t proj;
    projectile_init(&proj);

    proj.base.base.lifetime = 1.0f;

    projectile_update(&proj, 0.5f);

    // after life time (1.2 sec after)
    projectile_update(&proj, 0.7f);
}

TEST_CASE("projectile_calc_launch_param") {
    projectile_t proj;
    projectile_init(&proj);

    vec3_t start = {0, 0, 0};
    xform_set_position(&proj.base.xf, &start);
    proj.base.props.mass = 1.0f;

    vec3_t target = {10, 0, 0};

    launch_param_t result;
    bool success = projectile_calc_launch_param(
        &result, &proj, &target, 100.0f); // 100 N
    CHECK(success == true);
    CHECK(result.time_to_hit > 0.0f);
    CHECK(fabsf(result.direction.x) > 0.1f);
}

TEST_CASE("projectile_calc_launch_param_env") {
    projectile_t proj;
    projectile_init(&proj);

    vec3_t start = {0, 0, 0};
    xform_set_position(&proj.base.xf, &start);
    proj.base.props.mass = 1.0f;

    environ_t env = {};
    environ_init(&env);

    vec3_t target = {10, 0, 0};

    launch_param_t result;
    bool success = projectile_calc_launch_param_env(
        &result, &proj, &env, &target, 200.0f);
    CHECK(success == true);
    CHECK(result.time_to_hit > 0.0f);
    CHECK(fabsf(result.direction.x) > 0.1f);
}

TEST_CASE("entity_dynamic_calc_position") {
    entity_dynamic_t ed;
    entity_dynamic_init(&ed);

    vec3_t start = {0, 0, 0};
    xform_set_position(&ed.xf, &start);
    ed.velocity = {1, 2, 3};
    ed.props.friction = 0.0f;

    vec3_t pos;
    entity_dynamic_calc_position(&ed, 2.0f, &pos); // 2 sec after

    vec3_t expected = {2, 4, 6}; // p = v * dt
    CHECK(vec3_equal_tol(&pos, &expected, 1e-4f));
}


TEST_CASE("entity_dynamic_calc_position_env") {
    entity_dynamic_t ed;
    entity_dynamic_init(&ed);
    ed.props.drag_coef = 0.0f;
    ed.props.friction = 0.0f;

    vec3_t start = {0, 0, 0};
    xform_set_position(&ed.xf, &start);
    ed.velocity = {0, 10, 0};

    environ_t env;
    environ_init(&env);

    vec3_t pos;

    entity_dynamic_calc_position_env(&ed, &env, 1.0f, &pos);

    vec3_t expected = {0, 5.1f, 0};

    vec3_print(&pos);
    vec3_print(&expected);
    CHECK(vec3_equal_tol(&pos, &expected, 1.0f));
}
