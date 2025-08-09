#include "doctest.h"
#include "projectile_tick.h"
#include "guidance.h"
#include "propulsion.h"
#include "trajectory.h"
#include "xform.h"
#include "entity_dynamic.h"
#include <cmath>
#include <stdio.h>

// 테스트 도우미: 기본 환경, 바디 속성 생성
static environ_t make_env() {
    environ_t env;
    environ_init(&env);

    env.gravity = { 0.0f, 0.0f, -9.8f };

    return env;
}

static bodyprops_t make_body() {
    bodyprops_t b;
    bodyprops_init(&b);
    b.mass = 1.0f;
    b.drag_coef = 0.1f;
    return b;
}

TEST_CASE("projectile_tick_init sets safe defaults") {
    projectile_tick_t prt;
    projectile_tick_init(&prt);

    CHECK(prt.env == nullptr);
    CHECK(prt.propulsion == nullptr);
    CHECK(prt.trajectory == nullptr);
    // CHECK(prt.impact_pos == nullptr);
    CHECK(prt.elapsed == doctest::Approx(0.0f));
    CHECK(prt.impact_time == doctest::Approx(-1.0f));
}

TEST_CASE("projectile_tick_init_full deeply copies input values") {
    projectile_t proj = {};
    proj.base.props.mass = 5.0f;

    motion_state_t state = {};
    // motion_state_init(&state);
    // state.linear.position.x = 10.0f;

    entity_dynamic_t target = {};
    target.base.id = 42;

    integrator_t intgr = {};
    integrator_init_full(&intgr, INTEGRATOR_RK4, 0.016f, 
        &state, 
        nullptr, nullptr, nullptr);

    environ_t env = make_env();
    propulsion_t propulsion = {};
    propulsion_init(&propulsion);
    propulsion.target_thrust = 100.0f;

    projectile_tick_t prt;
    projectile_tick_init_full(&prt, &proj, 
        // &state, 
        &target, &intgr, &env, &propulsion, nullptr);

    CHECK(prt.proj.base.props.mass == 5.0f);
    // CHECK(prt.state.linear.position.x == 10.0f);
    CHECK(prt.target.base.id == 42);
    CHECK(prt.intgr->type == INTEGRATOR_RK4);
    CHECK(prt.env != nullptr);
    CHECK(prt.env->gravity.z == doctest::Approx(-9.8f));
    CHECK(prt.propulsion != nullptr);
    CHECK(prt.propulsion->target_thrust == doctest::Approx(100.0f));
    CHECK(prt.guidance_fn == nullptr);

    projectile_tick_free(&prt);
}

TEST_CASE("projectile_tick_assign performs deep copy") {
    projectile_t proj = {};
    proj.base.props.mass = 2.5f;

    motion_state_t state = {};
    motion_state_init(&state);
    state.linear.velocity.y = 15.0f;

    entity_dynamic_t target = {};
    target.base.id = 123;

    integrator_t intgr = {};
    integrator_init_full(&intgr, INTEGRATOR_SEMI_IMPLICIT, 0.033f, &state, nullptr, nullptr, nullptr);

    environ_t env = make_env();
    propulsion_t propulsion = {};
    propulsion.target_thrust = 50.0f;

    projectile_tick_t src;
    projectile_tick_init_full(&src, &proj, 
        // &state, 
        &target, &intgr, &env, &propulsion, nullptr);
    src.elapsed = 1.5f;
    src.max_time = 3.0f;
    src.impact_time = 2.0f;

    projectile_tick_t dst;
    projectile_tick_assign(&dst, &src);

    CHECK(dst.proj.base.props.mass == doctest::Approx(2.5f));
    // CHECK(dst.state.linear.velocity.y == doctest::Approx(15.0f));
    CHECK(dst.target.base.id == 123);
    CHECK(dst.intgr->type == INTEGRATOR_SEMI_IMPLICIT);
    CHECK(dst.env != nullptr);
    CHECK(dst.env->gravity.z == doctest::Approx(-9.8f));
    CHECK(dst.propulsion != nullptr);
    CHECK(dst.propulsion->target_thrust == doctest::Approx(50.0f));
    CHECK(dst.elapsed == doctest::Approx(1.5f));
    CHECK(dst.max_time == doctest::Approx(3.0f));
    CHECK(dst.impact_time == doctest::Approx(2.0f));

    projectile_tick_free(&src);
    projectile_tick_free(&dst);
}

TEST_CASE("projectile_tick integrates one step under gravity") {
    projectile_t proj = {};
    proj.base.props.mass = 1.0f;
    proj.base.xf.pos = {0.0f, 0.0f, 100.0f};
    proj.base.velocity = {10.0f, 0.0f, 0.0f};

    entity_dynamic_t target = {}; // 사용하지 않음

    // 환경: 중력만
    environ_t env = {};
    env.gravity = { 0.0f, 0.0f, -9.8f };

    // 통합 설정
    integrator_t intgr;
    motion_state_t state = {};
    entity_dynamic_to_motion_state(&proj.base, &state, nullptr, nullptr);
    integrator_init_full(&intgr, INTEGRATOR_RK4_ENV, 1.0f, &state, nullptr, &env, nullptr);

    // 발사체 틱 구성
    projectile_tick_t prt;
    projectile_tick_init_full(&prt, &proj, 
        // &state, 
        &target, &intgr, &env, nullptr, nullptr);

    // 실행 전 상태 기록
    vec3_t before_pos = prt.proj.base.xf.pos;
    vec3_t before_vel = prt.proj.base.velocity;

    tick_t* tk = tick_create();
    
    projectile_tick_prepare(&prt, tk);
    // bool result = projectile_tick(&prt, dt);
    tick_update(tk, 1.0f);

    // entity_dynamic_from_motion_state(&prt.proj.base, &state);
    // CHECK(result == true);
    CHECK(prt.elapsed == doctest::Approx(1.0f));
    vec3_print(&prt.proj.base.xf.pos);
    CHECK(prt.proj.base.xf.pos.x > before_pos.x); // 우측으로 이동
    CHECK(prt.proj.base.xf.pos.z < before_pos.z); // 중력 영향으로 하강
    CHECK(prt.proj.base.velocity.z < before_vel.z); // z속도 감소 (아래 방향)

    projectile_tick_complete(&prt, tk);

    projectile_tick_free(&prt);
    integrator_free(&intgr);
    tick_destroy(tk);
}
