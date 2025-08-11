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

    environ_t env = make_env();
    propulsion_t propulsion = {};
    propulsion_init(&propulsion);
    propulsion.target_thrust = 100.0f;

    projectile_tick_t prt;
    projectile_tick_init_full(&prt, &proj, 
        &target, &env, nullptr, &propulsion, nullptr, false);

    CHECK(prt.proj.base.props.mass == 5.0f);
    // CHECK(prt.state.linear.position.x == 10.0f);
    CHECK(prt.target.base.id == 42);
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

    environ_t env = make_env();
    propulsion_t propulsion = {};
    propulsion.target_thrust = 50.0f;

    projectile_tick_t src;
    projectile_tick_init_full(&src, &proj, 
        &target, &env, nullptr, &propulsion, nullptr, false);
    src.impact_time = 2.0f;

    projectile_tick_t dst;
    projectile_tick_assign(&dst, &src);

    CHECK(dst.proj.base.props.mass == doctest::Approx(2.5f));
    // CHECK(dst.state.linear.velocity.y == doctest::Approx(15.0f));
    CHECK(dst.target.base.id == 123);
    CHECK(dst.env != nullptr);
    CHECK(dst.env->gravity.z == doctest::Approx(-9.8f));
    CHECK(dst.propulsion != nullptr);
    CHECK(dst.propulsion->target_thrust == doctest::Approx(50.0f));
    CHECK(dst.impact_time == doctest::Approx(2.0f));

    projectile_tick_free(&src);
    projectile_tick_free(&dst);
}

TEST_CASE("projectile_tick integrates one step under gravity") {
    projectile_t proj = {};
    projectile_init(&proj);
    proj.base.props.mass = 1.0f;
    proj.base.xf.pos = {0.0f, 0.0f, 100.0f};
    proj.base.velocity = {10.0f, 0.0f, 0.0f};

    entity_dynamic_t target = {}; // 사용하지 않음

    // 환경: 중력만
    environ_t env = {};
    env.gravity = { 0.0f, 0.0f, -9.8f };

    // 발사체 틱 구성
    projectile_tick_t prt;
    projectile_tick_init_full(&prt, &proj, 
        &target, &env, nullptr, nullptr, nullptr, false);

    // 실행 전 상태 기록
    vec3_t before_pos = prt.proj.base.xf.pos;
    vec3_t before_vel = prt.proj.base.velocity;

    tick_t* tk = tick_create();
    
    projectile_tick_prepare(&prt, tk);
    // bool result = projectile_tick(&prt, dt);
    tick_update(tk, 1.0f);

    // entity_dynamic_from_motion_state(&prt.proj.base, &state);
    // CHECK(result == true);

    vec3_print(&prt.proj.base.xf.pos);
    CHECK(prt.proj.base.xf.pos.x > before_pos.x); // 우측으로 이동
    CHECK(prt.proj.base.xf.pos.z < before_pos.z); // 중력 영향으로 하강
    CHECK(prt.proj.base.velocity.z < before_vel.z); // z속도 감소 (아래 방향)

    projectile_tick_complete(&prt, tk);

    projectile_tick_free(&prt);
    tick_destroy(tk);
}

TEST_CASE("projectile_tick trajcetory free falling") {
    projectile_t proj = {};
    projectile_init(&proj);
    // 초기 높이를 100으로 설정
    proj.base.xf.pos = {0.0, 100.0f, 0.0f};

    // 기본 타겟은 {0, 0, 0}
    entity_dynamic_t target = {}; // 사용하지 않음
    entity_dynamic_init(&target);
    target.xf.pos = {10.0f, 0.0f, 0.0f};

    // 환경: 기본 생성자에 중력 존재
    environ_t env = {};
    environ_init(&env);

    // 발사체 틱 구성 추진기와 유도 장치 nullptr
    projectile_tick_t prt;
    projectile_tick_init_full(&prt, &proj, 
        &target, &env, nullptr, nullptr, nullptr, true);

    // 실행 전 상태 기록
    vec3_t before_pos = prt.proj.base.xf.pos;
    vec3_t before_vel = prt.proj.base.velocity;

    tick_t* tk = tick_create();
    
    projectile_tick_prepare(&prt, tk);
    // bool result = projectile_tick(&prt, dt);
    for(int i=0; i < 100; i++){
        tick_update(tk, 1.0f);
    }

    // entity_dynamic_from_motion_state(&prt.proj.base, &state);
    // CHECK(result == true);

    // vec3_print(&prt.proj.base.xf.pos);
    // CHECK(prt.proj.base.xf.pos.x > before_pos.x); // 우측으로 이동
    CHECK(prt.proj.base.xf.pos.y < before_pos.y); // 중력 영향으로 하강
    // CHECK(prt.proj.base.velocity.z < before_vel.z); // z속도 감소 (아래 방향)

    char buf[64];
    trajectory_print(prt.trajectory);
        printf("prt impact time : %f, impact pos : %s\n", 
        prt.impact_time,
        vec3_to_string(&prt.impact_pos, 64, buf));

    projectile_tick_complete(&prt, tk);

    projectile_tick_free(&prt);
    tick_destroy(tk);
}

TEST_CASE("projectile_tick trajcetory free falling on target") {
    projectile_t proj = {};
    projectile_init(&proj);
    // 초기 높이를 100으로 설정
    proj.base.xf.pos = {0.0, 100.0f, 0.0f};

    entity_dynamic_t target = {}; // 사용하지 않음
    entity_dynamic_init(&target);
    target.xf.pos = {0.0f, 10.0f, 0.0f};

    // 환경: 기본 생성자에 중력 존재
    environ_t env = {};
    environ_init(&env);

    // 통합 설정
    integrator_t intgr;

    // 발사체 틱 구성 추진기와 유도 장치 nullptr
    projectile_tick_t prt;
    projectile_tick_init_full(&prt, &proj, 
        &target, &env, nullptr, nullptr, nullptr, true);

    // 실행 전 상태 기록
    vec3_t before_pos = prt.proj.base.xf.pos;
    vec3_t before_vel = prt.proj.base.velocity;

    tick_t* tk = tick_create();
    
    projectile_tick_prepare(&prt, tk);
    // bool result = projectile_tick(&prt, dt);
    for(int i=0; i < 100; i++){
        tick_update(tk, 1.0f);
    }

    // entity_dynamic_from_motion_state(&prt.proj.base, &state);
    // CHECK(result == true);

    // vec3_print(&prt.proj.base.xf.pos);
    // CHECK(prt.proj.base.xf.pos.x > before_pos.x); // 우측으로 이동
    CHECK(prt.proj.base.xf.pos.y < before_pos.y); // 중력 영향으로 하강
    // CHECK(prt.proj.base.velocity.z < before_vel.z); // z속도 감소 (아래 방향)

    char buf[64];
    trajectory_print(prt.trajectory);
        printf("prt impact time : %f, impact pos : %s\n", 
        prt.impact_time,
        vec3_to_string(&prt.impact_pos, 64, buf));

    projectile_tick_complete(&prt, tk);

    projectile_tick_free(&prt);
    tick_destroy(tk);
}

TEST_CASE("projectile tick launch simulation") {
    projectile_t proj = {};
    projectile_init(&proj);
    // 초기 높이를 10으로 설정
    proj.base.xf.pos = {0.0f, 10.0f, 0.0f};
    // 45도 방향으로 발사
    proj.base.velocity = {100.0f, 100.0f, 0.0f};

    entity_dynamic_t target = {}; // 사용하지 않음
    entity_dynamic_init(&target);
    target.xf.pos = {1000.0f, 1000.0f, 0.0f};

    // 환경: 기본 생성자에 중력 존재
    environ_t env = {};
    environ_init(&env);

    // 통합 설정
    integrator_t intgr;

    // 발사체 틱 구성 추진기와 유도 장치 nullptr
    projectile_tick_t prt;
    projectile_tick_init_full(&prt, &proj, 
        &target, &env, nullptr, nullptr, nullptr, true);

    // 실행 전 상태 기록
    vec3_t before_pos = prt.proj.base.xf.pos;
    vec3_t before_vel = prt.proj.base.velocity;

    tick_t* tk = tick_create();
    
    projectile_tick_prepare(&prt, tk);
    // bool result = projectile_tick(&prt, dt);
    for(int i=0; i < 100; i++){
        tick_update(tk, 1.0f);
    }

    // entity_dynamic_from_motion_state(&prt.proj.base, &state);
    // CHECK(result == true);

    // vec3_print(&prt.proj.base.xf.pos);
    // CHECK(prt.proj.base.xf.pos.x > before_pos.x); // 우측으로 이동
    CHECK(prt.proj.base.xf.pos.y < before_pos.y); // 중력 영향으로 하강
    // CHECK(prt.proj.base.velocity.z < before_vel.z); // z속도 감소 (아래 방향)

    char buf[64];
    trajectory_print(prt.trajectory);
        printf("prt impact time : %f, impact pos : %s\n", 
        prt.impact_time,
        vec3_to_string(&prt.impact_pos, 64, buf));

    projectile_tick_complete(&prt, tk);

    projectile_tick_free(&prt);
    tick_destroy(tk);
}