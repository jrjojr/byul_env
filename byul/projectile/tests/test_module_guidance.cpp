#include "doctest.h"
#include "internal/guidance.h"
#include "internal/xform.h"
#include "internal/entity_dynamic.h"
#include "internal/environ.h"
#include "internal/projectile.h"
#include <cmath>
#include <stdio.h>

// 공통 0 벡터
static const vec3_t ZERO = {0, 0, 0};

// ---------------------------------------------------------
// guidance_none 테스트
// ---------------------------------------------------------
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

// ---------------------------------------------------------
// guidance_point 테스트
// ---------------------------------------------------------
TEST_CASE("guidance_point points towards target") {
    projectile_t proj = {};
    vec3_t dir;

    // 발사체 위치 (0, 0, 0)
    xform_set_position(&proj.base.xf, &ZERO);

    // 목표 위치 (10, 0, 0)
    vec3_t target = {10, 0, 0};

    const vec3_t* result 
    = guidance_point(&proj.base, 0.016f, &target, &dir);
    CHECK(result == &dir);

    // 예상 방향 (1, 0, 0)
    vec3_t expected = {1, 0, 0};
    CHECK(vec3_equal(&dir, &expected));
}

// ---------------------------------------------------------
// guidance_lead 테스트
// ---------------------------------------------------------
TEST_CASE("guidance_lead predicts target") {
    projectile_t proj{};
    projectile_init(&proj);

    entity_dynamic_t target{};
    entity_dynamic_init(&target);

    vec3_t dir;

    // 발사체 초기화
    xform_set_position(&proj.base.xf, &ZERO);
    proj.base.velocity = (vec3_t){1, 0, 0};

    // 타겟 초기화
    xform_set_position(&target.xf, &ZERO);
    vec3_t offset = {10, 0, 0};
    xform_translate(&target.xf, &offset);
    target.velocity = (vec3_t){-0.5f, 0, 0}; // 속도 조정

    const vec3_t* result 
    = guidance_lead(&proj.base, 0.016f, &target, &dir);
    CHECK(result == &dir);
    float dir_len = vec3_length(&dir);
    CHECK(fabsf(dir_len - 1.0f) < 1e-5f); // 단위 벡터 확인
}

// ---------------------------------------------------------
// 헬퍼 함수
// ---------------------------------------------------------
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

// ---------------------------------------------------------
// guidance_predict 테스트
// ---------------------------------------------------------
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
    CHECK(dir.x > 0.0f); // x 방향으로 향해야 함
}

// ---------------------------------------------------------
// guidance_predict_accel 테스트
// ---------------------------------------------------------
TEST_CASE("guidance_predict_accel handles acceleration") {
    vec3_t dir;
    vec3_t proj_pos = {0, 0, 0};
    vec3_t proj_vel = {5, 0, 0};
    projectile_t proj = create_test_projectile(proj_pos, proj_vel);

    vec3_t target_pos = {10, 0, 0};
    vec3_t target_vel = {-1, 0, 0};
    entity_dynamic_t target = create_test_target(target_pos, target_vel);

    // 타겟에 약간의 가속도 추가
    guidance_target_info_t info;
    info.target = target;
    info.current_time = 0.0f;

    const vec3_t* result 
    = guidance_predict_accel(&proj.base, 0.016f, &info, &dir);
    CHECK(result == &dir);
    char buf[64];
    printf("guidance_predict_accel :%s\n", vec3_to_string(&dir, buf, 64));
    CHECK(fabsf(vec3_length(&dir) - 1.0f) <= 1e-5f);
    CHECK(dir.x >= 0.0f);
}

// ---------------------------------------------------------
// guidance_predict_accel_env 테스트
// ---------------------------------------------------------
TEST_CASE("guidance_predict_accel_env with gravity and wind") {
    vec3_t dir;
    vec3_t proj_pos = {0, 0, 0};
    vec3_t proj_vel = {10, 0, 0};
    projectile_t proj = create_test_projectile(proj_pos, proj_vel);

    vec3_t target_pos = {20, 0, 0};
    vec3_t target_vel = {-2, 0, 0};
    entity_dynamic_t target = create_test_target(target_pos, target_vel);

    // 환경 설정
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
    // x 방향으로 향해야 하며, 약간의 y 변화가 있을 수 있음
    CHECK(dir.x > 0.0f);
}
