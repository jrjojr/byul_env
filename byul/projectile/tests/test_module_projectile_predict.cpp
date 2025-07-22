#include "doctest.h"
#include "internal/projectile_predict.h"
#include "internal/projectile_guidance.h"
#include "internal/projectile_propulsion.h"
#include "internal/trajectory.h"
#include "internal/xform.h"
#include "internal/entity_dynamic.h"
#include <cmath>

// -------------------------------------------
// 헬퍼 함수 (통일 버전)
// -------------------------------------------
static projectile_t create_test_projectile(const vec3_t& pos, const vec3_t& vel, float mass = 1.0f) {
    projectile_t proj{};
    projectile_init(&proj);
    xform_set_position(&proj.base.xf, &pos);
    proj.base.velocity = vel;
    proj.base.props.mass = mass;
    return proj;
}

static entity_dynamic_t create_test_target(const vec3_t& pos, const vec3_t& vel = {0, 0, 0}) {
    entity_dynamic_t target{};
    entity_dynamic_init(&target);
    xform_set_position(&target.xf, &pos);
    target.velocity = vel;
    return target;
}

// -------------------------------------------
// 기본 환경 함수 (중력만 적용)
// -------------------------------------------
static const vec3_t* simple_gravity(vec3_t* out_accel, float, void*) {
    if (out_accel) {
        *out_accel = {0, -9.8f, 0};
        return out_accel;
    }
    static vec3_t g = {0, -9.8f, 0};
    return &g;
}

// -------------------------------------------
// projectile_predict 테스트
// -------------------------------------------
// TEST_CASE("projectile_predict - ground collision") {
//     vec3_t start_pos = {0, 10, 0};
//     vec3_t start_vel = {2, 0, 0};
//     projectile_t proj = create_test_projectile(start_pos, start_vel);

//     projectile_result_t* result = projectile_result_create();

//     bool hit = projectile_predict(result, &proj,
//                                   nullptr,                // 추진력 없음
//                                   projectile_guidance_none,
//                                   nullptr,
//                                   nullptr,                // 타겟 없음
//                                   5.0f,                   // max_time
//                                   0.1f,                   // time_step
//                                   simple_gravity,         // 중력 환경
//                                   nullptr);

//     CHECK(hit == true);
//     CHECK(result->valid == true);
//     CHECK(result->impact_time > 0.0f);
//     CHECK(result->impact_pos.y == doctest::Approx(0.0f).epsilon(0.01f));

//     // trajectory_print(result->trajectory);
//     projectile_result_destroy(result);
// }

// TEST_CASE("projectile_predict - static target hit") {
//     vec3_t start_pos = {0, 0, 0};
//     vec3_t start_vel = {10, 0, 0};
//     projectile_t proj = create_test_projectile(start_pos, start_vel);

//     entity_dynamic_t target = create_test_target({15, 0, 0});
//     target_info_t target_info{&target, nullptr, 0.0f};

//     projectile_result_t* result = projectile_result_create();

//     bool hit = projectile_predict(result, &proj,
//                                   nullptr,                     // 추진력 없음
//                                   projectile_guidance_point,   // 타겟 조준
//                                   nullptr,
//                                   &target_info,
//                                   5.0f,
//                                   0.1f,
//                                   nullptr,                     // 환경 없음
//                                   nullptr);

//     CHECK(hit == true);
//     CHECK(result->valid == true);
//     CHECK(result->impact_time > 0.0f);
//     CHECK(result->impact_pos.x >= 14.0f);

//     // trajectory_print(result->trajectory);
//     projectile_result_destroy(result);
// }

// TEST_CASE("projectile_predict - moving target with lead guidance") {
//     vec3_t start_pos = {0, 0, 0};
//     vec3_t start_vel = {12, 0, 0};
//     projectile_t proj = create_test_projectile(start_pos, start_vel);

//     // 이동 타겟 (왼쪽 -2m/s)
//     entity_dynamic_t target = create_test_target({15, 0, 0}, {-2, 0, 0});
//     target_info_t target_info{&target, nullptr, 0.0f};

//     projectile_result_t* result = projectile_result_create();

//     bool hit = projectile_predict(result, &proj,
//                                   nullptr,                   // 추진력 없음
//                                   projectile_guidance_lead,  // 리드샷 유도
//                                   nullptr,
//                                   &target_info,
//                                   5.0f,
//                                   0.1f,
//                                   nullptr,
//                                   nullptr);

//     CHECK(hit == true);
//     CHECK(result->valid == true);
//     CHECK(result->impact_time > 0.0f);
//     CHECK(result->impact_pos.x > 10.0f);  // 타겟이 이동했기 때문에
//     trajectory_print(result->trajectory);

//     projectile_result_destroy(result);
// }

// TEST_CASE("projectile_predict - moving target with lead guidance with propulsion") {
//     vec3_t start_pos = {0, 0, 0};
//     vec3_t start_vel = {12, 0, 0};
//     projectile_t proj = create_test_projectile(start_pos, start_vel);

//     // 이동 타겟 (왼쪽 -2m/s)
//     entity_dynamic_t target = create_test_target({15, 0, 0}, {-2, 0, 0});
//     target_info_t target_info{&target, nullptr, 0.0f};

//     projectile_result_t* result = projectile_result_create();

//     propulsion_t propulsion;
//     propulsion_init(&propulsion);
//     bool hit = projectile_predict(result, &proj,
//                                   &propulsion,                   // 추진력 있음
//                                   projectile_guidance_lead,  // 리드샷 유도
//                                   nullptr,
//                                   &target_info,
//                                   5.0f,
//                                   0.1f,
//                                   nullptr,
//                                   nullptr);

//     CHECK(hit == true);
//     CHECK(result->valid == true);
//     CHECK(result->impact_time > 0.0f);
//     CHECK(result->impact_pos.x > 10.0f);  // 타겟이 이동했기 때문에
//     trajectory_print(result->trajectory);

//     projectile_result_destroy(result);
// }

// -------------------------------------------
// 테스트: 추진력 비교
// -------------------------------------------
TEST_CASE("projectile_predict - moving target lead vs propulsion") {
    vec3_t start_pos = {0, 0, 0};
    vec3_t start_vel = {6, 0, 0};  // 비교를 위해 낮은 초기속도
    projectile_t proj = create_test_projectile(start_pos, start_vel);

    // 이동 타겟 (-2 m/s)
    entity_dynamic_t target = create_test_target({20, 0, 0}, {-2, 0, 0});
    target_info_t target_info{&target, nullptr, 0.0f};

    // --- 1. 추진력 없음 ---
    projectile_result_t* result_no_prop = projectile_result_create();
    bool hit_no_prop = projectile_predict(result_no_prop, &proj,
                                          nullptr,                  // 추진력 없음
                                          projectile_guidance_lead, // 리드샷
                                          nullptr,
                                          &target_info,
                                          5.0f,
                                          0.1f,
                                          nullptr,
                                          nullptr);

    CHECK(hit_no_prop == true);
    CHECK(result_no_prop->valid == true);
    float impact_time_no_prop = result_no_prop->impact_time;
    float impact_x_no_prop = result_no_prop->impact_pos.x;
    trajectory_print(result_no_prop->trajectory);
    projectile_result_destroy(result_no_prop);

    // --- 2. 추진력 있음 ---
    projectile_result_t* result_prop = projectile_result_create();
    propulsion_t propulsion;
    propulsion_init(&propulsion);
    propulsion.current_thrust = 0.1f;  // 명확히 효과가 보이도록 큰 추진력
    propulsion.max_thrust = 30.0f;
    propulsion.fuel_remaining = 10.0f;  // 충분한 연료
    propulsion.burn_rate = 0.1f;
    propulsion.active = true;

    bool hit_prop = projectile_predict(result_prop, &proj,
                                       &propulsion,                // 추진력 적용
                                       projectile_guidance_lead,
                                       nullptr,
                                       &target_info,
                                       5.0f,
                                       0.1f,
                                       nullptr,
                                       nullptr);

    CHECK(hit_prop == true);
    CHECK(result_prop->valid == true);
    float impact_time_prop = result_prop->impact_time;
    float impact_x_prop = result_prop->impact_pos.x;
    trajectory_print(result_prop->trajectory);
    projectile_result_destroy(result_prop);

    // --- 비교 검증 ---
    // CHECK(impact_time_prop <= impact_time_no_prop);  // 추진력이 있으면 빠르게 명중
    CHECK(impact_x_prop >= impact_x_no_prop);        // 추진력이 있으면 더 멀리 간다
}