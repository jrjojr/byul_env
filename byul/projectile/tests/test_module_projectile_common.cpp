#include "doctest.h"
#include <iostream>
#include "math.h"

extern "C" {
#include "projectile.h"
#include "entity_dynamic.h"
#include "vec3.h"
#include "xform.h"
}

TEST_CASE("projectile_init 기본값 테스트") {
    projectile_t proj;
    projectile_init(&proj);

    // base 확인
    CHECK(proj.base.base.id == -1);
    vec3_t zero = {0,0,0};
    CHECK(vec3_equal(&proj.base.velocity, &zero));
    CHECK(vec3_equal(&proj.base.angular_velocity, &zero));
}

TEST_CASE("projectile_init_full 사용자 지정 초기화") {
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

TEST_CASE("projectile_assign 복사") {
    projectile_t src;
    projectile_init(&src);
    src.base.velocity = {2.0f, 3.0f, 4.0f};

    projectile_t dst;
    projectile_assign(&dst, &src);

    CHECK(vec3_equal(&dst.base.velocity, &src.base.velocity));
}

TEST_CASE("projectile_update 동작 테스트") {
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

    // 0.3초 경과
    projectile_update(&proj, 0.3f);
    CHECK(proj.base.base.age == doctest::Approx(0.3f));
    CHECK(userdata_val == 0);

    // 0.3초 추가 (총 0.6초 > 0.5초 수명)
    projectile_update(&proj, 0.3f);
    CHECK(proj.base.base.age == doctest::Approx(0.6f));
    CHECK(userdata_val == 999);
}

TEST_CASE("projectile_default_hit_cb 동작 테스트") {
    projectile_t proj;
    projectile_init(&proj);

    // stdout 캡처 없이 단순 호출만 체크
    projectile_default_hit_cb(&proj, nullptr);
    CHECK(true); // 단순 호출 성공 여부만 확인
}

TEST_CASE("projectile_update에서 on_hit 호출 테스트") {
    projectile_t proj;
    projectile_init(&proj);

    proj.base.base.lifetime = 1.0f;  // 1초 후 만료

    // 아직 수명 전
    projectile_update(&proj, 0.5f);

    // 수명 경과 (1.2초 후)
    projectile_update(&proj, 0.7f);
}

TEST_CASE("projectile_calc_launch_param - 기본 포물선 발사") {
    projectile_t proj;
    projectile_init(&proj);

    // 시작 위치 (0,0,0)
    vec3_t start = {0, 0, 0};
    xform_set_position(&proj.base.xf, &start);
    proj.base.props.mass = 1.0f;

    // 목표 위치 (10, 0, 0)
    vec3_t target = {10, 0, 0};

    launch_param_t result;
    bool success = projectile_calc_launch_param(&result, &proj, &target, 100.0f); // 100 N
    CHECK(success == true);
    CHECK(result.time_to_hit > 0.0f);
    CHECK(fabsf(result.direction.x) > 0.1f); // 속도 벡터의 x성분이 있어야 함
}

TEST_CASE("projectile_calc_launch_param_env - 환경 고려") {
    projectile_t proj;
    projectile_init(&proj);

    vec3_t start = {0, 0, 0};
    xform_set_position(&proj.base.xf, &start);
    proj.base.props.mass = 1.0f;

    environ_t env = {
        {0, -9.8f, 0},  // 중력
        {1.0f, 0, 0},   // 바람 (x방향)
        1.225f,         // 공기밀도
        0,              // 습도
        20,             // 온도
        101325          // 기압
    };

    vec3_t target = {10, 0, 0};

    launch_param_t result;
    bool success = projectile_calc_launch_param_env(&result, &proj, &env, &target, 200.0f);
    CHECK(success == true);
    CHECK(result.time_to_hit > 0.0f);
    CHECK(fabsf(result.direction.x) > 0.1f);
}

TEST_CASE("entity_dynamic_calc_position - 단순 위치 예측") {
    entity_dynamic_t ed;
    entity_dynamic_init(&ed);

    vec3_t start = {0, 0, 0};
    xform_set_position(&ed.xf, &start);
    ed.velocity = {1, 2, 3};
    ed.props.friction = 0.0f;  // 마찰 제거

    vec3_t pos;
    entity_dynamic_calc_position(&ed, 2.0f, &pos); // 2초 후

    vec3_t expected = {2, 4, 6}; // p = v * dt
    CHECK(vec3_equal_tol(&pos, &expected, 1e-4f));
}


TEST_CASE("entity_dynamic_calc_position_env - 환경 포함 예측") {
    entity_dynamic_t ed;
    entity_dynamic_init(&ed);
    ed.props.drag_coef = 0.0f;    // 드래그 제거
    ed.props.friction = 0.0f;     // 마찰 제거

    vec3_t start = {0, 0, 0};
    xform_set_position(&ed.xf, &start);
    ed.velocity = {0, 10, 0};     // 위로 발사

    // 순수 중력만 적용되는 환경
    environ_t env;
    environ_init(&env);

    vec3_t pos;
    //나의 현재 위치는 (0,0,0) , 속도는 (0,10,0) 1초후에 가속도로 속도가 줄어든다
    entity_dynamic_calc_position_env(&ed, &env, 1.0f, &pos); // 1초 후

    vec3_t expected = {0, 5.1f, 0};

    vec3_print(&pos);
    vec3_print(&expected);
    CHECK(vec3_equal_tol(&pos, &expected, 1.0f));
}
