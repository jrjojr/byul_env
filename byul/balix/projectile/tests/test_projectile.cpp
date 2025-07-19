#define DOCTEST_CONFIG_IMPLEMENT
#include "doctest.h"
#include <locale.h>
#include <iostream>

extern "C" {
#include "internal/xform.h"
#include "internal/projectile.h"

#include "internal/common.h"
}

static float sample_speed(const trajectory_sample_t& sample);

int main(int argc, char** argv) {
#ifdef _WIN32
    UINT original_cp = GetConsoleOutputCP();
    SetConsoleOutputCP(65001);                          // UTF-8 출력용
    setlocale(LC_ALL, "ko_KR.UTF-8");                   // UTF-8 로케일
#else
    setlocale(LC_ALL, "ko_KR.UTF-8");                   // 리눅스/맥에서도 설정
#endif

    std::cout << u8"🌟 UTF-8 콘솔 코드페이지로 전환하고 테스트 시작!\n";

    doctest::Context context;
    context.applyCommandLine(argc, argv);

    // context.setOption("success", true);      // 성공한 테스트도 출력
    // context.setOption("durations", true);    // 각 테스트 케이스 시간 출력

    int res = context.run();

    if (context.shouldExit()) {
        std::cout << u8"🌙 테스트 끝! 콘솔 코드페이지 원래대로 복구했습니다.\n";
#ifdef _WIN32
        SetConsoleOutputCP(original_cp);                // 원래 코드페이지 복원
        setlocale(LC_ALL, "");                          // 기본 로케일로 복귀
#endif
        return res;
    }

    std::cout << u8"🌙 테스트 종료. 콘솔 상태 복원 완료.\n";
#ifdef _WIN32
    SetConsoleOutputCP(original_cp);
    setlocale(LC_ALL, "");                              // 로케일 복원
#endif

    return res;
}

// --------------------------------------------
// TEST: projectile_init
// --------------------------------------------
TEST_CASE("projectile_init sets default values") {
    projectile_t proj;
    projectile_init(&proj);

    vec3_t zero = {0, 0, 0};

    // 초기화 값 확인
    vec3_t pos;
    xform_get_position(&proj.xf, &pos);
    CHECK(vec3_equal(&pos, &zero));
    CHECK(vec3_equal(&proj.velocity, &zero));
    CHECK(vec3_equal(&proj.acceleration, &zero));
    CHECK(vec3_equal(&proj.angular_velocity, &zero));

    CHECK(proj.age == doctest::Approx(0.0f));
    CHECK(proj.lifetime == doctest::Approx(10.0f));
    CHECK(proj.type == PROJECTILE_TYPE_SHELL);
    CHECK(proj.projectile_id == -1);
    CHECK(proj.on_hit != nullptr);
}

// --------------------------------------------
// TEST: projectile_init_full
// --------------------------------------------
TEST_CASE("projectile_init_full sets custom type and lifetime") {
    projectile_t proj;
    projectile_init_full(&proj, PROJECTILE_TYPE_MISSILE, 20.0f);

    CHECK(proj.type == PROJECTILE_TYPE_MISSILE);
    CHECK(proj.lifetime == doctest::Approx(20.0f));
}

// --------------------------------------------
// TEST: projectile_copy
// --------------------------------------------
TEST_CASE("projectile_copy copies all fields") {
    projectile_t src, dst;
    projectile_init_full(&src, PROJECTILE_TYPE_MISSILE, 15.0f);
    src.velocity = {1.0f, 2.0f, 3.0f};
    src.projectile_id = 42;

    projectile_copy(&dst, &src);

    CHECK(dst.type == PROJECTILE_TYPE_MISSILE);
    CHECK(dst.lifetime == doctest::Approx(15.0f));
    CHECK(vec3_equal(&dst.velocity, &src.velocity));
    CHECK(dst.projectile_id == 42);
}

// --------------------------------------------
// TEST: shell_init
// --------------------------------------------
TEST_CASE("shell_init sets default values") {
    shell_t shell;
    shell_init(&shell);

    CHECK(shell.base.type == PROJECTILE_TYPE_SHELL);
    CHECK(shell.drag_coef == doctest::Approx(0.0f));
    CHECK(shell.env_fn == projectile_env_default);
}

// --------------------------------------------
// TEST: missile_init
// --------------------------------------------
TEST_CASE("missile_init sets default values") {
    missile_t missile;
    missile_init(&missile);

    CHECK(missile.base.type == PROJECTILE_TYPE_MISSILE);
    vec3_t zero;
    vec3_zero(&zero);
    CHECK(vec3_equal(&missile.thrust, &zero));
    CHECK(missile.fuel == doctest::Approx(0.0f));
    CHECK(missile.guidance_fn == projectile_guidance_none);
    CHECK(missile.env_fn == projectile_env_default);
}

// --------------------------------------------
// TEST: projectile_predict simple free fall
// --------------------------------------------
TEST_CASE("projectile_predict simple free fall") {
    projectile_predictor_t predictor;
    projectile_predictor_init(&predictor);

    predictor.start_pos = {0, 10, 0};
    predictor.start_velocity = {0, 0, 0};
    predictor.env_fn = projectile_env_default;
    predictor.ground_height = 0.0f;
    predictor.max_time = 5.0f;
    predictor.time_step = 0.1f;

    projectile_result_t* result;
    result = projectile_result_create();
    bool success = projectile_predict(&predictor, result);

    CHECK(success == true);
    CHECK(result->valid == true);
    CHECK(result->impact_pos.y == doctest::Approx(0.0f).epsilon(0.01f));
    CHECK(result->impact_time > 0.0f);
    CHECK(result->impact_time < 5.0f);

                trajectory_print(result->trajectory);  
    projectile_result_destroy(result);
}

// --------------------------------------------
// TEST: projectile_predict angled shot (shell)
// --------------------------------------------
TEST_CASE("projectile_predict: angled shot shell") {
    // predictor 초기화
    projectile_predictor_t predictor;
    projectile_predictor_init(&predictor);

    predictor.start_pos = {0, 0.01f, 0};    // 약간 위에서 시작
    predictor.start_velocity = {5.0f, 5.0f, 0}; // X=5m/s, Y=5m/s (사선 발사)
    predictor.env_fn = projectile_env_default;  // 중력 적용
    predictor.ground_height = 0.0f;
    predictor.max_time = 5.0f;
    predictor.time_step = 0.05f;

    // 결과 객체 생성
    projectile_result_t* result = projectile_result_create();
    bool success = projectile_predict(&predictor, result);

    CHECK(success == true);
    CHECK(result->valid == true);

    // 충돌 위치 검증: Y축은 반드시 ground_height에 도달해야 한다.
    CHECK(result->impact_pos.y == doctest::Approx(0.0f).epsilon(0.01f));

    // X축은 반드시 양수 (앞쪽으로 이동)
    CHECK(result->impact_pos.x > 0.0f);

    // Y축 궤적에서 최대 높이는 약 1m 이상이어야 함
    bool max_height_reached = false;
    for (int i = 0; i < result->trajectory->count; ++i) {
        if (result->trajectory->samples[i].state.linear.position.y > 1.0f) {
            max_height_reached = true;
            break;
        }
    }
    CHECK(max_height_reached == true);

    // trajectory 출력 (디버깅용)
    trajectory_print(result->trajectory);
    projectile_result_destroy(result);
}

// ---------------------------------------------------------
// TEST: projectile_predict_missile – 자유 낙하 (thrust=0)
// ---------------------------------------------------------
TEST_CASE("projectile_predict_missile: free fall") {
    missile_predictor_t predictor;
    missile_predictor_init(&predictor);

    predictor.start_pos = {0, 10, 0};
    predictor.start_velocity = {0, 0, 0};
    predictor.thrust = {0, 0, 0};  // 추력 없음
    predictor.fuel = 0.0f;         // 연료 없음
    predictor.env_fn = projectile_env_default;  // 중력만 적용
    predictor.ground_height = 0.0f;
    predictor.max_time = 5.0f;
    predictor.time_step = 0.1f;
    projectile_result_t* result;
    result = projectile_result_create();
    bool success = projectile_predict_missile(&predictor, result);

    CHECK(success == true);
    CHECK(result->valid == true);
    CHECK(result->impact_pos.y == doctest::Approx(0.0f).epsilon(0.01f));
    CHECK(result->impact_time > 0.0f);
    CHECK(result->impact_time < 5.0f);

            trajectory_print(result->trajectory);  
    projectile_result_destroy(result);
}


// ---------------------------------------------------------
// TEST: projectile_predict_missile – 상향 thrust
// ---------------------------------------------------------
TEST_CASE("projectile_predict_missile: upward thrust") {
    missile_predictor_t predictor;
    missile_predictor_init(&predictor);

    predictor.start_pos = {0, 0.01, 0};
    predictor.start_velocity = {0, 0, 0};
    predictor.thrust = {0, 11, 0};  // Y축 위로 큰 추력
    predictor.fuel = 1.0f;          // 1초간 연료
    predictor.env_fn = projectile_env_default;  // 중력 포함
    predictor.ground_height = 0.0f;
    predictor.max_time = 5.0f;
    predictor.time_step = 0.1f;

    projectile_result_t* result;
    result = projectile_result_create();
    bool success = projectile_predict_missile(&predictor, result);

    CHECK(success == true);
    CHECK(result->valid == true);
    CHECK(result->impact_pos.y == doctest::Approx(0.0f).epsilon(0.1f));
    CHECK(result->impact_time > 1.0f);
    trajectory_print(result->trajectory);  
    projectile_result_destroy(result);  

}

// ---------------------------------------------------------
// TEST: projectile_predict_missile – guidance 없음
// ---------------------------------------------------------
TEST_CASE("projectile_predict_missile: no guidance") {
    missile_predictor_t predictor;
    missile_predictor_init(&predictor);

    predictor.start_pos = {0, 10, 0};
    predictor.start_velocity = {0, 0, 0};
    predictor.thrust = {5, 0, 0};   // X축으로 추력
    predictor.fuel = 0.5f;
    predictor.guidance_fn = projectile_guidance_none;
    predictor.env_fn = projectile_env_default;
    predictor.ground_height = 0.0f;
    predictor.max_time = 5.0f;
    predictor.time_step = 0.1f;

    projectile_result_t* result;
    result = projectile_result_create();
    bool success = projectile_predict_missile(&predictor, result);

    CHECK(success == true);
    CHECK(result->valid == true);
    CHECK(result->impact_pos.x > 0.0f);
    CHECK(result->impact_pos.y == doctest::Approx(0.0f).epsilon(0.1f));

        trajectory_print(result->trajectory);  
        projectile_result_destroy(result);  
}

TEST_CASE("projectile_guidance_to_target: simple target") {
    projectile_t proj;
    projectile_init(&proj);
    proj.velocity = {0, 0, 0};

    vec3_t target = {10, 0, 0}; // x축 +10 위치
    const vec3_t* dir = projectile_guidance_to_target(&proj, 0.1f, &target);

    REQUIRE(dir != nullptr);
    CHECK(doctest::Approx(dir->x).epsilon(0.01f) == 1.0f);
    CHECK(doctest::Approx(dir->y).epsilon(0.01f) == 0.0f);
    CHECK(doctest::Approx(dir->z).epsilon(0.01f) == 0.0f);
}

TEST_CASE("projectile_guidance_lead: moving target") {
    projectile_t proj;
    projectile_init(&proj);
    proj.velocity = {0, 0, 0};

    target_info_t target_info = {{10, 0, 0}, {1, 0, 0}}; // X+ 방향으로 이동 중
    const vec3_t* dir = projectile_guidance_lead(&proj, 0.1f, &target_info);

    REQUIRE(dir != nullptr);
    // 방향이 대략 X축 + 방향인지 확인
    CHECK(dir->x > 0.9f);
    CHECK(std::fabs(dir->y) < 0.01f);
    CHECK(std::fabs(dir->z) < 0.01f);
}

TEST_CASE("projectile_guidance_from_trajectory: fixed point trajectory") {
    projectile_t proj;
    projectile_init(&proj);

    trajectory_t* traj = trajectory_create_full(5);
    motion_state_t state;
    motion_state_init(&state);
    state.linear.position = {5, 0, 0};
    trajectory_add_sample(traj, 0.0f, &state);

    target_traj_info_t target_traj = {traj, 0.0f};
    const vec3_t* dir = projectile_guidance_from_trajectory(
        &proj, 0.1f, &target_traj);

    REQUIRE(dir != nullptr);
    CHECK(dir->x > 0.9f);
    CHECK(std::fabs(dir->y) < 0.01f);
    CHECK(std::fabs(dir->z) < 0.01f);

    trajectory_destroy(traj);
}


