#define DOCTEST_CONFIG_IMPLEMENT
#include "doctest.h"
#include <locale.h>
#include <iostream>

extern "C" {
#include "internal/xform.h"
#include "internal/projectile.h"
}

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

// ---------------------------------------------------------
// 테스트 콜백: on_hit 감지용
// ---------------------------------------------------------
static bool hit_called = false;
void test_hit_cb(const projectile_t* proj, void* userdata) {
    (void)proj; (void)userdata;
    hit_called = true;
}

// ---------------------------------------------------------
// Shell 테스트: 단순 중력만 적용
// ---------------------------------------------------------
TEST_CASE("Shell basic gravity") {
    shell_t shell{};
    shell.base.type = PROJECTILE_TYPE_SHELL;
    shell.base.projectile_id = 1;
    shell.base.velocity = {0, 0, 0};
    shell.base.acceleration = {0, -9.8f, 0};  // 중력
    shell.base.lifetime = 5.0f;
    shell.drag_coef = 0.0f;

    shell.env_fn = projectile_env_none;
    shell.base.on_hit = test_hit_cb;

    xform_t* x0 = xform_new_identity();        // ✅ 위치 초기화
    vec3_t v3 = vec3_t{0,0,0};
    xform_set_position(x0, &v3);
    shell.base.xf = *x0;
    xform_free(x0);

    hit_called = false;

    for (int i = 0; i < 100; ++i) {
        shell_update(&shell, 0.1f);
    }

    CHECK(shell.base.age >= shell.base.lifetime);
    CHECK(hit_called == true);

    vec3_t pos;
    xform_get_position(&shell.base.xf, &pos);
    CHECK(pos.y < 0.0f); // 중력으로 낙하했는지 확인
}

// ---------------------------------------------------------
// Missile 테스트: 타겟 유도 방향
// ---------------------------------------------------------
TEST_CASE("Missile guidance to target") {
    vec3_t target = {10, 0, 0};

    missile_t missile{};
    missile.base.type = PROJECTILE_TYPE_MISSILE;
    missile.base.projectile_id = 2;
    missile.base.velocity = {0, 0, 0};
    missile.base.acceleration = {0, 0, 0};
    missile.thrust = {5, 0, 0};
    missile.fuel = 10.0f;
    missile.guidance_fn = projectile_guidance_to_target;
    missile.guidance_userdata = &target;

    xform_t* x0 = xform_new_identity();        // ✅ 위치 초기화
    vec3_t v3 = vec3_t{0,0,0};
    xform_set_position(x0, &v3);
    missile.base.xf = *x0;
    xform_free(x0);

    for (int i = 0; i < 10; ++i)
        missile_update(&missile, 0.1f);

    vec3_t pos;
    xform_get_position(&missile.base.xf, &pos);

    CHECK(pos.x > 0.5f);  // 유도 방향으로 이동했는지 확인
    CHECK(missile.fuel < 10.0f);  // 연료가 소모되었는지
}

// ---------------------------------------------------------
// 회전 적용 테스트
// ---------------------------------------------------------
TEST_CASE("Projectile angular velocity applies rotation") {
    projectile_t proj{};
    proj.angular_velocity = {0, 1.0f, 0}; // Y축 자전

    xform_t* x0 = xform_new_identity();        // ✅ 회전 초기화
    vec3_t v3 = vec3_t{0,0,0};
    xform_set_position(x0, &v3);
    proj.xf = *x0;
    xform_free(x0);

    vec3_t forward = {0, 0, 1};
    vec3_t world_before, world_after;

    xform_apply_to_direction(&proj.xf, &forward, &world_before);

    // 180도 회전 (pi 라디안)
    projectile_apply_rotation(&proj, 3.14159f);

    xform_apply_to_direction(&proj.xf, &forward, &world_after);

    CHECK(world_before.z > 0.99f); // 원래 전방
    CHECK(world_after.z < -0.99f); // 회전 후 반대 방향
}

TEST_CASE("Projectile prediction with gravity only") {
    projectile_predictor_t pred{};
    pred.start_pos = {0, 10, 0};
    pred.start_velocity = {5, 0, 0};
    pred.gravity = {0, -9.8f, 0};
    pred.env_fn = projectile_env_none;
    pred.ground_height = 0.0f;
    pred.max_time = 10.0f;
    pred.time_step = 0.01f;

    projectile_result_t result{};
    bool ok = projectile_predict(&pred, &result);

    CHECK(ok == true);
    CHECK(result.valid == true);
    CHECK(result.impact_time > 1.0f);
    CHECK(result.impact_pos.y <= 0.0f);
    CHECK(result.impact_pos.x > 0.0f); // 전방으로 나아감
}

TEST_CASE("Missile prediction: basic vertical fall with thrust") {
    missile_predictor_t pred{};
    pred.start_pos = {0, 10, 0};
    pred.start_velocity = {0, 0, 0};
    pred.gravity = {0, -9.8f, 0};
    pred.thrust = {0, 10.0f, 0};  // 위로 향하는 thrust
    pred.fuel = 0.5f;  // 짧게만 작동
    pred.guidance_fn = projectile_guidance_none;
    pred.env_fn = projectile_env_none;
    pred.ground_height = 0.0f;
    pred.max_time = 10.0f;
    pred.time_step = 0.01f;

    projectile_result_t result{};
    bool ok = projectile_predict_missile(&pred, &result);

    CHECK(ok == true);
    CHECK(result.valid == true);
    CHECK(result.impact_time > 0.5f); // 중력과 thrust에 의해 낙하 지연됨
}

static float expect_y(const char* method) {
    if (strcmp(method, "euler") == 0) return 0.0f;
    if (strcmp(method, "semi") == 0) return -9.8f;
    if (strcmp(method, "rk4") == 0) return -4.9f;
    return 0.0f;
}

TEST_CASE("numeq_integrate: Euler") {
    state_vector_t state = {
        .position = {0, 0, 0},
        .velocity = {0, 0, 0},
        .acceleration = {0, 0, 0}
    };
    vec3_t accel = {0, -9.8f, 0};
    integrator_config_t cfg = {
        .type = INTEGRATOR_EULER,
        .time_step = 1.0f
    };

    numeq_integrate(&state, &accel, &cfg);

    CHECK(float_equal(state.position.y, expect_y("euler")));      // 위치 변화 없음
    CHECK(float_equal(state.velocity.y, -9.8f));                  // 속도만 반영
}

TEST_CASE("numeq_integrate: Semi-Implicit Euler") {
    state_vector_t state = {
        .position = {0, 0, 0},
        .velocity = {0, 0, 0},
        .acceleration = {0, 0, 0}
    };
    vec3_t accel = {0, -9.8f, 0};
    integrator_config_t cfg = {
        .type = INTEGRATOR_SEMI_IMPLICIT,
        .time_step = 1.0f
    };

    numeq_integrate(&state, &accel, &cfg);

    CHECK(float_equal(state.position.y, expect_y("semi")));       // 위치 = -9.8
    CHECK(float_equal(state.velocity.y, -9.8f));                  // 속도 = -9.8
}

TEST_CASE("numeq_integrate: RK4") {
    state_vector_t state = {
        .position = {0, 0, 0},
        .velocity = {0, 0, 0},
        .acceleration = {0, 0, 0}
    };
    vec3_t accel = {0, -9.8f, 0};
    integrator_config_t cfg = {
        .type = INTEGRATOR_RK4,
        .time_step = 1.0f
    };

    numeq_integrate(&state, &accel, &cfg);

    CHECK(float_equal(state.position.y, expect_y("rk4")));  // 위치 ≈ -4.9
    CHECK(float_equal(state.velocity.y, -9.8f));                  // 속도 = -9.8
}

static void reset_state(state_vector_t* s) {
    *s = (state_vector_t){
        .position = {0, 0, 0},
        .velocity = {10, 10, 0},
        .acceleration = {0, 0, 0}
    };
}

TEST_CASE("numeq_integrate: RK4 포물선 궤적") {
    state_vector_t s = {
        .position = {0, 0, 0},
        .velocity = {10, 10, 0},    // ✅ 위로 쏘기
        .acceleration = {0, 0, 0}
    };
    vec3_t accel = {0, -9.8f, 0};
    integrator_config_t cfg = { INTEGRATOR_RK4, 1.0f };

    float expected_y[] = {0.0f, 5.1f, 0.4f, -14.1f, -38.4f};

    for (int i = 0; i < 5; ++i) {
        CHECK_MESSAGE(
            fabsf(s.position.y - expected_y[i]) < 0.1f,
            "At t = ", i, ": got ", s.position.y, ", expected ", expected_y[i]
        );
        numeq_integrate(&s, &accel, &cfg);
    }
}

#include <iomanip>

static void simulate_trajectory_and_print(const char* label, integrator_type_t type) {
    std::cout << "=== " << label << " Trajectory ===\n";

    state_vector_t s;
    reset_state(&s);

    vec3_t gravity = {0, -9.8f, 0};
    integrator_config_t cfg = {
        .type = type,
        .time_step = 1.0f
    };

    float t = 0.0f;
    for (int i = 0; i <= 5; ++i) {
        std::cout << std::fixed << std::setprecision(2);
        std::cout << "t = " << t << "s → pos = ("
                  << s.position.x << ", "
                  << s.position.y << ", "
                  << s.position.z << ")\n";
        numeq_integrate(&s, &gravity, &cfg);
        t += cfg.time_step;
    }

    std::cout << "\n";
}

TEST_CASE("시각화 없이 좌표 리스트 출력 - 포물선 궤도") {
    simulate_trajectory_and_print("Euler", INTEGRATOR_EULER);
    simulate_trajectory_and_print("Semi-Implicit", INTEGRATOR_SEMI_IMPLICIT);
    simulate_trajectory_and_print("RK4", INTEGRATOR_RK4);
}
