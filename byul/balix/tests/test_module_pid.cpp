#include "doctest.h"

#include <cmath>
#include <cstdio>

extern "C" {
    #include "numeq_pid.h"
    #include "numeq_pid_vec3.h"
}

#include "vec3.hpp"
// -------------------- Scalar PID Tests --------------------

TEST_CASE("Scalar PID: Proportional only (Kp only)") {
    pid_controller_t pid;
    pid_init_full(&pid, 1.0f, 0.0f, 0.0f, 0.1f);

    float ctrl = pid_update(&pid, 5.0f, 2.0f);  // error = 3.0
    CHECK(ctrl == doctest::Approx(3.0f));
}

TEST_CASE("Scalar PID: Full PID output (Kp, Ki, Kd)") {
    pid_controller_t pid;
    pid_init_full(&pid, 2.0f, 0.5f, 1.0f, 1.0f);
    pid_reset(&pid);

    float out1 = pid_update(&pid, 4.0f, 1.0f);  // error = 3
    // P = 6, I = 1.5, D = 3 → total = 10.5
    CHECK(out1 == doctest::Approx(10.5f));

    float out2 = pid_update(&pid, 4.0f, 3.0f);  // error = 1
    // P = 2, I = 2.0, D = -2 → total = 2.0
    CHECK(out2 == doctest::Approx(2.0f));
}

TEST_CASE("Scalar PID: Output limit + Anti-windup") {
    pid_controller_t pid;
    pid_init_full(&pid, 1.0f, 1.0f, 0.0f, 1.0f);
    pid.output_limit = 2.0f;
    pid.anti_windup = true;

    float ctrl1 = pid_update(&pid, 10.0f, 0.0f);  // error = 10, output = 10 + 10 = 20 → clamp
    CHECK(ctrl1 <= 2.0f);

    float ctrl2 = pid_update(&pid, 10.0f, 0.0f);  // windup 없으면 적분 계속 증가
    CHECK(ctrl2 <= 2.0f);
}

TEST_CASE("Scalar PID: Set and Reset State") {
    pid_controller_t pid;
    pid_init_full(&pid, 1.0f, 1.0f, 1.0f, 1.0f);
    pid_set_state(&pid, 3.0f, 2.0f);

    CHECK(pid.integral == doctest::Approx(3.0f));
    CHECK(pid.prev_error == doctest::Approx(2.0f));

    pid_reset(&pid);
    CHECK(pid.integral == doctest::Approx(0.0f));
    CHECK(pid.prev_error == doctest::Approx(0.0f));
}

TEST_CASE("Scalar PID: Preview should not change state") {
    pid_controller_t pid;
    pid_init_full(&pid, 1.0f, 1.0f, 1.0f, 1.0f);
    pid_set_state(&pid, 5.0f, 2.0f);

    float before_integral = pid.integral;
    float preview = pid_preview(&pid, 4.0f, 1.0f);
    CHECK(preview > 0.0f);
    CHECK(pid.integral == before_integral);  // 상태 유지
}

// -------------------- Vec3 PID Tests --------------------

TEST_CASE("Vec3 PID: Basic update per axis") {
    pid_controller_vec3_t pid;
    pid_vec3_init_full(&pid, 1.0f, 0.0f, 0.0f, 1.0f);

    vec3_t target = {1.0f, 2.0f, 3.0f};
    vec3_t measured = {0.5f, 1.0f, 1.5f};
    vec3_t control = {0};

    pid_vec3_update(&pid, &target, &measured, &control);
    CHECK(control.x == doctest::Approx(0.5f));
    CHECK(control.y == doctest::Approx(1.0f));
    CHECK(control.z == doctest::Approx(1.5f));
}

TEST_CASE("Vec3 PID: Preview vs Update state check") {
    pid_controller_vec3_t pid;
    pid_vec3_init_full(&pid, 1.0f, 1.0f, 0.0f, 1.0f);

    vec3_t target = {2.0f, 2.0f, 2.0f};
    vec3_t measured = {0.0f, 0.0f, 0.0f};

    vec3_t ctrl_prev = {0};
    pid_vec3_preview(&pid, &target, &measured, &ctrl_prev);

    vec3_t ctrl_real = {0};
    pid_vec3_update(&pid, &target, &measured, &ctrl_real);

    // preview와 update의 값은 같아야 하지만 내부 상태는 update만 바뀜
    CHECK(ctrl_prev.x == doctest::Approx(ctrl_real.x));
}

TEST_CASE("Vec3 PID: Copy and Reset") {
    pid_controller_vec3_t pid1, pid2;
    pid_vec3_init_full(&pid1, 2.0f, 1.0f, 0.5f, 1.0f);
    Vec3 a = Vec3(1.0f, 2.0f, 3.0f);
    Vec3 b = Vec3(0.5f, 0.5f, 0.5);
    pid_vec3_set_state(&pid1,
        &a.v,
        &b.v
    );

    pid_vec3_assign(&pid2, &pid1);

    CHECK(pid2.x.integral == doctest::Approx(1.0f));
    CHECK(pid2.y.integral == doctest::Approx(2.0f));
    CHECK(pid2.z.integral == doctest::Approx(3.0f));

    pid_vec3_reset(&pid2);
    CHECK(pid2.x.integral == doctest::Approx(0.0f));
    CHECK(pid2.y.prev_error == doctest::Approx(0.0f));
}

TEST_CASE("pid_basic_init") {
    pid_controller_t pid;
    pid_init(&pid);

    CHECK(pid.kp == doctest::Approx(1.0f));
    CHECK(pid.ki == doctest::Approx(0.0f));
    CHECK(pid.kd == doctest::Approx(0.0f));
    CHECK(pid.dt == doctest::Approx(0.01f));
    CHECK(pid.integral == doctest::Approx(0.0f));
    CHECK(pid.prev_error == doctest::Approx(0.0f));
    CHECK(pid.output_limit == doctest::Approx(0.0f));
    CHECK(pid.anti_windup == false);
}

TEST_CASE("pid_update_simple_proportional") {
    pid_controller_t pid;
    pid_init_full(&pid, 2.0f, 0.0f, 0.0f, 0.1f); // Kp=2, Ki=0, Kd=0
    float target = 10.0f;
    float measured = 7.0f;

    float control = pid_update(&pid, target, measured);
    float expected = 2.0f * (target - measured); // P=Kp * error = 2 * 3 = 6
    CHECK(control == doctest::Approx(expected));
}

TEST_CASE("pid_update_with_integral_and_derivative") {
    pid_controller_t pid;
    pid_init_full(&pid, 1.0f, 0.5f, 0.1f, 0.1f); // P=1, I=0.5, D=0.1, dt=0.1
    pid_reset(&pid);

    float target = 10.0f;
    float measured = 8.0f;

    // 첫 번째 업데이트
    float control1 = pid_update(&pid, target, measured);

    CHECK(control1 == doctest::Approx(4.1f)); // P + I + D = 4.1


    // 두 번째 업데이트 (오차 변화 확인)
    measured = 9.0f; // error = 1
    float control2 = pid_update(&pid, target, measured);
    // P = 1 * 1 = 1
    // I = 0.1 + 0.5 * 1 * 0.1 = 0.15
    // D = 0.1 * (1 - 2) / 0.1 = -1.0
    CHECK(control2 == doctest::Approx(0.15f)); // 1 + 0.15 - 1 = 0.15
}

TEST_CASE("pid_output_limit_and_anti_windup") {
    pid_controller_t pid;
    pid_init_full(&pid, 10.0f, 5.0f, 0.0f, 0.1f); // 강한 P, I
    pid.output_limit = 5.0f; // 출력 제한
    pid.anti_windup = true;  // 적분항 제한 활성화

    float target = 10.0f;
    float measured = 0.0f;

    for (int i = 0; i < 10; ++i) {
        float control = pid_update(&pid, target, measured);
        CHECK(control <= 5.0f); // 출력이 5.0을 초과하지 않음
    }
}

TEST_CASE("pid_preview_test") {
    pid_controller_t pid;
    pid_init_full(&pid, 1.0f, 0.2f, 0.1f, 0.1f);

    float target = 10.0f;
    float measured = 5.0f;

    float preview = pid_preview(&pid, target, measured);
    float control = pid_update(&pid, target, measured);

    CHECK(std::fabs(preview - control) < 1e-4f); // preview는 update와 같은 결과
}
