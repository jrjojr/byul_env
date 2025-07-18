#include "doctest.h"

extern "C" {
    #include "internal/numeq_pid.h"
}

#include "internal/vec3.hpp"
// -------------------- Scalar PID Tests --------------------

TEST_CASE("Scalar PID: Proportional only (Kp only)") {
    pid_controller_t pid;
    pid_init(&pid, 1.0f, 0.0f, 0.0f, 0.1f);

    float ctrl = pid_update(&pid, 5.0f, 2.0f);  // error = 3.0
    CHECK(ctrl == doctest::Approx(3.0f));
}

TEST_CASE("Scalar PID: Full PID output (Kp, Ki, Kd)") {
    pid_controller_t pid;
    pid_init(&pid, 2.0f, 0.5f, 1.0f, 1.0f);
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
    pid_init(&pid, 1.0f, 1.0f, 0.0f, 1.0f);
    pid.output_limit = 2.0f;
    pid.anti_windup = true;

    float ctrl1 = pid_update(&pid, 10.0f, 0.0f);  // error = 10, output = 10 + 10 = 20 → clamp
    CHECK(ctrl1 <= 2.0f);

    float ctrl2 = pid_update(&pid, 10.0f, 0.0f);  // windup 없으면 적분 계속 증가
    CHECK(ctrl2 <= 2.0f);
}

TEST_CASE("Scalar PID: Set and Reset State") {
    pid_controller_t pid;
    pid_init(&pid, 1.0f, 1.0f, 1.0f, 1.0f);
    pid_set_state(&pid, 3.0f, 2.0f);

    CHECK(pid.integral == doctest::Approx(3.0f));
    CHECK(pid.prev_error == doctest::Approx(2.0f));

    pid_reset(&pid);
    CHECK(pid.integral == doctest::Approx(0.0f));
    CHECK(pid.prev_error == doctest::Approx(0.0f));
}

TEST_CASE("Scalar PID: Preview should not change state") {
    pid_controller_t pid;
    pid_init(&pid, 1.0f, 1.0f, 1.0f, 1.0f);
    pid_set_state(&pid, 5.0f, 2.0f);

    float before_integral = pid.integral;
    float preview = pid_preview(&pid, 4.0f, 1.0f);
    CHECK(preview > 0.0f);
    CHECK(pid.integral == before_integral);  // 상태 유지
}

// -------------------- Vec3 PID Tests --------------------

TEST_CASE("Vec3 PID: Basic update per axis") {
    pid_controller_vec3_t pid;
    pid_vec3_init(&pid, 1.0f, 0.0f, 0.0f, 1.0f);

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
    pid_vec3_init(&pid, 1.0f, 1.0f, 0.0f, 1.0f);

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
    pid_vec3_init(&pid1, 2.0f, 1.0f, 0.5f, 1.0f);
    Vec3 a = Vec3(1.0f, 2.0f, 3.0f);
    Vec3 b = Vec3(0.5f, 0.5f, 0.5);
    pid_vec3_set_state(&pid1,
        &a.v,
        &b.v
    );

    pid_vec3_copy(&pid2, &pid1);

    CHECK(pid2.x.integral == doctest::Approx(1.0f));
    CHECK(pid2.y.integral == doctest::Approx(2.0f));
    CHECK(pid2.z.integral == doctest::Approx(3.0f));

    pid_vec3_reset(&pid2);
    CHECK(pid2.x.integral == doctest::Approx(0.0f));
    CHECK(pid2.y.prev_error == doctest::Approx(0.0f));
}
