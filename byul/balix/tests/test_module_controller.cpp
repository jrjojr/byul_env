#include "doctest.h"
#include <locale.h>
#include <iostream>

extern "C" {
#include "internal/xform.h"
#include "internal/projectile.h"
#include "internal/controller.h"

#include "internal/common.h"
}

TEST_CASE("PID controller basic response") {
    controller_t* pid_ctrl = controller_create_pid(1.0f, 0.1f, 0.01f, 0.01f, 10.0f);
    REQUIRE(pid_ctrl != nullptr);

    // 목표값과 측정값 차이가 10이면, 양수 출력이 나와야 함
    float output = controller_compute(pid_ctrl, 10.0f, 0.0f, 0.01f);
    CHECK(output > 0.0f);

    // 리셋 후 상태가 초기화되는지 확인
    controller_reset(pid_ctrl);
    float output_after_reset = controller_compute(pid_ctrl, 10.0f, 0.0f, 0.01f);
    CHECK(output_after_reset > 0.0f);

    controller_destroy(pid_ctrl);
}

TEST_CASE("Bang-Bang controller switching") {
    controller_t* bang_ctrl = controller_create_bangbang(5.0f);
    REQUIRE(bang_ctrl != nullptr);

    // 측정값이 목표보다 작으면 +max_output
    float output1 = controller_compute(bang_ctrl, 10.0f, 5.0f, 0.01f);
    CHECK(output1 == doctest::Approx(5.0f));

    // 측정값이 목표보다 크면 -max_output
    float output2 = controller_compute(bang_ctrl, 10.0f, 15.0f, 0.01f);
    CHECK(output2 == doctest::Approx(-5.0f));

    controller_destroy(bang_ctrl);
}

TEST_CASE("MPC controller basic output") {
    mpc_config_t config;
    mpc_config_init(&config);

    environ_t env;
    environ_init(&env);

    bodyprops_t body;
    bodyprops_init(&body);

    controller_t* mpc_ctrl = controller_create_mpc(&config, &env, &body);
    REQUIRE(mpc_ctrl != nullptr);

    // 기본적으로 MPC는 유효한 float 값을 반환해야 함
    float output = controller_compute(mpc_ctrl, 10.0f, 5.0f, 0.01f);
    CHECK(std::isfinite(output));

    controller_reset(mpc_ctrl);
    controller_destroy(mpc_ctrl);
}
