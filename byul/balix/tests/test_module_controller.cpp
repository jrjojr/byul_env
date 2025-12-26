#include "doctest.h"
#include <locale.h>
#include <iostream>

extern "C" {
#include "xform.h"
#include "projectile.h"
#include "controller.h"

#include "float_core.h"
}

TEST_CASE("PID controller basic response") {
    controller_t* pid_ctrl = controller_create_pid(1.0f, 0.1f, 0.01f, 0.01f, 10.0f);
    REQUIRE(pid_ctrl != nullptr);

    // If the difference between setpoint and measured value is 10, output should be positive
    float output = controller_compute(pid_ctrl, 10.0f, 0.0f, 0.01f);
    CHECK(output > 0.0f);

    // After reset, state should be initialized
    controller_reset(pid_ctrl);
    float output_after_reset = controller_compute(pid_ctrl, 10.0f, 0.0f, 0.01f);
    CHECK(output_after_reset > 0.0f);

    controller_destroy(pid_ctrl);
}

TEST_CASE("Bang-Bang controller switching") {
    controller_t* bang_ctrl = controller_create_bangbang(5.0f);
    REQUIRE(bang_ctrl != nullptr);

    // If measured value is smaller than target, output should be +max_output
    float output1 = controller_compute(bang_ctrl, 10.0f, 5.0f, 0.01f);
    CHECK(output1 == doctest::Approx(5.0f));

    // If measured value is larger than target, output should be -max_output
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

    // MPC should return a valid float value
    float output = controller_compute(mpc_ctrl, 10.0f, 5.0f, 0.01f);
    CHECK(std::isfinite(output));

    controller_reset(mpc_ctrl);
    controller_destroy(mpc_ctrl);
}
