#include <doctest.h>
#include "propulsion.h"
#include "controller.h"
#include <cstdio>
#include <cstring>
#include <string>

TEST_CASE("propulsion basic init test") {
    propulsion_t p;
    propulsion_init(&p);

    CHECK(p.max_thrust == doctest::Approx(120.0f));
    CHECK(p.current_thrust == doctest::Approx(0.0f));
    CHECK(p.fuel_capacity == doctest::Approx(50.0f));
    CHECK(p.fuel_remaining == doctest::Approx(50.0f));
    CHECK(p.burn_rate == doctest::Approx(0.05));
    CHECK(p.controller == nullptr);
    CHECK(p.active);
}

TEST_CASE("propulsion full init test") {
    propulsion_t p;
    propulsion_init_full(&p, 200.0f, 200.0f, 150.0f, 2.0f, nullptr, true);

    CHECK(p.max_thrust == doctest::Approx(200.0f));
    CHECK(p.fuel_capacity == doctest::Approx(150.0f));
    CHECK(p.fuel_remaining == doctest::Approx(150.0f));
    CHECK(p.burn_rate == doctest::Approx(2.0f));
    CHECK(p.active);
}

TEST_CASE("propulsion predict functions") {
    propulsion_t p;
    propulsion_init_full(&p, 100.0f, 100.0f, 100.0f, 1.0f, nullptr, true);

    float runtime = propulsion_predict_runtime(&p, 100.0f);
    CHECK(runtime == doctest::Approx(1.0f)); // 100 / (1 * 100) = 1s

    p.current_thrust = 50.0f;
    float empty_time = propulsion_predict_empty_time(&p);
    CHECK(empty_time == doctest::Approx(2.0f)); // 100 / (1 * 50) = 2s

    float max_thrust = propulsion_predict_max_thrust(&p, 2.0f);
    CHECK(max_thrust >= 50.0f);
    CHECK(max_thrust <= 100.0f);
}

TEST_CASE("propulsion refuel and consume") {
    propulsion_t p;
    propulsion_init(&p);

    p.fuel_remaining = 20.0f;
    propulsion_refuel(&p, 50.0f);
    CHECK(p.fuel_remaining == doctest::Approx(50.0f));

    propulsion_consume(&p, 30.0f);
    CHECK(p.fuel_remaining == doctest::Approx(20.0f));

    propulsion_consume(&p, 100.0f);
    CHECK(p.fuel_remaining == doctest::Approx(0.0f));
    CHECK_FALSE(p.active);
}

TEST_CASE("propulsion string and json") {
    propulsion_t p;
    propulsion_init_full(&p, 100.0f, 100.0f, 100.0f, 1.0f, nullptr, true);
    p.current_thrust = 80.0f;
    p.fuel_remaining = 45.0f;

    char buffer[128];
    const char* str = propulsion_to_string(&p, sizeof(buffer), buffer);
    REQUIRE(str != nullptr);
    printf("[to_string] %s\n", str);
    CHECK(std::string(str).find("Thrust") != std::string::npos);
    CHECK(std::string(str).find("Fuel") != std::string::npos);

    const char* json = propulsion_to_json(&p, buffer, sizeof(buffer));
    REQUIRE(json != nullptr);
    printf("[to_json] %s\n", json);
    CHECK(std::string(json).find("thrust") != std::string::npos);
    CHECK(std::string(json).find("fuel") != std::string::npos);
}

TEST_CASE("propulsion with PID, MPC, and Bang-Bang controller simulation") {
    const float dt = 1.0f;
    const float target_thrust = 80.0f;
    const int max_steps = 20;

    // PID
    propulsion_t pid_prop;
    propulsion_init_full(&pid_prop, 100.0f, target_thrust, 500.0f, 1.0f, nullptr, true);
    controller_t* pid_ctrl = controller_create_pid(1.0f, 0.1f, 0.05f, dt, 100.0f);
    propulsion_attach_controller(&pid_prop, pid_ctrl);

    printf("\n[PID Controller Simulation]\n");
    int pid_steps = 0;
    while (!propulsion_is_empty(&pid_prop) && pid_steps < max_steps) {
        propulsion_update(&pid_prop, dt);
        printf("PID Step %2d | Thrust = %.2f N | Fuel = %.2f kg\n",
               pid_steps, pid_prop.current_thrust, pid_prop.fuel_remaining);
        pid_steps++;
    }
    controller_destroy(pid_ctrl);

    // MPC
    propulsion_t mpc_prop;
    propulsion_init_full(&mpc_prop, 100.0f, target_thrust, 500.0f, 1.0f, nullptr, true);
    mpc_config_t mpc_cfg;
    mpc_config_init(&mpc_cfg);
    mpc_cfg.max_accel = 80.0f;
    mpc_cfg.step_dt = dt;
    controller_t* mpc_ctrl = controller_create_mpc(&mpc_cfg, nullptr, nullptr);
    propulsion_attach_controller(&mpc_prop, mpc_ctrl);

    printf("\n[MPC Controller Simulation]\n");
    int mpc_steps = 0;
    while (!propulsion_is_empty(&mpc_prop) && mpc_steps < max_steps) {
        propulsion_update(&mpc_prop, dt);
        printf("MPC Step %2d | Thrust = %.2f N | Fuel = %.2f kg\n",
               mpc_steps, mpc_prop.current_thrust, mpc_prop.fuel_remaining);
        mpc_steps++;
    }
    controller_destroy(mpc_ctrl);

    // Bang-Bang
    propulsion_t bb_prop;
    propulsion_init_full(&bb_prop, 100.0f, 100.0f, 500.0f, 1.0f, nullptr, true);
    controller_t* bb_ctrl = controller_create_bangbang(100.0f);
    propulsion_attach_controller(&bb_prop, bb_ctrl);

    printf("\n[Bang-Bang Controller Simulation]\n");
    int bb_steps = 0;
    while (!propulsion_is_empty(&bb_prop) && bb_steps < max_steps) {
        propulsion_update(&bb_prop, dt);
        printf("Bang-Bang Step %2d | Thrust = %.2f N | Fuel = %.2f kg\n",
               bb_steps, bb_prop.current_thrust, bb_prop.fuel_remaining);
        bb_steps++;
    }
    controller_destroy(bb_ctrl);

    CHECK(pid_steps > 0);
    CHECK(mpc_steps > 0);
    CHECK(bb_steps > 0);
}

