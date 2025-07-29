#include "doctest.h"
#include "vec3.h"
#include "entity_dynamic.h"
#include "trajectory.h"
#include "entity_spring.h"
#include <iostream>

TEST_CASE("Spring Simulation - Two Entities Converging") {
    // --- Initial entity setup ---
    entity_dynamic_t e[2];
    entity_dynamic_init(&e[0]);
    entity_dynamic_init(&e[1]);

    // Entity initial positions (placed farther apart than L0)
    vec3_init_full(&e[0].xf.pos, 0.0f, 0.0f, 0.0f);
    vec3_init_full(&e[1].xf.pos, 5.0f, 0.0f, 0.0f);

    // Initial velocities are 0
    vec3_zero(&e[0].velocity);
    vec3_zero(&e[1].velocity);

    // --- Spring parameters ---
    const float k = 2.0f;      // Spring stiffness
    const float c = 0.1f;      // Damping coefficient
    const float L0 = 2.0f;     // Target distance
    const float dt = 0.1f;     // Simulation time step
    const int steps = 50;      // Number of simulation steps

    // --- Create trajectory ---
    trajectory_t* traj = trajectory_create_full(steps * 2);

    // --- Run simulation ---
    spring_simulate(traj, e, 2, dt, k, c, L0, steps);

    // --- Check final positions ---
    float final_dist = vec3_distance(&e[0].xf.pos, &e[1].xf.pos);
    CHECK(final_dist < 5.0f);          // Entities should move closer
    CHECK(final_dist > L0 - 0.5f);     // Should approach target distance

    // --- Clean up ---
    trajectory_destroy(traj);
}

TEST_CASE("Pairwise Spring - Triangle Stabilization") {
    entity_dynamic_t e[3];
    for (int i = 0; i < 3; ++i)
        entity_dynamic_init(&e[i]);

    // Initial triangle coordinates
    vec3_init_full(&e[0].xf.pos, 0.0f, 0.0f, 0.0f);
    vec3_init_full(&e[1].xf.pos, 5.0f, 0.0f, 0.0f);
    vec3_init_full(&e[2].xf.pos, 2.5f, 0.0f, 4.0f);

    for (int i = 0; i < 3; ++i)
        vec3_zero(&e[i].velocity);

    const float k = 1.0f;
    const float c = 0.5f;
    const float L0 = 3.0f;
    const float dt = 0.05f;
    const int steps = 100;

    trajectory_t* traj = trajectory_create_full(steps * 3);

    spring_simulate_pairwise(traj, e, 3, dt, k, c, L0, steps);

    // Final distance check
    float d01 = vec3_distance(&e[0].xf.pos, &e[1].xf.pos);
    float d12 = vec3_distance(&e[1].xf.pos, &e[2].xf.pos);
    float d20 = vec3_distance(&e[2].xf.pos, &e[0].xf.pos);

    CHECK(d01 > L0 - 0.5f);
    CHECK(d12 > L0 - 0.5f);
    CHECK(d20 > L0 - 0.5f);

    CHECK(d01 < L0 + 0.5f);
    CHECK(d12 < L0 + 0.5f);
    CHECK(d20 < L0 + 0.5f);

    trajectory_destroy(traj);
}

TEST_CASE("Spring network approach-and-separate dynamics") {
    // --- Initial entity setup (two people facing each other) ---
    entity_dynamic_t e[2];
    vec3_init_full(&e[0].xf.pos, 0.0f, 0.0f, 0.0f);  // First person
    vec3_init_full(&e[1].xf.pos, 3.0f, 0.0f, 0.0f);  // Second person (3m apart)

    for (int i = 0; i < 2; ++i) {
        vec3_zero(&e[i].velocity);
        e[i].props.mass = 70.0f;  // Approx. 70 kg body mass
    }

    // --- Spring settings ---
    // Natural length L0 = 2.0m (balance distance)
    // k = 50.0f (strong spring)
    // c = 5.0f (moderate damping)
    spring_link_t links[] = {
        {0, 1, 50.0f, 5.0f, 2.0f}
    };

    trajectory_t traj = {};
    trajectory_init_full(&traj, 3000);

    float dt = 0.01f;   // 10 ms
    int steps = 200;    // 4 seconds simulation
    spring_simulate_network(&traj, e, 2, links, 1, dt, steps);

    // --- Validation ---
    // 1. Entities should approach near L0=2.0m
    float dist = vec3_distance(&e[0].xf.pos, &e[1].xf.pos);
    CHECK(dist == doctest::Approx(2.0f).epsilon(0.2f));  // ±0.2m tolerance

    // 2. Velocities should decrease
    float v0 = vec3_length(&e[0].velocity);
    float v1 = vec3_length(&e[1].velocity);
    CHECK(v0 < 1.0f);
    CHECK(v1 < 1.0f);

    trajectory_free(&traj);
}
