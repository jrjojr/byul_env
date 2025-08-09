#include "doctest.h"
#include "entity_interaction.h"
#include "entity_dynamic.h"
#include "vec3.h"
#include <cmath>
#include <stdio.h>

TEST_CASE("entity_dynamic_distance returns correct Euclidean distance") {
    entity_dynamic_t a, b;
    entity_dynamic_init(&a);
    entity_dynamic_init(&b);

    a.xf.pos = {0.0f, 0.0f, 0.0f};
    b.xf.pos = {3.0f, 4.0f, 0.0f};

    float dist = entity_dynamic_distance(&a, &b);
    CHECK(doctest::Approx(dist) == 5.0f);
}

TEST_CASE("entity_dynamic_in_contact returns true when within radius + tolerance") {
    entity_dynamic_t a, b;
    entity_dynamic_init(&a);
    entity_dynamic_init(&b);

    a.xf.pos = {0.0f, 0.0f, 0.0f};
    b.xf.pos = {1.0f, 0.0f, 0.0f};

    a.base.width_range = 1;
    b.base.width_range = 1;
    a.base.influence_ratio = 1.0f;
    b.base.influence_ratio = 1.0f;

    bool contact = entity_dynamic_in_contact(&a, &b, 0.2f);
    CHECK(contact == true);
}

TEST_CASE("entity_dynamic_predict_collision_time returns valid positive time") {
    entity_dynamic_t a, b;
    entity_dynamic_init(&a);
    entity_dynamic_init(&b);

    a.xf.pos = {0.0f, 0.0f, 0.0f};
    b.xf.pos = {10.0f, 0.0f, 0.0f};

    a.velocity = {1.0f, 0.0f, 0.0f};
    b.velocity = {-1.0f, 0.0f, 0.0f};

    float t = entity_dynamic_predict_collision_time(&a, &b);
    CHECK(t > 0.0f);
    CHECK(doctest::Approx(t).epsilon(0.01) == 5.0f);
}

TEST_CASE("entity_dynamic_collision_point computes midpoint at collision") {
    entity_dynamic_t a, b;
    entity_dynamic_init(&a);
    entity_dynamic_init(&b);

    a.xf.pos = {0.0f, 0.0f, 0.0f};
    b.xf.pos = {10.0f, 0.0f, 0.0f};

    a.velocity = {1.0f, 0.0f, 0.0f};
    b.velocity = {-1.0f, 0.0f, 0.0f};

    vec3_t cp;
    int result = entity_dynamic_collision_point(&cp, &a, &b);
    CHECK(result == 1);
    CHECK(doctest::Approx(cp.x) == 5.0f);
    CHECK(doctest::Approx(cp.y) == 0.0f);
    CHECK(doctest::Approx(cp.z) == 0.0f);
}

TEST_CASE("entity_dynamic_predict_collision_time_env computes approx with gravity") {
    entity_dynamic_t a, b;
    entity_dynamic_init(&a);
    entity_dynamic_init(&b);
    environ_t env = { .gravity = {0.0f, -9.8f, 0.0f} };

    vec3_init_full(&a.xf.pos, 0.0f, 0.0f, 0.0f);
    vec3_init_full(&b.xf.pos, 10.0f, 0.0f, 0.0f);
    vec3_init_full(&a.velocity, 1.0f, 0.0f, 0.0f);
    vec3_init_full(&b.velocity, -1.0f, 0.0f, 0.0f);

    float t = entity_dynamic_predict_collision_time_env(&a, &b, &env);
    printf("collition time env : %f\n", t);
    CHECK(t > 0.0f);
}

TEST_CASE("entity_dynamic_collision_point_env returns collision point with env") {
    entity_dynamic_t a, b;
    entity_dynamic_init(&a);
    entity_dynamic_init(&b);
    environ_t env = { .gravity = {0.0f, -9.8f, 0.0f} };

    vec3_init_full(&a.xf.pos, 0.0f, 0.0f, 0.0f);
    vec3_init_full(&b.xf.pos, 10.0f, 0.0f, 0.0f);
    vec3_init_full(&a.velocity, 1.0f, 0.0f, 0.0f);
    vec3_init_full(&b.velocity, -1.0f, 0.0f, 0.0f);

    vec3_t out;
    int result = entity_dynamic_collision_point_env(&out, &a, &b, &env);
    CHECK(result == 1);

    char buf[64];
    printf("collision point env : %s\n",     vec3_to_string(&out, 64, buf));
    CHECK(out.x > 0.0f);
}
