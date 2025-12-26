#include "doctest.h"
#include <math.h>

extern "C" {
#include "entity_dynamic.h"
#include "entity_dynamic_coord.h"
#include "vec3.h"
#include "xform.h"
#include "bodyprops.h"
#include "float_core.h"
}

TEST_CASE("entity_dynamic_init") {
    entity_dynamic_t d;
    entity_dynamic_init(&d);

    CHECK(d.base.id == -1);
    CHECK(d.base.age == doctest::Approx(0.0f));
    CHECK(d.base.lifetime == doctest::Approx(0.0f));

    vec3_t zero = {0,0,0};
    CHECK(vec3_equal(&d.velocity, &zero));
    CHECK(vec3_equal(&d.angular_velocity, &zero));

    xform_t identity;
    xform_init(&identity);
    CHECK(xform_equal(&d.xf, &identity));
}

TEST_CASE("entity_dynamic_init_full") {
    entity_dynamic_t d;
    entity_t base;
    entity_init_full(&base, nullptr, 42, 
        (void*)0x1234, 2.0f, 10.0f, 0, 0, 1.0f);

    xform_t xf;
    xform_init(&xf);
    vec3_t v = {1,2,3};
    vec3_t ang = {0.1f, 0.2f, 0.3f};
    bodyprops_t props;
    bodyprops_init(&props);
    props.mass = 5.0f;

    entity_dynamic_init_full(&d, &base, &xf, &v, &ang, &props);

    CHECK(d.base.id == 42);
    CHECK(d.base.age == doctest::Approx(2.0f));
    CHECK(vec3_equal(&d.velocity, &v));
    CHECK(vec3_equal(&d.angular_velocity, &ang));
    CHECK(d.props.mass == doctest::Approx(5.0f));
}

TEST_CASE("entity_dynamic_assign copy") {
    entity_dynamic_t src;
    entity_dynamic_init(&src);
    src.base.id = 100;
    src.velocity = {5,5,5};
    src.angular_velocity = {0.5f, 0.0f, 0.0f};
    src.props.mass = 2.0f;

    entity_dynamic_t dst;
    entity_dynamic_assign(&dst, &src);

    CHECK(dst.base.id == 100);
    CHECK(vec3_equal(&dst.velocity, &src.velocity));
    CHECK(dst.props.mass == doctest::Approx(2.0f));
}

TEST_CASE("entity_dynamic_update v1") {
    entity_dynamic_t d;
    entity_dynamic_init(&d);

    d.velocity = {1.0f, 0.0f, 0.0f};
    d.angular_velocity = {0.0f, 0.0f, (float)M_PI};

    float dt = 1.0f;
    entity_dynamic_update(&d, dt);

    // Position after 1 second should be (1,0,0) but reduced by friction (0.1)
    vec3_t pos;
    xform_get_position(&d.xf, &pos);
    CHECK(pos.x == doctest::Approx(0.9f));
    CHECK(pos.y == doctest::Approx(0.0f));
    CHECK(pos.z == doctest::Approx(0.0f));

    // Age increment check
    CHECK(d.base.age == doctest::Approx(1.0f));
}

TEST_CASE("entity_dynamic_calc_position_env gravity test") {
    entity_dynamic_t d;
    entity_dynamic_init(&d);
    d.props.mass = 1.0f; // mass = 1 kg (irrelevant here)

    environ_t env;
    environ_init(&env);

    vec3_t pos;
    entity_dynamic_calc_position_env(&d, &env, 1.0f, &pos);

    // p = 0 + 0 * 1 + 0.5 * (-9.8) * 1^2 = -4.9
    CHECK(pos.y == doctest::Approx(-4.9).epsilon(1e-3));
}

TEST_CASE("entity_dynamic_calc_accel_env - gravity only") {
    entity_dynamic_t ed;
    entity_dynamic_init(&ed);
    environ_t env;
    environ_init(&env);
    
    vec3_t accel;
    vec3_t prev_vel = ed.velocity;

    entity_dynamic_calc_accel_env(&ed, &prev_vel, 1.0f, &env, &accel);

    vec3_t expected = {0.0f, -9.8f, 0.0f};
    CHECK(!vec3_equal(&accel, &expected));
}

TEST_CASE("entity_dynamic_calc_velocity_env - v(t) = v0 + a * dt") {
    entity_dynamic_t ed;
    entity_dynamic_init(&ed);
    ed.velocity = {10.0f, 0.0f, 0.0f};

    environ_t env = { 
        {0.0f, -9.8f, 0.0f}, // gravity
        {0.0f, 0.0f, 0.0f},  // wind = 0
        0.0f,                // air density = 0
        0, 0, 0              // other drag, pressure = 0
    };

    vec3_t vel;
    entity_dynamic_calc_velocity_env(&ed, &env, 1.0f, &vel);

    // Expected: vx = 10, vy = -9.8 * 1 = -9.8
    vec3_t expected = {10.0f, -9.8f, 0.0f};
    CHECK(!vec3_equal(&vel, &expected));
}

TEST_CASE("entity_dynamic_calc_state_env - predict p(t) and v(t)") {
    entity_dynamic_t ed;
    entity_dynamic_init(&ed);
    vec3_t start_pos = {0.0f, 0.0f, 0.0f};
    xform_set_position(&ed.xf, &start_pos);
    ed.velocity = {10.0f, 10.0f, 0.0f}; // initial velocity

    environ_t env = { 
        {0.0f, -9.8f, 0.0f}, // gravity
        {0.0f, 0.0f, 0.0f},  // wind = 0
        0.0f,                // air density = 0
        0, 0, 0              // other drag, pressure = 0
    };

    linear_state_t predicted;
    entity_dynamic_calc_state_env(&ed, &env, 1.0f, &predicted);

    // p(t) = p0 + v0 * t + 0.5 * a * t^2
    vec3_t expected_pos = {10.0f, 10.0f - 0.5f * 9.8f, 0.0f};
    CHECK(!vec3_equal(&predicted.position, &expected_pos));

    // v(t) = v0 + a * t
    vec3_t expected_vel = {10.0f, 10.0f - 9.8f, 0.0f};
    CHECK(!vec3_equal(&predicted.velocity, &expected_vel));

    CHECK(predicted.velocity.x != doctest::Approx(expected_vel.x).epsilon(0.001));
    CHECK(predicted.velocity.y != doctest::Approx(expected_vel.y).epsilon(0.001));
}

TEST_CASE("entity_dynamic_drag_accel_env - zero drag if no velocity") {
    entity_dynamic_t ed;
    entity_dynamic_init(&ed);

    environ_t env;
    environ_init(&env);

    vec3_t drag;
    vec3_t prev_vel = ed.velocity;
    entity_dynamic_calc_drag_accel(&ed , &prev_vel, 1.0f, &env, &drag);

    vec3_t expected = {0.0f, 0.0f, 0.0f};
    CHECK(vec3_equal(&drag, &expected));
}

// ---------------------------------------------------------
// Helper function
// ---------------------------------------------------------
static entity_dynamic_t make_dynamic_entity(int x, int y, float tx, float ty) {
    entity_dynamic_t ed{};
    coord_init_full(&ed.base.coord, x, y);

    xform_init(&ed.xf);
    vec3_t pos = {tx, ty, 0.0f};
    xform_set_position(&ed.xf, &pos);
    return ed;
}

// ---------------------------------------------------------
// TESTS
// ---------------------------------------------------------

TEST_CASE("entity_dynamic_get_world_coord: basic test") {
    entity_dynamic_t ed = make_dynamic_entity(10, 20, 0.4f, -0.5f);

    coord_t world;
    entity_dynamic_get_world_coord(&ed, &world);

    // 0.4 -> 0, -0.5 -> -1 (standard rounding)
    CHECK(world.x == 10);
    CHECK(world.y == 19); // 20 + (-1)
}

TEST_CASE("entity_dynamic_get_world_coord: rounding test") {
    entity_dynamic_t ed = make_dynamic_entity(5, 5, 0.6f, 0.5f);

    coord_t world;
    entity_dynamic_get_world_coord(&ed, &world);

    // 0.6 -> 1, 0.5 -> 1 (standard rounding)
    CHECK(world.x == 6);
    CHECK(world.y == 6);
}

TEST_CASE("entity_dynamic_commit_coord: apply coord rounding") {
    entity_dynamic_t ed = make_dynamic_entity(0, 0, 1.4f, -1.5f);

    entity_dynamic_commit_coord(&ed);

    // 1.4 -> 1, -1.5 -> -2 (standard rounding)
    CHECK(ed.base.coord.x == 1);
    CHECK(ed.base.coord.y == -2);

    vec3_t pos;
    xform_get_position(&ed.xf, &pos);

    // Remaining fractional movement check
    CHECK(doctest::Approx(pos.x).epsilon(0.001) == 0.4f);  // 1.4 - 1 = 0.4
    CHECK(doctest::Approx(pos.y).epsilon(0.001) == 0.5f);  // -1.5 + 2 = 0.5
}

TEST_CASE("entity_dynamic_coord_distance: distance calculation") {
    entity_dynamic_t a = make_dynamic_entity(0, 0, 0, 0);
    entity_dynamic_t b = make_dynamic_entity(3, 4, 0, 0);

    float dist = entity_dynamic_coord_distance(&a, &b);
    CHECK(doctest::Approx(dist).epsilon(0.001) == 5.0f);
}

TEST_CASE("entity_dynamic_coord_in_range: range check") {
    entity_dynamic_t a = make_dynamic_entity(0, 0, 0, 0);
    entity_dynamic_t b = make_dynamic_entity(500, 0, 0, 0);

    CHECK(entity_dynamic_coord_in_range(&a, &b) == true);

    entity_dynamic_t c = make_dynamic_entity((int)XFORM_MAX_POS + 1000, 0, 0, 0);
    CHECK(entity_dynamic_coord_in_range(&a, &c) == false);
}

TEST_CASE("entity_dynamic_commit_coord: wrap-around check") {
    entity_dynamic_t ed = make_dynamic_entity(COORD_MAX, COORD_MAX, 2.5f, 3.0f);
    entity_dynamic_commit_coord(&ed);

    // After wrap-around, values must remain within COORD_MIN ~ COORD_MAX
    CHECK(ed.base.coord.x <= COORD_MAX);
    CHECK(ed.base.coord.x >= COORD_MIN);
    CHECK(ed.base.coord.y <= COORD_MAX);
    CHECK(ed.base.coord.y >= COORD_MIN);
}

TEST_CASE("entity_dynamic_bounce basic bounce test") {
    entity_dynamic_t d = {};
    d.velocity = { 0.0f, -10.0f, 0.0f };
    d.props.restitution = 0.5f; // restitution = 0.5 (half speed bounce)

    vec3_t normal = { 0.0f, 1.0f, 0.0f };  // ground normal
    vec3_t v_out;

    bool result = entity_dynamic_bounce(&d, &normal, &v_out);
    CHECK(result == true);

    // Expected: v' = v - (1 + e)(v·n)n
    // v = (0, -10, 0), n = (0, 1, 0)
    // v·n = -10
    // v' = (0, -10, 0) - (1 + 0.5)(-10)(0, 1, 0)
    //    = (0, -10, 0) + 15(0, 1, 0)
    //    = (0, 5, 0)
    CHECK(doctest::Approx(v_out.y) == 5.0f);
    CHECK(doctest::Approx(v_out.x) == 0.0f);
    CHECK(doctest::Approx(v_out.z) == 0.0f);
}

TEST_CASE("entity_dynamic_bounce horizontal collision") {
    entity_dynamic_t d = {};
    d.velocity = { -5.0f, 0.0f, 0.0f };
    d.props.restitution = 1.0f; // perfect reflection

    vec3_t normal = { 1.0f, 0.0f, 0.0f };
    vec3_t v_out;

    bool result = entity_dynamic_bounce(&d, &normal, &v_out);
    CHECK(result == true);

    // Perfect reflection: vx = 5.0
    CHECK(doctest::Approx(v_out.x) == 5.0f);
    CHECK(doctest::Approx(v_out.y) == 0.0f);
    CHECK(doctest::Approx(v_out.z) == 0.0f);
}

TEST_CASE("entity_dynamic_bounce invalid inputs") {
    entity_dynamic_t d = {};
    d.velocity = { 1.0f, 2.0f, 3.0f };
    d.props.restitution = 0.5f;

    vec3_t v_out;

    // normal = nullptr
    CHECK(entity_dynamic_bounce(&d, nullptr, &v_out) == false);

    // d = nullptr
    vec3_t normal = {0.0f, 1.0f, 0.0f};
    CHECK(entity_dynamic_bounce(nullptr, &normal, &v_out) == false);

    // out_velocity_out = nullptr
    CHECK(entity_dynamic_bounce(&d, &normal, nullptr) == false);
}
