#include "doctest.h"
#include <locale.h>
#include <iostream>

extern "C" {
#include "xform.h"
}

#include "vec3.hpp"
#include "float_common.h"

TEST_CASE("xform: identity transform") {
    xform_t xf;
    xform_init(&xf);

    vec3_t pos;
    xform_get_position(&xf, &pos);
    CHECK(pos.x == doctest::Approx(0.0f));
    CHECK(pos.y == doctest::Approx(0.0f));
    CHECK(pos.z == doctest::Approx(0.0f));
}

TEST_CASE("xform: axis-angle roundtrip") {
    vec3_t pos = {1.0f, 2.0f, 3.0f};
    vec3_t axis = {0.0f, 1.0f, 0.0f};
    float radians = 3.1415926f / 2.0f;

    xform_t xf;
    xform_init_axis_angle(&xf, &pos, &axis, radians);

    vec3_t got_pos;
    vec3_t got_axis;
    float got_radians;

    xform_get_position(&xf, &got_pos);
    xform_get_axis_angle(&xf, &got_axis, &got_radians);

    CHECK(got_pos.x == doctest::Approx(pos.x));
    CHECK(got_pos.y == doctest::Approx(pos.y));
    CHECK(got_pos.z == doctest::Approx(pos.z));
    CHECK(got_radians == doctest::Approx(radians));
    CHECK(got_axis.y == doctest::Approx(1.0f));
}

TEST_CASE("xform: translate and apply") {
    xform_t xf;
    xform_init(&xf);

    vec3_t delta = {5.0f, 0.0f, 0.0f};
    xform_translate(&xf, &delta);

    vec3_t local = {1.0f, 0.0f, 0.0f};
    vec3_t world;

    xform_apply_to_point(&xf, &local, &world);

    CHECK(world.x == doctest::Approx(6.0f));
    CHECK(world.y == doctest::Approx(0.0f));
    CHECK(world.z == doctest::Approx(0.0f));
}

TEST_CASE("xform_equal works correctly") {
    xform_t a;
    xform_init(&a);

    xform_t b;
    xform_init(&b);

    CHECK(xform_equal(&a, &b));
}

TEST_CASE("xform: identity and clone") {
    xform_t xf;
    xform_init(&xf);

    xform_t copy;
    xform_assign(&copy, &xf);
    CHECK(xform_equal(&xf, &copy));
}

TEST_CASE("xform: from axis-angle and roundtrip") {
    vec3_t pos = {1, 2, 3};
    vec3_t axis = {0, 1, 0};
    float radians = 3.14159f;

    xform_t xf;
    xform_init_axis_angle(&xf, &pos, &axis, radians);

    vec3_t got_pos, got_axis;
    float got_radians;
    xform_get_position(&xf, &got_pos);
    xform_get_axis_angle(&xf, &got_axis, &got_radians);

    CHECK(got_pos.x == doctest::Approx(pos.x));
    CHECK(got_pos.y == doctest::Approx(pos.y));
    CHECK(got_pos.z == doctest::Approx(pos.z));

    CHECK(got_radians == doctest::Approx(radians));
    CHECK(got_axis.y == doctest::Approx(1.0f));
}

TEST_CASE("xform: from euler and roundtrip") {
    vec3_t pos = {0, 0, 0};
    float yaw = 1.0f, pitch = 0.5f, roll = 0.25f;

    xform_t xf;
    xform_init_euler(&xf, &pos, yaw, pitch, roll, EULER_ORDER_ZYX);

    float got_yaw, got_pitch, got_roll;
    xform_get_euler(&xf, &got_yaw, &got_pitch, &got_roll, EULER_ORDER_ZYX);

    CHECK(got_yaw == doctest::Approx(yaw));
    CHECK(got_pitch == doctest::Approx(pitch));
    CHECK(got_roll == doctest::Approx(roll));
}

TEST_CASE("xform: position setter and getter") {
    xform_t xf;
    xform_init(&xf);

    vec3_t p = {5, 10, 15};
    xform_set_position(&xf, &p);

    vec3_t got;
    xform_get_position(&xf, &got);
    CHECK(got.x == doctest::Approx(5.0f));
    CHECK(got.y == doctest::Approx(10.0f));
    CHECK(got.z == doctest::Approx(15.0f));
}

TEST_CASE("xform: translation") {
    xform_t xf;
    xform_init(&xf);

    vec3_t delta = {0, 0, 5};
    xform_set_euler(&xf, 0, 3.14159f / 2.0f, 0, EULER_ORDER_ZYX); // 90 deg yaw rotate

    xform_translate(&xf, &delta);

    vec3_t pos;
    xform_get_position(&xf, &pos);
    CHECK(pos.x == doctest::Approx(0.0f).epsilon(0.01));
    CHECK(pos.y == doctest::Approx(0.0f).epsilon(0.01));
    CHECK(pos.z == doctest::Approx(5.0f).epsilon(0.01));
}

TEST_CASE("xform: translate local") {
    xform_t xf;
    xform_init(&xf);
    vec3_t delta = {0, 0, 5};
    xform_set_euler(&xf, 0, 3.14159f / 2.0f, 0, EULER_ORDER_ZYX); // 90 deg yaw rotate

    xform_translate_local(&xf, &delta);

    vec3_t pos;
    xform_get_position(&xf, &pos);
    CHECK(pos.x == doctest::Approx(5.0f).epsilon(0.01));
    CHECK(pos.y == doctest::Approx(0.0f).epsilon(0.01));
    CHECK(pos.z == doctest::Approx(0.0f).epsilon(0.01));
}

TEST_CASE("xform: apply to point and direction") {
    xform_t xf;
    xform_init(&xf);
    vec3_t move = {5, 0, 0};
    xform_translate(&xf, &move);

    vec3_t local = {1, 0, 0};
    vec3_t world = {0};
    xform_apply_to_point(&xf, &local, &world);
    CHECK(world.x == doctest::Approx(6.0f));

    vec3_t dir;
    xform_apply_to_direction(&xf, &local, &dir);
    CHECK(dir.x == doctest::Approx(1.0f));
}

TEST_CASE("xform: local translate with yaw 90deg") {
    xform_t xf;
    xform_init(&xf);

    // Yaw 90 deg rotate (Z axis base rotation)
    float yaw_deg = 90.0f;
    float yaw_rad = 3.1415926f / 2.0f;
    xform_set_euler(&xf, 0.0f, yaw_rad, 0.0f, EULER_ORDER_ZYX);

    // Local Z axis +5 should move world +X direction
    vec3_t delta_local = {0.0f, 0.0f, 5.0f};
    xform_translate_local(&xf, &delta_local);

    vec3_t pos;
    xform_get_position(&xf, &pos);

    CHECK(pos.x == doctest::Approx(5.0f).epsilon(0.01f));
    CHECK(pos.y == doctest::Approx(0.0f).epsilon(0.01f));
    CHECK(pos.z == doctest::Approx(0.0f).epsilon(0.01f));
}

TEST_CASE("quat rotation: yaw 90deg rotates +Z to -X") {
    quat_t q;
    quat_init_euler(&q, M_PI / 2.0f, 0.0f, 0.0f, EULER_ORDER_ZYX);

    vec3_t forward = {0, 0, 1}; // +Z
    vec3_t rotated;
    quat_rotate_vector(&q, &forward, &rotated);

    CHECK(rotated.x == doctest::Approx(0.0f).epsilon(0.01f));
    CHECK(rotated.y == doctest::Approx(-1.0f).epsilon(0.01f));
    CHECK(rotated.z == doctest::Approx(0.0f).epsilon(0.01f));
}

TEST_CASE("quat rotation: yaw 90deg rotates +Z to +X v1") {
    quat_t q;
    quat_init_euler(&q, 0.0f, M_PI / 2.0f, 0.0f, EULER_ORDER_ZYX); // yaw = 90 degrees

    vec3_t forward = {0, 0, 1}; // +Z
    vec3_t rotated;
    quat_rotate_vector(&q, &forward, &rotated);

    CHECK(rotated.x == doctest::Approx(1.0f).epsilon(0.01f));
    CHECK(rotated.y == doctest::Approx(0.0f).epsilon(0.01f));
    CHECK(rotated.z == doctest::Approx(0.0f).epsilon(0.01f));
}

TEST_CASE("quat rotation: yaw -90deg rotates +Z to -X v2") {
    quat_t q;
    quat_init_euler(&q, 0.0f, -M_PI / 2.0f, 0.0f, EULER_ORDER_ZYX); // yaw = -90 degrees

    vec3_t forward = {0, 0, 1}; // +Z
    vec3_t rotated;
    quat_rotate_vector(&q, &forward, &rotated);

    CHECK(rotated.x == doctest::Approx(-1.0f).epsilon(0.01f));
    CHECK(rotated.y == doctest::Approx(0.0f).epsilon(0.01f));
    CHECK(rotated.z == doctest::Approx(0.0f).epsilon(0.01f));
}

TEST_CASE("xform: translate after euler rotation affects world, not local") {
    xform_t xf;
    xform_init(&xf);

    // 1. Euler rotation: yaw 90 degrees (rotation around Z axis)
    xform_set_euler(&xf, M_PI_2, 0, 0, EULER_ORDER_ZYX);  // yaw = 90 degrees

    // 2. Translation vector (world space)
    vec3_t delta = {0, 0, 5}; // +Z direction

    // 3. Translation in world space should not depend on rotation
    xform_translate(&xf, &delta);

    // 4. Position check
    vec3_t pos;
    xform_get_position(&xf, &pos);

    CHECK(pos.x == doctest::Approx(0.0f));
    CHECK(pos.y == doctest::Approx(0.0f));
    CHECK(pos.z == doctest::Approx(5.0f));  // movement along +Z, independent of rotation
}

TEST_CASE("xform: set_euler -> get_euler roundtrip (ZYX)") {
    xform_t xf;
    xform_init(&xf);

    float yaw_in = M_PI_2;     // 90 degrees
    float pitch_in = M_PI / 4; // 45 degrees
    float roll_in = M_PI / 6;  // 30 degrees

    xform_set_euler(&xf, yaw_in, pitch_in, roll_in, EULER_ORDER_ZYX);

    float yaw_out, pitch_out, roll_out;
    xform_get_euler(&xf, &yaw_out, &pitch_out, &roll_out, EULER_ORDER_ZYX);

    CHECK(yaw_out == doctest::Approx(yaw_in).epsilon(0.01));
    CHECK(pitch_out == doctest::Approx(pitch_in).epsilon(0.01));
    CHECK(roll_out == doctest::Approx(roll_in).epsilon(0.01));
}

TEST_CASE("xform_lerp should interpolate position and rotation linearly") {
    vec3_t pos_a = {0, 0, 0};
    vec3_t axis_a = {0, 1, 0}; // Y axis
    float rad_a = 0.0f;

    vec3_t pos_b = {10, 0, 0};
    vec3_t axis_b = {0, 1, 0}; // Y axis
    float rad_b = (float)M_PI;

    xform_t a, b, mid;
    xform_init_axis_angle(&a, &pos_a, &axis_a, rad_a);
    xform_init_axis_angle(&b, &pos_b, &axis_b, rad_b);
    xform_init(&mid);

    xform_lerp(&mid, &a, &b, 0.5f);

    vec3_t mid_pos;
    xform_get_position(&mid, &mid_pos);
    vec3_t v3 = {5, 0, 0};
    CHECK(vec3_equal(&mid_pos, &v3));

    vec3_t axis;
    float rad;
    xform_get_axis_angle(&mid, &axis, &rad);
    CHECK(float_equal(rad, (float)M_PI / 2));
}

TEST_CASE("xform_slerp should interpolate position linearly and rotation via slerp") {
    vec3_t pos_a = {0, 0, 0};
    vec3_t axis_a = {0, 1, 0}; // Y axis
    float rad_a = 0.0f;

    vec3_t pos_b = {10, 0, 0};
    vec3_t axis_b = {0, 1, 0};
    float rad_b = (float)M_PI;

    xform_t a, b, mid;
    xform_init_axis_angle(&a, &pos_a, &axis_a, rad_a);
    xform_init_axis_angle(&b, &pos_b, &axis_b, rad_b);
    xform_init(&mid);

    xform_slerp(&mid, &a, &b, 0.5f);

    vec3_t mid_pos;
    xform_get_position(&mid, &mid_pos);
    vec3_t v3 = {5, 0, 0};
    CHECK(vec3_equal(&mid_pos, &v3));

    vec3_t axis;
    float rad;
    xform_get_axis_angle(&mid, &axis, &rad);
    CHECK(float_equal(rad, (float)M_PI / 2));
}

// ---------------------------------------------------------
// Helper function
// ---------------------------------------------------------
static vec3_t make_vec3(float x, float y, float z) {
    vec3_t v = {x, y, z};
    return v;
}

TEST_CASE("xform_set_position: clamp test for range") {
    xform_t xf;
    xform_init(&xf);

    // Values exceeding the range
    vec3_t pos_outside = make_vec3(
        XFORM_POS_MAX + 100.0f,
        XFORM_POS_MIN - 50.0f,
        XFORM_POS_MAX + 1.0f
    );

    xform_set_position(&xf, &pos_outside);

    vec3_t pos_after;
    xform_get_position(&xf, &pos_after);

    CHECK(pos_after.x == doctest::Approx(XFORM_POS_MAX));
    CHECK(pos_after.y == doctest::Approx(XFORM_POS_MIN));
    CHECK(pos_after.z == doctest::Approx(XFORM_POS_MAX));
}

TEST_CASE("xform_translate: range clamped after movement") {
    xform_t xf;
    xform_init(&xf);

    vec3_t start = make_vec3(XFORM_POS_MAX - 1.0f, 0.0f, 0.0f);
    xform_set_position(&xf, &start);

    // X axis +10 should clamp to MAX
    vec3_t delta = make_vec3(10.0f, 0.0f, 0.0f);
    xform_translate(&xf, &delta);

    vec3_t pos_after;
    xform_get_position(&xf, &pos_after);
    CHECK(pos_after.x == doctest::Approx(XFORM_POS_MAX));
}

TEST_CASE("xform_translate_local: range clamped after movement") {
    xform_t xf;
    xform_init(&xf);

    vec3_t start = make_vec3(0.0f, XFORM_POS_MIN + 1.0f, 0.0f);
    xform_set_position(&xf, &start);

    // Y axis -10 should clamp to MIN
    vec3_t delta = make_vec3(0.0f, -10.0f, 0.0f);
    xform_translate_local(&xf, &delta);

    vec3_t pos_after;
    xform_get_position(&xf, &pos_after);
    CHECK(pos_after.y == doctest::Approx(XFORM_POS_MIN));
}

TEST_CASE("xform_lerp: range clamped after interpolation") {
    xform_t a, b, mid;
    xform_init(&a);
    xform_init(&b);

    vec3_t pos_a = make_vec3(XFORM_POS_MIN, XFORM_POS_MIN, XFORM_POS_MIN);
    vec3_t pos_b = make_vec3(XFORM_POS_MAX + 100.0f, XFORM_POS_MAX, XFORM_POS_MAX);
    xform_set_position(&a, &pos_a);
    xform_set_position(&b, &pos_b);

    // Interpolation t=0.5
    xform_lerp(&mid, &a, &b, 0.5f);

    vec3_t pos_mid;
    xform_get_position(&mid, &pos_mid);
    CHECK(pos_mid.x <= XFORM_POS_MAX);
    CHECK(pos_mid.x >= XFORM_POS_MIN);
}
