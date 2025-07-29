#include "doctest.h"
#include <cmath>
#include "numal.h"

TEST_CASE("quat_init / copy / equal / hash") {
    quat_t q1;
    quat_init(&q1);
    float w, x, y, z;
    quat_get(&q1, &w, &x, &y, &z);
    CHECK(w == doctest::Approx(1.0f));
    CHECK(x == doctest::Approx(0.0f));

    quat_t q2;
    quat_init_full(&q2, 1.0f, 2.0f, 3.0f, 4.0f);
    quat_get(&q2, &w, &x, &y, &z);
    CHECK(x == doctest::Approx(2.0f));

    quat_t q3;
    quat_assign(&q3, &q2);
    CHECK(quat_equal(&q3, &q2));

    unsigned int hash1 = quat_hash(&q3);
    unsigned int hash2 = quat_hash(&q2);
    CHECK(hash1 == hash2);
}

TEST_CASE("quat_init_axis_angle / to_axis_angle") {
    vec3_t axis;
    vec3_init_full(&axis, 0.0f, 1.0f, 0.0f);
    quat_t q;
    quat_init_axis_angle(&q, &axis, 3.1415926f);

    float radians;
    quat_to_axis_angle(&q, &axis, &radians);
    CHECK(axis.x == doctest::Approx(0.0f));
    CHECK(axis.y == doctest::Approx(1.0f));
    CHECK(radians == doctest::Approx(3.1415926f));
}

TEST_CASE("quat_conjugate / inverse") {
    quat_t q, conj, inv;
    quat_init_full(&q, 1.0f, 2.0f, 3.0f, 4.0f);
    quat_conjugate(&conj, &q);
    quat_inverse(&inv, &q);

    float w, x, y, z;
    float cw, cx, cy, cz;
    float iw, ix, iy, iz;
    quat_get(&q, &w, &x, &y, &z);
    quat_get(&conj, &cw, &cx, &cy, &cz);
    quat_get(&inv, &iw, &ix, &iy, &iz);

    CHECK(cw == doctest::Approx(w));
    CHECK(cx == doctest::Approx(-x));
    CHECK(ix == doctest::Approx(-x / (w*w + x*x + y*y + z*z)));
}

TEST_CASE("quat_mul") {
    quat_t a = {1.0f, 0.0f, 1.0f, 0.0f};
    quat_t b = {1.0f, 0.5f, 0.5f, 0.75f};
    quat_t out;
    quat_mul(&out, &a, &b);
    CHECK(std::isfinite(out.w));
}

TEST_CASE("quat_rotate_vector") {
    vec3_t axis = {0.0f, 0.0f, 1.0f};
    quat_t q;
    quat_init_axis_angle(&q, &axis, 3.1415926f); // 180 DEG ROTATE

    vec3_t org = {1.0f, 0.0f, 0.0f};
    vec3_t result;
    quat_rotate_vector(&q, &org, &result);
    CHECK(result.x == doctest::Approx(-1.0f));
    CHECK(result.y == doctest::Approx(0.0f));
}

TEST_CASE("quat_lerp / slerp") {
    quat_t a, b, l, s;
    quat_identity(&a);

    vec3_t axis = {0.0f, 1.0f, 0.0f};
    quat_init_axis_angle(&b, &axis, 3.1415926f); // 180 DEG ROTATE

    quat_lerp(&l, &a, &b, 0.5f);
    quat_slerp(&s, &a, &b, 0.5f);

    float len_l = std::sqrt(l.w*l.w + l.x*l.x + l.y*l.y + l.z*l.z);
    float len_s = std::sqrt(s.w*s.w + s.x*s.x + s.y*s.y + s.z*s.z);

    CHECK(len_l == doctest::Approx(1.0f));
    CHECK(len_s == doctest::Approx(1.0f));
}

TEST_CASE("quat_scale / div_scalar") {
    quat_t q = {2.0f, 4.0f, 6.0f, 8.0f};
    quat_t r;
    quat_scale(&r, &q, 0.5f);
    CHECK(r.x == doctest::Approx(2.0f));
}
