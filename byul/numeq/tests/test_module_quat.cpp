#include "doctest.h"
#include <locale.h>
#include <iostream>

extern "C" {
#include "internal/numeq.h"
}

TEST_CASE("quat_new / free / copy / equal / hash") {
    quat_t* q1 = quat_new();
    CHECK(q1 != nullptr);
    float w, x, y, z;
    quat_get(q1, &w, &x, &y, &z);
    CHECK(w == doctest::Approx(1.0f));
    CHECK(x == doctest::Approx(0.0f));

    quat_t* q2 = quat_new_full(1.0f, 2.0f, 3.0f, 4.0f);
    quat_get(q2, &w, &x, &y, &z);
    CHECK(x == doctest::Approx(2.0f));

    quat_free(q1);
    q1 = nullptr;
    q1 = quat_copy(q2);
    CHECK(quat_equal(q1, q2));

    unsigned int hash1 = quat_hash(q1);
    unsigned int hash2 = quat_hash(q2);
    CHECK(hash1 == hash2);

    quat_free(q1);
    quat_free(q2);
}

TEST_CASE("quat_from_axis_angle / to_axis_angle") {
    quat_t* q;
    vec3_t* axis = vec3_new_full(0.0, 1.0, 0.0);
    q = quat_new_axis_angle(axis, 3.1415926f);

    float x, y, z, radians;
    quat_to_axis_angle(q, axis, &radians);
    CHECK(axis->x == doctest::Approx(0.0f));
    CHECK(axis->y == doctest::Approx(1.0f));
    CHECK(radians == doctest::Approx(3.1415926f));

    vec3_free(axis);
    quat_free(q);
}

TEST_CASE("quat_conjugate / inverse") {
    quat_t* q = quat_new_full(1.0f, 2.0f, 3.0f, 4.0f);
    quat_t* conj = quat_new();
    quat_t* inv = quat_new();
    quat_conjugate(conj, q);
    quat_inverse(inv, q);
    float w, x, y, z;
    float cw, cx, cy, cz;
    float iw, ix, iy, iz;
    quat_get(q, &w, &x, &y, &z);
    quat_get(conj, &cw, &cx, &cy, &cz);
    quat_get(inv, &iw, &ix, &iy, &iz);

    CHECK(cw == doctest::Approx(w));
    CHECK(cx == doctest::Approx(-x));
    CHECK(ix == doctest::Approx(-x / (w*w + x*x + y*y + z*z)));

    quat_free(q);
    quat_free(conj);
    quat_free(inv);
}

TEST_CASE("quat_mul") {
    quat_t a = {1.0f, 0.0f, 1.0f, 0.0f};
    quat_t b = {1.0f, 0.5f, 0.5f, 0.75f};
    quat_t out;
    quat_mul(&out, &a, &b);
    CHECK(std::isfinite(out.w));
}

TEST_CASE("quat_rotate_vector") {
    quat_t* q;

    vec3_t axis = vec3_t{0.0, 0.0, 1.0};
    q = quat_new_axis_angle(&axis, 3.1415926f); // Z축 180도 회전
    float x = 1.0f, y = 0.0f, z = 0.0f;

    vec3_t org = vec3_t{1,0,0};
    vec3_t result ;
    quat_rotate_vector(&result, q, &org);
    CHECK(result.x == doctest::Approx(-1.0f));
    CHECK(result.y == doctest::Approx( 0.0f));

    quat_free(q);
}


TEST_CASE("quat_lerp / slerp") {
    quat_t a, b, l, s;
    quat_identity(&a);

    vec3_t axis = vec3_t{0,1,0};
    quat_t* q = quat_new_axis_angle(&axis, 3.1415926f); // 180도 회전

    b = *q;           // 💡 b를 q로 초기화
    quat_free(q);     // 메모리 누수 방지

    quat_lerp(&l, &a, &b, 0.5f);
    quat_slerp(&s, &a, &b, 0.5f);

    float len_l = std::sqrt(l.w*l.w + l.x*l.x + l.y*l.y + l.z*l.z);
    float len_s = std::sqrt(s.w*s.w + s.x*s.x + s.y*s.y + s.z*s.z);

    CHECK(len_l == doctest::Approx(1.0f));  // 단위화 되었는지 확인
    CHECK(len_s == doctest::Approx(1.0f));
}


TEST_CASE("quat_scale / div_scalar") {
    quat_t q = {2.0f, 4.0f, 6.0f, 8.0f};
    quat_t r;
    quat_scale(&r, &q, 0.5f);
    CHECK(r.x == doctest::Approx(2.0f));
}
