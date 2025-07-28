#include <cmath>
#include <doctest.h>
#include "dualquat.h"
#include "vec3.h"
#include "quat.h"

TEST_CASE("dualquat_identity creates unit dual quaternion") {
    dualquat_t dq;
    dualquat_identity(&dq);
    CHECK(dq.real.w == doctest::Approx(1.0f));
    CHECK(dq.real.x == doctest::Approx(0.0f));
    CHECK(dq.real.y == doctest::Approx(0.0f));
    CHECK(dq.real.z == doctest::Approx(0.0f));
    CHECK(dq.dual.w == doctest::Approx(0.0f));
    CHECK(dq.dual.x == doctest::Approx(0.0f));
    CHECK(dq.dual.y == doctest::Approx(0.0f));
    CHECK(dq.dual.z == doctest::Approx(0.0f));
}

TEST_CASE("dualquat_inverse reverses transformation") {
    dualquat_t dq, inv, res;
    vec3_t pos = {1.0f, 2.0f, 3.0f};
    quat_t qid; quat_identity(&qid);
    dualquat_init_quat_vec(&dq, &qid, &pos);
    dualquat_inverse(&inv, &dq);
    dualquat_mul(&res, &dq, &inv);

    vec3_t result;
    quat_t rot;
    dualquat_to_quat_vec(&res, &rot, &result);
    CHECK(result.x == doctest::Approx(0.0f).epsilon(1e-5));
    CHECK(result.y == doctest::Approx(0.0f).epsilon(1e-5));
    CHECK(result.z == doctest::Approx(0.0f).epsilon(1e-5));
}

TEST_CASE("dualquat_inverse reverses transformation v1") {
    dualquat_t dq, inv, id;
    vec3_t pos = {1.0f, 2.0f, 3.0f};
    quat_t qid; quat_identity(&qid);
    dualquat_init_quat_vec(&dq, &qid, &pos);
    dualquat_inverse(&inv, &dq);
    dualquat_mul(&id, &dq, &inv);
    CHECK(id.real.w == doctest::Approx(1.0f));
    CHECK(id.real.x == doctest::Approx(0.0f));
    CHECK(id.real.y == doctest::Approx(0.0f));
    CHECK(id.real.z == doctest::Approx(0.0f));
    CHECK(id.dual.w == doctest::Approx(0.0f));
    CHECK(id.dual.x == doctest::Approx(0.0f));
    CHECK(id.dual.y == doctest::Approx(0.0f));
    CHECK(id.dual.z == doctest::Approx(0.0f));
}

TEST_CASE("dualquat_align flips sign when real.w < 0") {
    dualquat_t dq, aligned;
    quat_set(&dq.real, -1.0f, 0.0f, 0.0f, 0.0f);
    quat_set(&dq.dual, 1.0f, 2.0f, 3.0f, 4.0f);
    dualquat_align(&aligned, &dq);
    CHECK(aligned.real.w == doctest::Approx(1.0f));
    CHECK(aligned.dual.x == doctest::Approx(-2.0f));
}

TEST_CASE("dualquat_blend_weighted produces midpoint") {
    dualquat_t a, b, out;
    vec3_t va = {0.0f, 0.0f, 0.0f};
    vec3_t vb = {2.0f, 0.0f, 0.0f};
    quat_t qid; quat_identity(&qid);
    dualquat_init_quat_vec(&a, &qid, &va);
    dualquat_init_quat_vec(&b, &qid, &vb);
    dualquat_blend_weighted(&out, &a, 0.5f, &b, 0.5f);
    quat_t rot; vec3_t pos;
    dualquat_to_quat_vec(&out, &rot, &pos);
    CHECK(pos.x == doctest::Approx(1.0f));
    CHECK(pos.y == doctest::Approx(0.0f));
    CHECK(pos.z == doctest::Approx(0.0f));
}

TEST_CASE("dualquat_lerp between translations") {
    dualquat_t a, b, out;
    vec3_t va = {0.0f, 0.0f, 0.0f};
    vec3_t vb = {10.0f, 0.0f, 0.0f};
    quat_t qid; quat_identity(&qid);
    dualquat_init_quat_vec(&a, &qid, &va);
    dualquat_init_quat_vec(&b, &qid, &vb);
    dualquat_lerp(&out, &a, &b, 0.25f);
    quat_t rot; vec3_t pos;
    dualquat_to_quat_vec(&out, &rot, &pos);
    CHECK(pos.x == doctest::Approx(2.5f));
    CHECK(pos.y == doctest::Approx(0.0f));
    CHECK(pos.z == doctest::Approx(0.0f));
}

TEST_CASE("dualquat_slerp between translations") {
    dualquat_t a, b, out;
    vec3_t va = {0.0f, 0.0f, 0.0f};
    vec3_t vb = {10.0f, 0.0f, 0.0f};
    quat_t qid; quat_identity(&qid);
    dualquat_init_quat_vec(&a, &qid, &va);
    dualquat_init_quat_vec(&b, &qid, &vb);
    dualquat_slerp(&out, &a, &b, 0.25f);
    quat_t rot; vec3_t pos;
    dualquat_to_quat_vec(&out, &rot, &pos);
    CHECK(pos.x == doctest::Approx(2.5f));
    CHECK(pos.y == doctest::Approx(0.0f));
    CHECK(pos.z == doctest::Approx(0.0f));
}

TEST_CASE("dualquat_apply_to_point_inplace: rotation and translation") {
    vec3_t axis = {0, 1, 0};
    quat_t rot;
    quat_init_axis_deg(&rot, &axis, 90.0f);
    vec3_t trans = {5.0f, 0.0f, 0.0f};
    dualquat_t dq;
    dualquat_init_quat_vec(&dq, &rot, &trans);
    vec3_t point = {1.0f, 0.0f, 0.0f};
    dualquat_apply_to_point_inplace(&dq, &point);
    CHECK(point.x == doctest::Approx(5.0f));
    CHECK(point.y == doctest::Approx(0.0f));
    CHECK(point.z == doctest::Approx(-1.0f));
}

TEST_CASE("dualquat_align ensures positive real.w") {
    dualquat_t dq, aligned;
    quat_set(&dq.real, -0.5f, -1.0f, 0.0f, 0.0f);
    quat_set(&dq.dual, 1.0f, 2.0f, 3.0f, 4.0f);
    dualquat_align(&aligned, &dq);
    CHECK(aligned.real.w > 0.0f);
    CHECK(aligned.real.x == doctest::Approx(1.0f));
    CHECK(aligned.dual.x == doctest::Approx(-2.0f));
}
