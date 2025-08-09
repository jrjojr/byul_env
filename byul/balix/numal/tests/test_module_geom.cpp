#include "doctest.h"
#include "vec3.h"
#include "geom.h"
#include <math.h>

TEST_CASE("Segment-segment intersection") {
    vec3_t a1 = {0, 0, 0}, a2 = {1, 0, 0};
    vec3_t b1 = {0.5f, -1, 0}, b2 = {0.5f, 1, 0};
    vec3_intersect_result_t res;
    CHECK(vec3_segment_intersect_closest(&a1, &a2, &b1, &b2, &res));
    CHECK(doctest::Approx(res.distance) == 0.0f);
    CHECK(doctest::Approx(res.point_a.x) == 0.5f);
    CHECK(doctest::Approx(res.point_b.x) == 0.5f);
}

TEST_CASE("Ray-plane intersection") {
    vec3_t origin = {0, 0, 0}, dir = {0, 1, 0};
    vec3_t plane_point = {0, 5, 0}, plane_normal = {0, 1, 0};
    float t = 0.0f;
    vec3_t hit;
    CHECK(vec3_ray_plane_intersect(
        &origin, &dir, &plane_point, &plane_normal, &t, &hit));
    CHECK(doctest::Approx(t) == 5.0f);
    CHECK(doctest::Approx(hit.y) == 5.0f);
}

TEST_CASE("Segment-sphere intersection") {
    vec3_t a = {0, 0, 0}, b = {10, 0, 0};
    vec3_t center = {5, 1, 0};
    float radius = 2.0f;
    CHECK(vec3_segment_sphere_intersect(&a, &b, &center, radius));
}

TEST_CASE("Ray-sphere intersection") {
    vec3_t origin = {0, 0, 0}, dir = {1, 0, 0};
    vec3_t center = {5, 0, 0};
    float radius = 1.0f, t = 0.0f;
    vec3_t point;
    CHECK(vec3_ray_sphere_intersect(
        &origin, &dir, &center, radius, &t, &point));
    CHECK(t > 0.0f);
    CHECK(doctest::Approx(point.x).epsilon(0.01) == 4.0f);
}

TEST_CASE("Point in triangle") {
    vec3_t a = {0, 0, 0}, b = {1, 0, 0}, c = {0, 1, 0}, p = {0.25f, 0.25f, 0};
    CHECK(vec3_point_in_triangle(&p, &a, &b, &c));
}

TEST_CASE("Triangle area") {
    vec3_t a = {0, 0, 0}, b = {1, 0, 0}, c = {0, 1, 0};
    float area = vec3_triangle_area(&a, &b, &c);
    CHECK(doctest::Approx(area) == 0.5f);
}

TEST_CASE("Sphere-sphere distance") {
    vec3_t c1 = {0, 0, 0}, c2 = {3, 0, 0};
    float d = vec3_sphere_sphere_distance(&c1, 1.0f, &c2, 1.0f);
    CHECK(doctest::Approx(d) == 1.0f);
}
