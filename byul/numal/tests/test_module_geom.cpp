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

// test_geom_all.cpp
#include "doctest.h"
#include "vec3.h"
#include "geom.h"
#include <cmath>

static inline vec3_t V(float x,float y,float z){ return vec3_t{ x,y,z }; }

// Small value-wrappers to avoid taking address of temporaries in tests
static inline float point_segment_param_v(vec3_t p, vec3_t a, vec3_t b){
    return vec3_point_segment_param(&p,&a,&b);
}
static inline float point_segment_distance_v(vec3_t p, vec3_t a, vec3_t b){
    return vec3_point_segment_distance(&p,&a,&b);
}
static inline void closest_point_on_segment_v(vec3_t* out, vec3_t p, vec3_t a, vec3_t b){
    vec3_closest_point_on_segment(out,&p,&a,&b);
}
static inline float point_plane_distance_v(vec3_t p, vec3_t p0, vec3_t n){
    return vec3_point_plane_distance(&p,&p0,&n);
}

TEST_CASE("Segment-segment intersection: closest points and zero distance"){
    vec3_t a1 = V(0,0,0), a2 = V(1,0,0);
    vec3_t b1 = V(0.5f,-1,0), b2 = V(0.5f,1,0);
    vec3_intersect_result_t res{};
    CHECK(vec3_segment_intersect_closest(&a1,&a2,&b1,&b2,&res));
    CHECK(doctest::Approx(res.distance).epsilon(1e-6) == 0.0f);
    CHECK(doctest::Approx(res.point_a.x).epsilon(1e-6) == 0.5f);
    CHECK(doctest::Approx(res.point_b.x).epsilon(1e-6) == 0.5f);
}

TEST_CASE("Segment-segment distance: skew segments"){
    vec3_t a1 = V(0,0,0), a2 = V(1,0,0);
    vec3_t b1 = V(0.5f,0.5f,1), b2 = V(0.5f,0.5f,2);
    float d = vec3_segment_segment_distance(&a1,&a2,&b1,&b2);
    CHECK(d > 0.0f);
    CHECK(doctest::Approx(d).epsilon(1e-6) == std::sqrt(0.5f*0.5f + 1.0f*1.0f)); // sqrt(1.25)
}

TEST_CASE("Point-segment: distance and closest point"){
    vec3_t A = V(0,0,0), B = V(10,0,0);
    CHECK(doctest::Approx(point_segment_param_v(V(-1,0,0),A,B)).epsilon(1e-6) == 0.0f);
    CHECK(doctest::Approx(point_segment_param_v(V( 5,0,0),A,B)).epsilon(1e-6) == 0.5f);
    CHECK(doctest::Approx(point_segment_param_v(V(12,0,0),A,B)).epsilon(1e-6) == 1.0f);
    CHECK(doctest::Approx(point_segment_param_v(V( 5,3,0),A,B)).epsilon(1e-6) == 0.5f);

    CHECK(doctest::Approx(point_segment_distance_v(V(5,3,0),A,B)).epsilon(1e-6) == 3.0f);

    vec3_t cp{};
    closest_point_on_segment_v(&cp, V(5,3,0), A, B);
    CHECK(doctest::Approx(cp.x).epsilon(1e-6) == 5.0f);
    CHECK(doctest::Approx(cp.y).epsilon(1e-6) == 0.0f);
}

TEST_CASE("Ray-plane intersection"){
    vec3_t origin = V(0,0,0), dir = V(0,1,0);
    vec3_t plane_point = V(0,5,0), plane_normal = V(0,1,0);
    float t = -1.0f; vec3_t hit{};
    CHECK(vec3_ray_plane_intersect(&origin,&dir,&plane_point,&plane_normal,&t,&hit));
    CHECK(doctest::Approx(t).epsilon(1e-6) == 5.0f);
    CHECK(doctest::Approx(hit.y).epsilon(1e-6) == 5.0f);
}

TEST_CASE("Point-plane distance and signed distance"){
    vec3_t P = V(0,7,0);
    vec3_t P0 = V(0,5,0);
    vec3_t n_unit = V(0,1,0);
    vec3_t n_nonunit = V(0,2,0);
    CHECK(doctest::Approx(point_plane_distance_v(P,P0,n_unit)).epsilon(1e-6) == 2.0f);
    CHECK(doctest::Approx(vec3_point_plane_signed_distance(&P,&P0,&n_nonunit)).epsilon(1e-6) == 2.0f);
}

TEST_CASE("Project on plane"){
    vec3_t out{}, p = V(1,2,3), p0 = V(0,0,0), n = V(0,0,1);
    vec3_project_on_plane(&out,&p,&p0,&n);
    CHECK(doctest::Approx(out.x).epsilon(1e-6) == 1.0f);
    CHECK(doctest::Approx(out.y).epsilon(1e-6) == 2.0f);
    CHECK(doctest::Approx(out.z).epsilon(1e-6) == 0.0f);
}

TEST_CASE("Segment-plane intersect in-range and parallel"){
    vec3_t A = V(0,-1,0), B = V(0,3,0);
    vec3_t planeP = V(0,1,0), planeN = V(0,1,0);
    float s = -1.0f; vec3_t hit{};
    CHECK(vec3_segment_plane_intersect(&A,&B,&planeP,&planeN,&s,&hit));
    CHECK(doctest::Approx(s).epsilon(1e-6) == 0.5f);
    CHECK(doctest::Approx(hit.y).epsilon(1e-6) == 1.0f);

    planeN = V(1,0,0);
    CHECK_FALSE(vec3_segment_plane_intersect(&A,&B,&planeP,&planeN,nullptr,nullptr));
}

TEST_CASE("Segment-sphere intersection (bool)") {
    vec3_t a = V(0,0,0), b = V(10,0,0);
    vec3_t center = V(5,1,0);
    float radius = 2.0f;
    CHECK(vec3_segment_sphere_intersect(&a,&b,&center,radius));
}

TEST_CASE("Segment-sphere intersection (ex: earliest hit s and point)"){
    vec3_t A = V(0,0,0), B = V(10,0,0);
    vec3_t Cc = V(5,1,0); float R = 2.0f;
    float s = -1.0f; vec3_t P{};
    CHECK(vec3_segment_sphere_intersect_ex(&A,&B,&Cc,R,&s,&P));
    float xexp = 5.0f - std::sqrt(R*R - 1.0f);
    CHECK(doctest::Approx(P.x).epsilon(1e-5) == xexp);
    float dx = P.x - Cc.x, dy = P.y - Cc.y, dz = P.z - Cc.z;
    CHECK(doctest::Approx(std::sqrt(dx*dx+dy*dy+dz*dz)).epsilon(1e-5) == R);
    CHECK(s >= 0.0f); CHECK(s <= 1.0f);
}

TEST_CASE("Ray-sphere intersection"){
    vec3_t origin = V(0,0,0), dir = V(1,0,0);
    vec3_t center = V(5,0,0);
    float radius = 1.0f, t = -1.0f;
    vec3_t point{};
    CHECK(vec3_ray_sphere_intersect(&origin,&dir,&center,radius,&t,&point));
    CHECK(t > 0.0f);
    CHECK(doctest::Approx(point.x).epsilon(1e-3) == 4.0f);
}

TEST_CASE("Point in triangle and triangle area"){
    vec3_t a = V(0,0,0), b = V(1,0,0), c = V(0,1,0), p = V(0.25f,0.25f,0);
    CHECK(vec3_point_in_triangle(&p,&a,&b,&c));
    float area = vec3_triangle_area(&a,&b,&c);
    CHECK(doctest::Approx(area).epsilon(1e-6) == 0.5f);
}

TEST_CASE("Sphere-sphere distance"){
    vec3_t c1 = V(0,0,0), c2 = V(3,0,0);
    float d = vec3_sphere_sphere_distance(&c1,1.0f,&c2,1.0f);
    CHECK(doctest::Approx(d).epsilon(1e-6) == 1.0f);
}

TEST_CASE("Colinearity: cosine and cross variants"){
    vec3_t a = V(1,0,0), b = V(2,0,0), c = V(1,0.1f,0);
    CHECK(vec3_colinear(&a,&b, GEOM_COLINEAR_COS));
    CHECK_FALSE(vec3_colinear(&a,&c, GEOM_COLINEAR_COS));
    CHECK(vec3_colinear_cross(&a,&b, 0.01f));
    CHECK_FALSE(vec3_colinear_cross(&a,&c, 0.01f));
}

TEST_CASE("Barycentric and inside test"){
    vec3_t A = V(0,0,0), B = V(1,0,0), C = V(0,1,0);
    vec3_t P = V(0.25f,0.25f,0), uvw{};
    vec3_barycentric(&uvw,&P,&A,&B,&C);
    CHECK(doctest::Approx(uvw.x).epsilon(1e-6) == 0.5f);
    CHECK(doctest::Approx(uvw.y).epsilon(1e-6) == 0.25f);
    CHECK(doctest::Approx(uvw.z).epsilon(1e-6) == 0.25f);
    CHECK(vec3_barycentric_inside(&uvw, 1e-6f));

    vec3_t Q = V(1.2f,-0.1f,0);
    vec3_barycentric(&uvw,&Q,&A,&B,&C);
    CHECK_FALSE(vec3_barycentric_inside(&uvw, 1e-6f));
}

TEST_CASE("Ray-triangle intersect front/back culling"){
    vec3_t A = V(0,0,0), B = V(1,0,0), C = V(0,1,0);
    vec3_t ro = V(0.2f,0.2f,-1.0f), rd = V(0,0,1);
    float t = -1.0f; vec3_t uvw{}, P{};
    CHECK(vec3_ray_triangle_intersect(&ro,&rd,&A,&B,&C,false,&t,&uvw,&P));
    CHECK(t >= 1.0f);
    CHECK(uvw.x >= -1e-5f); CHECK(uvw.y >= -1e-5f); CHECK(uvw.z >= -1e-5f);
    CHECK(doctest::Approx(uvw.x + uvw.y + uvw.z).epsilon(1e-5) == 1.0f);

    ro = V(0.2f,0.2f, 1.0f); rd = V(0,0,1);
    CHECK_FALSE(vec3_ray_triangle_intersect(&ro,&rd,&A,&B,&C,true,nullptr,nullptr,nullptr));
}

TEST_CASE("Segment-triangle intersect"){
    vec3_t A = V(0,0,0), B = V(1,0,0), C = V(0,1,0);
    vec3_t S0 = V(0.2f,0.2f,-1.0f), S1 = V(0.2f,0.2f,1.0f);
    float s = -1.0f; vec3_t P{};
    CHECK(vec3_segment_triangle_intersect(&S0,&S1,&A,&B,&C,false,&s,&P));
    CHECK(s >= 0.0f); CHECK(s <= 1.0f);
    CHECK(doctest::Approx(P.z).epsilon(1e-6) == 0.0f);
}

TEST_CASE("Triangle normal: unit and degenerate"){
    vec3_t A = V(0,0,0), B = V(1,0,0), C = V(0,1,0);
    vec3_t n{};
    CHECK(vec3_triangle_normal(&n,&A,&B,&C,true));
    CHECK(doctest::Approx(std::sqrt(n.x*n.x + n.y*n.y + n.z*n.z)).epsilon(1e-6) == 1.0f);
    CHECK(doctest::Approx(n.z).epsilon(1e-6) == 1.0f);

    vec3_t B2 = V(2,0,0), C2 = V(3,0,0);
    CHECK_FALSE(vec3_triangle_normal(&n,&A,&B2,&C2,true));
}
