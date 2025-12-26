// tests/test_plane.cpp
#include "doctest.h"
#include "plane.h"
#include "float_core.h"
#include <math.h>

TEST_CASE("plane: init default and full")
{
    plane_t p{};
    plane_init(&p);
    CHECK(doctest::Approx(p.normal_unit.x).epsilon(1e-6) == 0.0f);
    CHECK(doctest::Approx(p.normal_unit.y).epsilon(1e-6) == 1.0f);
    CHECK(doctest::Approx(p.normal_unit.z).epsilon(1e-6) == 0.0f);
    CHECK(doctest::Approx(p.d).epsilon(1e-6) == 0.0f);

    plane_t p2{};
    vec3_t v0 = {0, 2, 0};
    CHECK(plane_init_full(&p2, &v0, /*d*/ -3.0f));
    // normal is normalized internally
    CHECK(doctest::Approx(p2.normal_unit.x).epsilon(1e-6) == 0.0f);
    CHECK(doctest::Approx(p2.normal_unit.y).epsilon(1e-6) == 1.0f);
    CHECK(doctest::Approx(p2.normal_unit.z).epsilon(1e-6) == 0.0f);
    CHECK(doctest::Approx(p2.d).epsilon(1e-6) == -3.0f);
}

TEST_CASE("plane: point-normal and normal-height")
{
    plane_t p{};
    vec3_t v0 = {0, 1, 0};
    vec3_t v1 = {0, 3, 0};
    // y = 1 plane
    CHECK(plane_init_point_normal(&p, &v0, &v1));
    CHECK(doctest::Approx(p.normal_unit.y).epsilon(1e-6) == 1.0f);
    CHECK(doctest::Approx(p.d).epsilon(1e-6) == -1.0f);

    plane_t q{};
    CHECK(plane_init_normal_height(&q, &v0, /*h*/ 1.0f));
    CHECK(doctest::Approx(q.normal_unit.y).epsilon(1e-6) == 1.0f);
    CHECK(doctest::Approx(q.d).epsilon(1e-6) == -1.0f);
}

TEST_CASE("plane: init from three points")
{
    // Triangle on y=1 plane -> normal ~ (0,0,1), d = 0
    plane_t p{};
    vec3_t v0 = {0, 0, 0};
    vec3_t v1 = {1, 0, 0};
    vec3_t v2 = {0, 1, 0};
    CHECK(plane_init_points(&p, &v0, &v1, &v2));
    CHECK(doctest::Approx(p.normal_unit.x).epsilon(1e-6) == 0.0f);
    CHECK(doctest::Approx(p.normal_unit.y).epsilon(1e-6) == 0.0f);
    CHECK(doctest::Approx(p.normal_unit.z).epsilon(1e-6) == 1.0f);
    CHECK(doctest::Approx(p.d).epsilon(1e-6) == 0.0f);
}

TEST_CASE("plane: signed distance, project, anchor, translate, flip")
{
    plane_t p{};
    vec3_t n = {0, 1, 0};
    vec3_t vz = {0, 0, 0};

    // y = 1  =>  dot(n,x) = 1  =>  d = -1
    CHECK(plane_init_normal_height(&p, &n, 1.0f));

    // distance on y=1
    vec3_t y2 = {0, 2, 0};
    CHECK(doctest::Approx(plane_signed_distance(&p, &y2)).epsilon(1e-6) == 1.0f);
    CHECK(doctest::Approx(plane_signed_distance(&p, &vz )).epsilon(1e-6) == -1.0f);

    // project: use (1,2,3) -> (1,1,3)
    vec3_t prj{};
    vec3_t p123 = {1, 2, 3};
    plane_project(&prj, &p, &p123);
    CHECK(doctest::Approx(prj.x).epsilon(1e-6) == 1.0f);
    CHECK(doctest::Approx(prj.y).epsilon(1e-6) == 1.0f);
    CHECK(doctest::Approx(prj.z).epsilon(1e-6) == 3.0f);

    // anchor: -d*n = (0,1,0)
    vec3_t anc{};
    plane_anchor_point(&p, &anc);
    CHECK(doctest::Approx(anc.x).epsilon(1e-6) == 0.0f);
    CHECK(doctest::Approx(anc.y).epsilon(1e-6) == 1.0f);
    CHECK(doctest::Approx(anc.z).epsilon(1e-6) == 0.0f);

    // translate along +n by +2 -> y = 3 -> d = -3
    plane_translate_along_normal(&p, 2.0f);
    CHECK(doctest::Approx(p.d).epsilon(1e-6) == -3.0f);

    // on-plane point for y=3: (1,3,3) => distance 0
    vec3_t on_y3 = {1, 3, 3};
    CHECK(doctest::Approx(plane_signed_distance(&p, &on_y3)).epsilon(1e-6) == 0.0f);
    // below/above checks
    vec3_t y4 = {0, 4, 0};
    CHECK(doctest::Approx(plane_signed_distance(&p, &y2)).epsilon(1e-6) == -1.0f);
    CHECK(doctest::Approx(plane_signed_distance(&p, &y4)).epsilon(1e-6) ==  1.0f);

    // flip -> n -> -n, d -> -d. Signs invert.
    plane_flip(&p);
    CHECK(doctest::Approx(p.normal_unit.y).epsilon(1e-6) == -1.0f);
    CHECK(doctest::Approx(p.d).epsilon(1e-6) ==  3.0f);
    // distances flip sign compared to above
    CHECK(doctest::Approx(plane_signed_distance(&p, &y2)).epsilon(1e-6) ==  1.0f);
    CHECK(doctest::Approx(plane_signed_distance(&p, &y4)).epsilon(1e-6) == -1.0f);
}

TEST_CASE("plane: ray and segment intersection")
{
    // Ground plane y=0
    plane_t g{};
    vec3_t v0 = {0, 1, 0};
    vec3_t v1 = {0, -1, 0};
    CHECK(plane_init_normal_height(&g, &v0, /*h*/ 0.0f));

    // Ray from (0,1,0) toward -y
    float t = -1.0f;
    vec3_t hit{};
    CHECK(plane_ray_intersect(&g, &v0, &v1, &t, &hit));
    CHECK(doctest::Approx(t).epsilon(1e-6) == 1.0f);
    CHECK(doctest::Approx(hit.x).epsilon(1e-6) == 0.0f);
    CHECK(doctest::Approx(hit.y).epsilon(1e-6) == 0.0f);
    CHECK(doctest::Approx(hit.z).epsilon(1e-6) == 0.0f);

    // Segment from (0,1,0) to (0,-1,0)
    float s = -1.0f;
    vec3_t pt{};
    CHECK(plane_segment_intersect(&g, &v0, &v1, &s, &pt));
    CHECK(doctest::Approx(s).epsilon(1e-6) == 0.5f);
    CHECK(doctest::Approx(pt.y).epsilon(1e-6) == 0.0f);
}

TEST_CASE("plane: rotate axis-angle around origin (d invariant)")
{
    // Start with y=0 plane -> n=(0,1,0), d=0
    plane_t p{};
    plane_init(&p);

    vec3_t v0 = {1, 0, 0};
    // Rotate about +X by 90deg -> n' = (0,0,1), plane z=0, d stays 0
    CHECK(plane_rotate_axis_angle_origin(&p, &v0, float(M_PI_2)));
    CHECK(doctest::Approx(p.normal_unit.x).epsilon(1e-6) == 0.0f);
    CHECK(doctest::Approx(p.normal_unit.y).epsilon(1e-6) == 0.0f);
    CHECK(doctest::Approx(p.normal_unit.z).epsilon(1e-6) == 1.0f);
    CHECK(doctest::Approx(p.d).epsilon(1e-6) == 0.0f);
}

TEST_CASE("plane: rotate axis-angle around pivot (d updates)")
{
    // Start with y=1 plane: n=(0,1,0), d=-1
    plane_t p{};
    vec3_t v0 = {0, 1, 0};
    vec3_t v1 = {1, 0, 0};
    vec3_t v2 = {0, 0, 0};
    CHECK(plane_init_normal_height(&p, &v0, /*h*/ 1.0f));

    // Rotate about +X by 90deg around origin.
    // Anchor point (0,1,0) -> (0,0,1), n' = (0,0,1), so z = 1 -> d' = -1
    CHECK(plane_rotate_axis_angle_pivot(&p, &v1, float(M_PI_2), &v2));
    CHECK(doctest::Approx(p.normal_unit.x).epsilon(1e-6) == 0.0f);
    CHECK(doctest::Approx(p.normal_unit.y).epsilon(1e-6) == 0.0f);
    CHECK(doctest::Approx(p.normal_unit.z).epsilon(1e-6) == 1.0f);
    CHECK(doctest::Approx(p.d).epsilon(1e-6) == -1.0f);
}

TEST_CASE("plane: rotate using 3x3 matrix (column-major)")
{
    // y=0 plane
    plane_t p{};
    plane_init(&p);

    // Rx(90deg) column-major: columns are images of basis i,j,k
    // c0=(1,0,0), c1=(0,cos,sin), c2=(0,-sin,cos)
    const float c = 0.0f, s = 1.0f;
    const float Rx90_colmajor[9] = {
        1, 0, 0,
        0, c, s,
        0,-s, c
    };

    CHECK(plane_rotate_mat3_origin(&p, Rx90_colmajor));
    CHECK(doctest::Approx(p.normal_unit.x).epsilon(1e-6) == 0.0f);
    CHECK(doctest::Approx(p.normal_unit.y).epsilon(1e-6) == 0.0f);
    CHECK(doctest::Approx(p.normal_unit.z).epsilon(1e-6) == 1.0f);
    CHECK(doctest::Approx(p.d).epsilon(1e-6) == 0.0f);

    // Pivoted version: start from y=1, expect z=1 after rotation
    plane_t q{};
    vec3_t v0 = {0, 1, 0};
    vec3_t v1 = {0, 0, 0};
    CHECK(plane_init_normal_height(&q, &v0, 1.0f));
    CHECK(plane_rotate_mat3_pivot(&q, Rx90_colmajor, &v1));
    CHECK(doctest::Approx(q.normal_unit.z).epsilon(1e-6) == 1.0f);
    CHECK(doctest::Approx(q.d).epsilon(1e-6) == -1.0f);
}
