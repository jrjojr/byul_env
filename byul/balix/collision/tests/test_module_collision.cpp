#include "doctest.h"

#include <cmath>
#include <cfloat>

#include "collision.h"

// Small helpers ---------------------------------------------------------------
static inline vec3_t V(float x, float y, float z) {
    vec3_t v; 
    v.x = x; 
    v.y = y; 
    v.z = z; 
    return v;
}

static inline float L2(const vec3_t& a){ 
    return a.x*a.x + a.y*a.y + a.z*a.z; 
}

static inline float L (const vec3_t& a){ 
    return std::sqrt(L2(a)); 
}

static inline vec3_t add(const vec3_t& a, const vec3_t& b){ 
    return V(a.x+b.x, a.y+b.y, a.z+b.z); 
}

static inline vec3_t sub(const vec3_t& a, const vec3_t& b){ 
    return V(a.x-b.x, a.y-b.y, a.z-b.z); 
}

static inline vec3_t madd(const vec3_t& a, const vec3_t& b, float s){ 
    return V(a.x + b.x*s, a.y + b.y*s, a.z + b.z*s); 
}

static inline vec3_t project_kin(
    const vec3_t& p0, const vec3_t& v0, const vec3_t& a, float t){
    // r(t) = p0 + v0*t + 0.5*a*t^2
    return add( madd( madd(p0, v0, t), a, 0.5f*t*t), V(0,0,0) );
}


static inline void CHECK_VEC3_CLOSE(const vec3_t& a, const vec3_t& b, float eps = 1e-4f) {
    CHECK(doctest::Approx(a.x).epsilon(eps) == b.x);
    CHECK(doctest::Approx(a.y).epsilon(eps) == b.y);
    CHECK(doctest::Approx(a.z).epsilon(eps) == b.z);
}

// -----------------------------------------------------------------------------
// detect_plane_collision
// -----------------------------------------------------------------------------
TEST_CASE("detect_plane_collision - linear crossing, z=0 plane") {
    const float t_prev = 10.0f;
    const float dt = 2.0f;

    vec3_t pos_prev = V(0.0f, 0.0f, -1.0f);
    vec3_t vel_prev = V(0.0f, 0.0f,  1.0f); // so pos_curr = -1 + 1*2 = +1
    vec3_t accel    = V(0.0f, 0.0f,  0.0f);
    vec3_t pos_curr = V(0.0f, 0.0f,  1.0f);

    vec3_t plane_p  = V(0.0f, 0.0f,  0.0f);
    vec3_t plane_n  = V(0.0f, 0.0f,  1.0f);

    vec3_t hit; float thit = -1.0f;
    bool ok = detect_plane_collision(&pos_prev, &pos_curr, &vel_prev, &accel,
                                     &plane_p, &plane_n, t_prev, dt, &hit, &thit);

    CHECK(ok == true);
    CHECK(doctest::Approx(thit).epsilon(1e-5) == t_prev + dt * 0.5f); // 1.0s after t_prev
    CHECK_VEC3_CLOSE(hit, V(0.0f, 0.0f, 0.0f)); // snapped to plane
}

TEST_CASE("detect_plane_collision - accelerated crossing, zero initial velocity") {
    const float t_prev = 1.0f;
    const float dt = 2.0f;

    vec3_t pos_prev = V(0.0f, 0.0f, -1.0f);
    vec3_t vel_prev = V(0.0f, 0.0f,  0.0f);
    vec3_t accel    = V(0.0f, 0.0f,  1.0f);
    vec3_t pos_curr = V(0.0f, 0.0f, -1.0f + 0.5f * accel.z * dt * dt); // -> +1

    vec3_t plane_p  = V(0.0f, 0.0f,  0.0f);
    vec3_t plane_n  = V(0.0f, 0.0f,  1.0f);

    vec3_t hit; float thit = -1.0f;
    bool ok = detect_plane_collision(&pos_prev, &pos_curr, &vel_prev, &accel,
                                     &plane_p, &plane_n, t_prev, dt, &hit, &thit);

    CHECK(ok == true);
    // Solve -1 + 0.5 * 1 * t^2 = 0 -> t = sqrt(2)
    const float t_local = std::sqrt(2.0f);
    CHECK(doctest::Approx(thit).epsilon(1e-5) == t_prev + t_local);
    CHECK_VEC3_CLOSE(hit, V(0.0f, 0.0f, 0.0f));
}

TEST_CASE("detect_plane_collision - no hit when parallel and same side") {
    const float t_prev = 0.0f;
    const float dt = 1.0f;

    vec3_t pos_prev = V(0.0f, 0.0f, 1.0f);
    vec3_t vel_prev = V(1.0f, 0.0f, 0.0f);
    vec3_t accel    = V(0.0f, 0.0f, 0.0f);
    vec3_t pos_curr = V(1.0f, 0.0f, 1.0f);

    vec3_t plane_p  = V(0.0f, 0.0f, 0.0f);
    vec3_t plane_n  = V(0.0f, 0.0f, 1.0f);

    vec3_t hit; float thit = -1.0f;
    bool ok = detect_plane_collision(&pos_prev, &pos_curr, &vel_prev, &accel,
                                     &plane_p, &plane_n, t_prev, dt, &hit, &thit);
    CHECK(ok == false);
}

// -----------------------------------------------------------------------------
// detect_sphere_collision (static target)
// -----------------------------------------------------------------------------
TEST_CASE("detect_sphere_collision - linear approach, unit sphere at origin") {
    const float t_prev = 5.0f;
    const float dt = 3.0f;

    vec3_t p0 = V(-2.0f, 0.0f, 0.0f);
    vec3_t v0 = V( 1.0f, 0.0f, 0.0f);
    vec3_t a0 = V( 0.0f, 0.0f, 0.0f);

    vec3_t center = V(0.0f, 0.0f, 0.0f);
    float radius = 1.0f;

    vec3_t hit; float thit = -1.0f;
    bool ok = detect_sphere_collision(&p0, &v0, &a0, &center, radius, t_prev, dt, &hit, &thit);

    CHECK(ok == true);
    CHECK(doctest::Approx(thit).epsilon(1e-5) == t_prev + 1.0f);
    CHECK_VEC3_CLOSE(hit, V(-1.0f, 0.0f, 0.0f));
}

TEST_CASE("detect_sphere_collision - accelerating approach from rest") {
    const float t_prev = 0.0f;
    const float dt = 2.5f;

    vec3_t p0 = V(-4.0f, 0.0f, 0.0f);
    vec3_t v0 = V( 0.0f, 0.0f, 0.0f);
    vec3_t a0 = V( 2.0f, 0.0f, 0.0f);

    vec3_t center = V(0.0f, 0.0f, 0.0f);
    float radius = 1.0f;

    vec3_t hit; 
    float thit = -1.0f;
    bool ok = detect_sphere_collision_precise(&p0, &v0, &a0, &center, radius, t_prev, dt, &hit, &thit);

    CHECK(ok == true);
    // Solve -4 + 0.5*2*t^2 = -1 -> t^2 = 3 -> t = sqrt(3)
    const float t_local = std::sqrt(3.0f);
    CHECK(doctest::Approx(thit).epsilon(1e-5) == t_prev + t_local);
    CHECK(doctest::Approx(hit.x).epsilon(1e-4f) == -1.0f);
}

TEST_CASE("detect_sphere_collision - accelerating approach from rest, segment-based TOI per spec") {
    const float t_prev = 0.0f;
    const float dt = 2.5f;

    vec3_t p0 = V(-4.0f, 0.0f, 0.0f);
    vec3_t v0 = V( 0.0f, 0.0f, 0.0f);
    vec3_t a0 = V( 2.0f, 0.0f, 0.0f);

    vec3_t center = V(0.0f, 0.0f, 0.0f);
    float radius = 1.0f;

    // Segment-based formulation per current API contract:
    // P0 = -4, P1 = -4 + v*dt + 0.5*a*dt^2 = -4 + 6.25 = 2.25
    // Solve linear-on-segment: -4 + 6.25*s = -1 -> s = 0.48
    // So t_hit must be t_prev + s*dt = 1.2
    const float s_expected = 0.48f;
    const float t_expected = t_prev + s_expected * dt; // 1.2

    vec3_t hit; float thit = -1.0f;
    bool ok = detect_sphere_collision(&p0, &v0, &a0, &center, radius, t_prev, dt, &hit, &thit);

    CHECK(ok == true);
    CHECK(doctest::Approx(thit).epsilon(1e-5) == t_expected);

    // The implementation evaluates r(t_expected) and snaps to the sphere.
    // We only assert that the hit lies on the sphere surface (within tolerance).
    float dist = std::sqrt(hit.x*hit.x + hit.y*hit.y + hit.z*hit.z);
    CHECK(doctest::Approx(dist).epsilon(1e-4f) == radius);
}

TEST_CASE("detect_sphere_collision - miss") {
    const float t_prev = 0.0f;
    const float dt = 2.0f;

    vec3_t p0 = V(-2.0f, 0.0f, 0.0f);
    vec3_t v0 = V(-1.0f, 0.0f, 0.0f); // moving away
    vec3_t a0 = V( 0.0f, 0.0f, 0.0f);

    vec3_t center = V(0.0f, 0.0f, 0.0f);
    float radius = 1.0f;

    vec3_t hit; float thit = -1.0f;
    bool ok = detect_sphere_collision(&p0, &v0, &a0, &center, radius, t_prev, dt, &hit, &thit);

    CHECK(ok == false);
}

// -----------------------------------------------------------------------------
// detect_sphere_collision_moving (moving target)
// -----------------------------------------------------------------------------
TEST_CASE("detect_sphere_collision_moving - counter-moving along x-axis") {
    const float t_prev = 0.0f;
    const float dt = 2.0f;

    vec3_t p0 = V(0.0f, 0.0f, 0.0f);
    vec3_t v0 = V(1.0f, 0.0f, 0.0f);
    vec3_t a0 = V(0.0f, 0.0f, 0.0f);

    vec3_t c0 = V(3.0f, 0.0f, 0.0f);
    vec3_t cv = V(-1.0f, 0.0f, 0.0f);
    vec3_t ca = V(0.0f, 0.0f, 0.0f);
    float radius = 0.5f;

    // Distance |P(t) - C(t)| = |t - (3 - t)| = |2t - 3| => 0.5 at t = 1.25 (earliest)
    vec3_t hit; float thit = -1.0f;
    bool ok = detect_sphere_collision_moving(&p0, &v0, &a0, &c0, &cv, &ca,
                                             radius, t_prev, dt, &hit, &thit);

    CHECK(ok == true);
    CHECK(doctest::Approx(thit).epsilon(1e-5) == 1.25f);
    // At t = 1.25, P = 1.25, C = 1.75, contact along x at 1.25
    CHECK(doctest::Approx(hit.x).epsilon(1e-4f) == 1.25f);
    CHECK(doctest::Approx(hit.y).epsilon(1e-4f) == 0.0f);
    CHECK(doctest::Approx(hit.z).epsilon(1e-4f) == 0.0f);
}

// -----------------------------------------------------------------------------
// Triangle tests (moving / rotating variants)
// -----------------------------------------------------------------------------
TEST_CASE("detect_triangle_collision_moving - straight down through big triangle") {
    const float t_prev = 0.0f;
    const float dt = 2.0f;

    // Projectile
    vec3_t P0 = V(0.0f, 0.0f, 1.0f);
    vec3_t Vp = V(0.0f, 0.0f, -1.0f);
    vec3_t Ap = V(0.0f, 0.0f, 0.0f);

    // Large triangle on z = 0 plane
    vec3_t A0 = V(-1.0f, -1.0f, 0.0f);
    vec3_t B0 = V( 1.0f, -1.0f, 0.0f);
    vec3_t C0 = V( 0.0f,  1.0f, 0.0f);

    vec3_t Vt = V(0.0f, 0.0f, 0.0f);
    vec3_t At = V(0.0f, 0.0f, 0.0f);

    vec3_t hit; float thit = -1.0f;
    bool ok = detect_triangle_collision_moving(&P0, &Vp, &Ap,
                                               &A0, &B0, &C0,
                                               &Vt, &At,
                                               t_prev, dt, &hit, &thit);

    CHECK(ok == true);
    CHECK(doctest::Approx(thit).epsilon(1e-5) == 1.0f);
    CHECK_VEC3_CLOSE(hit, V(0.0f, 0.0f, 0.0f));
}

TEST_CASE("detect_triangle_collision_rotating - small rotation about z, still hits") {
    const float t_prev = 0.0f;
    const float dt = 2.0f;

    vec3_t P0 = V(0.0f, 0.0f, 1.0f);
    vec3_t Vp = V(0.0f, 0.0f, -1.0f);
    vec3_t Ap = V(0.0f, 0.0f, 0.0f);

    vec3_t A0 = V(-1.0f, -1.0f, 0.0f);
    vec3_t B0 = V( 1.0f, -1.0f, 0.0f);
    vec3_t C0 = V( 0.0f,  1.0f, 0.0f);

    vec3_t Vt = V(0.0f, 0.0f, 0.0f);
    vec3_t At = V(0.0f, 0.0f, 0.0f);

    vec3_t tri_center = V(0.0f, 0.0f, 0.0f);
    vec3_t omega = V(0.0f, 0.0f, 0.1f); // rad/s, small

    vec3_t hit; float thit = -1.0f;
    bool ok = detect_triangle_collision_rotating(&P0, &Vp, &Ap,
                                                 &A0, &B0, &C0,
                                                 &Vt, &At,
                                                 &tri_center, &omega,
                                                 t_prev, dt, &hit, &thit);

    CHECK(ok == true);
    CHECK(doctest::Approx(thit).epsilon(1e-5) == 1.0f);
    CHECK(doctest::Approx(hit.z).epsilon(1e-5) == 0.0f);
}

TEST_CASE("detect_triangle_collision_rotating_alpha - omega0=0, alpha about z") {
    const float t_prev = 0.0f;
    const float dt = 2.0f;

    vec3_t P0 = V(0.0f, 0.0f, 1.0f);
    vec3_t Vp = V(0.0f, 0.0f, -1.0f);
    vec3_t Ap = V(0.0f, 0.0f, 0.0f);

    vec3_t A0 = V(-1.0f, -1.0f, 0.0f);
    vec3_t B0 = V( 1.0f, -1.0f, 0.0f);
    vec3_t C0 = V( 0.0f,  1.0f, 0.0f);

    vec3_t Vt = V(0.0f, 0.0f, 0.0f);
    vec3_t At = V(0.0f, 0.0f, 0.0f);

    vec3_t tri_center = V(0.0f, 0.0f, 0.0f);
    vec3_t k_axis_unit = V(0.0f, 0.0f, 1.0f);
    vec3_t omega0 = V(0.0f, 0.0f, 0.0f);
    vec3_t alpha  = V(0.0f, 0.0f, 0.2f); // rad/s^2

    vec3_t hit; float thit = -1.0f;
    bool ok = detect_triangle_collision_rotating_alpha(&P0, &Vp, &Ap,
                                                       &A0, &B0, &C0,
                                                       &Vt, &At,
                                                       &tri_center, &k_axis_unit,
                                                       &omega0, &alpha,
                                                       t_prev, dt, &hit, &thit);

    CHECK(ok == true);
    CHECK(doctest::Approx(thit).epsilon(1e-5) == 1.0f);
    CHECK(doctest::Approx(hit.z).epsilon(1e-5) == 0.0f);
}

// --------------- test cases ---------------

TEST_CASE("moving_precise: 1D linear counter-move gives exact time"){
    // projectile
    vec3_t p0 = V(0,0,0);
    vec3_t vp = V(1,0,0);
    vec3_t ap = V(0,0,0);
    // target sphere center
    vec3_t c0 = V(3,0,0);
    vec3_t vc = V(-1,0,0);
    vec3_t ac = V(0,0,0);

    float R = 0.5f;
    float t_prev = 0.0f;
    float dt = 2.0f;

    // Relative motion along x: u0 = -3, vrel = +2, arel = 0
    // | -3 + 2 t | = 0.5 -> t = 1.25 (earliest)
    const float t_expected = 1.25f;

    vec3_t hit; float thit = -1.0f;
    bool ok = detect_sphere_collision_moving_precise(
        &p0,&vp,&ap,&c0,&vc,&ac,R,t_prev,dt,&hit,&thit);

    CHECK(ok == true);
    CHECK(doctest::Approx(thit).epsilon(1e-6) == t_expected);

    // verify hit lies on sphere at t_expected
    vec3_t c_t = project_kin(c0, vc, ac, t_expected);
    vec3_t diff = sub(hit, c_t);
    CHECK(doctest::Approx(L(diff)).epsilon(1e-5) == R);
}

TEST_CASE("moving_precise: 1D accelerating relative gives exact time"){
    // projectile
    vec3_t p0 = V(-4,0,0);
    vec3_t vp = V(0,0,0);
    vec3_t ap = V(2,0,0);
    // target
    vec3_t c0 = V(0,0,0);
    vec3_t vc = V(0,0,0);
    vec3_t ac = V(0.5f,0,0);

    float R = 1.0f;
    float t_prev = 0.0f;
    float dt = 2.3f; // window covers t=2

    // Relative: x(t) = -4 + 0*t + 0.5*(1.5)*t^2 = -4 + 0.75 t^2
    // |-4 + 0.75 t^2| = 1 -> 0.75 t^2 = 3 -> t = 2
    const float t_expected = 2.0f;

    vec3_t hit; float thit = -1.0f;
    bool ok = detect_sphere_collision_moving_precise(
        &p0,&vp,&ap,&c0,&vc,&ac,R,t_prev,dt,&hit,&thit);

    CHECK(ok == true);
    CHECK(doctest::Approx(thit).epsilon(1e-6) == t_expected);

    vec3_t c_t = project_kin(c0, vc, ac, t_expected);
    CHECK(doctest::Approx(L(sub(hit,c_t))).epsilon(1e-5) == R);
}

TEST_CASE("moving_precise: 3D oblique with curvature, invariants hold"){
    // Construct a case where the exact root is at t=1.0 by design, with lateral acceleration.
    // Build relative motion first:
    vec3_t u0 = V(1.0f, 0.0f, 0.0f);      // initial separation
    vec3_t vrel = V(-0.6f, 0.0f, 0.0f);   // toward +x origin
    vec3_t arel = V(0.2f, 0.3f, 0.0f);    // add perpendicular accel to trigger Newton

    float t_prev = 0.0f; float dt = 2.0f;
    // Choose R so that |u(1)| = R exactly
    // u(1) = u0 + vrel*1 + 0.5*arel*1^2 = (1 - 0.6 + 0.1, 0.15, 0) = (0.5, 0.15, 0)
    float R = std::sqrt(0.5f*0.5f + 0.15f*0.15f);

    // Now choose absolute states consistent with these relatives.
    // Let target be stationary for simplicity; projectile carries relative motion.
    vec3_t c0 = V(0,0,0), vc = V(0,0,0), ac = V(0,0,0);
    vec3_t p0 = add(c0, u0);              // p0 = u0
    vec3_t vp = vrel;                     // vp = vrel
    vec3_t ap = arel;                     // ap = arel

    vec3_t hit; 
    float thit = -1.0f;

    bool ok = detect_sphere_collision_moving_precise(
        &p0, &vp, &ap, &c0, &vc, &ac, R, t_prev, dt, &hit, &thit);

    CHECK(ok == true);

    // time should be close to the designed root 1.0 (Newton may not be exact in one step; give room)
    CHECK(thit >= 0.0f);
    CHECK(thit <= dt + 1e-6f);
    CHECK(doctest::Approx(thit).epsilon(1e-3) == 1.0f);

    // hit must lie on the sphere at that time
    vec3_t c_t = project_kin(c0, vc, ac, thit);
    CHECK(doctest::Approx(L(sub(hit,c_t))).epsilon(1e-5) == R);
}

TEST_CASE("moving_precise: start inside returns t_prev and snaps to surface"){
    vec3_t p0 = V(0.2f, 0.0f, 0.0f);
    vec3_t vp = V(0.0f, 0.0f, 0.0f);
    vec3_t ap = V(0.0f, 0.0f, 0.0f);

    vec3_t c0 = V(0,0,0), vc = V(0,0,0), ac = V(0,0,0);
    float R = 0.5f;

    float t_prev = 10.0f; float dt = 1.0f;

    vec3_t hit; float thit = -1.0f;
    bool ok = detect_sphere_collision_moving_precise(
        &p0,&vp,&ap,&c0,&vc,&ac,R,t_prev,dt,&hit,&thit);

    CHECK(ok == true);
    CHECK(doctest::Approx(thit).epsilon(1e-6) == t_prev);
    CHECK(doctest::Approx(L(sub(hit,c0))).epsilon(1e-5) == R);
}

TEST_CASE("moving_precise: miss returns false"){
    vec3_t p0 = V(-5,0,0);
    vec3_t vp = V(-1,0,0);
    vec3_t ap = V(0,0,0);

    vec3_t c0 = V(0,0,0), vc = V(0,0,0), ac = V(0,0,0);
    float R = 0.5f;

    float t_prev = 0.0f; float dt = 2.0f;

    vec3_t hit; float thit = -1.0f;
    bool ok = detect_sphere_collision_moving_precise(
        &p0,&vp,&ap,&c0,&vc,&ac,R,t_prev,dt,&hit,&thit);

    CHECK(ok == false);
}
