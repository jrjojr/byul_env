#include "geom.h"
#include <math.h>
#include <float.h>
#include "float_core.h"

bool vec3_colinear(const vec3_t* a, const vec3_t* b, float cos_eps) {
    float la2 = vec3_length_sq(a); 
    float lb2 = vec3_length_sq(b);

    // treat zeros as colinear to allow 1D with the other axis
    if (la2 <= VEC3_ABS_EPS_LEN2 || lb2 <= VEC3_ABS_EPS_LEN2) 
        return true;

    float dotv = vec3_dot(a, b);
    float cosang = dotv / sqrtf(la2 * lb2);

    return fabsf(cosang) >= cos_eps;
}

// small helpers
static inline float clamp01f(float x){ return x < 0.0f ? 0.0f : (x > 1.0f ? 1.0f : x); }

// -----------------------------------------------------------------------------
// Colinearity (cross-product based)
// -----------------------------------------------------------------------------
bool vec3_colinear_cross(const vec3_t* a, const vec3_t* b, float tau){
    if (!a || !b) return true;
    float la2 = vec3_length_sq(a);
    float lb2 = vec3_length_sq(b);
    if (la2 <= VEC3_ABS_EPS_LEN2 || lb2 <= VEC3_ABS_EPS_LEN2) return true; // treat zeros as colinear
    vec3_t c; vec3_cross(&c, a, b);
    float lhs2 = vec3_length_sq(&c);           // |a x b|^2
    float tau2 = tau * tau;
    float rhs2 = tau2 * la2 * lb2;             // (tau |a||b|)^2
    return lhs2 <= rhs2;
}

// -----------------------------------------------------------------------------
// Projection helpers for segments and triangles
// -----------------------------------------------------------------------------
float vec3_point_segment_param(const vec3_t* p, const vec3_t* a, const vec3_t* b){
    vec3_t ab, ap;
    vec3_sub(&ab, b, a);
    float ab2 = vec3_length_sq(&ab);
    if (ab2 <= VEC3_ABS_EPS_LEN2) return 0.0f; // degenerate segment
    vec3_sub(&ap, p, a);
    float s = vec3_dot(&ap, &ab) / ab2;
    return clamp01f(s);
}

void vec3_barycentric(vec3_t* uvw, const vec3_t* p,
                      const vec3_t* a, const vec3_t* b, const vec3_t* c){
    vec3_t v0, v1, v2;
    vec3_sub(&v0, b, a);
    vec3_sub(&v1, c, a);
    vec3_sub(&v2, p, a);

    float d00 = vec3_dot(&v0, &v0);
    float d01 = vec3_dot(&v0, &v1);
    float d11 = vec3_dot(&v1, &v1);
    float d20 = vec3_dot(&v2, &v0);
    float d21 = vec3_dot(&v2, &v1);

    float denom = d00 * d11 - d01 * d01;
    if (fabsf(denom) <= FLOAT_EPSILON_TINY) {
        // Degenerate triangle -> pick A
        uvw->x = 1.0f; uvw->y = 0.0f; uvw->z = 0.0f;
        return;
    }
    float inv = 1.0f / denom;
    float v = (d11 * d20 - d01 * d21) * inv;
    float w = (d00 * d21 - d01 * d20) * inv;
    float u = 1.0f - v - w;
    uvw->x = u; uvw->y = v; uvw->z = w;
}

bool vec3_barycentric_inside(const vec3_t* uvw, float eps){
    // inside if u>=-eps, v>=-eps, w>=-eps and u+v+w ~= 1
    if (uvw->x < -eps || uvw->y < -eps || uvw->z < -eps) return false;
    float s = uvw->x + uvw->y + uvw->z;
    return fabsf(s - 1.0f) <= fmaxf(eps, FLOAT_EPSILON);
}

// -----------------------------------------------------------------------------
// Plane helpers
// -----------------------------------------------------------------------------
float vec3_point_plane_signed_distance(
    const vec3_t* point, const vec3_t* plane_point, const vec3_t* plane_normal){
    vec3_t r; vec3_sub(&r, point, plane_point);
    float n2 = vec3_length_sq(plane_normal);
    if (n2 <= FLOAT_EPSILON_TINY) return 0.0f;
    float invn = 1.0f / sqrtf(n2);
    vec3_t nunit = *plane_normal; vec3_iscale(&nunit, invn);
    return vec3_dot(&r, &nunit);
}

void vec3_project_on_plane(vec3_t* out,
                           const vec3_t* p,
                           const vec3_t* plane_point,
                           const vec3_t* plane_normal_unit){
    vec3_t r; vec3_sub(&r, p, plane_point);
    float d = vec3_dot(&r, plane_normal_unit);
    *out = *p;
    vec3_t off = *plane_normal_unit; vec3_iscale(&off, d);
    vec3_isub(out, &off);
}

bool vec3_segment_plane_intersect(
    const vec3_t* a, const vec3_t* b,
    const vec3_t* plane_point, const vec3_t* plane_normal,
    float* out_s, vec3_t* out_point){
    vec3_t ab; vec3_sub(&ab, b, a);
    float denom = vec3_dot(plane_normal, &ab);
    if (fabsf(denom) <= FLOAT_EPSILON_TINY) return false; // parallel

    vec3_t ap0; vec3_sub(&ap0, plane_point, a);
    float s = vec3_dot(plane_normal, &ap0) / denom;
    if (s < 0.0f || s > 1.0f) return false;

    if (out_s) *out_s = s;
    if (out_point){
        *out_point = *a;
        vec3_t step = ab; vec3_iscale(&step, s);
        vec3_iadd(out_point, &step);
    }
    return true;
}

// -----------------------------------------------------------------------------
// Ray/Segment <-> Triangle (Moller-Trumbore)
// -----------------------------------------------------------------------------
bool vec3_ray_triangle_intersect(
    const vec3_t* ray_origin, const vec3_t* ray_dir,
    const vec3_t* a, const vec3_t* b, const vec3_t* c,
    bool cull_backface, float* out_t, vec3_t* out_uv, vec3_t* out_point){
    const float EPS = FLOAT_EPSILON_TINY;

    vec3_t e1, e2;
    vec3_sub(&e1, b, a);
    vec3_sub(&e2, c, a);

    vec3_t pvec; vec3_cross(&pvec, ray_dir, &e2);
    float det = vec3_dot(&e1, &pvec);

    if (cull_backface){
        if (det <= EPS) return false;
        float invDet = 1.0f / det;

        vec3_t tvec; vec3_sub(&tvec, ray_origin, a);
        float u = vec3_dot(&tvec, &pvec) * invDet;
        if (u < 0.0f || u > 1.0f) return false;

        vec3_t qvec; vec3_cross(&qvec, &tvec, &e1);
        float v = vec3_dot(ray_dir, &qvec) * invDet;
        if (v < 0.0f || u + v > 1.0f) return false;

        float t = vec3_dot(&e2, &qvec) * invDet;
        if (t < 0.0f) return false;

        if (out_t)  *out_t = t;
        if (out_uv) { out_uv->x = u; out_uv->y = v; out_uv->z = 1.0f - u - v; }
        if (out_point){
            *out_point = *ray_origin;
            vec3_t step = *ray_dir; vec3_iscale(&step, t);
            vec3_iadd(out_point, &step);
        }
        return true;
    }else{
        if (fabsf(det) <= EPS) return false;
        float invDet = 1.0f / det;

        vec3_t tvec; vec3_sub(&tvec, ray_origin, a);
        float u = vec3_dot(&tvec, &pvec) * invDet;
        if (u < -EPS || u > 1.0f + EPS) return false;

        vec3_t qvec; vec3_cross(&qvec, &tvec, &e1);
        float v = vec3_dot(ray_dir, &qvec) * invDet;
        if (v < -EPS || u + v > 1.0f + EPS) return false;

        float t = vec3_dot(&e2, &qvec) * invDet;
        if (t < 0.0f) return false;

        if (out_t)  *out_t = t;
        if (out_uv) { out_uv->x = u; out_uv->y = v; out_uv->z = 1.0f - u - v; }
        if (out_point){
            *out_point = *ray_origin;
            vec3_t step = *ray_dir; vec3_iscale(&step, t);
            vec3_iadd(out_point, &step);
        }
        return true;
    }
}

bool vec3_segment_triangle_intersect(
    const vec3_t* a0, const vec3_t* a1,
    const vec3_t* ta, const vec3_t* tb, const vec3_t* tc,
    bool cull_backface, float* out_s, vec3_t* out_point){
    vec3_t d; vec3_sub(&d, a1, a0);
    float t; vec3_t uvw, ip;
    bool ok = vec3_ray_triangle_intersect(a0, &d, ta, tb, tc, cull_backface, &t, &uvw, &ip);
    if (!ok) return false;
    // segment param s in [0,1] if t between 0..1 along d
    float d2 = vec3_length_sq(&d);
    if (d2 <= VEC3_ABS_EPS_LEN2) return false;
    // Here, t is already along 'd' (not normalized). s = t / |d|
    float len = sqrtf(d2);
    float s = t / len;
    if (s < 0.0f || s > 1.0f) return false;
    if (out_s) *out_s = s;
    if (out_point) *out_point = ip;
    return true;
}

// -----------------------------------------------------------------------------
// Sphere helpers (segment-sphere with outputs)
// -----------------------------------------------------------------------------
bool vec3_segment_sphere_intersect_ex(
    const vec3_t* a, const vec3_t* b,
    const vec3_t* sphere_center, float sphere_radius,
    float* out_s, vec3_t* out_pt){
    vec3_t d; vec3_sub(&d, b, a);            // segment direction (not normalized)
    vec3_t m; vec3_sub(&m, a, sphere_center);

    float A = vec3_dot(&d, &d);
    if (A <= VEC3_ABS_EPS_LEN2) return false; // degenerate segment

    float B = 2.0f * vec3_dot(&m, &d);
    float C = vec3_dot(&m, &m) - sphere_radius * sphere_radius;

    // If start inside, treat as no "entry" intersection for this API.
    if (C <= 0.0f) return false;

    float disc = B*B - 4.0f*A*C;
    if (disc <= 0.0f) return false; // tangent or miss -> false per our ray policy

    float sqrt_disc = sqrtf(disc);
    float inv2A = 0.5f / A;

    float s0 = (-B - sqrt_disc) * inv2A;
    float s1 = (-B + sqrt_disc) * inv2A;

    // earliest s in [0,1]
    float s = INFINITY;
    if (s0 >= 0.0f && s0 <= 1.0f) s = s0;
    if (s1 >= 0.0f && s1 <= 1.0f) s = fminf(s, s1);
    if (!isfinite(s)) return false;

    if (out_s) *out_s = s;
    if (out_pt){
        *out_pt = *a;
        vec3_t step = d; vec3_iscale(&step, s);
        vec3_iadd(out_pt, &step);
    }
    return true;
}

// -----------------------------------------------------------------------------
// Triangle normal
// -----------------------------------------------------------------------------
bool vec3_triangle_normal(vec3_t* out_n,
                          const vec3_t* a, const vec3_t* b, const vec3_t* c,
                          bool normalize){
    vec3_t e1, e2;
    vec3_sub(&e1, b, a);
    vec3_sub(&e2, c, a);
    vec3_cross(out_n, &e1, &e2);
    float n2 = vec3_length_sq(out_n);
    if (n2 <= FLOAT_EPSILON_TINY) return false;
    if (normalize){
        float inv = 1.0f / sqrtf(n2);
        vec3_iscale(out_n, inv);
    }
    return true;
}

bool vec3_segment_intersect_closest(
    const vec3_t* a1, const vec3_t* a2,
    const vec3_t* b1, const vec3_t* b2,
    vec3_intersect_result_t* out)
{
    vec3_t p = *a1, q = *b1;
    vec3_t d1, d2, r;
    vec3_sub(&d1, a2, a1);
    vec3_sub(&d2, b2, b1);
    vec3_sub(&r, a1, b1);

    float a = vec3_dot(&d1, &d1);
    float e = vec3_dot(&d2, &d2);
    float f = vec3_dot(&d2, &r);

    float s, t;
    float c = vec3_dot(&d1, &r);
    float b = vec3_dot(&d1, &d2);
    float denom = a * e - b * b;

    if (denom != 0.0f)
        s = (b * f - c * e) / denom;
    else
        s = 0.0f;

    s = fmaxf(0.0f, fminf(1.0f, s));
    t = (b * s + f) / e;
    t = fmaxf(0.0f, fminf(1.0f, t));

    vec3_madd(&out->point_a, a1, &d1, s);
    vec3_madd(&out->point_b, b1, &d2, t);
    vec3_t diff;
    vec3_sub(&diff, &out->point_a, &out->point_b);
    out->distance = vec3_length(&diff);
    out->intersect = (out->distance < FLOAT_EPSILON);

    return out->intersect;
}

float vec3_segment_segment_distance(
    const vec3_t* a1, const vec3_t* a2,
    const vec3_t* b1, const vec3_t* b2)
{
    vec3_intersect_result_t res;
    vec3_segment_intersect_closest(a1, a2, b1, b2, &res);
    return res.distance;
}

float vec3_point_segment_distance(
    const vec3_t* p, const vec3_t* a, const vec3_t* b)
{
    vec3_t ab, ap;
    vec3_sub(&ab, b, a);
    vec3_sub(&ap, p, a);
    float t = vec3_dot(&ap, &ab) / vec3_dot(&ab, &ab);
    t = fmaxf(0.0f, fminf(1.0f, t));
    vec3_t proj;
    vec3_madd(&proj, a, &ab, t);
    vec3_t diff;
    vec3_sub(&diff, p, &proj);
    return vec3_length(&diff);
}

void vec3_closest_point_on_segment(
    vec3_t* out, const vec3_t* p,
    const vec3_t* a, const vec3_t* b)
{
    vec3_t ab, ap;
    vec3_sub(&ab, b, a);
    vec3_sub(&ap, p, a);
    float t = vec3_dot(&ap, &ab) / vec3_dot(&ab, &ab);
    t = fmaxf(0.0f, fminf(1.0f, t));
    vec3_madd(out, a, &ab, t);
}

bool vec3_ray_plane_intersect(
    const vec3_t* ray_origin,
    const vec3_t* ray_dir,
    const vec3_t* plane_point,
    const vec3_t* plane_normal,
    float* out_t,
    vec3_t* out_point)
{
    float denom = vec3_dot(ray_dir, plane_normal);
    if (fabsf(denom) < FLOAT_EPSILON) return false;

    vec3_t diff;
    vec3_sub(&diff, plane_point, ray_origin);
    float t = vec3_dot(&diff, plane_normal) / denom;
    if (t < 0.0f) return false;

    if (out_t) *out_t = t;
    if (out_point) vec3_madd(out_point, ray_origin, ray_dir, t);
    return true;
}

float vec3_point_plane_distance(
    const vec3_t* point,
    const vec3_t* plane_point,
    const vec3_t* plane_normal)
{
    vec3_t diff;
    vec3_sub(&diff, point, plane_point);
    return fabsf(vec3_dot(&diff, plane_normal));
}

bool vec3_segment_sphere_intersect(
    const vec3_t* a,
    const vec3_t* b,
    const vec3_t* center,
    float radius)
{
    vec3_t ab;
    vec3_sub(&ab, b, a);

    vec3_t ac;
    vec3_sub(&ac, center, a);

    float t = vec3_dot(&ac, &ab) / vec3_dot(&ab, &ab);
    t = fmaxf(0.0f, fminf(1.0f, t));

    vec3_t closest;
    vec3_madd(&closest, a, &ab, t);

    vec3_t diff;
    vec3_sub(&diff, &closest, center);
    return vec3_length_sq(&diff) <= radius * radius;
}

bool vec3_ray_sphere_intersect(
    const vec3_t* ray_origin,
    const vec3_t* ray_dir,
    const vec3_t* center,
    float radius,
    float* out_t,
    vec3_t* out_point)
{
    vec3_t oc;
    vec3_sub(&oc, ray_origin, center);
    float b = 2.0f * vec3_dot(&oc, ray_dir);
    float c = vec3_dot(&oc, &oc) - radius * radius;
    float discriminant = b * b - 4.0f * c;

    if (discriminant < 0.0f) return false;

    float t = (-b - sqrtf(discriminant)) * 0.5f;
    if (t < 0.0f) t = (-b + sqrtf(discriminant)) * 0.5f;
    if (t < 0.0f) return false;

    if (out_t) *out_t = t;
    if (out_point) vec3_madd(out_point, ray_origin, ray_dir, t);
    return true;
}

bool vec3_point_in_triangle(
    const vec3_t* p,
    const vec3_t* a,
    const vec3_t* b,
    const vec3_t* c)
{
    vec3_t v0, v1, v2;
    vec3_sub(&v0, c, a);
    vec3_sub(&v1, b, a);
    vec3_sub(&v2, p, a);

    float d00 = vec3_dot(&v0, &v0);
    float d01 = vec3_dot(&v0, &v1);
    float d11 = vec3_dot(&v1, &v1);
    float d20 = vec3_dot(&v2, &v0);
    float d21 = vec3_dot(&v2, &v1);

    float denom = d00 * d11 - d01 * d01;
    if (denom == 0.0f) return false;

    float v = (d11 * d20 - d01 * d21) / denom;
    float w = (d00 * d21 - d01 * d20) / denom;
    float u = 1.0f - v - w;

    return (u >= 0.0f && v >= 0.0f && w >= 0.0f);
}

float vec3_triangle_area(
    const vec3_t* a,
    const vec3_t* b,
    const vec3_t* c)
{
    vec3_t ab, ac, cross;
    vec3_sub(&ab, b, a);
    vec3_sub(&ac, c, a);
    vec3_cross(&cross, &ab, &ac);
    return 0.5f * vec3_length(&cross);
}

float vec3_sphere_sphere_distance(
    const vec3_t* c1, float r1,
    const vec3_t* c2, float r2)
{
    float center_dist = vec3_distance(c1, c2);
    return fmaxf(0.0f, center_dist - (r1 + r2));
}
