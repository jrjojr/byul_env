#include "geom.h"
#include <math.h>
#include <float.h>
#include "float_common.h"

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
