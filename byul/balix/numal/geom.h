#ifndef GEOM_H
#define GEOM_H

#include "vec3.h"

#ifdef __cplusplus
extern "C" {
#endif

// -----------------------------------------------------------------------------
// Structs for intersection results
// -----------------------------------------------------------------------------

typedef struct {
    vec3_t point_a;
    vec3_t point_b;
    float distance;
    bool intersect;
} vec3_intersect_result_t;

// -----------------------------------------------------------------------------
// Segment ↔ Segment
// -----------------------------------------------------------------------------
BYUL_API bool vec3_segment_intersect_closest(
    const vec3_t* a1, const vec3_t* a2,
    const vec3_t* b1, const vec3_t* b2,
    vec3_intersect_result_t* out);

BYUL_API float vec3_segment_segment_distance(
    const vec3_t* a1, const vec3_t* a2,
    const vec3_t* b1, const vec3_t* b2);

// -----------------------------------------------------------------------------
// Point ↔ Segment
// -----------------------------------------------------------------------------
BYUL_API float vec3_point_segment_distance(
    const vec3_t* point, const vec3_t* a, const vec3_t* b);

BYUL_API void vec3_closest_point_on_segment(
    vec3_t* out, const vec3_t* p,
    const vec3_t* a, const vec3_t* b);

// -----------------------------------------------------------------------------
// Ray ↔ Plane
// -----------------------------------------------------------------------------
BYUL_API bool vec3_ray_plane_intersect(
    const vec3_t* ray_origin,
    const vec3_t* ray_dir,
    const vec3_t* plane_point,
    const vec3_t* plane_normal,
    float* out_t,
    vec3_t* out_point);

// -----------------------------------------------------------------------------
// Point ↔ Plane
// -----------------------------------------------------------------------------
BYUL_API float vec3_point_plane_distance(
    const vec3_t* point,
    const vec3_t* plane_point,
    const vec3_t* plane_normal);

// -----------------------------------------------------------------------------
// Segment ↔ Sphere / Ray ↔ Sphere
// -----------------------------------------------------------------------------
BYUL_API bool vec3_segment_sphere_intersect(
    const vec3_t* a,
    const vec3_t* b,
    const vec3_t* sphere_center,
    float sphere_radius);

BYUL_API bool vec3_ray_sphere_intersect(
    const vec3_t* ray_origin,
    const vec3_t* ray_dir,
    const vec3_t* sphere_center,
    float sphere_radius,
    float* out_t,
    vec3_t* out_point);

// -----------------------------------------------------------------------------
// Triangle
// -----------------------------------------------------------------------------
BYUL_API bool vec3_point_in_triangle(
    const vec3_t* p,
    const vec3_t* a,
    const vec3_t* b,
    const vec3_t* c);

BYUL_API float vec3_triangle_area(
    const vec3_t* a,
    const vec3_t* b,
    const vec3_t* c);

// -----------------------------------------------------------------------------
// Sphere ↔ Sphere
// -----------------------------------------------------------------------------
BYUL_API float vec3_sphere_sphere_distance(
    const vec3_t* c1, float r1,
    const vec3_t* c2, float r2);

#ifdef __cplusplus
}
#endif

#endif // GEOM_H
