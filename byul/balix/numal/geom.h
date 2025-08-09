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
// Segment <-> Segment
// -----------------------------------------------------------------------------
BYUL_API bool vec3_segment_intersect_closest(
    const vec3_t* a1, const vec3_t* a2,
    const vec3_t* b1, const vec3_t* b2,
    vec3_intersect_result_t* out);

BYUL_API float vec3_segment_segment_distance(
    const vec3_t* a1, const vec3_t* a2,
    const vec3_t* b1, const vec3_t* b2);

// -----------------------------------------------------------------------------
// Point <-> Segment
// -----------------------------------------------------------------------------
BYUL_API float vec3_point_segment_distance(
    const vec3_t* point, const vec3_t* a, const vec3_t* b);

BYUL_API void vec3_closest_point_on_segment(
    vec3_t* out, const vec3_t* p,
    const vec3_t* a, const vec3_t* b);

/**
 * @brief Computes the intersection point between a ray and a plane.
 *
 * The ray is defined parametrically as:
 *    R(t) = ray_origin + t * ray_dir
 *
 * The plane is defined by a point on the plane and a normal vector.
 * This function solves for the scalar t where the ray intersects the plane, 
 * and computes the intersection point if it exists.
 *
 * @param[in]  ray_origin     Origin point of the ray (start of the ray).
 * @param[in]  ray_dir        Direction vector of the ray (does not need to be normalized).
 * @param[in]  plane_point    A known point on the plane.
 * @param[in]  plane_normal   The normal vector of the plane (should be normalized).
 * @param[out] out_t          (Optional) Distance along the ray to the intersection point (t).
 *                            Will be set to a positive value if intersection occurs.
 * @param[out] out_point      (Optional) The point of intersection (ray_origin + t * ray_dir).
 *
 * @return true if the ray intersects the plane in the positive direction (t >= 0), false otherwise.
 *
 * @note If the ray is parallel to the plane (denominator near zero), the function returns false.
 */
BYUL_API bool vec3_ray_plane_intersect(
    const vec3_t* ray_origin,
    const vec3_t* ray_dir,
    const vec3_t* plane_point,
    const vec3_t* plane_normal,
    float* out_t,
    vec3_t* out_point);


// -----------------------------------------------------------------------------
// Point <-> Plane
// -----------------------------------------------------------------------------
BYUL_API float vec3_point_plane_distance(
    const vec3_t* point,
    const vec3_t* plane_point,
    const vec3_t* plane_normal);

// -----------------------------------------------------------------------------
// Segment <-> Sphere / Ray <-> Sphere
// -----------------------------------------------------------------------------
BYUL_API bool vec3_segment_sphere_intersect(
    const vec3_t* a,
    const vec3_t* b,
    const vec3_t* sphere_center,
    float sphere_radius);

/**
 * @brief Computes the intersection point between a ray and a sphere.
 *
 * The ray is defined parametrically as:
 *    R(t) = ray_origin + t * ray_dir
 *
 * The sphere is defined by a center point and radius.
 * This function solves for the closest intersection point in the positive direction of the ray.
 *
 * @param[in]  ray_origin     Origin point of the ray.
 * @param[in]  ray_dir        Direction vector of the ray (should be normalized).
 * @param[in]  sphere_center  Center point of the sphere.
 * @param[in]  sphere_radius  Radius of the sphere.
 * @param[out] out_t          (Optional) Distance along the ray to the intersection point (t).
 *                            Will be the smallest positive root if two exist.
 * @param[out] out_point      (Optional) The point of intersection (ray_origin + t * ray_dir).
 *
 * @return true if the ray intersects the sphere in the forward direction (t >= 0), false otherwise.
 *
 * @note If the ray starts inside the sphere, the function returns the first exit point.
 *       If the ray misses the sphere or is tangent, returns false.
 */
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
// Sphere <-> Sphere
// -----------------------------------------------------------------------------
BYUL_API float vec3_sphere_sphere_distance(
    const vec3_t* c1, float r1,
    const vec3_t* c2, float r2);

#ifdef __cplusplus
}
#endif

#endif // GEOM_H
