#ifndef GEOM_H
#define GEOM_H

#include "vec3.h"

// cos = 0.99 -> about 8.1 deg
// cos = 0.999 -> about 2.6 deg
// cos = 0.9999 -> about 0.81 deg
// cos = 0.99999 -> about 0.26 deg
#ifndef GEOM_COLINEAR_COS
#define GEOM_COLINEAR_COS 0.999f   // cos(theta) threshold for near colinearity
#endif

#ifdef __cplusplus
extern "C" {
#endif

BYUL_API bool vec3_colinear(const vec3_t* a, const vec3_t* b, float cos_eps);

// -----------------------------------------------------------------------------
// Collinearity: cross-product based alternative (use area ratio instead of angles; robust to noise)
// -----------------------------------------------------------------------------
/**
 * @brief Near-colinearity using cross-product magnitude: 
 * |a x b| <= tau * |a||b|.
 * @param a First vector
 * @param b Second vector
 * @param tau Sine threshold (e.g., ~0.05 -> about 2.9 deg)
 * @return true if nearly colinear
 */
BYUL_API bool vec3_colinear_cross(const vec3_t* a, const vec3_t* b, float tau);

// -----------------------------------------------------------------------------
// Projection helpers for segments and triangles
// -----------------------------------------------------------------------------
/**
 * @brief Closest parameter s in [0,1] of point P onto segment AB.
 * @return s where A + s(B-A) is closest to P.
 */
BYUL_API float vec3_point_segment_param(
    const vec3_t* p, const vec3_t* a, const vec3_t* b);

/**
 * @brief Compute barycentric coordinates (u,v,w) 
 * of point P w.r.t. triangle ABC.
 *        u+v+w=1. No clamping; can be outside.
 * @param[out] uvw  uvw.x=u, uvw.y=v, uvw.z=w
 */
BYUL_API void vec3_barycentric(vec3_t* uvw, const vec3_t* p,
    const vec3_t* a, const vec3_t* b, const vec3_t* c);

/**
 * @brief Inside test from barycentrics with tolerance.
 * @param eps Tolerance (e.g., 1e-6f)
 */
BYUL_API bool vec3_barycentric_inside(const vec3_t* uvw, float eps);

// -----------------------------------------------------------------------------
// Plane helpers
// -----------------------------------------------------------------------------
/**
 * @brief Signed distance from point to plane 
 * (n should be unit-length if you want metric units).
 */
BYUL_API float vec3_point_plane_signed_distance(
    const vec3_t* point, const vec3_t* plane_point, const vec3_t* plane_normal);

/**
 * @brief Project a point onto a plane: p - dot(p-p0,n_unit) * n_unit.
 */
BYUL_API void vec3_project_on_plane(vec3_t* out,
                                    const vec3_t* p,
                                    const vec3_t* plane_point,
                                    const vec3_t* plane_normal_unit);

/**
 * @brief Segment-plane intersection. 
 * Returns true if intersects within segment s in [0,1].
 * @param[out] out_s  Param on segment AB. out_point optional.
 */
BYUL_API bool vec3_segment_plane_intersect(
    const vec3_t* a, const vec3_t* b,
    const vec3_t* plane_point, const vec3_t* plane_normal,
    float* out_s, vec3_t* out_point);

// -----------------------------------------------------------------------------
// Ray/Segment <-> Triangle
// -----------------------------------------------------------------------------
/**
 * @brief Ray-triangle intersection (Moller-Trumbore).
 * @param cull_backface If true, backfaces are culled.
 * @param[out] out_t  Ray param t >= 0
 * @param[out] out_uv Barycentrics (u,v). w = 1-u-v
 */
BYUL_API bool vec3_ray_triangle_intersect(
    const vec3_t* ray_origin, const vec3_t* ray_dir,
    const vec3_t* a, const vec3_t* b, const vec3_t* c,
    bool cull_backface, float* out_t, vec3_t* out_uv, vec3_t* out_point);

/**
 * @brief Segment-triangle intersection via Moller-Trumbore on segment AB.
 * @param[out] out_s Segment param in [0,1]
 */
BYUL_API bool vec3_segment_triangle_intersect(
    const vec3_t* a0, const vec3_t* a1,
    const vec3_t* ta, const vec3_t* tb, const vec3_t* tc,
    bool cull_backface, float* out_s, vec3_t* out_point);

/**
 * @brief Segment-sphere test with outputs (earliest hit).
 * @param[out] out_s  Segment param s in [0,1]
 * @param[out] out_pt Intersection point
 */
BYUL_API bool vec3_segment_sphere_intersect_ex(
    const vec3_t* a, const vec3_t* b,
    const vec3_t* sphere_center, float sphere_radius,
    float* out_s, vec3_t* out_pt);

// -----------------------------------------------------------------------------
// Triangle normal
// -----------------------------------------------------------------------------
/**
 * @brief Triangle normal (unit if normalize=true; undefined if area ~ 0).
 */
BYUL_API bool vec3_triangle_normal(vec3_t* out_n,
    const vec3_t* a, const vec3_t* b, const vec3_t* c,
    bool normalize);

// -----------------------------------------------------------------------------
// Structs for intersection results
// -----------------------------------------------------------------------------

typedef struct s_vec3_intersect_result{
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
 * This function solves for the closest intersection point 
 * in the positive direction of the ray.
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
