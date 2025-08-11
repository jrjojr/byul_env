// plane.h
#ifndef GEOM_PLANE_H
#define GEOM_PLANE_H
#include "vec3.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct s_plane {
    vec3_t normal_unit;  // unit-length
    float  d;            // dot(n, x) + d = 0
} plane_t;

BYUL_API void  plane_init(plane_t* plane);

BYUL_API bool  plane_init_full(plane_t* plane, const vec3_t* normal, float d);

BYUL_API bool  plane_init_point_normal(plane_t* plane, 
    const vec3_t* point, const vec3_t* normal);

BYUL_API bool  plane_init_normal_height(plane_t* plane, 
    const vec3_t* normal, float h);

BYUL_API bool  plane_init_points(plane_t* plane, 
    const vec3_t* a, const vec3_t* b, const vec3_t* c);

BYUL_API void  plane_assign(plane_t* out, const plane_t* src);

BYUL_API float plane_signed_distance(const plane_t* p, const vec3_t* x);

BYUL_API void  plane_project(vec3_t* out, const plane_t* p, const vec3_t* x);

BYUL_API void plane_anchor_point(const plane_t* p, vec3_t* out);

BYUL_API void  plane_flip(plane_t* p);

BYUL_API void  plane_translate_along_normal(plane_t* p, float delta_h);

BYUL_API bool  plane_ray_intersect(const plane_t* p, 
    const vec3_t* ro, const vec3_t* rd,
    float* out_t, vec3_t* out_point);

BYUL_API bool  plane_segment_intersect(const plane_t* p, 
    const vec3_t* a, const vec3_t* b,
    float* out_s, vec3_t* out_point);


// 회전: 축-각(원점 기준)
BYUL_API bool plane_rotate_axis_angle_origin(
    plane_t* p, const vec3_t* axis_unit, float angle_rad);

// 회전: 축-각(피벗 기준, 피벗을 지나 회전)
BYUL_API bool plane_rotate_axis_angle_pivot(plane_t* p, 
    const vec3_t* axis_unit, float angle_rad, const vec3_t* pivot);

// 복사본으로 회전 결과 받기
BYUL_API bool plane_rotated_axis_angle_origin(plane_t* out, 
    const plane_t* in, const vec3_t* axis_unit, float angle_rad);

BYUL_API bool plane_rotated_axis_angle_pivot(plane_t* out, 
    const plane_t* in, const vec3_t* axis_unit, 
    float angle_rad, const vec3_t* pivot);

// 선택: 3x3 회전행렬 사용(열 우선 m[9] 가정: [c0 c1 c2])
BYUL_API bool plane_rotate_mat3_origin(
    plane_t* p, const float m3x3[9]);

BYUL_API bool plane_rotate_mat3_pivot(
    plane_t* p, const float m3x3[9], const vec3_t* pivot);

#ifdef __cplusplus
}
#endif
#endif  // GEOM_PLANE_H
