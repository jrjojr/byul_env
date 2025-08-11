// plane.c (또는 plane.h에 static inline으로 넣어도 됨)
#include "plane.h"
#include "float_common.h"   // float_zero, FLOAT_EPSILON_TINY 등
#include <math.h>

// 내부 헬퍼: 안전 정규화 (영벡터면 false)
static inline bool plane_safe_normalize(vec3_t* n_unit)
{
    float len2 = vec3_length_sq(n_unit);
    if (len2 <= 1e-20f) return false;
    vec3_normalize(n_unit); // 있으면 vec3_normalize 사용
    return true;
}

// 기본 초기화: n = (0,1,0), d = 0
void plane_init(plane_t* plane)
{
    if (!plane) return;
    plane->normal_unit.x = 0.0f;
    plane->normal_unit.y = 1.0f;
    plane->normal_unit.z = 0.0f;
    plane->d = 0.0f;
}

// n_unit, d 로 초기화 (n은 내부에서 단위화 보장)
bool plane_init_full(plane_t* plane, const vec3_t* normal, float d)
{
    if (!plane || !normal) return false;
    plane->normal_unit = *normal;
    if (!plane_safe_normalize(&plane->normal_unit)) {
        // fallback
        plane_init(plane);
        return false;
    }
    plane->d = d;
    return true;
}

// 점-법선으로 초기화: d = -dot(n_unit, point)
bool plane_init_point_normal(
    plane_t* plane, const vec3_t* point, const vec3_t* normal)
{
    if (!plane || !point || !normal) return false;
    plane->normal_unit = *normal;
    if (!plane_safe_normalize(&plane->normal_unit)) {
        plane_init(plane);
        return false;
    }
    plane->d = -vec3_dot(&plane->normal_unit, point);
    return true;
}

// 법선과 높이 h (dot(n, x) = h) 로 초기화: d = -h
bool plane_init_normal_height(plane_t* plane, const vec3_t* normal, float h)
{
    if (!plane || !normal) return false;
    plane->normal_unit = *normal;
    if (!plane_safe_normalize(&plane->normal_unit)) {
        plane_init(plane);
        return false;
    }
    plane->d = -h;
    return true;
}

// 세 점으로 초기화: n = normalize((b-a) x (c-a)), d = -dot(n, a)
bool plane_init_points(plane_t* plane, 
    const vec3_t* a, const vec3_t* b, const vec3_t* c)
{
    if (!plane || !a || !b || !c) return false;
    vec3_t ab = *b; 
    vec3_isub(&ab, a);

    vec3_t ac = *c; 
    vec3_isub(&ac, a);

    vec3_t n; 
    vec3_cross(&n, &ab, &ac);
    
    if (!plane_safe_normalize(&n)) { plane_init(plane); return false; }
    plane->normal_unit = n;
    plane->d = -vec3_dot(&n, a);
    return true;
}

void plane_assign(plane_t* out, const plane_t* src){
    if (!out || !src) return;
    out->normal_unit = src->normal_unit;
    out->d = src->d;
}

// dot(n, x) + d
float plane_signed_distance(const plane_t* p, const vec3_t* x){
    if (!p || !x) return 0.0f;
    return vec3_dot(&p->normal_unit, x) + p->d;
}

// x_proj = x - dist * n
void plane_project(vec3_t* out, const plane_t* p, const vec3_t* x){
    if (!out || !p || !x) return;
    float dist = plane_signed_distance(p, x);
    vec3_t step = p->normal_unit; 
    vec3_iscale(&step, dist);
    *out = *x; 
    vec3_isub(out, &step);
}

// anchor point: -d * n
void plane_anchor_point(const plane_t* p, vec3_t* out){
    vec3_t x0 = {0,0,0};
    if (!p) return;
    x0 = p->normal_unit; 
    vec3_iscale(&x0, -p->d);
    *out = x0;
}

void plane_flip(plane_t* p){
    if (!p) return;
    vec3_iscale(&p->normal_unit, -1.0f);
    p->d = -p->d;
}

// +n 방향으로 delta_h 만큼 평행이동 (h = -d 증가)
void plane_translate_along_normal(plane_t* p, float delta_h){
    if (!p) return;
    p->d -= delta_h;
}

// 레이-평면 교차: R(t)=ro + t*rd, t >= 0
bool plane_ray_intersect(const plane_t* p, const vec3_t* ro, const vec3_t* rd,
                         float* out_t, vec3_t* out_point){
    if (!p || !ro || !rd) return false;
    float denom = vec3_dot(rd, &p->normal_unit);
    if (fabsf(denom) <= FLOAT_EPSILON_TINY) return false; // almost parallel
    float t = -(vec3_dot(&p->normal_unit, ro) + p->d) / denom;
    if (t < 0.0f) return false;
    if (out_t) *out_t = t;
    if (out_point){
        *out_point = *ro; 
        vec3_t step = *rd; 
        vec3_iscale(&step, t); 
        vec3_iadd(out_point, &step);
    }
    return true;
}

// 선분-평면 교차: s in [0,1]
bool plane_segment_intersect(
    const plane_t* p, const vec3_t* a, const vec3_t* b,
    float* out_s, vec3_t* out_point){
    if (!p || !a || !b) return false;
    float da = plane_signed_distance(p, a);
    float db = plane_signed_distance(p, b);
    float denom = db - da;
    // parallel or both on plane    
    if (fabsf(denom) <= FLOAT_EPSILON_TINY) return false; 
    float s = -da / denom;
    if (s < 0.0f || s > 1.0f) return false;
    if (out_s) *out_s = s;
    if (out_point){
        vec3_t ab = *b; vec3_isub(&ab, a);
        *out_point = *a; 
        vec3_t step = ab; 
        vec3_iscale(&step, s); 
        vec3_iadd(out_point, &step);
    }
    return true;
}

static inline bool safe_normalize(vec3_t* v){
    float l2 = vec3_length_sq(v);
    if (l2 <= 1e-20f) return false;
    float inv = 1.0f / sqrtf(l2);
    vec3_iscale(v, inv);
    return true;
}

// 로드리게스: v' = v*cos + (k x v)*sin + k*(k.v)*(1-cos), k는 단위
static inline vec3_t rotate_axis_angle_vec(const vec3_t* v,
                                           const vec3_t* axis_unit,
                                           float angle){
    float c = cosf(angle), s = sinf(angle);
    vec3_t k = *axis_unit;
    vec3_t kv; vec3_cross(&kv, &k, v);
    vec3_t term1 = *v;       vec3_iscale(&term1, c);
    vec3_t term2 = kv;       vec3_iscale(&term2, s);
    float kdotv = vec3_dot(&k, v);
    vec3_t term3 = k;        vec3_iscale(&term3, (1.0f - c) * kdotv);
    vec3_t out = term1;      vec3_iadd(&out, &term2); vec3_iadd(&out, &term3);
    return out;
}

static inline vec3_t mul_mat3_vec3_colmajor(const float m[9], const vec3_t* v){
    // m = [c0 c1 c2], 열 우선, out = m * v
    vec3_t out;
    out.x = m[0]*v->x + m[3]*v->y + m[6]*v->z;
    out.y = m[1]*v->x + m[4]*v->y + m[7]*v->z;
    out.z = m[2]*v->x + m[5]*v->y + m[8]*v->z;
    return out;
}

// 원점 회전: n' = R n, d 불변
bool plane_rotate_axis_angle_origin(
    plane_t* p, const vec3_t* axis_unit, float angle_rad)
{
    if (!p || !axis_unit) return false;
    vec3_t k = *axis_unit;
    if (!safe_normalize(&k)) return false;

    p->normal_unit = rotate_axis_angle_vec(&p->normal_unit, &k, angle_rad);
    // 회전행렬이 정규직교라면 길이 보존되지만, 수치 오차 보정을 위해 재정규화
    (void)safe_normalize(&p->normal_unit);
    // d 는 원점 회전에서 불변
    return true;
}

// 피벗 회전: n' = R n, x0' = pivot + R (x0 - pivot), d' = -dot(n', x0')
bool plane_rotate_axis_angle_pivot(
    plane_t* p, const vec3_t* axis_unit, float angle_rad, const vec3_t* pivot)
{
    if (!p || !axis_unit || !pivot) return false;
    vec3_t k = *axis_unit;
    if (!safe_normalize(&k)) return false;

    // 1) 법선 회전
    vec3_t n_new = rotate_axis_angle_vec(&p->normal_unit, &k, angle_rad);
    (void)safe_normalize(&n_new);

    // 2) 평면 위 임의의 점 x0 (원점에서 최근접점)
    vec3_t x0;
    plane_anchor_point(p, &x0);        // x0 = -d * n

    // 3) 피벗 기준으로 점 회전
    vec3_t x0_rel = x0; vec3_isub(&x0_rel, pivot);
    vec3_t x0_rot = rotate_axis_angle_vec(&x0_rel, &k, angle_rad);
    vec3_t x0_new = *pivot; vec3_iadd(&x0_new, &x0_rot);

    // 4) d' 갱신
    p->normal_unit = n_new;
    p->d = -vec3_dot(&p->normal_unit, &x0_new);
    return true;
}

bool plane_rotated_axis_angle_origin(
    plane_t* out, const plane_t* in, const vec3_t* axis_unit, float angle_rad)
{
    if (!out || !in) return false;
    *out = *in;
    return plane_rotate_axis_angle_origin(out, axis_unit, angle_rad);
}

bool plane_rotated_axis_angle_pivot(plane_t* out, 
    const plane_t* in, const vec3_t* axis_unit, 
    float angle_rad, const vec3_t* pivot)
{
    if (!out || !in) return false;
    *out = *in;
    return plane_rotate_axis_angle_pivot(out, axis_unit, angle_rad, pivot);
}

bool plane_rotate_mat3_origin(plane_t* p, const float m3x3[9]){
    if (!p || !m3x3) return false;
    p->normal_unit = mul_mat3_vec3_colmajor(m3x3, &p->normal_unit);
    (void)safe_normalize(&p->normal_unit);
    // d 불변
    return true;
}

bool plane_rotate_mat3_pivot(plane_t* p, 
    const float m3x3[9], const vec3_t* pivot){
    if (!p || !m3x3 || !pivot) return false;

    vec3_t n_new = mul_mat3_vec3_colmajor(m3x3, &p->normal_unit);
    (void)safe_normalize(&n_new);

    vec3_t x0;
    plane_anchor_point(p, &x0);

    vec3_t x0_rel = x0; vec3_isub(&x0_rel, pivot);
    vec3_t x0_rot  = mul_mat3_vec3_colmajor(m3x3, &x0_rel);
    vec3_t x0_new  = *pivot; vec3_iadd(&x0_new, &x0_rot);

    p->normal_unit = n_new;
    p->d = -vec3_dot(&p->normal_unit, &x0_new);
    return true;
}
