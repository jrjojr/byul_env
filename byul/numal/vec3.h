#ifndef VEC3_H
#define VEC3_H

#include <stdint.h>
#include "byul_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 3차원 벡터 구조체
 * 
 * 3D 좌표 또는 방향을 표현하기 위한 구조체입니다.
 * 주로 위치(position), 이동량(delta), 방향(direction)을 나타내는 데 사용됩니다.
 */
typedef struct s_vec3 {
    float x; ///< X 좌표
    float y; ///< Y 좌표
    float z; ///< Z 좌표
} vec3_t;

/**
 * @brief 새로운 vec3 벡터를 생성합니다.
 * 
 * @param x X 좌표
 * @param y Y 좌표
 * @param z Z 좌표
 * @return vec3_t* 새로 생성된 벡터 (heap 할당됨, vec3_free로 해제 필요)
 */
BYUL_API vec3_t* vec3_new_full(float x, float y, float z);

/**
 * @brief 기본값 (0,0,0)으로 vec3를 생성합니다.
 * 
 * @return vec3_t* 기본 벡터 (heap 할당됨)
 */
BYUL_API vec3_t* vec3_new(void);

/**
 * @brief vec3 메모리 해제
 * 
 * vec3_new 또는 vec3_new_full로 생성된 벡터를 해제합니다.
 * 
 * @param v 해제할 벡터
 */
BYUL_API void vec3_free(vec3_t* v);

/**
 * @brief vec3 복사
 * 
 */
BYUL_API vec3_t* vec3_copy(const vec3_t* src);

/**
 * @brief 두 벡터의 동등성 비교
 * 
 * @param a 벡터 A
 * @param b 벡터 B
 * @return int 같으면 1, 다르면 0
 */
BYUL_API int vec3_equal(const vec3_t* a, const vec3_t* b);

/**
 * @brief vec3의 해시값을 계산합니다.
 * 
 * float 값을 정수형으로 변환해 단순 해싱합니다.
 * 
 * @param v 해시 대상 벡터
 * @return uint32_t 해시값
 */
BYUL_API uint32_t vec3_hash(const vec3_t* v);

BYUL_API void vec3_zero(vec3_t* out);

BYUL_API void vec3_add(vec3_t* out, const vec3_t* a, const vec3_t* b);

BYUL_API void vec3_sub(vec3_t* out, const vec3_t* a, const vec3_t* b);

BYUL_API void vec3_mul(vec3_t* out, const vec3_t* a, const vec3_t* b);

BYUL_API void vec3_div(vec3_t* out, const vec3_t* a, const vec3_t* b);

BYUL_API void vec3_scale(vec3_t* out, const vec3_t* a, float scalar);

BYUL_API float vec3_dot(const vec3_t* a, const vec3_t* b);

BYUL_API void vec3_cross(vec3_t* out, const vec3_t* a, const vec3_t* b);

BYUL_API float vec3_length(const vec3_t* a);

BYUL_API void vec3_normalize(vec3_t* out, const vec3_t* a);

BYUL_API float vec3_distance(const vec3_t* a, const vec3_t* b);

/**
 * @brief 선형 보간을 사용해 start와 goal 사이의 위치를 계산합니다.
 * 
 * @param out 결과 위치 (출력)
 * @param start 시작 위치
 * @param goal 목표 위치
 * @param t 보간 계수 (0.0 ~ 1.0 사이값)
 */
BYUL_API void vec3_lerp(vec3_t* out, 
    const vec3_t* start, const vec3_t* goal, float t);

/**
 * @brief vec3를 위치 정보만 담은 4x4 변환 행렬로 변환합니다.
 * 
 * 회전이 없는 단순 위치 변환 행렬을 생성합니다.
 * 
 * @param v 위치 벡터
 * @param out_mat4 16개 float 배열 (column-major 방식)
 */
BYUL_API void vec3_to_mat4(const vec3_t* v, float* out_mat4);

#ifdef __cplusplus
}
#endif

#endif // VEC3_H
