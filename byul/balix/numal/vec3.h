#ifndef VEC3_H
#define VEC3_H

#include <stdint.h>
#include "byul_config.h"

#ifdef __cplusplus
extern "C" {
#endif
#include <cstddef>

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

BYUL_API void vec3_init_full(vec3_t* out, float x, float y, float z);

/**
 * @brief 기본값 (0,0,0)으로 vec3를 생성합니다.
 * 
 */
BYUL_API void vec3_init(vec3_t* out);

/**
 * @brief vec3 복사
 * 
 */
BYUL_API void vec3_assign(vec3_t* out, const vec3_t* src);

/**
 * @brief 두 벡터의 동등성 비교
 * 
 * @param a 벡터 A
 * @param b 벡터 B
 */
BYUL_API bool vec3_equal(const vec3_t* a, const vec3_t* b);

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

BYUL_API void vec3_negate(vec3_t* out, const vec3_t* a);

BYUL_API void vec3_add(vec3_t* out, const vec3_t* a, const vec3_t* b);

BYUL_API void vec3_sub(vec3_t* out, const vec3_t* a, const vec3_t* b);

BYUL_API void vec3_mul(vec3_t* out, const vec3_t* a, const vec3_t* b);

BYUL_API void vec3_div(vec3_t* out, const vec3_t* a, const vec3_t* b);

BYUL_API void vec3_scale(vec3_t* out, const vec3_t* a, float scalar);

BYUL_API float vec3_dot(const vec3_t* a, const vec3_t* b);

BYUL_API void vec3_cross(vec3_t* out, const vec3_t* a, const vec3_t* b);

BYUL_API float vec3_length(const vec3_t* a);

// sqrt를 사용하지 않고 제곱값을 반환한다 계산효율때문에 sqrt는 좀 무겁다  제곱보다...
BYUL_API float vec3_length_sq(const vec3_t* a);

BYUL_API void vec3_normalize(vec3_t* a);

BYUL_API void vec3_unit(vec3_t* out, const vec3_t* src);

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

/**
 * @brief vec3_t 벡터가 (0,0,0)인지 검사합니다.
 *
 * @param v 검사할 벡터
 * @return true  모든 성분이 0에 매우 가까우면 true
 * @return false 하나라도 0이 아니면 false
 */
BYUL_API bool vec3_is_zero(const vec3_t* v);

/**
 * @brief vec3 값을 문자열로 변환
 * @param v 변환할 벡터
 * @param buffer 출력 버퍼 (최소 64바이트 권장)
 * @param buffer_size 버퍼 크기
 * @return buffer (편의상 반환)
 */
BYUL_API char* vec3_to_string(
    const vec3_t* v, char* buffer, size_t buffer_size);

/**
 * @brief vec3 값을 콘솔에 출력
 * @param v 출력할 벡터
 */
BYUL_API void vec3_print(const vec3_t* v);

#ifdef __cplusplus
}
#endif

#endif // VEC3_H
