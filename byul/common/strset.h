#ifndef STRSET_H
#define STRSET_H

#include "byul_common.h"
#include <stddef.h>   // size_t

#ifdef __cplusplus
extern "C" {
#endif

/// @brief 문자열 집합을 위한 불투명 포인터 타입
typedef struct s_strset strset_t;

/// @brief 순회 콜백
typedef void (*strset_func)(const char* item, void* userdata);

// ---------------------- 생성/소멸 ----------------------

BYUL_API strset_t* strset_create();
BYUL_API void      strset_destroy(strset_t* ss);

// ---------------------- 기본 연산 ----------------------

BYUL_API bool      strset_add(strset_t* ss, const char* item);
BYUL_API bool      strset_contains(const strset_t* ss, const char* item);
BYUL_API bool      strset_remove(strset_t* ss, const char* item);

BYUL_API size_t    strset_size(const strset_t* ss);
BYUL_API void      strset_clear(strset_t* ss);

// ---------------------- Peek/Pop ----------------------

BYUL_API const char* strset_peek(const strset_t* ss);
BYUL_API char*       strset_pop(strset_t* ss);

// ---------------------- 순회 ----------------------

BYUL_API void      strset_foreach(strset_t* ss, strset_func fn, void* userdata);

// ---------------------- 복사/비교 ----------------------

BYUL_API strset_t* strset_copy(const strset_t* original);
BYUL_API bool      strset_equal(const strset_t* a, const strset_t* b);

// ---------------------- 집합 연산 ----------------------

BYUL_API strset_t* strset_union(const strset_t* a, const strset_t* b);
BYUL_API strset_t* strset_intersect(const strset_t* a, const strset_t* b);
BYUL_API strset_t* strset_difference(const strset_t* a, const strset_t* b);

#ifdef __cplusplus
}
#endif

#endif // STRSET_H
