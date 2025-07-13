#ifndef CORE_H
#define CORE_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "byul_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/// @brief float 비교를 위한 epsilon
#define FLOAT_EPSILON 1e-5f

// ---------------------- float 비교 정확도 ----------------------

BYUL_API bool float_equal(float a, float b);
BYUL_API int  float_compare(float a, float b, void* userdata);
BYUL_API int  int_compare(int a, int b, void* userdata);

// ---------------------- hashset_t 타입 정의 ----------------------

typedef void* hashkey;
typedef void* valueptr;

/// @brief 사용자 정의 순회 콜백
typedef void (*hashset_func)(hashkey item, valueptr userdata);

/// @brief opaque 포인터: C++ 구현체를 감싼 구조
typedef struct s_hashset hashset_t;

BYUL_API hashset_t* hashset_new();
BYUL_API void     hashset_free(hashset_t* hs);

BYUL_API bool     hashset_add(hashset_t* hs, hashkey item);
BYUL_API bool     hashset_contains(const hashset_t* hs, hashkey item);
BYUL_API bool     hashset_remove(hashset_t* hs, hashkey item);

BYUL_API size_t   hashset_size(const hashset_t* hs);
BYUL_API void     hashset_clear(hashset_t* hs);

BYUL_API hashkey  hashset_peek(hashset_t* hs);
BYUL_API hashkey  hashset_pop(hashset_t* hs);

BYUL_API void     hashset_foreach(hashset_t* hs, 
    hashset_func fn, valueptr userdata);

BYUL_API hashset_t* hashset_copy(const hashset_t* original);
BYUL_API bool     hashset_equal(const hashset_t* a, const hashset_t* b);
BYUL_API uint32_t hashset_hash(const hashkey key);

BYUL_API hashset_t* hashset_union(const hashset_t* a, const hashset_t* b);
BYUL_API hashset_t* hashset_intersect(const hashset_t* a, const hashset_t* b);
BYUL_API hashset_t* hashset_difference(
    const hashset_t* a, const hashset_t* b);

#ifdef __cplusplus
}
#endif

#endif // CORE_H
