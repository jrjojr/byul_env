#ifndef HASHSET_H
#define HASHSET_H

#include "byul_common.h"
#include <stddef.h>   // size_t

#ifdef __cplusplus
extern "C" {
#endif

// ---------------------- hashset_t type definition ----------------------

typedef void* hashkey;
typedef void* valueptr;

/// @brief User-defined iteration callback
typedef void (*hashset_func)(hashkey item, valueptr userdata);

/// @brief Opaque pointer: Structure wrapping the C++ implementation
typedef struct s_hashset hashset_t;

BYUL_API hashset_t* hashset_create();
BYUL_API void     hashset_destroy(hashset_t* hs);

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
BYUL_API unsigned int hashset_hash(const hashkey key);

BYUL_API hashset_t* hashset_union(const hashset_t* a, const hashset_t* b);
BYUL_API hashset_t* hashset_intersect(const hashset_t* a, const hashset_t* b);
BYUL_API hashset_t* hashset_difference(
    const hashset_t* a, const hashset_t* b);

#ifdef __cplusplus
}
#endif

#endif // HASHSET_H
