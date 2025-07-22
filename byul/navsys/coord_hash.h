#ifndef COORD_HASH_H
#define COORD_HASH_H

#include <stdint.h>
#include <stdbool.h>
#include "byul_config.h"
#include "internal/coord.h"
#include "internal/coord_list.h"  // for coord_list_t

#ifdef __cplusplus
extern "C" {
#endif

typedef struct s_coord_hash coord_hash_t;

typedef void* (*coord_hash_copy_func)(const void* value);
typedef void  (*coord_hash_destroy_func)(void* value);

BYUL_API void* int_copy(const void* p);
BYUL_API void int_destroy(void* p);

BYUL_API void* float_copy(const void* p);
BYUL_API void float_destroy(void* p);

BYUL_API void* double_copy(const void* p);
BYUL_API void double_destroy(void* p);

// 생성/해제

// 기본형 (int)
BYUL_API coord_hash_t* coord_hash_create();  

BYUL_API coord_hash_t* coord_hash_create_full(coord_hash_copy_func copy_func,
                                  coord_hash_destroy_func free_func);
    
BYUL_API void          coord_hash_destroy(coord_hash_t* hash);
BYUL_API coord_hash_t* coord_hash_copy(const coord_hash_t* original);

BYUL_API uint32_t coord_hash_hash(const coord_hash_t* h);

// 기본 연산
BYUL_API int   coord_hash_length(const coord_hash_t* hash);
BYUL_API bool  coord_hash_is_empty(const coord_hash_t* hash);
BYUL_API void* coord_hash_get(const coord_hash_t* hash, const coord_t* key);
BYUL_API bool  coord_hash_contains(
    const coord_hash_t* hash, const coord_t* key);

// 설정/수정
BYUL_API void  coord_hash_set(coord_hash_t* hash, 
    const coord_t* key, void* value); 

BYUL_API bool  coord_hash_insert(coord_hash_t* hash, 
    const coord_t* key, void* value); 

// 키는 유지하고 값은 변경한다. 이미 키가 존재하면
BYUL_API bool coord_hash_replace(coord_hash_t* hash, 
    const coord_t* key, void* value);    
    
BYUL_API bool  coord_hash_remove(coord_hash_t* hash, const coord_t* key);
BYUL_API void  coord_hash_clear(coord_hash_t* hash);
BYUL_API void  coord_hash_remove_all(coord_hash_t* hash);

// 비교
BYUL_API bool  coord_hash_equal(const coord_hash_t* a, const coord_hash_t* b);

// 키/값 조회
BYUL_API coord_list_t* coord_hash_keys(const coord_hash_t* hash);
BYUL_API void**        coord_hash_values(
    const coord_hash_t* hash, int* out_count);

// 반복
typedef void (*coord_hash_func)(
    const coord_t* key, void* value, void* user_data);

BYUL_API void  coord_hash_foreach(
    coord_hash_t* hash, coord_hash_func func, void* user_data);

// 변환
BYUL_API coord_list_t* coord_hash_to_list(const coord_hash_t* hash);

// 확장
BYUL_API void coord_hash_export(
    const coord_hash_t* hash,
    coord_list_t* keys_out,
    void** values_out,
    int* count_out);

typedef struct s_coord_hash_iter coord_hash_iter_t;

BYUL_API coord_hash_iter_t* coord_hash_iter_create(
    const coord_hash_t* hash);

BYUL_API bool coord_hash_iter_next(
    coord_hash_iter_t* iter, coord_t** key_out, void** val_out);

BYUL_API void coord_hash_iter_destroy(
    coord_hash_iter_t* iter);

BYUL_API char* coord_hash_to_string(const coord_hash_t* hash);
BYUL_API void coord_hash_print(const coord_hash_t* hash);

#ifdef __cplusplus
}
#endif

#endif // COORD_HASH_H
