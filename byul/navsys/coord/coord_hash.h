#ifndef COORD_HASH_H
#define COORD_HASH_H

#include <stdint.h>
#include <stdbool.h>
#include "byul_common.h"
#include "coord.h"
#include "coord_list.h"  // for coord_list_t

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

// Creation/Destruction

// Default type (int)
BYUL_API coord_hash_t* coord_hash_create();  

BYUL_API coord_hash_t* coord_hash_create_full(coord_hash_copy_func copy_func,
                                  coord_hash_destroy_func destroy_func);
    
BYUL_API void          coord_hash_destroy(coord_hash_t* hash);
BYUL_API coord_hash_t* coord_hash_copy(const coord_hash_t* original);

BYUL_API uint32_t coord_hash_hash(const coord_hash_t* h);

// Basic Operations
BYUL_API int   coord_hash_length(const coord_hash_t* hash);
BYUL_API bool  coord_hash_is_empty(const coord_hash_t* hash);

/**
 * @brief Retrieve a value from the hash map using a coord_t key.
 *
 * This function attempts to look up the value associated with the given key
 * in the coord_hash_t map using fast hash lookup (O(1) expected).
 *
 * @param[in]  hash  Hash table to query
 * @param[in]  key   Pointer to coord_t to be used as key
 * @return Pointer to stored value if found, or NULL if not found
 */
BYUL_API void* coord_hash_get(const coord_hash_t* hash, const coord_t* key);

/**
 * @brief Retrieve a value from the hash map by directly comparing x and y coordinates.
 *
 * This function performs a linear search over the hash map and compares the values
 * of x and y instead of relying on hash or pointer identity. 
 * 
 * @warning This function performs a linear scan (O(n) complexity). It is significantly
 *          slower than coord_hash_get() and should only be used when correctness is
 *          more important than performance.
 *
 * @param[in]  hash  Hash table to search
 * @param[in]  x     X coordinate of the key
 * @param[in]  y     Y coordinate of the key
 * @return Pointer to stored value if found, or NULL if not found
 */
BYUL_API void* coord_hash_get_xy(const coord_hash_t* hash, int x, int y);

BYUL_API bool  coord_hash_contains(
    const coord_hash_t* hash, const coord_t* key);

// Set/Modify
BYUL_API void  coord_hash_set(coord_hash_t* hash, 
    const coord_t* key, void* value); 

BYUL_API bool  coord_hash_insert(coord_hash_t* hash, 
    const coord_t* key, void* value); 

/**
 * @brief Insert a value into the hash table using raw x and y coordinates.
 *
 * Internally creates a temporary coord_t and uses coord_hash_insert().
 *
 * @param[in]  hash   Hash table to insert into
 * @param[in]  x      X coordinate of the key
 * @param[in]  y      Y coordinate of the key
 * @param[in]  value  Pointer to value to store
 * @return true on success, false on failure (null key or hash)
 */
BYUL_API bool coord_hash_insert_xy(
    coord_hash_t* hash, int x, int y, void* value);

// Keep the key but change its value. If the key already exists
BYUL_API bool coord_hash_replace(coord_hash_t* hash, 
    const coord_t* key, void* value);    

BYUL_API bool coord_hash_replace_xy(
    coord_hash_t* hash, int x, int y, void* value);
    
BYUL_API bool  coord_hash_remove(coord_hash_t* hash, const coord_t* key);
BYUL_API void  coord_hash_clear(coord_hash_t* hash);
BYUL_API void  coord_hash_remove_all(coord_hash_t* hash);

// Comparison
BYUL_API bool  coord_hash_equal(const coord_hash_t* a, const coord_hash_t* b);

// Key/Value Access
BYUL_API coord_list_t* coord_hash_keys(const coord_hash_t* hash);
BYUL_API void**        coord_hash_values(
    const coord_hash_t* hash, int* out_count);

// Iteration
typedef void (*coord_hash_func)(
    const coord_t* key, void* value, void* user_data);

BYUL_API void  coord_hash_foreach(
    coord_hash_t* hash, coord_hash_func func, void* user_data);

// Conversion
BYUL_API coord_list_t* coord_hash_to_list(const coord_hash_t* hash);

// Export
BYUL_API void coord_hash_export(
    const coord_hash_t* hash,
    coord_list_t* keys_out,
    void** values_out,
    int* count_out);

typedef struct s_coord_hash_iter coord_hash_iter_t;

BYUL_API coord_hash_iter_t* coord_hash_iter_create(
    const coord_hash_t* hash);

BYUL_API bool coord_hash_iter_next(
    coord_hash_iter_t* iter, coord_t* key_out, void** val_out);

BYUL_API void coord_hash_iter_destroy(
    coord_hash_iter_t* iter);

BYUL_API char* coord_hash_to_string(const coord_hash_t* hash);
BYUL_API void coord_hash_print(const coord_hash_t* hash);

#ifdef __cplusplus
}
#endif

#endif // COORD_HASH_H
