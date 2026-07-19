#ifndef COORD_HASH_H
#define COORD_HASH_H

#include <stdbool.h>
#include <stdint.h>

#include "byul_config.h"
#include "coord.h"
#include "coord_list.h"  // for coord_list_t
#include "navsys_status.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct s_coord_hash coord_hash_t;

typedef void* (*coord_hash_copy_func)(const void* value);
typedef void  (*coord_hash_destroy_func)(void* value);

/**
 * @brief Copies one value for canonical coord hash storage.
 *
 * On success, out_copy receives one value owned by the table. On failure,
 * out_copy must remain unchanged.
 */
typedef navsys_status_t (*coord_hash_value_copy_func_ex)(
    const void* source, void** out_copy, void* userdata);

/**
 * @brief Destroys one non-NULL value created by the matching copy callback.
 *
 * The callback must not throw or reenter an operation on the same table.
 */
typedef void (*coord_hash_value_destroy_func_ex)(
    void* value, void* userdata);

/**
 * @brief Compares two stored values without taking ownership.
 */
typedef bool (*coord_hash_value_equal_func_ex)(
    const void* lhs, const void* rhs, void* userdata);

#define BYUL_COORD_HASH_CREATE_INFO_ABI_VERSION UINT32_C(1)

/**
 * @brief Immutable callback binding for coord_hash_create_ex().
 *
 * Set struct_size to sizeof(coord_hash_create_info_t) and abi_version to
 * BYUL_COORD_HASH_CREATE_INFO_ABI_VERSION. userdata is borrowed until the
 * table is destroyed and is passed back unchanged to every callback.
 *
 * @byul.storage basic-value
 * @byul.zero_valid false
 * @byul.copy_semantics trivial-copy
 * @byul.thread_safety thread-compatible
 */
typedef struct s_coord_hash_create_info {
    uint32_t struct_size;
    uint32_t abi_version;
    coord_hash_value_copy_func_ex copy_value;
    coord_hash_value_destroy_func_ex destroy_value;
    coord_hash_value_equal_func_ex equal_value;
    void* userdata;
} coord_hash_create_info_t;

/**
 * @brief Creates a table with one atomic callback and userdata binding.
 *
 * Failure preserves out_hash. The table accepts NULL values. A non-NULL value
 * mutation requires copy_value; otherwise it returns NAVSYS_STATUS_UNSUPPORTED.
 *
 * @param[in] info Complete versioned creation information.
 * @param[out] out_hash Receives a caller-owned table on success.
 * @return Common Navsys status value.
 * @retval NAVSYS_STATUS_OK The table was created.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT A pointer, size, version, or callback
 *     pairing is invalid.
 * @retval NAVSYS_STATUS_OUT_OF_MEMORY Table storage could not be allocated.
 * @byul.nullable info false
 * @byul.nullable out_hash false
 * @byul.lifetime out_hash caller-owned
 * @byul.error enum:navsys_status_t
 * @byul.side_effect mutates:out_hash-on-success
 * @byul.thread_safety thread-compatible
 * @byul.blocking false
 */
BYUL_API navsys_status_t coord_hash_create_ex(
    const coord_hash_create_info_t* info, coord_hash_t** out_hash);

/**
 * @brief Creates an independent deep copy with the source callback binding.
 *
 * Failure preserves out_hash and the source. A source without a copy callback
 * returns NAVSYS_STATUS_UNSUPPORTED instead of creating NULL-filled entries.
 *
 * @param[in] source Table to copy.
 * @param[out] out_hash Receives a caller-owned table on success.
 * @return Common Navsys status value.
 * @retval NAVSYS_STATUS_OK The table was copied.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT A required pointer is NULL.
 * @retval NAVSYS_STATUS_UNSUPPORTED No copy callback is bound.
 * @retval NAVSYS_STATUS_CALLBACK_FAILED A callback threw or violated its output
 *     contract.
 * @retval NAVSYS_STATUS_OUT_OF_MEMORY Copy or table allocation failed.
 * @retval NAVSYS_STATUS_IN_PROGRESS A callback on source is active.
 * @byul.nullable source false
 * @byul.nullable out_hash false
 * @byul.lifetime out_hash caller-owned
 * @byul.error enum:navsys_status_t
 * @byul.side_effect mutates:out_hash-on-success
 * @byul.thread_safety thread-compatible
 * @byul.blocking false
 */
BYUL_API navsys_status_t coord_hash_copy_ex(
    const coord_hash_t* source, coord_hash_t** out_hash);

/**
 * @brief Inserts a copied value only when key is absent.
 *
 * NULL is a valid value. Failure preserves the table.
 *
 * @param[in,out] hash Table to mutate.
 * @param[in] key Key copied by value when insertion succeeds.
 * @param[in] value Borrowed source value copied during the call, or NULL.
 * @return Common Navsys status value.
 * @retval NAVSYS_STATUS_OK A new entry was inserted.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT A required pointer is NULL or key
 *     already exists.
 * @retval NAVSYS_STATUS_UNSUPPORTED A non-NULL value has no copy callback.
 * @retval NAVSYS_STATUS_CALLBACK_FAILED A callback failed.
 * @retval NAVSYS_STATUS_OUT_OF_MEMORY Copy or insertion allocation failed.
 * @retval NAVSYS_STATUS_IN_PROGRESS A callback on hash is active.
 * @byul.nullable hash false
 * @byul.nullable key false
 * @byul.nullable value true
 * @byul.error enum:navsys_status_t
 * @byul.side_effect mutates:hash-on-success
 * @byul.thread_safety thread-compatible
 * @byul.blocking false
 */
BYUL_API navsys_status_t coord_hash_insert_copy(
    coord_hash_t* hash, const coord_t* key, const void* value);

/**
 * @brief Replaces a copied value only when key exists.
 *
 * NULL is a valid value. Failure preserves the table and old value.
 *
 * @param[in,out] hash Table to mutate.
 * @param[in] key Existing key to replace.
 * @param[in] value Borrowed source value copied during the call, or NULL.
 * @return Common Navsys status value.
 * @retval NAVSYS_STATUS_OK The existing value was replaced.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT A required pointer is NULL.
 * @retval NAVSYS_STATUS_NOT_FOUND The key does not exist.
 * @retval NAVSYS_STATUS_UNSUPPORTED A non-NULL value has no copy callback.
 * @retval NAVSYS_STATUS_CALLBACK_FAILED A callback failed.
 * @retval NAVSYS_STATUS_OUT_OF_MEMORY Value copy allocation failed.
 * @retval NAVSYS_STATUS_IN_PROGRESS A callback on hash is active.
 * @byul.nullable hash false
 * @byul.nullable key false
 * @byul.nullable value true
 * @byul.error enum:navsys_status_t
 * @byul.side_effect mutates:hash-on-success
 * @byul.thread_safety thread-compatible
 * @byul.blocking false
 */
BYUL_API navsys_status_t coord_hash_replace_copy(
    coord_hash_t* hash, const coord_t* key, const void* value);

/**
 * @brief Inserts or replaces one copied value.
 *
 * NULL is a valid value. Failure preserves hash and out_inserted.
 *
 * @param[in,out] hash Table to mutate.
 * @param[in] key Key to insert or replace.
 * @param[in] value Borrowed source value copied during the call, or NULL.
 * @param[out] out_inserted Receives true for insertion or false for replacement.
 * @return Common Navsys status value.
 * @retval NAVSYS_STATUS_OK The value was committed.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT A required pointer is NULL.
 * @retval NAVSYS_STATUS_UNSUPPORTED A non-NULL value has no copy callback.
 * @retval NAVSYS_STATUS_CALLBACK_FAILED A callback failed.
 * @retval NAVSYS_STATUS_OUT_OF_MEMORY Copy or insertion allocation failed.
 * @retval NAVSYS_STATUS_IN_PROGRESS A callback on hash is active.
 * @byul.nullable hash false
 * @byul.nullable key false
 * @byul.nullable value true
 * @byul.nullable out_inserted false
 * @byul.error enum:navsys_status_t
 * @byul.side_effect mutates:hash,out_inserted-on-success
 * @byul.thread_safety thread-compatible
 * @byul.blocking false
 */
BYUL_API navsys_status_t coord_hash_upsert_copy(
    coord_hash_t* hash, const coord_t* key, const void* value,
    bool* out_inserted);

BYUL_API void* int_copy(const void* p);
BYUL_API void int_destroy(void* p);

BYUL_API void* scalar_copy(const void* p);
BYUL_API void scalar_destroy(void* p);

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
