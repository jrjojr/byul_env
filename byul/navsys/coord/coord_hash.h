#ifndef COORD_HASH_H
#define COORD_HASH_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "byul_config.h"
#include "coord.h"
#include "coord_list.h"  // for coord_list_t
#include "navsys_status.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct s_coord_hash coord_hash_t;
typedef struct s_coord_hash_iter coord_hash_iter_t;

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

/**
 * @brief Caller buffer로 내보내는 key와 borrowed value view다.
 *
 * value는 table의 다음 mutation 또는 destroy 전까지만 유효하다.
 *
 * @byul.storage basic-value
 * @byul.zero_valid true
 * @byul.copy_semantics trivial-copy
 * @byul.thread_safety thread-compatible
 */
typedef struct s_coord_hash_entry_view {
    coord_t key;
    const void* value;
} coord_hash_entry_view_t;

/**
 * @brief Table의 현재 entry 수를 반환한다.
 *
 * @param[in] hash 조회할 table.
 * @return Entry 수. hash가 NULL이면 0이다.
 * @byul.nullable hash true
 * @byul.thread_safety thread-compatible
 * @byul.blocking false
 */
BYUL_API size_t coord_hash_size(const coord_hash_t* hash);

/**
 * @brief Key를 조회하고 stored NULL과 not-found를 구별한다.
 *
 * 성공하면 out_found를 기록하고, key가 있으면 out_borrowed_value에 다음 mutation 또는
 * destroy 전까지 유효한 borrowed pointer를 기록한다. Stored NULL이면 found는 true이고
 * value는 NULL이다. Key가 없으면 found는 false이고 value는 NULL이다.
 *
 * @param[in] hash 조회할 table.
 * @param[in] key 조회할 key.
 * @param[out] out_borrowed_value Borrowed value를 받을 storage.
 * @param[out] out_found Key 존재 여부를 받을 storage.
 * @return 공통 Navsys 상태 값.
 * @retval NAVSYS_STATUS_OK 조회 결과를 기록했다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT 필수 pointer가 NULL이다.
 * @byul.nullable hash false
 * @byul.nullable key false
 * @byul.nullable out_borrowed_value false
 * @byul.nullable out_found false
 * @byul.lifetime out_borrowed_value borrowed-until-next-mutation-or-destroy
 * @byul.error enum:navsys_status_t
 * @byul.side_effect mutates:out_borrowed_value,out_found-on-success
 * @byul.thread_safety thread-compatible
 * @byul.blocking false
 */
BYUL_API navsys_status_t coord_hash_find(
    const coord_hash_t* hash, const coord_t* key,
    const void** out_borrowed_value, bool* out_found);

/**
 * @brief 두 table의 key 집합이 같은지 비교한다.
 *
 * @param[in] a 비교할 첫 table.
 * @param[in] b 비교할 둘째 table.
 * @param[out] out_equal key 집합 비교 결과를 받을 storage.
 * @return 공통 Navsys 상태 값.
 * @retval NAVSYS_STATUS_OK 비교 결과를 기록했다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT 필수 pointer가 NULL이다.
 * @byul.nullable a false
 * @byul.nullable b false
 * @byul.nullable out_equal false
 * @byul.error enum:navsys_status_t
 * @byul.side_effect mutates:out_equal-on-success
 * @byul.thread_safety thread-compatible
 * @byul.blocking false
 */
BYUL_API navsys_status_t coord_hash_equal_keys(
    const coord_hash_t* a, const coord_hash_t* b, bool* out_equal);

/**
 * @brief 같은 immutable equal callback binding으로 key와 value를 비교한다.
 *
 * 두 table의 equal callback과 userdata identity가 같아야 한다. NULL value끼리는
 * callback 없이 같고, 한쪽만 NULL이면 다르다. 실패하면 out_equal을 보존한다.
 *
 * @param[in] a 비교할 첫 table.
 * @param[in] b 비교할 둘째 table.
 * @param[out] out_equal 전체 비교 결과를 받을 storage.
 * @return 공통 Navsys 상태 값.
 * @retval NAVSYS_STATUS_OK 비교 결과를 기록했다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT 필수 pointer가 NULL이다.
 * @retval NAVSYS_STATUS_UNSUPPORTED equal callback binding이 없거나 서로 다르다.
 * @retval NAVSYS_STATUS_CALLBACK_FAILED equal callback이 예외를 던졌다.
 * @retval NAVSYS_STATUS_IN_PROGRESS 같은 table의 callback이 실행 중이다.
 * @byul.nullable a false
 * @byul.nullable b false
 * @byul.nullable out_equal false
 * @byul.error enum:navsys_status_t
 * @byul.side_effect mutates:out_equal-on-success
 * @byul.thread_safety thread-compatible
 * @byul.blocking false
 */
BYUL_API navsys_status_t coord_hash_equal_full(
    const coord_hash_t* a, const coord_hash_t* b, bool* out_equal);

/**
 * @brief Key를 caller buffer에 내보낸다.
 *
 * out_keys가 NULL이고 capacity가 0이면 필요한 수를 out_count에 기록한다. Buffer가
 * 짧으면 out_count만 기록하고 buffer는 보존한다. 순서는 보장하지 않는다.
 *
 * @param[in] hash 내보낼 table.
 * @param[out] out_keys caller-provided key buffer 또는 query 시 NULL.
 * @param[in] capacity out_keys의 coord_t element 용량.
 * @param[out] out_count 필요한 전체 element 수.
 * @return 공통 Navsys 상태 값.
 * @retval NAVSYS_STATUS_OK query 또는 전체 export가 완료됐다.
 * @retval NAVSYS_STATUS_INCOMPLETE capacity가 부족하다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT pointer/capacity 조합이 잘못됐다.
 * @byul.nullable hash false
 * @byul.nullable out_keys true
 * @byul.nullable out_count false
 * @byul.capacity out_keys capacity
 * @byul.error enum:navsys_status_t
 * @byul.side_effect mutates:out_count-always,out_keys-on-success
 * @byul.thread_safety thread-compatible
 * @byul.blocking false
 */
BYUL_API navsys_status_t coord_hash_export_keys(
    const coord_hash_t* hash, coord_t* out_keys,
    size_t capacity, size_t* out_count);

/**
 * @brief Key와 borrowed value view를 caller buffer에 내보낸다.
 *
 * Query/short-buffer 정책과 순서 계약은 coord_hash_export_keys()와 같다. Export한
 * value pointer는 table의 다음 mutation 또는 destroy 전까지만 유효하다.
 *
 * @param[in] hash 내보낼 table.
 * @param[out] out_entries caller-provided entry buffer 또는 query 시 NULL.
 * @param[in] capacity out_entries의 element 용량.
 * @param[out] out_count 필요한 전체 element 수.
 * @return 공통 Navsys 상태 값.
 * @retval NAVSYS_STATUS_OK query 또는 전체 export가 완료됐다.
 * @retval NAVSYS_STATUS_INCOMPLETE capacity가 부족하다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT pointer/capacity 조합이 잘못됐다.
 * @byul.nullable hash false
 * @byul.nullable out_entries true
 * @byul.nullable out_count false
 * @byul.capacity out_entries capacity
 * @byul.lifetime out_entries borrowed-elements-until-next-mutation-or-destroy
 * @byul.error enum:navsys_status_t
 * @byul.side_effect mutates:out_count-always,out_entries-on-success
 * @byul.thread_safety thread-compatible
 * @byul.blocking false
 */
BYUL_API navsys_status_t coord_hash_export_entries(
    const coord_hash_t* hash, coord_hash_entry_view_t* out_entries,
    size_t capacity, size_t* out_count);

/**
 * @brief Iterator의 다음 entry를 generation 검증과 함께 반환한다.
 *
 * Parent mutation 전에는 output value가 borrowed다. Parent가 destroy돼도 iterator
 * 자체는 destroy할 수 있으며 next는 INVALIDATED를 반환한다.
 *
 * @param[in,out] iter 진행할 iterator.
 * @param[out] key_out 다음 key를 받을 storage.
 * @param[out] value_out 다음 borrowed value를 받을 storage.
 * @return 공통 Navsys 상태 값.
 * @retval NAVSYS_STATUS_OK 다음 entry를 기록했다.
 * @retval NAVSYS_STATUS_NOT_FOUND iterator가 끝났다.
 * @retval NAVSYS_STATUS_INVALIDATED parent가 mutation 또는 destroy됐다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT 필수 pointer가 NULL이다.
 * @byul.nullable iter false
 * @byul.nullable key_out false
 * @byul.nullable value_out false
 * @byul.lifetime value_out borrowed-until-parent-mutation-or-destroy
 * @byul.error enum:navsys_status_t
 * @byul.side_effect mutates:iter,key_out,value_out-on-success
 * @byul.thread_safety thread-compatible
 * @byul.blocking false
 */
BYUL_API navsys_status_t coord_hash_iter_next_ex(
    coord_hash_iter_t* iter, coord_t* key_out, const void** value_out);

/**
 * @brief Table key를 결정적 순서의 UTF-8 문자열로 기록한다.
 *
 * 필요한 byte 수는 NUL을 포함한다. out_buffer가 NULL이고 capacity가 0이면
 * out_required만 기록한다. 짧은 buffer는 보존한다.
 *
 * @param[in] hash format할 table.
 * @param[out] out_buffer caller-provided byte buffer 또는 query 시 NULL.
 * @param[in] capacity out_buffer의 byte 용량.
 * @param[out] out_required NUL을 포함한 필요한 byte 수.
 * @return 공통 Navsys 상태 값.
 * @retval NAVSYS_STATUS_OK query 또는 전체 기록이 완료됐다.
 * @retval NAVSYS_STATUS_INCOMPLETE capacity가 부족하다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT pointer/capacity 조합이 잘못됐다.
 * @retval NAVSYS_STATUS_OUT_OF_MEMORY 임시 format storage 할당에 실패했다.
 * @retval NAVSYS_STATUS_CORRUPT_STATE 내부 문자열 변환이 실패했다.
 * @byul.nullable hash false
 * @byul.nullable out_buffer true
 * @byul.nullable out_required false
 * @byul.capacity out_buffer capacity
 * @byul.encoding out_buffer utf-8
 * @byul.error enum:navsys_status_t
 * @byul.side_effect mutates:out_required-always,out_buffer-on-success
 * @byul.thread_safety thread-compatible
 * @byul.blocking false
 */
BYUL_API navsys_status_t coord_hash_format(
    const coord_hash_t* hash, char* out_buffer,
    size_t capacity, size_t* out_required);

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
