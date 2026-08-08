/*
 * Copyright (c) 2025-2026 ByulPapa (byuldev@outlook.kr)
 * This file is part of the Byul World project.
 * Licensed under the Byul World Source-Available Non-Commercial License v1.0 (2025).
 * See the LICENSE file in the project root for full license terms.
 */

/**
 * @file dstar_lite_pqueue.h
 * @brief Exact lexicographic priority queue used by D* Lite.
 *
 * The queue owns copies of coordinates and keys and keeps at most one key for
 * each coordinate. It is not internally synchronized.
 */

#ifndef BYUL_DSTAR_LITE_PQUEUE_H
#define BYUL_DSTAR_LITE_PQUEUE_H

#include <stddef.h>

#include "byul_config.h"
#include "coord.h"
#include "dstar_lite_key.h"
#include "navsys_status.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct s_dstar_lite_pqueue dstar_lite_pqueue_t;

/** An atomic by-value queue entry. */
typedef struct s_dstar_lite_pqueue_entry {
    dstar_lite_key_t key;
    coord_t coord;
} dstar_lite_pqueue_entry_t;

/**
 * @brief Creates an empty queue through a status-based output contract.
 * @param[out] out_queue Receives the caller-owned queue on success.
 * @return A status code; NAVSYS_STATUS_OK on success.
 * @byul.nullable out_queue false
 */
BYUL_API navsys_status_t dstar_lite_pqueue_create_ex(
    dstar_lite_pqueue_t** out_queue);

/**
 * @brief Creates a value-independent queue copy.
 * @param[in] source Queue to copy.
 * @param[out] out_queue Receives the caller-owned copy on success.
 * @return A status code; NAVSYS_STATUS_OK on success.
 * @byul.nullable source false
 * @byul.nullable out_queue false
 */
BYUL_API navsys_status_t dstar_lite_pqueue_copy_ex(
    const dstar_lite_pqueue_t* source,
    dstar_lite_pqueue_t** out_queue);

/**
 * @brief Inserts or replaces one coordinate using an exact key.
 * @param[in,out] queue Queue to mutate.
 * @param[in] coord Coordinate copied into the queue.
 * @param[in] key Exact key copied into the queue.
 * @return A status code; NAVSYS_STATUS_OK on success.
 * @byul.nullable queue false
 * @byul.nullable coord false
 * @byul.nullable key false
 */
BYUL_API navsys_status_t dstar_lite_pqueue_upsert(
    dstar_lite_pqueue_t* queue,
    const coord_t* coord,
    const dstar_lite_key_t* key);

/**
 * @brief Copies the minimum exact entry without removing it.
 * @param[in] queue Queue to inspect.
 * @param[out] out_entry Destination entry, preserved on failure.
 * @return NAVSYS_STATUS_OK or NAVSYS_STATUS_NOT_FOUND when empty.
 * @byul.nullable queue false
 * @byul.nullable out_entry false
 */
BYUL_API navsys_status_t dstar_lite_pqueue_peek_min(
    const dstar_lite_pqueue_t* queue,
    dstar_lite_pqueue_entry_t* out_entry);

/**
 * @brief Removes and copies the minimum exact entry atomically.
 * @param[in,out] queue Queue to mutate.
 * @param[out] out_entry Destination entry, preserved on failure.
 * @return NAVSYS_STATUS_OK or NAVSYS_STATUS_NOT_FOUND when empty.
 * @byul.nullable queue false
 * @byul.nullable out_entry false
 */
BYUL_API navsys_status_t dstar_lite_pqueue_pop_min(
    dstar_lite_pqueue_t* queue,
    dstar_lite_pqueue_entry_t* out_entry);

/**
 * @brief Copies the exact key associated with a coordinate.
 * @param[in] queue Queue to inspect.
 * @param[in] coord Coordinate to find.
 * @param[out] out_key Destination key, preserved when absent.
 * @param[out] out_found Receives whether an entry exists.
 * @return A status code; NAVSYS_STATUS_OK on success.
 * @byul.nullable queue false
 * @byul.nullable coord false
 * @byul.nullable out_key false
 * @byul.nullable out_found false
 */
BYUL_API navsys_status_t dstar_lite_pqueue_find_key(
    const dstar_lite_pqueue_t* queue,
    const coord_t* coord,
    dstar_lite_key_t* out_key,
    bool* out_found);

/**
 * @brief Removes an entry by coordinate.
 * @param[in,out] queue Queue to mutate.
 * @param[in] coord Coordinate to remove.
 * @param[out] out_removed Receives whether an entry was removed.
 * @return A status code; NAVSYS_STATUS_OK on success.
 * @byul.nullable queue false
 * @byul.nullable coord false
 * @byul.nullable out_removed false
 */
BYUL_API navsys_status_t dstar_lite_pqueue_remove_ex(
    dstar_lite_pqueue_t* queue,
    const coord_t* coord,
    bool* out_removed);

/**
 * @brief Returns the number of live coordinate entries.
 * @param[in] queue Queue to inspect.
 * @return Live entry count, or zero for invalid input.
 * @byul.nullable queue true
 */
BYUL_API size_t dstar_lite_pqueue_size(
    const dstar_lite_pqueue_t* queue);

/**
 * @brief Reports whether a queue has no live entries.
 * @param[in] queue Queue to inspect.
 * @return true when empty or invalid; false otherwise.
 * @byul.nullable queue true
 */
BYUL_API bool dstar_lite_pqueue_empty(
    const dstar_lite_pqueue_t* queue);

/**
 * @brief Removes all entries from a queue.
 * @param[in,out] queue Queue to clear; NULL is accepted.
 * @byul.nullable queue true
 */
BYUL_API void dstar_lite_pqueue_clear(dstar_lite_pqueue_t* queue);

/**
 * @brief Creates an empty priority queue.
 * @return A caller-owned queue, or NULL when allocation fails.
 * @byul.nullable return true
 * @byul.lifetime return caller-owned
 */
BYUL_API dstar_lite_pqueue_t* dstar_lite_pqueue_create(void);

/**
 * @brief Destroys a priority queue.
 * @param[in,out] q Queue to destroy; NULL is accepted.
 * @byul.nullable q true
 */
BYUL_API void dstar_lite_pqueue_destroy(dstar_lite_pqueue_t* q);

/**
 * @brief Creates a value-independent copy of a priority queue.
 * @param[in] src Queue to copy.
 * @return A caller-owned copy, or NULL for invalid input or allocation failure.
 * @byul.nullable src false
 * @byul.nullable return true
 * @byul.lifetime return caller-owned
 */
BYUL_API dstar_lite_pqueue_t* dstar_lite_pqueue_copy(
    const dstar_lite_pqueue_t* src);

/**
 * @brief Inserts or replaces the exact key associated with a coordinate.
 * @param[in,out] q Queue to mutate.
 * @param[in] key Exact D* Lite key copied into the queue.
 * @param[in] c Coordinate copied into the queue.
 * @byul.nullable q false
 * @byul.nullable key false
 * @byul.nullable c false
 */
BYUL_API void dstar_lite_pqueue_push(
    dstar_lite_pqueue_t* q,
    const dstar_lite_key_t* key,
    const coord_t* c);

/**
 * @brief Returns the coordinate with the smallest key without removing it.
 * @param[in] q Queue to inspect.
 * @return A borrowed coordinate, or NULL when empty or invalid.
 * @byul.nullable q false
 * @byul.nullable return true
 * @byul.lifetime return borrowed:q
 */
BYUL_API const coord_t* dstar_lite_pqueue_peek(dstar_lite_pqueue_t* q);

/**
 * @brief Removes and returns the coordinate with the smallest key.
 * @param[in,out] q Queue to mutate.
 * @return A caller-owned coordinate, or NULL when empty or invalid.
 * @byul.nullable q false
 * @byul.nullable return true
 * @byul.lifetime return caller-owned
 */
BYUL_API coord_t* dstar_lite_pqueue_pop(dstar_lite_pqueue_t* q);

/**
 * @brief Reports whether a queue has no entries.
 * @param[in] q Queue to inspect.
 * @return true when empty or invalid; false otherwise.
 * @byul.nullable q true
 */
BYUL_API bool dstar_lite_pqueue_is_empty(dstar_lite_pqueue_t* q);

/**
 * @brief Removes the entry associated with a coordinate.
 * @param[in,out] q Queue to mutate.
 * @param[in] u Coordinate to remove.
 * @return true when an entry was removed; false otherwise.
 * @byul.nullable q false
 * @byul.nullable u false
 */
BYUL_API bool dstar_lite_pqueue_remove(
    dstar_lite_pqueue_t* q, const coord_t* u);

/**
 * @brief Removes an entry only when both coordinate and exact key match.
 * @param[in,out] q Queue to mutate.
 * @param[in] key Exact key to match.
 * @param[in] c Coordinate to match.
 * @return true when an entry was removed; false otherwise.
 * @byul.nullable q false
 * @byul.nullable key false
 * @byul.nullable c false
 */
BYUL_API bool dstar_lite_pqueue_remove_full(
    dstar_lite_pqueue_t* q,
    const dstar_lite_key_t* key,
    const coord_t* c);

/**
 * @brief Finds the exact key currently associated with a coordinate.
 * @param[in] q Queue to inspect.
 * @param[in] c Coordinate to find.
 * @return A borrowed key, or NULL when absent or invalid.
 * @byul.nullable q false
 * @byul.nullable c false
 * @byul.nullable return true
 * @byul.lifetime return borrowed:q
 */
BYUL_API dstar_lite_key_t* dstar_lite_pqueue_get_key_by_coord(
    dstar_lite_pqueue_t* q, const coord_t* c);

/**
 * @brief Copies the smallest key in a queue.
 * @param[in] q Queue to inspect.
 * @return A caller-owned key, or NULL when empty or invalid.
 * @byul.nullable q false
 * @byul.nullable return true
 * @byul.lifetime return caller-owned
 */
BYUL_API dstar_lite_key_t* dstar_lite_pqueue_top_key(dstar_lite_pqueue_t* q);

/**
 * @brief Reports whether a coordinate has an entry.
 * @param[in] q Queue to inspect.
 * @param[in] u Coordinate to find.
 * @return true when present; false otherwise.
 * @byul.nullable q false
 * @byul.nullable u false
 */
BYUL_API bool dstar_lite_pqueue_contains(
    dstar_lite_pqueue_t* q, const coord_t* u);

#ifdef __cplusplus
}
#endif

#endif /* BYUL_DSTAR_LITE_PQUEUE_H */
