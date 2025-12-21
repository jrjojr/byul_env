/* roll.h
 *
 * "Roll" helpers built on top of rng.h.
 *
 * Design goals:
 * - English-only (MSVC friendly)
 * - Deterministic when you pass a byul_rng_t*
 * - Small, C99-compatible API (works in C++ via extern "C")
 *
 */

#ifndef BYUL_ROLL_H
#define BYUL_ROLL_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#include "rng.h"

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------- */
/* Public API                                                                 */
/* -------------------------------------------------------------------------- */

/* Uniform integer roll in [1 .. sides].
 * - If sides == 0, returns 0.
 */
uint32_t byul_roll(byul_rng_t* rng, uint32_t sides);

/* Pick one value from a pool (uniform).
 * - 'values' must have 'count' elements.
 * - If count == 0, returns 0.
 * - Duplicates are allowed and naturally increase probability.
 */
int32_t byul_roll_pick(const int32_t* values, size_t count, byul_rng_t* rng);

/* Weighted pick:
 * - 'values' and 'weights' must have 'count' elements.
 * - weights may be 0 (effectively excluded).
 * - If total weight == 0 or count == 0, returns 0.
 */
int32_t byul_roll_pick_weighted(const int32_t* values,
                                const uint32_t* weights,
                                size_t count,
                                byul_rng_t* rng);

/* Percent check:
 * - percent: 0..100
 * - returns true with probability = percent%
 */
bool byul_roll_check_percent(byul_rng_t* rng, uint32_t percent);

/* Take one value from a mutable pool WITHOUT replacement (removes the chosen item).
 * - pool: mutable array of int32_t
 * - count: in/out; decreases by 1 on success
 * - If pool == NULL or count == NULL or *count == 0, returns 0 and does not modify.
 *
 * Complexity: O(1) (swap-remove).
 */
int32_t byul_roll_take(int32_t* pool, size_t* count, byul_rng_t* rng);

/* Take one value from a mutable pool WITHOUT replacement using weights.
 * - pool: mutable array of int32_t (will be modified)
 * - weights: mutable array of uint32_t aligned with pool (will be modified)
 * - count: in/out; decreases by 1 on success
 * - If total weight == 0 or *count == 0, returns 0 and does not modify.
 *
 * Behavior:
 * - Select index i with probability proportional to weights[i].
 * - Remove chosen element via swap-remove on BOTH pool and weights.
 *
 * Complexity: O(n) per call.
 */
int32_t byul_roll_take_weighted(int32_t* pool,
                                uint32_t* weights,
                                size_t* count,
                                byul_rng_t* rng);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* BYUL_ROLL_H */
