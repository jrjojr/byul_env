/* shuffle.h
 *
 * RNG module helper: in-place Fisher–Yates shuffle (uniform permutation).
 *
 * Design goals:
 * - English-only (MSVC friendly)
 * - Deterministic when you pass a byul_rng_t*
 * - Small, C99-compatible API (works in C++ via extern "C")
 *
 * Notes:
 * - This header provides NO macros and NO inline wrappers.
 * - The API operates in-place and does not allocate memory.
 * - Higher-level helpers (roll, distributions) are intentionally separated.
 *
 * Naming policy:
 * - Public linker-facing symbols use the byul_* prefix.
 * - Function names are kept minimal and explicit.
 */

#ifndef BYUL_RNG_SHUFFLE_H
#define BYUL_RNG_SHUFFLE_H

#include <stdint.h>
#include <stddef.h>

#include "rng_core.h" /* byul_rng_t */

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------- */
/* Public API                                                                 */
/* -------------------------------------------------------------------------- */

/* In-place shuffle of an array with arbitrary element size.
 *
 * Parameters:
 * - base      : pointer to the first element
 * - count     : number of elements
 * - elem_size : size of each element in bytes
 * - rng       : RNG state (must be non-NULL for deterministic behavior)
 *
 * Behavior:
 * - If base == NULL, rng == NULL, elem_size == 0, or count < 2: no-op.
 * - Uses the Fisher–Yates algorithm.
 * - Produces a uniform random permutation assuming the underlying RNG
 *   provides unbiased bounded integers.
 */
void byul_shuffle(void* base,
                  size_t count,
                  size_t elem_size,
                  byul_rng_t* rng);

/* Typed convenience: shuffle int32_t array in-place.
 *
 * Equivalent to:
 *   byul_shuffle(values, count, sizeof(int32_t), rng);
 */
void byul_shuffle_i32(int32_t* values, size_t count, byul_rng_t* rng);

/* Typed convenience: shuffle uint32_t array in-place.
 *
 * Equivalent to:
 *   byul_shuffle(values, count, sizeof(uint32_t), rng);
 */
void byul_shuffle_u32(uint32_t* values, size_t count, byul_rng_t* rng);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* BYUL_RNG_SHUFFLE_H */
