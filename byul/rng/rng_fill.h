/* rng_fill.h
 *
 * RNG batch helpers: fill user-provided arrays with random values.
 *
 * Design goals:
 * - English-only (MSVC friendly)
 * - Deterministic when you pass a byul_rng_t*
 * - Small, C99-compatible API (works in C++ via extern "C")
 *
 * Notes:
 * - This header provides NO macros and NO inline wrappers.
 * - These functions do not allocate memory.
 * - The caller owns the output buffers and their sizes.
 *
 * Module contrast:
 * - rng.h            : raw random numbers (u32/u64/f32/f64, bounded ranges)
 * - rng_fill.h       : batch generation into arrays (fill buffers)
 * - roll.h           : discrete selection helpers (pick, weighted pick, take)
 * - shuffle.h        : in-place permutation (Fisher–Yates)
 * - distributions.h  : statistical samplers (normal, exponential, etc.)
 */

#ifndef BYUL_RNG_FILL_H
#define BYUL_RNG_FILL_H

#include <stdint.h>
#include <stddef.h>

#include "rng.h"

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------- */
/* Core integer fills                                                         */
/* -------------------------------------------------------------------------- */

/* Fill out[0..count) with byul_rng_u32() draws.
 * - If rng == NULL or out == NULL: no-op.
 */
void byul_rng_fill_u32(byul_rng_t* rng, uint32_t* out, size_t count);

/* Fill out[0..count) with byul_rng_u64() draws.
 * - If rng == NULL or out == NULL: no-op.
 */
void byul_rng_fill_u64(byul_rng_t* rng, uint64_t* out, size_t count);

/* -------------------------------------------------------------------------- */
/* Bounded integer fills (unbiased)                                           */
/* -------------------------------------------------------------------------- */

/* Fill out[0..count) with values in [0, max).
 * - Uses byul_rng_range_u32() to avoid modulo bias.
 * - If max == 0: fills with 0.
 * - If rng == NULL or out == NULL: no-op.
 */
void byul_rng_fill_range_u32(byul_rng_t* rng, uint32_t* out, size_t count, uint32_t max);

/* Fill out[0..count) with values in [0, max).
 * - Uses byul_rng_range_u64() to avoid modulo bias.
 * - If max == 0: fills with 0.
 * - If rng == NULL or out == NULL: no-op.
 */
void byul_rng_fill_range_u64(byul_rng_t* rng, uint64_t* out, size_t count, uint64_t max);

/* -------------------------------------------------------------------------- */
/* Floating-point fills                                                       */
/* -------------------------------------------------------------------------- */

/* Fill out[0..count) with values in [0, 1).
 * - Uses byul_rng_f32().
 * - If rng == NULL or out == NULL: no-op.
 */
void byul_rng_fill_f32(byul_rng_t* rng, float* out, size_t count);

/* Fill out[0..count) with values in [0, 1).
 * - Uses byul_rng_f64().
 * - If rng == NULL or out == NULL: no-op.
 */
void byul_rng_fill_f64(byul_rng_t* rng, double* out, size_t count);

/* Fill out[0..count) with values in [min, max).
 * - Uses byul_rng_range_f64().
 * - If max < min, implementation should swap them (predictable behavior).
 * - If min == max, fills with min.
 * - If rng == NULL or out == NULL: no-op.
 */
void byul_rng_fill_range_f64(byul_rng_t* rng, double* out, size_t count, double min, double max);

/* Signed integer fills */
void byul_rng_fill_range_i32(byul_rng_t* rng,
                             int32_t* out,
                             size_t count,
                             int32_t min,
                             int32_t max);

void byul_rng_fill_range_i64(byul_rng_t* rng,
                             int64_t* out,
                             size_t count,
                             int64_t min,
                             int64_t max);


#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* BYUL_RNG_FILL_H */
