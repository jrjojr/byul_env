/* rng.h
 *
 * Minimal RNG core for byul.
 *
 * Design goals:
 * - English-only (MSVC friendly)
 * - Deterministic when you pass a byul_rng_t*
 * - Small, C99-compatible API (works in C++ via extern "C")
 *
 * Notes:
 * - Higher-level helpers (roll, shuffle, distributions) should include this file.
 */

#ifndef BYUL_RNG_H
#define BYUL_RNG_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------- */
/* RNG state                                                                  */
/* -------------------------------------------------------------------------- */

/* byul_rng_t is an RNG state container (PCG32 in implementation).
 * - state: internal state
 * - inc: stream/sequence selector (must be odd in PCG; enforced in .c/.cpp)
 */
typedef struct s_byul_rng {
    uint64_t state;
    uint64_t inc;
} byul_rng_t;

/* -------------------------------------------------------------------------- */
/* Core API                                                                   */
/* -------------------------------------------------------------------------- */

/* Initialize RNG state. 'seed' can be any value (including 0). */
void byul_rng_init(byul_rng_t* rng, uint64_t seed);

/* Set stream/sequence id (useful for per-system streams).
 * - stream_id can be any value; implementation will map it to a valid stream.
 */
void byul_rng_set_stream(byul_rng_t* rng, uint64_t stream_id);

/* core integers */
uint32_t byul_rng_u32(byul_rng_t* rng);
uint64_t byul_rng_u64(byul_rng_t* rng);

/* bounded integers (unbiased) */
uint32_t byul_rng_range_u32(byul_rng_t* rng, uint32_t max);
uint64_t byul_rng_range_u64(byul_rng_t* rng, uint64_t max);

/* floating point */
float  byul_rng_f32(byul_rng_t* rng);   // [0,1)
double byul_rng_f64(byul_rng_t* rng);   // [0,1)

/* ranges */
double byul_rng_range_f64(byul_rng_t* rng, double min, double max);

float byul_rng_range_f32(byul_rng_t* rng, float min, float max);
int32_t byul_rng_range_i32(byul_rng_t* rng, int32_t min, int32_t max);
int64_t byul_rng_range_i64(byul_rng_t* rng, int64_t min, int64_t max);

/* boolean / chance */
bool byul_rng_bool(byul_rng_t* rng);
bool byul_rng_chance_f64(byul_rng_t* rng, double p);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* BYUL_RNG_H */
