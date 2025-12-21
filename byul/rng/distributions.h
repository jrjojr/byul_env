/* distributions.h
 *
 * Statistical distribution samplers built on top of rng.h.
 *
 * Design goals:
 * - English-only (MSVC friendly)
 * - Deterministic when you pass a byul_rng_t*
 * - Small, C99-compatible API (works in C++ via extern "C")
 *
 * Notes:
 * - This header provides NO macros and NO inline wrappers.
 * - These functions generate samples following specific probability distributions.
 * - The core RNG (byul_rng_*) must already exist and be initialized by the caller.
 *
 * Module contrast:
 * - rng        : raw random numbers (u32/u64/f32/f64, bounded ranges)
 * - roll       : discrete selection helpers (dice, pick, weighted pick, take)
 * - shuffle    : in-place permutation (Fisher–Yates)
 * - distributions : statistical samplers (normal, exponential, etc.)
 */

#ifndef BYUL_DISTRIBUTIONS_H
#define BYUL_DISTRIBUTIONS_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#include "rng.h"

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------- */
/* Continuous distributions                                                   */
/* -------------------------------------------------------------------------- */

/* Uniform real in [min, max).
 * - If max < min, the implementation should swap them (or behave predictably).
 * - If min == max, returns min.
 *
 * Note:
 * - This is a thin semantic wrapper around byul_rng_range_f64().
 */
double byul_dist_uniform_f64(byul_rng_t* rng, double min, double max);

/* Normal (Gaussian) distribution N(mean, stddev^2).
 * - stddev must be >= 0. If stddev == 0, returns mean.
 *
 * Implementation note:
 * - A typical implementation uses Box–Muller or Marsaglia polar method.
 * - If a cached spare sample is used, the cache must be stored outside of byul_rng_t
 *   (or in a separate distribution state struct) to keep rng.h minimal.
 */
double byul_dist_normal_f64(byul_rng_t* rng, double mean, double stddev);

/* Exponential distribution with rate lambda (> 0).
 * - Returns x >= 0.
 * - If lambda <= 0, returns 0.
 *
 * Typical formula:
 * - x = -log(1 - U) / lambda, where U ~ Uniform[0,1).
 */
double byul_dist_exponential_f64(byul_rng_t* rng, double lambda);

/* Triangular distribution on [a, b] with mode c.
 * - Requires a <= c <= b for the standard definition.
 * - If inputs are invalid, implementation should behave predictably (e.g., clamp c).
 *
 * Useful for "game-feel" randomness: more central mass than uniform.
 */
double byul_dist_triangular_f64(byul_rng_t* rng, double a, double b, double c);

/* -------------------------------------------------------------------------- */
/* Discrete distributions                                                     */
/* -------------------------------------------------------------------------- */

/* Bernoulli distribution: returns true with probability p.
 * - p is interpreted as [0,1]. Values outside are clamped.
 *
 * Note:
 * - This is a semantic wrapper around byul_rng_chance_f64().
 */
bool byul_dist_bernoulli(byul_rng_t* rng, double p);

/* Poisson distribution with mean lambda (>= 0).
 * - Returns k >= 0.
 * - If lambda <= 0, returns 0.
 *
 * Implementation note:
 * - For small lambda, Knuth's algorithm is common.
 * - For large lambda, alternative methods may be preferred for performance,
 *   but the initial implementation can target small/moderate lambda.
 */
uint32_t byul_dist_poisson_u32(byul_rng_t* rng, double lambda);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* BYUL_DISTRIBUTIONS_H */
