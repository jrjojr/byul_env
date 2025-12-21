/* rng_fill.cpp
 *
 * RNG batch helpers: fill user-provided arrays with random values.
 *
 * Design goals:
 * - English-only (MSVC friendly)
 * - Deterministic when you pass a byul_rng_t*
 * - No allocations, no macros, no inline wrappers
 *
 * Notes:
 * - Functions are no-ops when rng == NULL or out == NULL.
 * - Range helpers behave predictably when bounds are invalid:
 *   - For [0,max): if max == 0 -> fill with 0.
 *   - For [min,max): if max <= min -> fill with min.
 */

#include "rng_fill.h"

#include <stdint.h>
#include <stddef.h>

extern "C" void byul_rng_fill_u32(byul_rng_t* rng, uint32_t* out, size_t count)
{
    if (!rng || !out) return;

    for (size_t i = 0; i < count; ++i)
        out[i] = byul_rng_u32(rng);
}

extern "C" void byul_rng_fill_u64(byul_rng_t* rng, uint64_t* out, size_t count)
{
    if (!rng || !out) return;

    for (size_t i = 0; i < count; ++i)
        out[i] = byul_rng_u64(rng);
}

extern "C" void byul_rng_fill_range_u32(byul_rng_t* rng, uint32_t* out, size_t count, uint32_t max)
{
    if (!rng || !out) return;

    if (max == 0u)
    {
        for (size_t i = 0; i < count; ++i)
            out[i] = 0u;
        return;
    }

    for (size_t i = 0; i < count; ++i)
        out[i] = byul_rng_range_u32(rng, max);
}

extern "C" void byul_rng_fill_range_u64(byul_rng_t* rng, uint64_t* out, size_t count, uint64_t max)
{
    if (!rng || !out) return;

    if (max == 0ULL)
    {
        for (size_t i = 0; i < count; ++i)
            out[i] = 0ULL;
        return;
    }

    for (size_t i = 0; i < count; ++i)
        out[i] = byul_rng_range_u64(rng, max);
}

extern "C" void byul_rng_fill_f32(byul_rng_t* rng, float* out, size_t count)
{
    if (!rng || !out) return;

    for (size_t i = 0; i < count; ++i)
        out[i] = byul_rng_f32(rng);
}

extern "C" void byul_rng_fill_f64(byul_rng_t* rng, double* out, size_t count)
{
    if (!rng || !out) return;

    for (size_t i = 0; i < count; ++i)
        out[i] = byul_rng_f64(rng);
}

extern "C" void byul_rng_fill_range_f64(byul_rng_t* rng, double* out, size_t count, double min, double max)
{
    if (!rng || !out) return;

    if (max < min)
    {
        const double t = min;
        min = max;
        max = t;
    }

    if (min == max)
    {
        for (size_t i = 0; i < count; ++i)
            out[i] = min;
        return;
    }

    for (size_t i = 0; i < count; ++i)
        out[i] = byul_rng_range_f64(rng, min, max);
}

/* -------------------------------------------------------------------------- */
/* Optional signed min/max fills                                               */
/* -------------------------------------------------------------------------- */
/* If you add these declarations to rng_fill.h, this implementation is ready.
 *
 * int32: fill [min, max) (predictable: if max <= min -> min)
 * int64: fill [min, max) (predictable: if max <= min -> min)
 */

extern "C" void byul_rng_fill_range_i32(byul_rng_t* rng,
                                       int32_t* out,
                                       size_t count,
                                       int32_t min,
                                       int32_t max)
{
    if (!rng || !out) return;

    if (max <= min)
    {
        for (size_t i = 0; i < count; ++i)
            out[i] = min;
        return;
    }

    const uint64_t width = (uint64_t)((int64_t)max - (int64_t)min); /* fits */
    for (size_t i = 0; i < count; ++i)
    {
        const uint64_t r = byul_rng_range_u64(rng, width); /* [0,width) */
        out[i] = (int32_t)((int64_t)min + (int64_t)r);
    }
}

extern "C" void byul_rng_fill_range_i64(byul_rng_t* rng,
                                       int64_t* out,
                                       size_t count,
                                       int64_t min,
                                       int64_t max)
{
    if (!rng || !out) return;

    if (max <= min)
    {
        for (size_t i = 0; i < count; ++i)
            out[i] = min;
        return;
    }

    /* Use 128-bit intermediate to avoid overflow in (max - min). */
#if defined(__SIZEOF_INT128__)
    const __int128 diff = (__int128)max - (__int128)min; /* > 0 */
    const uint64_t width = (uint64_t)diff;               /* fits (<= 2^64-1) */
#else
    /* Fallback: this is safe for most practical ranges, but extreme ranges may overflow. */
    const uint64_t width = (uint64_t)(max - min);
#endif

    for (size_t i = 0; i < count; ++i)
    {
        const uint64_t r = byul_rng_range_u64(rng, width); /* [0,width) */
#if defined(__SIZEOF_INT128__)
        const __int128 v = (__int128)min + (__int128)r;
        out[i] = (int64_t)v;
#else
        out[i] = (int64_t)(min + (int64_t)r);
#endif
    }
}
