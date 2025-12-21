/* rng.cpp
 *
 * PCG32-based RNG core for byul.
 *
 * Properties:
 * - Deterministic given the same (seed, stream_id).
 * - Provides unbiased bounded integers via rejection sampling.
 * - Provides float/double in [0, 1).
 *
 * Notes:
 * - byul_rng_t stores (state, inc) for PCG32.
 * - inc must be odd; enforced here.
 */

#include "rng.h"

#include <stdint.h>
#include <stdbool.h>

#include <limits>

/* -------------------------------------------------------------------------- */
/* PCG32 core                                                                 */
/* -------------------------------------------------------------------------- */

static inline uint32_t byul__pcg32_next(byul_rng_t* rng)
{
    /* PCG-XSH-RR 64->32 */
    const uint64_t oldstate = rng->state;
    rng->state = oldstate * 6364136223846793005ULL + rng->inc;

    const uint32_t xorshifted = (uint32_t)(((oldstate >> 18u) ^ oldstate) >> 27u);
    const uint32_t rot = (uint32_t)(oldstate >> 59u);
    return (xorshifted >> rot) | (xorshifted << ((uint32_t)(-(int32_t)rot) & 31u));
}

static inline uint64_t byul__pcg_make_inc(uint64_t stream_id)
{
    /* PCG requires inc to be odd. The canonical mapping is (stream<<1)|1. */
    return (stream_id << 1u) | 1u;
}

/* -------------------------------------------------------------------------- */
/* Public API                                                                 */
/* -------------------------------------------------------------------------- */

extern "C" void byul_rng_init(byul_rng_t* rng, uint64_t seed)
{
    if (!rng) return;

    /* Default stream if user never calls set_stream. */
    rng->state = 0ULL;
    rng->inc   = byul__pcg_make_inc(1ULL);

    /* PCG recommended seeding sequence */
    (void)byul__pcg32_next(rng);
    rng->state += seed;
    (void)byul__pcg32_next(rng);
}

extern "C" void byul_rng_set_stream(byul_rng_t* rng, uint64_t stream_id)
{
    if (!rng) return;

    /* Re-seed into the new stream while preserving current state as "seed". */
    const uint64_t seed = rng->state;

    rng->state = 0ULL;
    rng->inc   = byul__pcg_make_inc(stream_id);

    (void)byul__pcg32_next(rng);
    rng->state += seed;
    (void)byul__pcg32_next(rng);
}

extern "C" uint32_t byul_rng_u32(byul_rng_t* rng)
{
    if (!rng) return 0u;
    return byul__pcg32_next(rng);
}

extern "C" uint64_t byul_rng_u64(byul_rng_t* rng)
{
    if (!rng) return 0ULL;

    /* Compose 64 bits from two 32-bit draws. */
    const uint64_t hi = (uint64_t)byul__pcg32_next(rng);
    const uint64_t lo = (uint64_t)byul__pcg32_next(rng);
    return (hi << 32u) | lo;
}

/* -------------------------------------------------------------------------- */
/* Unbiased bounded integers                                                  */
/* -------------------------------------------------------------------------- */

extern "C" uint32_t byul_rng_range_u32(byul_rng_t* rng, uint32_t max)
{
    if (!rng) return 0u;
    if (max == 0u) return 0u;

    /* Rejection sampling to avoid modulo bias. */
    const uint32_t threshold = (uint32_t)(-max) % max;

    for (;;)
    {
        const uint32_t r = byul__pcg32_next(rng);
        if (r >= threshold)
            return r % max;
    }
}

extern "C" uint64_t byul_rng_range_u64(byul_rng_t* rng, uint64_t max)
{
    if (!rng) return 0ULL;
    if (max == 0ULL) return 0ULL;

    /* 64-bit rejection sampling */
    const uint64_t threshold = (uint64_t)(-max) % max;

    for (;;)
    {
        const uint64_t r = byul_rng_u64(rng);
        if (r >= threshold)
            return r % max;
    }
}

/* -------------------------------------------------------------------------- */
/* Floating point                                                             */
/* -------------------------------------------------------------------------- */

extern "C" float byul_rng_f32(byul_rng_t* rng)
{
    if (!rng) return 0.0f;

    /* Make a 24-bit mantissa float in [0,1): r / 2^24 */
    const uint32_t r = byul__pcg32_next(rng);
    const uint32_t mant = r >> 8u; /* top 24 bits */
    return (float)mant * (1.0f / 16777216.0f); /* 2^24 */
}

extern "C" double byul_rng_f64(byul_rng_t* rng)
{
    if (!rng) return 0.0;

    /* Make a 53-bit mantissa double in [0,1): r / 2^53 */
    const uint64_t r = byul_rng_u64(rng);
    const uint64_t mant = r >> 11u; /* top 53 bits */
    return (double)mant * (1.0 / 9007199254740992.0); /* 2^53 */
}

/* -------------------------------------------------------------------------- */
/* Ranges                                                                     */
/* -------------------------------------------------------------------------- */

extern "C" double byul_rng_range_f64(byul_rng_t* rng, double min, double max)
{
    if (!rng) return min;

    /* If user passes reversed range, swap to keep behavior predictable. */
    if (max < min)
    {
        const double t = min;
        min = max;
        max = t;
    }

    const double u = byul_rng_f64(rng); /* [0,1) */
    return min + (max - min) * u;
}

/* -------------------------------------------------------------------------- */
/* Boolean / chance                                                           */
/* -------------------------------------------------------------------------- */

extern "C" bool byul_rng_bool(byul_rng_t* rng)
{
    if (!rng) return false;
    return (byul__pcg32_next(rng) & 1u) != 0u;
}

extern "C" bool byul_rng_chance_f64(byul_rng_t* rng, double p)
{
    if (!rng) return false;

    /* Clamp p to [0,1] */
    if (p <= 0.0) return false;
    if (p >= 1.0) return true;

    return byul_rng_f64(rng) < p;
}

extern "C" float byul_rng_range_f32(byul_rng_t* rng, float min, float max)
{
    if (!rng) return min;

    /* Predictable behavior for invalid bounds */
    if (max <= min)
        return min;

    /* U in [0,1) */
    const float u = byul_rng_f32(rng);
    return min + (max - min) * u;
}

extern "C" int32_t byul_rng_range_i32(byul_rng_t* rng, int32_t min, int32_t max)
{
    if (!rng) return min;

    /* Predictable behavior for invalid bounds */
    if (max <= min)
        return min;

    /* width fits in uint64_t by construction */
    const uint64_t width = (uint64_t)((int64_t)max - (int64_t)min);

    /* [0, width) */
    const uint64_t r = byul_rng_range_u64(rng, width);
    return (int32_t)((int64_t)min + (int64_t)r);
}

extern "C" int64_t byul_rng_range_i64(byul_rng_t* rng, int64_t min, int64_t max)
{
    if (!rng) return min;

    /* Predictable behavior for invalid bounds */
    if (max <= min)
        return min;

#if defined(__SIZEOF_INT128__)
    const __int128 diff = (__int128)max - (__int128)min;
    const uint64_t width = (uint64_t)diff;

    const uint64_t r = byul_rng_range_u64(rng, width);
    const __int128 v = (__int128)min + (__int128)r;
    return (int64_t)v;
#else
    /* Fallback: assumes (max - min) does not overflow int64_t */
    const uint64_t width = (uint64_t)(max - min);

    const uint64_t r = byul_rng_range_u64(rng, width);
    return (int64_t)(min + (int64_t)r);
#endif
}
