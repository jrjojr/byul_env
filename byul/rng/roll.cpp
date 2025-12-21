/* roll.cpp
 *
 * "Roll" helpers built on top of rng.cpp (byul_rng_*).
 *
 * Design goals:
 * - English-only (MSVC friendly)
 * - Deterministic when you pass a byul_rng_t*
 * - C99-compatible API surface (implemented here in C++).
 *
 * Notes:
 * - byul_roll(...) returns [1..sides] (0 if sides==0).
 * - Weighted operations use uint64_t accumulation for safety.
 */

#include "roll.h"

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* -------------------------------------------------------------------------- */
/* Public API                                                                 */
/* -------------------------------------------------------------------------- */

extern "C" uint32_t byul_roll(byul_rng_t* rng, uint32_t sides)
{
    if (!rng) return 0u;
    if (sides == 0u) return 0u;

    /* rng_range_u32 gives [0..sides) without modulo bias */
    return byul_rng_range_u32(rng, sides) + 1u;
}

extern "C" int32_t byul_roll_pick(const int32_t* values, size_t count, byul_rng_t* rng)
{
    if (!rng) return 0;
    if (!values) return 0;
    if (count == 0u) return 0;

    const uint32_t idx = byul_rng_range_u32(rng, (uint32_t)count);
    return values[(size_t)idx];
}

extern "C" int32_t byul_roll_pick_weighted(const int32_t* values,
                                           const uint32_t* weights,
                                           size_t count,
                                           byul_rng_t* rng)
{
    if (!rng) return 0;
    if (!values || !weights) return 0;
    if (count == 0u) return 0;

    uint64_t total = 0ULL;
    for (size_t i = 0; i < count; ++i)
        total += (uint64_t)weights[i];

    if (total == 0ULL)
        return 0;

    /* Draw r in [0..total) */
    const uint64_t r = byul_rng_range_u64(rng, total);

    uint64_t acc = 0ULL;
    for (size_t i = 0; i < count; ++i)
    {
        const uint64_t w = (uint64_t)weights[i];
        if (w == 0ULL) continue;

        acc += w;
        if (r < acc)
            return values[i];
    }

    /* Should never happen, but keep a safe fallback. */
    return values[count - 1u];
}

extern "C" bool byul_roll_check_percent(byul_rng_t* rng, uint32_t percent)
{
    if (!rng) return false;

    if (percent == 0u) return false;
    if (percent >= 100u) return true;

    /* percent% == percent / 100.0 */
    const double p = (double)percent * 0.01;
    return byul_rng_chance_f64(rng, p);
}

extern "C" int32_t byul_roll_take(int32_t* pool, size_t* count, byul_rng_t* rng)
{
    if (!rng) return 0;
    if (!pool || !count) return 0;
    if (*count == 0u) return 0;

    const size_t n = *count;

    /* pick index in [0..n) */
    const uint32_t idx_u32 = byul_rng_range_u32(rng, (uint32_t)n);
    const size_t idx = (size_t)idx_u32;

    const int32_t chosen = pool[idx];

    /* swap-remove */
    const size_t last = n - 1u;
    pool[idx] = pool[last];
    *count = last;

    return chosen;
}

extern "C" int32_t byul_roll_take_weighted(int32_t* pool,
                                          uint32_t* weights,
                                          size_t* count,
                                          byul_rng_t* rng)
{
    if (!rng) return 0;
    if (!pool || !weights || !count) return 0;
    if (*count == 0u) return 0;

    const size_t n = *count;

    uint64_t total = 0ULL;
    for (size_t i = 0; i < n; ++i)
        total += (uint64_t)weights[i];

    if (total == 0ULL)
        return 0;

    const uint64_t r = byul_rng_range_u64(rng, total);

    uint64_t acc = 0ULL;
    size_t chosen_idx = (size_t)-1;

    for (size_t i = 0; i < n; ++i)
    {
        const uint64_t w = (uint64_t)weights[i];
        if (w == 0ULL) continue;

        acc += w;
        if (r < acc)
        {
            chosen_idx = i;
            break;
        }
    }

    if (chosen_idx == (size_t)-1)
        chosen_idx = n - 1u; /* defensive fallback */

    const int32_t chosen = pool[chosen_idx];

    /* swap-remove on both arrays */
    const size_t last = n - 1u;
    pool[chosen_idx] = pool[last];
    weights[chosen_idx] = weights[last];

    *count = last;

    return chosen;
}
