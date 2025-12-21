/* shuffle.cpp
 *
 * Fisher–Yates shuffle helpers for the rng module.
 *
 * Implementation notes:
 * - Uses rejection sampling to avoid modulo bias when selecting j in [0..i].
 * - Deterministic given the same byul_rng_t state.
 * - English-only (MSVC friendly)
 */

#include "shuffle.h"

#include <stdint.h>
#include <stddef.h>
#include <string.h>

/* -------------------------------------------------------------------------- */
/* Internal helpers                                                           */
/* -------------------------------------------------------------------------- */

static inline uint32_t byul__bounded_u32(byul_rng_t* rng, uint32_t bound_exclusive)
{
    if (bound_exclusive == 0u) return 0u;

    /* Rejection sampling (Daniel Lemire style threshold trick) */
    const uint32_t threshold = (uint32_t)(-bound_exclusive) % bound_exclusive;

    for (;;)
    {
        const uint32_t r = byul_rng_u32(rng);
        if (r >= threshold)
            return r % bound_exclusive;
    }
}

static inline void byul__swap_bytes(uint8_t* a, uint8_t* b, size_t n)
{
    /* Small stack buffer for typical element sizes; fallback to byte swap. */
    uint8_t tmp[256];

    if (n == 0 || a == b) return;

    if (n <= sizeof(tmp))
    {
        memcpy(tmp, a, n);
        memcpy(a, b, n);
        memcpy(b, tmp, n);
        return;
    }

    /* Large element: swap byte-by-byte to avoid heap allocation. */
    for (size_t i = 0; i < n; ++i)
    {
        const uint8_t t = a[i];
        a[i] = b[i];
        b[i] = t;
    }
}

/* -------------------------------------------------------------------------- */
/* Public API                                                                 */
/* -------------------------------------------------------------------------- */

extern "C" void byul_shuffle(void* base,
                                 size_t count,
                                 size_t elem_size,
                                 byul_rng_t* rng)
{
    if (!base || !rng) return;
    if (elem_size == 0u) return;
    if (count < 2u) return;

    uint8_t* bytes = (uint8_t*)base;

    /* Fisher–Yates: for i = count-1 .. 1, swap i with random j in [0..i] */
    for (size_t i = count - 1u; i > 0u; --i)
    {
        const uint32_t j = byul__bounded_u32(rng, (uint32_t)(i + 1u));

        if ((size_t)j == i)
            continue;

        uint8_t* a = bytes + (i * elem_size);
        uint8_t* b = bytes + ((size_t)j * elem_size);
        byul__swap_bytes(a, b, elem_size);
    }
}

extern "C" void byul_shuffle_i32(int32_t* values, size_t count, byul_rng_t* rng)
{
    byul_shuffle((void*)values, count, sizeof(int32_t), rng);
}

extern "C" void byul_shuffle_u32(uint32_t* values, size_t count, byul_rng_t* rng)
{
    byul_shuffle((void*)values, count, sizeof(uint32_t), rng);
}
