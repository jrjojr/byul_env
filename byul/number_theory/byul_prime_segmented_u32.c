#include "byul_prime_segmented_u32.h"
#include <string.h>

/* ---------------- bit helpers (LSB-first) ---------------- */

static inline void byul_bits_set_all(uint8_t *bits, size_t bytes)
{
    memset(bits, 0xFF, bytes);
}

static inline void byul_bits_clear(uint8_t *bits, size_t idx)
{
    bits[idx >> 3] &= (uint8_t)~(1u << (idx & 7));
}

static inline bool byul_bits_test(const uint8_t *bits, size_t idx)
{
    return (bits[idx >> 3] & (uint8_t)(1u << (idx & 7))) != 0;
}

/* ---------------- integer sqrt (floor) for u32 ----------------
 * No float, no recursion. Binary method.
 */
static uint32_t byul_isqrt_u32(uint32_t n)
{
    uint32_t res = 0;
    uint32_t bit = 1UL << 30; /* The second-to-top bit set */

    while (bit > n) bit >>= 2;

    while (bit != 0) {
        if (n >= res + bit) {
            n -= res + bit;
            res = (res >> 1) + bit;
        } else {
            res >>= 1;
        }
        bit >>= 2;
    }
    return res;
}

/* ---------------- base primes up to sqrt(end) ---------------- */

size_t byul_make_base_primes_u32(
    uint32_t end_inclusive,
    uint16_t *primes_out,
    size_t primes_cap,
    uint8_t *sieve_bits,
    size_t sieve_bits_bytes
)
{
    if (!primes_out || primes_cap == 0) return 0;
    if (!sieve_bits || sieve_bits_bytes == 0) return 0;

    const uint32_t limit = byul_isqrt_u32(end_inclusive);
    size_t out_count = 0;

    /* Always include 2 if it fits */
    primes_out[out_count++] = 2;

    if (limit < 3) return out_count;

    /* odds count from 3..limit inclusive */
    const uint32_t odds_count_u32 = ((limit - 3) / 2) + 1;
    const size_t odds_count = (size_t)odds_count_u32;
    const size_t need_bytes = (odds_count + 7) >> 3;
    if (sieve_bits_bytes < need_bytes) {
        /* Not enough memory to build base primes */
        return 0;
    }

    byul_bits_set_all(sieve_bits, need_bytes);

    /* Sieve: represent odd x as idx = (x - 3)/2 */
    for (uint32_t p = 3; (uint32_t)p * (uint32_t)p <= limit; p += 2) {
        const size_t p_idx = (size_t)((p - 3) >> 1);
        if (!byul_bits_test(sieve_bits, p_idx)) continue;

        /* start at p*p, step 2p (odd multiples only) */
        uint32_t j = (uint32_t)p * (uint32_t)p;
        const uint32_t step = (uint32_t)(p << 1);

        for (; j <= limit; j += step) {
            const size_t j_idx = (size_t)((j - 3) >> 1);
            byul_bits_clear(sieve_bits, j_idx);
        }
    }

    /* Collect primes */
    for (uint32_t x = 3; x <= limit; x += 2) {
        const size_t idx = (size_t)((x - 3) >> 1);
        if (byul_bits_test(sieve_bits, idx)) {
            if (out_count >= primes_cap) break;
            primes_out[out_count++] = (uint16_t)x;
        }
    }

    return out_count;
}

/* ---------------- segmented sieve emit ---------------- */

static inline uint32_t byul_u32_max(uint32_t a, uint32_t b) { return (a > b) ? a : b; }

void byul_segmented_primes_u32_emit(
    uint32_t start,
    uint32_t end_exclusive,
    const uint16_t *base_primes,
    size_t base_prime_count,
    uint8_t *window_bits,
    size_t window_bits_bytes,
    byul_prime_emit_u32_cb emit,
    void *user
)
{
    if (!emit) return;
    if (start >= end_exclusive) return;
    if (!base_primes || base_prime_count == 0) return;
    if (!window_bits || window_bits_bytes == 0) return;

    /* Emit 2 if within range */
    if (start <= 2 && 2 < end_exclusive) {
        if (!emit(2, user)) return;
    }

    /* Make start odd for odd-only processing */
    uint32_t cur = start;
    if (cur <= 3) cur = 3;
    if ((cur & 1u) == 0) cur++;

    const uint32_t window_odds = (uint32_t)(window_bits_bytes * 8u);
    if (window_odds == 0) return;

    while (cur < end_exclusive) {
        /* segment covers odd numbers from seg_start to seg_end (exclusive) */
        const uint32_t seg_start = cur;
        uint32_t seg_end = seg_start + (window_odds * 2u);
        if (seg_end < seg_start) seg_end = end_exclusive; /* overflow guard */
        if (seg_end > end_exclusive) seg_end = end_exclusive;

        /* number of odds in this segment */
        uint32_t seg_len = seg_end - seg_start;
        uint32_t seg_odds = (seg_len + 1u) >> 1; /* since seg_start is odd */
        size_t seg_need_bytes = ((size_t)seg_odds + 7) >> 3;

        /* init all as prime (1) */
        byul_bits_set_all(window_bits, seg_need_bytes);

        /* mark composites using base primes (skip 2, we handle odds only) */
        for (size_t i = 0; i < base_prime_count; i++) {
            const uint32_t p = (uint32_t)base_primes[i];
            if (p == 2) continue;

            /* if p*p > seg_end, still may need to mark multiples inside segment when seg_start large.
               but base primes are <= sqrt(end), so OK. */
            uint32_t p2 = p * p;

            /* find first multiple m in [seg_start, seg_end) */
            uint32_t m;
            if (p2 >= seg_start) {
                m = p2;
            } else {
                /* ceil(seg_start / p) * p */
                uint32_t q = seg_start / p;
                if (q * p < seg_start) q++;
                m = q * p;
            }

            /* ensure m is odd (since we store odds only) */
            if ((m & 1u) == 0) m += p;

            const uint32_t step = p << 1; /* 2p */
            for (; m < seg_end; m += step) {
                /* idx of odd m in segment: (m - seg_start)/2 */
                uint32_t idx = (m - seg_start) >> 1;
                byul_bits_clear(window_bits, (size_t)idx);
            }
        }

        /* handle 1 if inside segment (not prime) */
        if (seg_start == 1) {
            byul_bits_clear(window_bits, 0);
        }

        /* emit primes */
        for (uint32_t k = 0; k < seg_odds; k++) {
            if (byul_bits_test(window_bits, (size_t)k)) {
                uint32_t prime = seg_start + (k << 1);
                if (prime >= end_exclusive) break;
                if (!emit(prime, user)) return;
            }
        }

        /* advance to next segment */
        cur = seg_end;
        if ((cur & 1u) == 0) cur++;
    }
}
