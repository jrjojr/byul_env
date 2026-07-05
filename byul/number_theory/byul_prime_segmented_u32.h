#ifndef BYUL_PRIME_SEGMENTED_U32_H
#define BYUL_PRIME_SEGMENTED_U32_H

#include "byul_config.h"

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Callback returns:
 *  - true  : continue
 *  - false : stop early
 */
typedef bool (*byul_prime_emit_u32_cb)(uint32_t prime, void *user);

/*
 * Build base primes up to floor(sqrt(end_inclusive)).
 *
 * - primes_out: array of uint16_t (because sqrt(2^32) <= 65535)
 * - sieve_bits: bitset for odds in [3..limit], caller provides
 *
 * Returns number of primes written to primes_out.
 *
 * NOTE:
 *  - You must size sieve_bits to cover odds up to limit.
 *  - Required bits = count of odd numbers from 3..limit inclusive
 *                  = ((limit - 3) / 2) + 1  (if limit >= 3)
 */
BYUL_API size_t byul_make_base_primes_u32(
    uint32_t end_inclusive,
    uint16_t *primes_out,
    size_t primes_cap,
    uint8_t *sieve_bits,
    size_t sieve_bits_bytes
);

/*
 * Segmented sieve over [start, end_exclusive)
 * Emits primes via callback.
 *
 * Memory:
 *  - window_bits is a bitset for odds in the current segment window.
 *
 * window_bits_bytes determines window length:
 *  - bits = window_bits_bytes * 8
 *  - odds = bits
 *  - numbers covered = odds * 2  (because only odds stored)
 *
 * Example: 512 bytes -> 4096 odds -> 8192 numbers per window.
 */
BYUL_API void byul_segmented_primes_u32_emit(
    uint32_t start,
    uint32_t end_exclusive,
    const uint16_t *base_primes,
    size_t base_prime_count,
    uint8_t *window_bits,
    size_t window_bits_bytes,
    byul_prime_emit_u32_cb emit,
    void *user
);

#ifdef __cplusplus
}
#endif

#endif /* BYUL_PRIME_SEGMENTED_U32_H */
