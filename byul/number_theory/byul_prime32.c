#include "byul_prime32.h"
#include "byul_base_primes_65535.h"

bool byul_is_prime_u32(uint32_t n)
{
    if (n < 2) return false;
    if (n == 2 || n == 3) return true;
    if ((n & 1u) == 0) return false;

    /* trial divide by base primes up to sqrt(n) */
    for (uint16_t i = 0; i < byul_base_primes_65535_count; i++) {
        uint32_t p = (uint32_t)BYUL_BASE_PRIME_AT(byul_base_primes_65535, i);
        uint32_t p2 = p * p;
        if (p2 > n) break;
        if ((n % p) == 0) return (n == p);
    }
    return true;
}

uint32_t byul_next_prime_u32(uint32_t n)
{
    if (n <= 2) return 2;
    if ((n & 1u) == 0) n++;

    for (;; n += 2) {
        if (byul_is_prime_u32(n)) return n;
        if (n >= 0xFFFFFFFDu) break; /* prevent overflow on +2 */
    }
    return 0;
}

bool byul_factor_u32_emit(uint32_t n, byul_factor_emit_u32_cb emit, void *user)
{
    if (!emit) return false;
    if (n < 2) return true;

    for (uint16_t i = 0; i < byul_base_primes_65535_count; i++) {
        uint32_t p = (uint32_t)BYUL_BASE_PRIME_AT(byul_base_primes_65535, i);
        uint32_t p2 = p * p;
        if (p2 > n) break;

        if ((n % p) == 0) {
            uint32_t cnt = 0;
            do { n /= p; cnt++; } while ((n % p) == 0);
            if (!emit(p, cnt, user)) return false;
        }
    }

    if (n > 1) {
        /* remaining prime factor */
        if (!emit(n, 1, user)) return false;
    }
    return true;
}
