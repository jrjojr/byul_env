#ifndef BYUL_PRIME32_H
#define BYUL_PRIME32_H

#include "byul_config.h"

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

BYUL_API bool byul_is_prime_u32(uint32_t n);

/* next prime >= n (n<2 => 2). returns 0 on overflow/not found */
BYUL_API uint32_t byul_next_prime_u32(uint32_t n);

/* factorization emit: prime factor p with multiplicity count */
typedef bool (*byul_factor_emit_u32_cb)(uint32_t p, uint32_t count, void *user);
BYUL_API bool byul_factor_u32_emit(uint32_t n, byul_factor_emit_u32_cb emit, void *user);

#ifdef __cplusplus
}
#endif

#endif
