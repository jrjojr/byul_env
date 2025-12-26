/* rng.h */

#ifndef BYUL_RNG_H
#define BYUL_RNG_H

#include "rng_core.h"
#include "roll.h"
#include "shuffle.h"
#include "distributions.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Returns module version string, e.g. "rng 1.0.0 (byul 0.9.3)". */
const char* byul_rng_version_string(void);

/* Optional: numeric version */
void byul_rng_version(int* major, int* minor, int* patch);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* BYUL_RNG_H */
