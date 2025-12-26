/* rng.cpp */
#include "rng.h"

/* You can wire this to your global project version macros if you have them. */
#ifndef BYUL_RNG_VERSION_MAJOR
#define BYUL_RNG_VERSION_MAJOR 1
#endif
#ifndef BYUL_RNG_VERSION_MINOR
#define BYUL_RNG_VERSION_MINOR 0
#endif
#ifndef BYUL_RNG_VERSION_PATCH
#define BYUL_RNG_VERSION_PATCH 0
#endif

const char* byul_rng_version_string(void)
{
    return "rng " "1.0.0";
}

void byul_rng_version(int* major, int* minor, int* patch)
{
    if (major) *major = BYUL_RNG_VERSION_MAJOR;
    if (minor) *minor = BYUL_RNG_VERSION_MINOR;
    if (patch) *patch = BYUL_RNG_VERSION_PATCH;
}
