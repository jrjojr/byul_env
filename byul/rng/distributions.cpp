/* distributions.cpp
 *
 * Statistical distribution samplers built on top of rng.cpp.
 *
 * Design goals:
 * - English-only (MSVC friendly)
 * - Deterministic when you pass a byul_rng_t*
 * - Small API surface (declared in distributions.h)
 *
 * Notes:
 * - No macros, no inline wrappers.
 * - Normal distribution uses Box–Muller (no caching to keep RNG state minimal).
 * - Poisson uses Knuth's algorithm (good for small/moderate lambda).
 */

#include "distributions.h"

#include <stdint.h>
#include <stdbool.h>

#include <math.h>   /* log, sqrt, cos, sin */
#include <float.h>  /* DBL_MIN */

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* -------------------------------------------------------------------------- */
/* Helpers                                                                    */
/* -------------------------------------------------------------------------- */

static inline double byul__clamp_f64(double x, double lo, double hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

/* -------------------------------------------------------------------------- */
/* Continuous                                                                  */
/* -------------------------------------------------------------------------- */

extern "C" double byul_dist_uniform_f64(byul_rng_t* rng, double min, double max)
{
    if (!rng) return min;

    if (max < min)
    {
        const double t = min;
        min = max;
        max = t;
    }

    if (min == max)
        return min;

    return byul_rng_range_f64(rng, min, max);
}

extern "C" double byul_dist_normal_f64(byul_rng_t* rng, double mean, double stddev)
{
    if (!rng) return mean;
    if (stddev <= 0.0) return mean;

    /* Box–Muller transform:
       z0 = sqrt(-2 ln U1) * cos(2π U2), where U1,U2 ~ Uniform(0,1]
       We use U1 in (0,1) and clamp away from 0 to avoid log(0). */
    double u1 = byul_rng_f64(rng); /* [0,1) */
    double u2 = byul_rng_f64(rng); /* [0,1) */

    /* Ensure u1 is not 0 (or too close), to keep log(u1) valid. */
    if (u1 < DBL_MIN) u1 = DBL_MIN;

    const double r = sqrt(-2.0 * log(u1));
    const double theta = 2.0 * (double)M_PI * u2;

    const double z0 = r * cos(theta);
    return mean + z0 * stddev;
}

extern "C" double byul_dist_exponential_f64(byul_rng_t* rng, double lambda)
{
    if (!rng) return 0.0;
    if (lambda <= 0.0) return 0.0;

    /* Use U in (0,1] to avoid log(0).
       With U in [0,1), use (1 - U) in (0,1]. */
    double u = byul_rng_f64(rng); /* [0,1) */
    double one_minus_u = 1.0 - u;
    if (one_minus_u < DBL_MIN) one_minus_u = DBL_MIN;

    return -log(one_minus_u) / lambda;
}

extern "C" double byul_dist_triangular_f64(byul_rng_t* rng, double a, double b, double c)
{
    if (!rng) return a;

    /* Normalize ordering */
    if (b < a)
    {
        const double t = a;
        a = b;
        b = t;
    }

    if (a == b)
        return a;

    /* Clamp mode into [a,b] for predictable behavior. */
    c = byul__clamp_f64(c, a, b);

    const double u = byul_rng_f64(rng); /* [0,1) */

    const double fc = (c - a) / (b - a); /* CDF at mode */

    if (u < fc)
    {
        /* a + sqrt(u*(b-a)*(c-a)) */
        return a + sqrt(u * (b - a) * (c - a));
    }
    else
    {
        /* b - sqrt((1-u)*(b-a)*(b-c)) */
        return b - sqrt((1.0 - u) * (b - a) * (b - c));
    }
}

/* -------------------------------------------------------------------------- */
/* Discrete                                                                    */
/* -------------------------------------------------------------------------- */

extern "C" bool byul_dist_bernoulli(byul_rng_t* rng, double p)
{
    if (!rng) return false;

    /* Clamp p to [0,1] */
    p = byul__clamp_f64(p, 0.0, 1.0);
    return byul_rng_chance_f64(rng, p);
}

extern "C" uint32_t byul_dist_poisson_u32(byul_rng_t* rng, double lambda)
{
    if (!rng) return 0u;
    if (lambda <= 0.0) return 0u;

    /* Knuth's algorithm:
       L = exp(-lambda)
       k = 0
       p = 1
       do:
         k++
         p *= U
       while p > L
       return k-1
     */
    const double L = exp(-lambda);

    uint32_t k = 0u;
    double p = 1.0;

    for (;;)
    {
        ++k;
        const double u = byul_rng_f64(rng); /* [0,1) */
        p *= u;

        if (p <= L)
            break;

        /* Defensive: avoid potential runaway if lambda is huge.
           This algorithm is not intended for very large lambda. */
        if (k == 0xFFFFFFFFu)
            break;
    }

    return k - 1u;
}
