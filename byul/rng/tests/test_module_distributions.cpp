// test_module_distributions.cpp

#include "doctest.h"

#include "rng.h"
#include "distributions.h"

#include <stdint.h>
#include <math.h>

static inline bool byul__isfinite(double x)
{
#if defined(_MSC_VER)
    return _finite(x) != 0;
#else
    return isfinite(x) != 0;
#endif
}

TEST_CASE("dist_uniform_f64: returns within [min,max) and handles swapped bounds")
{
    byul_rng_t rng;
    byul_rng_init(&rng, 123ULL);

    for (int i = 0; i < 5000; ++i)
    {
        const double v = byul_dist_uniform_f64(&rng, -2.0, 5.0);
        CHECK(v >= -2.0);
        CHECK(v < 5.0);
    }

    for (int i = 0; i < 5000; ++i)
    {
        const double v = byul_dist_uniform_f64(&rng, 5.0, -2.0);
        CHECK(v >= -2.0);
        CHECK(v < 5.0);
    }

    CHECK(byul_dist_uniform_f64(&rng, 7.0, 7.0) == doctest::Approx(7.0));
}

TEST_CASE("dist_bernoulli: clamps p and matches extremes")
{
    byul_rng_t rng;
    byul_rng_init(&rng, 1ULL);

    CHECK(byul_dist_bernoulli(&rng, -1.0) == false);
    CHECK(byul_dist_bernoulli(&rng, 0.0) == false);
    CHECK(byul_dist_bernoulli(&rng, 1.0) == true);
    CHECK(byul_dist_bernoulli(&rng, 2.0) == true);
}

TEST_CASE("dist_exponential_f64: returns >= 0 and handles invalid lambda")
{
    byul_rng_t rng;
    byul_rng_init(&rng, 42ULL);

    CHECK(byul_dist_exponential_f64(&rng, 0.0) == doctest::Approx(0.0));
    CHECK(byul_dist_exponential_f64(&rng, -3.0) == doctest::Approx(0.0));

    for (int i = 0; i < 5000; ++i)
    {
        const double x = byul_dist_exponential_f64(&rng, 2.0);
        CHECK(x >= 0.0);
        CHECK(byul__isfinite(x));
    }
}

TEST_CASE("dist_triangular_f64: returns within [a,b] and clamps mode")
{
    byul_rng_t rng;
    byul_rng_init(&rng, 777ULL);

    for (int i = 0; i < 5000; ++i)
    {
        const double x = byul_dist_triangular_f64(&rng, -1.0, 3.0, 2.0);
        CHECK(x >= -1.0);
        CHECK(x <= 3.0);
        CHECK(byul__isfinite(x));
    }

    /* reversed bounds should still behave predictably */
    for (int i = 0; i < 5000; ++i)
    {
        const double x = byul_dist_triangular_f64(&rng, 3.0, -1.0, 2.0);
        CHECK(x >= -1.0);
        CHECK(x <= 3.0);
        CHECK(byul__isfinite(x));
    }

    /* mode outside range should be clamped (smoke test) */
    for (int i = 0; i < 2000; ++i)
    {
        const double x = byul_dist_triangular_f64(&rng, 0.0, 1.0, 999.0);
        CHECK(x >= 0.0);
        CHECK(x <= 1.0);
        CHECK(byul__isfinite(x));
    }
}

TEST_CASE("dist_normal_f64: stddev==0 returns mean, stddev>0 returns finite values")
{
    byul_rng_t rng;
    byul_rng_init(&rng, 2024ULL);

    for (int i = 0; i < 100; ++i)
    {
        const double x = byul_dist_normal_f64(&rng, 10.0, 0.0);
        CHECK(x == doctest::Approx(10.0));
    }

    for (int i = 0; i < 10000; ++i)
    {
        const double x = byul_dist_normal_f64(&rng, 0.0, 1.0);
        CHECK(byul__isfinite(x));
    }
}

TEST_CASE("dist_normal_f64: sample mean is near expected (rough, non-flaky)")
{
    byul_rng_t rng;
    byul_rng_init(&rng, 9999ULL);

    const double mean = 2.5;
    const double stddev = 1.25;

    const int N = 20000;
    double sum = 0.0;

    for (int i = 0; i < N; ++i)
        sum += byul_dist_normal_f64(&rng, mean, stddev);

    const double sample_mean = sum / (double)N;

    CHECK(sample_mean == doctest::Approx(mean).epsilon(0.05));
}

TEST_CASE("dist_poisson_u32: lambda<=0 returns 0, lambda>0 returns non-negative")
{
    byul_rng_t rng;
    byul_rng_init(&rng, 31415ULL);

    CHECK(byul_dist_poisson_u32(&rng, 0.0) == 0u);
    CHECK(byul_dist_poisson_u32(&rng, -1.0) == 0u);

    for (int i = 0; i < 10000; ++i)
    {
        const uint32_t k = byul_dist_poisson_u32(&rng, 2.0);
        /* non-negative by type; just smoke */
        (void)k;
    }
}

TEST_CASE("dist_poisson_u32: sample mean is near lambda (rough, non-flaky)")
{
    byul_rng_t rng;
    byul_rng_init(&rng, 271828ULL);

    const double lambda = 3.0;
    const int N = 20000;

    double sum = 0.0;
    for (int i = 0; i < N; ++i)
        sum += (double)byul_dist_poisson_u32(&rng, lambda);

    const double sample_mean = sum / (double)N;

    CHECK(sample_mean == doctest::Approx(lambda).epsilon(0.05));
}

TEST_CASE("distributions: deterministic for same seed")
{
    byul_rng_t a;
    byul_rng_t b;

    byul_rng_init(&a, 123456ULL);
    byul_rng_init(&b, 123456ULL);

    for (int i = 0; i < 1000; ++i)
    {
        CHECK(byul_dist_uniform_f64(&a, -5.0, 5.0) == byul_dist_uniform_f64(&b, -5.0, 5.0));
        CHECK(byul_dist_exponential_f64(&a, 1.5) == byul_dist_exponential_f64(&b, 1.5));
        CHECK(byul_dist_triangular_f64(&a, 0.0, 1.0, 0.3) == byul_dist_triangular_f64(&b, 0.0, 1.0, 0.3));
        CHECK(byul_dist_normal_f64(&a, 2.0, 3.0) == byul_dist_normal_f64(&b, 2.0, 3.0));
        CHECK(byul_dist_poisson_u32(&a, 2.0) == byul_dist_poisson_u32(&b, 2.0));
    }
}
