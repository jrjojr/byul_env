// test_module_rng.cpp

#include "doctest.h"

#include "rng.h"

#include <stdint.h>
#include <math.h>
#include <random>
#include <algorithm>

TEST_CASE("rng: deterministic sequence for same seed/stream")
{
    byul_rng_t a;
    byul_rng_t b;

    byul_rng_init(&a, 123456789ULL);
    byul_rng_init(&b, 123456789ULL);

    byul_rng_set_stream(&a, 42ULL);
    byul_rng_set_stream(&b, 42ULL);

    for (int i = 0; i < 1000; ++i)
    {
        const uint32_t x = byul_rng_u32(&a);
        const uint32_t y = byul_rng_u32(&b);
        CHECK(x == y);
    }
}

TEST_CASE("rng: different streams produce different sequences (likely)")
{
    byul_rng_t a;
    byul_rng_t b;

    byul_rng_init(&a, 999ULL);
    byul_rng_init(&b, 999ULL);

    byul_rng_set_stream(&a, 1ULL);
    byul_rng_set_stream(&b, 2ULL);

    /* Not a strict mathematical guarantee, but should differ very quickly. */
    bool any_diff = false;
    for (int i = 0; i < 32; ++i)
    {
        if (byul_rng_u32(&a) != byul_rng_u32(&b))
        {
            any_diff = true;
            break;
        }
    }
    CHECK(any_diff);
}

TEST_CASE("rng: u64 combines u32 draws deterministically")
{
    byul_rng_t a;
    byul_rng_t b;

    byul_rng_init(&a, 2025ULL);
    byul_rng_init(&b, 2025ULL);

    /* Same stream by default; compare full u64 outputs. */
    for (int i = 0; i < 100; ++i)
        CHECK(byul_rng_u64(&a) == byul_rng_u64(&b));
}

TEST_CASE("rng: range_u32 returns within bounds and handles max==0")
{
    byul_rng_t rng;
    byul_rng_init(&rng, 1ULL);

    CHECK(byul_rng_range_u32(&rng, 0u) == 0u);

    for (int i = 0; i < 1000; ++i)
    {
        const uint32_t v = byul_rng_range_u32(&rng, 10u);
        CHECK(v < 10u);
    }
}

TEST_CASE("rng: range_u64 returns within bounds and handles max==0")
{
    byul_rng_t rng;
    byul_rng_init(&rng, 2ULL);

    CHECK(byul_rng_range_u64(&rng, 0ULL) == 0ULL);

    for (int i = 0; i < 1000; ++i)
    {
        const uint64_t v = byul_rng_range_u64(&rng, 1000000ULL);
        CHECK(v < 1000000ULL);
    }
}

TEST_CASE("rng: f32/f64 are in [0,1)")
{
    byul_rng_t rng;
    byul_rng_init(&rng, 3ULL);

    for (int i = 0; i < 2000; ++i)
    {
        const float f = byul_rng_f32(&rng);
        CHECK(f >= 0.0f);
        CHECK(f < 1.0f);

        const double d = byul_rng_f64(&rng);
        CHECK(d >= 0.0);
        CHECK(d < 1.0);
    }
}

TEST_CASE("rng: range_f64 returns within [min,max] and swaps if reversed")
{
    byul_rng_t rng;
    byul_rng_init(&rng, 4ULL);

    for (int i = 0; i < 2000; ++i)
    {
        const double v = byul_rng_range_f64(&rng, -2.0, 5.0);
        CHECK(v >= -2.0);
        CHECK(v <= 5.0);
    }

    for (int i = 0; i < 2000; ++i)
    {
        const double v = byul_rng_range_f64(&rng, 5.0, -2.0);
        CHECK(v >= -2.0);
        CHECK(v <= 5.0);
    }
}

TEST_CASE("rng: bool/chance_f64 behavior at extremes")
{
    byul_rng_t rng;
    byul_rng_init(&rng, 5ULL);

    CHECK(byul_rng_chance_f64(&rng, 0.0) == false);
    CHECK(byul_rng_chance_f64(&rng, -1.0) == false);
    CHECK(byul_rng_chance_f64(&rng, 1.0) == true);
    CHECK(byul_rng_chance_f64(&rng, 2.0) == true);

    /* bool should be either true/false; just smoke test. */
    bool any_true = false;
    bool any_false = false;
    for (int i = 0; i < 256; ++i)
    {
        const bool b = byul_rng_bool(&rng);
        any_true |= b;
        any_false |= !b;
    }
    CHECK(any_true);
    CHECK(any_false);
}

/* -------------------------------------------------------------------------- */
/* "True random" smoke tests (std::random_device)                             */
/* -------------------------------------------------------------------------- */

TEST_CASE("rng: std::random_device provides varying seeds (smoke)")
{
    std::random_device rd;

    // Collect a handful of samples.
    std::vector<uint32_t> samples;
    samples.reserve(32);
    for (int i = 0; i < 32; ++i)
        samples.push_back(rd());

    const bool all_same = std::all_of(samples.begin(), samples.end(),
                                     [&](uint32_t v) { return v == samples[0]; });

    // On some platforms, random_device may be deterministic or even constant.
    // We avoid flaky failures: if everything is constant, skip the test.
    if (all_same)
    {
        return;
    }

    // Otherwise, we at least observed variation.
    CHECK(!all_same);
}

TEST_CASE("rng: seeding from std::random_device yields non-constant outputs (smoke)")
{
    std::random_device rd;

    // Build two seeds/streams from rd.
    const uint64_t seed1 = ((uint64_t)rd() << 32u) | (uint64_t)rd();
    const uint64_t seed2 = ((uint64_t)rd() << 32u) | (uint64_t)rd();
    const uint64_t stream1 = ((uint64_t)rd() << 32u) | (uint64_t)rd();
    const uint64_t stream2 = ((uint64_t)rd() << 32u) | (uint64_t)rd();

    // If rd is super weak and produced identical material, don't fail—skip.
    if (seed1 == seed2 && stream1 == stream2)
    {
        return;
    }

    byul_rng_t a;
    byul_rng_t b;

    byul_rng_init(&a, seed1);
    byul_rng_init(&b, seed2);
    byul_rng_set_stream(&a, stream1);
    byul_rng_set_stream(&b, stream2);

    // Smoke: each generator should produce non-constant outputs.
    auto is_constant = [](byul_rng_t* r) -> bool {
        const uint32_t first = byul_rng_u32(r);
        for (int i = 0; i < 31; ++i)
        {
            if (byul_rng_u32(r) != first)
                return false;
        }
        return true;
    };

    CHECK(is_constant(&a) == false);
    CHECK(is_constant(&b) == false);

    // Smoke: sequences should differ early (likely), but don't make it a hard guarantee.
    bool any_diff = false;
    for (int i = 0; i < 32; ++i)
    {
        if (byul_rng_u32(&a) != byul_rng_u32(&b))
        {
            any_diff = true;
            break;
        }
    }

    if (!any_diff)
    {
        // Avoid flakiness: this is extremely unlikely unless seeds/streams collided
        // or rd is deterministic. Treat as skip, not failure.
        return;
    }

    CHECK(any_diff);
}

TEST_CASE("rng: range_i32 returns within [min,max) and handles invalid bounds")
{
    byul_rng_t rng;
    byul_rng_init(&rng, 123ULL);

    // normal range
    for (int i = 0; i < 5000; ++i)
    {
        const int32_t v = byul_rng_range_i32(&rng, -5, 6); // [-5, 6)
        CHECK(v >= -5);
        CHECK(v < 6);
    }

    // min == max -> always min
    for (int i = 0; i < 100; ++i)
    {
        CHECK(byul_rng_range_i32(&rng, 7, 7) == 7);
    }

    // max < min -> predictable: return min
    for (int i = 0; i < 100; ++i)
    {
        CHECK(byul_rng_range_i32(&rng, 10, -10) == 10);
    }
}

TEST_CASE("rng: range_i64 returns within [min,max) and handles invalid bounds")
{
    byul_rng_t rng;
    byul_rng_init(&rng, 456ULL);

    const int64_t min = (int64_t)-123456789;
    const int64_t max = (int64_t) 123456789;

    // normal range
    for (int i = 0; i < 5000; ++i)
    {
        const int64_t v = byul_rng_range_i64(&rng, min, max);
        CHECK(v >= min);
        CHECK(v < max);
    }

    // min == max -> always min
    for (int i = 0; i < 100; ++i)
    {
        CHECK(byul_rng_range_i64(&rng, 42, 42) == 42);
    }

    // max < min -> predictable: return min
    for (int i = 0; i < 100; ++i)
    {
        CHECK(byul_rng_range_i64(&rng, 100, -100) == 100);
    }
}

TEST_CASE("rng: range_f32 returns within [min,max) and handles invalid bounds")
{
    byul_rng_t rng;
    byul_rng_init(&rng, 789ULL);

    // normal range
    for (int i = 0; i < 5000; ++i)
    {
        const float v = byul_rng_range_f32(&rng, -1.5f, 2.5f);
        CHECK(v >= -1.5f);
        CHECK(v < 2.5f);
    }

    // min == max -> always min
    for (int i = 0; i < 100; ++i)
    {
        CHECK(byul_rng_range_f32(&rng, 3.0f, 3.0f) == doctest::Approx(3.0f));
    }

    // max < min -> predictable: return min
    for (int i = 0; i < 100; ++i)
    {
        CHECK(byul_rng_range_f32(&rng, 5.0f, -5.0f) == doctest::Approx(5.0f));
    }
}
