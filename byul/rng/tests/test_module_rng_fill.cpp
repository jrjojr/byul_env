// test_module_rng_fill.cpp

#include "doctest.h"

#include "rng.h"
#include "rng_fill.h"

#include <stdint.h>
#include <stddef.h>
#include <cmath>

TEST_CASE("rng_fill_u32/u64: deterministic for same seed")
{
    byul_rng_t a;
    byul_rng_t b;

    byul_rng_init(&a, 12345ULL);
    byul_rng_init(&b, 12345ULL);

    uint32_t out1[256];
    uint32_t out2[256];

    byul_rng_fill_u32(&a, out1, 256);
    byul_rng_fill_u32(&b, out2, 256);

    for (size_t i = 0; i < 256; ++i)
        CHECK(out1[i] == out2[i]);

    /* u64 */
    byul_rng_init(&a, 999ULL);
    byul_rng_init(&b, 999ULL);

    uint64_t out64_1[128];
    uint64_t out64_2[128];

    byul_rng_fill_u64(&a, out64_1, 128);
    byul_rng_fill_u64(&b, out64_2, 128);

    for (size_t i = 0; i < 128; ++i)
        CHECK(out64_1[i] == out64_2[i]);
}

TEST_CASE("rng_fill_range_u32/u64: within [0,max) and max==0 fills zeros")
{
    byul_rng_t rng;
    byul_rng_init(&rng, 1ULL);

    uint32_t out32[512];
    byul_rng_fill_range_u32(&rng, out32, 512, 10u);
    for (size_t i = 0; i < 512; ++i)
        CHECK(out32[i] < 10u);

    uint32_t z32[64];
    byul_rng_fill_range_u32(&rng, z32, 64, 0u);
    for (size_t i = 0; i < 64; ++i)
        CHECK(z32[i] == 0u);

    uint64_t out64[256];
    byul_rng_fill_range_u64(&rng, out64, 256, 1000000ULL);
    for (size_t i = 0; i < 256; ++i)
        CHECK(out64[i] < 1000000ULL);

    uint64_t z64[64];
    byul_rng_fill_range_u64(&rng, z64, 64, 0ULL);
    for (size_t i = 0; i < 64; ++i)
        CHECK(z64[i] == 0ULL);
}

TEST_CASE("rng_fill_f32/f64: values in [0,1)")
{
    byul_rng_t rng;
    byul_rng_init(&rng, 2ULL);

    float out_f32[1024];
    byul_rng_fill_f32(&rng, out_f32, 1024);
    for (size_t i = 0; i < 1024; ++i)
    {
        CHECK(out_f32[i] >= 0.0f);
        CHECK(out_f32[i] < 1.0f);
    }

    double out_f64[1024];
    byul_rng_fill_f64(&rng, out_f64, 1024);
    for (size_t i = 0; i < 1024; ++i)
    {
        CHECK(out_f64[i] >= 0.0);
        CHECK(out_f64[i] < 1.0);
    }
}

TEST_CASE("rng_fill_range_f64: values in [min,max) and handles swapped bounds")
{
    byul_rng_t rng;
    byul_rng_init(&rng, 3ULL);

    double out[2048];

    byul_rng_fill_range_f64(&rng, out, 2048, -2.0, 5.0);
    for (size_t i = 0; i < 2048; ++i)
    {
        CHECK(out[i] >= -2.0);
        CHECK(out[i] < 5.0);
    }

    byul_rng_fill_range_f64(&rng, out, 2048, 5.0, -2.0);
    for (size_t i = 0; i < 2048; ++i)
    {
        CHECK(out[i] >= -2.0);
        CHECK(out[i] < 5.0);
    }

    byul_rng_fill_range_f64(&rng, out, 16, 7.0, 7.0);
    for (size_t i = 0; i < 16; ++i)
        CHECK(out[i] == doctest::Approx(7.0));
}

TEST_CASE("rng_fill_range_i32: values in [min,max) and invalid bounds fill min")
{
    byul_rng_t rng;
    byul_rng_init(&rng, 4ULL);

    int32_t out[4096];

    byul_rng_fill_range_i32(&rng, out, 4096, -5, 6); /* [-5,6) */
    for (size_t i = 0; i < 4096; ++i)
    {
        CHECK(out[i] >= -5);
        CHECK(out[i] < 6);
    }

    byul_rng_fill_range_i32(&rng, out, 32, 10, 10);
    for (size_t i = 0; i < 32; ++i)
        CHECK(out[i] == 10);

    byul_rng_fill_range_i32(&rng, out, 32, 10, -10); /* invalid -> fill min */
    for (size_t i = 0; i < 32; ++i)
        CHECK(out[i] == 10);
}

TEST_CASE("rng_fill_range_i64: values in [min,max) and invalid bounds fill min")
{
    byul_rng_t rng;
    byul_rng_init(&rng, 5ULL);

    int64_t out[2048];

    byul_rng_fill_range_i64(&rng, out, 2048, (int64_t)-123456789, (int64_t)123456789);
    for (size_t i = 0; i < 2048; ++i)
    {
        CHECK(out[i] >= (int64_t)-123456789);
        CHECK(out[i] < (int64_t)123456789);
    }

    byul_rng_fill_range_i64(&rng, out, 16, (int64_t)7, (int64_t)7);
    for (size_t i = 0; i < 16; ++i)
        CHECK(out[i] == (int64_t)7);

    byul_rng_fill_range_i64(&rng, out, 16, (int64_t)7, (int64_t)-7);
    for (size_t i = 0; i < 16; ++i)
        CHECK(out[i] == (int64_t)7);
}

TEST_CASE("rng_fill: no-op on NULL pointers (smoke)")
{
    byul_rng_t rng;
    byul_rng_init(&rng, 6ULL);

    uint32_t out[4] = {0,0,0,0};

    byul_rng_fill_u32(nullptr, out, 4);
    CHECK(out[0] == 0u);

    byul_rng_fill_u32(&rng, nullptr, 4);

    byul_rng_fill_range_u32(nullptr, out, 4, 10u);
    CHECK(out[1] == 0u);

    double dout[4] = {0,0,0,0};
    byul_rng_fill_range_f64(nullptr, dout, 4, 0.0, 1.0);
    CHECK(dout[0] == doctest::Approx(0.0));
}
