#include "doctest.h"

#include "rng.h"
#include "shuffle.h"

#include <stdint.h>

TEST_CASE("shuffle: null/degenerate inputs are no-ops")
{
    byul_rng_t rng;
    byul_rng_init(&rng, 123);

    int32_t a[] = { 1, 2, 3 };
    const int32_t a0 = a[0], a1 = a[1], a2 = a[2];

    // count < 2 -> no-op
    byul_shuffle_i32(a, 1, &rng);
    CHECK(a[0] == a0);

    // elem_size == 0 -> no-op
    byul_shuffle((void*)a, 3, 0, &rng);
    CHECK(a[0] == a0);
    CHECK(a[1] == a1);
    CHECK(a[2] == a2);

    // base == nullptr -> no-op
    byul_shuffle(nullptr, 3, sizeof(int32_t), &rng);

    // rng == nullptr -> no-op
    byul_shuffle((void*)a, 3, sizeof(int32_t), nullptr);
    CHECK(a[0] == a0);
    CHECK(a[1] == a1);
    CHECK(a[2] == a2);
}

TEST_CASE("shuffle_i32: deterministic for the same seed")
{
    byul_rng_t rng1;
    byul_rng_t rng2;
    byul_rng_init(&rng1, 999);
    byul_rng_init(&rng2, 999);

    int32_t a1[] = { 1, 2, 3, 4, 5, 6, 7, 8 };
    int32_t a2[] = { 1, 2, 3, 4, 5, 6, 7, 8 };

    byul_shuffle_i32(a1, 8, &rng1);
    byul_shuffle_i32(a2, 8, &rng2);

    for (int i = 0; i < 8; ++i)
        CHECK(a1[i] == a2[i]);
}

TEST_CASE("shuffle_i32: preserves multiset (no loss/duplication)")
{
    byul_rng_t rng;
    byul_rng_init(&rng, 2025);

    int32_t a[] = { 10, 10, 20, 30, 30, 30, 40, 50 };
    const size_t n = sizeof(a) / sizeof(a[0]);

    // Count occurrences before
    int count10 = 0, count20 = 0, count30 = 0, count40 = 0, count50 = 0;
    for (size_t i = 0; i < n; ++i)
    {
        if (a[i] == 10) count10++;
        else if (a[i] == 20) count20++;
        else if (a[i] == 30) count30++;
        else if (a[i] == 40) count40++;
        else if (a[i] == 50) count50++;
        else CHECK(false); // unexpected value
    }

    byul_shuffle_i32(a, n, &rng);

    // Count occurrences after
    int c10 = 0, c20 = 0, c30 = 0, c40 = 0, c50 = 0;
    for (size_t i = 0; i < n; ++i)
    {
        if (a[i] == 10) c10++;
        else if (a[i] == 20) c20++;
        else if (a[i] == 30) c30++;
        else if (a[i] == 40) c40++;
        else if (a[i] == 50) c50++;
        else CHECK(false); // unexpected value
    }

    CHECK(c10 == count10);
    CHECK(c20 == count20);
    CHECK(c30 == count30);
    CHECK(c40 == count40);
    CHECK(c50 == count50);
}

TEST_CASE("shuffle_u32: works and is deterministic")
{
    byul_rng_t rng1;
    byul_rng_t rng2;
    byul_rng_init(&rng1, 314159);
    byul_rng_init(&rng2, 314159);

    uint32_t a1[] = { 0u, 1u, 2u, 3u, 4u, 5u, 6u, 7u, 8u, 9u };
    uint32_t a2[] = { 0u, 1u, 2u, 3u, 4u, 5u, 6u, 7u, 8u, 9u };

    byul_shuffle_u32(a1, 10, &rng1);
    byul_shuffle_u32(a2, 10, &rng2);

    for (int i = 0; i < 10; ++i)
        CHECK(a1[i] == a2[i]);
}
