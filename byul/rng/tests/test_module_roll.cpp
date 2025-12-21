// test_module_roll.cpp

#include "doctest.h"

#include "rng.h"
#include "roll.h"

#include <stdint.h>

TEST_CASE("roll: byul_roll returns 0 for sides==0 and [1..sides] otherwise")
{
    byul_rng_t rng;
    byul_rng_init(&rng, 123ULL);

    CHECK(byul_roll(&rng, 0u) == 0u);

    for (int i = 0; i < 2000; ++i)
    {
        const uint32_t v = byul_roll(&rng, 8u);
        CHECK(v >= 1u);
        CHECK(v <= 8u);
    }
}

TEST_CASE("roll_pick: returns 0 for invalid args")
{
    byul_rng_t rng;
    byul_rng_init(&rng, 1ULL);

    const int32_t vals[] = { 10, 20, 30 };

    CHECK(byul_roll_pick(nullptr, 3, &rng) == 0);
    CHECK(byul_roll_pick(vals, 0, &rng) == 0);
    CHECK(byul_roll_pick(vals, 3, nullptr) == 0);
}

TEST_CASE("roll_pick: deterministic for same seed")
{
    byul_rng_t a;
    byul_rng_t b;

    byul_rng_init(&a, 777ULL);
    byul_rng_init(&b, 777ULL);

    const int32_t vals[] = { 1, 2, 3, 4, 5, 1, 3 };
    const size_t n = sizeof(vals) / sizeof(vals[0]);

    for (int i = 0; i < 256; ++i)
        CHECK(byul_roll_pick(vals, n, &a) == byul_roll_pick(vals, n, &b));
}

TEST_CASE("roll_pick_weighted: respects zero weights and total==0")
{
    byul_rng_t rng;
    byul_rng_init(&rng, 999ULL);

    const int32_t vals[] = { 10, 20, 30 };
    const uint32_t w0[]  = { 0, 0, 0 };
    const uint32_t w1[]  = { 0, 5, 0 };

    CHECK(byul_roll_pick_weighted(vals, w0, 3, &rng) == 0);

    /* Only '20' is selectable. */
    for (int i = 0; i < 256; ++i)
        CHECK(byul_roll_pick_weighted(vals, w1, 3, &rng) == 20);
}

TEST_CASE("roll_pick_weighted: deterministic for same seed")
{
    byul_rng_t a;
    byul_rng_t b;

    byul_rng_init(&a, 2024ULL);
    byul_rng_init(&b, 2024ULL);

    const int32_t vals[] = { 10, 20, 30, 40 };
    const uint32_t w[]   = { 1, 2, 3, 4 };

    for (int i = 0; i < 256; ++i)
        CHECK(byul_roll_pick_weighted(vals, w, 4, &a) ==
              byul_roll_pick_weighted(vals, w, 4, &b));
}

TEST_CASE("roll_check_percent: clamps 0 and 100 extremes")
{
    byul_rng_t rng;
    byul_rng_init(&rng, 42ULL);

    CHECK(byul_roll_check_percent(&rng, 0u) == false);
    CHECK(byul_roll_check_percent(&rng, 100u) == true);

    /* Smoke test for mid value */
    bool any_true = false;
    bool any_false = false;
    for (int i = 0; i < 512; ++i)
    {
        const bool r = byul_roll_check_percent(&rng, 50u);
        any_true |= r;
        any_false |= !r;
    }
    CHECK(any_true);
    CHECK(any_false);
}

TEST_CASE("roll_take: removes items without replacement (swap-remove)")
{
    byul_rng_t rng;
    byul_rng_init(&rng, 1234ULL);

    int32_t pool[] = { 1, 2, 3, 4, 5 };
    size_t count = 5;

    int32_t picked[5] = { 0,0,0,0,0 };

    for (size_t i = 0; i < 5; ++i)
    {
        picked[i] = byul_roll_take(pool, &count, &rng);
        CHECK(count == 4 - i);
    }

    /* Ensure all picked values are from original set and appear once. */
    auto seen = [](const int32_t* arr, size_t n, int32_t v) -> int {
        int c = 0;
        for (size_t i = 0; i < n; ++i) if (arr[i] == v) ++c;
        return c;
    };

    CHECK(seen(picked, 5, 1) == 1);
    CHECK(seen(picked, 5, 2) == 1);
    CHECK(seen(picked, 5, 3) == 1);
    CHECK(seen(picked, 5, 4) == 1);
    CHECK(seen(picked, 5, 5) == 1);

    CHECK(byul_roll_take(pool, &count, &rng) == 0);
    CHECK(count == 0);
}

TEST_CASE("roll_take_weighted: respects weights and removes chosen")
{
    byul_rng_t rng;
    byul_rng_init(&rng, 2025ULL);

    int32_t pool[] = { 10, 20, 30 };
    uint32_t w[]   = { 0, 5, 1 };
    size_t count = 3;

    /* First pick cannot be 10 because weight=0. */
    const int32_t first = byul_roll_take_weighted(pool, w, &count, &rng);
    CHECK(first != 10);
    CHECK(first != 0);
    CHECK(count == 2);

    /* Second pick should still be selectable (there is still positive weight). */
    const int32_t second = byul_roll_take_weighted(pool, w, &count, &rng);
    CHECK(second != 0);
    CHECK(count == 1);

    /* Now it is possible that the only remaining item has weight 0.
       In that case total weight == 0, so the function must return 0 and not modify count. */
    const int32_t third = byul_roll_take_weighted(pool, w, &count, &rng);
    CHECK(third == 0);
    CHECK(count == 1);

    /* If the caller wants to allow taking the remaining item, they must provide positive weight.
       (Demonstrate expected behavior after enabling it.) */
    w[0] = 1;
    const int32_t fourth = byul_roll_take_weighted(pool, w, &count, &rng);
    CHECK(fourth != 0);
    CHECK(count == 0);

    /* Now empty: returns 0 */
    CHECK(byul_roll_take_weighted(pool, w, &count, &rng) == 0);
}
