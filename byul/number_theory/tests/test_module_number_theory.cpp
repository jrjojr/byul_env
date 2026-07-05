#include "doctest.h"

#include <stdint.h>
#include <vector>

extern "C" {
    #include "byul_prime32.h"
    #include "byul_prime_segmented_u32.h"
}

struct FactorRecord {
    uint32_t prime;
    uint32_t count;
};

static bool collect_factor(uint32_t prime, uint32_t count, void* user) {
    auto* factors = static_cast<std::vector<FactorRecord>*>(user);
    factors->push_back({prime, count});
    return true;
}

static bool collect_prime(uint32_t prime, void* user) {
    auto* primes = static_cast<std::vector<uint32_t>*>(user);
    primes->push_back(prime);
    return true;
}

TEST_CASE("prime32: primality checks") {
    CHECK_FALSE(byul_is_prime_u32(0));
    CHECK_FALSE(byul_is_prime_u32(1));
    CHECK(byul_is_prime_u32(2));
    CHECK(byul_is_prime_u32(3));
    CHECK_FALSE(byul_is_prime_u32(4));
    CHECK(byul_is_prime_u32(65521));
    CHECK_FALSE(byul_is_prime_u32(65535));
}

TEST_CASE("prime32: next prime") {
    CHECK(byul_next_prime_u32(0) == 2);
    CHECK(byul_next_prime_u32(2) == 2);
    CHECK(byul_next_prime_u32(14) == 17);
    CHECK(byul_next_prime_u32(100) == 101);
}

TEST_CASE("prime32: factorization callback") {
    std::vector<FactorRecord> factors;

    REQUIRE(byul_factor_u32_emit(360, collect_factor, &factors));
    REQUIRE(factors.size() == 3);

    CHECK(factors[0].prime == 2);
    CHECK(factors[0].count == 3);
    CHECK(factors[1].prime == 3);
    CHECK(factors[1].count == 2);
    CHECK(factors[2].prime == 5);
    CHECK(factors[2].count == 1);
}

TEST_CASE("prime segmented u32: base prime generation") {
    uint16_t base_primes[16] = {};
    uint8_t sieve_bits[8] = {};

    size_t count = byul_make_base_primes_u32(100, base_primes, 16, sieve_bits, sizeof(sieve_bits));

    REQUIRE(count == 4);
    CHECK(base_primes[0] == 2);
    CHECK(base_primes[1] == 3);
    CHECK(base_primes[2] == 5);
    CHECK(base_primes[3] == 7);
}

TEST_CASE("prime segmented u32: emits primes in range") {
    uint16_t base_primes[16] = {};
    uint8_t sieve_bits[8] = {};
    uint8_t window_bits[8] = {};
    std::vector<uint32_t> primes;

    size_t base_count = byul_make_base_primes_u32(30, base_primes, 16, sieve_bits, sizeof(sieve_bits));
    REQUIRE(base_count > 0);

    byul_segmented_primes_u32_emit(10, 30, base_primes, base_count, window_bits, sizeof(window_bits), collect_prime, &primes);

    const std::vector<uint32_t> expected = {11, 13, 17, 19, 23, 29};
    CHECK(primes == expected);
}
