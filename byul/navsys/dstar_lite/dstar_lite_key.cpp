#include "dstar_lite_key.h"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <new>

namespace {

constexpr float kLegacyRelativeTolerance = 1e-5f;

bool legacy_component_equal(float lhs, float rhs) noexcept {
    if (lhs == rhs) return true;
    const float difference = std::fabs(lhs - rhs);
    const float largest = std::fmax(std::fabs(lhs), std::fabs(rhs));
    return difference <= kLegacyRelativeTolerance * largest;
}

bool component_is_valid(float value) noexcept {
    return !std::isnan(value)
        && !(std::isinf(value) && std::signbit(value));
}

float canonicalize_component(float value) noexcept {
    return value == 0.0f ? 0.0f : value;
}

bool key_is_valid(const dstar_lite_key_t& key) noexcept {
    return component_is_valid(key.k1) && component_is_valid(key.k2);
}

std::uint32_t component_bits(float value) noexcept {
    const float canonical = canonicalize_component(value);
    std::uint32_t bits = 0;
    static_assert(sizeof(bits) == sizeof(canonical));
    std::memcpy(&bits, &canonical, sizeof(bits));
    return bits;
}

} // namespace

size_t dstar_lite_key_sizeof(void) {
    return sizeof(dstar_lite_key_t);
}

size_t dstar_lite_key_alignof(void) {
    return alignof(dstar_lite_key_t);
}

size_t dstar_lite_key_offsetof_k1(void) {
    return offsetof(dstar_lite_key_t, k1);
}

size_t dstar_lite_key_offsetof_k2(void) {
    return offsetof(dstar_lite_key_t, k2);
}

navsys_status_t dstar_lite_key_init(
    dstar_lite_key_t* out_key, float k1, float k2) {
    if (!out_key || !component_is_valid(k1) || !component_is_valid(k2)) {
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    }
    const dstar_lite_key_t canonical = {
        canonicalize_component(k1),
        canonicalize_component(k2)
    };
    *out_key = canonical;
    return NAVSYS_STATUS_OK;
}

navsys_status_t dstar_lite_key_create_ex(
    float k1, float k2, dstar_lite_key_t** out_key) {
    if (!out_key || !component_is_valid(k1) || !component_is_valid(k2)) {
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    }
    try {
        dstar_lite_key_t* result = new dstar_lite_key_t{
            canonicalize_component(k1),
            canonicalize_component(k2)
        };
        *out_key = result;
        return NAVSYS_STATUS_OK;
    } catch (...) {
        return NAVSYS_STATUS_OUT_OF_MEMORY;
    }
}

navsys_status_t dstar_lite_key_copy_ex(
    const dstar_lite_key_t* source,
    dstar_lite_key_t** out_key) {
    if (!source || !out_key) return NAVSYS_STATUS_INVALID_ARGUMENT;
    const dstar_lite_key_t snapshot = *source;
    if (!key_is_valid(snapshot)) return NAVSYS_STATUS_INVALID_ARGUMENT;
    return dstar_lite_key_create_ex(snapshot.k1, snapshot.k2, out_key);
}

bool dstar_lite_key_equal_exact(
    const dstar_lite_key_t* lhs,
    const dstar_lite_key_t* rhs) {
    if (!lhs || !rhs || !key_is_valid(*lhs) || !key_is_valid(*rhs)) {
        return false;
    }
    return canonicalize_component(lhs->k1) == canonicalize_component(rhs->k1)
        && canonicalize_component(lhs->k2) == canonicalize_component(rhs->k2);
}

navsys_status_t dstar_lite_key_compare_exact(
    const dstar_lite_key_t* lhs,
    const dstar_lite_key_t* rhs,
    int* out_compare) {
    if (!lhs || !rhs || !out_compare
        || !key_is_valid(*lhs) || !key_is_valid(*rhs)) {
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    }
    const float lhs_k1 = canonicalize_component(lhs->k1);
    const float rhs_k1 = canonicalize_component(rhs->k1);
    const float lhs_k2 = canonicalize_component(lhs->k2);
    const float rhs_k2 = canonicalize_component(rhs->k2);
    int result = 0;
    if (lhs_k1 < rhs_k1) result = -1;
    else if (lhs_k1 > rhs_k1) result = 1;
    else if (lhs_k2 < rhs_k2) result = -1;
    else if (lhs_k2 > rhs_k2) result = 1;
    *out_compare = result;
    return NAVSYS_STATUS_OK;
}

std::uint32_t dstar_lite_key_hash_exact(const dstar_lite_key_t* key) {
    if (!key || !key_is_valid(*key)) return 0;
    return (component_bits(key->k1) * UINT32_C(31))
        ^ component_bits(key->k2);
}

bool dstar_lite_key_equal(
    const dstar_lite_key_t* dsk0, const dstar_lite_key_t* dsk1) {
    if (!dsk0 || !dsk1) return false;
    return legacy_component_equal(dsk0->k1, dsk1->k1)
        && legacy_component_equal(dsk0->k2, dsk1->k2);
}

int dstar_lite_key_compare(
    const dstar_lite_key_t* dsk0, const dstar_lite_key_t* dsk1) {
    if (dstar_lite_key_equal(dsk0, dsk1)) return 0;
    if (dsk0->k1 < dsk1->k1) return -1;
    if (dsk0->k1 > dsk1->k1) return 1;
    if (dsk0->k2 < dsk1->k2) return -1;
    if (dsk0->k2 > dsk1->k2) return 1;
    return 0;
}

unsigned int dstar_lite_key_hash(const dstar_lite_key_t* key) {
    if (!key) return 0;
    union { float f; std::uint32_t u; } u1, u2;
    u1.f = key->k1;
    u2.f = key->k2;
    return (u1.u * 31) ^ u2.u;
}

dstar_lite_key_t* dstar_lite_key_create() {
    try {
        return new dstar_lite_key_t{0.0f, 0.0f};
    } catch (...) {
        return nullptr;
    }
}

dstar_lite_key_t* dstar_lite_key_create_full(float k1, float k2) {
    try {
        return new dstar_lite_key_t{k1, k2};
    } catch (...) {
        return nullptr;
    }
}

dstar_lite_key_t* dstar_lite_key_copy(const dstar_lite_key_t* key) {
    if (!key) return nullptr;
    const dstar_lite_key_t snapshot = *key;
    try {
        return new dstar_lite_key_t{snapshot.k1, snapshot.k2};
    } catch (...) {
        return nullptr;
    }
}

void dstar_lite_key_destroy(dstar_lite_key_t* key) {
    delete key;
}
