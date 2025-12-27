#include "dstar_lite_key.h"
#include "scalar.h"
#include <cmath>
#include <cstdint>

bool dstar_lite_key_equal(
    const dstar_lite_key_t* dsk0, const dstar_lite_key_t* dsk1) {

    if (!dsk0 || !dsk1) return false;

    return (scalar_equal(dsk0->k1, dsk1->k1)) &&
           (scalar_equal(dsk0->k2, dsk1->k2));
}

int dstar_lite_key_compare(
    const dstar_lite_key_t* dsk0, const dstar_lite_key_t* dsk1) {

    if (dstar_lite_key_equal(dsk0, dsk1)) return 0;

    if (dsk0->k1 < dsk1->k1)
        return -1;
    else if (dsk0->k1 > dsk1->k1)
        return 1;
    else {
        if (dsk0->k2 < dsk1->k2)
            return -1;
        else if (dsk0->k2 > dsk1->k2)
            return 1;
        else
            return 0;
    }
}

unsigned int dstar_lite_key_hash(const dstar_lite_key_t* key) {
    if (!key) return 0;

    union { float f; std::uint32_t u; } u1, u2;
    u1.f = key->k1;
    u2.f = key->k2;

    return (u1.u * 31) ^ u2.u;
}

dstar_lite_key_t* dstar_lite_key_create() {
    return new dstar_lite_key_t{0.0f, 0.0f};
}

dstar_lite_key_t* dstar_lite_key_create_full(float k1, float k2) {
    return new dstar_lite_key_t{k1, k2};
}

dstar_lite_key_t* dstar_lite_key_copy(const dstar_lite_key_t* key) {
    if (!key) return nullptr;
    return new dstar_lite_key_t{key->k1, key->k2};
}

void dstar_lite_key_destroy(dstar_lite_key_t* key) {
    delete key;
}
