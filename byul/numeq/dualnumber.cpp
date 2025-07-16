#include "internal/dualnumber.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <new>

dualnumber_t* dualnumber_new() {
    return new (std::nothrow) dualnumber_t{0.0f, 0.0f};
}

dualnumber_t* dualnumber_new_full(float re, float du) {
    return new (std::nothrow) dualnumber_t{re, du};
}

void dualnumber_free(dualnumber_t* d) {
    delete d;
}

dualnumber_t* dualnumber_copy(const dualnumber_t* src) {
    if (!src) return nullptr;
    return dualnumber_new_full(src->re, src->du);
}

int dualnumber_equal(const dualnumber_t* a, const dualnumber_t* b) {
    return a && b && a->re == b->re && a->du == b->du;
}

unsigned int dualnumber_hash(const dualnumber_t* a) {
    if (!a) return 0;
    unsigned int h = 0;
    h ^= *(unsigned int*)&a->re;
    h ^= *(unsigned int*)&a->du;
    return h;
}

dualnumber_t* dualnumber_neg(const dualnumber_t* a) {
    return dualnumber_new_full(-a->re, -a->du);
}

dualnumber_t* dualnumber_add(const dualnumber_t* a, const dualnumber_t* b) {
    return dualnumber_new_full(a->re + b->re, a->du + b->du);
}

dualnumber_t* dualnumber_sub(const dualnumber_t* a, const dualnumber_t* b) {
    return dualnumber_new_full(a->re - b->re, a->du - b->du);
}

dualnumber_t* dualnumber_mul(const dualnumber_t* a, const dualnumber_t* b) {
    return dualnumber_new_full(a->re * b->re, a->re * b->du + a->du * b->re);
}

dualnumber_t* dualnumber_div(const dualnumber_t* a, const dualnumber_t* b) {
    float denom = b->re * b->re;
    return dualnumber_new_full(a->re / b->re, (a->du * b->re - a->re * b->du) / denom);
}

dualnumber_t* dualnumber_scale(const dualnumber_t* a, float s) {
    return dualnumber_new_full(a->re * s, a->du * s);
}

dualnumber_t* dualnumber_invscale(const dualnumber_t* a, float s) {
    return dualnumber_new_full(a->re / s, a->du / s);
}

dualnumber_t* dualnumber_powf(const dualnumber_t* a, float n) {
    float real_pow = powf(a->re, n);
    float dual_part = n * powf(a->re, n - 1.0f) * a->du;
    return dualnumber_new_full(real_pow, dual_part);
}
