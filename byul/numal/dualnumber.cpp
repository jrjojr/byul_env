#include "internal/dualnumber.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <new>

void dualnumber_init(dualnumber_t* out) {
    if (!out) return;
    out->re = 0.0f;
    out->du = 0.0f;
}

void dualnumber_init_full(dualnumber_t* out, float re, float du) {
    if (!out) return;
    out->re = re;
    out->du = du;
}

void dualnumber_assign(dualnumber_t* out, const dualnumber_t* src) {
    if (!out || !src) return;
    out->re = src->re;
    out->du = src->du;
}

bool dualnumber_equal(const dualnumber_t* a, const dualnumber_t* b) {
    return a && b && a->re == b->re && a->du == b->du;
}

unsigned int dualnumber_hash(const dualnumber_t* a) {
    if (!a) return 0;
    unsigned int h = 0;
    h ^= *(const unsigned int*)&a->re;
    h ^= *(const unsigned int*)&a->du;
    return h;
}

void dualnumber_neg(dualnumber_t* out, const dualnumber_t* a) {
    if (!out || !a) return;
    out->re = -a->re;
    out->du = -a->du;
}

void dualnumber_add(dualnumber_t* out, 
    const dualnumber_t* a, const dualnumber_t* b) {

    if (!out || !a || !b) return;
    out->re = a->re + b->re;
    out->du = a->du + b->du;
}

void dualnumber_sub(dualnumber_t* out, 
    const dualnumber_t* a, const dualnumber_t* b) {

    if (!out || !a || !b) return;
    out->re = a->re - b->re;
    out->du = a->du - b->du;
}

void dualnumber_mul(dualnumber_t* out, 
    const dualnumber_t* a, const dualnumber_t* b) {

    if (!out || !a || !b) return;
    out->re = a->re * b->re;
    out->du = a->re * b->du + a->du * b->re;
}

void dualnumber_div(dualnumber_t* out, 
    const dualnumber_t* a, const dualnumber_t* b) {
        
    if (!out || !a || !b || b->re == 0.0f) return;
    float denom = b->re * b->re;
    out->re = a->re / b->re;
    out->du = (a->du * b->re - a->re * b->du) / denom;
}

void dualnumber_scale(dualnumber_t* out, const dualnumber_t* a, float s) {
    if (!out || !a) return;
    out->re = a->re * s;
    out->du = a->du * s;
}

void dualnumber_invscale(dualnumber_t* out, const dualnumber_t* a, float s) {
    if (!out || !a || s == 0.0f) return;
    out->re = a->re / s;
    out->du = a->du / s;
}

void dualnumber_powf(dualnumber_t* out, const dualnumber_t* a, float n) {
    if (!out || !a) return;
    float real_pow = powf(a->re, n);
    float dual_part = n * powf(a->re, n - 1.0f) * a->du;
    out->re = real_pow;
    out->du = dual_part;
}
