#include "coord.h"
#include <cmath>
#include <cstdlib>
#include <stdio.h>
#include "float_common.h"

// ------------------------ 내부 유틸리티 ------------------------

/**
 * @brief COORD_MIN ~ COORD_MAX 범위를 넘어가면 wrap-around 처리
 */
static inline int coord_wrap_value(int v) {
    const int RANGE = COORD_MAX - COORD_MIN + 1;
    int offset = v - COORD_MIN;
    offset = ((offset % RANGE) + RANGE) % RANGE; // 항상 0~RANGE-1
    return COORD_MIN + offset;
}

// ------------------------ 생성/해제 ------------------------

coord_t* coord_create_full(int x, int y) {
    coord_t* c = new coord_t;
    c->x = coord_wrap_value(x);
    c->y = coord_wrap_value(y);
    return c;
}

coord_t* coord_create(void) {
    coord_t* c = new coord_t;
    c->x = 0;
    c->y = 0;
    return c;
}

void coord_destroy(coord_t* c) {
    delete c;
}

coord_t* coord_copy(const coord_t* c) {
    if (!c) return nullptr;
    coord_t* new_c = new coord_t;
    new_c->x = c->x;
    new_c->y = c->y;
    return new_c;
}

// ------------------------ 초기화 및 복사 ------------------------

void coord_init(coord_t* c) {
    if (!c) return;
    c->x = 0;
    c->y = 0;
}

void coord_init_full(coord_t* c, int x, int y) {
    if (!c) return;
    c->x = coord_wrap_value(x);
    c->y = coord_wrap_value(y);
}

void coord_assign(coord_t* dst, const coord_t* src) {
    if (!dst || !src) return;
    dst->x = src->x;
    dst->y = src->y;
}

// ------------------------ 가감승제 ------------------------

void coord_add(coord_t* dst, const coord_t* a, const coord_t* b) {
    if (!dst || !a || !b) return;
    dst->x = coord_wrap_value(a->x + b->x);
    dst->y = coord_wrap_value(a->y + b->y);
}

void coord_sub(coord_t* dst, const coord_t* a, const coord_t* b) {
    if (!dst || !a || !b) return;
    dst->x = coord_wrap_value(a->x - b->x);
    dst->y = coord_wrap_value(a->y - b->y);
}

void coord_mul(coord_t* dst, const coord_t* a, int scalar) {
    if (!dst || !a) return;
    dst->x = coord_wrap_value(a->x * scalar);
    dst->y = coord_wrap_value(a->y * scalar);
}

void coord_div(coord_t* dst, const coord_t* a, int scalar) {
    if (!dst || !a || scalar == 0) return;
    dst->x = coord_wrap_value(a->x / scalar);
    dst->y = coord_wrap_value(a->y / scalar);
}

// ------------------------ 자기 연산 ------------------------

void coord_iadd(coord_t* c, const coord_t* other) {
    if (!c || !other) return;
    c->x = coord_wrap_value(c->x + other->x);
    c->y = coord_wrap_value(c->y + other->y);
}

void coord_isub(coord_t* c, const coord_t* other) {
    if (!c || !other) return;
    c->x = coord_wrap_value(c->x - other->x);
    c->y = coord_wrap_value(c->y - other->y);
}

void coord_imul(coord_t* c, int scalar) {
    if (!c) return;
    c->x = coord_wrap_value(c->x * scalar);
    c->y = coord_wrap_value(c->y * scalar);
}

void coord_idiv(coord_t* c, int scalar) {
    if (!c || scalar == 0) return;
    c->x = coord_wrap_value(c->x / scalar);
    c->y = coord_wrap_value(c->y / scalar);
}

// ------------------------ 비교/해시 ------------------------

unsigned coord_hash(const coord_t* c) {
    if (!c) return 0;
    return (static_cast<unsigned>(c->x) * 73856093u) ^
           (static_cast<unsigned>(c->y) * 19349663u);
}

bool coord_equal(const coord_t* c1, const coord_t* c2) {
    return c1 && c2 && (c1->x == c2->x) && (c1->y == c2->y);
}

int coord_compare(const coord_t* c1, const coord_t* c2) {
    if (!c1 || !c2) return 0;
    if (c1->x == c2->x) return c1->y - c2->y;
    return c1->x - c2->x;
}

// ------------------------ 거리 계산 ------------------------

float coord_distance(const coord_t* a, const coord_t* b) {
    if (!a || !b) return 0.0f;
    float dx = static_cast<float>(b->x - a->x);
    float dy = static_cast<float>(b->y - a->y);
    return std::sqrt(dx * dx + dy * dy);
}

int coord_manhattan_distance(const coord_t* a, const coord_t* b) {
    if (!a || !b) return 0;
    return std::abs(b->x - a->x) + std::abs(b->y - a->y);
}

double coord_angle(const coord_t* a, const coord_t* b) {
    if (!a || !b) return 0.0;
    double dx = static_cast<double>(b->x - a->x);
    double dy = static_cast<double>(b->y - a->y);
    double angle = atan2(dy, dx);
    if (angle < 0) angle += 2.0 * M_PI;
    return angle;
}

double coord_degree(const coord_t* a, const coord_t* b) {
    return coord_angle(a, b) * (180.0 / M_PI);
}

// ------------------------ 목표 방향 ------------------------

void coord_next_to_goal(coord_t* out, const coord_t* start, const coord_t* goal) {
    if (!out || !start || !goal) return;
    *out = *start;

    if (start->x < goal->x) out->x = coord_wrap_value(start->x + 1);
    else if (start->x > goal->x) out->x = coord_wrap_value(start->x - 1);

    if (start->y < goal->y) out->y = coord_wrap_value(start->y + 1);
    else if (start->y > goal->y) out->y = coord_wrap_value(start->y - 1);
}

// ------------------------ 좌표 접근자 ------------------------

int coord_get_x(const coord_t* c) {
    return c ? c->x : 0;
}

void coord_set_x(coord_t* c, int x) {
    if (!c) return;
    c->x = coord_wrap_value(x);
}

int coord_get_y(const coord_t* c) {
    return c ? c->y : 0;
}

void coord_set_y(coord_t* c, int y) {
    if (!c) return;
    c->y = coord_wrap_value(y);
}

void coord_set(coord_t* c, int x, int y) {
    if (!c) return;
    c->x = coord_wrap_value(x);
    c->y = coord_wrap_value(y);
}

void coord_fetch(const coord_t* c, int* out_x, int* out_y) {
    if (!c) return;
    if (out_x) *out_x = c->x;
    if (out_y) *out_y = c->y;
}

// ------------------------ 호환성 함수 ------------------------

const coord_t* make_tmp_coord(int x, int y) {
    static coord_t tmp;
    tmp.x = coord_wrap_value(x);
    tmp.y = coord_wrap_value(y);
    return &tmp;
}

coord_t* coord_clone_next_to_goal(
    const coord_t* start, const coord_t* goal) {
    coord_t* out = new coord_t;
    coord_next_to_goal(out, start, goal);
    return out;
}

char* coord_to_string(const coord_t* c, char* buffer, size_t buffer_size) {
    if (!c || !buffer || buffer_size < 16) return NULL;
    snprintf(buffer, buffer_size, "(%d, %d)", c->x, c->y);
    return buffer;
}

void coord_print(const coord_t* c) {
    if (!c) {
        printf("(null coord)\n");
        return;
    }
    printf("(%d, %d)\n", c->x, c->y);
}
