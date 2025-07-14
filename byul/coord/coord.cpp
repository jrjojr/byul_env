#include "coord.hpp"
#include <cmath>
#include <new>
#include <cstdint>
#include <cstdlib>
#include <utility>

// C++11부터 제공되는 std 기능 사용

coord_t* coord_new_full(int x, int y) {
    // coord_t* c = new (std::nothrow) coord_t;
    coord_t* c = new coord_t();
    if (!c) return nullptr;
    c->x = x;
    c->y = y;
    return c;
}

coord_t* coord_new() {
    return coord_new_full(0, 0);
}

void coord_free(coord_t* c) {
    delete c;
}

// unsigned coord_hash(const coord_t* c) {
//     if (!c) return 0u;
//     return (static_cast<unsigned>(c->x) << 16) ^ static_cast<unsigned>(c->y);
// }

unsigned coord_hash(const coord_t* c) {
    if (!c) return 0u;
    int x = c->x;
    int y = c->y;
    return static_cast<unsigned>(x * 73856093 ^ y * 19349663);
}

bool coord_equal(const coord_t* c1, const coord_t* c2) {
    if (!c1 || !c2) return false;
    return c1->x == c2->x && c1->y == c2->y;
}

int coord_compare(const coord_t* c1, const coord_t* c2) {
    if (c1 == c2) return 0;
    if (!c1) return -1;
    if (!c2) return 1;
    int d1 = std::abs(c1->x) + std::abs(c1->y);
    int d2 = std::abs(c2->x) + std::abs(c2->y);
    return (d1 > d2) - (d1 < d2);
}

coord_t* coord_copy(const coord_t* c) {
    if (!c) return nullptr;
    return coord_new_full(c->x, c->y);
}

int coord_get_x(const coord_t* c) {
    return c ? c->x : 0;
}

void coord_set_x(coord_t* c, int x) {
    if (c) c->x = x;
}

int coord_get_y(const coord_t* c) {
    return c ? c->y : 0;
}

void coord_set_y(coord_t* c, int y) {
    if (c) c->y = y;
}

void coord_set(coord_t* c, int x, int y) {
    if (c) {
        c->x = x;
        c->y = y;
    }
}

void coord_fetch(const coord_t* c, int* out_x, int* out_y) {
    if (c) {
        if (out_x) *out_x = c->x;
        if (out_y) *out_y = c->y;
    } else {
        if (out_x) *out_x = 0;
        if (out_y) *out_y = 0;
    }
}

float coord_distance(const coord_t* a, const coord_t* b) {
    if (!a || !b) return 0.0f;
    int dx = a->x - b->x;
    int dy = a->y - b->y;
    return std::sqrt(dx * dx + dy * dy);
}

int coord_manhattan_distance(const coord_t* a, const coord_t* b) {
    if (!a || !b) return 0;
    return std::abs(a->x - b->x) + std::abs(a->y - b->y);
}

// double coord_degree(const coord_t* a, const coord_t* b) {
//     if (!a || !b) return 0.0;
//     double dx = static_cast<double>(b->x - a->x);
//     double dy = static_cast<double>(b->y - a->y);
//     double rad = std::atan2(dy, dx);
//     double deg = rad * (180.0 / M_PI);
//     if (deg < 0.0) deg += 360.0;
//     return deg;
// }

double coord_degree(const coord_t* a, const coord_t* b) {
    if (!a || !b) return 0.0;
    if (a->x == b->x && a->y == b->y)
        return std::numeric_limits<double>::quiet_NaN(); // 방향 없음

    // 게임 기준: +Y 아래 방향이면 아래 줄 활성화
    // double dy = static_cast<double>(a->y - b->y);  // y축 반전
    double dx = static_cast<double>(b->x - a->x);
    double dy = static_cast<double>(b->y - a->y);  // 기본 수학 좌표계

    double rad = std::atan2(dy, dx);
    double deg = rad * (180.0 / M_PI);
    if (deg < 0.0) deg += 360.0;

    return deg;
}

// const coord_t* make_tmp_coord(int x, int y) {
//     static thread_local coord_t temp;
//     temp.x = x;
//     temp.y = y;
//     return &temp;
// }

#include <vector>
#include <memory>

const coord_t* make_tmp_coord(int x, int y) {
    struct CoordPool {
        std::vector<coord_t*> pool;
        ~CoordPool() {
            for (auto c : pool) delete c;
        }
    };
    static thread_local CoordPool thread_coords;

    coord_t* c = new coord_t{x, y};
    thread_coords.pool.push_back(c);
    return c;
}


coord_t* coord_clone_next_to_goal(
    const coord_t* start, const coord_t* goal) 
{
    if (!start || !goal) return nullptr;

    int dx = (goal->x > start->x) - (goal->x < start->x); // -1, 0, 1
    int dy = (goal->y > start->y) - (goal->y < start->y); // -1, 0, 1

    return coord_new_full(start->x + dx, start->y + dy);
}