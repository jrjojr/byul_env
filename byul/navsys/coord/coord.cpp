#include "coord.h"
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <new>

namespace {
constexpr double kPi = 3.14159265358979323846;

bool coord_component_is_valid(std::int64_t value) noexcept {
    return value >= BYUL_COORD_COMPONENT_MIN
        && value <= BYUL_COORD_COMPONENT_MAX;
}

bool coord_components_are_valid(const coord_t& coord) noexcept {
    return coord_component_is_valid(coord.x)
        && coord_component_is_valid(coord.y);
}

navsys_status_t coord_commit_checked(
    coord_t* out, std::int64_t x, std::int64_t y) noexcept {
    if (!out || !coord_component_is_valid(x) || !coord_component_is_valid(y)) {
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    }
    out->x = static_cast<int>(x);
    out->y = static_cast<int>(y);
    return NAVSYS_STATUS_OK;
}
}

// ------------------------ Internal Utilities ------------------------

/**
 * @brief Wrap-around handling if value exceeds COORD_MIN ~ COORD_MAX range
 */
static inline int coord_wrap_value(int v) {
    const int RANGE = COORD_MAX - COORD_MIN + 1;
    int offset = v - COORD_MIN;
    offset = ((offset % RANGE) + RANGE) % RANGE; // Always in 0~RANGE-1
    return COORD_MIN + offset;
}

// ------------------------ ABI Layout Diagnostics ------------------------

size_t coord_sizeof(void) {
    return sizeof(coord_t);
}

size_t coord_alignof(void) {
    return alignof(coord_t);
}

size_t coord_offsetof_x(void) {
    return offsetof(coord_t, x);
}

size_t coord_offsetof_y(void) {
    return offsetof(coord_t, y);
}

// ------------------------ Checked Value API ------------------------

navsys_status_t coord_init_checked(
    coord_t* out_coord, int32_t x, int32_t y) {
    return coord_commit_checked(out_coord, x, y);
}

navsys_status_t coord_create_checked(
    int32_t x, int32_t y, coord_t** out_coord) {
    if (!out_coord
        || !coord_component_is_valid(x)
        || !coord_component_is_valid(y)) {
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    }

    try {
        coord_t* result = new coord_t{
            static_cast<int>(x),
            static_cast<int>(y)
        };
        *out_coord = result;
        return NAVSYS_STATUS_OK;
    } catch (const std::bad_alloc&) {
        return NAVSYS_STATUS_OUT_OF_MEMORY;
    } catch (...) {
        return NAVSYS_STATUS_OUT_OF_MEMORY;
    }
}

navsys_status_t coord_copy_checked(
    const coord_t* source, coord_t** out_coord) {
    if (!source || !out_coord) {
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    }

    const coord_t snapshot = *source;
    try {
        coord_t* result = new coord_t{snapshot.x, snapshot.y};
        *out_coord = result;
        return NAVSYS_STATUS_OK;
    } catch (const std::bad_alloc&) {
        return NAVSYS_STATUS_OUT_OF_MEMORY;
    } catch (...) {
        return NAVSYS_STATUS_OUT_OF_MEMORY;
    }
}

navsys_status_t coord_add_checked(
    coord_t* out, const coord_t* a, const coord_t* b) {
    if (!out || !a || !b) {
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    }
    const coord_t lhs = *a;
    const coord_t rhs = *b;
    if (!coord_components_are_valid(lhs)
        || !coord_components_are_valid(rhs)) {
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    }
    return coord_commit_checked(
        out,
        static_cast<std::int64_t>(lhs.x) + rhs.x,
        static_cast<std::int64_t>(lhs.y) + rhs.y);
}

navsys_status_t coord_sub_checked(
    coord_t* out, const coord_t* a, const coord_t* b) {
    if (!out || !a || !b) {
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    }
    const coord_t lhs = *a;
    const coord_t rhs = *b;
    if (!coord_components_are_valid(lhs)
        || !coord_components_are_valid(rhs)) {
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    }
    return coord_commit_checked(
        out,
        static_cast<std::int64_t>(lhs.x) - rhs.x,
        static_cast<std::int64_t>(lhs.y) - rhs.y);
}

navsys_status_t coord_mul_checked(
    coord_t* out, const coord_t* a, int32_t scalar) {
    if (!out || !a) {
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    }
    const coord_t input = *a;
    if (!coord_components_are_valid(input)) {
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    }
    return coord_commit_checked(
        out,
        static_cast<std::int64_t>(input.x) * scalar,
        static_cast<std::int64_t>(input.y) * scalar);
}

navsys_status_t coord_div_checked(
    coord_t* out, const coord_t* a, int32_t scalar) {
    if (!out || !a || scalar == 0) {
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    }
    const coord_t input = *a;
    if (!coord_components_are_valid(input)) {
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    }
    return coord_commit_checked(
        out,
        static_cast<std::int64_t>(input.x) / scalar,
        static_cast<std::int64_t>(input.y) / scalar);
}

navsys_status_t coord_compare_canonical(
    const coord_t* a, const coord_t* b, int* out_order) {
    if (!a || !b || !out_order) {
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    }

    int order = 0;
    if (a->x < b->x || (a->x == b->x && a->y < b->y)) {
        order = -1;
    } else if (a->x > b->x || (a->x == b->x && a->y > b->y)) {
        order = 1;
    }
    *out_order = order;
    return NAVSYS_STATUS_OK;
}

navsys_status_t coord_distance_f64(
    const coord_t* a, const coord_t* b, double* out_distance) {
    if (!a || !b || !out_distance) {
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    }
    const std::int64_t dx =
        static_cast<std::int64_t>(b->x) - a->x;
    const std::int64_t dy =
        static_cast<std::int64_t>(b->y) - a->y;
    const double result = std::hypot(
        static_cast<double>(dx),
        static_cast<double>(dy));
    *out_distance = result;
    return NAVSYS_STATUS_OK;
}

navsys_status_t coord_manhattan_distance_i64(
    const coord_t* a, const coord_t* b, int64_t* out_distance) {
    if (!a || !b || !out_distance) {
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    }
    const std::int64_t dx =
        static_cast<std::int64_t>(b->x) - a->x;
    const std::int64_t dy =
        static_cast<std::int64_t>(b->y) - a->y;
    *out_distance = (dx < 0 ? -dx : dx) + (dy < 0 ? -dy : dy);
    return NAVSYS_STATUS_OK;
}

navsys_status_t coord_angle_rad(
    const coord_t* from, const coord_t* to, double* out_angle) {
    if (!from || !to || !out_angle
        || (from->x == to->x && from->y == to->y)) {
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    }
    const std::int64_t dx =
        static_cast<std::int64_t>(to->x) - from->x;
    const std::int64_t dy =
        static_cast<std::int64_t>(to->y) - from->y;
    double result = std::atan2(
        static_cast<double>(dy),
        static_cast<double>(dx));
    if (result < 0.0) {
        result += 2.0 * kPi;
    }
    *out_angle = result;
    return NAVSYS_STATUS_OK;
}

navsys_status_t coord_angle_deg(
    const coord_t* from, const coord_t* to, double* out_angle) {
    if (!out_angle) {
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    }
    double radians = 0.0;
    const navsys_status_t status =
        coord_angle_rad(from, to, &radians);
    if (status != NAVSYS_STATUS_OK) {
        return status;
    }
    *out_angle = radians * (180.0 / kPi);
    return NAVSYS_STATUS_OK;
}

navsys_status_t coord_step_toward(
    const coord_t* start, const coord_t* goal, coord_t* out_next) {
    if (!start || !goal || !out_next) {
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    }
    const coord_t start_snapshot = *start;
    const coord_t goal_snapshot = *goal;
    coord_t result = start_snapshot;

    if (start_snapshot.x < goal_snapshot.x) {
        ++result.x;
    } else if (start_snapshot.x > goal_snapshot.x) {
        --result.x;
    }
    if (start_snapshot.y < goal_snapshot.y) {
        ++result.y;
    } else if (start_snapshot.y > goal_snapshot.y) {
        --result.y;
    }

    *out_next = result;
    return NAVSYS_STATUS_OK;
}

navsys_status_t coord_format(
    const coord_t* coord,
    char* out_buffer,
    size_t capacity,
    size_t* out_required) {
    if (!coord || !out_required
        || (!out_buffer && capacity != 0)
        || (out_buffer && capacity == 0)) {
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    }

    char formatted[64];
    const int written = std::snprintf(
        formatted,
        sizeof(formatted),
        "(%d, %d)",
        coord->x,
        coord->y);
    if (written < 0
        || static_cast<size_t>(written) >= sizeof(formatted)) {
        return NAVSYS_STATUS_CORRUPT_STATE;
    }

    const size_t required = static_cast<size_t>(written) + 1;
    if (!out_buffer) {
        *out_required = required;
        return NAVSYS_STATUS_OK;
    }
    if (capacity < required) {
        *out_required = required;
        return NAVSYS_STATUS_INCOMPLETE;
    }

    std::memcpy(out_buffer, formatted, required);
    *out_required = required;
    return NAVSYS_STATUS_OK;
}

// ------------------------ Creation/Destruction ------------------------

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

// ------------------------ Initialization and Copy ------------------------

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

// ------------------------ Arithmetic Operations ------------------------

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

// ------------------------ In-place Operations ------------------------

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

// ------------------------ Comparison/Hash ------------------------

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

// ------------------------ Distance Calculation ------------------------

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
    if (angle < 0) angle += 2.0 * kPi;
    return angle;
}

double coord_degree(const coord_t* a, const coord_t* b) {
    return coord_angle(a, b) * (180.0 / kPi);
}

// ------------------------ Next Step Towards Goal ------------------------

void coord_next_to_goal(coord_t* out, const coord_t* start, const coord_t* goal) {
    if (!out || !start || !goal) return;
    *out = *start;

    if (start->x < goal->x) out->x = coord_wrap_value(start->x + 1);
    else if (start->x > goal->x) out->x = coord_wrap_value(start->x - 1);

    if (start->y < goal->y) out->y = coord_wrap_value(start->y + 1);
    else if (start->y > goal->y) out->y = coord_wrap_value(start->y - 1);
}

// ------------------------ Accessors ------------------------

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

// ------------------------ Compatibility Functions ------------------------

coord_t make_tmp_coord(int x, int y) {
    coord_t c;
    coord_init_full(&c, x, y);
    return c;
}


coord_t* coord_clone_next_to_goal(
    const coord_t* start, const coord_t* goal) {
    coord_t next{};
    if (coord_step_toward(start, goal, &next) != NAVSYS_STATUS_OK) {
        return nullptr;
    }
    coord_t* out = nullptr;
    if (coord_create_checked(next.x, next.y, &out) != NAVSYS_STATUS_OK) {
        return nullptr;
    }
    return out;
}

char* coord_to_string(const coord_t* c, size_t buffer_size, char* buffer) {
    size_t required = 0;
    return coord_format(c, buffer, buffer_size, &required)
            == NAVSYS_STATUS_OK
        ? buffer
        : nullptr;
}

void coord_print(const coord_t* c) {
    if (!c) {
        printf("(null coord)\n");
        return;
    }
    char buffer[64];
    size_t required = 0;
    if (coord_format(c, buffer, sizeof(buffer), &required)
        == NAVSYS_STATUS_OK) {
        std::fputs(buffer, stdout);
        std::fputc('\n', stdout);
    }
}
