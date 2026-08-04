#include "obstacle.h"
#include "internal/obstacle_private.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <new>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <vector>

namespace {

struct generate_controls_t {
    obstacle_raster_rule_t raster_rule = OBSTACLE_RASTER_CELL_CENTER;
    uint64_t seed = 0;
    uint64_t max_cells = 0;
    obstacle_generate_cancel_func cancel_func = nullptr;
    void* cancel_userdata = nullptr;
};

navsys_status_t read_generate_controls(
    const obstacle_generate_options_t* options,
    generate_controls_t& out) {
    if (!options) return NAVSYS_STATUS_OK;
    if (options->struct_size < sizeof(obstacle_generate_options_t)
        || options->abi_version != OBSTACLE_GENERATE_OPTIONS_ABI_VERSION
        || options->reserved0 != 0
        || (options->raster_rule != OBSTACLE_RASTER_CELL_CENTER
            && options->raster_rule != OBSTACLE_RASTER_ALL_TOUCHED)) {
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    }
    out.raster_rule = options->raster_rule;
    out.seed = options->seed;
    out.max_cells = options->max_cells;
    out.cancel_func = options->cancel_func;
    out.cancel_userdata = options->cancel_userdata;
    return NAVSYS_STATUS_OK;
}

navsys_status_t poll_generate_cancel(const generate_controls_t& controls) {
    if (!controls.cancel_func) return NAVSYS_STATUS_OK;
    try {
        return controls.cancel_func(controls.cancel_userdata)
            ? NAVSYS_STATUS_CANCELLED : NAVSYS_STATUS_OK;
    } catch (...) {
        return NAVSYS_STATUS_CALLBACK_FAILED;
    }
}

bool coord_less(const coord_t& a, const coord_t& b) {
    return a.x < b.x || (a.x == b.x && a.y < b.y);
}

bool coord_same(const coord_t& a, const coord_t& b) {
    return a.x == b.x && a.y == b.y;
}

navsys_status_t validate_positive_extent(
    int32_t x0, int32_t y0, int32_t width, int32_t height) {
    if (width < 0 || height < 0) return NAVSYS_STATUS_INVALID_ARGUMENT;
    if (width == 0 || height == 0) return NAVSYS_STATUS_OK;
    const int64_t x1 = static_cast<int64_t>(x0) + width - 1;
    const int64_t y1 = static_cast<int64_t>(y0) + height - 1;
    if (x1 > INT32_MAX || y1 > INT32_MAX)
        return NAVSYS_STATUS_LIMIT_REACHED;
    return NAVSYS_STATUS_OK;
}

navsys_status_t materialize_sorted_generated(
    std::vector<coord_t>& coords,
    int32_t x0,
    int32_t y0,
    int32_t width,
    int32_t height,
    const generate_controls_t& controls,
    obstacle_t** out_obstacle) {
    if (controls.max_cells != 0 && coords.size() > controls.max_cells)
        return NAVSYS_STATUS_LIMIT_REACHED;
    obstacle_t* generated = nullptr;
    navsys_status_t status = obstacle_create_checked(
        x0, y0, width, height, &generated);
    if (status != NAVSYS_STATUS_OK) return status;
    for (size_t index = 0; index < coords.size(); ++index) {
        if ((index & 1023u) == 0u) {
            status = poll_generate_cancel(controls);
            if (status != NAVSYS_STATUS_OK) {
                obstacle_destroy(generated);
                return status;
            }
        }
        bool changed = false;
        status = obstacle_set_blocked(
            generated, coords[index].x, coords[index].y, true, &changed);
        if (status != NAVSYS_STATUS_OK || !changed) {
            obstacle_destroy(generated);
            return status == NAVSYS_STATUS_OK
                ? NAVSYS_STATUS_CORRUPT_STATE : status;
        }
    }
    *out_obstacle = generated;
    return NAVSYS_STATUS_OK;
}

navsys_status_t materialize_generated(
    std::vector<coord_t>& coords,
    int32_t empty_x0,
    int32_t empty_y0,
    const generate_controls_t& controls,
    obstacle_t** out_obstacle) {
    navsys_status_t status = poll_generate_cancel(controls);
    if (status != NAVSYS_STATUS_OK) return status;
    std::sort(coords.begin(), coords.end(), coord_less);
    coords.erase(std::unique(coords.begin(), coords.end(), coord_same), coords.end());

    int32_t x0 = empty_x0;
    int32_t y0 = empty_y0;
    int32_t width = 0;
    int32_t height = 0;
    if (!coords.empty()) {
        const auto x_bounds = std::minmax_element(
            coords.begin(), coords.end(),
            [](const coord_t& a, const coord_t& b) { return a.x < b.x; });
        const auto y_bounds = std::minmax_element(
            coords.begin(), coords.end(),
            [](const coord_t& a, const coord_t& b) { return a.y < b.y; });
        const int64_t width64 = static_cast<int64_t>(x_bounds.second->x)
            - x_bounds.first->x + 1;
        const int64_t height64 = static_cast<int64_t>(y_bounds.second->y)
            - y_bounds.first->y + 1;
        if (width64 > INT32_MAX || height64 > INT32_MAX)
            return NAVSYS_STATUS_LIMIT_REACHED;
        x0 = x_bounds.first->x;
        y0 = y_bounds.first->y;
        width = static_cast<int32_t>(width64);
        height = static_cast<int32_t>(height64);
    }
    return materialize_sorted_generated(
        coords, x0, y0, width, height, controls, out_obstacle);
}

navsys_status_t materialize_generated_in_extent(
    std::vector<coord_t>& coords,
    int32_t x0,
    int32_t y0,
    int32_t width,
    int32_t height,
    const generate_controls_t& controls,
    obstacle_t** out_obstacle) {
    navsys_status_t status = poll_generate_cancel(controls);
    if (status != NAVSYS_STATUS_OK) return status;
    std::sort(coords.begin(), coords.end(), coord_less);
    coords.erase(std::unique(coords.begin(), coords.end(), coord_same), coords.end());
    const int64_t x1 = static_cast<int64_t>(x0) + width;
    const int64_t y1 = static_cast<int64_t>(y0) + height;
    for (const coord_t& coord : coords) {
        if (coord.x < x0 || coord.x >= x1 || coord.y < y0 || coord.y >= y1)
            return NAVSYS_STATUS_CORRUPT_STATE;
    }
    return materialize_sorted_generated(
        coords, x0, y0, width, height, controls, out_obstacle);
}

uint64_t splitmix64_v1(uint64_t& state) {
    state += UINT64_C(0x9E3779B97F4A7C15);
    uint64_t value = state;
    value = (value ^ (value >> 30)) * UINT64_C(0xBF58476D1CE4E5B9);
    value = (value ^ (value >> 27)) * UINT64_C(0x94D049BB133111EB);
    return value ^ (value >> 31);
}

bool probability_selects(uint64_t draw, double probability) {
    if (probability <= 0.0) return false;
    if (probability >= 1.0) return true;
    int exponent = 0;
    const double fraction = std::frexp(probability, &exponent);
    const uint64_t significand = static_cast<uint64_t>(
        std::ldexp(fraction, 53));
    const int shift = exponent + 11;
    if (shift >= 0) {
        const uint64_t threshold = significand << shift;
        return draw < threshold;
    }
    const unsigned right_shift = static_cast<unsigned>(-shift);
    uint64_t threshold = 1;
    if (right_shift < 64) {
        const uint64_t divisor = UINT64_C(1) << right_shift;
        threshold = significand / divisor
            + (significand % divisor != 0 ? 1u : 0u);
    }
    return draw < threshold;
}

struct signed_product_t {
    bool negative;
    uint64_t magnitude;
};

signed_product_t signed_product(int64_t a, int64_t b) {
    const bool negative = (a < 0) != (b < 0);
    const uint64_t ua = a < 0
        ? static_cast<uint64_t>(-(a + 1)) + 1u
        : static_cast<uint64_t>(a);
    const uint64_t ub = b < 0
        ? static_cast<uint64_t>(-(b + 1)) + 1u
        : static_cast<uint64_t>(b);
    return {negative && ua != 0 && ub != 0, ua * ub};
}

int orientation_sign(const coord_t& a, const coord_t& b, const coord_t& p) {
    const int64_t abx = static_cast<int64_t>(b.x) - a.x;
    const int64_t aby = static_cast<int64_t>(b.y) - a.y;
    const int64_t apx = static_cast<int64_t>(p.x) - a.x;
    const int64_t apy = static_cast<int64_t>(p.y) - a.y;
    const signed_product_t left = signed_product(abx, apy);
    const signed_product_t right = signed_product(aby, apx);
    if (left.negative != right.negative)
        return left.negative ? -1 : 1;
    if (left.magnitude == right.magnitude) return 0;
    const bool left_greater = left.magnitude > right.magnitude;
    if (!left.negative) return left_greater ? 1 : -1;
    return left_greater ? -1 : 1;
}

struct wide_magnitude_t {
    uint64_t high;
    uint64_t low;
};

struct wide_signed_product_t {
    bool negative;
    wide_magnitude_t magnitude;
};

struct scaled_point_t {
    int64_t x;
    int64_t y;
};

wide_magnitude_t multiply_wide(uint64_t a, uint64_t b) {
    const uint64_t a_low = static_cast<uint32_t>(a);
    const uint64_t a_high = a >> 32;
    const uint64_t b_low = static_cast<uint32_t>(b);
    const uint64_t b_high = b >> 32;
    const uint64_t low_product = a_low * b_low;
    const uint64_t middle1 = a_high * b_low;
    const uint64_t middle2 = a_low * b_high;
    const uint64_t high_product = a_high * b_high;
    const uint64_t carry = (low_product >> 32)
        + static_cast<uint32_t>(middle1)
        + static_cast<uint32_t>(middle2);
    return {
        high_product + (middle1 >> 32) + (middle2 >> 32) + (carry >> 32),
        (carry << 32) | static_cast<uint32_t>(low_product)};
}

wide_signed_product_t signed_product_wide(int64_t a, int64_t b) {
    const bool negative = (a < 0) != (b < 0);
    const uint64_t ua = a < 0
        ? static_cast<uint64_t>(-(a + 1)) + 1u
        : static_cast<uint64_t>(a);
    const uint64_t ub = b < 0
        ? static_cast<uint64_t>(-(b + 1)) + 1u
        : static_cast<uint64_t>(b);
    return {
        negative && ua != 0 && ub != 0,
        multiply_wide(ua, ub)};
}

int compare_wide(const wide_magnitude_t& a, const wide_magnitude_t& b) {
    if (a.high != b.high) return a.high < b.high ? -1 : 1;
    if (a.low != b.low) return a.low < b.low ? -1 : 1;
    return 0;
}

int orientation_sign_scaled(
    const scaled_point_t& a,
    const scaled_point_t& b,
    const scaled_point_t& p) {
    const wide_signed_product_t left = signed_product_wide(
        b.x - a.x, p.y - a.y);
    const wide_signed_product_t right = signed_product_wide(
        b.y - a.y, p.x - a.x);
    if (left.negative != right.negative)
        return left.negative ? -1 : 1;
    const int magnitude_order = compare_wide(left.magnitude, right.magnitude);
    return left.negative ? -magnitude_order : magnitude_order;
}

bool point_on_segment_scaled(
    const scaled_point_t& point,
    const scaled_point_t& a,
    const scaled_point_t& b) {
    return orientation_sign_scaled(a, b, point) == 0
        && std::min(a.x, b.x) <= point.x
        && point.x <= std::max(a.x, b.x)
        && std::min(a.y, b.y) <= point.y
        && point.y <= std::max(a.y, b.y);
}

bool segments_touch_scaled(
    const scaled_point_t& a,
    const scaled_point_t& b,
    const scaled_point_t& c,
    const scaled_point_t& d) {
    const int abc = orientation_sign_scaled(a, b, c);
    const int abd = orientation_sign_scaled(a, b, d);
    const int cda = orientation_sign_scaled(c, d, a);
    const int cdb = orientation_sign_scaled(c, d, b);
    if (abc == 0 && point_on_segment_scaled(c, a, b)) return true;
    if (abd == 0 && point_on_segment_scaled(d, a, b)) return true;
    if (cda == 0 && point_on_segment_scaled(a, c, d)) return true;
    if (cdb == 0 && point_on_segment_scaled(b, c, d)) return true;
    return (abc < 0) != (abd < 0) && (cda < 0) != (cdb < 0);
}

scaled_point_t scale_coord(const coord_t& point) {
    return {
        static_cast<int64_t>(point.x) * 2,
        static_cast<int64_t>(point.y) * 2};
}

void make_scaled_cell_square(
    const coord_t& cell, scaled_point_t (&square)[4]) {
    const int64_t x = static_cast<int64_t>(cell.x) * 2;
    const int64_t y = static_cast<int64_t>(cell.y) * 2;
    square[0] = {x - 1, y - 1};
    square[1] = {x + 1, y - 1};
    square[2] = {x + 1, y + 1};
    square[3] = {x - 1, y + 1};
}

bool point_covered_by_scaled_ring(
    const scaled_point_t& point,
    const coord_t* vertices,
    size_t count,
    obstacle_polygon_fill_rule_t fill_rule) {
    bool parity = false;
    int64_t winding = 0;
    for (size_t index = 0; index < count; ++index) {
        const scaled_point_t a = scale_coord(vertices[index]);
        const scaled_point_t b = scale_coord(vertices[(index + 1) % count]);
        const int sign = orientation_sign_scaled(a, b, point);
        if (sign == 0 && point_on_segment_scaled(point, a, b)) return true;
        if ((a.y > point.y) != (b.y > point.y)
            && ((sign > 0) == (b.y > a.y))) {
            parity = !parity;
        }
        if (a.y <= point.y && point.y < b.y && sign > 0) ++winding;
        else if (b.y <= point.y && point.y < a.y && sign < 0) --winding;
    }
    return fill_rule == OBSTACLE_POLYGON_EVEN_ODD
        ? parity : winding != 0;
}

bool cell_touches_ring(
    const coord_t& cell,
    const coord_t* vertices,
    size_t count,
    obstacle_polygon_fill_rule_t fill_rule) {
    scaled_point_t square[4];
    make_scaled_cell_square(cell, square);
    for (const scaled_point_t& corner : square) {
        if (point_covered_by_scaled_ring(
                corner, vertices, count, fill_rule)) {
            return true;
        }
    }
    for (size_t index = 0; index < count; ++index) {
        const scaled_point_t point = scale_coord(vertices[index]);
        if (square[0].x <= point.x && point.x <= square[2].x
            && square[0].y <= point.y && point.y <= square[2].y) {
            return true;
        }
        const scaled_point_t next = scale_coord(vertices[(index + 1) % count]);
        for (size_t side = 0; side < 4; ++side) {
            if (segments_touch_scaled(
                    point, next, square[side], square[(side + 1) % 4])) {
                return true;
            }
        }
    }
    return false;
}

bool point_on_segment(
    const coord_t& point, const coord_t& a, const coord_t& b) {
    return orientation_sign(a, b, point) == 0
        && std::min(a.x, b.x) <= point.x
        && point.x <= std::max(a.x, b.x)
        && std::min(a.y, b.y) <= point.y
        && point.y <= std::max(a.y, b.y);
}

navsys_status_t validate_ring(const coord_t* vertices, size_t count) {
    if (!vertices || count < 3) return NAVSYS_STATUS_INVALID_ARGUMENT;
    for (size_t index = 0; index < count; ++index) {
        if (coord_same(vertices[index], vertices[(index + 1) % count]))
            return NAVSYS_STATUS_INVALID_ARGUMENT;
    }
    bool non_collinear = false;
    for (size_t index = 2; index < count; ++index) {
        if (orientation_sign(vertices[0], vertices[1], vertices[index]) != 0) {
            non_collinear = true;
            break;
        }
    }
    return non_collinear ? NAVSYS_STATUS_OK : NAVSYS_STATUS_INVALID_ARGUMENT;
}

bool point_covered_by_ring(
    const coord_t& point,
    const coord_t* vertices,
    size_t count,
    obstacle_polygon_fill_rule_t fill_rule) {
    bool parity = false;
    int64_t winding = 0;
    for (size_t index = 0; index < count; ++index) {
        const coord_t& a = vertices[index];
        const coord_t& b = vertices[(index + 1) % count];
        const int sign = orientation_sign(a, b, point);
        if (sign == 0 && point_on_segment(point, a, b)) return true;
        if ((a.y > point.y) != (b.y > point.y)
            && ((sign > 0) == (b.y > a.y))) {
            parity = !parity;
        }
        if (a.y <= point.y && point.y < b.y && sign > 0) ++winding;
        else if (b.y <= point.y && point.y < a.y && sign < 0) --winding;
    }
    return fill_rule == OBSTACLE_POLYGON_EVEN_ODD
        ? parity : winding != 0;
}

void append_nearest_offsets(
    std::vector<int64_t>& offsets,
    const signed_product_t& numerator,
    uint64_t denominator) {
    const uint64_t quotient = numerator.magnitude / denominator;
    const uint64_t remainder = numerator.magnitude % denominator;
    const uint64_t rounded = quotient
        + (2 * remainder > denominator ? 1u : 0u);
    offsets.push_back(numerator.negative
        ? -static_cast<int64_t>(rounded)
        : static_cast<int64_t>(rounded));
    if (2 * remainder == denominator) {
        const uint64_t other = quotient + 1u;
        offsets.push_back(numerator.negative
            ? -static_cast<int64_t>(other)
            : static_cast<int64_t>(other));
    }
}

navsys_status_t append_line_cells(
    const coord_t& start,
    const coord_t& end,
    uint32_t radius,
    const generate_controls_t& controls,
    std::vector<coord_t>& out) {
    const int64_t min_x = static_cast<int64_t>(std::min(start.x, end.x)) - radius;
    const int64_t max_x = static_cast<int64_t>(std::max(start.x, end.x)) + radius;
    const int64_t min_y = static_cast<int64_t>(std::min(start.y, end.y)) - radius;
    const int64_t max_y = static_cast<int64_t>(std::max(start.y, end.y)) + radius;
    if (min_x < INT32_MIN || max_x > INT32_MAX
        || min_y < INT32_MIN || max_y > INT32_MAX) {
        return NAVSYS_STATUS_LIMIT_REACHED;
    }
    const int64_t dx = static_cast<int64_t>(end.x) - start.x;
    const int64_t dy = static_cast<int64_t>(end.y) - start.y;
    const bool x_major = std::llabs(dx) >= std::llabs(dy);
    const uint64_t steps = static_cast<uint64_t>(
        x_major ? std::llabs(dx) : std::llabs(dy));
    const int64_t major_sign = x_major ? (dx < 0 ? -1 : 1) : (dy < 0 ? -1 : 1);
    std::vector<int64_t> offsets;
    offsets.reserve(2);
    for (uint64_t step = 0; step <= steps; ++step) {
        if ((step & 1023u) == 0u) {
            const navsys_status_t status = poll_generate_cancel(controls);
            if (status != NAVSYS_STATUS_OK) return status;
        }
        offsets.clear();
        if (steps == 0) offsets.push_back(0);
        else append_nearest_offsets(
            offsets,
            signed_product(x_major ? dy : dx, static_cast<int64_t>(step)),
            steps);
        for (const int64_t offset : offsets) {
            const int64_t center_x = x_major
                ? static_cast<int64_t>(start.x) + major_sign * static_cast<int64_t>(step)
                : static_cast<int64_t>(start.x) + offset;
            const int64_t center_y = x_major
                ? static_cast<int64_t>(start.y) + offset
                : static_cast<int64_t>(start.y) + major_sign * static_cast<int64_t>(step);
            for (int64_t oy = -static_cast<int64_t>(radius);
                 oy <= static_cast<int64_t>(radius); ++oy) {
                for (int64_t ox = -static_cast<int64_t>(radius);
                     ox <= static_cast<int64_t>(radius); ++ox) {
                    out.push_back(coord_t{
                        static_cast<int32_t>(center_x + ox),
                        static_cast<int32_t>(center_y + oy)});
                }
            }
        }
    }
    return NAVSYS_STATUS_OK;
}

navsys_status_t append_all_touched_line_cells(
    const coord_t& start,
    const coord_t& end,
    uint32_t radius,
    const generate_controls_t& controls,
    std::vector<coord_t>& out) {
    const int64_t min_x = static_cast<int64_t>(std::min(start.x, end.x)) - radius;
    const int64_t max_x = static_cast<int64_t>(std::max(start.x, end.x)) + radius;
    const int64_t min_y = static_cast<int64_t>(std::min(start.y, end.y)) - radius;
    const int64_t max_y = static_cast<int64_t>(std::max(start.y, end.y)) + radius;
    if (min_x < INT32_MIN || max_x > INT32_MAX
        || min_y < INT32_MIN || max_y > INT32_MAX) {
        return NAVSYS_STATUS_LIMIT_REACHED;
    }
    const int64_t dx = static_cast<int64_t>(end.x) - start.x;
    const int64_t dy = static_cast<int64_t>(end.y) - start.y;
    const uint64_t abs_dx = static_cast<uint64_t>(std::llabs(dx));
    const uint64_t abs_dy = static_cast<uint64_t>(std::llabs(dy));
    const int64_t step_x = dx < 0 ? -1 : 1;
    const int64_t step_y = dy < 0 ? -1 : 1;
    int64_t x = start.x;
    int64_t y = start.y;
    uint64_t next_x_cross = 1;
    uint64_t next_y_cross = 1;
    uint64_t visited = 0;
    const auto append_cell = [&](int64_t center_x, int64_t center_y) {
        for (int64_t oy = -static_cast<int64_t>(radius);
             oy <= static_cast<int64_t>(radius); ++oy) {
            for (int64_t ox = -static_cast<int64_t>(radius);
                 ox <= static_cast<int64_t>(radius); ++ox) {
                out.push_back(coord_t{
                    static_cast<int32_t>(center_x + ox),
                    static_cast<int32_t>(center_y + oy)});
            }
        }
    };
    append_cell(x, y);
    while (x != end.x || y != end.y) {
        if ((visited++ & 1023u) == 0u) {
            const navsys_status_t status = poll_generate_cancel(controls);
            if (status != NAVSYS_STATUS_OK) return status;
        }
        int crossing_order = 0;
        if (x == end.x) crossing_order = 1;
        else if (y == end.y) crossing_order = -1;
        else crossing_order = compare_wide(
            multiply_wide(next_x_cross, abs_dy),
            multiply_wide(next_y_cross, abs_dx));
        if (crossing_order < 0) {
            x += step_x;
            next_x_cross += 2;
            append_cell(x, y);
        } else if (crossing_order > 0) {
            y += step_y;
            next_y_cross += 2;
            append_cell(x, y);
        } else {
            append_cell(x + step_x, y);
            append_cell(x, y + step_y);
            x += step_x;
            y += step_y;
            next_x_cross += 2;
            next_y_cross += 2;
            append_cell(x, y);
        }
    }
    return NAVSYS_STATUS_OK;
}

navsys_status_t append_raster_line_cells(
    const coord_t& start,
    const coord_t& end,
    uint32_t radius,
    const generate_controls_t& controls,
    std::vector<coord_t>& out) {
    return controls.raster_rule == OBSTACLE_RASTER_ALL_TOUCHED
        ? append_all_touched_line_cells(start, end, radius, controls, out)
        : append_line_cells(start, end, radius, controls, out);
}

navsys_status_t validate_enclosure_desc(
    const obstacle_enclosure_desc_t* desc) {
    if (!desc
        || desc->struct_size < sizeof(obstacle_enclosure_desc_t)
        || desc->abi_version != OBSTACLE_ENCLOSURE_DESC_ABI_VERSION
        || desc->width <= 0 || desc->height <= 0
        || desc->wall_thickness_cells == 0
        || desc->wall_thickness_cells > INT32_MAX
        || desc->open_side < OBSTACLE_ENCLOSURE_CLOSED
        || desc->open_side > OBSTACLE_ENCLOSURE_OPEN_DOWN) {
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    }
    navsys_status_t status = validate_positive_extent(
        desc->x0, desc->y0, desc->width, desc->height);
    if (status != NAVSYS_STATUS_OK) return status;
    const uint64_t minimum_extent =
        static_cast<uint64_t>(desc->wall_thickness_cells) * 2u + 1u;
    if (static_cast<uint64_t>(desc->width) < minimum_extent
        || static_cast<uint64_t>(desc->height) < minimum_extent) {
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    }
    if (desc->open_side == OBSTACLE_ENCLOSURE_CLOSED) {
        return desc->aperture_offset_cells == 0
                && desc->aperture_length_cells == 0
            ? NAVSYS_STATUS_OK : NAVSYS_STATUS_INVALID_ARGUMENT;
    }
    if (desc->aperture_length_cells == 0)
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    const uint64_t side_length =
        desc->open_side == OBSTACLE_ENCLOSURE_OPEN_UP
            || desc->open_side == OBSTACLE_ENCLOSURE_OPEN_DOWN
        ? static_cast<uint64_t>(desc->width)
        : static_cast<uint64_t>(desc->height);
    const uint64_t offset = desc->aperture_offset_cells;
    const uint64_t length = desc->aperture_length_cells;
    return offset <= side_length && length <= side_length - offset
        ? NAVSYS_STATUS_OK : NAVSYS_STATUS_INVALID_ARGUMENT;
}

navsys_status_t validate_cross_desc(const obstacle_cross_desc_t* desc) {
    if (!desc
        || desc->struct_size < sizeof(obstacle_cross_desc_t)
        || desc->abi_version != OBSTACLE_CROSS_DESC_ABI_VERSION
        || desc->reserved0 != 0 || desc->reserved1 != 0) {
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    }
    const uint64_t reach = static_cast<uint64_t>(desc->arm_length_cells)
        + desc->radius_cells;
    if (reach * 2u + 1u > INT32_MAX
        || static_cast<int64_t>(desc->center.x) - static_cast<int64_t>(reach)
            < INT32_MIN
        || static_cast<int64_t>(desc->center.x) + static_cast<int64_t>(reach)
            > INT32_MAX
        || static_cast<int64_t>(desc->center.y) - static_cast<int64_t>(reach)
            < INT32_MIN
        || static_cast<int64_t>(desc->center.y) + static_cast<int64_t>(reach)
            > INT32_MAX) {
        return NAVSYS_STATUS_LIMIT_REACHED;
    }
    return NAVSYS_STATUS_OK;
}

navsys_status_t validate_spiral_desc(const obstacle_spiral_desc_t* desc) {
    if (!desc
        || desc->struct_size < sizeof(obstacle_spiral_desc_t)
        || desc->abi_version != OBSTACLE_SPIRAL_DESC_ABI_VERSION
        || desc->pitch_cells == 0 || desc->reserved0 != 0
        || (desc->direction != OBSTACLE_SPIRAL_CLOCKWISE
            && desc->direction != OBSTACLE_SPIRAL_COUNTER_CLOCKWISE)
        || (desc->clip_rule != OBSTACLE_SPIRAL_CLIP_PATH_ONLY
            && desc->clip_rule != OBSTACLE_SPIRAL_CLIP_OUTPUT)) {
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    }
    const uint64_t reach = desc->clip_rule == OBSTACLE_SPIRAL_CLIP_OUTPUT
        ? static_cast<uint64_t>(desc->max_radius_cells)
        : static_cast<uint64_t>(desc->max_radius_cells)
            + desc->path_radius_cells;
    if (reach * 2u + 1u > INT32_MAX
        || static_cast<int64_t>(desc->center.x) - static_cast<int64_t>(reach)
            < INT32_MIN
        || static_cast<int64_t>(desc->center.x) + static_cast<int64_t>(reach)
            > INT32_MAX
        || static_cast<int64_t>(desc->center.y) - static_cast<int64_t>(reach)
            < INT32_MIN
        || static_cast<int64_t>(desc->center.y) + static_cast<int64_t>(reach)
            > INT32_MAX) {
        return NAVSYS_STATUS_LIMIT_REACHED;
    }
    return NAVSYS_STATUS_OK;
}

} // namespace

navsys_status_t obstacle_generate_options_init(
    obstacle_generate_options_t* options) {
    if (!options) return NAVSYS_STATUS_INVALID_ARGUMENT;
    obstacle_generate_options_t initialized{};
    initialized.struct_size = sizeof(initialized);
    initialized.abi_version = OBSTACLE_GENERATE_OPTIONS_ABI_VERSION;
    initialized.raster_rule = OBSTACLE_RASTER_CELL_CENTER;
    *options = initialized;
    return NAVSYS_STATUS_OK;
}

navsys_status_t obstacle_generate_filled_rect(
    int32_t x0, int32_t y0, int32_t width, int32_t height,
    const obstacle_generate_options_t* options,
    obstacle_t** out_obstacle) {
    if (!out_obstacle) return NAVSYS_STATUS_INVALID_ARGUMENT;
    *out_obstacle = nullptr;
    generate_controls_t controls;
    navsys_status_t status = read_generate_controls(options, controls);
    if (status != NAVSYS_STATUS_OK) return status;
    status = validate_positive_extent(x0, y0, width, height);
    if (status != NAVSYS_STATUS_OK) return status;
    status = poll_generate_cancel(controls);
    if (status != NAVSYS_STATUS_OK) return status;
    const uint64_t count = static_cast<uint64_t>(width)
        * static_cast<uint64_t>(height);
    if (controls.max_cells != 0 && count > controls.max_cells)
        return NAVSYS_STATUS_LIMIT_REACHED;
    if (count > static_cast<uint64_t>(SIZE_MAX))
        return NAVSYS_STATUS_LIMIT_REACHED;
    try {
        std::vector<coord_t> coords;
        coords.reserve(static_cast<size_t>(count));
        uint64_t visited = 0;
        for (int64_t y = y0; y < static_cast<int64_t>(y0) + height; ++y) {
            for (int64_t x = x0; x < static_cast<int64_t>(x0) + width; ++x) {
                if ((visited++ & 1023u) == 0u) {
                    status = poll_generate_cancel(controls);
                    if (status != NAVSYS_STATUS_OK) return status;
                }
                coords.push_back(coord_t{
                    static_cast<int32_t>(x), static_cast<int32_t>(y)});
            }
        }
        return materialize_generated(coords, x0, y0, controls, out_obstacle);
    } catch (const std::bad_alloc&) {
        return NAVSYS_STATUS_OUT_OF_MEMORY;
    } catch (...) {
        return NAVSYS_STATUS_CORRUPT_STATE;
    }
}

navsys_status_t obstacle_generate_rect_outline(
    int32_t x0, int32_t y0, int32_t width, int32_t height,
    uint32_t wall_thickness_cells,
    const obstacle_generate_options_t* options,
    obstacle_t** out_obstacle) {
    if (!out_obstacle) return NAVSYS_STATUS_INVALID_ARGUMENT;
    *out_obstacle = nullptr;
    obstacle_enclosure_desc_t desc{};
    desc.struct_size = sizeof(desc);
    desc.abi_version = OBSTACLE_ENCLOSURE_DESC_ABI_VERSION;
    desc.x0 = x0;
    desc.y0 = y0;
    desc.width = width;
    desc.height = height;
    desc.wall_thickness_cells = wall_thickness_cells;
    desc.open_side = OBSTACLE_ENCLOSURE_CLOSED;
    return obstacle_generate_enclosure(&desc, options, out_obstacle);
}

navsys_status_t obstacle_generate_random_rect(
    int32_t x0, int32_t y0, int32_t width, int32_t height,
    double blocked_probability,
    const obstacle_generate_options_t* options,
    obstacle_t** out_obstacle) {
    if (!out_obstacle) return NAVSYS_STATUS_INVALID_ARGUMENT;
    *out_obstacle = nullptr;
    if (!std::isfinite(blocked_probability)
        || blocked_probability < 0.0 || blocked_probability > 1.0) {
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    }
    generate_controls_t controls;
    navsys_status_t status = read_generate_controls(options, controls);
    if (status != NAVSYS_STATUS_OK) return status;
    status = validate_positive_extent(x0, y0, width, height);
    if (status != NAVSYS_STATUS_OK) return status;
    status = poll_generate_cancel(controls);
    if (status != NAVSYS_STATUS_OK) return status;
    try {
        std::vector<coord_t> coords;
        uint64_t state = controls.seed;
        uint64_t visited = 0;
        for (int64_t y = y0; y < static_cast<int64_t>(y0) + height; ++y) {
            for (int64_t x = x0; x < static_cast<int64_t>(x0) + width; ++x) {
                if ((visited++ & 1023u) == 0u) {
                    status = poll_generate_cancel(controls);
                    if (status != NAVSYS_STATUS_OK) return status;
                }
                if (probability_selects(
                        splitmix64_v1(state), blocked_probability)) {
                    if (controls.max_cells != 0
                        && coords.size() >= controls.max_cells) {
                        return NAVSYS_STATUS_LIMIT_REACHED;
                    }
                    coords.push_back(coord_t{
                        static_cast<int32_t>(x), static_cast<int32_t>(y)});
                }
            }
        }
        return materialize_generated(coords, x0, y0, controls, out_obstacle);
    } catch (const std::bad_alloc&) {
        return NAVSYS_STATUS_OUT_OF_MEMORY;
    } catch (...) {
        return NAVSYS_STATUS_CORRUPT_STATE;
    }
}

navsys_status_t obstacle_generate_line(
    const coord_t* start, const coord_t* end, uint32_t radius_cells,
    const obstacle_generate_options_t* options,
    obstacle_t** out_obstacle) {
    if (!out_obstacle) return NAVSYS_STATUS_INVALID_ARGUMENT;
    *out_obstacle = nullptr;
    if (!start || !end) return NAVSYS_STATUS_INVALID_ARGUMENT;
    generate_controls_t controls;
    navsys_status_t status = read_generate_controls(options, controls);
    if (status != NAVSYS_STATUS_OK) return status;
    try {
        std::vector<coord_t> coords;
        status = controls.raster_rule == OBSTACLE_RASTER_ALL_TOUCHED
            ? append_all_touched_line_cells(
                *start, *end, radius_cells, controls, coords)
            : append_line_cells(
                *start, *end, radius_cells, controls, coords);
        if (status != NAVSYS_STATUS_OK) return status;
        return materialize_generated(
            coords, start->x, start->y, controls, out_obstacle);
    } catch (const std::bad_alloc&) {
        return NAVSYS_STATUS_OUT_OF_MEMORY;
    } catch (...) {
        return NAVSYS_STATUS_CORRUPT_STATE;
    }
}

navsys_status_t obstacle_generate_polygon(
    const coord_t* vertices, size_t vertex_count,
    obstacle_polygon_fill_rule_t fill_rule,
    const obstacle_generate_options_t* options,
    obstacle_t** out_obstacle) {
    if (!out_obstacle) return NAVSYS_STATUS_INVALID_ARGUMENT;
    *out_obstacle = nullptr;
    if (fill_rule != OBSTACLE_POLYGON_EVEN_ODD
        && fill_rule != OBSTACLE_POLYGON_NON_ZERO) {
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    }
    navsys_status_t status = validate_ring(vertices, vertex_count);
    if (status != NAVSYS_STATUS_OK) return status;
    generate_controls_t controls;
    status = read_generate_controls(options, controls);
    if (status != NAVSYS_STATUS_OK) return status;
    int32_t min_x = vertices[0].x;
    int32_t max_x = vertices[0].x;
    int32_t min_y = vertices[0].y;
    int32_t max_y = vertices[0].y;
    for (size_t index = 1; index < vertex_count; ++index) {
        min_x = std::min(min_x, vertices[index].x);
        max_x = std::max(max_x, vertices[index].x);
        min_y = std::min(min_y, vertices[index].y);
        max_y = std::max(max_y, vertices[index].y);
    }
    const int64_t width64 = static_cast<int64_t>(max_x) - min_x + 1;
    const int64_t height64 = static_cast<int64_t>(max_y) - min_y + 1;
    if (width64 > INT32_MAX || height64 > INT32_MAX)
        return NAVSYS_STATUS_LIMIT_REACHED;
    try {
        std::vector<coord_t> coords;
        uint64_t visited = 0;
        for (int64_t y = min_y; y <= max_y; ++y) {
            for (int64_t x = min_x; x <= max_x; ++x) {
                if ((visited++ & 1023u) == 0u) {
                    status = poll_generate_cancel(controls);
                    if (status != NAVSYS_STATUS_OK) return status;
                }
                const coord_t point{
                    static_cast<int32_t>(x), static_cast<int32_t>(y)};
                const bool selected = controls.raster_rule
                        == OBSTACLE_RASTER_ALL_TOUCHED
                    ? cell_touches_ring(
                        point, vertices, vertex_count, fill_rule)
                    : point_covered_by_ring(
                        point, vertices, vertex_count, fill_rule);
                if (selected) {
                    if (controls.max_cells != 0
                        && coords.size() >= controls.max_cells) {
                        return NAVSYS_STATUS_LIMIT_REACHED;
                    }
                    coords.push_back(point);
                }
            }
        }
        return materialize_generated(
            coords, min_x, min_y, controls, out_obstacle);
    } catch (const std::bad_alloc&) {
        return NAVSYS_STATUS_OUT_OF_MEMORY;
    } catch (...) {
        return NAVSYS_STATUS_CORRUPT_STATE;
    }
}

navsys_status_t obstacle_generate_polygon_outline(
    const coord_t* vertices, size_t vertex_count, uint32_t radius_cells,
    const obstacle_generate_options_t* options,
    obstacle_t** out_obstacle) {
    if (!out_obstacle) return NAVSYS_STATUS_INVALID_ARGUMENT;
    *out_obstacle = nullptr;
    navsys_status_t status = validate_ring(vertices, vertex_count);
    if (status != NAVSYS_STATUS_OK) return status;
    generate_controls_t controls;
    status = read_generate_controls(options, controls);
    if (status != NAVSYS_STATUS_OK) return status;
    int32_t min_x = vertices[0].x;
    int32_t max_x = vertices[0].x;
    int32_t min_y = vertices[0].y;
    int32_t max_y = vertices[0].y;
    for (size_t index = 1; index < vertex_count; ++index) {
        min_x = std::min(min_x, vertices[index].x);
        max_x = std::max(max_x, vertices[index].x);
        min_y = std::min(min_y, vertices[index].y);
        max_y = std::max(max_y, vertices[index].y);
    }
    const int64_t expanded_width = static_cast<int64_t>(max_x) - min_x
        + static_cast<uint64_t>(radius_cells) * 2u + 1;
    const int64_t expanded_height = static_cast<int64_t>(max_y) - min_y
        + static_cast<uint64_t>(radius_cells) * 2u + 1;
    if (expanded_width > INT32_MAX || expanded_height > INT32_MAX)
        return NAVSYS_STATUS_LIMIT_REACHED;
    try {
        std::vector<coord_t> coords;
        for (size_t index = 0; index < vertex_count; ++index) {
            status = controls.raster_rule == OBSTACLE_RASTER_ALL_TOUCHED
                ? append_all_touched_line_cells(
                    vertices[index], vertices[(index + 1) % vertex_count],
                    radius_cells, controls, coords)
                : append_line_cells(
                    vertices[index], vertices[(index + 1) % vertex_count],
                    radius_cells, controls, coords);
            if (status != NAVSYS_STATUS_OK) return status;
        }
        return materialize_generated(
            coords, vertices[0].x, vertices[0].y,
            controls, out_obstacle);
    } catch (const std::bad_alloc&) {
        return NAVSYS_STATUS_OUT_OF_MEMORY;
    } catch (...) {
        return NAVSYS_STATUS_CORRUPT_STATE;
    }
}

navsys_status_t obstacle_generate_triangle(
    const coord_t* a, const coord_t* b, const coord_t* c,
    obstacle_polygon_fill_rule_t fill_rule,
    const obstacle_generate_options_t* options,
    obstacle_t** out_obstacle) {
    if (!out_obstacle) return NAVSYS_STATUS_INVALID_ARGUMENT;
    *out_obstacle = nullptr;
    if (!a || !b || !c) return NAVSYS_STATUS_INVALID_ARGUMENT;
    const coord_t vertices[3] = {*a, *b, *c};
    return obstacle_generate_polygon(
        vertices, 3, fill_rule, options, out_obstacle);
}

navsys_status_t obstacle_generate_triangle_outline(
    const coord_t* a, const coord_t* b, const coord_t* c,
    uint32_t radius_cells,
    const obstacle_generate_options_t* options,
    obstacle_t** out_obstacle) {
    if (!out_obstacle) return NAVSYS_STATUS_INVALID_ARGUMENT;
    *out_obstacle = nullptr;
    if (!a || !b || !c) return NAVSYS_STATUS_INVALID_ARGUMENT;
    const coord_t vertices[3] = {*a, *b, *c};
    return obstacle_generate_polygon_outline(
        vertices, 3, radius_cells, options, out_obstacle);
}

navsys_status_t obstacle_enclosure_desc_init(
    obstacle_enclosure_desc_t* desc) {
    if (!desc) return NAVSYS_STATUS_INVALID_ARGUMENT;
    obstacle_enclosure_desc_t initialized{};
    initialized.struct_size = sizeof(initialized);
    initialized.abi_version = OBSTACLE_ENCLOSURE_DESC_ABI_VERSION;
    initialized.width = 3;
    initialized.height = 3;
    initialized.wall_thickness_cells = 1;
    initialized.open_side = OBSTACLE_ENCLOSURE_CLOSED;
    *desc = initialized;
    return NAVSYS_STATUS_OK;
}

navsys_status_t obstacle_cross_desc_init(obstacle_cross_desc_t* desc) {
    if (!desc) return NAVSYS_STATUS_INVALID_ARGUMENT;
    obstacle_cross_desc_t initialized{};
    initialized.struct_size = sizeof(initialized);
    initialized.abi_version = OBSTACLE_CROSS_DESC_ABI_VERSION;
    *desc = initialized;
    return NAVSYS_STATUS_OK;
}

navsys_status_t obstacle_spiral_desc_init(obstacle_spiral_desc_t* desc) {
    if (!desc) return NAVSYS_STATUS_INVALID_ARGUMENT;
    obstacle_spiral_desc_t initialized{};
    initialized.struct_size = sizeof(initialized);
    initialized.abi_version = OBSTACLE_SPIRAL_DESC_ABI_VERSION;
    initialized.pitch_cells = 1;
    initialized.direction = OBSTACLE_SPIRAL_CLOCKWISE;
    initialized.clip_rule = OBSTACLE_SPIRAL_CLIP_PATH_ONLY;
    *desc = initialized;
    return NAVSYS_STATUS_OK;
}

navsys_status_t obstacle_generate_enclosure(
    const obstacle_enclosure_desc_t* desc,
    const obstacle_generate_options_t* options,
    obstacle_t** out_obstacle) {
    if (!out_obstacle) return NAVSYS_STATUS_INVALID_ARGUMENT;
    *out_obstacle = nullptr;
    navsys_status_t status = validate_enclosure_desc(desc);
    if (status != NAVSYS_STATUS_OK) return status;
    generate_controls_t controls;
    status = read_generate_controls(options, controls);
    if (status != NAVSYS_STATUS_OK) return status;
    try {
        std::vector<coord_t> coords;
        const int64_t thickness = desc->wall_thickness_cells;
        const int64_t aperture_begin = desc->aperture_offset_cells;
        const int64_t aperture_end = aperture_begin
            + desc->aperture_length_cells;
        uint64_t visited = 0;
        for (int64_t ry = 0; ry < desc->height; ++ry) {
            for (int64_t rx = 0; rx < desc->width; ++rx) {
                if ((visited++ & 1023u) == 0u) {
                    status = poll_generate_cancel(controls);
                    if (status != NAVSYS_STATUS_OK) return status;
                }
                const bool wall = rx < thickness
                    || rx >= static_cast<int64_t>(desc->width) - thickness
                    || ry < thickness
                    || ry >= static_cast<int64_t>(desc->height) - thickness;
                if (!wall) continue;
                bool aperture = false;
                switch (desc->open_side) {
                case OBSTACLE_ENCLOSURE_OPEN_UP:
                    aperture = ry < thickness
                        && aperture_begin <= rx && rx < aperture_end;
                    break;
                case OBSTACLE_ENCLOSURE_OPEN_DOWN:
                    aperture = ry >= static_cast<int64_t>(desc->height) - thickness
                        && aperture_begin <= rx && rx < aperture_end;
                    break;
                case OBSTACLE_ENCLOSURE_OPEN_LEFT:
                    aperture = rx < thickness
                        && aperture_begin <= ry && ry < aperture_end;
                    break;
                case OBSTACLE_ENCLOSURE_OPEN_RIGHT:
                    aperture = rx >= static_cast<int64_t>(desc->width) - thickness
                        && aperture_begin <= ry && ry < aperture_end;
                    break;
                default:
                    break;
                }
                if (aperture) continue;
                if (controls.max_cells != 0
                    && coords.size() >= controls.max_cells) {
                    return NAVSYS_STATUS_LIMIT_REACHED;
                }
                coords.push_back(coord_t{
                    static_cast<int32_t>(static_cast<int64_t>(desc->x0) + rx),
                    static_cast<int32_t>(static_cast<int64_t>(desc->y0) + ry)});
            }
        }
        return materialize_generated_in_extent(
            coords, desc->x0, desc->y0, desc->width, desc->height,
            controls, out_obstacle);
    } catch (const std::bad_alloc&) {
        return NAVSYS_STATUS_OUT_OF_MEMORY;
    } catch (...) {
        return NAVSYS_STATUS_CORRUPT_STATE;
    }
}

navsys_status_t obstacle_generate_cross(
    const obstacle_cross_desc_t* desc,
    const obstacle_generate_options_t* options,
    obstacle_t** out_obstacle) {
    if (!out_obstacle) return NAVSYS_STATUS_INVALID_ARGUMENT;
    *out_obstacle = nullptr;
    navsys_status_t status = validate_cross_desc(desc);
    if (status != NAVSYS_STATUS_OK) return status;
    generate_controls_t controls;
    status = read_generate_controls(options, controls);
    if (status != NAVSYS_STATUS_OK) return status;
    try {
        std::vector<coord_t> coords;
        const int64_t arm = desc->arm_length_cells;
        const coord_t horizontal_start = {
            static_cast<int32_t>(static_cast<int64_t>(desc->center.x) - arm),
            desc->center.y};
        const coord_t horizontal_end = {
            static_cast<int32_t>(static_cast<int64_t>(desc->center.x) + arm),
            desc->center.y};
        const coord_t vertical_start = {
            desc->center.x,
            static_cast<int32_t>(static_cast<int64_t>(desc->center.y) - arm)};
        const coord_t vertical_end = {
            desc->center.x,
            static_cast<int32_t>(static_cast<int64_t>(desc->center.y) + arm)};
        status = append_raster_line_cells(
            horizontal_start, horizontal_end, desc->radius_cells,
            controls, coords);
        if (status != NAVSYS_STATUS_OK) return status;
        status = append_raster_line_cells(
            vertical_start, vertical_end, desc->radius_cells,
            controls, coords);
        if (status != NAVSYS_STATUS_OK) return status;
        return materialize_generated(
            coords, desc->center.x, desc->center.y,
            controls, out_obstacle);
    } catch (const std::bad_alloc&) {
        return NAVSYS_STATUS_OUT_OF_MEMORY;
    } catch (...) {
        return NAVSYS_STATUS_CORRUPT_STATE;
    }
}

navsys_status_t obstacle_generate_spiral(
    const obstacle_spiral_desc_t* desc,
    const obstacle_generate_options_t* options,
    obstacle_t** out_obstacle) {
    if (!out_obstacle) return NAVSYS_STATUS_INVALID_ARGUMENT;
    *out_obstacle = nullptr;
    navsys_status_t status = validate_spiral_desc(desc);
    if (status != NAVSYS_STATUS_OK) return status;
    generate_controls_t controls;
    status = read_generate_controls(options, controls);
    if (status != NAVSYS_STATUS_OK) return status;
    try {
        std::vector<coord_t> coords;
        const int clockwise_dx[4] = {1, 0, -1, 0};
        const int clockwise_dy[4] = {0, 1, 0, -1};
        const int counter_clockwise_dx[4] = {0, 1, 0, -1};
        const int counter_clockwise_dy[4] = {1, 0, -1, 0};
        const int* dx = desc->direction == OBSTACLE_SPIRAL_CLOCKWISE
            ? clockwise_dx : counter_clockwise_dx;
        const int* dy = desc->direction == OBSTACLE_SPIRAL_CLOCKWISE
            ? clockwise_dy : counter_clockwise_dy;
        uint64_t visited = 0;
        const auto append_center = [&](int64_t center_x, int64_t center_y) {
            for (int64_t oy = -static_cast<int64_t>(desc->path_radius_cells);
                 oy <= static_cast<int64_t>(desc->path_radius_cells); ++oy) {
                for (int64_t ox = -static_cast<int64_t>(desc->path_radius_cells);
                     ox <= static_cast<int64_t>(desc->path_radius_cells); ++ox) {
                    if ((visited++ & 1023u) == 0u) {
                        const navsys_status_t poll = poll_generate_cancel(controls);
                        if (poll != NAVSYS_STATUS_OK) return poll;
                    }
                    const int64_t x = center_x + ox;
                    const int64_t y = center_y + oy;
                    if (desc->clip_rule == OBSTACLE_SPIRAL_CLIP_OUTPUT
                        && (std::llabs(x - desc->center.x) > desc->max_radius_cells
                            || std::llabs(y - desc->center.y)
                                > desc->max_radius_cells)) {
                        continue;
                    }
                    coords.push_back(coord_t{
                        static_cast<int32_t>(x), static_cast<int32_t>(y)});
                }
            }
            return NAVSYS_STATUS_OK;
        };
        int64_t x = desc->center.x;
        int64_t y = desc->center.y;
        status = append_center(x, y);
        if (status != NAVSYS_STATUS_OK) return status;
        uint64_t leg_length = desc->pitch_cells;
        uint32_t direction_index = 0;
        uint64_t completed_legs = 0;
        bool complete = false;
        while (!complete) {
            for (uint64_t step = 0; step < leg_length; ++step) {
                const int64_t next_x = x + dx[direction_index];
                const int64_t next_y = y + dy[direction_index];
                if (std::llabs(next_x - desc->center.x) > desc->max_radius_cells
                    || std::llabs(next_y - desc->center.y)
                        > desc->max_radius_cells) {
                    complete = true;
                    break;
                }
                x = next_x;
                y = next_y;
                status = append_center(x, y);
                if (status != NAVSYS_STATUS_OK) return status;
            }
            if (complete) break;
            direction_index = (direction_index + 1u) % 4u;
            ++completed_legs;
            if ((completed_legs & 1u) == 0u)
                leg_length += desc->pitch_cells;
        }
        if (desc->clip_rule == OBSTACLE_SPIRAL_CLIP_OUTPUT) {
            const int64_t radius = desc->max_radius_cells;
            return materialize_generated_in_extent(
                coords,
                static_cast<int32_t>(static_cast<int64_t>(desc->center.x) - radius),
                static_cast<int32_t>(static_cast<int64_t>(desc->center.y) - radius),
                static_cast<int32_t>(radius * 2 + 1),
                static_cast<int32_t>(radius * 2 + 1),
                controls, out_obstacle);
        }
        return materialize_generated(
            coords, desc->center.x, desc->center.y,
            controls, out_obstacle);
    } catch (const std::bad_alloc&) {
        return NAVSYS_STATUS_OUT_OF_MEMORY;
    } catch (...) {
        return NAVSYS_STATUS_CORRUPT_STATE;
    }
}

obstacle_t* obstacle_make_rect_all_blocked(
    int x0, int y0, int width, int height) {
    if (width <= 0 || height <= 0) return nullptr;
    obstacle_t* obstacle = nullptr;
    return obstacle_generate_filled_rect(
            x0, y0, width, height, nullptr, &obstacle)
            == NAVSYS_STATUS_OK
        ? obstacle : nullptr;
}

obstacle_t* obstacle_make_rect_random_blocked(
    int x0, int y0, int width, int height, float ratio) {

    if (width <= 0 || height <= 0 || ratio <= 0.0f) return nullptr;
    if (ratio > 1.0f) ratio = 1.0f;

    obstacle_t* obstacle = obstacle_create_full(x0, y0, width, height);
    if (!obstacle) return nullptr;

    srand((unsigned int)time(nullptr));

    for (int dy = 0; dy < height; ++dy) {
        for (int dx = 0; dx < width; ++dx) {
            if ((float)rand() / RAND_MAX <= ratio) {
                coord_t c = { x0 + dx, y0 + dy };
                coord_hash_insert(obstacle->blocked, &c, nullptr);
            }
        }
    }
    return obstacle;
}

obstacle_t* obstacle_make_beam(
    const coord_t* start, const coord_t* goal, int range){

    if (!start || !goal) return nullptr;

    int width = goal->x - start->x;
    int height = goal->y - start->y;
    obstacle_t* obstacle = obstacle_create_full(
        start->x, start->y, width, height);

    coord_t* cur = coord_copy(start);

    if(range <= 0){
        while(!coord_equal(cur, goal)){
            coord_t* next = coord_clone_next_to_goal(cur, goal);

            if (!obstacle_is_coord_blocked(obstacle, next->x, next->y)) {
                obstacle_block_coord(obstacle, next->x, next->y);
            }
            coord_set(cur, next->x, next->y);
            coord_destroy(next);
        }
    }

    // while(cur != goal){
    while(!coord_equal(cur, goal)){
        coord_t* next = coord_clone_next_to_goal(cur, goal);

        coord_list_t* neighbors = obstacle_create_neighbors_all_range(
            obstacle, next->x, next->y, range-1);
        for(int i=0; i < coord_list_length(neighbors); i++){
            const coord_t* c = coord_list_get(neighbors, i);

            if (!obstacle_is_coord_blocked(obstacle, c->x, c->y)){
                obstacle_block_coord(obstacle, c->x, c->y);
            }
        }
        coord_set(cur, next->x, next->y);
        coord_list_destroy(neighbors);
        coord_destroy(next);
    }
    coord_destroy(cur);
    return obstacle;        
}

obstacle_t* obstacle_make_torus(
    const coord_t* start, const coord_t* goal, int thickness)
{
    if (!start || !goal || thickness <= 0) return NULL;

    const int32_t min_x = std::min(start->x, goal->x);
    const int32_t min_y = std::min(start->y, goal->y);
    const int64_t width = static_cast<int64_t>(std::max(start->x, goal->x))
        - min_x + 1;
    const int64_t height = static_cast<int64_t>(std::max(start->y, goal->y))
        - min_y + 1;
    if (width > INT32_MAX || height > INT32_MAX) return nullptr;
    obstacle_t* obstacle = nullptr;
    return obstacle_generate_rect_outline(
            min_x, min_y,
            static_cast<int32_t>(width), static_cast<int32_t>(height),
            static_cast<uint32_t>(thickness), nullptr, &obstacle)
            == NAVSYS_STATUS_OK
        ? obstacle : nullptr;
}

obstacle_t* obstacle_make_enclosure(
    const coord_t* start, const coord_t* goal, int thickness,
    enclosure_open_dir_t open)
{
    if (!start || !goal || thickness <= 0) return NULL;

    int min_x = (start->x < goal->x) ? start->x : goal->x;
    int max_x = (start->x > goal->x) ? start->x : goal->x;
    int min_y = (start->y < goal->y) ? start->y : goal->y;
    int max_y = (start->y > goal->y) ? start->y : goal->y;

    int width = max_x - min_x + 1;
    int height = max_y - min_y + 1;

    obstacle_t* obs = obstacle_create_full(min_x, min_y, width, height);

    for (int x = 0; x < width; ++x) {
        for (int y = 0; y < height; ++y) {
            bool block = false;

            bool is_top = (y < thickness);
            bool is_bottom = (y >= height - thickness);
            bool is_left = (x < thickness);
            bool is_right = (x >= width - thickness);

            if (is_top && open != ENCLOSURE_OPEN_UP)
                block = true;
            else if (is_bottom && open != ENCLOSURE_OPEN_DOWN)
                block = true;
            else if (is_left && open != ENCLOSURE_OPEN_LEFT)
                block = true;
            else if (is_right && open != ENCLOSURE_OPEN_RIGHT)
                block = true;

            if (block)
                obstacle_block_coord(obs, min_x + x, min_y + y);
        }
    }

    return obs;
}

obstacle_t* obstacle_make_cross(
    const coord_t* center, int length, int range)
{
    if (!center || length < 0 || range < 0)
        return NULL;

    obstacle_cross_desc_t desc{};
    if (obstacle_cross_desc_init(&desc) != NAVSYS_STATUS_OK) return nullptr;
    desc.center = *center;
    desc.arm_length_cells = static_cast<uint32_t>(length);
    desc.radius_cells = static_cast<uint32_t>(range);
    obstacle_t* obstacle = nullptr;
    return obstacle_generate_cross(&desc, nullptr, &obstacle)
            == NAVSYS_STATUS_OK
        ? obstacle : nullptr;
}

obstacle_t* obstacle_make_spiral(
    const coord_t* center,
    int radius,
    int turns,
    int range,
    int gap,
    spiral_dir_t direction)
{
    if (!center || radius <= 0 || turns <= 0 || range < 0 || gap < 0)
        return NULL;

    int min_x = center->x - radius;
    int max_x = center->x + radius;
    int min_y = center->y - radius;
    int max_y = center->y + radius;

    int width = max_x - min_x + 1;
    int height = max_y - min_y + 1;

    obstacle_t* obs = obstacle_create_full(min_x, min_y, width, height);
    if (!obs) return NULL;

    const int dx_cw[4]  = { 1, 0, -1,  0};
    const int dy_cw[4]  = { 0, 1,  0, -1};

    const int dx_ccw[4] = { 0, 1,  0, -1};
    const int dy_ccw[4] = { 1, 0, -1,  0};

    const int* dx = (direction == SPIRAL_COUNTER_CLOCKWISE) ? dx_ccw : dx_cw;
    const int* dy = (direction == SPIRAL_COUNTER_CLOCKWISE) ? dy_ccw : dy_cw;

    int cx = center->x;
    int cy = center->y;

    int len = 1;
    int dir = 0;
    int step = 0;
    int max_steps = turns * 4;

    if (range == 0) {
        obstacle_block_coord(obs, cx, cy);
    } else {
        size_t changed_count = 0;
        (void)obstacle_block_square(
            obs, cx, cy, range, &changed_count);
    }

    while (step < max_steps) {
        bool active_turn = (gap == 0 || step % (gap + 1) == 0);

        for (int i = 0; i < len; ++i) {
            cx += dx[dir];
            cy += dy[dir];

            if (!active_turn)
                continue;

            if (range == 0) {
                obstacle_block_coord(obs, cx, cy);
            } else {
                size_t changed_count = 0;
                (void)obstacle_block_square(
                    obs, cx, cy, range, &changed_count);
            }
        }

        dir = (dir + 1) % 4;
        step++;

        if (step % 2 == 0)
            len++;
    }

    return obs;
}

static bool is_point_in_triangle(int px, int py, 
    const coord_t* a, const coord_t* b, const coord_t* c) {
    int ax = a->x, ay = a->y;
    int bx = b->x, by = b->y;
    int cx = c->x, cy = c->y;

    int v0x = cx - ax;
    int v0y = cy - ay;
    int v1x = bx - ax;
    int v1y = by - ay;
    int v2x = px - ax;
    int v2y = py - ay;

    int dot00 = v0x * v0x + v0y * v0y;
    int dot01 = v0x * v1x + v0y * v1y;
    int dot02 = v0x * v2x + v0y * v2y;
    int dot11 = v1x * v1x + v1y * v1y;
    int dot12 = v1x * v2x + v1y * v2y;

    int denom = dot00 * dot11 - dot01 * dot01;
    if (denom == 0) return false;

    float u = (float)(dot11 * dot02 - dot01 * dot12) / denom;
    float v = (float)(dot00 * dot12 - dot01 * dot02) / denom;

    return (u >= 0 && v >= 0 && u + v <= 1.0f);
}

obstacle_t* obstacle_make_triangle(
    const coord_t* a, const coord_t* b, const coord_t* c)
{
    if (!a || !b || !c) return NULL;

    int min_x = a->x;
    int max_x = a->x;
    int min_y = a->y;
    int max_y = a->y;

    const coord_t* pts[3] = {a, b, c};
    for (int i = 1; i < 3; ++i) {
        if (pts[i]->x < min_x) min_x = pts[i]->x;
        if (pts[i]->x > max_x) max_x = pts[i]->x;
        if (pts[i]->y < min_y) min_y = pts[i]->y;
        if (pts[i]->y > max_y) max_y = pts[i]->y;
    }

    int width = max_x - min_x + 1;
    int height = max_y - min_y + 1;

    obstacle_t* obs = obstacle_create_full(min_x, min_y, width, height);
    if (!obs) return NULL;

    for (int x = min_x; x <= max_x; ++x) {
        for (int y = min_y; y <= max_y; ++y) {
            if (is_point_in_triangle(x, y, a, b, c)) {
                obstacle_block_coord(obs, x, y);
            }
        }
    }

    return obs;
}

static void block_line_segment(obstacle_t* obs, 
    int x0, int y0, int x1, int y1, int thickness) {
    size_t changed_count = 0;
    (void)obstacle_block_line(
        obs, x0, y0, x1, y1, thickness, &changed_count);
}

obstacle_t* obstacle_make_triangle_torus(
    const coord_t* a,
    const coord_t* b,
    const coord_t* c,
    int thickness)
{
    if (!a || !b || !c || thickness < 0) return NULL;

    int min_x = a->x, max_x = a->x;
    int min_y = a->y, max_y = a->y;

    const coord_t* pts[3] = {a, b, c};
    for (int i = 1; i < 3; ++i) {
        if (pts[i]->x < min_x) min_x = pts[i]->x;
        if (pts[i]->x > max_x) max_x = pts[i]->x;
        if (pts[i]->y < min_y) min_y = pts[i]->y;
        if (pts[i]->y > max_y) max_y = pts[i]->y;
    }

    int width = max_x - min_x + 1 + thickness * 2;
    int height = max_y - min_y + 1 + thickness * 2;

    obstacle_t* obs = obstacle_create_full(
        min_x - thickness, min_y - thickness, width, height);

    if (!obs) return NULL;

    block_line_segment(obs, a->x, a->y, b->x, b->y, thickness);
    block_line_segment(obs, b->x, b->y, c->x, c->y, thickness);
    block_line_segment(obs, c->x, c->y, a->x, a->y, thickness);

    return obs;
}

static bool point_in_polygon(int x, int y, const coord_list_t* list) {
    int count = coord_list_length(list);
    if (count < 3) return false;

    bool inside = false;

    for (int i = 0, j = count - 1; i < count; j = i++) {
        const coord_t* a = coord_list_get(list, j);
        const coord_t* b = coord_list_get(list, i);
        if (!a || !b) continue;

        int x1 = a->x, y1 = a->y;
        int x2 = b->x, y2 = b->y;

        if ((y1 > y) != (y2 > y)) {
            float intersect = (float)(x2 - x1) * (y - y1) / (float)(y2 - y1) + x1;
            if (x < intersect)
                inside = !inside;
        }
    }

    return inside;
}

obstacle_t* obstacle_make_polygon(coord_list_t* list) {
    int count = coord_list_length(list);
    if (count < 3) return NULL;

    const coord_t* c0 = coord_list_get(list, 0);
    if (!c0) return NULL;

    int min_x = c0->x, max_x = c0->x;
    int min_y = c0->y, max_y = c0->y;

    for (int i = 1; i < count; ++i) {
        const coord_t* c = coord_list_get(list, i);
        if (!c) continue;

        if (c->x < min_x) min_x = c->x;
        if (c->x > max_x) max_x = c->x;
        if (c->y < min_y) min_y = c->y;
        if (c->y > max_y) max_y = c->y;
    }

    int width = max_x - min_x + 1;
    int height = max_y - min_y + 1;

    obstacle_t* obs = obstacle_create_full(min_x, min_y, width, height);
    if (!obs) return NULL;

    for (int x = min_x; x <= max_x; ++x) {
        for (int y = min_y; y <= max_y; ++y) {
            if (point_in_polygon(x, y, list)) {
                obstacle_block_coord(obs, x, y);
            }
        }
    }

    return obs;
}

obstacle_t* obstacle_make_polygon_torus(coord_list_t* list, int thickness) {
    int count = coord_list_length(list);
    if (count < 3 || thickness < 0) return NULL;

    const coord_t* first = coord_list_get(list, 0);
    if (!first) return NULL;

    int min_x = first->x, max_x = first->x;
    int min_y = first->y, max_y = first->y;

    for (int i = 1; i < count; ++i) {
        const coord_t* c = coord_list_get(list, i);
        if (!c) continue;
        if (c->x < min_x) min_x = c->x;
        if (c->x > max_x) max_x = c->x;
        if (c->y < min_y) min_y = c->y;
        if (c->y > max_y) max_y = c->y;
    }

    int width = max_x - min_x + 1 + thickness * 2;
    int height = max_y - min_y + 1 + thickness * 2;

    obstacle_t* obs = obstacle_create_full(min_x - thickness, min_y - thickness, width, height);
    if (!obs) return NULL;

    for (int i = 0; i < count; ++i) {
        const coord_t* a = coord_list_get(list, i);
        const coord_t* b = coord_list_get(list, (i + 1) % count);
        if (a && b) {
            block_line_segment(obs, a->x, a->y, b->x, b->y, thickness);
        }
    }

    return obs;
}
