#include "internal/route_carver_geometry.hpp"

#include <algorithm>
#include <cstdint>
#include <limits>
#include <new>
#include <numeric>
#include <utility>
#include <vector>

namespace byul::navsys::route_carver::internal {
namespace {

using seed_callback = navsys_status_t (*)(int64_t, int64_t, void*);

bool valid_metric(carve_metric metric) {
    return metric == carve_metric::chebyshev_square
        || metric == carve_metric::manhattan_diamond
        || metric == carve_metric::euclidean_disk;
}

bool valid_coverage(line_coverage coverage) {
    return coverage == line_coverage::center_cells
        || coverage == line_coverage::supercover_cells;
}

bool valid_extent(const carve_extent& extent) {
    return (!extent.bounded_x || extent.min_x < extent.max_x)
        && (!extent.bounded_y || extent.min_y < extent.max_y);
}

navsys_status_t poll_cancel(const carve_candidate_options& options) {
    if (!options.cancel_func) return NAVSYS_STATUS_OK;
    try {
        return options.cancel_func(options.cancel_userdata)
            ? NAVSYS_STATUS_CANCELLED
            : NAVSYS_STATUS_OK;
    } catch (...) {
        return NAVSYS_STATUS_CALLBACK_FAILED;
    }
}

bool inside_extent(const carve_extent& extent, int64_t x, int64_t y) {
    return (!extent.bounded_x || (x >= extent.min_x && x < extent.max_x))
        && (!extent.bounded_y || (y >= extent.min_y && y < extent.max_y));
}

uint64_t absolute_delta(int lhs, int rhs) {
    const int64_t delta = static_cast<int64_t>(lhs) - rhs;
    return static_cast<uint64_t>(delta < 0 ? -delta : delta);
}

int compare_positive_fractions(
    uint64_t lhs_numerator,
    uint64_t lhs_denominator,
    uint64_t rhs_numerator,
    uint64_t rhs_denominator) {
    bool inverted = false;
    for (;;) {
        const uint64_t lhs_quotient = lhs_numerator / lhs_denominator;
        const uint64_t rhs_quotient = rhs_numerator / rhs_denominator;
        if (lhs_quotient != rhs_quotient) {
            const int comparison = lhs_quotient < rhs_quotient ? -1 : 1;
            return inverted ? -comparison : comparison;
        }
        const uint64_t lhs_remainder = lhs_numerator % lhs_denominator;
        const uint64_t rhs_remainder = rhs_numerator % rhs_denominator;
        if (lhs_remainder == 0 || rhs_remainder == 0) {
            if (lhs_remainder == rhs_remainder) return 0;
            const int comparison = lhs_remainder == 0 ? -1 : 1;
            return inverted ? -comparison : comparison;
        }
        lhs_numerator = lhs_denominator;
        lhs_denominator = lhs_remainder;
        rhs_numerator = rhs_denominator;
        rhs_denominator = rhs_remainder;
        inverted = !inverted;
    }
}

navsys_status_t enumerate_center_seeds(
    const coord_t& start,
    const coord_t& end,
    seed_callback callback,
    void* userdata) {
    const uint64_t delta_x = absolute_delta(end.x, start.x);
    const uint64_t delta_y = absolute_delta(end.y, start.y);
    const bool x_major = delta_x >= delta_y;
    const uint64_t major = x_major ? delta_x : delta_y;
    const uint64_t minor = x_major ? delta_y : delta_x;
    if (major + 1 > maximum_raster_steps)
        return NAVSYS_STATUS_LIMIT_REACHED;

    const int64_t major_start = x_major ? start.x : start.y;
    const int64_t minor_start = x_major ? start.y : start.x;
    const int major_sign = (x_major ? end.x : end.y) >= major_start ? 1 : -1;
    const int minor_sign = (x_major ? end.y : end.x) >= minor_start ? 1 : -1;
    uint64_t quotient = 0;
    uint64_t remainder = 0;
    for (uint64_t index = 0; index <= major; ++index) {
        const int64_t major_value = major_start
            + static_cast<int64_t>(index) * major_sign;
        const int64_t minor_base = minor_start
            + static_cast<int64_t>(quotient) * minor_sign;
        const uint64_t doubled_remainder = remainder * 2;
        const int64_t nearest = minor_base
            + (doubled_remainder > major ? minor_sign : 0);
        navsys_status_t status = x_major
            ? callback(major_value, nearest, userdata)
            : callback(nearest, major_value, userdata);
        if (status != NAVSYS_STATUS_OK) return status;
        if (major != 0 && doubled_remainder == major) {
            const int64_t tied = minor_base + minor_sign;
            status = x_major
                ? callback(major_value, tied, userdata)
                : callback(tied, major_value, userdata);
            if (status != NAVSYS_STATUS_OK) return status;
        }
        if (index != major) {
            remainder += minor;
            if (remainder >= major) {
                remainder -= major;
                ++quotient;
            }
        }
    }
    return NAVSYS_STATUS_OK;
}

navsys_status_t enumerate_supercover_seeds(
    const coord_t& start,
    const coord_t& end,
    seed_callback callback,
    void* userdata) {
    const uint64_t delta_x = absolute_delta(end.x, start.x);
    const uint64_t delta_y = absolute_delta(end.y, start.y);
    if (delta_x + delta_y + 1 > maximum_raster_steps)
        return NAVSYS_STATUS_LIMIT_REACHED;

    int64_t x = start.x;
    int64_t y = start.y;
    const int step_x = end.x >= start.x ? 1 : -1;
    const int step_y = end.y >= start.y ? 1 : -1;
    uint64_t next_x = 1;
    uint64_t next_y = 1;
    navsys_status_t status = callback(x, y, userdata);
    if (status != NAVSYS_STATUS_OK) return status;

    while (x != end.x || y != end.y) {
        if (x == end.x) {
            y += step_y;
            next_y += 2;
            status = callback(x, y, userdata);
        } else if (y == end.y) {
            x += step_x;
            next_x += 2;
            status = callback(x, y, userdata);
        } else {
            const int comparison = compare_positive_fractions(
                next_x, delta_x, next_y, delta_y);
            if (comparison < 0) {
                x += step_x;
                next_x += 2;
                status = callback(x, y, userdata);
            } else if (comparison > 0) {
                y += step_y;
                next_y += 2;
                status = callback(x, y, userdata);
            } else {
                status = callback(x + step_x, y, userdata);
                if (status != NAVSYS_STATUS_OK) return status;
                status = callback(x, y + step_y, userdata);
                if (status != NAVSYS_STATUS_OK) return status;
                x += step_x;
                y += step_y;
                next_x += 2;
                next_y += 2;
                status = callback(x, y, userdata);
            }
        }
        if (status != NAVSYS_STATUS_OK) return status;
    }
    return NAVSYS_STATUS_OK;
}

struct seed_context {
    const coord_t& start;
    const coord_t& end;
    const carve_candidate_options& options;
    std::vector<std::pair<int64_t, int64_t>>& seeds;
    size_t poll_counter;
};

navsys_status_t collect_seed(int64_t x, int64_t y, void* userdata) {
    auto& context = *static_cast<seed_context*>(userdata);
    if (++context.poll_counter == carve_poll_interval_cells) {
        context.poll_counter = 0;
        const navsys_status_t cancel_status = poll_cancel(context.options);
        if (cancel_status != NAVSYS_STATUS_OK) return cancel_status;
    }
    if (context.start.x != context.end.x || context.start.y != context.end.y) {
        if (!context.options.include_start
            && x == context.start.x && y == context.start.y) {
            return NAVSYS_STATUS_OK;
        }
        if (!context.options.include_end
            && x == context.end.x && y == context.end.y) {
            return NAVSYS_STATUS_OK;
        }
    }
    if (context.seeds.size() >= context.options.max_cells + 2)
        return NAVSYS_STATUS_LIMIT_REACHED;
    context.seeds.emplace_back(x, y);
    return NAVSYS_STATUS_OK;
}

uint64_t coordinate_key(int64_t x, int64_t y) {
    return static_cast<uint64_t>(static_cast<uint32_t>(x)) << 32
        | static_cast<uint32_t>(y);
}

uint64_t mix_key(uint64_t value) {
    value ^= value >> 30;
    value *= UINT64_C(0xbf58476d1ce4e5b9);
    value ^= value >> 27;
    value *= UINT64_C(0x94d049bb133111eb);
    return value ^ (value >> 31);
}

class unique_candidates {
public:
    unique_candidates(size_t maximum, size_t expected)
        : maximum_(maximum) {
        const size_t capacity = std::max<size_t>(1, std::min(maximum, expected));
        coordinates_.reserve(capacity);
        size_t table_size = 2;
        while (table_size < capacity * 2) table_size <<= 1;
        keys_.resize(table_size);
        occupied_.resize(table_size);
    }

    navsys_status_t add(int64_t x, int64_t y) {
        if (x < std::numeric_limits<int>::min()
            || x > std::numeric_limits<int>::max()
            || y < std::numeric_limits<int>::min()
            || y > std::numeric_limits<int>::max()) {
            return NAVSYS_STATUS_LIMIT_REACHED;
        }
        const uint64_t key = coordinate_key(x, y);
        const size_t mask = keys_.size() - 1;
        size_t slot = static_cast<size_t>(mix_key(key)) & mask;
        while (occupied_[slot]) {
            if (keys_[slot] == key) return NAVSYS_STATUS_OK;
            slot = (slot + 1) & mask;
        }
        if (coordinates_.size() >= maximum_)
            return NAVSYS_STATUS_LIMIT_REACHED;
        occupied_[slot] = 1;
        keys_[slot] = key;
        coordinates_.push_back({static_cast<int>(x), static_cast<int>(y)});
        return NAVSYS_STATUS_OK;
    }

    std::vector<coord_t> finish() {
        std::sort(coordinates_.begin(), coordinates_.end(),
            [](const coord_t& lhs, const coord_t& rhs) {
                return lhs.y != rhs.y ? lhs.y < rhs.y : lhs.x < rhs.x;
            });
        return std::move(coordinates_);
    }

private:
    size_t maximum_;
    std::vector<coord_t> coordinates_;
    std::vector<uint64_t> keys_;
    std::vector<uint8_t> occupied_;
};

uint64_t count_threshold(const carve_candidate_options& options) {
    return static_cast<uint64_t>(options.max_cells) + 2;
}

bool metric_minimum_exceeds_limit(
    uint32_t radius, const carve_candidate_options& options) {
    const uint64_t threshold = count_threshold(options);
    const uint64_t r = radius;
    if (options.metric == carve_metric::chebyshev_square) {
        const uint64_t side = r * 2 + 1;
        return side > threshold || side * side > threshold;
    }
    if (r > threshold) return true;
    const uint64_t diamond = 1 + 2 * r * (r + 1);
    return diamond > threshold;
}

bool metric_includes(carve_metric metric, int64_t dx, int64_t dy, uint32_t radius) {
    const uint64_t x = static_cast<uint64_t>(dx < 0 ? -dx : dx);
    const uint64_t y = static_cast<uint64_t>(dy < 0 ? -dy : dy);
    const uint64_t r = radius;
    if (metric == carve_metric::chebyshev_square)
        return std::max(x, y) <= r;
    if (metric == carve_metric::manhattan_diamond)
        return x + y <= r;
    if (x > r || y > r) return false;
    return x * x <= r * r - y * y;
}

navsys_status_t validate_options(
    const carve_extent& extent,
    const carve_candidate_options& options,
    bool needs_coverage) {
    if (!valid_extent(extent) || options.max_cells == 0)
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    if (options.max_cells > maximum_candidate_cells)
        return NAVSYS_STATUS_LIMIT_REACHED;
    if (!valid_metric(options.metric)
        || (needs_coverage && !valid_coverage(options.coverage))) {
        return NAVSYS_STATUS_UNSUPPORTED;
    }
    return NAVSYS_STATUS_OK;
}

navsys_status_t expand_seeds(
    const std::vector<std::pair<int64_t, int64_t>>& seeds,
    const coord_t* excluded_start,
    const coord_t* excluded_end,
    const carve_extent& extent,
    const carve_candidate_options& options,
    std::vector<coord_t>& output) {
    if (seeds.empty()) {
        std::vector<coord_t> empty;
        output.swap(empty);
        return NAVSYS_STATUS_OK;
    }
    const bool fully_bounded_clip = options.clip_to_extent
        && extent.bounded_x && extent.bounded_y;
    if (!fully_bounded_clip
        && metric_minimum_exceeds_limit(options.radius_cells, options)) {
        return NAVSYS_STATUS_LIMIT_REACHED;
    }

    unique_candidates candidates(options.max_cells, options.max_cells);
    const int64_t radius = options.radius_cells;
    size_t poll_counter = 0;
    for (const auto& seed : seeds) {
        int64_t minimum_x = -radius;
        int64_t maximum_x = radius;
        int64_t minimum_y = -radius;
        int64_t maximum_y = radius;
        if (options.clip_to_extent) {
            if (extent.bounded_x) {
                minimum_x = std::max(minimum_x, extent.min_x - seed.first);
                maximum_x = std::min(maximum_x, extent.max_x - 1 - seed.first);
            }
            if (extent.bounded_y) {
                minimum_y = std::max(minimum_y, extent.min_y - seed.second);
                maximum_y = std::min(maximum_y, extent.max_y - 1 - seed.second);
            }
        }
        if (minimum_x > maximum_x || minimum_y > maximum_y) continue;
        for (int64_t dy = minimum_y; dy <= maximum_y; ++dy) {
            navsys_status_t cancel_status = poll_cancel(options);
            if (cancel_status != NAVSYS_STATUS_OK) return cancel_status;
            for (int64_t dx = minimum_x; dx <= maximum_x; ++dx) {
                if (++poll_counter == carve_poll_interval_cells) {
                    poll_counter = 0;
                    cancel_status = poll_cancel(options);
                    if (cancel_status != NAVSYS_STATUS_OK)
                        return cancel_status;
                }
                if (!metric_includes(
                        options.metric, dx, dy, options.radius_cells)) {
                    continue;
                }
                const int64_t x = seed.first + dx;
                const int64_t y = seed.second + dy;
                if (excluded_start
                    && x == excluded_start->x && y == excluded_start->y) {
                    continue;
                }
                if (excluded_end
                    && x == excluded_end->x && y == excluded_end->y) {
                    continue;
                }
                if (!inside_extent(extent, x, y)) {
                    if (options.clip_to_extent) continue;
                    return NAVSYS_STATUS_NOT_FOUND;
                }
                const navsys_status_t status = candidates.add(x, y);
                if (status != NAVSYS_STATUS_OK) return status;
            }
        }
    }
    const navsys_status_t cancel_status = poll_cancel(options);
    if (cancel_status != NAVSYS_STATUS_OK) return cancel_status;
    std::vector<coord_t> completed = candidates.finish();
    output.swap(completed);
    return NAVSYS_STATUS_OK;
}

} // namespace

carve_extent unbounded_carve_extent() {
    return {false, false, 0, 0, 0, 0};
}

carve_extent positive_carve_extent(int width, int height) {
    return {true, true, 0, width, 0, height};
}

navsys_status_t enumerate_line_candidates(
    const coord_t& start,
    const coord_t& end,
    const carve_extent& extent,
    const carve_candidate_options& options,
    std::vector<coord_t>& output) {
    try {
        const navsys_status_t validation = validate_options(extent, options, true);
        if (validation != NAVSYS_STATUS_OK) return validation;
        const navsys_status_t initial_cancel = poll_cancel(options);
        if (initial_cancel != NAVSYS_STATUS_OK) return initial_cancel;
        if (start.x == end.x && start.y == end.y
            && !options.include_start && !options.include_end) {
            std::vector<coord_t> empty;
            output.swap(empty);
            return NAVSYS_STATUS_OK;
        }

        const uint64_t delta_x = absolute_delta(end.x, start.x);
        const uint64_t delta_y = absolute_delta(end.y, start.y);
        const uint64_t seed_upper = options.coverage == line_coverage::center_cells
            ? 2 * (std::max(delta_x, delta_y) + 1)
            : 2 * (delta_x + delta_y) + 1;
        const size_t seed_capacity = static_cast<size_t>(std::min<uint64_t>(
            seed_upper, static_cast<uint64_t>(options.max_cells) + 2));
        std::vector<std::pair<int64_t, int64_t>> seeds;
        seeds.reserve(seed_capacity);
        seed_context context{start, end, options, seeds, 0};
        const navsys_status_t status = options.coverage == line_coverage::center_cells
            ? enumerate_center_seeds(start, end, collect_seed, &context)
            : enumerate_supercover_seeds(start, end, collect_seed, &context);
        if (status != NAVSYS_STATUS_OK) return status;

        const bool distinct = start.x != end.x || start.y != end.y;
        const coord_t* excluded_start = distinct && !options.include_start ? &start : nullptr;
        const coord_t* excluded_end = distinct && !options.include_end ? &end : nullptr;
        return expand_seeds(
            seeds, excluded_start, excluded_end, extent, options, output);
    } catch (const std::bad_alloc&) {
        return NAVSYS_STATUS_OUT_OF_MEMORY;
    } catch (...) {
        return NAVSYS_STATUS_CORRUPT_STATE;
    }
}

navsys_status_t enumerate_area_candidates(
    const coord_t& center,
    const carve_extent& extent,
    const carve_candidate_options& options,
    std::vector<coord_t>& output) {
    try {
        const navsys_status_t validation = validate_options(extent, options, false);
        if (validation != NAVSYS_STATUS_OK) return validation;
        const navsys_status_t initial_cancel = poll_cancel(options);
        if (initial_cancel != NAVSYS_STATUS_OK) return initial_cancel;
        const std::vector<std::pair<int64_t, int64_t>> seeds{
            {center.x, center.y}};
        return expand_seeds(seeds, nullptr, nullptr, extent, options, output);
    } catch (const std::bad_alloc&) {
        return NAVSYS_STATUS_OUT_OF_MEMORY;
    } catch (...) {
        return NAVSYS_STATUS_CORRUPT_STATE;
    }
}

} // namespace byul::navsys::route_carver::internal
