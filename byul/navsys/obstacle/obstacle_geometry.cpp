#include "obstacle.h"
#include "internal/obstacle_private.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <new>
#include <vector>

namespace {

constexpr double pi = 3.14159265358979323846;
constexpr int legacy_dx8[] = {0, -1, 1, 0, -1, -1, 1, 1};
constexpr int legacy_dy8[] = {-1, 0, 0, 1, -1, 1, -1, 1};

struct obstacle_extent_t {
    int64_t min_x;
    int64_t max_x;
    int64_t min_y;
    int64_t max_y;
};

obstacle_extent_t obstacle_extent(const obstacle_t* obstacle) {
    const int64_t x1 = static_cast<int64_t>(obstacle->x0) + obstacle->width;
    const int64_t y1 = static_cast<int64_t>(obstacle->y0) + obstacle->height;
    return {
        obstacle->width >= 0 ? obstacle->x0 : x1,
        obstacle->width >= 0 ? x1 : obstacle->x0,
        obstacle->height >= 0 ? obstacle->y0 : y1,
        obstacle->height >= 0 ? y1 : obstacle->y0
    };
}

bool extent_contains(
    const obstacle_extent_t& extent, int64_t x, int64_t y) {
    return x >= extent.min_x && x < extent.max_x
        && y >= extent.min_y && y < extent.max_y;
}

bool valid_export_buffer(
    const void* out_buffer, size_t capacity, const size_t* out_count) {
    return out_count
        && ((!out_buffer && capacity == 0) || (out_buffer && capacity != 0));
}

double normalized_degree(double degree) {
    degree = std::fmod(degree, 360.0);
    return degree < 0.0 ? degree + 360.0 : degree;
}

double coordinate_degree(int64_t dx, int64_t dy) {
    double degree = std::atan2(
        static_cast<double>(dy), static_cast<double>(dx)) * 180.0 / pi;
    return degree < 0.0 ? degree + 360.0 : degree;
}

navsys_status_t collect_immediate_neighbors(
    const obstacle_t* obstacle,
    int32_t x,
    int32_t y,
    bool traversable_only,
    coord_t (&neighbors)[8],
    size_t& count) {
    if (!obstacle) return NAVSYS_STATUS_INVALID_ARGUMENT;
    if (!obstacle->blocked) return NAVSYS_STATUS_CORRUPT_STATE;
    const obstacle_extent_t extent = obstacle_extent(obstacle);
    if (!extent_contains(extent, x, y)) return NAVSYS_STATUS_NOT_FOUND;

    count = 0;
    for (size_t index = 0; index < 8; ++index) {
        const int64_t nx = static_cast<int64_t>(x) + legacy_dx8[index];
        const int64_t ny = static_cast<int64_t>(y) + legacy_dy8[index];
        if (!extent_contains(extent, nx, ny)) continue;
        const coord_t candidate{
            static_cast<int>(nx), static_cast<int>(ny)};
        if (traversable_only
            && coord_hash_contains(obstacle->blocked, &candidate)) {
            continue;
        }
        neighbors[count++] = candidate;
    }
    return NAVSYS_STATUS_OK;
}

navsys_status_t export_immediate_neighbors(
    const obstacle_t* obstacle,
    int32_t x,
    int32_t y,
    bool traversable_only,
    coord_t* out_coords,
    size_t capacity,
    size_t* out_count) {
    if (!valid_export_buffer(out_coords, capacity, out_count))
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    coord_t neighbors[8]{};
    size_t count = 0;
    const navsys_status_t status = collect_immediate_neighbors(
        obstacle, x, y, traversable_only, neighbors, count);
    if (status != NAVSYS_STATUS_OK) return status;
    if (!out_coords) {
        *out_count = count;
        return NAVSYS_STATUS_OK;
    }
    if (capacity < count) {
        *out_count = count;
        return NAVSYS_STATUS_INCOMPLETE;
    }
    for (size_t index = 0; index < count; ++index)
        out_coords[index] = neighbors[index];
    *out_count = count;
    return NAVSYS_STATUS_OK;
}

template <typename Exporter>
coord_list_t* create_exported_coords(Exporter exporter) {
    size_t count = 0;
    if (exporter(nullptr, 0, &count) != NAVSYS_STATUS_OK) return nullptr;
    try {
        std::vector<coord_t> coordinates(count);
        if (count != 0
            && exporter(coordinates.data(), coordinates.size(), &count)
                != NAVSYS_STATUS_OK) {
            return nullptr;
        }
        coord_list_t* result = nullptr;
        if (coord_list_create_ex(&result) != NAVSYS_STATUS_OK) return nullptr;
        if (coord_list_reserve(result, count) != NAVSYS_STATUS_OK) {
            coord_list_destroy(result);
            return nullptr;
        }
        for (size_t index = 0; index < count; ++index) {
            if (coord_list_push_back_ex(result, &coordinates[index])
                != NAVSYS_STATUS_OK) {
                coord_list_destroy(result);
                return nullptr;
            }
        }
        return result;
    } catch (...) {
        return nullptr;
    }
}

navsys_status_t copy_blocked_hash(
    const obstacle_t* obstacle, coord_hash_t** out_copy) {
    if (!obstacle || !out_copy) return NAVSYS_STATUS_INVALID_ARGUMENT;
    if (!obstacle->blocked) return NAVSYS_STATUS_CORRUPT_STATE;
    const navsys_status_t status = coord_hash_copy_ex(
        obstacle->blocked, out_copy);
    if (status == NAVSYS_STATUS_OK || status == NAVSYS_STATUS_OUT_OF_MEMORY)
        return status;
    return NAVSYS_STATUS_CORRUPT_STATE;
}

navsys_status_t block_square_in_hash(
    coord_hash_t* blocked,
    const obstacle_extent_t& extent,
    int64_t center_x,
    int64_t center_y,
    int32_t radius,
    size_t& changed) {
    const int64_t minimum_x = std::max(
        extent.min_x, center_x - static_cast<int64_t>(radius));
    const int64_t maximum_x = std::min(
        extent.max_x - 1, center_x + static_cast<int64_t>(radius));
    const int64_t minimum_y = std::max(
        extent.min_y, center_y - static_cast<int64_t>(radius));
    const int64_t maximum_y = std::min(
        extent.max_y - 1, center_y + static_cast<int64_t>(radius));
    if (minimum_x > maximum_x || minimum_y > maximum_y)
        return NAVSYS_STATUS_OK;

    for (int64_t x = minimum_x; x <= maximum_x; ++x) {
        for (int64_t y = minimum_y; y <= maximum_y; ++y) {
            const coord_t coordinate{
                static_cast<int>(x), static_cast<int>(y)};
            bool inserted = false;
            const navsys_status_t status = coord_hash_upsert_copy(
                blocked, &coordinate, nullptr, &inserted);
            if (status != NAVSYS_STATUS_OK) return status;
            if (inserted) {
                if (changed == std::numeric_limits<size_t>::max())
                    return NAVSYS_STATUS_LIMIT_REACHED;
                ++changed;
            }
        }
    }
    return NAVSYS_STATUS_OK;
}

navsys_status_t commit_blocked_hash(
    obstacle_t* obstacle, coord_hash_t* prepared,
    size_t changed, size_t* out_changed_count) {
    coord_hash_t* previous = obstacle->blocked;
    obstacle->blocked = prepared;
    *out_changed_count = changed;
    coord_hash_destroy(previous);
    return NAVSYS_STATUS_OK;
}

} // namespace

navsys_status_t obstacle_export_neighbors(
    const obstacle_t* obstacle,
    int32_t x,
    int32_t y,
    bool traversable_only,
    coord_t* out_coords,
    size_t capacity,
    size_t* out_count) {
    try {
        return export_immediate_neighbors(
            obstacle, x, y, traversable_only,
            out_coords, capacity, out_count);
    } catch (const std::bad_alloc&) {
        return NAVSYS_STATUS_OUT_OF_MEMORY;
    } catch (...) {
        return NAVSYS_STATUS_CORRUPT_STATE;
    }
}

navsys_status_t obstacle_export_neighbors_range(
    const obstacle_t* obstacle,
    int32_t x,
    int32_t y,
    int32_t range,
    coord_t* out_coords,
    size_t capacity,
    size_t* out_count) {
    if (!obstacle || range < 0
        || !valid_export_buffer(out_coords, capacity, out_count)) {
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    }
    if (!obstacle->blocked) return NAVSYS_STATUS_CORRUPT_STATE;
    const obstacle_extent_t extent = obstacle_extent(obstacle);
    if (!extent_contains(extent, x, y)) return NAVSYS_STATUS_NOT_FOUND;

    const int64_t radius = static_cast<int64_t>(range) + 1;
    const int64_t minimum_x = std::max(extent.min_x, static_cast<int64_t>(x) - radius);
    const int64_t maximum_x = std::min(extent.max_x - 1, static_cast<int64_t>(x) + radius);
    const int64_t minimum_y = std::max(extent.min_y, static_cast<int64_t>(y) - radius);
    const int64_t maximum_y = std::min(extent.max_y - 1, static_cast<int64_t>(y) + radius);
    const uint64_t width = static_cast<uint64_t>(maximum_x - minimum_x + 1);
    const uint64_t height = static_cast<uint64_t>(maximum_y - minimum_y + 1);
    uint64_t required64 = width * height;
    if (range == 0) --required64;
    if (required64 > std::numeric_limits<size_t>::max())
        return NAVSYS_STATUS_LIMIT_REACHED;
    const size_t required = static_cast<size_t>(required64);

    if (!out_coords) {
        *out_count = required;
        return NAVSYS_STATUS_OK;
    }
    if (capacity < required) {
        *out_count = required;
        return NAVSYS_STATUS_INCOMPLETE;
    }
    size_t index = 0;
    for (int64_t nx = minimum_x; nx <= maximum_x; ++nx) {
        for (int64_t ny = minimum_y; ny <= maximum_y; ++ny) {
            if (range == 0 && nx == x && ny == y) continue;
            out_coords[index++] = {
                static_cast<int>(nx), static_cast<int>(ny)};
        }
    }
    *out_count = required;
    return NAVSYS_STATUS_OK;
}

navsys_status_t obstacle_fetch_neighbor_at_degree(
    const obstacle_t* obstacle,
    int32_t x,
    int32_t y,
    double degree,
    coord_t* out_coord) {
    if (!out_coord || !std::isfinite(degree))
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    coord_t neighbors[8]{};
    size_t count = 0;
    const navsys_status_t status = collect_immediate_neighbors(
        obstacle, x, y, false, neighbors, count);
    if (status != NAVSYS_STATUS_OK) return status;
    if (count == 0) return NAVSYS_STATUS_NOT_FOUND;

    const double target = normalized_degree(degree);
    size_t best = 0;
    double best_difference = 361.0;
    double best_angle = 361.0;
    for (size_t index = 0; index < count; ++index) {
        const double angle = coordinate_degree(
            static_cast<int64_t>(neighbors[index].x) - x,
            static_cast<int64_t>(neighbors[index].y) - y);
        double difference = std::fabs(target - angle);
        if (difference > 180.0) difference = 360.0 - difference;
        if (difference < best_difference
            || (difference == best_difference && angle < best_angle)) {
            best = index;
            best_difference = difference;
            best_angle = angle;
        }
    }
    *out_coord = neighbors[best];
    return NAVSYS_STATUS_OK;
}

navsys_status_t obstacle_fetch_neighbor_at_goal(
    const obstacle_t* obstacle,
    const coord_t* center,
    const coord_t* goal,
    coord_t* out_coord) {
    if (!center || !goal || !out_coord
        || (center->x == goal->x && center->y == goal->y)) {
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    }
    const double degree = coordinate_degree(
        static_cast<int64_t>(goal->x) - center->x,
        static_cast<int64_t>(goal->y) - center->y);
    return obstacle_fetch_neighbor_at_degree(
        obstacle, center->x, center->y, degree, out_coord);
}

navsys_status_t obstacle_export_neighbors_at_degree_range(
    const obstacle_t* obstacle,
    const coord_t* center,
    const coord_t* goal,
    double start_deg,
    double end_deg,
    int32_t range,
    coord_t* out_coords,
    size_t capacity,
    size_t* out_count) {
    if (!obstacle || !center || !goal || range < 0
        || !std::isfinite(start_deg) || !std::isfinite(end_deg)
        || (center->x == goal->x && center->y == goal->y)
        || !valid_export_buffer(out_coords, capacity, out_count)) {
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    }
    if (!obstacle->blocked) return NAVSYS_STATUS_CORRUPT_STATE;
    const obstacle_extent_t extent = obstacle_extent(obstacle);
    if (!extent_contains(extent, center->x, center->y))
        return NAVSYS_STATUS_NOT_FOUND;

    const double center_degree = coordinate_degree(
        static_cast<int64_t>(goal->x) - center->x,
        static_cast<int64_t>(goal->y) - center->y);
    const double minimum = normalized_degree(center_degree + start_deg);
    const double maximum = normalized_degree(center_degree + end_deg);
    const bool wraps = minimum > maximum;
    auto included = [&](int64_t dx, int64_t dy) {
        if (dx == 0 && dy == 0) return false;
        const double degree = coordinate_degree(dx, dy);
        return wraps ? degree >= minimum || degree <= maximum
                     : degree >= minimum && degree <= maximum;
    };

    const int64_t minimum_x = std::max(
        extent.min_x, static_cast<int64_t>(center->x) - range);
    const int64_t maximum_x = std::min(
        extent.max_x - 1, static_cast<int64_t>(center->x) + range);
    const int64_t minimum_y = std::max(
        extent.min_y, static_cast<int64_t>(center->y) - range);
    const int64_t maximum_y = std::min(
        extent.max_y - 1, static_cast<int64_t>(center->y) + range);

    size_t required = 0;
    for (int64_t x = minimum_x; x <= maximum_x; ++x) {
        for (int64_t y = minimum_y; y <= maximum_y; ++y) {
            if (!included(x - center->x, y - center->y)) continue;
            if (required == std::numeric_limits<size_t>::max())
                return NAVSYS_STATUS_LIMIT_REACHED;
            ++required;
        }
    }
    if (!out_coords) {
        *out_count = required;
        return NAVSYS_STATUS_OK;
    }
    if (capacity < required) {
        *out_count = required;
        return NAVSYS_STATUS_INCOMPLETE;
    }
    size_t index = 0;
    for (int64_t x = minimum_x; x <= maximum_x; ++x) {
        for (int64_t y = minimum_y; y <= maximum_y; ++y) {
            if (!included(x - center->x, y - center->y)) continue;
            out_coords[index++] = {
                static_cast<int>(x), static_cast<int>(y)};
        }
    }
    *out_count = required;
    return NAVSYS_STATUS_OK;
}

coord_list_t* obstacle_create_neighbors(
    const obstacle_t* obstacle, int32_t x, int32_t y) {
    return create_exported_coords(
        [&](coord_t* output, size_t capacity, size_t* count) {
            return obstacle_export_neighbors(
                obstacle, x, y, true, output, capacity, count);
        });
}

coord_list_t* obstacle_create_neighbors_all(
    const obstacle_t* obstacle, int32_t x, int32_t y) {
    return create_exported_coords(
        [&](coord_t* output, size_t capacity, size_t* count) {
            return obstacle_export_neighbors(
                obstacle, x, y, false, output, capacity, count);
        });
}

coord_list_t* obstacle_create_neighbors_all_range(
    const obstacle_t* obstacle, int32_t x, int32_t y, int32_t range) {
    return create_exported_coords(
        [&](coord_t* output, size_t capacity, size_t* count) {
            return obstacle_export_neighbors_range(
                obstacle, x, y, range, output, capacity, count);
        });
}

coord_t* obstacle_create_neighbor_at_degree(
    const obstacle_t* obstacle, int32_t x, int32_t y, double degree) {
    coord_t result{};
    if (obstacle_fetch_neighbor_at_degree(
            obstacle, x, y, degree, &result) != NAVSYS_STATUS_OK) {
        return nullptr;
    }
    try {
        return coord_create_full(result.x, result.y);
    } catch (...) {
        return nullptr;
    }
}

coord_t* obstacle_create_neighbor_at_goal(
    const obstacle_t* obstacle, const coord_t* center, const coord_t* goal) {
    coord_t result{};
    if (obstacle_fetch_neighbor_at_goal(
            obstacle, center, goal, &result) != NAVSYS_STATUS_OK) {
        return nullptr;
    }
    try {
        return coord_create_full(result.x, result.y);
    } catch (...) {
        return nullptr;
    }
}

coord_list_t* obstacle_create_neighbors_at_degree_range(
    const obstacle_t* obstacle,
    const coord_t* center,
    const coord_t* goal,
    double start_deg,
    double end_deg,
    int32_t range) {
    return create_exported_coords(
        [&](coord_t* output, size_t capacity, size_t* count) {
            return obstacle_export_neighbors_at_degree_range(
                obstacle, center, goal, start_deg, end_deg, range,
                output, capacity, count);
        });
}

navsys_status_t obstacle_block_square(
    obstacle_t* obstacle,
    int32_t x,
    int32_t y,
    int32_t radius,
    size_t* out_changed_count) {
    if (!obstacle || !out_changed_count || radius < 0)
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    if (!obstacle->blocked) return NAVSYS_STATUS_CORRUPT_STATE;
    const obstacle_extent_t extent = obstacle_extent(obstacle);
    if (!extent_contains(extent, x, y)) return NAVSYS_STATUS_NOT_FOUND;

    coord_hash_t* prepared = nullptr;
    try {
        navsys_status_t status = copy_blocked_hash(obstacle, &prepared);
        if (status != NAVSYS_STATUS_OK) return status;
        size_t changed = 0;
        status = block_square_in_hash(
            prepared, extent, x, y, radius, changed);
        if (status != NAVSYS_STATUS_OK) {
            coord_hash_destroy(prepared);
            return status;
        }
        return commit_blocked_hash(
            obstacle, prepared, changed, out_changed_count);
    } catch (const std::bad_alloc&) {
        coord_hash_destroy(prepared);
        return NAVSYS_STATUS_OUT_OF_MEMORY;
    } catch (...) {
        coord_hash_destroy(prepared);
        return NAVSYS_STATUS_CORRUPT_STATE;
    }
}

navsys_status_t obstacle_block_line(
    obstacle_t* obstacle,
    int32_t x0,
    int32_t y0,
    int32_t x1,
    int32_t y1,
    int32_t radius,
    size_t* out_changed_count) {
    if (!obstacle || !out_changed_count || radius < 0)
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    if (!obstacle->blocked) return NAVSYS_STATUS_CORRUPT_STATE;
    const obstacle_extent_t extent = obstacle_extent(obstacle);
    if (!extent_contains(extent, x0, y0)
        || !extent_contains(extent, x1, y1)) {
        return NAVSYS_STATUS_NOT_FOUND;
    }

    coord_hash_t* prepared = nullptr;
    try {
        navsys_status_t status = copy_blocked_hash(obstacle, &prepared);
        if (status != NAVSYS_STATUS_OK) return status;
        size_t changed = 0;
        int64_t current_x = x0;
        int64_t current_y = y0;
        const int64_t target_x = x1;
        const int64_t target_y = y1;
        const int64_t dx = std::llabs(target_x - current_x);
        const int64_t sx = current_x < target_x ? 1 : -1;
        const int64_t dy = -std::llabs(target_y - current_y);
        const int64_t sy = current_y < target_y ? 1 : -1;
        int64_t error = dx + dy;

        while (true) {
            status = block_square_in_hash(
                prepared, extent, current_x, current_y, radius, changed);
            if (status != NAVSYS_STATUS_OK) {
                coord_hash_destroy(prepared);
                return status;
            }
            if (current_x == target_x && current_y == target_y) break;
            const int64_t doubled_error = error * 2;
            if (doubled_error >= dy) {
                error += dy;
                current_x += sx;
            }
            if (doubled_error <= dx) {
                error += dx;
                current_y += sy;
            }
        }
        return commit_blocked_hash(
            obstacle, prepared, changed, out_changed_count);
    } catch (const std::bad_alloc&) {
        coord_hash_destroy(prepared);
        return NAVSYS_STATUS_OUT_OF_MEMORY;
    } catch (...) {
        coord_hash_destroy(prepared);
        return NAVSYS_STATUS_CORRUPT_STATE;
    }
}

coord_list_t* obstacle_clone_neighbors(
    const obstacle_t* obstacle, int x, int y) {
    return obstacle_create_neighbors(obstacle, x, y);
}

coord_list_t* obstacle_clone_neighbors_all(
    const obstacle_t* obstacle, int x, int y) {
    return obstacle_create_neighbors_all(obstacle, x, y);
}

coord_list_t* obstacle_clone_neighbors_all_range(
    obstacle_t* obstacle, int x, int y, int range) {
    return obstacle_create_neighbors_all_range(obstacle, x, y, range);
}

coord_t* obstacle_clone_neighbor_at_degree(
    const obstacle_t* obstacle, int x, int y, double degree) {
    return obstacle_create_neighbor_at_degree(obstacle, x, y, degree);
}

coord_t* obstacle_clone_neighbor_at_goal(
    const obstacle_t* obstacle, const coord_t* center, const coord_t* goal) {
    return obstacle_create_neighbor_at_goal(obstacle, center, goal);
}

coord_list_t* obstacle_clone_neighbors_at_degree_range(
    const obstacle_t* obstacle,
    const coord_t* center,
    const coord_t* goal,
    double start_deg,
    double end_deg,
    int range) {
    return obstacle_create_neighbors_at_degree_range(
        obstacle, center, goal, start_deg, end_deg, range);
}

void obstacle_block_range(obstacle_t* obstacle, int x, int y, int range) {
    size_t changed = 0;
    (void)obstacle_block_square(obstacle, x, y, range, &changed);
}

void obstacle_block_straight(
    obstacle_t* obstacle,
    int x0,
    int y0,
    int x1,
    int y1,
    int range) {
    size_t changed = 0;
    (void)obstacle_block_line(
        obstacle, x0, y0, x1, y1, range, &changed);
}
