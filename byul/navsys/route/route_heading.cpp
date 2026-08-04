#include "route.h"

#include <cmath>
#include <limits>

struct s_route_heading_tracker {
    double mean_x;
    double mean_y;
    size_t sample_count;
};

namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kRadiansToDegrees = 180.0 / kPi;

int sign_of(long long value) {
    return (value > 0) - (value < 0);
}

navsys_status_t direction_from_delta(
    long long delta_x,
    long long delta_y,
    route_dir_t* out_direction) {
    if (!out_direction)
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    const int normalized_x = sign_of(delta_x);
    const int normalized_y = sign_of(delta_y);
    if (normalized_x == 0 && normalized_y == 0)
        return NAVSYS_STATUS_NOT_FOUND;

    static constexpr route_dir_t kDirections[3][3] = {
        {ROUTE_DIR_UP_LEFT, ROUTE_DIR_LEFT, ROUTE_DIR_DOWN_LEFT},
        {ROUTE_DIR_UP, ROUTE_DIR_UNKNOWN, ROUTE_DIR_DOWN},
        {ROUTE_DIR_UP_RIGHT, ROUTE_DIR_RIGHT, ROUTE_DIR_DOWN_RIGHT},
    };
    *out_direction = kDirections[normalized_x + 1][normalized_y + 1];
    return NAVSYS_STATUS_OK;
}

double normalize_heading(double degrees) {
    while (degrees >= 180.0)
        degrees -= 360.0;
    while (degrees < -180.0)
        degrees += 360.0;
    return degrees;
}

double heading_degrees(double x, double y) {
    return normalize_heading(std::atan2(y, x) * kRadiansToDegrees);
}

navsys_status_t unit_vector(
    double x,
    double y,
    double* out_x,
    double* out_y) {
    const double length = std::hypot(x, y);
    if (!std::isfinite(length) || length == 0.0)
        return NAVSYS_STATUS_NOT_FOUND;
    *out_x = x / length;
    *out_y = y / length;
    return NAVSYS_STATUS_OK;
}

}  // namespace

navsys_status_t route_direction_between(
    const coord_t* from,
    const coord_t* to,
    route_dir_t* out_direction) {
    if (!from || !to || !out_direction)
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    return direction_from_delta(
        static_cast<long long>(coord_get_x(to)) - coord_get_x(from),
        static_cast<long long>(coord_get_y(to)) - coord_get_y(from),
        out_direction);
}

navsys_status_t route_direction_from_vector(
    const coord_t* vector,
    route_dir_t* out_direction) {
    if (!vector || !out_direction)
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    return direction_from_delta(
        static_cast<long long>(coord_get_x(vector)),
        static_cast<long long>(coord_get_y(vector)),
        out_direction);
}

navsys_status_t route_direction_fetch_vector(
    route_dir_t direction,
    coord_t* out_vector) {
    if (!out_vector || direction <= ROUTE_DIR_UNKNOWN ||
        direction >= ROUTE_DIR_COUNT) {
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    }
    static constexpr int kVectors[9][2] = {
        {0, 0}, {1, 0}, {1, -1}, {0, -1}, {-1, -1},
        {-1, 0}, {-1, 1}, {0, 1}, {1, 1},
    };
    const coord_t result = {
        kVectors[direction][0], kVectors[direction][1]};
    *out_vector = result;
    return NAVSYS_STATUS_OK;
}

navsys_status_t route_fetch_direction_at(
    const route_t* route,
    size_t index,
    route_dir_t* out_direction) {
    if (!route || !out_direction)
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    const size_t count = route_get_coord_count(route);
    if (count < 2 || index >= count)
        return NAVSYS_STATUS_NOT_FOUND;
    coord_t from{};
    coord_t to{};
    const size_t from_index = index + 1 < count ? index : index - 1;
    const size_t to_index = index + 1 < count ? index + 1 : index;
    navsys_status_t status = route_fetch_coord(route, from_index, &from);
    if (status == NAVSYS_STATUS_OK)
        status = route_fetch_coord(route, to_index, &to);
    if (status != NAVSYS_STATUS_OK)
        return status;
    return route_direction_between(&from, &to, out_direction);
}

navsys_status_t route_compute_recent_facing(
    const route_t* route,
    size_t history,
    route_dir_t* out_direction) {
    if (!route || !out_direction || history == 0)
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    const size_t count = route_get_coord_count(route);
    if (count < 2)
        return NAVSYS_STATUS_NOT_FOUND;
    const size_t from_index = history >= count ? 0 : count - history - 1;
    coord_t from{};
    coord_t to{};
    navsys_status_t status = route_fetch_coord(route, from_index, &from);
    if (status == NAVSYS_STATUS_OK)
        status = route_fetch_coord(route, count - 1, &to);
    if (status != NAVSYS_STATUS_OK)
        return status;
    return route_direction_between(&from, &to, out_direction);
}

navsys_status_t route_compute_recent_heading_degrees(
    const route_t* route,
    size_t history,
    double* out_heading_degrees) {
    if (!route || !out_heading_degrees || history == 0)
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    const size_t count = route_get_coord_count(route);
    if (count < 2)
        return NAVSYS_STATUS_NOT_FOUND;
    const size_t from_index = history >= count ? 0 : count - history - 1;
    coord_t from{};
    coord_t to{};
    navsys_status_t status = route_fetch_coord(route, from_index, &from);
    if (status == NAVSYS_STATUS_OK)
        status = route_fetch_coord(route, count - 1, &to);
    if (status != NAVSYS_STATUS_OK)
        return status;
    const double delta_x = static_cast<double>(coord_get_x(&to)) -
        static_cast<double>(coord_get_x(&from));
    const double delta_y = static_cast<double>(coord_get_y(&to)) -
        static_cast<double>(coord_get_y(&from));
    if (delta_x == 0.0 && delta_y == 0.0)
        return NAVSYS_STATUS_NOT_FOUND;
    *out_heading_degrees = heading_degrees(delta_x, delta_y);
    return NAVSYS_STATUS_OK;
}

navsys_status_t route_heading_tracker_create(
    route_heading_tracker_t** out_tracker) {
    if (!out_tracker)
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    route_heading_tracker_t* tracker = nullptr;
    try {
        tracker = new route_heading_tracker_t{};
    } catch (...) {
        return NAVSYS_STATUS_OUT_OF_MEMORY;
    }
    *out_tracker = tracker;
    return NAVSYS_STATUS_OK;
}

void route_heading_tracker_destroy(route_heading_tracker_t* tracker) {
    delete tracker;
}

navsys_status_t route_heading_tracker_reset(
    route_heading_tracker_t* tracker) {
    if (!tracker)
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    *tracker = route_heading_tracker_t{};
    return NAVSYS_STATUS_OK;
}

size_t route_heading_tracker_get_sample_count(
    const route_heading_tracker_t* tracker) {
    return tracker ? tracker->sample_count : 0;
}

navsys_status_t route_heading_tracker_fetch_heading_degrees(
    const route_heading_tracker_t* tracker,
    double* out_heading_degrees) {
    if (!tracker || !out_heading_degrees)
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    double x = 0.0;
    double y = 0.0;
    if (tracker->sample_count == 0 ||
        unit_vector(tracker->mean_x, tracker->mean_y, &x, &y) !=
            NAVSYS_STATUS_OK) {
        return NAVSYS_STATUS_NOT_FOUND;
    }
    *out_heading_degrees = heading_degrees(x, y);
    return NAVSYS_STATUS_OK;
}

navsys_status_t route_heading_tracker_observe_vector(
    route_heading_tracker_t* tracker,
    const coord_t* vector,
    double threshold_degrees,
    double* out_angle_degrees,
    bool* out_changed) {
    if (!tracker || !vector || !out_angle_degrees || !out_changed ||
        !std::isfinite(threshold_degrees) || threshold_degrees < 0.0 ||
        threshold_degrees > 180.0) {
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    }
    double sample_x = 0.0;
    double sample_y = 0.0;
    navsys_status_t status = unit_vector(
        static_cast<double>(coord_get_x(vector)),
        static_cast<double>(coord_get_y(vector)),
        &sample_x,
        &sample_y);
    if (status != NAVSYS_STATUS_OK)
        return status;

    double angle = 0.0;
    bool changed = false;
    if (tracker->sample_count != 0) {
        double mean_x = 0.0;
        double mean_y = 0.0;
        if (unit_vector(
                tracker->mean_x, tracker->mean_y, &mean_x, &mean_y) ==
            NAVSYS_STATUS_OK) {
            const double dot = std::fmax(
                -1.0, std::fmin(1.0, sample_x * mean_x + sample_y * mean_y));
            angle = std::acos(dot) * kRadiansToDegrees;
            changed = angle >= threshold_degrees;
        }
    }

    if (tracker->sample_count == 0) {
        tracker->mean_x = sample_x;
        tracker->mean_y = sample_y;
        tracker->sample_count = 1;
    } else {
        const size_t next_count = tracker->sample_count ==
            std::numeric_limits<size_t>::max()
            ? tracker->sample_count
            : tracker->sample_count + 1;
        const double weight = 1.0 / static_cast<double>(next_count);
        tracker->mean_x += (sample_x - tracker->mean_x) * weight;
        tracker->mean_y += (sample_y - tracker->mean_y) * weight;
        tracker->sample_count = next_count;
    }
    *out_angle_degrees = angle;
    *out_changed = changed;
    return NAVSYS_STATUS_OK;
}

navsys_status_t route_heading_tracker_observe(
    route_heading_tracker_t* tracker,
    const coord_t* from,
    const coord_t* to,
    double threshold_degrees,
    double* out_angle_degrees,
    bool* out_changed) {
    if (!from || !to)
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    const double delta_x = static_cast<double>(coord_get_x(to)) -
        static_cast<double>(coord_get_x(from));
    const double delta_y = static_cast<double>(coord_get_y(to)) -
        static_cast<double>(coord_get_y(from));
    if (!tracker || !out_angle_degrees || !out_changed ||
        !std::isfinite(threshold_degrees) || threshold_degrees < 0.0 ||
        threshold_degrees > 180.0) {
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    }
    double sample_x = 0.0;
    double sample_y = 0.0;
    const navsys_status_t status = unit_vector(
        delta_x, delta_y, &sample_x, &sample_y);
    if (status != NAVSYS_STATUS_OK)
        return status;
    const coord_t normalized_vector = {
        static_cast<int>(std::round(sample_x * 1000000000.0)),
        static_cast<int>(std::round(sample_y * 1000000000.0))};
    return route_heading_tracker_observe_vector(
        tracker, &normalized_vector, threshold_degrees,
        out_angle_degrees, out_changed);
}
