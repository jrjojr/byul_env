#include "route_finder_core.h"
#include "route_finder_evaluation.h"
#include "cost_coord_pq.h"
#include "coord_list.h"
#include "coord.h"

#include <cmath>
#include <vector>
#include <limits>

namespace {

navsys_status_t validate_cost_arguments(
    const navgrid_t* navgrid,
    const coord_t* from,
    const coord_t* to,
    float* out_cost) {
    return navgrid && from && to && out_cost
        ? NAVSYS_STATUS_OK
        : NAVSYS_STATUS_INVALID_ARGUMENT;
}

navsys_status_t validate_heuristic_arguments(
    const coord_t* from,
    const coord_t* goal,
    float* out_estimate) {
    return from && goal && out_estimate
        ? NAVSYS_STATUS_OK
        : NAVSYS_STATUS_INVALID_ARGUMENT;
}

float absolute_delta(int lhs, int rhs) {
    return static_cast<float>(std::fabs(
        static_cast<double>(lhs) - static_cast<double>(rhs)));
}

} // namespace

navsys_status_t route_finder_cost_unit(
    const navgrid_t* navgrid,
    const coord_t* from,
    const coord_t* to,
    float* out_cost,
    void*) {
    navsys_status_t status = validate_cost_arguments(
        navgrid, from, to, out_cost);
    if (status != NAVSYS_STATUS_OK) return status;
    *out_cost = 1.0f;
    return NAVSYS_STATUS_OK;
}

navsys_status_t route_finder_cost_zero(
    const navgrid_t* navgrid,
    const coord_t* from,
    const coord_t* to,
    float* out_cost,
    void*) {
    navsys_status_t status = validate_cost_arguments(
        navgrid, from, to, out_cost);
    if (status != NAVSYS_STATUS_OK) return status;
    *out_cost = 0.0f;
    return NAVSYS_STATUS_OK;
}

navsys_status_t route_finder_cost_diagonal(
    const navgrid_t* navgrid,
    const coord_t* from,
    const coord_t* to,
    float* out_cost,
    void*) {
    navsys_status_t status = validate_cost_arguments(
        navgrid, from, to, out_cost);
    if (status != NAVSYS_STATUS_OK) return status;

    const float dx = absolute_delta(from->x, to->x);
    const float dy = absolute_delta(from->y, to->y);
    if (dx > 1.0f || dy > 1.0f || (dx == 0.0f && dy == 0.0f))
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    *out_cost = dx != 0.0f && dy != 0.0f
        ? 1.4142135623730951f
        : 1.0f;
    return NAVSYS_STATUS_OK;
}

navsys_status_t route_finder_heuristic_euclidean(
    const coord_t* from,
    const coord_t* goal,
    float* out_estimate,
    void*) {
    navsys_status_t status = validate_heuristic_arguments(
        from, goal, out_estimate);
    if (status != NAVSYS_STATUS_OK) return status;
    *out_estimate = std::hypot(
        absolute_delta(from->x, goal->x),
        absolute_delta(from->y, goal->y));
    return std::isfinite(*out_estimate)
        ? NAVSYS_STATUS_OK
        : NAVSYS_STATUS_INVALID_ARGUMENT;
}

navsys_status_t route_finder_heuristic_manhattan(
    const coord_t* from,
    const coord_t* goal,
    float* out_estimate,
    void*) {
    navsys_status_t status = validate_heuristic_arguments(
        from, goal, out_estimate);
    if (status != NAVSYS_STATUS_OK) return status;
    *out_estimate = absolute_delta(from->x, goal->x)
        + absolute_delta(from->y, goal->y);
    return std::isfinite(*out_estimate)
        ? NAVSYS_STATUS_OK
        : NAVSYS_STATUS_INVALID_ARGUMENT;
}

navsys_status_t route_finder_heuristic_chebyshev(
    const coord_t* from,
    const coord_t* goal,
    float* out_estimate,
    void*) {
    navsys_status_t status = validate_heuristic_arguments(
        from, goal, out_estimate);
    if (status != NAVSYS_STATUS_OK) return status;
    *out_estimate = std::max(
        absolute_delta(from->x, goal->x),
        absolute_delta(from->y, goal->y));
    return NAVSYS_STATUS_OK;
}

navsys_status_t route_finder_heuristic_octile(
    const coord_t* from,
    const coord_t* goal,
    float* out_estimate,
    void*) {
    navsys_status_t status = validate_heuristic_arguments(
        from, goal, out_estimate);
    if (status != NAVSYS_STATUS_OK) return status;
    const float dx = absolute_delta(from->x, goal->x);
    const float dy = absolute_delta(from->y, goal->y);
    const float minimum = std::min(dx, dy);
    const float maximum = std::max(dx, dy);
    *out_estimate = maximum + (1.4142135623730951f - 1.0f) * minimum;
    return std::isfinite(*out_estimate)
        ? NAVSYS_STATUS_OK
        : NAVSYS_STATUS_INVALID_ARGUMENT;
}

navsys_status_t route_finder_heuristic_zero(
    const coord_t* from,
    const coord_t* goal,
    float* out_estimate,
    void*) {
    navsys_status_t status = validate_heuristic_arguments(
        from, goal, out_estimate);
    if (status != NAVSYS_STATUS_OK) return status;
    *out_estimate = 0.0f;
    return NAVSYS_STATUS_OK;
}

navsys_status_t route_finder_heuristic_default(
    const coord_t* from,
    const coord_t* goal,
    float* out_estimate,
    void* userdata) {
    return route_finder_heuristic_zero(
        from, goal, out_estimate, userdata);
}


float default_cost(const navgrid_t* m, 
    const coord_t* start, const coord_t* goal, void* userdata) {
    return 1.0f;
}

float zero_cost(const navgrid_t* m, 
    const coord_t* start, const coord_t* goal, void* userdata) {
    return 0.0f;
}

float diagonal_cost(const navgrid_t* m, 
    const coord_t* start, const coord_t* goal, void* userdata) {
    if (!start || !goal) return std::numeric_limits<float>::max();
    int dx = std::abs(start->x - goal->x);
    int dy = std::abs(start->y - goal->y);
    return (dx != 0 && dy != 0) ? DIAGONAL_COST : 1.0f;
}

float euclidean_heuristic(
    const coord_t* start, const coord_t* goal, void* userdata) {

    if (!start || !goal) return std::numeric_limits<float>::max();
    int dx = start->x - goal->x;
    int dy = start->y - goal->y;
    return std::sqrt(static_cast<float>(dx * dx + dy * dy));
}

float manhattan_heuristic(
    const coord_t* start, const coord_t* goal, void* userdata) {

    if (!start || !goal) return std::numeric_limits<float>::max();
    return static_cast<float>(
        std::abs(start->x - goal->x) + std::abs(start->y - goal->y));
}

float chebyshev_heuristic(
    const coord_t* start, const coord_t* goal, void* userdata) {

    if (!start || !goal) return std::numeric_limits<float>::max();
    int dx = std::abs(start->x - goal->x);
    int dy = std::abs(start->y - goal->y);
    return static_cast<float>(std::max(dx, dy));
}

float octile_heuristic(
    const coord_t* start, const coord_t* goal, void* userdata) {

    if (!start || !goal) return std::numeric_limits<float>::max();
    int dx = std::abs(start->x - goal->x);
    int dy = std::abs(start->y - goal->y);
    float F = std::sqrt(2.0f) - 1.0f;
    return static_cast<float>(std::max(dx, dy) + F * std::min(dx, dy));
}

float zero_heuristic(
    const coord_t* start, const coord_t* goal, void* userdata) {

    return 0.0f;
}

float default_heuristic(
    const coord_t* start, const coord_t* goal, void* userdata) {
        
    return euclidean_heuristic(start, goal, userdata);
}

