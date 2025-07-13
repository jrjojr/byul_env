#include "internal/route_finder_common.h"
#include "internal/cost_coord_pq.h"
#include "internal/coord_list.h"
#include "coord.hpp"

#include <cmath>
#include <vector>
#include <limits>

// 비용 함수

float default_cost(const map_t* m, 
    const coord_t* start, const coord_t* goal, void* userdata) {
    return 1.0f;
}

float zero_cost(const map_t* m, 
    const coord_t* start, const coord_t* goal, void* userdata) {
    return 0.0f;
}

float diagonal_cost(const map_t* m, 
    const coord_t* start, const coord_t* goal, void* userdata) {
    if (!start || !goal) return std::numeric_limits<float>::max();
    int dx = std::abs(start->x - goal->x);
    int dy = std::abs(start->y - goal->y);
    return (dx != 0 && dy != 0) ? DIAGONAL_COST : 1.0f;
}

// 휴리스틱 함수

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
