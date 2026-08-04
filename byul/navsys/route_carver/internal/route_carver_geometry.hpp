/*
 * Copyright (c) 2025-2026 ByulPapa (byuldev@outlook.kr)
 * This file is part of the Byul World project.
 * Licensed under the Byul World Source-Available Non-Commercial License v1.0 (2025).
 * See the LICENSE file in the project root for full license terms.
 */

#ifndef BYUL_ROUTE_CARVER_GEOMETRY_INTERNAL_HPP
#define BYUL_ROUTE_CARVER_GEOMETRY_INTERNAL_HPP

#include "coord.h"
#include "navsys_status.h"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace byul::navsys::route_carver::internal {

using carve_cancel_func = bool (*)(void* userdata);

inline constexpr size_t carve_poll_interval_cells = 64;

enum class carve_metric : uint32_t {
    chebyshev_square = 0,
    manhattan_diamond = 1,
    euclidean_disk = 2
};

enum class line_coverage : uint32_t {
    center_cells = 0,
    supercover_cells = 1
};

struct carve_extent {
    bool bounded_x;
    bool bounded_y;
    int64_t min_x;
    int64_t max_x;
    int64_t min_y;
    int64_t max_y;
};

struct carve_candidate_options {
    uint32_t radius_cells;
    carve_metric metric;
    line_coverage coverage;
    bool include_start;
    bool include_end;
    bool clip_to_extent;
    size_t max_cells;
    carve_cancel_func cancel_func;
    void* cancel_userdata;
};

inline constexpr size_t maximum_candidate_cells = 262144;
inline constexpr uint64_t maximum_raster_steps = 1048576;

carve_extent unbounded_carve_extent();
carve_extent positive_carve_extent(int width, int height);

navsys_status_t enumerate_line_candidates(
    const coord_t& start,
    const coord_t& end,
    const carve_extent& extent,
    const carve_candidate_options& options,
    std::vector<coord_t>& output);

navsys_status_t enumerate_area_candidates(
    const coord_t& center,
    const carve_extent& extent,
    const carve_candidate_options& options,
    std::vector<coord_t>& output);

} // namespace byul::navsys::route_carver::internal

#endif /* BYUL_ROUTE_CARVER_GEOMETRY_INTERNAL_HPP */
