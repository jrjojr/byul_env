#include "doctest.h"

#include "../route_carver/internal/route_carver_geometry.hpp"

#include <algorithm>
#include <climits>
#include <cstddef>
#include <set>
#include <utility>
#include <vector>

namespace {

namespace geometry = byul::navsys::route_carver::internal;

geometry::carve_candidate_options options(
    geometry::carve_metric metric = geometry::carve_metric::chebyshev_square,
    geometry::line_coverage coverage = geometry::line_coverage::center_cells) {
    return {0, metric, coverage, true, true, false, 4096};
}

std::set<std::pair<int, int>> as_set(const std::vector<coord_t>& coords) {
    std::set<std::pair<int, int>> result;
    for (const coord_t& coord : coords) result.emplace(coord.x, coord.y);
    return result;
}

std::set<std::pair<int, int>> rotated(
    const std::set<std::pair<int, int>>& coords) {
    std::set<std::pair<int, int>> result;
    for (const auto& coord : coords) result.emplace(-coord.second, coord.first);
    return result;
}

void check_row_major_unique(const std::vector<coord_t>& coords) {
    for (std::size_t index = 1; index < coords.size(); ++index) {
        const coord_t& previous = coords[index - 1];
        const coord_t& current = coords[index];
        CHECK((previous.y < current.y
            || (previous.y == current.y && previous.x < current.x)));
    }
}

} // namespace

TEST_CASE("route carver checked line geometry matches exact tie and corner goldens") {
    std::vector<coord_t> output;
    auto current = options();
    CHECK(geometry::enumerate_line_candidates(
        {0, 0}, {2, 1}, geometry::unbounded_carve_extent(), current, output)
        == NAVSYS_STATUS_OK);
    CHECK(as_set(output) == std::set<std::pair<int, int>>{
        {0, 0}, {1, 0}, {1, 1}, {2, 1}});
    check_row_major_unique(output);

    current.coverage = geometry::line_coverage::supercover_cells;
    CHECK(geometry::enumerate_line_candidates(
        {0, 0}, {1, 1}, geometry::unbounded_carve_extent(), current, output)
        == NAVSYS_STATUS_OK);
    CHECK(as_set(output) == std::set<std::pair<int, int>>{
        {0, 0}, {1, 0}, {0, 1}, {1, 1}});
    check_row_major_unique(output);
}

TEST_CASE("route carver checked lines are exhaustive reversal and rotation invariant") {
    for (const geometry::line_coverage coverage : {
            geometry::line_coverage::center_cells,
            geometry::line_coverage::supercover_cells}) {
        auto current = options(geometry::carve_metric::chebyshev_square, coverage);
        for (int start_y = -3; start_y <= 3; ++start_y) {
            for (int start_x = -3; start_x <= 3; ++start_x) {
                for (int end_y = -3; end_y <= 3; ++end_y) {
                    for (int end_x = -3; end_x <= 3; ++end_x) {
                        std::vector<coord_t> forward;
                        std::vector<coord_t> reverse;
                        std::vector<coord_t> turn;
                        const coord_t start{start_x, start_y};
                        const coord_t end{end_x, end_y};
                        REQUIRE(geometry::enumerate_line_candidates(
                            start, end, geometry::unbounded_carve_extent(),
                            current, forward) == NAVSYS_STATUS_OK);
                        REQUIRE(geometry::enumerate_line_candidates(
                            end, start, geometry::unbounded_carve_extent(),
                            current, reverse) == NAVSYS_STATUS_OK);
                        REQUIRE(geometry::enumerate_line_candidates(
                            {-start_y, start_x}, {-end_y, end_x},
                            geometry::unbounded_carve_extent(), current, turn)
                            == NAVSYS_STATUS_OK);
                        CHECK(as_set(forward) == as_set(reverse));
                        CHECK(rotated(as_set(forward)) == as_set(turn));
                        check_row_major_unique(forward);
                    }
                }
            }
        }
    }
}

TEST_CASE("route carver checked area metrics have exact radius counts") {
    struct metric_case {
        geometry::carve_metric metric;
        std::size_t radius_four_count;
    };
    for (const metric_case& fixture : {
            metric_case{geometry::carve_metric::chebyshev_square, 81},
            metric_case{geometry::carve_metric::manhattan_diamond, 41},
            metric_case{geometry::carve_metric::euclidean_disk, 49}}) {
        for (uint32_t radius = 0; radius <= 4; ++radius) {
            auto current = options(fixture.metric);
            current.radius_cells = radius;
            std::vector<coord_t> output;
            REQUIRE(geometry::enumerate_area_candidates(
                {0, 0}, geometry::unbounded_carve_extent(), current, output)
                == NAVSYS_STATUS_OK);
            CHECK(as_set(output).size() == output.size());
            CHECK(as_set(output).count({0, 0}) == 1);
            check_row_major_unique(output);
            if (radius == 4) CHECK(output.size() == fixture.radius_four_count);
        }
    }
}

TEST_CASE("route carver checked endpoint clip and reject options are exact") {
    auto current = options();
    current.radius_cells = 1;
    current.include_start = false;
    std::vector<coord_t> output;
    REQUIRE(geometry::enumerate_line_candidates(
        {0, 1}, {2, 1}, geometry::unbounded_carve_extent(), current, output)
        == NAVSYS_STATUS_OK);
    CHECK(as_set(output).count({0, 1}) == 0);
    CHECK(as_set(output).count({2, 1}) == 1);

    current = options();
    current.radius_cells = 1;
    current.clip_to_extent = true;
    REQUIRE(geometry::enumerate_area_candidates(
        {0, 0}, geometry::positive_carve_extent(3, 3), current, output)
        == NAVSYS_STATUS_OK);
    CHECK(as_set(output) == std::set<std::pair<int, int>>{
        {0, 0}, {1, 0}, {0, 1}, {1, 1}});

    current.clip_to_extent = false;
    const std::vector<coord_t> sentinel{{77, 88}};
    output = sentinel;
    CHECK(geometry::enumerate_area_candidates(
        {0, 0}, geometry::positive_carve_extent(3, 3), current, output)
        == NAVSYS_STATUS_NOT_FOUND);
    CHECK(as_set(output) == as_set(sentinel));
}

TEST_CASE("route carver checked limits overflow and unsupported preserve output") {
    const std::vector<coord_t> sentinel{{77, 88}};
    std::vector<coord_t> output = sentinel;
    auto current = options();
    current.max_cells = 3;
    CHECK(geometry::enumerate_line_candidates(
        {0, 0}, {10, 0}, geometry::unbounded_carve_extent(), current, output)
        == NAVSYS_STATUS_LIMIT_REACHED);
    CHECK(as_set(output) == as_set(sentinel));

    current = options();
    CHECK(geometry::enumerate_line_candidates(
        {INT_MIN, 0}, {INT_MAX, 0}, geometry::unbounded_carve_extent(),
        current, output) == NAVSYS_STATUS_LIMIT_REACHED);
    CHECK(as_set(output) == as_set(sentinel));

    current.radius_cells = UINT32_MAX;
    CHECK(geometry::enumerate_area_candidates(
        {0, 0}, geometry::unbounded_carve_extent(), current, output)
        == NAVSYS_STATUS_LIMIT_REACHED);
    CHECK(as_set(output) == as_set(sentinel));
    current.clip_to_extent = true;
    CHECK(geometry::enumerate_area_candidates(
        {0, 0}, geometry::unbounded_carve_extent(), current, output)
        == NAVSYS_STATUS_LIMIT_REACHED);
    CHECK(as_set(output) == as_set(sentinel));

    current = options();
    current.metric = static_cast<geometry::carve_metric>(99);
    CHECK(geometry::enumerate_area_candidates(
        {0, 0}, geometry::unbounded_carve_extent(), current, output)
        == NAVSYS_STATUS_UNSUPPORTED);
    CHECK(as_set(output) == as_set(sentinel));

    current = options();
    current.coverage = static_cast<geometry::line_coverage>(99);
    CHECK(geometry::enumerate_line_candidates(
        {0, 0}, {1, 1}, geometry::unbounded_carve_extent(), current, output)
        == NAVSYS_STATUS_UNSUPPORTED);
    CHECK(as_set(output) == as_set(sentinel));

    current = options();
    current.max_cells = geometry::maximum_candidate_cells + 1;
    CHECK(geometry::enumerate_area_candidates(
        {0, 0}, geometry::unbounded_carve_extent(), current, output)
        == NAVSYS_STATUS_LIMIT_REACHED);
    CHECK(as_set(output) == as_set(sentinel));
}

TEST_CASE("route carver checked point line has one explicit endpoint decision") {
    auto current = options();
    current.include_start = false;
    std::vector<coord_t> output;
    REQUIRE(geometry::enumerate_line_candidates(
        {3, -2}, {3, -2}, geometry::unbounded_carve_extent(), current, output)
        == NAVSYS_STATUS_OK);
    CHECK(as_set(output) == std::set<std::pair<int, int>>{{3, -2}});

    current.include_end = false;
    REQUIRE(geometry::enumerate_line_candidates(
        {3, -2}, {3, -2}, geometry::unbounded_carve_extent(), current, output)
        == NAVSYS_STATUS_OK);
    CHECK(output.empty());
}
