#include "coord.h"
#include "navgrid.h"
#include "route.h"
#include "route_finder.h"

#include <cstring>
#include <iostream>

namespace {

struct work_budget {
    route_finder_type_t type;
    const char* name;
    int max_retry_count;
};

constexpr work_budget budgets[] = {
    {ROUTE_FINDER_ASTAR, "astar", 55},
    {ROUTE_FINDER_BFS, "bfs", 100},
    {ROUTE_FINDER_DIJKSTRA, "dijkstra", 98},
    {ROUTE_FINDER_WEIGHTED_ASTAR, "weighted_astar", 60},
};

constexpr int fixture_width = 10;
constexpr int fixture_height = 10;

int fail(const char* message) {
    std::cerr << "route finder performance gate setup failed: "
              << message << '\n';
    return 2;
}

}  // namespace

int main(int argc, char** argv) {
    const bool inject_regression =
        argc == 2
        && std::strcmp(argv[1], "--inject-performance-regression") == 0;
    if (argc > 2 || (argc == 2 && !inject_regression))
        return fail("unsupported argument");

    navgrid_t* navgrid = navgrid_create_full(
        fixture_width,
        fixture_height,
        NAVGRID_DIR_8,
        is_coord_blocked_navgrid);
    if (!navgrid)
        return fail("navgrid allocation");

    for (int y = 1; y < fixture_height; ++y) {
        if (!navgrid_block_coord(navgrid, 5, y)) {
            navgrid_destroy(navgrid);
            return fail("obstacle construction");
        }
    }

    route_finder_t* finder = route_finder_create(navgrid);
    if (!finder) {
        navgrid_destroy(navgrid);
        return fail("route finder allocation");
    }

    const coord_t start = {0, 0};
    const coord_t goal = {9, 9};
    route_finder_set_start(finder, &start);
    route_finder_set_goal(finder, &goal);

    int result = 0;
    for (size_t index = 0; index < sizeof(budgets) / sizeof(budgets[0]);
         ++index) {
        const work_budget& budget = budgets[index];
        if (route_finder_set_type_checked(finder, budget.type)
            != NAVSYS_STATUS_OK) {
            result = fail("algorithm selection");
            break;
        }

        route_t* route = nullptr;
        route_finder_run_stats_t stats = {};
        const navsys_status_t status =
            route_finder_run_ex(finder, &route, &stats);
        if (status != NAVSYS_STATUS_OK || !route || !stats.complete) {
            route_destroy(route);
            result = fail("fixture route");
            break;
        }

        int observed_retry_count = stats.total_retry_count;
        if (inject_regression && index == 0)
            observed_retry_count = budget.max_retry_count + 1;

        std::cout << budget.name
                  << ": retries=" << observed_retry_count
                  << ", budget=" << budget.max_retry_count << '\n';
        route_destroy(route);

        if (observed_retry_count > budget.max_retry_count) {
            std::cerr << "route finder work budget exceeded: "
                      << budget.name
                      << " observed=" << observed_retry_count
                      << " budget=" << budget.max_retry_count << '\n';
            result = 4;
            break;
        }
    }

    route_finder_destroy(finder);
    navgrid_destroy(navgrid);
    return result;
}
