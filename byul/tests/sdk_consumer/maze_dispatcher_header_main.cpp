#include <cassert>
#include <initializer_list>
#include <limits>
#include <type_traits>

#include "maze.h"

static_assert(MAZE_TYPE_RECURSIVE == 0);
static_assert(MAZE_TYPE_PRIM == 1);
static_assert(MAZE_TYPE_BINARY == 2);
static_assert(MAZE_TYPE_ELLER == 3);
static_assert(MAZE_TYPE_ALDOUS_BRODER == 4);
static_assert(MAZE_TYPE_WILSON == 5);
static_assert(MAZE_TYPE_HUNT_AND_KILL == 6);
static_assert(MAZE_TYPE_SIDEWINDER == 7);
static_assert(MAZE_TYPE_RECURSIVE_DIVISION == 8);
static_assert(MAZE_TYPE_KRUSKAL == 9);
static_assert(MAZE_TYPE_ROOM_BLEND == 10);
static_assert(sizeof(maze_type_t) == 4);
static_assert(std::is_same_v<
    decltype(&maze_make),
    maze_t* (*)(int, int, int, int, maze_type_t)>);
static_assert(std::is_same_v<
    decltype(&byul_maze_algorithm_is_supported),
    navsys_status_t (*)(byul_maze_algorithm_t, bool*)>);

int main() {
    byul_maze_generate_options_t options{
        sizeof(byul_maze_generate_options_t),
        BYUL_MAZE_GENERATE_OPTIONS_ABI_VERSION,
        UINT64_C(123),
        0,
        0,
        nullptr,
        nullptr
    };
    for (int value = BYUL_MAZE_ALGORITHM_RECURSIVE_BACKTRACKER;
         value <= BYUL_MAZE_ALGORITHM_ROOM_BLEND; ++value) {
        const auto algorithm = static_cast<byul_maze_algorithm_t>(value);
        bool supported = false;
        assert(byul_maze_algorithm_is_supported(algorithm, &supported)
            == NAVSYS_STATUS_OK);
        assert(supported);
        maze_t* checked = nullptr;
        assert(byul_maze_generate(
            algorithm, 0, 0, 9, 9, &options, &checked) == NAVSYS_STATUS_OK);
        assert(checked != nullptr);
        maze_destroy(checked);
    }

    maze_t* expected = maze_make(7, -3, 3, 3, MAZE_TYPE_KRUSKAL);
    assert(expected != nullptr);

    for (const int invalid : {-1, 11, std::numeric_limits<int>::max()}) {
        maze_t* actual = maze_make(
            7, -3, 3, 3, static_cast<maze_type_t>(invalid));
        assert(actual != nullptr);
        assert(maze_equal(actual, expected));
        assert(maze_hash(actual) == maze_hash(expected));
        maze_destroy(actual);
    }
    maze_destroy(expected);
    return 0;
}
