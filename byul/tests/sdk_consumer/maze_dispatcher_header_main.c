#include <assert.h>
#include <limits.h>

#include "maze.h"

_Static_assert(MAZE_TYPE_RECURSIVE == 0, "recursive enum ABI");
_Static_assert(MAZE_TYPE_PRIM == 1, "Prim enum ABI");
_Static_assert(MAZE_TYPE_BINARY == 2, "binary enum ABI");
_Static_assert(MAZE_TYPE_ELLER == 3, "Eller enum ABI");
_Static_assert(MAZE_TYPE_ALDOUS_BRODER == 4, "Aldous-Broder enum ABI");
_Static_assert(MAZE_TYPE_WILSON == 5, "Wilson enum ABI");
_Static_assert(MAZE_TYPE_HUNT_AND_KILL == 6, "hunt-and-kill enum ABI");
_Static_assert(MAZE_TYPE_SIDEWINDER == 7, "Sidewinder enum ABI");
_Static_assert(MAZE_TYPE_RECURSIVE_DIVISION == 8, "division enum ABI");
_Static_assert(MAZE_TYPE_KRUSKAL == 9, "Kruskal enum ABI");
_Static_assert(MAZE_TYPE_ROOM_BLEND == 10, "room-blend enum ABI");
_Static_assert(sizeof(maze_type_t) == 4, "maze_type_t ABI size");

typedef maze_t* (*maze_make_signature_t)(
    int, int, int, int, maze_type_t);
typedef navsys_status_t (*maze_support_signature_t)(
    byul_maze_algorithm_t, bool*);

int main(void) {
    maze_make_signature_t make = maze_make;
    maze_support_signature_t support_query = byul_maze_algorithm_is_supported;
    byul_maze_generate_options_t options = {
        sizeof(byul_maze_generate_options_t),
        BYUL_MAZE_GENERATE_OPTIONS_ABI_VERSION,
        UINT64_C(123),
        0,
        0,
        NULL,
        NULL
    };
    for (int value = BYUL_MAZE_ALGORITHM_RECURSIVE_BACKTRACKER;
         value <= BYUL_MAZE_ALGORITHM_ROOM_BLEND; ++value) {
        bool supported = false;
        assert(support_query((byul_maze_algorithm_t)value, &supported)
            == NAVSYS_STATUS_OK);
        assert(supported);
        maze_t* checked = NULL;
        assert(byul_maze_generate(
            (byul_maze_algorithm_t)value,
            0, 0, 9, 9, &options, &checked) == NAVSYS_STATUS_OK);
        assert(checked != NULL);
        maze_destroy(checked);
    }

    maze_t* expected = make(7, -3, 3, 3, MAZE_TYPE_KRUSKAL);
    assert(expected != NULL);

    const int invalid_values[] = {-1, 11, INT_MAX};
    for (size_t i = 0; i < sizeof(invalid_values) / sizeof(invalid_values[0]);
         ++i) {
        maze_t* actual = make(
            7, -3, 3, 3, (maze_type_t)invalid_values[i]);
        assert(actual != NULL);
        assert(maze_equal(actual, expected));
        assert(maze_hash(actual) == maze_hash(expected));
        maze_destroy(actual);
    }
    maze_destroy(expected);
    return 0;
}
