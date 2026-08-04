from pathlib import Path

from byul_wrapper.maze import c_maze, maze_algorithm_is_supported


REPOSITORY_ROOT = Path(__file__).resolve().parents[4]
WRAPPER_SOURCE = REPOSITORY_ROOT / "tools/python/byul_wrapper/byul_wrapper/maze.py"
PUBLIC_HEADER = REPOSITORY_ROOT / "byul/navsys/maze/maze_core.h"
PRIVATE_HEADER = REPOSITORY_ROOT / "byul/navsys/maze/internal/maze_private.hpp"
ABI1_HEADER = REPOSITORY_ROOT / "byul/navsys/maze/compat/abi1/maze_abi1.h"


def test_maze_public_header_is_opaque_and_abi1_layout_is_opt_in():
    public = PUBLIC_HEADER.read_text(encoding="utf-8")
    private = PRIVATE_HEADER.read_text(encoding="utf-8")
    compat = ABI1_HEADER.read_text(encoding="utf-8")
    assert "typedef struct s_maze maze_t;" in public
    assert "struct s_maze {" not in public
    assert "struct s_maze {" in private
    assert "struct s_maze {" in compat
    assert "BYUL_MAZE_ABI_VERSION UINT32_C(2)" in public
    assert "BYUL_MAZE_ABI1_VERSION UINT32_C(1)" in compat


def test_maze_wrapper_uses_opaque_ready_accessors():
    source = WRAPPER_SOURCE.read_text(encoding="utf-8")
    assert "C.byul_maze_create" in source
    assert "C.byul_maze_set_blocked" in source
    assert "C.byul_maze_is_blocked" in source
    assert "C.byul_maze_check_abi" in source
    assert "C.byul_maze_generate" in source
    assert "C.byul_maze_algorithm_is_supported" in source
    assert "._c.x0" not in source
    assert "._c.blocked" not in source


def test_maze_wrapper_checked_lifecycle_and_enumeration_load():
    with c_maze(-2, 4, 5, 3) as maze:
        assert maze.extent == (-2, 4, 5, 3)
        assert maze.set_blocked(-1, 5)
        assert not maze.set_blocked(-1, 5)
        assert maze.is_blocked(-1, 5)
        assert {(coord.x, coord.y) for coord in maze.blocked_coords()} == {(-1, 5)}
        maze.translate(3, -2)
        assert maze.extent == (1, 2, 5, 3)
        assert maze.is_blocked(2, 3)


def test_maze_wrapper_loads_all_checked_dispatcher_algorithms():
    for algorithm in range(11):
        assert maze_algorithm_is_supported(algorithm)
        with c_maze.generate(
            algorithm, -3, 6, 9, 9, seed=20260804
        ) as maze:
            assert maze.extent == (-3, 6, 9, 9)
