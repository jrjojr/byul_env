from pathlib import Path

from byul_wrapper.maze import (
    c_maze,
    maze_algorithm_is_supported,
    maze_binary_bias_is_supported,
    maze_sidewinder_sweep_is_supported,
)


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
    assert "C.byul_maze_generate_aldous_broder" in source
    assert "C.byul_maze_algorithm_is_supported" in source
    assert "C.byul_maze_generate_binary_tree" in source
    assert "C.byul_maze_binary_bias_is_supported" in source
    assert "C.byul_maze_generate_eller" in source
    assert "C.byul_maze_generate_hunt_and_kill" in source
    assert "C.byul_maze_generate_wilson" in source
    assert "C.byul_maze_generate_sidewinder" in source
    assert "C.byul_maze_generate_recursive_division" in source
    assert "C.byul_maze_generate_randomized_kruskal" in source
    assert "C.byul_maze_generate_randomized_prim" in source
    assert "C.byul_maze_generate_recursive_backtracker" in source
    assert "C.byul_maze_generate_room_blend" in source
    assert "C.byul_maze_sidewinder_sweep_is_supported" in source
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


def test_maze_wrapper_loads_all_binary_tree_biases():
    hashes = []
    for bias in range(4):
        assert maze_binary_bias_is_supported(bias)
        with c_maze.generate_binary_tree(
            -2, 7, 5, 5, bias, seed=1
        ) as maze:
            assert maze.extent == (-2, 7, 5, 5)
            hashes.append(maze.hash)
    assert hashes == [470646451, 499477593, 470646451, 499477593]


def test_maze_wrapper_loads_checked_eller_generator():
    with c_maze.generate_eller(-5, 8, 9, 9, seed=0) as maze:
        assert maze.extent == (-5, 8, 9, 9)
        assert maze.hash == 789167229


def test_maze_wrapper_loads_checked_hunt_and_kill_generator():
    with c_maze.generate_hunt_and_kill(-5, 8, 9, 9, seed=0) as maze:
        assert maze.extent == (-5, 8, 9, 9)
        assert maze.hash == 26398801


def test_maze_wrapper_loads_checked_wilson_generator():
    with c_maze.generate_wilson(-5, 8, 9, 9, seed=0) as maze:
        assert maze.extent == (-5, 8, 9, 9)
        assert maze.hash == 424385079


def test_maze_wrapper_loads_checked_aldous_broder_generator():
    with c_maze.generate_aldous_broder(-5, 8, 9, 9, seed=0) as maze:
        assert maze.extent == (-5, 8, 9, 9)
        assert maze.hash == 744322881


def test_maze_wrapper_loads_all_checked_sidewinder_sweeps():
    hashes = []
    for sweep in range(4):
        assert maze_sidewinder_sweep_is_supported(sweep)
        with c_maze.generate_sidewinder(
            -5, 8, 9, 9, sweep, seed=0
        ) as maze:
            assert maze.extent == (-5, 8, 9, 9)
            hashes.append(maze.hash)
    assert hashes == [915955447, 907534649, 464412339, 455990389]


def test_maze_wrapper_loads_checked_recursive_division_generator():
    with c_maze.generate_recursive_division(-5, 8, 9, 9, seed=0) as maze:
        assert maze.extent == (-5, 8, 9, 9)
        assert maze.hash == 669558005


def test_maze_wrapper_loads_checked_randomized_kruskal_generator():
    with c_maze.generate_randomized_kruskal(-5, 8, 9, 9, seed=0) as maze:
        assert maze.extent == (-5, 8, 9, 9)
        assert maze.hash == 73237245


def test_maze_wrapper_loads_checked_randomized_prim_generator():
    with c_maze.generate_randomized_prim(-5, 8, 9, 9, seed=0) as maze:
        assert maze.extent == (-5, 8, 9, 9)
        assert maze.hash == 857387639


def test_maze_wrapper_loads_checked_recursive_backtracker_generator():
    with c_maze.generate_recursive_backtracker(-5, 8, 9, 9, seed=0) as maze:
        assert maze.extent == (-5, 8, 9, 9)
        assert maze.hash == 303425655


def test_maze_wrapper_loads_checked_room_blend_generator():
    with c_maze.generate_room_blend(-5, 8, 9, 9, seed=0) as maze:
        assert maze.extent == (-5, 8, 9, 9)
        assert maze.hash == 453713525
