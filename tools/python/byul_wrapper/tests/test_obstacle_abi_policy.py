from pathlib import Path
import json
import re

import pytest

from byul_wrapper.ffi_core import C
from byul_wrapper.obstacle import (
    OBSTACLE_ABI_FINGERPRINT,
    OBSTACLE_ABI_VERSION,
    c_obstacle,
)
from byul_wrapper.navsys_status import NavsysLimitReachedError
from byul_wrapper_generator import parse_header


REPOSITORY_ROOT = Path(__file__).resolve().parents[4]
OBSTACLE_ROOT = REPOSITORY_ROOT / "byul" / "navsys" / "obstacle"
CORE_HEADER = OBSTACLE_ROOT / "obstacle_core.h"
GEOMETRY_HEADER = OBSTACLE_ROOT / "obstacle.h"
ABI1_HEADER = OBSTACLE_ROOT / "compat" / "abi1" / "obstacle_abi1.h"
PRIVATE_HEADER = OBSTACLE_ROOT / "internal" / "obstacle_private.hpp"
WRAPPER = (
    REPOSITORY_ROOT / "tools" / "python" / "byul_wrapper"
    / "byul_wrapper" / "obstacle.py"
)
WRAPPER_POLICY = (
    REPOSITORY_ROOT / "docs" / "ko" / "todo" / "navsys"
    / "obstacle-generator-wrapper-policy.json"
)


def test_obstacle_public_handle_is_opaque_and_abi1_layout_is_opt_in():
    public = CORE_HEADER.read_text(encoding="utf-8")
    compat = ABI1_HEADER.read_text(encoding="utf-8")
    private = PRIVATE_HEADER.read_text(encoding="utf-8")

    assert re.search(r"typedef\s+struct\s+s_obstacle\s+obstacle_t\s*;", public)
    assert not re.search(r"struct\s+s_obstacle\s*\{", public)
    for source in (compat, private):
        assert re.search(r"struct\s+s_obstacle\s*\{", source)
        for field in ("int x0;", "int y0;", "int width;", "int height;",
                      "coord_hash_t* blocked;"):
            assert field in source
    assert re.search(
        r"static_assert\(\s*sizeof\(s_obstacle\)\s*==\s*"
        r"\(sizeof\(void\*\)\s*==\s*8\s*\?\s*24\s*:\s*20\)",
        private,
    )
    assert "BYUL_OBSTACLE_ABI_VERSION UINT32_C(2)" in public
    assert "BYUL_OBSTACLE_ABI1_VERSION UINT32_C(1)" in compat


def test_obstacle_declaration_definition_export_wrapper_intersection():
    declarations = {
        item.name
        for header in (CORE_HEADER, GEOMETRY_HEADER)
        for item in parse_header(header)
    }
    sources = "\n".join(
        path.read_text(encoding="utf-8")
        for path in (
            OBSTACLE_ROOT / "obstacle_core.cpp",
            OBSTACLE_ROOT / "obstacle.cpp",
            OBSTACLE_ROOT / "obstacle_geometry.cpp",
        )
    )
    wrapper = WRAPPER.read_text(encoding="utf-8")
    cdef = wrapper.split('ffi.cdef("""', 1)[1].split('""")', 1)[0]

    assert not {
        name for name in declarations
        if not re.search(
            rf"\b{re.escape(name)}\s*\([^;{{}}]*\)\s*\{{",
            sources,
            re.MULTILINE | re.DOTALL,
        )
    }
    assert declarations <= set(re.findall(r"\b(obstacle_\w+)\s*\(", cdef))
    assert not {name for name in declarations if not hasattr(C, name)}


def test_obstacle_generator_symbol_and_enum_manifest():
    expected_generators = {
        "obstacle_generate_options_init",
        "obstacle_generate_filled_rect",
        "obstacle_generate_rect_outline",
        "obstacle_generate_random_rect",
        "obstacle_generate_line",
        "obstacle_generate_polygon",
        "obstacle_generate_polygon_outline",
        "obstacle_generate_triangle",
        "obstacle_generate_triangle_outline",
        "obstacle_enclosure_desc_init",
        "obstacle_cross_desc_init",
        "obstacle_spiral_desc_init",
        "obstacle_generate_enclosure",
        "obstacle_generate_cross",
        "obstacle_generate_spiral",
        "obstacle_make_rect_all_blocked",
        "obstacle_make_rect_random_blocked",
        "obstacle_make_beam",
        "obstacle_make_torus",
        "obstacle_make_enclosure",
        "obstacle_make_cross",
        "obstacle_make_spiral",
        "obstacle_make_triangle",
        "obstacle_make_triangle_torus",
        "obstacle_make_polygon",
        "obstacle_make_polygon_torus",
    }
    public = GEOMETRY_HEADER.read_text(encoding="utf-8")
    declarations = {item.name for item in parse_header(GEOMETRY_HEADER)}
    assert expected_generators <= declarations
    assert not {name for name in expected_generators if not hasattr(C, name)}

    expected_enclosure = {
        "ENCLOSURE_OPEN_UNKNOWN": 0,
        "ENCLOSURE_OPEN_RIGHT": 1,
        "ENCLOSURE_OPEN_UP": 2,
        "ENCLOSURE_OPEN_LEFT": 3,
        "ENCLOSURE_OPEN_DOWN": 4,
    }
    expected_spiral = {
        "SPIRAL_CLOCKWISE": 0,
        "SPIRAL_COUNTER_CLOCKWISE": 1,
    }
    expected_raster = {
        "OBSTACLE_RASTER_CELL_CENTER": 0,
        "OBSTACLE_RASTER_ALL_TOUCHED": 1,
        "OBSTACLE_POLYGON_EVEN_ODD": 0,
        "OBSTACLE_POLYGON_NON_ZERO": 1,
        "OBSTACLE_ENCLOSURE_CLOSED": 0,
        "OBSTACLE_ENCLOSURE_OPEN_RIGHT": 1,
        "OBSTACLE_ENCLOSURE_OPEN_UP": 2,
        "OBSTACLE_ENCLOSURE_OPEN_LEFT": 3,
        "OBSTACLE_ENCLOSURE_OPEN_DOWN": 4,
        "OBSTACLE_SPIRAL_CLOCKWISE": 0,
        "OBSTACLE_SPIRAL_COUNTER_CLOCKWISE": 1,
        "OBSTACLE_SPIRAL_CLIP_PATH_ONLY": 0,
        "OBSTACLE_SPIRAL_CLIP_OUTPUT": 1,
    }
    for name, value in (
        expected_enclosure | expected_spiral | expected_raster
    ).items():
        assert re.search(rf"\b{re.escape(name)}\b", public)
        assert getattr(C, name) == value


def test_obstacle_wrapper_uses_current_abi_and_buffer_export():
    assert OBSTACLE_ABI_VERSION == 2
    assert OBSTACLE_ABI_FINGERPRINT == 0x4F42535402000000
    with c_obstacle(1, 2, 7, 9) as obstacle:
        assert obstacle.origin == (1, 2)
        assert (obstacle.width, obstacle.height) == (7, 9)
        assert obstacle.set_blocked(2, 3)
        assert {(coord.x, coord.y) for coord in obstacle.blocked_coords()} == {(2, 3)}


def test_obstacle_generator_wrapper_policy_is_closed():
    policy = json.loads(WRAPPER_POLICY.read_text(encoding="utf-8"))
    expected = {
        "obstacle_generate_filled_rect",
        "obstacle_generate_rect_outline",
        "obstacle_generate_random_rect",
        "obstacle_generate_line",
        "obstacle_generate_polygon",
        "obstacle_generate_polygon_outline",
        "obstacle_generate_triangle",
        "obstacle_generate_triangle_outline",
        "obstacle_generate_enclosure",
        "obstacle_generate_cross",
        "obstacle_generate_spiral",
    }
    assert set(policy["canonical_checked_generators"]) == expected
    assert set(policy["mapping"]) == {"ownership", "status", "seed", "options"}
    assert set(policy["excluded_high_level_surface"]) == {
        "cancel_func", "legacy_obstacle_make_symbols"
    }


def test_checked_generator_wrapper_maps_ownership_status_and_seed():
    with c_obstacle.generate_filled_rect(3, 4, 2, 2) as filled:
        assert filled.origin == (3, 4)
        assert {(p.x, p.y) for p in filled.blocked_coords()} == {
            (3, 4), (4, 4), (3, 5), (4, 5)
        }

    first = c_obstacle.generate_random_rect(0, 0, 8, 8, 0.5, seed=17)
    second = c_obstacle.generate_random_rect(0, 0, 8, 8, 0.5, seed=17)
    try:
        assert {(p.x, p.y) for p in first.blocked_coords()} == {
            (p.x, p.y) for p in second.blocked_coords()
        }
    finally:
        first.close()
        second.close()

    with pytest.raises(NavsysLimitReachedError):
        c_obstacle.generate_filled_rect(0, 0, 4, 4, max_cells=3)


def test_checked_generator_wrapper_manual_shape_factories_load():
    factories = [
        c_obstacle.generate_rect_outline(0, 0, 5, 5, 1),
        c_obstacle.generate_line((0, 0), (2, 2)),
        c_obstacle.generate_polygon([(0, 0), (3, 0), (0, 3)]),
        c_obstacle.generate_polygon_outline([(0, 0), (3, 0), (0, 3)]),
        c_obstacle.generate_triangle((0, 0), (3, 0), (0, 3)),
        c_obstacle.generate_triangle_outline((0, 0), (3, 0), (0, 3)),
        c_obstacle.generate_enclosure(0, 0, 5, 5, 1),
        c_obstacle.generate_cross((0, 0), 2),
        c_obstacle.generate_spiral((0, 0), 3, 2),
    ]
    try:
        assert all(obstacle.blocked_coords() for obstacle in factories)
    finally:
        for obstacle in factories:
            obstacle.close()


def test_default_consumers_do_not_use_obstacle_direct_fields():
    consumer_root = REPOSITORY_ROOT / "byul" / "tests" / "sdk_consumer"
    for name in (
        "main.c",
        "main.cpp",
        "obstacle_header_main.c",
        "obstacle_header_main.cpp",
    ):
        assert "obstacle->" not in (consumer_root / name).read_text(encoding="utf-8")

    cmake = (REPOSITORY_ROOT / "byul" / "CMakeLists.txt").read_text(
        encoding="utf-8"
    )
    assert "obstacle/compat/abi1/obstacle_abi1.h" in cmake
    assert "COMPONENT byul_sdk_abi1_compat" in cmake
