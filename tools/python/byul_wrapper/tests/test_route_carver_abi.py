from __future__ import annotations

from pathlib import Path

from byul_wrapper import route_carver


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def test_route_carver_generated_cdef_contains_checked_status_surface():
    source = (PROJECT_ROOT / "byul_wrapper" / "route_carver.py").read_text(
        encoding="utf-8"
    )
    assert "navsys_status_t navgrid_carve_line(" in source
    assert "navsys_status_t navgrid_carve_area(" in source
    assert "typedef struct s_navgrid_carve_options" in source
    assert "navgrid_carve_cancel_func" in source


def test_route_carver_status_values_round_trip_through_shared_ffi():
    assert route_carver.ffi.sizeof("navgrid_carve_options_t") >= 48
    assert int(route_carver.NavsysStatus.OK) == 0
    assert int(route_carver.NavsysStatus.CANCELLED) == -9
    assert int(route_carver.NavsysStatus.LIMIT_REACHED) == -10


def test_route_carver_checked_no_change_round_trips_through_native_library():
    grid = route_carver.C.navgrid_create_full(3, 3, 8, route_carver.ffi.NULL)
    assert grid != route_carver.ffi.NULL
    center = route_carver.ffi.new("coord_t*", {"x": 1, "y": 1})
    options = route_carver.ffi.new("navgrid_carve_options_t*")
    options.struct_size = route_carver.ffi.sizeof("navgrid_carve_options_t")
    options.abi_version = 1
    options.radius_cells = 0
    options.metric = 0
    options.line_coverage = 0
    options.match = 1
    options.flags = 8
    options.max_cells = 16
    changed = route_carver.ffi.new("size_t*", 99)
    try:
        status = route_carver.C.navgrid_carve_area(
            grid, center, options, changed
        )
        assert status == int(route_carver.NavsysStatus.OK)
        assert changed[0] == 0
    finally:
        route_carver.C.navgrid_destroy(grid)
