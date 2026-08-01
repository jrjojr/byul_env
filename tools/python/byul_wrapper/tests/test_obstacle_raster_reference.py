import math

import pytest

from obstacle_raster_reference import (
    covered_by_ring,
    random_rect,
    rasterize_line,
    rasterize_polygon,
    validate_ring,
)


def _map(points, transform):
    return {transform(point) for point in points}


def test_boundary_is_covered_and_all_touched_has_exact_triangle_golden():
    triangle = [(0, 0), (2, 0), (0, 2)]
    center = rasterize_polygon(triangle)
    touched = rasterize_polygon(triangle, raster_rule="all_touched")

    assert covered_by_ring((1, 1), triangle, "even_odd")
    assert center == {(0, 0), (1, 0), (2, 0), (0, 1), (1, 1), (0, 2)}
    assert touched == center | {(2, 1), (1, 2)}


def test_fill_rule_resolves_self_intersection_without_fallback():
    twice_wound_square = [
        (0, 0), (2, 0), (2, 2), (0, 2),
        (0, 0), (2, 0), (2, 2), (0, 2),
    ]
    even_odd = rasterize_polygon(twice_wound_square, fill_rule="even_odd")
    non_zero = rasterize_polygon(twice_wound_square, fill_rule="non_zero")

    assert (1, 1) not in even_odd
    assert (1, 1) in non_zero
    assert even_odd < non_zero


@pytest.mark.parametrize("fill_rule", ["even_odd", "non_zero"])
@pytest.mark.parametrize("raster_rule", ["cell_center", "all_touched"])
def test_polygon_rotation_reflection_and_vertex_reversal_metamorphics(
    fill_rule, raster_rule
):
    polygon = [(-3, -1), (2, -2), (4, 1), (1, 4), (-2, 3)]
    baseline = rasterize_polygon(
        polygon, fill_rule=fill_rule, raster_rule=raster_rule
    )
    transforms = [
        lambda point: (-point[1], point[0]),
        lambda point: (-point[0], point[1]),
        lambda point: (point[0], -point[1]),
    ]
    for transform in transforms:
        assert rasterize_polygon(
            [transform(point) for point in polygon],
            fill_rule=fill_rule,
            raster_rule=raster_rule,
        ) == _map(baseline, transform)
    assert rasterize_polygon(
        list(reversed(polygon)), fill_rule=fill_rule, raster_rule=raster_rule
    ) == baseline


@pytest.mark.parametrize("raster_rule", ["cell_center", "all_touched"])
def test_line_endpoints_reversal_and_supercover(raster_rule):
    forward = rasterize_line((-2, -1), (3, 2), raster_rule=raster_rule)
    reverse = rasterize_line((3, 2), (-2, -1), raster_rule=raster_rule)
    assert forward == reverse
    assert {(-2, -1), (3, 2)} <= forward

    if raster_rule == "all_touched":
        assert rasterize_line((0, 0), (1, 1), raster_rule=raster_rule) == {
            (0, 0), (1, 0), (0, 1), (1, 1)
        }


@pytest.mark.parametrize("raster_rule", ["cell_center", "all_touched"])
def test_every_tiny_line_is_reversal_invariant(raster_rule):
    points = [(x, y) for y in range(-3, 4) for x in range(-3, 4)]
    for start in points:
        for end in points:
            assert rasterize_line(start, end, raster_rule=raster_rule) == rasterize_line(
                end, start, raster_rule=raster_rule
            )


@pytest.mark.parametrize("raster_rule", ["cell_center", "all_touched"])
def test_tiny_lines_preserve_rotation_and_reflection(raster_rule):
    transforms = [
        lambda point: (-point[1], point[0]),
        lambda point: (-point[0], point[1]),
        lambda point: (point[0], -point[1]),
    ]
    for start in [(0, 0), (-2, 1)]:
        for end in [(3, 1), (2, -3), (-1, 3)]:
            baseline = rasterize_line(start, end, raster_rule=raster_rule)
            for transform in transforms:
                assert rasterize_line(
                    transform(start), transform(end), raster_rule=raster_rule
                ) == _map(baseline, transform)


@pytest.mark.parametrize(
    "ring",
    [[], [(0, 0), (1, 0)], [(0, 0), (1, 0), (1, 0)], [(0, 0), (1, 0), (2, 0)]],
)
def test_invalid_ring_is_rejected(ring):
    with pytest.raises(ValueError):
        validate_ring(ring)


def test_splitmix64_probability_contract_and_seed_golden():
    assert random_rect((5, -2), 4, 3, 0.0, 17) == set()
    assert random_rect((5, -2), 4, 3, 1.0, 17) == {
        (x, y) for y in range(-2, 1) for x in range(5, 9)
    }
    assert random_rect((5, -2), 4, 3, 0.5, 17) == {
        (6, -2), (7, -2), (8, -2), (5, -1), (6, -1), (7, -1),
        (5, 0), (7, 0), (8, 0)
    }
    assert random_rect((5, -2), 4, 3, 0.5, 17) == random_rect(
        (5, -2), 4, 3, 0.5, 17
    )
    with pytest.raises(ValueError):
        random_rect((0, 0), 1, 1, math.nan, 0)
