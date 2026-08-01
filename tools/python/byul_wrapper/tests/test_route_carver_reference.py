import pytest

from route_carver_reference import (
    OutsideExtentError,
    area_candidates,
    line_candidates,
    metric_offsets,
    raster_line,
)


def _map(points, transform):
    return {transform(point) for point in points}


@pytest.mark.parametrize("coverage", ["center_cells", "supercover_cells"])
def test_exhaustive_7x7_lines_are_reversible_and_transform_exactly(coverage):
    points = [(x, y) for y in range(-3, 4) for x in range(-3, 4)]
    rotate = lambda point: (-point[1], point[0])
    translate = lambda point: (point[0] + 5, point[1] - 7)
    for start in points:
        for end in points:
            baseline = raster_line(start, end, coverage)
            assert start in baseline
            assert end in baseline
            assert baseline == raster_line(end, start, coverage)
            assert _map(baseline, rotate) == raster_line(
                rotate(start), rotate(end), coverage
            )
            assert _map(baseline, translate) == raster_line(
                translate(start), translate(end), coverage
            )


def test_line_ties_and_closed_corner_touch_have_frozen_golden_sets():
    assert raster_line((0, 0), (2, 1), "center_cells") == {
        (0, 0), (1, 0), (1, 1), (2, 1)
    }
    assert raster_line((0, 0), (1, 1), "supercover_cells") == {
        (0, 0), (1, 0), (0, 1), (1, 1)
    }
    assert raster_line((0, 0), (0, 0), "supercover_cells") == {(0, 0)}


@pytest.mark.parametrize(
    ("metric", "radius", "expected_count"),
    [
        ("chebyshev_square", 0, 1),
        ("chebyshev_square", 4, 81),
        ("manhattan_diamond", 4, 41),
        ("euclidean_disk", 4, 49),
    ],
)
def test_radius_0_to_4_metric_predicates_and_counts(metric, radius, expected_count):
    for current_radius in range(radius + 1):
        offsets = metric_offsets(current_radius, metric)
        assert (0, 0) in offsets
        assert offsets == {(-x, y) for x, y in offsets}
        assert offsets == {(x, -y) for x, y in offsets}
        assert offsets == {(-y, x) for x, y in offsets}
    assert len(metric_offsets(radius, metric)) == expected_count


@pytest.mark.parametrize("metric", [
    "chebyshev_square", "manhattan_diamond", "euclidean_disk"
])
def test_area_radius_0_to_4_is_translation_rotation_and_mode_invariant(metric):
    rotate = lambda point: (-point[1], point[0])
    for radius in range(5):
        baseline = area_candidates((0, 0), radius=radius, metric=metric)
        assert area_candidates(
            (0, 0), radius=radius, metric=metric, direction_mode=4
        ) == baseline
        assert _map(baseline, rotate) == area_candidates(
            (0, 0), radius=radius, metric=metric
        )
        assert {(x + 11, y - 13) for x, y in baseline} == area_candidates(
            (11, -13), radius=radius, metric=metric
        )


@pytest.mark.parametrize("coverage", ["center_cells", "supercover_cells"])
@pytest.mark.parametrize("metric", [
    "chebyshev_square", "manhattan_diamond", "euclidean_disk"
])
def test_line_options_produce_one_unique_mode_independent_set(coverage, metric):
    kwargs = {
        "radius": 2,
        "metric": metric,
        "coverage": coverage,
        "include_start": False,
        "include_end": True,
    }
    four = line_candidates((-2, -1), (3, 2), direction_mode=4, **kwargs)
    eight = line_candidates((-2, -1), (3, 2), direction_mode=8, **kwargs)
    assert four == eight
    assert (-2, -1) not in four
    assert (3, 2) in four


def test_point_line_endpoint_flags_are_unambiguous():
    point = (3, -2)
    assert line_candidates(point, point) == {point}
    assert line_candidates(point, point, include_start=False) == {point}
    assert line_candidates(point, point, include_end=False) == {point}
    assert line_candidates(
        point, point, include_start=False, include_end=False
    ) == set()


def test_clip_and_reject_are_operation_wide_and_deterministic():
    clipped = area_candidates(
        (0, 0), radius=1, extent=(3, 3), clip_to_extent=True
    )
    assert clipped == {(0, 0), (1, 0), (0, 1), (1, 1)}
    with pytest.raises(OutsideExtentError):
        area_candidates((0, 0), radius=1, extent=(3, 3))
    with pytest.raises(OutsideExtentError):
        line_candidates((-1, 1), (1, 1), extent=(3, 3))
    assert line_candidates(
        (-1, 1), (1, 1), extent=(3, 3), clip_to_extent=True
    ) == {(0, 1), (1, 1)}


@pytest.mark.parametrize("bad_metric", ["", "square", "unknown"])
def test_invalid_metric_is_rejected_without_fallback(bad_metric):
    with pytest.raises(ValueError):
        metric_offsets(1, bad_metric)


def test_invalid_coverage_radius_extent_and_mode_are_rejected():
    with pytest.raises(ValueError):
        raster_line((0, 0), (1, 1), "unknown")
    with pytest.raises(ValueError):
        metric_offsets(-1, "chebyshev_square")
    with pytest.raises(ValueError):
        area_candidates((0, 0), extent=(0, 3))
    with pytest.raises(ValueError):
        line_candidates((0, 0), (1, 1), direction_mode=6)
