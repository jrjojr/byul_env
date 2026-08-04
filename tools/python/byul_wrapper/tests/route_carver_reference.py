"""Exact-integer, production-independent Route Carver raster oracle.

This module is test-only.  It deliberately imports no BYUL native or wrapper
code.  Python arbitrary-precision integers and exact fractions define the
candidate set before the C++ implementation is introduced.
"""

from __future__ import annotations

from fractions import Fraction

Point = tuple[int, int]
Extent = tuple[int, int]


class OutsideExtentError(ValueError):
    """Raised when reject mode sees at least one candidate outside extent."""


def _nearest_integers(numerator: int, denominator: int) -> tuple[int, ...]:
    quotient, remainder = divmod(numerator, denominator)
    doubled = 2 * remainder
    if doubled < denominator:
        return (quotient,)
    if doubled > denominator:
        return (quotient + 1,)
    # An exact half-cell tie has two equally near cell centers.  Keeping both
    # makes the set translation, reversal, reflection, and rotation invariant.
    return (quotient, quotient + 1)


def _center_cells(start: Point, end: Point) -> set[Point]:
    x0, y0 = start
    x1, y1 = end
    dx = x1 - x0
    dy = y1 - y0
    result: set[Point] = set()
    if abs(dx) >= abs(dy):
        if dx == 0:
            return {start}
        if dx < 0:
            x0, y0, x1, y1 = x1, y1, x0, y0
            dx = -dx
            dy = -dy
        for x in range(x0, x1 + 1):
            numerator = y0 * dx + dy * (x - x0)
            result.update((x, y) for y in _nearest_integers(numerator, dx))
        return result

    if dy < 0:
        x0, y0, x1, y1 = x1, y1, x0, y0
        dx = -dx
        dy = -dy
    for y in range(y0, y1 + 1):
        numerator = x0 * dy + dx * (y - y0)
        result.update((x, y) for x in _nearest_integers(numerator, dy))
    return result


def _segment_touches_closed_cell(start: Point, end: Point, cell: Point) -> bool:
    """Exact Liang-Barsky slab test in coordinates scaled by two."""

    px, py = 2 * start[0], 2 * start[1]
    qx, qy = 2 * end[0], 2 * end[1]
    minimum = (2 * cell[0] - 1, 2 * cell[1] - 1)
    maximum = (2 * cell[0] + 1, 2 * cell[1] + 1)
    lower = Fraction(0)
    upper = Fraction(1)
    for origin, delta, slab_min, slab_max in (
        (px, qx - px, minimum[0], maximum[0]),
        (py, qy - py, minimum[1], maximum[1]),
    ):
        if delta == 0:
            if not slab_min <= origin <= slab_max:
                return False
            continue
        first = Fraction(slab_min - origin, delta)
        second = Fraction(slab_max - origin, delta)
        if first > second:
            first, second = second, first
        lower = max(lower, first)
        upper = min(upper, second)
        if lower > upper:
            return False
    return True


def _supercover_cells(start: Point, end: Point) -> set[Point]:
    result: set[Point] = set()
    for y in range(min(start[1], end[1]), max(start[1], end[1]) + 1):
        for x in range(min(start[0], end[0]), max(start[0], end[0]) + 1):
            if _segment_touches_closed_cell(start, end, (x, y)):
                result.add((x, y))
    return result


def raster_line(start: Point, end: Point, coverage: str) -> set[Point]:
    if coverage == "center_cells":
        return _center_cells(start, end)
    if coverage == "supercover_cells":
        return _supercover_cells(start, end)
    raise ValueError("unsupported line coverage")


def metric_offsets(radius: int, metric: str) -> set[Point]:
    if radius < 0:
        raise ValueError("radius must be non-negative")
    result: set[Point] = set()
    for dy in range(-radius, radius + 1):
        for dx in range(-radius, radius + 1):
            if metric == "chebyshev_square":
                included = max(abs(dx), abs(dy)) <= radius
            elif metric == "manhattan_diamond":
                included = abs(dx) + abs(dy) <= radius
            elif metric == "euclidean_disk":
                included = dx * dx + dy * dy <= radius * radius
            else:
                raise ValueError("unsupported carve metric")
            if included:
                result.add((dx, dy))
    return result


def _apply_extent(
    candidates: set[Point], extent: Extent | None, clip_to_extent: bool
) -> set[Point]:
    if extent is None:
        return candidates
    width, height = extent
    if width <= 0 or height <= 0:
        raise ValueError("reference extents must be positive")
    inside = {
        point
        for point in candidates
        if 0 <= point[0] < width and 0 <= point[1] < height
    }
    if not clip_to_extent and inside != candidates:
        raise OutsideExtentError("candidate footprint crosses the grid extent")
    return inside


def line_candidates(
    start: Point,
    end: Point,
    *,
    radius: int = 0,
    metric: str = "chebyshev_square",
    coverage: str = "center_cells",
    include_start: bool = True,
    include_end: bool = True,
    extent: Extent | None = None,
    clip_to_extent: bool = False,
    direction_mode: int = 8,
) -> set[Point]:
    if direction_mode not in {4, 8}:
        raise ValueError("unsupported direction mode")

    seeds = raster_line(start, end, coverage)
    if start == end:
        if not (include_start or include_end):
            seeds.clear()
    else:
        if not include_start:
            seeds.discard(start)
        if not include_end:
            seeds.discard(end)

    offsets = metric_offsets(radius, metric)
    candidates = {
        (point[0] + offset[0], point[1] + offset[1])
        for point in seeds
        for offset in offsets
    }
    # Endpoint exclusion is a final coordinate postcondition; dilation from an
    # adjacent seed cannot silently add the excluded endpoint back.
    if start != end:
        if not include_start:
            candidates.discard(start)
        if not include_end:
            candidates.discard(end)
    return _apply_extent(candidates, extent, clip_to_extent)


def area_candidates(
    center: Point,
    *,
    radius: int = 0,
    metric: str = "chebyshev_square",
    extent: Extent | None = None,
    clip_to_extent: bool = False,
    direction_mode: int = 8,
) -> set[Point]:
    if direction_mode not in {4, 8}:
        raise ValueError("unsupported direction mode")
    candidates = {
        (center[0] + offset[0], center[1] + offset[1])
        for offset in metric_offsets(radius, metric)
    }
    return _apply_extent(candidates, extent, clip_to_extent)
