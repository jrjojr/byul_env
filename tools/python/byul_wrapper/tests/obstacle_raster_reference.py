"""Independent exact-integer reference rasterizer for the obstacle contract.

This test-only module deliberately does not import BYUL production code.  Geometry
is evaluated with Python arbitrary-precision integers; cell footprints are scaled
by two so half-cell boundaries stay exact.
"""

from __future__ import annotations

from collections.abc import Iterable, Sequence

Point = tuple[int, int]


def _orient(a: Point, b: Point, c: Point) -> int:
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def _on_segment(point: Point, a: Point, b: Point) -> bool:
    return (
        _orient(a, b, point) == 0
        and min(a[0], b[0]) <= point[0] <= max(a[0], b[0])
        and min(a[1], b[1]) <= point[1] <= max(a[1], b[1])
    )


def _segments_touch(a: Point, b: Point, c: Point, d: Point) -> bool:
    ab_c = _orient(a, b, c)
    ab_d = _orient(a, b, d)
    cd_a = _orient(c, d, a)
    cd_b = _orient(c, d, b)
    if ((ab_c > 0) != (ab_d > 0)) and ((cd_a > 0) != (cd_b > 0)):
        return True
    return (
        (ab_c == 0 and _on_segment(c, a, b))
        or (ab_d == 0 and _on_segment(d, a, b))
        or (cd_a == 0 and _on_segment(a, c, d))
        or (cd_b == 0 and _on_segment(b, c, d))
    )


def _edges(vertices: Sequence[Point]) -> Iterable[tuple[Point, Point]]:
    return zip(vertices, vertices[1:] + vertices[:1])


def validate_ring(vertices: Sequence[Point]) -> None:
    if len(vertices) < 3:
        raise ValueError("a polygon ring needs at least three vertices")
    if any(a == b for a, b in _edges(vertices)):
        raise ValueError("consecutive vertices, including closing vertices, differ")
    a, b = vertices[0], vertices[1]
    if all(_orient(a, b, point) == 0 for point in vertices[2:]):
        raise ValueError("an all-collinear ring has no fill")


def covered_by_ring(point: Point, vertices: Sequence[Point], fill_rule: str) -> bool:
    """Return true for an interior or boundary point, like covered_by."""

    if fill_rule not in {"even_odd", "non_zero"}:
        raise ValueError("unsupported fill rule")
    if any(_on_segment(point, a, b) for a, b in _edges(vertices)):
        return True

    px, py = point
    parity = False
    winding = 0
    for a, b in _edges(vertices):
        if (a[1] > py) != (b[1] > py):
            cross = _orient(a, b, point)
            if (cross > 0) == (b[1] > a[1]):
                parity = not parity
        if a[1] <= py < b[1] and _orient(a, b, point) > 0:
            winding += 1
        elif b[1] <= py < a[1] and _orient(a, b, point) < 0:
            winding -= 1
    return parity if fill_rule == "even_odd" else winding != 0


def _cell_touches_ring(cell: Point, vertices: Sequence[Point], fill_rule: str) -> bool:
    cx, cy = cell
    scaled_vertices = [(2 * x, 2 * y) for x, y in vertices]
    square = [
        (2 * cx - 1, 2 * cy - 1),
        (2 * cx + 1, 2 * cy - 1),
        (2 * cx + 1, 2 * cy + 1),
        (2 * cx - 1, 2 * cy + 1),
    ]
    if any(covered_by_ring(corner, scaled_vertices, fill_rule) for corner in square):
        return True
    if any(
        square[0][0] <= point[0] <= square[2][0]
        and square[0][1] <= point[1] <= square[2][1]
        for point in scaled_vertices
    ):
        return True
    return any(
        _segments_touch(a, b, c, d)
        for a, b in _edges(scaled_vertices)
        for c, d in _edges(square)
    )


def rasterize_polygon(
    vertices: Sequence[Point],
    *,
    raster_rule: str = "cell_center",
    fill_rule: str = "even_odd",
) -> set[Point]:
    validate_ring(vertices)
    if raster_rule not in {"cell_center", "all_touched"}:
        raise ValueError("unsupported raster rule")
    min_x = min(x for x, _ in vertices)
    max_x = max(x for x, _ in vertices)
    min_y = min(y for _, y in vertices)
    max_y = max(y for _, y in vertices)
    result: set[Point] = set()
    for y in range(min_y, max_y + 1):
        for x in range(min_x, max_x + 1):
            selected = (
                covered_by_ring((x, y), vertices, fill_rule)
                if raster_rule == "cell_center"
                else _cell_touches_ring((x, y), vertices, fill_rule)
            )
            if selected:
                result.add((x, y))
    return result


def rasterize_line(start: Point, end: Point, *, raster_rule: str = "cell_center") -> set[Point]:
    if raster_rule == "all_touched":
        result: set[Point] = set()
        a = (2 * start[0], 2 * start[1])
        b = (2 * end[0], 2 * end[1])
        for y in range(min(start[1], end[1]), max(start[1], end[1]) + 1):
            for x in range(min(start[0], end[0]), max(start[0], end[0]) + 1):
                square = [
                    (2 * x - 1, 2 * y - 1),
                    (2 * x + 1, 2 * y - 1),
                    (2 * x + 1, 2 * y + 1),
                    (2 * x - 1, 2 * y + 1),
                ]
                if any(_segments_touch(a, b, c, d) for c, d in _edges(square)) or (
                    square[0][0] <= a[0] <= square[2][0]
                    and square[0][1] <= a[1] <= square[2][1]
                ):
                    result.add((x, y))
        return result
    if raster_rule != "cell_center":
        raise ValueError("unsupported raster rule")

    def nearest_integers(numerator: int, denominator: int) -> tuple[int, ...]:
        quotient, remainder = divmod(numerator, denominator)
        doubled = 2 * remainder
        if doubled < denominator:
            return (quotient,)
        if doubled > denominator:
            return (quotient + 1,)
        return (quotient, quotient + 1)

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
            result.update((x, y) for y in nearest_integers(numerator, dx))
    else:
        if dy < 0:
            x0, y0, x1, y1 = x1, y1, x0, y0
            dx = -dx
            dy = -dy
        for y in range(y0, y1 + 1):
            numerator = x0 * dy + dx * (y - y0)
            result.update((x, y) for x in nearest_integers(numerator, dy))
    return result


def splitmix64_v1(state: int) -> tuple[int, int]:
    mask = (1 << 64) - 1
    state = (state + 0x9E3779B97F4A7C15) & mask
    value = state
    value = ((value ^ (value >> 30)) * 0xBF58476D1CE4E5B9) & mask
    value = ((value ^ (value >> 27)) * 0x94D049BB133111EB) & mask
    return state, (value ^ (value >> 31)) & mask


def random_rect(
    origin: Point, width: int, height: int, probability: float, seed: int
) -> set[Point]:
    if width < 0 or height < 0 or not 0.0 <= probability <= 1.0:
        raise ValueError("invalid random rectangle")
    numerator, denominator = probability.as_integer_ratio()
    state = seed & ((1 << 64) - 1)
    result: set[Point] = set()
    for y in range(origin[1], origin[1] + height):
        for x in range(origin[0], origin[0] + width):
            state, draw = splitmix64_v1(state)
            if draw * denominator < numerator * (1 << 64):
                result.add((x, y))
    return result
