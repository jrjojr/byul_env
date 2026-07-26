import gc
import unittest
import weakref

from byul_wrapper.coord import c_coord
from byul_wrapper.ffi_core import ffi
from byul_wrapper.route import (
    RouteCompletion,
    RouteDir,
    RouteJoinPolicy,
    c_route,
    c_route_builder,
    c_route_heading_tracker,
)


class RouteTest(unittest.TestCase):
    def test_null_pointer_and_closed_route_are_rejected(self):
        with self.assertRaises(ValueError):
            c_route(raw_ptr=ffi.NULL, own=False)

        route = c_route()
        route.close()
        closed_calls = (
            route.ptr,
            route.cost,
            route.coord_count,
            route.export_coords,
            route.completion,
            route.format,
            route.identity_hash,
            route.__enter__,
        )
        for call in closed_calls:
            with self.subTest(call=call.__name__):
                with self.assertRaises(ReferenceError):
                    call()

        route.close()

    def test_route_arguments_validate_type_and_lifetime(self):
        with c_route() as route:
            with self.assertRaises(TypeError):
                route.append(object())
            with self.assertRaises(TypeError):
                route.add_coord(object())

            closed_route = c_route()
            closed_route.close()
            with self.assertRaises(ReferenceError):
                route.append(closed_route)

            closed_coord = c_coord(1, 2)
            closed_coord.close()
            with self.assertRaises(ReferenceError):
                route.add_coord(closed_coord)

    def test_context_manager_closes_route_when_body_raises(self):
        route = c_route()
        with self.assertRaisesRegex(RuntimeError, "body failure"):
            with route:
                raise RuntimeError("body failure")
        with self.assertRaises(ReferenceError):
            route.ptr()

    def test_add_query_and_clear_coordinates(self):
        with c_coord(1, 1) as first, c_coord(2, 1) as second, c_route() as route:
            route.add_coord(first)
            route.add_coord(second)

            self.assertEqual(route.length(), 2)
            self.assertEqual(route.coord_count(), 2)
            self.assertTrue(route.contains(first))
            self.assertEqual(route.coord_at(0).to_tuple(), (1, 1))
            self.assertEqual(route.last().to_tuple(), (2, 1))
            self.assertEqual(route.fetch_coord(1), (2, 1))
            with self.assertRaises(IndexError):
                route.fetch_coord(2)
            self.assertEqual(route.export_coords(), [(1, 1), (2, 1)])

            route.clear_coords()
            self.assertEqual(route.length(), 0)
            self.assertEqual(route.export_coords(), [])

    def test_cost_success_and_retry_properties(self):
        with c_route(cost=3.5) as route:
            self.assertEqual(route.completion(), RouteCompletion.NONE)
            route.set_success(True)
            route.set_retry_count(7)

            self.assertAlmostEqual(route.cost(), 3.5)
            self.assertAlmostEqual(route.total_cost(), 3.5)
            self.assertTrue(route.is_success())
            self.assertEqual(route.completion(), RouteCompletion.COMPLETE)
            self.assertEqual(route.retry_count(), 7)

    def test_partial_completion(self):
        with c_coord(4, 5) as coord, c_route() as route:
            route.add_coord(coord)
            self.assertEqual(route.completion(), RouteCompletion.PARTIAL)

    def test_borrowed_views_retain_parent_and_detect_explicit_close(self):
        route = c_route()
        with c_coord(2, 3) as coord:
            route.add_coord(coord)
        borrowed = route.coords()
        route_ref = weakref.ref(route)

        del route
        gc.collect()
        self.assertIsNotNone(route_ref())
        self.assertEqual(len(borrowed), 1)

        parent = route_ref()
        parent.close()
        with self.assertRaises(ReferenceError):
            len(borrowed)
        del parent
        borrowed.close()
        del borrowed
        gc.collect()
        self.assertIsNone(route_ref())

    def test_returned_coordinates_are_independent_owned_values(self):
        route = c_route()
        with c_coord(1, 1) as first, c_coord(2, 1) as second:
            route.add_coord(first)
            route.add_coord(second)

        last = route.last()
        indexed = route.coord_at(0)
        direction = route.look_at(0)
        route.close()

        try:
            self.assertEqual(last.to_tuple(), (2, 1))
            self.assertEqual(indexed.to_tuple(), (1, 1))
            self.assertEqual(direction.to_tuple(), (1, 0))
        finally:
            last.close()
            indexed.close()
            direction.close()

    def test_builder_is_transactional_and_returns_owned_route(self):
        with c_route() as source, c_coord(0, 0) as first, c_coord(1, 0) as second:
            source.add_coord(first)
            source.add_coord(second)
            with source.to_builder() as builder:
                builder.append(source, RouteJoinPolicy.DEDUP_BOUNDARY)
                builder.set_total_cost(4.5)
                builder.set_completion(RouteCompletion.COMPLETE)
                result = builder.finish()

            try:
                self.assertEqual(
                    result.export_coords(),
                    [(0, 0), (1, 0), (0, 0), (1, 0)],
                )
                self.assertAlmostEqual(result.total_cost(), 4.5)
                self.assertEqual(result.completion(), RouteCompletion.COMPLETE)
                self.assertEqual(result.retry_count(), 0)
            finally:
                result.close()

    def test_builder_finish_consumes_builder_and_close_is_idempotent(self):
        builder = c_route_builder()
        with c_coord(3, 4) as coord:
            builder.push(coord)
        result = builder.finish()
        try:
            self.assertEqual(result.export_coords(), [(3, 4)])
            with self.assertRaises(ReferenceError):
                builder.finish()
            with self.assertRaises(ReferenceError):
                builder.__enter__()
        finally:
            builder.close()
            builder.close()
            result.close()

    def test_builder_finalizer_and_repeated_owned_results(self):
        builder = c_route_builder()
        builder_ref = weakref.ref(builder)
        del builder
        gc.collect()
        self.assertIsNone(builder_ref())

        for value in range(250):
            with c_route_builder() as repeated:
                with c_coord(value, -value) as coord:
                    repeated.push(coord)
                result = repeated.finish()
            try:
                self.assertEqual(result.fetch_coord(0), (value, -value))
            finally:
                result.close()

    def test_builder_slice_and_remove(self):
        with c_route_builder() as builder:
            for value in range(4):
                with c_coord(value, -value) as coord:
                    builder.push(coord)
            source = builder.finish()

        try:
            with c_route_builder() as sliced:
                sliced.assign_slice(source, 1, 4)
                self.assertEqual(sliced.remove(1), (2, -2))
                result = sliced.finish()
            try:
                self.assertEqual(result.export_coords(), [(1, -1), (3, -3)])
                self.assertEqual(result.completion(), RouteCompletion.PARTIAL)
            finally:
                result.close()
        finally:
            source.close()

    def test_canonical_direction_values_and_route_queries(self):
        expected = {
            RouteDir.RIGHT: (1, 0),
            RouteDir.UP_RIGHT: (1, -1),
            RouteDir.UP: (0, -1),
            RouteDir.UP_LEFT: (-1, -1),
            RouteDir.LEFT: (-1, 0),
            RouteDir.DOWN_LEFT: (-1, 1),
            RouteDir.DOWN: (0, 1),
            RouteDir.DOWN_RIGHT: (1, 1),
        }
        for _ in range(1000):
            for direction, vector in expected.items():
                with c_route.direction_to_coord(direction) as converted:
                    self.assertEqual(converted.to_tuple(), vector)

        with c_coord(0, 0) as start, c_coord(100, -50) as goal:
            self.assertEqual(
                c_route.calc_direction(start, goal), RouteDir.UP_RIGHT
            )

        with c_route() as route:
            for x, y in ((0, 0), (5, 0), (5, -8)):
                with c_coord(x, y) as point:
                    route.add_coord(point)
            self.assertEqual(route.direction_at(0), RouteDir.RIGHT)
            self.assertEqual(route.direction_at(2), RouteDir.UP)
            self.assertEqual(route.recent_facing(2), RouteDir.UP_RIGHT)
            self.assertAlmostEqual(route.recent_heading_degrees(1), -90.0)

    def test_heading_tracker_is_independent_and_deterministic(self):
        route = c_route(cost=7.5)
        route.set_success(True)
        with c_route_heading_tracker() as tracker:
            with c_coord(1, 0) as right, c_coord(0, 1) as down:
                changed, angle = tracker.observe_vector(right, 90.0)
                self.assertFalse(changed)
                self.assertAlmostEqual(angle, 0.0)
                changed, angle = tracker.observe_vector(down, 90.0)
                self.assertTrue(changed)
                self.assertAlmostEqual(angle, 90.0)
                self.assertEqual(tracker.sample_count, 2)
                self.assertAlmostEqual(tracker.heading_degrees, 45.0)
                tracker.reset()
                self.assertEqual(tracker.sample_count, 0)
        self.assertAlmostEqual(route.cost(), 7.5)
        self.assertTrue(route.is_success())
        route.close()

    def test_identity_content_find_slice_and_format_use_canonical_api(self):
        with c_route() as route:
            for x, y in ((1, 2), (-3, 4)):
                with c_coord(x, y) as point:
                    route.add_coord(point)
            route.set_cost(2.5)
            route.set_success(True)
            with route.to_builder() as builder:
                copy = builder.finish()

            try:
                self.assertFalse(route.is_same(copy))
                self.assertNotEqual(route.identity_hash(), copy.identity_hash())
                self.assertTrue(route.content_equal(copy))
                self.assertEqual(route.content_hash(), copy.content_hash())
                self.assertEqual(
                    route.format(),
                    "Route(len : 2): (1, 2) -> (-3, 4)\n",
                )
                self.assertEqual(
                    route.to_string(),
                    "Route(len : 2): (1, 2) -> (-3, 4)",
                )
                with c_coord(-3, 4) as present, c_coord(9, 9) as missing:
                    self.assertEqual(route.find(present), 1)
                    self.assertEqual(route.find(missing), -1)
                with route.slice(1, 2) as sliced:
                    self.assertEqual(sliced.export_coords(), [(-3, 4)])
            finally:
                copy.close()


if __name__ == "__main__":
    unittest.main()
