import gc
import unittest
import weakref

from byul_wrapper.coord import c_coord
from byul_wrapper.route import RouteCompletion, c_route


class RouteTest(unittest.TestCase):
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


if __name__ == "__main__":
    unittest.main()
