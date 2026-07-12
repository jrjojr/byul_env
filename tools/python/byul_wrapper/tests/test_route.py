import unittest

from byul_wrapper.coord import c_coord
from byul_wrapper.route import c_route


class RouteTest(unittest.TestCase):
    def test_add_query_and_clear_coordinates(self):
        with c_coord(1, 1) as first, c_coord(2, 1) as second, c_route() as route:
            route.add_coord(first)
            route.add_coord(second)

            self.assertEqual(route.length(), 2)
            self.assertTrue(route.contains(first))
            self.assertEqual(route.coord_at(0).to_tuple(), (1, 1))
            self.assertEqual(route.last().to_tuple(), (2, 1))

            route.clear_coords()
            self.assertEqual(route.length(), 0)

    def test_cost_success_and_retry_properties(self):
        with c_route(cost=3.5) as route:
            route.set_success(True)
            route.set_retry_count(7)

            self.assertAlmostEqual(route.cost(), 3.5)
            self.assertTrue(route.is_success())
            self.assertEqual(route.retry_count(), 7)


if __name__ == "__main__":
    unittest.main()
