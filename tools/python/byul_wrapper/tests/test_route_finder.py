import unittest

from byul_wrapper.coord import c_coord
from byul_wrapper.navgrid import c_navgrid, NavgridDirMode
from byul_wrapper.route_finder import c_route_finder, RouteFinderType
from byul_wrapper.console import c_console

class TestRouteFinder(unittest.TestCase):
    def setUp(self):
        self.navgrid = c_navgrid(width=10, height=10, mode=NavgridDirMode.DIR_8)
        self.route_finder = c_route_finder(self.navgrid, debug=True)

        self.start = c_coord(0, 0)
        self.goal = c_coord(9, 9)
        self.route_finder.set_start(self.start)
        self.route_finder.set_goal(self.goal)

        # // 장애물 삽입 (세로 차단)
        for i in range(1, 10):
            self.navgrid.block(5, i)

    def tearDown(self):
        self.route_finder.close()

    def test_capability_query(self):
        supported = c_route_finder.list_supported_route_finders()

        self.assertEqual(11, len(supported))
        self.assertIn(RouteFinderType.ASTAR, supported)
        self.assertIn(RouteFinderType.WEIGHTED_ASTAR, supported)
        self.assertNotIn(RouteFinderType.DSTAR_LITE, supported)
        self.assertFalse(
            c_route_finder.is_supported(RouteFinderType.BELLMAN_FORD)
        )

    def test_typed_algorithm_configs(self):
        bindings = [
            (
                self.route_finder.bind_fringe_search_config,
                0.5,
                RouteFinderType.FRINGE_SEARCH,
            ),
            (
                self.route_finder.bind_rta_star_config,
                7,
                RouteFinderType.RTA_STAR,
            ),
            (
                self.route_finder.bind_sma_star_config,
                64,
                RouteFinderType.SMA_STAR,
            ),
            (
                self.route_finder.bind_weighted_astar_config,
                2.0,
                RouteFinderType.WEIGHTED_ASTAR,
            ),
        ]

        for bind, value, expected_type in bindings:
            bind(value)
            self.assertEqual(expected_type, self.route_finder.get_type())
            self.assertIsNotNone(self.route_finder._algorithm_config)

        with self.assertRaises(ValueError):
            self.route_finder.bind_weighted_astar_config(10.1)
        with self.assertRaises(ValueError):
            self.route_finder.bind_fringe_search_config(float("nan"))
        with self.assertRaises(ValueError):
            self.route_finder.bind_weighted_astar_config(float("inf"))
        self.assertEqual(
            RouteFinderType.WEIGHTED_ASTAR,
            self.route_finder.get_type(),
        )
        self.assertIsNotNone(self.route_finder._algorithm_config)

        self.route_finder.unbind_algorithm_config()
        self.assertEqual(
            RouteFinderType.WEIGHTED_ASTAR,
            self.route_finder.get_type(),
        )
        self.assertIsNone(self.route_finder._algorithm_config)

    def test_find_default(self):
        print("test_find_default")
        route = self.route_finder.find()
        route.print()
        c_console.print_ascii_with_visited_count(self.navgrid, route)

    def test_find_bfs(self):
        print('test_find_bfs')
        self.route_finder.set_type(RouteFinderType.BFS)
        route = self.route_finder.find()
        route.print()
        c_console.print_ascii_with_visited_count(self.navgrid, route)

    def test_find_astar(self):
        print('test_find_astar')
        self.route_finder.set_type(RouteFinderType.ASTAR)
        route = self.route_finder.find()
        route.print()
        c_console.print_ascii_with_visited_count(self.navgrid, route)        

    def test_find_dfs(self):
        print('test_find_dfs')
        #include "internal/dfs.h"
        self.route_finder.set_type(RouteFinderType.DFS)
        route = self.route_finder.find()
        route.print()
        c_console.print_ascii_with_visited_count(self.navgrid, route)        

    def test_find_dijkstra(self):
        print('test_find_dijkstra')
        #include "internal/dijkstra.h"
        self.route_finder.set_type(RouteFinderType.DIJKSTRA)
        route = self.route_finder.find()
        route.print()
        c_console.print_ascii_with_visited_count(self.navgrid, route)        

    def test_find_fast_marching(self):
        print('test_find_fast_marching')
        #include "internal/fast_marching.h"
        self.route_finder.set_type(RouteFinderType.FAST_MARCHING)
        route = self.route_finder.find()
        route.print()
        c_console.print_ascii_with_visited_count(self.navgrid, route)        

    def test_find_fringe_search(self):
        print('test_find_fringe_search')
        #include "internal/fringe_search.h"
        self.route_finder.set_type(RouteFinderType.FRINGE_SEARCH)
        route = self.route_finder.find()
        route.print()
        c_console.print_ascii_with_visited_count(self.navgrid, route)        

    def test_find_greedy_best_first(self):
        print('test_find_greedy_best_first')
        #include "internal/greedy_best_first.h"
        self.route_finder.set_type(RouteFinderType.GREEDY_BEST_FIRST)
        route = self.route_finder.find()
        route.print()
        c_console.print_ascii_with_visited_count(self.navgrid, route)        

    def test_find_ida_star(self):
        print('test_find_ida_star')
        #include "internal/ida_star.h"
        self.route_finder.set_type(RouteFinderType.IDA_STAR)
        route = self.route_finder.find()
        route.print()
        c_console.print_ascii_with_visited_count(self.navgrid, route)        

    def test_find_rta_star(self):
        print('test_find_rta_star')
        #include "internal/rta_star.h"
        self.route_finder.set_type(RouteFinderType.RTA_STAR)
        route = self.route_finder.find()
        route.print()
        c_console.print_ascii_with_visited_count(self.navgrid, route)        

    def test_find_sma_star(self):
        print('test_find_sma_star')
        #include "internal/sma_star.h"
        self.route_finder.set_type(RouteFinderType.SMA_STAR)
        route = self.route_finder.find()
        route.print()
        c_console.print_ascii_with_visited_count(self.navgrid, route)        

    def test_find_weighted_astar(self):
        print('/.test_find_weighted_astar')
        #include "internal/weighted_astar.h"        
        self.route_finder.set_type(RouteFinderType.WEIGHTED_ASTAR)
        route = self.route_finder.find()
        route.print()
        c_console.print_ascii_with_visited_count(self.navgrid, route)                

# 🔽 여기서부터 직접 실행 시 동작
if __name__ == '__main__':
    unittest.main()
