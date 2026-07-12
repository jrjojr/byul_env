from pathlib import Path
import sys

wrapper_path = Path(__file__).resolve().parents[1] / "modules"

sys.path.insert(0, str(wrapper_path.resolve()))

import unittest

from coord import c_coord
from navgrid import c_navgrid, NavgridDirMode
from route_finder import c_route_finder, RouteFinderType
from console import c_console

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
