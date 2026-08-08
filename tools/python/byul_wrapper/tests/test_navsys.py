import ast
import importlib
from pathlib import Path
import pkgutil
import unittest

import byul_wrapper
from byul_wrapper.ffi_core import C

from byul_wrapper.coord import c_coord
from byul_wrapper.navgrid import c_navgrid, NavgridDirMode
from byul_wrapper.navsys import (
    find_path,
    get_algorithm_descriptor,
    is_algorithm_supported,
)
from byul_wrapper.navsys_status import NavsysStatus, NavsysUnsupportedError
from byul_wrapper.route_finder import RouteFinderType


class TestNavsysFacade(unittest.TestCase):
    def test_all_concrete_wrapper_c_symbols_resolve(self):
        referenced = set()
        modules = []
        for module_info in pkgutil.iter_modules(byul_wrapper.__path__):
            module = importlib.import_module(
                f"byul_wrapper.{module_info.name}")
            modules.append(module.__name__)
            source = Path(module.__file__).read_text(encoding="utf-8")
            tree = ast.parse(source, filename=module.__file__)
            referenced.update(
                node.attr
                for node in ast.walk(tree)
                if isinstance(node, ast.Attribute)
                and isinstance(node.value, ast.Name)
                and node.value.id == "C"
            )

        self.assertTrue(modules)
        self.assertTrue(referenced)
        self.assertEqual(
            [name for name in sorted(referenced) if not hasattr(C, name)],
            [],
        )

    def setUp(self):
        self.grid = c_navgrid(
            width=10, height=10, mode=NavgridDirMode.DIR_8, own=True)
        self.start = c_coord(0, 0)
        self.goal = c_coord(9, 9)

    def tearDown(self):
        self.grid.close()
        self.start.close()
        self.goal.close()

    def test_capability_descriptor(self):
        self.assertTrue(is_algorithm_supported(RouteFinderType.ASTAR))
        self.assertTrue(is_algorithm_supported(RouteFinderType.DSTAR_LITE))
        self.assertFalse(is_algorithm_supported(RouteFinderType.FAST_MARCHING))
        descriptor = get_algorithm_descriptor(RouteFinderType.DSTAR_LITE)
        self.assertTrue(descriptor.supported)
        self.assertTrue(descriptor.incremental)
        self.assertTrue(descriptor.complete)

    def test_astar_weighted_and_dstar_lite(self):
        status, route, stats = find_path(
            self.grid, self.start, self.goal, RouteFinderType.ASTAR)
        self.assertEqual(NavsysStatus.OK, status)
        self.assertIsNotNone(route)
        self.assertTrue(stats.complete)
        self.assertEqual(route.coord_count(), stats.route_length)
        route.close()

        status, route, stats = find_path(
            self.grid, self.start, self.goal,
            RouteFinderType.WEIGHTED_ASTAR, weight=2.0)
        self.assertEqual(NavsysStatus.OK, status)
        self.assertEqual(RouteFinderType.WEIGHTED_ASTAR, stats.algorithm)
        route.close()

        status, route, stats = find_path(
            self.grid, self.start, self.goal,
            RouteFinderType.DSTAR_LITE, max_expansions=0)
        self.assertEqual(NavsysStatus.OK, status)
        self.assertTrue(stats.complete)
        route.close()

    def test_operational_status_and_unsupported_error(self):
        status, route, stats = find_path(
            self.grid, self.start, self.goal,
            RouteFinderType.ASTAR, cancel=lambda: True)
        self.assertEqual(NavsysStatus.CANCELLED, status)
        self.assertEqual(NavsysStatus.CANCELLED, stats.status)
        if route is not None:
            route.close()

        with self.assertRaises(NavsysUnsupportedError):
            find_path(
                self.grid, self.start, self.goal,
                RouteFinderType.FAST_MARCHING)


if __name__ == "__main__":
    unittest.main()
