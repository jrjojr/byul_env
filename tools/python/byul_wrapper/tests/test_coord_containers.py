import unittest

from byul_wrapper.coord import c_coord
from byul_wrapper.coord_hash import c_coord_hash, c_coord_hash_iter
from byul_wrapper.coord_list import c_coord_list
from byul_wrapper.cost_coord_pq import c_cost_coord_pq
from byul_wrapper.ffi_core import ffi
from byul_wrapper.route_finder_common import c_route_func_registry


class CoordTest(unittest.TestCase):
    def test_coordinate_copy_is_independent(self):
        with c_coord(3, 7) as coord:
            copied = coord.copy()
            try:
                copied.x = 10
                self.assertEqual(coord.to_tuple(), (3, 7))
                self.assertEqual(copied.to_tuple(), (10, 7))
            finally:
                copied.close()


class CoordListTest(unittest.TestCase):
    def test_append_iterate_copy_and_clear(self):
        with c_coord(1, 2) as coord, c_coord_list() as coords:
            coords.append(coord)
            self.assertEqual(len(coords), 1)
            self.assertEqual(coords[0].to_tuple(), (1, 2))

            copied = coords.copy()
            try:
                self.assertTrue(copied.equals(coords))
            finally:
                copied.close()

            coords.clear()
            self.assertTrue(coords.empty())


class CoordHashTest(unittest.TestCase):
    def test_insert_lookup_iteration_and_remove(self):
        with c_coord(4, 5) as key, c_coord_hash() as values:
            value = ffi.new_handle(17)
            self.assertTrue(values.insert(key, value))
            self.assertTrue(values.contains(key))
            self.assertEqual(len(values), 1)

            iterator = c_coord_hash_iter(values)
            try:
                iterated_key, _ = next(iterator)
                self.assertEqual(iterated_key.to_tuple(), (4, 5))
            finally:
                iterator.close()

            self.assertTrue(values.remove(key))
            self.assertTrue(values.empty())


class CostCoordPriorityQueueTest(unittest.TestCase):
    def test_push_contains_and_remove(self):
        with c_coord(2, 3) as coord, c_cost_coord_pq() as queue:
            queue.push(1.5, coord)
            self.assertEqual(len(queue), 1)
            self.assertTrue(queue.contains(coord))
            self.assertTrue(queue.remove(1.5, coord))
            self.assertTrue(queue.is_empty())


class RouteFunctionRegistryTest(unittest.TestCase):
    def test_registers_and_resolves_functions(self):
        registry = c_route_func_registry()
        cost = object()
        heuristic = object()

        registry.register_cost("cost", cost)
        registry.register_heuristic("heuristic", heuristic)

        self.assertIs(registry.get_cost_func("cost"), cost)
        self.assertIs(registry.get_heuristic_func("heuristic"), heuristic)
        self.assertEqual(registry.all_cost_names(), ["cost"])
        self.assertEqual(registry.all_heuristic_names(), ["heuristic"])


if __name__ == "__main__":
    unittest.main()
