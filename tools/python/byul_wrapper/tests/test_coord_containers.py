import unittest

from byul_wrapper.coord import c_coord
from byul_wrapper.coord_hash import c_coord_hash, c_coord_hash_iter
from byul_wrapper.coord_list import c_coord_list
from byul_wrapper.cost_coord_pq import c_cost_coord_pq
from byul_wrapper.ffi_core import ffi
from byul_wrapper.navsys_status import (
    NavsysInvalidArgumentError,
    NavsysOutOfMemoryError,
    NavsysStatus,
    raise_for_status,
)
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

    def test_mutable_coordinate_is_unhashable_even_when_equal(self):
        with c_coord(3, 7) as first, c_coord(3, 7) as second:
            self.assertEqual(first, second)
            with self.assertRaises(TypeError):
                hash(first)
            with self.assertRaises(TypeError):
                {first}

    def test_close_invalidates_every_public_access(self):
        coord = c_coord(3, 7)
        coord.close()
        coord.close()

        operations = (
            lambda: coord.x,
            lambda: coord.y,
            lambda: coord.to_tuple(),
            lambda: coord.ptr(),
            lambda: coord.copy(),
            lambda: coord.format(),
            lambda: str(coord),
            lambda: repr(coord),
        )
        for operation in operations:
            with self.subTest(operation=operation):
                with self.assertRaises(ReferenceError):
                    operation()
        with self.assertRaises(ReferenceError):
            coord.x = 1
        with self.assertRaises(ReferenceError):
            coord.y = 1

    def test_checked_operations_raise_typed_status_exceptions(self):
        with (
            c_coord(c_coord.COORD_MAX, 0) as maximum,
            c_coord(1, 0) as one,
            c_coord(0, 0) as origin,
        ):
            with self.assertRaises(NavsysInvalidArgumentError) as overflow:
                maximum + one
            self.assertEqual(
                overflow.exception.status, NavsysStatus.INVALID_ARGUMENT
            )
            with self.assertRaises(NavsysInvalidArgumentError):
                origin // 0
            with self.assertRaises(NavsysInvalidArgumentError):
                origin.degree(origin)
            self.assertEqual(maximum.to_tuple(), (c_coord.COORD_MAX, 0))

        with self.assertRaises(NavsysOutOfMemoryError):
            raise_for_status(
                NavsysStatus.OUT_OF_MEMORY, "coord_create_checked"
            )

    def test_native_format_and_checked_metrics(self):
        with c_coord(-3, 7) as first, c_coord(1, 4) as second:
            self.assertEqual(first.format(), "(-3, 7)")
            self.assertEqual(first.manhattan_distance(second), 7)
            self.assertAlmostEqual(first.distance(second), 5.0)


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
