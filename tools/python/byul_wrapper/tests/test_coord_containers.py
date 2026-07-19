import gc
import unittest
import weakref

from byul_wrapper.coord import c_coord
from byul_wrapper.coord_hash import c_coord_hash, c_coord_hash_iter
from byul_wrapper.coord_list import c_coord_list
from byul_wrapper.cost_coord_pq import c_cost_coord_pq
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

    def test_empty_pop_index_and_slice_contracts(self):
        with c_coord_list() as coords:
            self.assertIsNone(coords.front())
            self.assertIsNone(coords.back())
            with self.assertRaises(IndexError):
                coords.pop()
            with self.assertRaises(IndexError):
                coords.pop_front()
            with self.assertRaises(IndexError):
                _ = coords[0]

            empty = coords[0:0]
            try:
                self.assertEqual(len(empty), 0)
            finally:
                empty.close()

            with self.assertRaises(ValueError):
                _ = coords[::2]

    def test_fetch_values_survive_mutation_parent_close_and_gc(self):
        coords = c_coord_list()
        with c_coord(4, 5) as first:
            coords.append(first)
        fetched = coords[0]
        for index in range(32):
            with c_coord(index, -index) as value:
                coords.append(value)
        self.assertEqual(fetched.to_tuple(), (4, 5))

        parent_ref = weakref.ref(coords)
        coords.close()
        del coords
        gc.collect()
        self.assertIsNone(parent_ref())
        self.assertEqual(fetched.to_tuple(), (4, 5))
        fetched.close()

    def test_checked_mutation_copy_export_and_close_state(self):
        with (
            c_coord(1, 2) as first,
            c_coord(3, 4) as second,
            c_coord_list.from_list([first, second]) as coords,
        ):
            self.assertEqual(
                [coord.to_tuple() for coord in coords.to_list()],
                [(1, 2), (3, 4)],
            )
            full = coords[:]
            empty = coords[1:1]
            try:
                self.assertTrue(full.equals(coords))
                self.assertEqual(len(empty), 0)
            finally:
                full.close()
                empty.close()

            self.assertEqual(coords.index(second), 1)
            self.assertTrue(coords.remove_value(first))
            self.assertFalse(coords.remove_value(first))
            self.assertEqual(coords.pop_front().to_tuple(), (3, 4))

        closed = c_coord_list()
        closed.close()
        closed.close()
        with self.assertRaises(ReferenceError):
            len(closed)
        with self.assertRaises(ReferenceError):
            closed.ptr()


class CoordHashTest(unittest.TestCase):
    def test_insert_lookup_iteration_and_remove(self):
        with c_coord(4, 5) as key, c_coord_hash() as values:
            self.assertTrue(values.insert(key, 17))
            self.assertTrue(values.contains(key))
            self.assertEqual(values.get(key), 17)
            self.assertEqual(len(values), 1)

            iterator = c_coord_hash_iter(values)
            try:
                iterated_key, iterated_value = next(iterator)
                self.assertEqual(iterated_key.to_tuple(), (4, 5))
                self.assertEqual(iterated_value, 17)
                iterated_key.close()
            finally:
                iterator.close()

            self.assertTrue(values.remove(key))
            self.assertTrue(values.empty())

    def test_typed_upsert_export_and_format(self):
        with (
            c_coord(2, 3) as later,
            c_coord(-1, 4) as earlier,
            c_coord_hash(value_type="float") as values,
        ):
            self.assertTrue(values.upsert(later, 1.5))
            self.assertFalse(values.upsert(later, 2.5))
            self.assertTrue(values.insert(earlier, 4.0))
            self.assertEqual(values.get(later), 2.5)
            self.assertCountEqual(values.values(), [2.5, 4.0])
            self.assertEqual(values.format(), "(-1,4) (2,3) ")
            entries = values.entries()
            try:
                self.assertCountEqual(
                    [(key.to_tuple(), value) for key, value in entries],
                    [((-1, 4), 4.0), ((2, 3), 2.5)],
                )
            finally:
                for key, _ in entries:
                    key.close()

    def test_none_codec_rejects_non_null_values(self):
        with c_coord(1, 1) as key, c_coord_hash(value_type="none") as values:
            self.assertTrue(values.insert(key, None))
            self.assertIsNone(values.get(key))
            with self.assertRaises(TypeError):
                values.upsert(key, 1)

    def test_iterator_retains_parent_until_close(self):
        key = c_coord(7, 8)
        values = c_coord_hash()
        values.insert(key, 70)
        iterator = iter(values)
        values_ref = weakref.ref(values)
        del values
        gc.collect()
        self.assertIsNotNone(values_ref())

        iterated_key, iterated_value = next(iterator)
        self.assertEqual(iterated_key.to_tuple(), (7, 8))
        self.assertEqual(iterated_value, 70)
        iterated_key.close()
        iterator.close()
        gc.collect()
        self.assertIsNone(values_ref())
        key.close()

    def test_close_invalidates_public_access(self):
        values = c_coord_hash()
        values.close()
        values.close()
        with self.assertRaises(ReferenceError):
            len(values)
        with self.assertRaises(ReferenceError):
            values.ptr()


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
