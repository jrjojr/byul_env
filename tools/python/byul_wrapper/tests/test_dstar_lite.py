import math
import unittest

from byul_wrapper.coord import c_coord
from byul_wrapper.dstar_lite import c_dstar_lite
from byul_wrapper.dstar_lite_key import c_dstar_lite_key
from byul_wrapper.dstar_lite_pqueue import c_dstar_lite_pqueue
from byul_wrapper.navgrid import NavgridDirMode, c_navgrid
from byul_wrapper.navsys_status import (
    NavsysCallbackError,
    NavsysCancelledError,
    NavsysInvalidArgumentError,
    NavsysStatus,
)


class DStarLiteSupportTest(unittest.TestCase):
    def test_key_copy_and_comparison(self):
        with c_dstar_lite_key(1.0, 2.0) as key:
            copied = key.copy()
            try:
                self.assertEqual(key.to_tuple(), (1.0, 2.0))
                self.assertEqual(copied, key)
            finally:
                copied.close()

    def test_key_uses_exact_immutable_value_semantics(self):
        exact = c_dstar_lite_key(1.0, 2.0)
        close = c_dstar_lite_key(1.0000001192092896, 2.0)
        negative_zero = c_dstar_lite_key(-0.0, 0.0)
        positive_zero = c_dstar_lite_key(0.0, 0.0)
        try:
            self.assertNotEqual(exact, close)
            self.assertLess(exact, close)
            self.assertEqual(len({exact, close}), 2)
            self.assertEqual({exact: "exact"}[exact], "exact")
            self.assertEqual(negative_zero, positive_zero)
            self.assertEqual(hash(negative_zero), hash(positive_zero))
            self.assertEqual(len({negative_zero, positive_zero}), 1)
            with self.assertRaises(AttributeError):
                exact.k1 = 7.0
            pointer_snapshot = exact.ptr()
            pointer_snapshot.k1 = 7.0
            self.assertEqual(exact.to_tuple(), (1.0, 2.0))
            copied = exact.copy()
            try:
                self.assertEqual(hash(exact), hash(copied))
            finally:
                copied.close()

            exact_hash = hash(exact)
            exact.close()
            self.assertEqual(hash(exact), exact_hash)
            self.assertEqual(exact.to_tuple(), (1.0, 2.0))
            with self.assertRaises(ReferenceError):
                exact.ptr()
        finally:
            exact.close()
            close.close()
            negative_zero.close()
            positive_zero.close()

    def test_key_closeness_is_explicit_and_nontransitive(self):
        a = c_dstar_lite_key(1.0, 0.0)
        b = c_dstar_lite_key(1.000009, 0.0)
        c = c_dstar_lite_key(1.000018, 0.0)
        try:
            tolerances = {
                "absolute_tolerance": 0.0,
                "relative_tolerance": 1e-5,
            }
            self.assertTrue(a.is_close(b, **tolerances))
            self.assertTrue(b.is_close(c, **tolerances))
            self.assertFalse(a.is_close(c, **tolerances))
            self.assertEqual(len({a, b, c}), 3)
            with self.assertRaises(NavsysInvalidArgumentError):
                a.is_close(
                    b,
                    absolute_tolerance=-1.0,
                    relative_tolerance=0.0,
                )
        finally:
            a.close()
            b.close()
            c.close()

    def test_key_maps_invalid_canonical_values_to_status_exception(self):
        for invalid in (math.nan, -math.inf):
            with self.subTest(invalid=invalid):
                with self.assertRaises(NavsysInvalidArgumentError):
                    c_dstar_lite_key(invalid, 0.0)

        with c_dstar_lite_key(math.inf, -0.0) as key:
            self.assertEqual(key.to_tuple(), (math.inf, 0.0))

    def test_priority_queue_push_and_remove(self):
        with c_dstar_lite_pqueue() as queue:
            with c_dstar_lite_key(1.0, 2.0) as key, c_coord(3, 4) as coord:
                queue.push(key, coord)
                self.assertFalse(queue.is_empty())
                self.assertTrue(queue.contains(coord))
                self.assertTrue(queue.remove(coord))
                self.assertTrue(queue.is_empty())

    def test_priority_queue_replaces_coordinate_key_without_borrowing(self):
        with c_dstar_lite_pqueue() as queue:
            with c_coord(3, 4) as coord:
                with c_dstar_lite_key(9.0, 1.0) as old_key:
                    queue.push(old_key, coord)
                with c_dstar_lite_key(2.0, 1.0) as new_key:
                    queue.push(new_key, coord)
                observed = queue.find_key_by_coord(coord)
                popped = queue.pop()
                try:
                    self.assertEqual(observed.to_tuple(), (2.0, 1.0))
                    self.assertEqual((popped.x, popped.y), (3, 4))
                    self.assertTrue(queue.is_empty())
                finally:
                    observed.close()
                    popped.close()

    def test_finds_route_on_empty_grid(self):
        with c_navgrid(width=5, height=5, mode=NavgridDirMode.DIR_8) as navgrid:
            with c_coord(0, 0) as start, c_coord(4, 4) as goal:
                with c_dstar_lite(navgrid, start) as finder:
                    finder.set_goal(goal)
                    route = finder.find()
                    self.assertIsNotNone(route)
                    try:
                        self.assertGreater(route.length(), 0)
                    finally:
                        route.close()

    def test_incremental_replan_stats_and_sticky_cancel(self):
        with c_navgrid(width=5, height=5, mode=NavgridDirMode.DIR_8) as navgrid:
            with c_coord(0, 0) as start, c_coord(4, 4) as goal:
                with c_dstar_lite(navgrid, start, goal) as finder:
                    route, stats = finder.replan()
                    try:
                        self.assertEqual(stats["last_status"], NavsysStatus.OK)
                        self.assertEqual(stats["replans"], 1)
                        first_step = route.coord_at(1)
                        with first_step:
                            finder.notify_edge_changes([
                                (start, first_step, 1.0, math.inf)
                            ])
                            repaired, repaired_stats = finder.replan()
                            try:
                                changed_step = repaired.coord_at(1)
                                try:
                                    self.assertNotEqual(
                                        (first_step.x, first_step.y),
                                        (changed_step.x, changed_step.y),
                                    )
                                finally:
                                    changed_step.close()
                                self.assertEqual(repaired_stats["edge_changes"], 1)
                                self.assertEqual(repaired_stats["replans"], 2)
                            finally:
                                repaired.close()
                    finally:
                        route.close()

                    finder.request_cancel()
                    self.assertTrue(finder.is_cancel_requested())
                    with self.assertRaises(NavsysCancelledError):
                        finder.replan()

    def test_callback_failure_and_no_path_are_distinct(self):
        with c_navgrid(width=3, height=3, mode=NavgridDirMode.DIR_8) as navgrid:
            with c_coord(0, 0) as start, c_coord(2, 2) as goal:
                with c_dstar_lite(
                    navgrid, start, goal,
                    cost_fn=lambda *_args: (_ for _ in ()).throw(ValueError()),
                ) as failing:
                    with self.assertRaises(NavsysCallbackError):
                        failing.replan()
                with c_dstar_lite(
                    navgrid, start, goal,
                    cost_fn=lambda *_args: math.inf,
                ) as blocked:
                    route, stats = blocked.replan()
                    self.assertIsNone(route)
                    self.assertEqual(stats["last_status"], NavsysStatus.NO_PATH)


if __name__ == "__main__":
    unittest.main()
