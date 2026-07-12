import unittest

from byul_wrapper.coord import c_coord
from byul_wrapper.dstar_lite import c_dstar_lite
from byul_wrapper.dstar_lite_key import c_dstar_lite_key
from byul_wrapper.dstar_lite_pqueue import c_dstar_lite_pqueue
from byul_wrapper.navgrid import NavgridDirMode, c_navgrid


class DStarLiteSupportTest(unittest.TestCase):
    def test_key_copy_and_comparison(self):
        with c_dstar_lite_key(1.0, 2.0) as key:
            copied = key.copy()
            try:
                self.assertEqual(key.to_tuple(), (1.0, 2.0))
                self.assertEqual(copied, key)
            finally:
                copied.close()

    def test_priority_queue_push_and_remove(self):
        with c_dstar_lite_pqueue() as queue:
            with c_dstar_lite_key(1.0, 2.0) as key, c_coord(3, 4) as coord:
                queue.push(key, coord)
                self.assertFalse(queue.is_empty())
                self.assertTrue(queue.contains(coord))
                self.assertTrue(queue.remove(coord))
                self.assertTrue(queue.is_empty())

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


if __name__ == "__main__":
    unittest.main()
