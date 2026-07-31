import gc
import unittest
import weakref

from byul_wrapper.coord import c_coord
from byul_wrapper.ffi_core import C, ffi
from byul_wrapper.navgrid import c_navgrid, NavgridDirMode
from byul_wrapper.navsys_status import NavsysStatus


class TestNavcellCheckedAbi(unittest.TestCase):
    def test_checked_create_validate_copy_round_trip(self):
        supported = ffi.new("bool*")
        self.assertEqual(
            C.navcell_is_terrain_supported(2, supported),
            NavsysStatus.OK,
        )
        self.assertTrue(supported[0])

        output = ffi.new("navcell_t**")
        self.assertEqual(
            C.navcell_create_checked(2, 2147483647, output),
            NavsysStatus.OK,
        )
        self.assertNotEqual(output[0], ffi.NULL)
        cell = output[0]
        try:
            self.assertEqual(C.navcell_validate(cell), NavsysStatus.OK)
            self.assertEqual(cell.terrain, 2)
            self.assertEqual(cell.height, 2147483647)

            copied_output = ffi.new("navcell_t**")
            self.assertEqual(
                C.navcell_copy_checked(cell, copied_output),
                NavsysStatus.OK,
            )
            copied = copied_output[0]
            try:
                self.assertNotEqual(copied, ffi.NULL)
                self.assertNotEqual(copied, cell)
                self.assertEqual(copied.terrain, cell.terrain)
                self.assertEqual(copied.height, cell.height)
            finally:
                C.navcell_destroy(copied)
        finally:
            C.navcell_destroy(cell)

    def test_unknown_terrain_is_queryable_but_not_constructible(self):
        supported = ffi.new("bool*", True)
        self.assertEqual(
            C.navcell_is_terrain_supported(101, supported),
            NavsysStatus.OK,
        )
        self.assertFalse(supported[0])

        output = ffi.new("navcell_t**")
        self.assertEqual(
            C.navcell_create_checked(101, 0, output),
            NavsysStatus.UNSUPPORTED,
        )
        self.assertEqual(output[0], ffi.NULL)


class TestNavgridTerrainPolicy(unittest.TestCase):
    def test_blocked_coords_uses_materialized_cell_snapshot(self):
        grid = c_navgrid(width=8, height=8, mode=NavgridDirMode.DIR_4)
        try:
            grid.set_cell_checked(5, 1, 2, 17)
            overlay, changed = grid.apply_blocked_overlay([(5, 1), (3, 3)])
            self.assertEqual(changed, 2)
            entries = grid.cell_entries()
            self.assertEqual(
                entries,
                [
                    (3, 3, 0, 0, False, True),
                    (5, 1, 2, 17, True, True),
                ],
            )
            blocked = grid.blocked_coords()
            try:
                self.assertEqual(
                    [(coord.x, coord.y) for coord in blocked],
                    [(3, 3), (5, 1)],
                )
            finally:
                blocked.close()
            self.assertEqual(grid.remove_blocked_overlay(overlay), 2)
        finally:
            grid.close()

    def test_copy_retains_python_callback_after_source_finalization(self):
        calls = []

        def blocked(grid, x, y, userdata):
            calls.append((x, y))
            return (x, y) == (2, 1)

        callback_ref = weakref.ref(blocked)
        source = c_navgrid(
            width=3,
            height=3,
            mode=NavgridDirMode.DIR_4,
            py_func=blocked,
            own=True,
        )
        copied = source.copy()
        del blocked
        source.close()
        del source
        gc.collect()

        self.assertIsNotNone(callback_ref())
        neighbors = copied.neighbors(1, 1)
        try:
            coords = {(coord.x, coord.y) for coord in neighbors}
        finally:
            neighbors.close()
        self.assertNotIn((2, 1), coords)
        self.assertTrue(calls)

        copied.set_is_coord_blocked_fn(None)
        gc.collect()
        self.assertIsNone(callback_ref())
        self.assertIsNone(copied.get_is_coord_blocked_fn())
        copied.close()

    def test_block_unblock_restores_sparse_default(self):
        grid = c_navgrid()
        try:
            self.assertTrue(grid.block(4, 4))
            self.assertTrue(grid.is_blocked(4, 4))
            self.assertTrue(grid.unblock(4, 4))
            self.assertFalse(grid.is_blocked(4, 4))

            fetched = ffi.new("navcell_t*")
            self.assertEqual(C.navgrid_fetch_cell(grid._c, 4, 4, fetched), -1)
        finally:
            grid.close()

    def test_block_refuses_to_overwrite_user_cell(self):
        grid = c_navgrid()
        try:
            user_cell = ffi.new("navcell_t*", {"terrain": 2, "height": 73})
            self.assertTrue(C.navgrid_set_cell(grid._c, 4, 4, user_cell))
            self.assertFalse(grid.block(4, 4))
            self.assertFalse(grid.unblock(4, 4))

            fetched = ffi.new("navcell_t*")
            self.assertEqual(C.navgrid_fetch_cell(grid._c, 4, 4, fetched), 0)
            self.assertEqual(fetched.terrain, 2)
            self.assertEqual(fetched.height, 73)
        finally:
            grid.close()

    def test_checked_overlay_preserves_custom_cell_and_source_order(self):
        grid = c_navgrid(
            width=8,
            height=8,
            mode=NavgridDirMode.DIR_4,
            own=True,
        )
        try:
            prior, had_prior, changed = grid.set_cell_checked(3, 4, 2, 73)
            self.assertEqual(prior, (0, 0))
            self.assertFalse(had_prior)
            self.assertTrue(changed)

            first, first_changed = grid.apply_blocked_overlay([(3, 4), (5, 6)])
            second, second_changed = grid.apply_blocked_overlay([(3, 4), (5, 6)])
            self.assertEqual(first_changed, 2)
            self.assertEqual(second_changed, 0)
            self.assertTrue(grid.is_blocked(3, 4))
            self.assertEqual(grid.remove_blocked_overlay(first), 0)
            self.assertTrue(grid.is_blocked(3, 4))
            self.assertEqual(grid.remove_blocked_overlay(second), 2)
            self.assertFalse(grid.is_blocked(3, 4))
            self.assertEqual(grid.fetch_cell_checked(3, 4), (True, 2, 73))

            self.assertTrue(grid.block_checked(3, 4))
            self.assertFalse(grid.block_checked(3, 4))
            self.assertTrue(grid.unblock_checked(3, 4))
            self.assertEqual(grid.fetch_cell_checked(3, 4), (True, 2, 73))
        finally:
            grid.close()

class TestMapMakeAtDegree(unittest.TestCase):
    def setUp(self):
        self.navgrid = c_navgrid(width=5, height=5, mode=NavgridDirMode.DIR_8)

    def tearDown(self):
        self.navgrid.close()

    def test_neighbor_at_degree(self):
        neighbor = self.navgrid.neighbor_at_degree(2, 2, 0.0)  # 0도: →
        self.assertEqual((neighbor.x, neighbor.y), (3, 2))

    def test_clone_neighbor_at_goal(self):
        center = c_coord(2, 2)
        goal = c_coord(4, 1)  # ↗ 방향
        neighbor = self.navgrid.neighbor_at_goal(center, goal)
        self.assertEqual((neighbor.x, neighbor.y), (3, 1))

    def test_neighbors_at_degree_range(self):
        center = c_coord(2, 2)
        goal = c_coord(4, 2)  # → 방향

        neighbors = self.navgrid.neighbors_at_degree_range(
            center, goal, -45.0, 45.0, 1
        )

        # 명시적으로 c_coord로 감싸기
        coords = []
        for ptr in neighbors:
            coord_obj = ptr
            coords.append((coord_obj.x, coord_obj.y))

        result = sorted(coords)
        expected = sorted([(3, 1), (3, 2), (3, 3)])
        self.assertEqual(result, expected)

# 🔽 여기서부터 직접 실행 시 동작
if __name__ == '__main__':
    unittest.main()
