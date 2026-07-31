import json
from pathlib import Path
import unittest


REPOSITORY_ROOT = Path(__file__).resolve().parents[4]
POLICY = (
    REPOSITORY_ROOT / "docs" / "ko" / "todo" / "navsys"
    / "navgrid-semantic-policy.json"
)
ABI_POLICY = (
    REPOSITORY_ROOT / "docs" / "ko" / "todo" / "navsys"
    / "navgrid-abi-policy.json"
)
TERRAIN_POLICY = (
    REPOSITORY_ROOT / "docs" / "ko" / "todo" / "navsys"
    / "navgrid-terrain-policy.json"
)


def load(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def circular_distance(lhs: float, rhs: float) -> float:
    difference = abs(lhs - rhs) % 360.0
    return min(difference, 360.0 - difference)


class NavgridSemanticPolicyTest(unittest.TestCase):
    def test_policy_links_and_query_statuses_are_closed(self):
        policy = load(POLICY)
        abi = load(ABI_POLICY)
        terrain = load(TERRAIN_POLICY)

        self.assertEqual(abi["semantic_policy"], "docs/ko/todo/navsys/navgrid-semantic-policy.json")
        self.assertEqual(policy["terrain_policy"], "docs/ko/todo/navsys/navgrid-terrain-policy.json")
        self.assertTrue(terrain["terrain_policy"]["TERRAIN_TYPE_FORBIDDEN"]["built_in_blocked"])
        statuses = policy["coordinate_query_result"]["status_matrix"]
        self.assertEqual(statuses["outside_bounded_extent"], "NAVSYS_STATUS_NOT_FOUND")
        self.assertIn("presence=absent", statuses["inside_absent"])
        self.assertIn("blocked=true", statuses["inside_present_blocked"])
        self.assertEqual(statuses["callback_non_ok_or_exception"], "NAVSYS_STATUS_CALLBACK_FAILED")
        self.assertEqual(statuses["invalid_stored_cell"], "NAVSYS_STATUS_CORRUPT_STATE")
        self.assertIn("preserve", policy["coordinate_query_result"]["failure_output"])

    def test_nonzero_origin_half_open_extents_are_exhaustive_on_tiny_grids(self):
        extent = load(POLICY)["canonical_extent"]
        self.assertEqual(extent["bounded"]["width_min"], 1)
        self.assertEqual(extent["bounded"]["height_min"], 1)
        for origin_x in (-2, 3):
            for origin_y in (-3, 2):
                for width in (1, 2, 3):
                    for height in (1, 2, 3):
                        for x in range(origin_x - 1, origin_x + width + 1):
                            for y in range(origin_y - 1, origin_y + height + 1):
                                inside = (
                                    origin_x <= x < origin_x + width
                                    and origin_y <= y < origin_y + height
                                )
                                expected = x in range(origin_x, origin_x + width) and y in range(origin_y, origin_y + height)
                                self.assertEqual(inside, expected)
        self.assertEqual(extent["unbounded"]["canonical_origin"], [0, 0])
        self.assertEqual(extent["unbounded"]["canonical_extent"], [0, 0])

    def test_neighbor_orders_are_unique_and_four_way_is_cardinal(self):
        orders = load(POLICY)["canonical_neighbor_order"]
        four = orders["NAVGRID_DIR_4"]
        eight = orders["NAVGRID_DIR_8"]

        self.assertEqual([item["angle"] for item in four], [0, 90, 180, 270])
        self.assertEqual([item["angle"] for item in eight], list(range(0, 360, 45)))
        self.assertEqual([(item["dx"], item["dy"]) for item in four], [(1, 0), (0, 1), (-1, 0), (0, -1)])
        self.assertEqual(len({(item["dx"], item["dy"]) for item in eight}), 8)
        self.assertTrue(all(abs(item["dx"]) + abs(item["dy"]) == 1 for item in four))
        self.assertFalse(orders["center_included"])

    def test_all_diagonal_corner_policies_cover_every_orthogonal_state(self):
        policies = load(POLICY)["diagonal_policies"]
        for name, rule in policies.items():
            if not name.startswith("NAVGRID_DIAGONAL_"):
                continue
            for first_open in (False, True):
                for second_open in (False, True):
                    if not rule["diagonal_enabled"]:
                        allowed = False
                    else:
                        allowed = (first_open + second_open) >= rule["minimum_open_orthogonals"]
                    if name == "NAVGRID_DIAGONAL_NEVER":
                        self.assertFalse(allowed)
                    elif name == "NAVGRID_DIAGONAL_ALWAYS":
                        self.assertTrue(allowed)
                    elif name.endswith("ONE_ORTHOGONAL_OPEN"):
                        self.assertEqual(allowed, first_open or second_open)
                    elif name.endswith("BOTH_ORTHOGONALS_OPEN"):
                        self.assertEqual(allowed, first_open and second_open)

    def test_degree_selection_uses_axis_rotation_and_deterministic_ties(self):
        policy = load(POLICY)
        orders = policy["canonical_neighbor_order"]
        self.assertEqual(policy["coordinate_system"]["angle_zero"], "+x")
        self.assertIn("atan2(+y,+x)", policy["coordinate_system"]["angle_positive"])
        for mode, probes in {
            "NAVGRID_DIR_4": {0: "E", 45: "E", 90: "S", 225: "W", 359: "E"},
            "NAVGRID_DIR_8": {0: "E", 22.5: "E", 45: "SE", 90: "S", 337.5: "E"},
        }.items():
            candidates = orders[mode]
            for requested, expected in probes.items():
                selected = min(
                    candidates,
                    key=lambda item: (circular_distance(requested, item["angle"]), item["angle"]),
                )
                self.assertEqual(selected["name"], expected)

    def test_resize_clip_is_atomic_and_never_restores_clipped_cells(self):
        resize = load(POLICY)["canonical_resize"]
        self.assertEqual(resize["policy"], "clip")
        cells = {(x, y) for x in range(-2, 4) for y in range(-2, 4)}
        origin_x, origin_y, width, height = -1, 0, 3, 2
        clipped = {
            cell for cell in cells
            if origin_x <= cell[0] < origin_x + width
            and origin_y <= cell[1] < origin_y + height
        }
        self.assertEqual(clipped, {(-1, 0), (-1, 1), (0, 0), (0, 1), (1, 0), (1, 1)})
        expanded = set(clipped)
        self.assertEqual(expanded, clipped)
        self.assertIn("preserves extent cells overlays and outputs", resize["failure"])

    def test_legacy_semantics_remain_separate_from_canonical_contract(self):
        policy = load(POLICY)
        legacy = policy["legacy_extent_characterization"]
        legacy_order = policy["legacy_neighbor_order"]
        self.assertIn("independently unbounded", legacy["zero_axis"])
        self.assertEqual(legacy_order["NAVGRID_DIR_4"], [[0, -1], [-1, 0], [1, 0], [0, 1]])
        self.assertEqual(len(legacy_order["NAVGRID_DIR_8"]), 8)
        self.assertIn("legacy-only", legacy["migration"])


if __name__ == "__main__":
    unittest.main()
