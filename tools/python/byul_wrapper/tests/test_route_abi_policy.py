import json
from pathlib import Path
import re
import sys
import unittest


REPOSITORY_ROOT = Path(__file__).resolve().parents[4]
WRAPPER_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WRAPPER_ROOT))

from byul_wrapper_generator import parse_header


HEADER = REPOSITORY_ROOT / "byul" / "navsys" / "route" / "route.h"
INVENTORY = (
    REPOSITORY_ROOT
    / "docs"
    / "ko"
    / "todo"
    / "navsys"
    / "route-abi-consumer-inventory.json"
)
POLICY = (
    REPOSITORY_ROOT
    / "docs"
    / "ko"
    / "todo"
    / "navsys"
    / "route-abi-policy.json"
)


def load_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


class RouteAbiPolicyTest(unittest.TestCase):
    def test_policy_covers_the_legacy_inventory_exactly(self):
        inventory = load_json(INVENTORY)
        policy = load_json(POLICY)
        legacy = {
            row["name"]
            for row in inventory["header"]["functions"]
            if row["surface"] == "legacy-abi-1"
        }
        rows = policy["symbols"]
        names = {row["name"] for row in rows}

        self.assertEqual(len(rows), len(names))
        self.assertEqual(policy["legacy_symbol_count"], 44)
        self.assertEqual(names, legacy)
        self.assertEqual(
            {row["disposition"] for row in rows},
            {"keep", "deprecate-forward"},
        )

    def test_header_annotations_match_the_policy_exactly(self):
        source = HEADER.read_text(encoding="utf-8")
        annotated = set(
            re.findall(
                r"BYUL_DEPRECATED\([^\n]*\)\s*"
                r"BYUL_API\s+[^;]*?\b([A-Za-z_]\w*)\s*\(",
                source,
                re.MULTILINE,
            )
        )
        policy = load_json(POLICY)
        deprecated = {
            row["name"]
            for row in policy["symbols"]
            if row["disposition"] == "deprecate-forward"
        }

        self.assertEqual(policy["deprecated_symbol_count"], 35)
        self.assertEqual(annotated, deprecated)

    def test_every_deprecation_has_a_target_or_explicit_boundary(self):
        declarations = {row.name for row in parse_header(HEADER)}
        policy = load_json(POLICY)
        for row in policy["symbols"]:
            if row["disposition"] != "deprecate-forward":
                self.assertTrue(row.get("reason"), row["name"])
                continue
            self.assertTrue(
                row.get("replacement") or row.get("no_replacement_reason"),
                row["name"],
            )
            for replacement in row.get("replacement", []):
                symbol = replacement.split("(", 1)[0]
                if symbol.startswith(("route_", "navsys_")):
                    self.assertIn(symbol, declarations, row["name"])

    def test_abi_2_boundary_never_changes_the_abi_1_header(self):
        policy = load_json(POLICY)
        boundary = policy["abi_2_boundary"]

        self.assertEqual(boundary["public_struct_body"], "internalize")
        self.assertIn("ABI-major", boundary["activation"])
        self.assertIn("never hides or removes", boundary["activation"])
        self.assertEqual(
            policy["result_value_fields"],
            ["coordinates", "completion", "cost"],
        )
        self.assertIn("NaN is unequal", policy["content_float_policy"])


if __name__ == "__main__":
    unittest.main()
