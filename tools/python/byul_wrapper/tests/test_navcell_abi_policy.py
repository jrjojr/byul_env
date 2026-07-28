import json
from pathlib import Path
import re
import sys
import unittest


REPOSITORY_ROOT = Path(__file__).resolve().parents[4]
WRAPPER_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WRAPPER_ROOT))

from byul_wrapper_generator import parse_header


HEADER = REPOSITORY_ROOT / "byul" / "navsys" / "navgrid" / "navcell.h"
SOURCE = REPOSITORY_ROOT / "byul" / "navsys" / "navgrid" / "navcell.cpp"
NAVGRID_SOURCE = REPOSITORY_ROOT / "byul" / "navsys" / "navgrid" / "navgrid.cpp"
MANIFEST = (
    REPOSITORY_ROOT / "docs" / "ko" / "todo" / "navsys"
    / "navcell-abi-policy.json"
)
WRAPPER = WRAPPER_ROOT / "byul_wrapper" / "navgrid.py"
ALLOWED_DISPOSITIONS = {"keep", "deprecate-forward", "abi-major"}


def load_manifest() -> dict:
    return json.loads(MANIFEST.read_text(encoding="utf-8"))


class NavcellAbiPolicyTest(unittest.TestCase):
    def test_current_header_function_inventory_matches_manifest_exactly(self):
        manifest = load_manifest()
        declarations = parse_header(HEADER)
        actual = {declaration.name for declaration in declarations}

        self.assertEqual(len(declarations), len(actual))
        self.assertEqual(actual, set(manifest["current_functions"]))
        self.assertTrue(
            set(manifest["current_functions"].values()) <= ALLOWED_DISPOSITIONS
        )
        self.assertTrue(
            actual.isdisjoint(manifest["forbidden_stage_one_symbols"])
        )

    def test_enum_values_and_anonymous_value_layout_match_manifest(self):
        manifest = load_manifest()
        source = HEADER.read_text(encoding="utf-8")
        enum = re.search(
            r"typedef\s+enum\s+e_terrain_type\s*\{(?P<body>.*?)\}"
            r"\s*terrain_type_t\s*;",
            source,
            re.DOTALL,
        )
        self.assertIsNotNone(enum)

        actual_values = {}
        next_value = 0
        for name, explicit in re.findall(
            r"^\s*(TERRAIN_TYPE_[A-Z_]+)\s*(?:=\s*(-?\d+))?\s*,?\s*$",
            enum.group("body"),
            re.MULTILINE,
        ):
            if explicit:
                next_value = int(explicit)
            actual_values[name] = next_value
            next_value += 1
        self.assertEqual(
            actual_values,
            manifest["current_enums"]["terrain_type_t"]["values"],
        )

        struct = re.search(
            r"typedef\s+struct\s*\{(?P<body>.*?)\}\s*navcell_t\s*;",
            source,
            re.DOTALL,
        )
        self.assertIsNotNone(struct)
        self.assertNotRegex(source, r"typedef\s+struct\s+s_navcell")
        self.assertIsNone(manifest["current_types"]["navcell_t"]["tag"])
        fields = [
            re.sub(r"\s+", " ", match.group(1)).strip()
            for match in re.finditer(
                r"^\s*([^/;]+?)\s*;", struct.group("body"), re.MULTILINE
            )
        ]
        expected = [
            field["declaration"]
            for field in manifest["current_types"]["navcell_t"]["fields"]
        ]
        self.assertEqual(fields, expected)

    def test_generated_cffi_matches_current_header(self):
        generated = WRAPPER.read_text(encoding="utf-8").split(
            "/* Source: byul/navsys/navgrid/navcell.h */", 1
        )[1].split("/* Source: byul/navsys/navgrid/navgrid.h */", 1)[0]
        manifest = load_manifest()

        for symbol in manifest["current_enums"]["terrain_type_t"]["values"]:
            self.assertRegex(generated, rf"\b{re.escape(symbol)}\b")
        for symbol in manifest["current_functions"]:
            self.assertRegex(generated, rf"\b{re.escape(symbol)}\s*\(")
        for symbol in manifest["forbidden_stage_one_symbols"]:
            self.assertNotRegex(generated, rf"\b{re.escape(symbol)}\s*\(")
        self.assertRegex(generated, r"typedef\s+struct\s*\{")
        self.assertRegex(generated, r"terrain_type_t\s+terrain\s*;")
        self.assertRegex(generated, r"int\s+height\s*;")

    def test_legacy_failure_and_validation_are_characterized_not_fixed(self):
        source = SOURCE.read_text(encoding="utf-8")
        semantics = load_manifest()["legacy_semantics"]

        self.assertIn("new navcell_t{}", source)
        self.assertNotRegex(source, r"\btry\b|\bcatch\b")
        self.assertIn("may escape the C ABI", semantics["allocation_failure"])
        self.assertEqual(semantics["validation"], {"terrain": "none", "height": "none"})
        self.assertIn("full C int range", semantics["height_range"])

    def test_current_navgrid_semantics_and_consumer_inventory_are_recorded(self):
        manifest = load_manifest()
        source = NAVGRID_SOURCE.read_text(encoding="utf-8")
        policy = manifest["legacy_semantics"]["navgrid_policy"]

        self.assertRegex(
            source,
            r"out\.terrain\s*==\s*TERRAIN_TYPE_FORBIDDEN",
        )
        self.assertEqual(policy["built_in_blocked"], ["TERRAIN_TYPE_FORBIDDEN"])
        self.assertIn("route cost callbacks", policy["built_in_cost"])
        for paths in manifest["consumer_inventory"].values():
            if isinstance(paths, list):
                for path in paths:
                    self.assertTrue((REPOSITORY_ROOT / path).is_file(), path)
        self.assertIn("No navcell_t serializer", manifest["consumer_inventory"]["serialization"])

    def test_stage_two_contracts_are_plans_not_current_symbols(self):
        manifest = load_manifest()
        planned = manifest["planned_contracts"]

        self.assertEqual(
            planned["terrain_validation"]["supported_values"],
            [0, 1, 2, 3, 100],
        )
        self.assertEqual(
            planned["terrain_validation"]["unknown_validation"],
            "UNSUPPORTED",
        )
        self.assertIn(
            "no output mutation",
            planned["checked_create"]["allocation_failure"],
        )


if __name__ == "__main__":
    unittest.main()
