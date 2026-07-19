import json
from pathlib import Path
import re
import sys
import unittest


REPOSITORY_ROOT = Path(__file__).resolve().parents[4]
WRAPPER_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WRAPPER_ROOT))

from byul_wrapper_generator import parse_header


HEADER = REPOSITORY_ROOT / "byul" / "navsys" / "coord" / "coord.h"
MANIFEST = (
    REPOSITORY_ROOT
    / "docs"
    / "ko"
    / "todo"
    / "navsys"
    / "coord-abi-policy.json"
)
TODO = (
    REPOSITORY_ROOT
    / "docs"
    / "ko"
    / "todo"
    / "navsys"
    / "todo-navsys-coord-coord.org"
)
ALLOWED_DISPOSITIONS = {
    "keep",
    "add",
    "deprecate-forward",
    "abi-major",
    "internalize",
}
ALLOWED_FAILURE_POLICIES = {
    "preserve-all-outputs",
    "write-out_required-preserve-out_buffer",
}


def load_manifest() -> dict:
    return json.loads(MANIFEST.read_text(encoding="utf-8"))


class CoordAbiPolicyTest(unittest.TestCase):
    def test_current_header_inventory_matches_manifest_exactly(self):
        manifest = load_manifest()
        declarations = parse_header(HEADER)
        actual_functions = {declaration.name for declaration in declarations}
        expected_functions = set(manifest["current_functions"])

        self.assertEqual(len(declarations), len(actual_functions))
        self.assertEqual(actual_functions, expected_functions)
        self.assertTrue(
            set(manifest["current_functions"].values()) <= ALLOWED_DISPOSITIONS
        )

    def test_todo_disposition_table_matches_manifest_exactly(self):
        manifest = load_manifest()
        source = TODO.read_text(encoding="utf-8")
        table = source.split("* 현행 선언 전수 목록과 처분", 1)[1].split(
            "추가할 checked 선언", 1
        )[0]
        table_symbols = set(re.findall(r"^\|\s+=([^=]+)=\s+\|", table, re.MULTILINE))
        manifest_symbols = (
            set(manifest["current_constants"])
            | set(manifest["current_types"])
            | set(manifest["current_functions"])
        )

        self.assertEqual(table_symbols, manifest_symbols)

    def test_constants_and_value_layout_match_manifest(self):
        manifest = load_manifest()
        source = HEADER.read_text(encoding="utf-8")

        for name, contract in manifest["current_constants"].items():
            match = re.search(
                rf"^\s*#define\s+{re.escape(name)}\s+\(([-+]?\d+)\)",
                source,
                re.MULTILINE,
            )
            self.assertIsNotNone(match, name)
            self.assertEqual(int(match.group(1)), contract["value"])
            self.assertIn(contract["disposition"], ALLOWED_DISPOSITIONS)

        struct = re.search(
            r"typedef\s+struct\s+s_coord\s*\{(?P<body>.*?)\}\s*coord_t\s*;",
            source,
            re.DOTALL,
        )
        self.assertIsNotNone(struct)
        fields = [
            re.sub(r"\s+", " ", match.group(1)).strip()
            for match in re.finditer(
                r"^\s*([^/;]+?)\s*;",
                struct.group("body"),
                re.MULTILINE,
            )
        ]
        expected_fields = [
            field["declaration"]
            for field in manifest["current_types"]["coord_t"]["fields"]
        ]
        self.assertEqual(fields, expected_fields)

    def test_planned_checked_contracts_define_failure_output_policy(self):
        manifest = load_manifest()
        planned = manifest["planned_contracts"]

        self.assertEqual(len(planned), 14)
        for symbol, contract in planned.items():
            with self.subTest(symbol=symbol):
                self.assertTrue(contract["success_outputs"])
                self.assertTrue(contract["failures"])
                self.assertTrue(
                    set(contract["failures"].values())
                    <= ALLOWED_FAILURE_POLICIES
                )

        format_contract = planned["coord_format"]
        self.assertTrue(format_contract["required_size_includes_nul"])
        self.assertEqual(format_contract["encoding"], "utf-8")


if __name__ == "__main__":
    unittest.main()
