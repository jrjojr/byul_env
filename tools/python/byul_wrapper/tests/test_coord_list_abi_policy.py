import json
from pathlib import Path
import re
import sys
import unittest


REPOSITORY_ROOT = Path(__file__).resolve().parents[4]
WRAPPER_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WRAPPER_ROOT))

from byul_wrapper_generator import parse_header


HEADER = REPOSITORY_ROOT / "byul" / "navsys" / "coord" / "coord_list.h"
MANIFEST = (
    REPOSITORY_ROOT
    / "docs"
    / "ko"
    / "todo"
    / "navsys"
    / "coord-list-abi-policy.json"
)
TODO = (
    REPOSITORY_ROOT
    / "docs"
    / "ko"
    / "todo"
    / "navsys"
    / "todo-navsys-coord-coord-list.org"
)
ALLOWED_DISPOSITIONS = {"keep", "add", "deprecate-forward", "abi-major"}


def load_manifest() -> dict:
    return json.loads(MANIFEST.read_text(encoding="utf-8"))


class CoordListAbiPolicyTest(unittest.TestCase):
    def test_current_header_inventory_matches_manifest_exactly(self):
        manifest = load_manifest()
        source = HEADER.read_text(encoding="utf-8")
        declarations = parse_header(HEADER)
        actual_functions = {declaration.name for declaration in declarations}
        actual_types = set(
            re.findall(
                r"typedef\s+struct\s+\w+\s+(\w+)\s*;",
                source,
                re.MULTILINE,
            )
        )

        self.assertEqual(len(declarations), len(actual_functions))
        self.assertEqual(actual_functions, set(manifest["current_functions"]))
        self.assertEqual(actual_types, set(manifest["current_types"]))
        legacy_functions = {
            name
            for name, disposition in manifest["current_functions"].items()
            if disposition != "add"
        }
        self.assertEqual(
            manifest["legacy_declaration_count"],
            len(legacy_functions) + len(actual_types),
        )
        dispositions = (
            set(manifest["current_functions"].values())
            | set(manifest["current_types"].values())
            | set(manifest["canonical_additions"].values())
        )
        self.assertTrue(dispositions <= ALLOWED_DISPOSITIONS)

    def test_todo_disposition_table_matches_manifest_exactly(self):
        source = TODO.read_text(encoding="utf-8")
        table = source.split("* 현행 선언 전수 목록과 처분", 1)[1].split(
            "추가할 status/size_t", 1
        )[0]
        table_symbols = set(
            re.findall(r"^\|\s+=([^=]+)=\s+\|", table, re.MULTILINE)
        )
        manifest = load_manifest()
        manifest_symbols = set(manifest["current_types"]) | {
            name
            for name, disposition in manifest["current_functions"].items()
            if disposition != "add"
        }

        self.assertEqual(table_symbols, manifest_symbols)

    def test_every_deprecated_function_has_a_migration_target(self):
        manifest = load_manifest()
        deprecated = {
            name
            for name, disposition in manifest["current_functions"].items()
            if disposition == "deprecate-forward"
        }

        self.assertEqual(deprecated, set(manifest["migration"]))
        canonical = set(manifest["canonical_additions"])
        self.assertTrue(set(manifest["migration"].values()) <= canonical)

    def test_legacy_characterization_separates_valid_zero_from_empty(self):
        behavior = load_manifest()["legacy_characterization"]

        self.assertEqual(behavior["null_receiver"]["pop_front_back"], [0, 0])
        self.assertEqual(behavior["null_receiver"]["find"], -1)
        self.assertEqual(
            behavior["duplicates"]["remove_value"], "remove-first-only"
        )
        self.assertEqual(behavior["range"]["sublist"], "[start,end)")
        self.assertEqual(behavior["range"]["empty"], "null")
        self.assertEqual(
            behavior["borrowed_lifetime"]["get_front_back"],
            "until-next-non-const-operation-or-destroy",
        )

    def test_canonical_decisions_are_unambiguous(self):
        decisions = load_manifest()["canonical_decisions"]

        self.assertEqual(
            decisions["empty_pop_status"], "NAVSYS_STATUS_NOT_FOUND"
        )
        self.assertEqual(
            decisions["remove_value"],
            "remove-first-only-with-out-removed",
        )
        self.assertEqual(
            decisions["slice_range"], "[begin,end)-including-empty"
        )
        self.assertEqual(decisions["canonical_index_type"], "size_t")
        self.assertEqual(
            decisions["failure_guarantee"],
            "preserve-list-and-all-out-parameters",
        )


if __name__ == "__main__":
    unittest.main()
