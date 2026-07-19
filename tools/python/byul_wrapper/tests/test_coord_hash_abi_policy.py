import json
from pathlib import Path
import re
import sys
import unittest


REPOSITORY_ROOT = Path(__file__).resolve().parents[4]
WRAPPER_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WRAPPER_ROOT))

from byul_wrapper_generator import parse_header


HEADER = REPOSITORY_ROOT / "byul" / "navsys" / "coord" / "coord_hash.h"
MANIFEST = (
    REPOSITORY_ROOT
    / "docs"
    / "ko"
    / "todo"
    / "navsys"
    / "coord-hash-abi-policy.json"
)
TODO = (
    REPOSITORY_ROOT
    / "docs"
    / "ko"
    / "todo"
    / "navsys"
    / "todo-navsys-coord-coord-hash.org"
)
ALLOWED_DISPOSITIONS = {
    "keep",
    "add",
    "deprecate-forward",
    "abi-major",
    "internalize",
}


def load_manifest() -> dict:
    return json.loads(MANIFEST.read_text(encoding="utf-8"))


def header_type_names(source: str) -> set[str]:
    opaque_types = set(
        re.findall(
            r"typedef\s+struct\s+\w+\s+(\w+)\s*;",
            source,
            re.MULTILINE,
        )
    )
    complete_struct_types = set(
        re.findall(
            r"typedef\s+struct\s+\w+\s*\{[^}]*\}\s*(\w+)\s*;",
            source,
            re.MULTILINE | re.DOTALL,
        )
    )
    callback_types = set(
        re.findall(
            r"typedef\s+[^;]*?\(\s*\*(\w+)\s*\)\s*\([^;]*\)\s*;",
            source,
            re.MULTILINE | re.DOTALL,
        )
    )
    return opaque_types | complete_struct_types | callback_types


class CoordHashAbiPolicyTest(unittest.TestCase):
    def test_current_header_inventory_matches_manifest_exactly(self):
        manifest = load_manifest()
        source = HEADER.read_text(encoding="utf-8")
        declarations = parse_header(HEADER)
        actual_functions = {declaration.name for declaration in declarations}
        actual_types = header_type_names(source)

        self.assertEqual(len(declarations), len(actual_functions))
        self.assertEqual(actual_functions, set(manifest["current_functions"]))
        self.assertEqual(actual_types, set(manifest["current_types"]))
        self.assertEqual(
            manifest["declaration_count"],
            len(actual_functions) + len(actual_types),
        )
        dispositions = (
            set(manifest["current_functions"].values())
            | set(manifest["current_types"].values())
        )
        self.assertTrue(dispositions <= ALLOWED_DISPOSITIONS)

    def test_todo_disposition_table_matches_manifest_exactly(self):
        manifest = load_manifest()
        source = TODO.read_text(encoding="utf-8")
        table = source.split("* 현행 선언 전수 목록과 처분", 1)[1].split(
            "새 create-info", 1
        )[0]
        table_symbols = set(
            re.findall(r"^\|\s+=([^=]+)=\s+\|", table, re.MULTILINE)
        )
        manifest_symbols = (
            {
                name
                for name, disposition in manifest["current_types"].items()
                if disposition != "add"
            }
            | {
                name
                for name, disposition in manifest["current_functions"].items()
                if disposition != "add"
            }
        )

        self.assertEqual(table_symbols, manifest_symbols)
        self.assertEqual(
            manifest["legacy_declaration_count"],
            len(manifest_symbols),
        )

    def test_legacy_operation_matrix_covers_every_ownership_transition(self):
        manifest = load_manifest()
        matrix = manifest["legacy_operation_matrix"]

        self.assertEqual(
            set(matrix),
            {
                "coord_hash_create",
                "coord_hash_create_full",
                "coord_hash_copy",
                "coord_hash_set",
                "coord_hash_insert",
                "coord_hash_replace",
                "coord_hash_remove",
                "coord_hash_clear",
                "coord_hash_destroy",
            },
        )
        self.assertEqual(
            matrix["coord_hash_set"]["stored_value"],
            "exact-input-pointer",
        )
        self.assertEqual(
            matrix["coord_hash_set"]["duplicate"],
            "overwrite-without-destroying-old-value",
        )
        self.assertEqual(
            matrix["coord_hash_insert"]["duplicate"],
            "destroy-old-then-copy-upsert",
        )
        self.assertEqual(
            matrix["coord_hash_replace"]["missing"],
            "copy-insert",
        )

    def test_unsafe_set_is_explicitly_legacy_only(self):
        decisions = load_manifest()["compatibility_decisions"]

        self.assertEqual(decisions["coord_hash_set"]["abi_1"], "legacy-only")
        self.assertEqual(
            decisions["coord_hash_set"]["canonical_forwarding"],
            "impossible-without-changing-ownership",
        )
        self.assertEqual(
            decisions["coord_hash_insert"]["abi_1_semantic_name"],
            "copy-upsert",
        )
        self.assertEqual(
            decisions["coord_hash_replace"]["abi_1_semantic_name"],
            "copy-upsert",
        )

    def test_canonical_mutation_contracts_preserve_failure_outputs(self):
        contracts = load_manifest()["canonical_mutation_contracts"]

        self.assertEqual(contracts["create_info_abi_version"], 1)
        self.assertEqual(
            contracts["callback_binding"],
            "immutable-function-userdata-pair",
        )
        self.assertEqual(
            contracts["coord_hash_copy_ex"]["missing_copy_callback"],
            "NAVSYS_STATUS_UNSUPPORTED",
        )
        self.assertEqual(
            contracts["coord_hash_insert_copy"]["duplicate"],
            "NAVSYS_STATUS_INVALID_ARGUMENT",
        )
        self.assertEqual(
            contracts["coord_hash_replace_copy"]["missing"],
            "NAVSYS_STATUS_NOT_FOUND",
        )
        self.assertEqual(
            contracts["coord_hash_upsert_copy"]["failure"],
            "preserve-table-and-out-inserted",
        )


if __name__ == "__main__":
    unittest.main()
