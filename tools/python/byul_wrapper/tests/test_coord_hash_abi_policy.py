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
COORD_CMAKE = REPOSITORY_ROOT / "byul" / "navsys" / "coord" / "CMakeLists.txt"
NAVSYS_CMAKE = REPOSITORY_ROOT / "byul" / "navsys" / "CMakeLists.txt"
ROOT_CMAKE = REPOSITORY_ROOT / "byul" / "CMakeLists.txt"
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

    def test_canonical_query_contracts_separate_observable_results(self):
        contracts = load_manifest()["canonical_query_contracts"]

        self.assertEqual(
            contracts["coord_hash_find"]["stored_null"],
            "NAVSYS_STATUS_OK-found-true-value-null",
        )
        self.assertEqual(
            contracts["coord_hash_find"]["not_found"],
            "NAVSYS_STATUS_OK-found-false-value-null",
        )
        self.assertEqual(
            contracts["coord_hash_iter_next_ex"]["end"],
            "NAVSYS_STATUS_NOT_FOUND",
        )
        self.assertEqual(
            contracts["coord_hash_iter_next_ex"][
                "parent_mutation_or_destroy"
            ],
            "NAVSYS_STATUS_INVALIDATED",
        )
        self.assertEqual(
            contracts["caller_buffer"]["short"],
            "write-required-preserve-buffer-return-incomplete",
        )
        self.assertEqual(
            contracts["coord_hash_equal_full"]["failure"],
            "preserve-out-equal",
        )

    def test_static_and_root_targets_share_the_approved_coord_inventory(self):
        manifest = load_manifest()
        inventory = manifest["build_inventory"]
        coord_cmake = COORD_CMAKE.read_text(encoding="utf-8")
        navsys_cmake = NAVSYS_CMAKE.read_text(encoding="utf-8")
        root_cmake = ROOT_CMAKE.read_text(encoding="utf-8")

        for path in inventory["sources"] + inventory["headers"]:
            relative = path.split("byul/navsys/coord/", 1)[1]
            self.assertEqual(
                coord_cmake.count(f"${{CMAKE_CURRENT_SOURCE_DIR}}/{relative}"),
                1,
            )
        self.assertIn("${coord_SOURCES}", navsys_cmake)
        self.assertIn("${navsys_SOURCES}", root_cmake)
        self.assertRegex(
            coord_cmake,
            r"target_compile_definitions\(\$\{PROJECT_NAME\}\s+PRIVATE\s+"
            + inventory["module_compile_definition"],
        )
        self.assertRegex(
            root_cmake,
            r"target_compile_definitions\(\$\{PROJECT_NAME\}\s+PRIVATE\s+"
            + inventory["root_compile_definition"],
        )

    def test_abi_fingerprint_covers_every_added_function_and_abi2_decision(self):
        manifest = load_manifest()
        fingerprint = manifest["abi_fingerprint"]
        added_functions = {
            name
            for name, disposition in manifest["current_functions"].items()
            if disposition == "add"
        }

        self.assertEqual(set(fingerprint["new_symbols"]), added_functions)
        self.assertEqual(
            fingerprint["coord_hash_create_info_t"]["x64_offsets"],
            [0, 4, 8, 16, 24, 32],
        )
        self.assertEqual(
            fingerprint["coord_hash_entry_view_t"]["x64_offsets"],
            [0, 8],
        )
        self.assertEqual(
            set(fingerprint["callback_types"]),
            {
                "coord_hash_value_copy_func_ex",
                "coord_hash_value_destroy_func_ex",
                "coord_hash_value_equal_func_ex",
            },
        )

        decision = manifest["abi_2_decision"]
        self.assertEqual(decision["status"], "approved")
        self.assertEqual(decision["public_generic_coord_hash"], "retire")
        self.assertEqual(
            decision["replacement"], "domain-specific-typed-map-families"
        )
        self.assertTrue(
            any("coord_hash_set" in step for step in decision["migration"])
        )


if __name__ == "__main__":
    unittest.main()
