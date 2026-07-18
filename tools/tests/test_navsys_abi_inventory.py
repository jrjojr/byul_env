import copy
import importlib.util
from pathlib import Path
import sys
import unittest


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
MODULE_PATH = REPOSITORY_ROOT / "tools/navsys_abi_inventory.py"
SPEC = importlib.util.spec_from_file_location("navsys_abi_inventory", MODULE_PATH)
assert SPEC and SPEC.loader
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class NavsysAbiInventoryTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.vocabulary = MODULE.load_json(MODULE.DEFAULT_VOCABULARY)
        cls.inventory = MODULE.build_inventory(
            MODULE.load_json(MODULE.DEFAULT_ROLE_MANIFEST),
            cls.vocabulary,
        )

    def test_all_approved_navsys_headers_are_present(self):
        self.assertEqual(46, self.inventory["summary"]["headers"])
        self.assertEqual(
            46,
            len({header["path"] for header in self.inventory["headers"]}),
        )
        for header in self.inventory["headers"]:
            with self.subTest(header=header["path"]):
                self.assertTrue(
                    (REPOSITORY_ROOT / header["owner_todo"]).is_file()
                )

    def test_every_exported_declaration_has_a_mapping(self):
        symbols = self.inventory["symbols"]
        self.assertGreater(len(symbols), 0)
        self.assertEqual(0, self.inventory["summary"]["unclassified"])
        for symbol in symbols:
            with self.subTest(symbol=symbol["name"]):
                mapping = self.vocabulary["legacy_return_mapping"][
                    symbol["return_form"]
                ]
                self.assertEqual(mapping["canonical"], symbol["canonical"])
                self.assertTrue(symbol["signature"].endswith(";"))

    def test_header_path_and_signature_mutations_are_distinguished(self):
        missing_header = copy.deepcopy(self.inventory)
        removed = missing_header["headers"].pop()
        findings = MODULE.inventory_findings(self.inventory, missing_header)
        self.assertIn(
            ("missing-header", removed["path"]),
            {(row.code, row.subject) for row in findings},
        )

        wrong_path = copy.deepcopy(self.inventory)
        wrong_path["headers"][0]["path"] = "byul/navsys/wrong-path.h"
        findings = MODULE.inventory_findings(self.inventory, wrong_path)
        codes = {row.code for row in findings}
        self.assertEqual({"missing-header", "unexpected-header"}, codes)

        changed_signature = copy.deepcopy(self.inventory)
        changed = changed_signature["symbols"][0]
        changed["signature"] = changed["signature"].replace(");", ", int mode);")
        findings = MODULE.inventory_findings(self.inventory, changed_signature)
        self.assertEqual(
            [("signature-mismatch", f"{changed['header']}:{changed['name']}")],
            [(row.code, row.subject) for row in findings],
        )

        missing_symbol = copy.deepcopy(self.inventory)
        removed_symbol = missing_symbol["symbols"].pop()
        findings = MODULE.inventory_findings(self.inventory, missing_symbol)
        self.assertEqual(
            [(
                "missing-symbol",
                f"{removed_symbol['header']}:{removed_symbol['name']}",
            )],
            [(row.code, row.subject) for row in findings],
        )

    def test_duplicate_declarations_are_explicit_debt(self):
        duplicates = self.inventory["known_debt"]["duplicate_declarations"]

        self.assertEqual(
            [
                {
                    "header": "byul/navsys/obstacle/obstacle_core.h",
                    "symbol": "obstacle_clear",
                    "count": 2,
                    "disposition": "route-to-owner-child",
                }
            ],
            duplicates,
        )
        self.assertEqual(
            len(duplicates),
            self.inventory["summary"]["duplicate_declarations"],
        )

    def test_every_header_audit_issue_routes_to_an_owner_child(self):
        debt = self.inventory["known_debt"]["header_audit"]

        self.assertGreater(len(debt), 0)
        self.assertEqual(len(debt), self.inventory["summary"]["audit_issues"])
        for issue in debt:
            with self.subTest(header=issue["header"], code=issue["code"]):
                self.assertEqual("route-to-owner-child", issue["disposition"])
                self.assertTrue((REPOSITORY_ROOT / issue["owner_todo"]).is_file())
                self.assertTrue(issue["code"])
                self.assertTrue(issue["message"])

    def test_classifier_distinguishes_predicate_index_and_count(self):
        self.assertEqual(
            "bool_predicate",
            MODULE.classify_return("coord_hash_contains", "bool"),
        )
        self.assertEqual(
            "int_index_minus_one",
            MODULE.classify_return("coord_list_find", "int"),
        )
        self.assertEqual(
            "int_count",
            MODULE.classify_return("coord_list_length", "int"),
        )

    def test_lifecycle_inventory_preserves_resource_operations(self):
        resources = {
            row["resource"]: row
            for row in self.inventory["lifecycle"]["resources"]
        }

        self.assertIn("coord", resources)
        self.assertEqual(
            {"create", "copy", "init", "destroy"},
            set(resources["coord"]["operation_kinds"]),
        )
        self.assertIn("route_finder", resources)
        self.assertEqual(
            {"create", "copy", "init", "free", "destroy"},
            set(resources["route_finder"]["operation_kinds"]),
        )

    def test_every_create_family_has_a_destroy_operation(self):
        for resource in self.inventory["lifecycle"]["resources"]:
            kinds = set(resource["operation_kinds"])
            if "create" not in kinds:
                continue
            with self.subTest(resource=resource["resource"]):
                self.assertIn("destroy", kinds)

    def test_split_callback_bindings_are_explicit(self):
        bindings = {
            (row["family"], row["slot"]): row
            for row in self.inventory["callback_bindings"]
        }

        cost = bindings[("route_finder", "cost_callback")]
        self.assertEqual(
            ["function", "userdata"],
            cost["setter_components"],
        )
        self.assertEqual(
            "split-function-and-userdata-setters",
            cost["current_atomicity"],
        )
        self.assertEqual(
            "partial-export-surface",
            bindings[("navgrid", "is_coord_blocked_callback")][
                "current_atomicity"
            ],
        )

    def test_lifecycle_and_callback_summary_counts_match_rows(self):
        summary = self.inventory["summary"]

        self.assertEqual(
            len(self.inventory["lifecycle"]["resources"]),
            summary["lifecycle_resources"],
        )
        self.assertEqual(
            len(self.inventory["lifecycle"]["operations"]),
            summary["lifecycle_operations"],
        )
        self.assertEqual(
            len(self.inventory["callback_bindings"]),
            summary["callback_bindings"],
        )


if __name__ == "__main__":
    unittest.main()
