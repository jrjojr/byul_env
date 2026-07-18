import importlib.util
from pathlib import Path
import unittest


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
MODULE_PATH = REPOSITORY_ROOT / "tools/navsys_abi_inventory.py"
SPEC = importlib.util.spec_from_file_location("navsys_abi_inventory", MODULE_PATH)
assert SPEC and SPEC.loader
MODULE = importlib.util.module_from_spec(SPEC)
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
        self.assertEqual(43, self.inventory["summary"]["headers"])
        self.assertEqual(
            43,
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


if __name__ == "__main__":
    unittest.main()
