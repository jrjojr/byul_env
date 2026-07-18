import json
from pathlib import Path
import unittest


REPOSITORY_ROOT = Path(__file__).resolve().parents[4]
MANIFEST = (
    REPOSITORY_ROOT
    / "docs"
    / "ko"
    / "todo"
    / "navsys"
    / "navsys-abi-vocabulary.json"
)
COORD_POLICY = (
    REPOSITORY_ROOT
    / "docs"
    / "ko"
    / "todo"
    / "navsys"
    / "coord-abi-policy.json"
)


def load_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


class NavsysAbiVocabularyTest(unittest.TestCase):
    def test_status_values_are_unique_and_non_success_is_negative(self):
        status = load_json(MANIFEST)["status"]
        values = status["values"]

        self.assertEqual(values["NAVSYS_STATUS_OK"], 0)
        self.assertEqual(len(values), len(set(values.values())))
        self.assertTrue(
            all(value < 0 for name, value in values.items() if name != "NAVSYS_STATUS_OK")
        )

        classified = {
            name
            for names in status["classification"].values()
            for name in names
        }
        self.assertEqual(classified, set(values))

    def test_owner_and_include_dag_are_approved_but_not_materialized(self):
        owner = load_json(MANIFEST)["status"]["owner"]

        self.assertEqual(owner["planned_header"], "byul/navsys/navsys_status.h")
        self.assertEqual(owner["role"], "public-foundation")
        self.assertFalse(owner["materialized"])
        self.assertFalse((REPOSITORY_ROOT / owner["planned_header"]).exists())
        self.assertEqual(len(owner["include_dag"]), 3)

    def test_coord_policy_uses_the_common_status_values(self):
        common = load_json(MANIFEST)["status"]["values"]
        required = load_json(COORD_POLICY)["status_dependency"]["required_values"]

        self.assertEqual(
            required,
            {name: common[name] for name in required},
        )

    def test_namespace_policy_preserves_family_prefixes(self):
        namespace = load_json(MANIFEST)["c_symbol_namespace"]
        prefixes = namespace["canonical_family_prefixes"]

        self.assertEqual(namespace["policy"], "preserve-representative-family-prefix")
        self.assertEqual(len(prefixes), len(set(prefixes)))
        self.assertTrue(all(prefix.endswith("_") for prefix in prefixes))
        self.assertNotIn("navsys_", prefixes)
        self.assertNotIn("byul_navsys_", prefixes)

    def test_every_legacy_return_form_has_a_canonical_mapping(self):
        mappings = load_json(MANIFEST)["legacy_return_mapping"]
        required = {
            "nullable_pointer",
            "bool_predicate",
            "bool_operation",
            "int_zero_nonzero_status",
            "int_count",
            "int_index_minus_one",
            "void_fallible_operation",
            "enum_result",
            "floating_result",
            "callback_return",
        }

        self.assertEqual(set(mappings), required)
        for name, mapping in mappings.items():
            with self.subTest(name=name):
                self.assertTrue(mapping["current_meaning"])
                self.assertTrue(mapping["canonical"])


if __name__ == "__main__":
    unittest.main()
