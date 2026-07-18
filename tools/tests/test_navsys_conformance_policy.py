import copy
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
MODULE_PATH = REPOSITORY_ROOT / "tools/navsys_conformance_policy.py"
SPEC = importlib.util.spec_from_file_location(
    "navsys_conformance_policy",
    MODULE_PATH,
)
assert SPEC and SPEC.loader
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def load_json(path):
    return json.loads(path.read_text(encoding="utf-8"))


class NavsysConformancePolicyTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.policy = load_json(MODULE.POLICY_PATH)
        cls.inventory = load_json(MODULE.INVENTORY_PATH)
        header = cls.inventory["headers"][0]
        cls.valid_record = {
            "header": header["path"],
            "owner_todo": header["owner_todo"],
            "category": "abi",
            "layer": "root_shared",
            "revision": "7d9ef84",
            "host": "windows-11-x64",
            "target": "windows-x64",
            "toolchain": "msvc-19.44",
            "configuration": "Release",
            "command": [
                "tools/python/.venv/Scripts/python.exe",
                "tools/header_abi_audit.py",
            ],
            "artifacts": [
                "docs/ko/todo/navsys/evidence/sample/abi-root-shared.json"
            ],
            "verdict": "pass",
            "assertions": 1,
            "limitations": [],
        }

    def test_policy_covers_all_headers_and_owner_todos(self):
        self.assertEqual([], MODULE.validate_policy(
            self.policy,
            self.inventory,
        ))
        self.assertEqual(46, self.policy["coverage"]["headers"])
        self.assertEqual(43, self.policy["coverage"]["child_owner_todos"])

    def test_committed_evidence_tree_is_valid(self):
        findings, records = MODULE.validate_evidence_tree(
            self.policy,
            self.inventory,
        )
        self.assertEqual([], findings)
        self.assertEqual(2, records)

    def test_categories_and_layers_are_not_interchangeable(self):
        self.assertEqual(
            {"correctness", "abi", "performance"},
            set(self.policy["categories"]),
        )
        self.assertEqual(
            {"module_static", "root_shared", "wrapper", "installed_sdk"},
            set(self.policy["layers"]),
        )

        invalid = copy.deepcopy(self.policy)
        invalid["categories"].pop("performance")
        codes = {
            row.code for row in MODULE.validate_policy(invalid, self.inventory)
        }
        self.assertIn("category-set", codes)

        invalid = copy.deepcopy(self.policy)
        invalid["layers"]["wrapper"]["categories"].append("performance")
        codes = {
            row.code for row in MODULE.validate_policy(invalid, self.inventory)
        }
        self.assertIn("layer-category", codes)

        invalid = copy.deepcopy(self.policy)
        invalid["record_schema"]["required_fields"].remove("revision")
        codes = {
            row.code for row in MODULE.validate_policy(invalid, self.inventory)
        }
        self.assertIn("record-field-set", codes)

        invalid = copy.deepcopy(self.policy)
        invalid["categories"]["abi"]["must_not_use"] = []
        invalid["record_schema"]["rules"].remove("pass-requires-artifact")
        codes = {
            row.code for row in MODULE.validate_policy(invalid, self.inventory)
        }
        self.assertEqual(
            {"category-contract", "record-rule-set"},
            codes,
        )

    def test_valid_evidence_record_passes(self):
        self.assertEqual([], MODULE.validate_record(
            self.policy,
            self.inventory,
            self.valid_record,
        ))

    def test_wrong_owner_layer_and_artifact_path_fail_closed(self):
        invalid = copy.deepcopy(self.valid_record)
        invalid["owner_todo"] = "docs/ko/todo/navsys/wrong.org"
        invalid["category"] = "performance"
        invalid["layer"] = "wrapper"
        invalid["command"] = [
            "tools/python/.venv/Scripts/python.exe",
            "tools/python/byul_wrapper/generate_wrapper_abi.py",
            "--check",
        ]
        invalid["artifacts"] = ["C:\\temp\\result.json"]
        codes = {
            row.code
            for row in MODULE.validate_record(
                self.policy,
                self.inventory,
                invalid,
            )
        }
        self.assertEqual(
            {"owner-mismatch", "layer-category", "artifact-path"},
            codes,
        )

    def test_terminal_verdicts_require_evidence_or_limitation(self):
        for verdict, expected_code in (
            ("pass", "missing-artifact"),
            ("fail", "missing-artifact"),
            ("not-run", "missing-limitation"),
            ("not-applicable", "missing-limitation"),
        ):
            with self.subTest(verdict=verdict):
                invalid = copy.deepcopy(self.valid_record)
                invalid["verdict"] = verdict
                invalid["artifacts"] = []
                invalid["limitations"] = []
                codes = {
                    row.code
                    for row in MODULE.validate_record(
                        self.policy,
                        self.inventory,
                        invalid,
                    )
                }
                self.assertIn(expected_code, codes)

    def test_record_types_are_checked(self):
        invalid = copy.deepcopy(self.valid_record)
        invalid["assertions"] = True
        invalid["limitations"] = "none"
        invalid["toolchain"] = ""
        codes = {
            row.code
            for row in MODULE.validate_record(
                self.policy,
                self.inventory,
                invalid,
            )
        }
        self.assertEqual({"record-type"}, codes)

        invalid = copy.deepcopy(self.valid_record)
        invalid["command"] = ["wrong-command"]
        codes = {
            row.code
            for row in MODULE.validate_record(
                self.policy,
                self.inventory,
                invalid,
            )
        }
        self.assertEqual({"command-template-mismatch"}, codes)

    def test_artifact_revision_and_header_must_match_record(self):
        payload = {
            "revision": "deadbee",
            "header": "byul/navsys/wrong.h",
        }
        codes = {
            row.code
            for row in MODULE.validate_artifact_payload(
                self.valid_record,
                "sample.json",
                payload,
            )
        }
        self.assertEqual(
            {
                "artifact-revision-mismatch",
                "artifact-header-mismatch",
            },
            codes,
        )

    def test_tree_rejects_missing_artifact_duplicate_and_bad_json(self):
        records = sorted(MODULE.EVIDENCE_ROOT.rglob("*.record.json"))
        self.assertGreaterEqual(len(records), 1)
        source = load_json(records[0])
        source["artifacts"] = [
            "docs/ko/todo/navsys/evidence/missing.result.json"
        ]

        with tempfile.TemporaryDirectory(dir=REPOSITORY_ROOT) as directory:
            root = Path(directory)
            first = root / "first.record.json"
            second = root / "second.record.json"
            bad = root / "bad.record.json"
            first.write_text(json.dumps(source), encoding="utf-8")
            second.write_text(json.dumps(source), encoding="utf-8")
            bad.write_text("{", encoding="utf-8")

            findings, count = MODULE.validate_evidence_tree(
                self.policy,
                self.inventory,
                root,
            )
        self.assertEqual(3, count)
        codes = {row.code for row in findings}
        self.assertIn("missing-artifact-file", codes)
        self.assertIn("duplicate-record", codes)
        self.assertIn("invalid-record-json", codes)


if __name__ == "__main__":
    unittest.main()
