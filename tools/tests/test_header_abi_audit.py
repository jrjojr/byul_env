import copy
import importlib.util
from pathlib import Path
import tempfile
import sys
import unittest


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
MODULE_PATH = REPOSITORY_ROOT / "tools/header_abi_audit.py"
SPEC = importlib.util.spec_from_file_location("header_abi_audit", MODULE_PATH)
assert SPEC and SPEC.loader
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class HeaderAbiAuditTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        current = REPOSITORY_ROOT / "docs/ko/todo/header-refactor-current"
        cls.manifest = MODULE.load_json(current / "header-role-manifest.json")
        cls.build = MODULE.load_json(current / "msvc-release-build.json")
        cls.abi = MODULE.load_json(current / "msvc-release-abi.json")
        cls.metadata = MODULE.load_json(current / "header-audit.json")

    def _codes(self, findings):
        return {item.code for item in findings}

    def test_inventory_mutation_is_detected(self):
        manifest = copy.deepcopy(self.manifest)
        manifest["headers"].pop()
        findings = MODULE.manifest_findings(manifest)
        self.assertIn("unmanifested-header", self._codes(findings))

    def test_export_mutation_is_detected(self):
        _, parse_header = MODULE._load_header_parser()
        symbols = {
            item.name
            for item in parse_header(REPOSITORY_ROOT / "byul/byul.h")
        }
        self.assertTrue(symbols)
        build = copy.deepcopy(self.build)
        build["exports"] = [
            row for row in build["exports"] if row["name"] not in symbols
        ]
        findings = MODULE.export_findings(symbols, build)
        self.assertIn("missing-root-export", self._codes(findings))

    def test_abi_summary_mutation_is_detected(self):
        abi = copy.deepcopy(self.abi)
        abi["summary"]["complete_types"] -= 1
        findings, fingerprint = MODULE.abi_snapshot_findings(abi)
        self.assertIn("abi-summary-mismatch", self._codes(findings))
        self.assertEqual(64, len(fingerprint))

    def test_wrapper_row_mutation_is_detected(self):
        manifest = copy.deepcopy(self.manifest)
        generated = next(
            row
            for row in manifest["headers"]
            if row["wrapper"]["mode"] == "generated"
        )
        generated["wrapper"] = {"mode": "manual", "reason": "mutation"}
        findings = MODULE.wrapper_findings(manifest)
        self.assertIn("wrapper-row-missing", self._codes(findings))

    def test_only_exact_invariant_exclusion_is_applied(self):
        findings = [
            MODULE.Finding("missing-root-export", "old_symbol", "missing"),
            MODULE.Finding("missing-root-export", "new_symbol", "missing"),
        ]
        exclusions = {
            "exclusions": [
                {
                    "code": "missing-root-export",
                    "subject": "old_symbol",
                    "reason": "known",
                    "owner": "test",
                    "target_stage": "test",
                }
            ]
        }
        remaining, excluded = MODULE.apply_invariant_exclusions(
            findings, exclusions
        )
        self.assertEqual(["new_symbol"], [item.subject for item in remaining])
        self.assertEqual(["old_symbol"], [item.subject for item in excluded])

    def test_structural_negative_and_fixed_fixture(self):
        with tempfile.TemporaryDirectory(dir=REPOSITORY_ROOT) as directory:
            path = Path(directory) / "sample.h"
            path.write_text("BYUL_API void sample(void);\n", encoding="utf-8")
            codes = self._codes(MODULE.structural_findings(path))
            self.assertIn("header-license", codes)
            self.assertIn("file-doxygen", codes)
            self.assertIn("include-guard", codes)
            self.assertIn("c-linkage", codes)

            license_text = MODULE.LICENSE_PATH.read_text(encoding="utf-8").rstrip()
            path.write_text(
                license_text
                + "\n\n"
                + "/** @file sample.h\n * @brief Sample API.\n */\n"
                + "#ifndef BYUL_SAMPLE_H\n#define BYUL_SAMPLE_H\n"
                + "#ifdef __cplusplus\nextern \"C\" {\n#endif\n"
                + "BYUL_API void sample(void);\n"
                + "#ifdef __cplusplus\n}\n#endif\n"
                + "#endif /* BYUL_SAMPLE_H */\n",
                encoding="utf-8",
            )
            self.assertEqual([], MODULE.structural_findings(path))

    def test_standalone_source_includes_one_header(self):
        source = MODULE.standalone_source("byul/navsys/coord/coord.h")
        self.assertEqual(
            '#include "byul/navsys/coord/coord.h"\n'
            "int main(void) { return 0; }\n",
            source,
        )


if __name__ == "__main__":
    unittest.main()
