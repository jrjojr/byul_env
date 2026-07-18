from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path
import sys
import tempfile
import unittest


SCRIPT = Path(__file__).resolve().parents[1] / "header_refactor_baseline.py"
SPEC = spec_from_file_location("header_refactor_baseline", SCRIPT)
BASELINE = module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(BASELINE)

BUILD_SCRIPT = Path(__file__).resolve().parents[1] / "header_refactor_build_snapshot.py"
BUILD_SPEC = spec_from_file_location("header_refactor_build_snapshot", BUILD_SCRIPT)
BUILD_SNAPSHOT = module_from_spec(BUILD_SPEC)
assert BUILD_SPEC.loader is not None
BUILD_SPEC.loader.exec_module(BUILD_SNAPSHOT)

ABI_SCRIPT = Path(__file__).resolve().parents[1] / "header_refactor_abi_probe.py"
ABI_SPEC = spec_from_file_location("header_refactor_abi_probe", ABI_SCRIPT)
ABI_PROBE = module_from_spec(ABI_SPEC)
assert ABI_SPEC.loader is not None
sys.modules[ABI_SPEC.name] = ABI_PROBE
ABI_SPEC.loader.exec_module(ABI_PROBE)


class HeaderRefactorBaselineTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.manifest = BASELINE.build_manifest()

    def test_tracked_inventory_has_expected_unique_assets(self):
        rows = self.manifest["headers"]
        self.assertEqual(136, len(rows))
        self.assertEqual(105, self.manifest["summary"]["byul_headers"])
        self.assertEqual(31, self.manifest["summary"]["tool_headers"])
        self.assertEqual(len(rows), len({row["asset_id"] for row in rows}))
        self.assertEqual(
            len(rows), len({row["current_path"] for row in rows})
        )

    def test_worktree_inventory_handles_header_replacement(self):
        headers = BASELINE.tracked_headers()
        paths = {BASELINE.relative(path) for path in headers}
        self.assertIn(
            "byul/navsys/coord/internal/coord_ops.hpp", paths
        )
        self.assertNotIn("byul/navsys/coord/coord.hpp", paths)
        self.assertTrue(all(path.is_file() for path in headers))

        row = next(
            row
            for row in self.manifest["headers"]
            if row["current_path"]
            == "byul/navsys/coord/internal/coord_ops.hpp"
        )
        self.assertEqual("private-implementation", row["role_candidate"])
        self.assertEqual("internalize", row["naming_disposition"])

    def test_every_row_has_required_baseline_fields(self):
        required = {
            "asset_id",
            "current_path",
            "sha256",
            "line_count",
            "language",
            "license",
            "include_guard",
            "role_candidate",
            "current_owner",
            "public_module_boundary_candidate",
            "responsibility",
            "representative_type_candidate",
            "naming_disposition",
            "compatibility_path",
            "successor_headers",
            "module_header_groups",
            "root_header_inventory",
            "install_inventory",
            "umbrella_consumers",
            "wrapper_modules_registered",
            "source_definition_candidates",
            "decision_status",
        }
        for row in self.manifest["headers"]:
            self.assertTrue(required <= row.keys(), row["current_path"])
            self.assertIn(
                row["naming_disposition"], BASELINE.DISPOSITION_VALUES
            )

    def test_naming_report_is_deterministic(self):
        first = BASELINE.build_naming_report(self.manifest)
        second = BASELINE.build_naming_report(self.manifest)
        self.assertEqual(first, second)

    def test_wrapper_inventory_reuses_canonical_generator_manifest(self):
        self.assertEqual(
            27, self.manifest["summary"]["wrapper_registered_headers"]
        )

    def test_atomic_write_uses_utf8_and_replaces_content(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "sample.json"
            BASELINE.atomic_write(path, '{"value":"초기"}\n')
            BASELINE.atomic_write(path, '{"value":"완료"}\n')
            self.assertEqual('{"value":"완료"}\n', path.read_text(encoding="utf-8"))

    def test_dumpbin_export_parser(self):
        output = """
    ordinal hint RVA      name
          1    0 00001040 byul_print_version
         17   10 00001230 byul_version_string
        Summary
"""
        self.assertEqual(
            [
                {
                    "ordinal": 1,
                    "hint": "0",
                    "rva": "00001040",
                    "name": "byul_print_version",
                },
                {
                    "ordinal": 17,
                    "hint": "10",
                    "rva": "00001230",
                    "name": "byul_version_string",
                },
            ],
            BUILD_SNAPSHOT.parse_dumpbin_exports(output),
        )

    def test_abi_declaration_parser_helpers(self):
        fields, skipped = ABI_PROBE.field_names(
            "int x; double values[3]; void (*callback)(int); unsigned flag : 1;"
        )
        self.assertEqual(("x", "values", "callback"), fields)
        self.assertEqual(("unsigned flag : 1",), skipped)
        self.assertEqual(
            ("MODE_ZERO", "MODE_ONE"),
            ABI_PROBE.enum_names("MODE_ZERO = 0, MODE_ONE = 4"),
        )
        complete = ABI_PROBE.complete_typedefs(
            "typedef struct s_outer { int mode; union { int x; float y; } u; } outer_t;",
            "struct",
        )
        self.assertEqual(1, len(complete))
        nested_fields, nested_skipped = ABI_PROBE.field_names(complete[0][0])
        self.assertEqual(("mode", "u"), nested_fields)
        self.assertEqual((), nested_skipped)

    def test_probe_output_parser(self):
        types, enums = ABI_PROBE.parse_probe(
            "TYPE\tcoord_t\t8\t4\n"
            "FIELD\tcoord_t\tx\t0\n"
            "ENUM\tmode_t\tMODE_A\t3\n"
        )
        self.assertEqual(
            {"size": 8, "align": 4, "fields": {"x": 0}},
            types["coord_t"],
        )
        self.assertEqual({"MODE_A": 3}, enums["mode_t"])


if __name__ == "__main__":
    unittest.main()
