import importlib.util
from pathlib import Path
import unittest


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
MODULE_PATH = REPOSITORY_ROOT / "tools/header_role_manifest.py"
SPEC = importlib.util.spec_from_file_location("header_role_manifest", MODULE_PATH)
assert SPEC and SPEC.loader
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class HeaderRoleManifestTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        source = MODULE.load_json(
            REPOSITORY_ROOT
            / "docs/ko/todo/header-refactor-current/header-inventory.json"
        )
        cls.manifest, cls.report = MODULE.build_outputs(source)
        cls.by_path = {
            row["current_path"]: row for row in cls.manifest["headers"]
        }

    def test_all_existing_assets_are_approved_once(self):
        self.assertEqual(137, self.manifest["summary"]["headers"])
        self.assertEqual(137, self.manifest["summary"]["approved"])
        self.assertEqual(137, len(self.by_path))

    def test_navgrid_callback_guard_remains_internal(self):
        row = self.by_path[
            "byul/navsys/navgrid/internal/navgrid_callback.hpp"
        ]
        self.assertEqual("internal", row["primary_role"])
        self.assertFalse(row["approved_install"])
        self.assertEqual("excluded", row["wrapper"]["mode"])

    def test_boundary_decisions_cover_stage_one_unknowns(self):
        self.assertEqual(
            "public-aggregate",
            self.by_path["byul/balix/numeq/numeq.h"]["primary_role"],
        )
        self.assertEqual(
            "public-foundation",
            self.by_path["byul/rng/rng_core.h"]["primary_role"],
        )
        self.assertEqual(
            "deprecated-reference",
            self.by_path["byul/aerial/aerial.h"]["primary_role"],
        )
        self.assertEqual(
            "compatibility-forwarder",
            self.by_path[
                "byul/navsys/dstar_lite/dstar_lite_key.hpp"
            ]["primary_role"],
        )

    def test_cpp_facades_remain_installed_but_not_wrapped(self):
        for path in (
            "byul/numal/vec3.hpp",
            "byul/numal/quat.hpp",
            "byul/numal/dualquat.hpp",
        ):
            row = self.by_path[path]
            self.assertEqual("cpp-facade", row["primary_role"])
            self.assertTrue(row["approved_install"])
            self.assertEqual("excluded", row["wrapper"]["mode"])

    def test_renames_have_versioned_compatibility_paths(self):
        forwarders = [
            row
            for row in self.manifest["headers"]
            if row["naming"]["disposition"] == "forward"
        ]
        self.assertGreater(len(forwarders), 0)
        for row in forwarders:
            naming = row["naming"]
            self.assertEqual(row["current_path"], naming["compatibility_path"])
            self.assertEqual("1.1.0", naming["deprecated_since"])
            self.assertEqual("2.0.0", naming["remove_in"])
            self.assertNotEqual(row["current_path"], naming["canonical_path"])

    def test_rng_install_gap_is_visible(self):
        self.assertIn("byul/rng/rng.h", self.report["additions"])
        self.assertIn("byul/rng/rng_core.h", self.report["additions"])

    def test_internal_dstar_successor_is_not_installed(self):
        self.assertNotIn(
            "byul/navsys/dstar_lite/internal/dstar_lite_key_ops.hpp",
            self.report["approved_intent_paths"],
        )
        self.assertIn(
            "byul/navsys/dstar_lite/dstar_lite_key.hpp",
            self.report["approved_intent_paths"],
        )

    def test_current_include_graph_and_sdk_basenames_are_valid(self):
        self.assertEqual(0, self.manifest["summary"]["current_include_cycles"])
        self.assertEqual(0, self.manifest["summary"]["sdk_basename_collisions"])
        self.assertEqual(
            0, self.manifest["summary"]["abi_2_sdk_basename_collisions"]
        )

    def test_math_engine_successors_are_independent_approved_assets(self):
        successors = self.manifest["created_successors"]
        self.assertEqual(20, len(successors))
        self.assertEqual(20, self.manifest["summary"]["successor_headers"])
        self.assertGreater(self.manifest["summary"]["successor_links"], 20)
        self.assertEqual(
            len(successors),
            len({row["canonical_path"] for row in successors}),
        )
        for row in successors:
            self.assertTrue(row["asset_id"].startswith("created:"))
            self.assertEqual("math_engine", row["owner_module"])
            self.assertEqual("byul_math_", row["symbol_prefix"])
            self.assertTrue((REPOSITORY_ROOT / row["design_todo"]).is_file())
            self.assertTrue(row["source_assets"])

    def test_public_resource_migrations_preserve_abi_one_layout(self):
        migrations = self.manifest["public_resource_migrations"]
        self.assertEqual(10, len(migrations))
        self.assertEqual(
            10, self.manifest["summary"]["public_resource_migrations"]
        )
        self.assertEqual([], self.manifest["remaining_stage_2_design_gate"])
        for row in migrations:
            self.assertEqual("preserve", row["abi_1_x"]["layout"])
            self.assertTrue(row["abi_1_x"]["additive_api"])
            self.assertEqual(
                "replace the exposed layout only at the explicit ABI-major 2.0 gate",
                row["abi_2_0"]["transition"],
            )
            self.assertTrue((REPOSITORY_ROOT / row["design_todo"]).is_file())


if __name__ == "__main__":
    unittest.main()
