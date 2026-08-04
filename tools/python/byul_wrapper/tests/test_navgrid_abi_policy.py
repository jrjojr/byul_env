import json
from pathlib import Path
import re
import sys
import unittest


REPOSITORY_ROOT = Path(__file__).resolve().parents[4]
WRAPPER_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WRAPPER_ROOT))

from byul_wrapper_generator import parse_header


HEADER = REPOSITORY_ROOT / "byul" / "navsys" / "navgrid" / "navgrid.h"
ABI1_HEADER = (
    REPOSITORY_ROOT / "byul" / "navsys" / "navgrid" / "compat" / "abi1"
    / "navgrid_abi1.h"
)
SOURCE = REPOSITORY_ROOT / "byul" / "navsys" / "navgrid" / "navgrid.cpp"
MANIFEST = (
    REPOSITORY_ROOT / "docs" / "ko" / "todo" / "navsys"
    / "navgrid-abi-policy.json"
)
LIFECYCLE_POLICY = (
    REPOSITORY_ROOT / "docs" / "ko" / "todo" / "navsys"
    / "navsys-lifecycle-policy.json"
)
WRAPPER = WRAPPER_ROOT / "byul_wrapper" / "navgrid.py"
ALLOWED_DISPOSITIONS = {"keep", "add", "deprecate-forward", "abi-major"}


def load_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


class NavgridAbiPolicyTest(unittest.TestCase):
    def test_current_header_function_inventory_matches_manifest_exactly(self):
        manifest = load_json(MANIFEST)
        declarations = parse_header(HEADER)
        actual = {declaration.name for declaration in declarations}

        self.assertEqual(len(declarations), len(actual))
        self.assertEqual(actual, set(manifest["current_functions"]))
        self.assertEqual(len(actual), manifest["layout_verification"]["export_count"])
        self.assertTrue(
            set(manifest["current_functions"].values()) <= ALLOWED_DISPOSITIONS
        )

    def test_enum_struct_and_callback_declarations_match_manifest(self):
        manifest = load_json(MANIFEST)
        header = HEADER.read_text(encoding="utf-8")
        enum = re.search(
            r"typedef\s+enum\s*\{(?P<body>.*?)\}\s*navgrid_dir_mode_t\s*;",
            header,
            re.DOTALL,
        )
        self.assertIsNotNone(enum)
        names = re.findall(r"^\s*(NAVGRID_DIR_[48])\s*,?\s*$", enum.group("body"), re.MULTILINE)
        self.assertEqual(names, list(manifest["current_enum"]["values"]))
        self.assertEqual(manifest["current_enum"]["values"], {"NAVGRID_DIR_4": 0, "NAVGRID_DIR_8": 1})

        entry = manifest["current_cell_entry"]
        self.assertEqual(entry["size"], 20)
        self.assertEqual(entry["alignment"], 4)
        for field in entry["fields"]:
            self.assertIn(field["declaration"], header)

        self.assertRegex(header, r"typedef\s+struct\s+s_navgrid\s+navgrid_t\s*;")
        self.assertNotRegex(header, r"struct\s+s_navgrid\s*\{")
        self.assertEqual(manifest["current_type"]["public_fields"], [])

        abi1_header = ABI1_HEADER.read_text(encoding="utf-8")
        struct = re.search(
            r"struct\s+s_navgrid\s*\{(?P<body>.*?)\}\s*;",
            abi1_header,
            re.DOTALL,
        )
        self.assertIsNotNone(struct)
        fields = [
            re.sub(r"\s+", " ", match.group(1)).strip()
            for match in re.finditer(
                r"^\s*([^/;]+?)\s*;", struct.group("body"), re.MULTILINE
            )
        ]
        expected = [
            field["declaration"]
            for field in manifest["abi1_compatibility_layout"]["fields"]
        ]
        self.assertEqual(fields, expected)
        self.assertIn("BYUL_NAVGRID_ABI_VERSION UINT32_C(2)", header)
        self.assertIn(
            "BYUL_NAVGRID_ABI_FINGERPRINT UINT64_C(0x4e4752494402002f)",
            header,
        )
        self.assertRegex(
            header,
            r"typedef\s+bool\s*\(\*is_coord_blocked_func\)\s*\(\s*"
            r"const\s+void\*\s+context\s*,\s*int\s+x\s*,\s*int\s+y\s*,\s*"
            r"void\*\s+userdata\s*\)\s*;",
        )

    def test_generated_cffi_contains_the_frozen_surface(self):
        wrapper = WRAPPER.read_text(encoding="utf-8")
        generated = wrapper.split(
            "/* Source: byul/navsys/navgrid/navgrid.h */", 1
        )[1].split('\n""")', 1)[0]
        manifest = load_json(MANIFEST)

        for symbol in manifest["current_functions"]:
            self.assertRegex(generated, rf"\b{re.escape(symbol)}\s*\(")
        self.assertRegex(generated, r"typedef\s+struct\s+s_navgrid\s+navgrid_t\s*;")
        self.assertNotRegex(generated, r"struct\s+s_navgrid\s*\{")
        self.assertNotRegex(generated, r"\bnavgrid_get_blocked_coords\s*\(")

    def test_consumer_and_opaque_migration_inventory_is_materialized(self):
        manifest = load_json(MANIFEST)
        inventory = manifest["consumer_inventory"]
        for category, paths in inventory.items():
            for path in paths:
                self.assertTrue((REPOSITORY_ROOT / path).exists(), f"{category}: {path}")

        migration = manifest["abi_major_migration"]
        self.assertEqual(migration["target"], "opaque-navgrid-handle")
        self.assertEqual(migration["status"], "implemented")
        self.assertIn("cell count/export/visitor", migration["replace_public_cell_map"])
        self.assertIn("atomic binding", migration["replace_public_callback_fields"])

        migrated_sources = {
            "byul/console/console.cpp": ("navgrid->width", "navgrid->height"),
            "byul/navsys/route_finder/fast_marching.cpp": ("m->width", "m->height"),
            "byul/tests/sdk_consumer/main.c": ("navgrid->",),
            "byul/tests/sdk_consumer/main.cpp": ("public_grid->",),
        }
        for path, forbidden in migrated_sources.items():
            source = (REPOSITORY_ROOT / path).read_text(encoding="utf-8")
            for token in forbidden:
                self.assertNotIn(token, source, f"{path}: {token}")

    def test_known_defects_are_characterized_without_claiming_their_fix(self):
        manifest = load_json(MANIFEST)
        source = SOURCE.read_text(encoding="utf-8")
        wrapper = WRAPPER.read_text(encoding="utf-8")
        current = manifest["current_characterization"]

        self.assertRegex(source, r"new\s+navgrid_t\s*\{\}")
        self.assertIn("navgrid->is_coord_blocked_fn_userdata = nullptr", source)
        self.assertRegex(
            source,
            r"coord_hash_copy_ex\(navgrid->cell_map,\s*&copied_map\)",
        )
        self.assertIn("failure-atomic-deep-copy", current["copy_cell_map"])
        self.assertIn("retains-both", current["python_copy_binding"])
        self.assertIn(
            "copied._ffi_is_coord_blocked_func = self._ffi_is_coord_blocked_func",
            wrapper,
        )
        self.assertNotIn("C.navgrid_get_blocked_coords", wrapper)
        self.assertIn("materialized-cell-export", current["python_blocked_coords"])
        self.assertEqual(
            current["fix_owner_stages"]["copy-and-binding"],
            "completed-in-stage-3",
        )
        self.assertEqual(
            current["fix_owner_stages"]["checked-mutation-and-overlay"],
            "completed-in-stage-4",
        )
        self.assertEqual(
            current["fix_owner_stages"]["caller-buffer-and-blocked-coords-wrapper"],
            "completed-in-stage-5",
        )
        self.assertIn("preserves-private-ABI1-binary-layout", current["overlay_storage"])
        self.assertEqual(
            current["fix_owner_stages"]["opaque-layout"],
            "completed-in-stage-6",
        )
        for symbol in (
            "navgrid_set_cell_ex",
            "navgrid_fetch_cell_ex",
            "navgrid_block_coord_ex",
            "navgrid_unblock_coord_ex",
            "navgrid_apply_blocked_overlay",
            "navgrid_remove_blocked_overlay",
            "navgrid_export_neighbors",
            "navgrid_export_neighbors_range",
            "navgrid_fetch_neighbor_at_degree",
            "navgrid_fetch_neighbor_at_goal",
            "navgrid_export_neighbors_at_degree_range",
            "navgrid_export_cells",
        ):
            self.assertIn(f"C.{symbol}", wrapper)

    def test_callback_contract_matches_common_lifecycle_policy(self):
        manifest = load_json(MANIFEST)
        lifecycle = load_json(LIFECYCLE_POLICY)
        binding = next(
            item for item in lifecycle["callback_bindings"]
            if item["family"] == "navgrid"
        )
        callback = manifest["callback"]

        self.assertEqual(binding["bind_symbol"], "navgrid_bind_is_coord_blocked_func")
        self.assertEqual(binding["unbind_symbol"], "navgrid_unbind_is_coord_blocked_func")
        self.assertEqual(
            callback["userdata_ownership"], "caller-owned-borrowed-by-BYUL"
        )
        self.assertIn("both-source-and-copy", callback["copy_userdata_lifetime"])
        self.assertIn("retain-the-source-CFFI-callback-owner", callback["python_copy_policy"])
        self.assertEqual(callback["destroy_policy"], "destroy-never-frees-callback-userdata")
        self.assertIn("function-and-userdata-null", binding["unbind_result"])


if __name__ == "__main__":
    unittest.main()
