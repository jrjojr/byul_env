import json
from pathlib import Path
import unittest


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
POLICY = (
    REPOSITORY_ROOT
    / "docs/ko/todo/navsys/navsys-lifecycle-policy.json"
)
INVENTORY = (
    REPOSITORY_ROOT
    / "docs/ko/todo/navsys/navsys-current-abi-inventory.json"
)


def load_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


class NavsysLifecyclePolicyTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.policy = load_json(POLICY)
        cls.inventory = load_json(INVENTORY)

    def test_every_inventory_resource_has_exactly_one_model(self):
        inventory_resources = {
            row["resource"]
            for row in self.inventory["lifecycle"]["resources"]
        }
        policy_resources = [
            row["resource"]
            for row in self.policy["resource_models"]
        ]

        self.assertEqual(len(policy_resources), len(set(policy_resources)))
        self.assertEqual(inventory_resources, set(policy_resources))
        for row in self.policy["resource_models"]:
            with self.subTest(resource=row["resource"]):
                self.assertTrue(row["storage"])
                self.assertTrue(row["termination"])

    def test_every_inventory_callback_slot_has_one_atomic_target(self):
        inventory_slots = {
            (row["family"], row["slot"])
            for row in self.inventory["callback_bindings"]
        }
        policy_slots = [
            (row["family"], row["slot"])
            for row in self.policy["callback_bindings"]
        ]

        self.assertEqual(len(policy_slots), len(set(policy_slots)))
        self.assertEqual(inventory_slots, set(policy_slots))

    def test_bind_and_unbind_symbols_are_unique_and_family_scoped(self):
        exported_symbols = {
            symbol
            for header in self.inventory["headers"]
            for symbol in header["symbols"]
        }
        symbols = []
        for row in self.policy["callback_bindings"]:
            with self.subTest(family=row["family"], slot=row["slot"]):
                bind = row["bind_symbol"]
                unbind = row["unbind_symbol"]
                self.assertTrue(bind.startswith(f"{row['family']}_bind_"))
                self.assertTrue(unbind.startswith(f"{row['family']}_unbind_"))
                self.assertTrue(row["unbind_result"])
                self.assertIn(bind, exported_symbols)
                self.assertIn(unbind, exported_symbols)
                symbols.extend((bind, unbind))

        self.assertEqual(len(symbols), len(set(symbols)))

    def test_abi_one_and_two_changes_are_separated(self):
        abi_1 = self.policy["abi_1_policy"]
        abi_2 = self.policy["abi_2_policy"]

        self.assertEqual("additive-only", abi_1["compatibility"])
        self.assertEqual("preserve", abi_1["public_struct_layout"])
        self.assertEqual(
            "function-and-userdata-change-together-or-remain-unchanged",
            abi_1["binding_commit"],
        )
        self.assertEqual(
            "materialized-in-current-abi-inventory",
            abi_1["binding_surface"],
        )
        self.assertIn(
            "remove-public-struct-fields",
            abi_2["breaking_changes"],
        )
        self.assertIn(
            "change-callback-signatures-to-navsys-status",
            abi_2["breaking_changes"],
        )

    def test_retained_userdata_is_borrowed_with_a_finite_lifetime(self):
        abi_1 = self.policy["abi_1_policy"]

        self.assertEqual(
            "borrowed-by-BYUL",
            abi_1["callback_context_ownership"],
        )
        self.assertIn(
            "unbind",
            abi_1["callback_context_lifetime"],
        )
        self.assertEqual(
            "exclusive-owner-mutation-required",
            abi_1["thread_safety"],
        )
        self.assertEqual(
            "runtime-guarded-and-tested",
            abi_1["reentrancy_enforcement"]["route_finder"],
        )
        self.assertEqual(
            "caller-contract-only",
            abi_1["reentrancy_enforcement"]["navgrid"],
        )
        self.assertEqual(
            "caller-contract-only",
            abi_1["reentrancy_enforcement"]["dstar_lite"],
        )


if __name__ == "__main__":
    unittest.main()
