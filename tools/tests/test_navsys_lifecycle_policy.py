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
ABI_SNAPSHOT = (
    REPOSITORY_ROOT
    / "docs/ko/todo/header-refactor-current/msvc-release-abi.json"
)
VOCABULARY = (
    REPOSITORY_ROOT
    / "docs/ko/todo/navsys/navsys-abi-vocabulary.json"
)
SDK_CONSUMER = REPOSITORY_ROOT / "byul/tests/sdk_consumer/main.c"
ALLOCATION_FAILURE_FIXTURE = (
    REPOSITORY_ROOT
    / "byul/navsys/tests/test_navsys_allocation_failure.cpp"
)
NAVSYS_TEST_CMAKE = REPOSITORY_ROOT / "byul/navsys/tests/CMakeLists.txt"
ROUTE_FINDER_WRAPPER = (
    REPOSITORY_ROOT
    / "tools/python/byul_wrapper/byul_wrapper/route_finder.py"
)


def load_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


class NavsysLifecyclePolicyTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.policy = load_json(POLICY)
        cls.inventory = load_json(INVENTORY)
        cls.abi_snapshot = load_json(ABI_SNAPSHOT)
        cls.vocabulary = load_json(VOCABULARY)
        cls.sdk_consumer = SDK_CONSUMER.read_text(encoding="utf-8")
        cls.allocation_failure_fixture = (
            ALLOCATION_FAILURE_FIXTURE.read_text(encoding="utf-8")
        )
        cls.navsys_test_cmake = NAVSYS_TEST_CMAKE.read_text(encoding="utf-8")
        cls.route_finder_wrapper = ROUTE_FINDER_WRAPPER.read_text(
            encoding="utf-8"
        )

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
            "runtime-guarded-and-tested",
            abi_1["reentrancy_enforcement"]["navgrid"],
        )
        self.assertEqual(
            "runtime-guarded-and-tested",
            abi_1["reentrancy_enforcement"]["dstar_lite"],
        )

    def test_abi_two_allocator_contract_is_failure_atomic(self):
        abi_2 = self.policy["abi_2_policy"]
        allocator = abi_2["allocator_binding"]

        self.assertEqual("immutable-create-time-per-owner", allocator["scope"])
        self.assertEqual("borrowed-by-BYUL", allocator["userdata_ownership"])
        self.assertIn("deallocate-with-original-binding", allocator["operations"])
        self.assertEqual(
            "null-maps-to-NAVSYS_STATUS_OUT_OF_MEMORY",
            allocator["allocation_failure"],
        )
        self.assertEqual(
            -2,
            self.vocabulary["status"]["values"][
                "NAVSYS_STATUS_OUT_OF_MEMORY"
            ],
        )
        self.assertEqual(
            "preserve-all-output-pointers",
            allocator["output_guarantee"],
        )

    def test_current_create_paths_have_allocator_failure_injection(self):
        abi_1 = self.policy["abi_1_policy"]
        injection = abi_1["allocation_failure_injection"]

        self.assertEqual(
            "byul/navsys/tests/test_navsys_allocation_failure.cpp",
            injection["fixture"],
        )
        self.assertEqual(
            {"navgrid", "route_finder", "dstar_lite"},
            set(injection["owners"]),
        )
        self.assertEqual({"msvc", "mingw"}, set(injection["toolchains"]))
        self.assertEqual(
            "no-exception-escape-and-no-owned-allocation-leak",
            injection["result"],
        )
        for family in injection["owners"]:
            with self.subTest(family=family):
                self.assertIn(
                    f'"{family}"',
                    self.allocation_failure_fixture,
                )
        self.assertIn(
            "add_test(\n"
            "    NAME test_navsys_allocation_failure",
            self.navsys_test_cmake,
        )
        release_gates = self.policy["abi_2_policy"]["release_gates"]
        self.assertEqual(
            "verified-in-abi-1-msvc-and-mingw-static-module",
            release_gates["legacy-create-allocation-failure"],
        )
        self.assertEqual(
            "required-before-abi-2-release",
            release_gates["abi-2-bound-allocator-failure"],
        )
        self.assertEqual(
            "approved",
            release_gates["stage-2-contract-approval"],
        )
        self.assertEqual(
            "after-runtime-foundation-before-abi-2-release",
            release_gates["abi-2-runtime-gate-phase"],
        )

    def test_opaque_candidates_have_an_abi_one_layout_baseline(self):
        abi_2 = self.policy["abi_2_policy"]
        transition = abi_2["handle_transition"]
        complete_types = {
            row["name"] for row in self.abi_snapshot["types"]
        }

        self.assertEqual(
            "docs/ko/todo/header-refactor-current/msvc-release-abi.json",
            transition["abi_1_layout_snapshot"],
        )
        self.assertLessEqual(
            set(abi_2["opaque_struct_candidates"]),
            complete_types,
        )
        self.assertEqual(
            "reject-before-first-object-operation",
            transition["cross_major_loading"],
        )
        self.assertEqual(
            "implemented-installed-sdk-x64-public-field-consumer",
            transition["abi_1_fixture"],
        )
        self.assertEqual(
            "verified-in-abi-1-msvc-and-mingw",
            abi_2["release_gates"]["old-layout-consumer"],
        )
        self.assertEqual(
            "required-before-abi-2-release",
            abi_2["release_gates"]["new-handle-consumer"],
        )
        self.assertEqual(
            "contract-approved-runtime-implementation-pending",
            abi_2["release_gates"]["current_state"],
        )

    def test_abi_one_consumer_pins_every_candidate_field(self):
        candidates = set(
            self.policy["abi_2_policy"]["opaque_struct_candidates"]
        )
        layouts = {
            row["name"]: row
            for row in self.abi_snapshot["types"]
            if row["name"] in candidates
        }

        self.assertEqual(candidates, set(layouts))
        for name, row in layouts.items():
            with self.subTest(type=name):
                self.assertIn(
                    f"ABI1_TYPE_LAYOUT({name}, {row['size']}, "
                    f"{row['align']});",
                    self.sdk_consumer,
                )
                for field, offset in row["fields"].items():
                    self.assertIn(
                        f"ABI1_FIELD_OFFSET({name}, {field}, {offset});",
                        self.sdk_consumer,
                    )

    def test_route_finder_capability_query_reaches_consumers(self):
        exported_symbols = {
            symbol
            for header in self.inventory["headers"]
            for symbol in header["symbols"]
        }

        self.assertIn("route_finder_is_supported", exported_symbols)
        self.assertIn(
            "route_finder_is_supported(ROUTE_FINDER_ASTAR)",
            self.sdk_consumer,
        )
        self.assertIn(
            "bool route_finder_is_supported(route_finder_type_t type);",
            self.route_finder_wrapper,
        )
        self.assertIn(
            "def list_supported_route_finders():",
            self.route_finder_wrapper,
        )

    def test_typed_route_finder_configs_reach_all_consumers(self):
        binding = self.policy["abi_1_policy"]["algorithm_config_binding"]
        exported_symbols = {
            symbol
            for header in self.inventory["headers"]
            for symbol in header["symbols"]
        }

        self.assertEqual(
            "type-and-config-pointer-change-together-or-remain-unchanged",
            binding["binding_commit"],
        )
        self.assertEqual(
            "clear-config-pointer-and-preserve-selected-type",
            binding["unbind_result"],
        )
        self.assertEqual(4, len(binding["configs"]))
        for config in binding["configs"]:
            with self.subTest(config=config["config"]):
                self.assertIn(config["bind_symbol"], exported_symbols)
                self.assertIn(config["bind_symbol"], self.sdk_consumer)
                self.assertIn(config["bind_symbol"], self.route_finder_wrapper)
                self.assertIn(config["config"], self.route_finder_wrapper)

        self.assertIn(binding["unbind_symbol"], exported_symbols)
        self.assertIn(binding["unbind_symbol"], self.sdk_consumer)
        self.assertIn(binding["unbind_symbol"], self.route_finder_wrapper)
        self.assertIn(
            "self._algorithm_config = config",
            self.route_finder_wrapper,
        )

    def test_route_finder_run_contract_reaches_all_consumers(self):
        contract = self.policy["abi_1_policy"]["route_finder_run_contract"]
        exported_symbols = {
            symbol
            for header in self.inventory["headers"]
            for symbol in header["symbols"]
        }

        self.assertEqual(
            {
                "NAVSYS_STATUS_OK",
                "NAVSYS_STATUS_NO_PATH",
                "NAVSYS_STATUS_LIMIT_REACHED",
            },
            set(contract["normal_termination"]),
        )
        self.assertEqual(
            "preserve-route-and-stats",
            contract["failure_output"],
        )
        self.assertEqual(
            "call-scoped-cooperative-polling",
            contract["cancellation"],
        )
        self.assertEqual(
            "before-each-expansion-and-during-unbounded-reconstruction",
            contract["cancel_polling"],
        )
        self.assertEqual(
            "synchronous-calling-thread",
            contract["cancel_callback_thread"],
        )
        self.assertEqual(
            "deferred-until-algorithms-expose-caller-storage",
            contract["workspace"],
        )
        for symbol in [
            *contract["checked_setters"],
            contract["run_symbol"],
            contract["controlled_run_symbol"],
        ]:
            with self.subTest(symbol=symbol):
                self.assertIn(symbol, exported_symbols)
                self.assertIn(symbol, self.sdk_consumer)
                self.assertIn(symbol, self.route_finder_wrapper)
        self.assertIn(contract["stats_type"], self.route_finder_wrapper)
        self.assertIn(contract["options_type"], self.route_finder_wrapper)
        self.assertIn(contract["cancel_callback"], self.route_finder_wrapper)
        self.assertIn("def find_ex(", self.route_finder_wrapper)
        self.assertIn("cancel_callback", self.route_finder_wrapper)


if __name__ == "__main__":
    unittest.main()
