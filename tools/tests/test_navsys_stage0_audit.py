import copy
import importlib.util
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "tools/navsys_stage0_audit.py"
SPEC = importlib.util.spec_from_file_location("navsys_stage0_audit", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


class NavsysStage0AuditTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.payload = MODULE.build_payload()

    def test_current_boundary_is_fully_classified(self):
        self.assertEqual([], MODULE.validate_payload(self.payload))
        self.assertGreater(self.payload["summary"]["dependency_edges"], 0)
        self.assertGreater(self.payload["summary"]["wrapper_reference_symbols"], 0)

    def test_dependency_mutation_is_detected_by_payload_difference(self):
        mutated = copy.deepcopy(self.payload)
        mutated["dependency_edges"].pop()
        self.assertNotEqual(self.payload, mutated)

    def test_unclassified_dependency_fails_closed(self):
        mutated = copy.deepcopy(self.payload)
        mutated["summary"]["unclassified_dependency_edges"] = 1
        self.assertIn(
            "unclassified-dependency-edge",
            MODULE.validate_payload(mutated),
        )

    def test_unclassified_wrapper_finding_fails_closed(self):
        mutated = copy.deepcopy(self.payload)
        mutated["summary"]["unclassified_wrapper_findings"] = 1
        self.assertIn(
            "unclassified-wrapper-finding",
            MODULE.validate_payload(mutated),
        )


if __name__ == "__main__":
    unittest.main()
