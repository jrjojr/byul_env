import copy
import importlib.util
from pathlib import Path
import sys
import unittest


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
MODULE_PATH = REPOSITORY_ROOT / "tools/navsys_cancellation_audit.py"
SPEC = importlib.util.spec_from_file_location(
    "navsys_cancellation_audit",
    MODULE_PATH,
)
assert SPEC and SPEC.loader
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class NavsysCancellationAuditTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.sources = MODULE.load_algorithm_sources(
            MODULE.DEFAULT_SOURCE_ROOT
        )

    def test_all_dispatcher_algorithms_retain_polling_points(self):
        self.assertEqual(11, len(MODULE.EXPECTED_POLL_COUNTS))
        self.assertEqual(14, sum(MODULE.EXPECTED_POLL_COUNTS.values()))
        self.assertEqual([], MODULE.cancellation_findings(self.sources))

    def test_cancel_ignore_mutation_is_detected(self):
        mutated = copy.deepcopy(self.sources)
        original = mutated["astar.cpp"]
        mutated["astar.cpp"] = original.replace(
            "&& !route_finder_poll_cancel_internal()",
            "&& true",
            1,
        )
        self.assertNotEqual(original, mutated["astar.cpp"])

        findings = MODULE.cancellation_findings(mutated)
        self.assertEqual(
            [(
                "cancel-poll-count",
                "astar.cpp",
                "expected 1 cancellation polling points, found 0",
            )],
            [
                (row.code, row.subject, row.message)
                for row in findings
            ],
        )

    def test_missing_algorithm_source_is_detected(self):
        mutated = copy.deepcopy(self.sources)
        mutated.pop("weighted_astar.cpp")
        findings = MODULE.cancellation_findings(mutated)
        self.assertEqual(
            [(
                "missing-algorithm-source",
                "weighted_astar.cpp",
            )],
            [(row.code, row.subject) for row in findings],
        )


if __name__ == "__main__":
    unittest.main()
