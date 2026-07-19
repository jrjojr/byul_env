from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path
import unittest


SCRIPT = Path(__file__).resolve().parents[1] / "run_tests.py"
SPEC = spec_from_file_location("byul_wrapper_tests_runner", SCRIPT)
RUNNER = module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(RUNNER)


class TestsRunnerTest(unittest.TestCase):
    def test_default_command_runs_entire_pytest_directory(self):
        commands = RUNNER.build_commands("pytest", unit_only=False, quiet=False)

        self.assertEqual(len(commands), 1)
        self.assertIn(str(RUNNER.TEST_ROOT), commands[0])
        self.assertIn("-v", commands[0])

    def test_unit_only_pytest_selects_generator_tests(self):
        command = RUNNER.build_command("pytest", unit_only=True, quiet=True)

        self.assertIn(str(RUNNER.GENERATOR_TESTS[0]), command)
        self.assertIn(str(RUNNER.GENERATOR_TESTS[1]), command)
        self.assertIn("-q", command)

    def test_unit_only_unittest_builds_one_discovery_per_file(self):
        commands = RUNNER.build_commands("unittest", unit_only=True, quiet=False)

        self.assertEqual(len(commands), 7)
        self.assertIn("test_generate_wrapper_abi.py", commands[0])
        self.assertIn("test_header_parser.py", commands[1])
        self.assertIn("test_coord_abi_policy.py", commands[2])
        self.assertIn("test_coord_hash_abi_policy.py", commands[3])
        self.assertIn("test_navsys_abi_vocabulary.py", commands[4])
        self.assertIn("test_tests_runner.py", commands[5])
        self.assertIn("test_wrapper_coverage.py", commands[6])

    def test_argument_parser_defaults_to_all_pytest_tests(self):
        args = RUNNER.parse_args([])

        self.assertEqual(args.framework, "pytest")
        self.assertFalse(args.unit_only)


if __name__ == "__main__":
    unittest.main()
