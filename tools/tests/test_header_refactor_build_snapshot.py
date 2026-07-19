import importlib.util
from pathlib import Path
import unittest


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
MODULE_PATH = REPOSITORY_ROOT / "tools/header_refactor_build_snapshot.py"
SPEC = importlib.util.spec_from_file_location(
    "header_refactor_build_snapshot", MODULE_PATH
)
assert SPEC and SPEC.loader
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class HeaderRefactorBuildSnapshotTest(unittest.TestCase):
    def test_parses_posix_nm_and_excludes_undefined_symbols(self):
        output = (
            "byul_version T 0000000000001000 10\n"
            "byul_status R 0000000000002000 4\n"
            "__imp_byul_status I 0000000000003000 8\n"
            "malloc U\n"
        )
        self.assertEqual(
            [
                {"name": "byul_version", "type": "T"},
                {"name": "byul_status", "type": "R"},
            ],
            MODULE.parse_nm_exports(output),
        )

    def test_dumpbin_parser_remains_backward_compatible(self):
        output = "      1    0 00001000 byul_version\n"
        self.assertEqual("byul_version", MODULE.parse_dumpbin_exports(output)[0]["name"])


if __name__ == "__main__":
    unittest.main()
