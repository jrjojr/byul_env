import ast
from pathlib import Path
import unittest


PROJECT_ROOT = Path(__file__).resolve().parents[1]
WRAPPER_ROOT = PROJECT_ROOT / "byul_wrapper"
TEST_ROOT = PROJECT_ROOT / "tests"


def class_names(path: Path) -> set[str]:
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    return {
        node.name
        for node in ast.walk(tree)
        if isinstance(node, ast.ClassDef) and node.name.startswith("c_")
    }


def referenced_names(path: Path) -> set[str]:
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    return {
        node.id
        for node in ast.walk(tree)
        if isinstance(node, ast.Name) and node.id.startswith("c_")
    }


class WrapperCoverageTest(unittest.TestCase):
    def test_every_c_wrapper_class_is_referenced_by_a_test(self):
        wrappers = set().union(
            *(class_names(path) for path in WRAPPER_ROOT.glob("*.py"))
        )
        tested = set().union(
            *(
                referenced_names(path)
                for path in TEST_ROOT.glob("test_*.py")
                if path.name != Path(__file__).name
            )
        )

        self.assertTrue(wrappers, "no c_* wrapper classes were discovered")
        self.assertEqual(
            wrappers - tested,
            set(),
            "c_* wrapper classes without a test reference",
        )


if __name__ == "__main__":
    unittest.main()
