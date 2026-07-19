"""Run the BYUL Python wrapper test suite from one command-line entry point."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import subprocess
import sys


ROOT = Path(__file__).resolve().parent
TEST_ROOT = ROOT / "tests"
GENERATOR_TESTS = (
    TEST_ROOT / "test_generate_wrapper_abi.py",
    TEST_ROOT / "test_header_parser.py",
    TEST_ROOT / "test_coord_abi_policy.py",
    TEST_ROOT / "test_coord_hash_abi_policy.py",
    TEST_ROOT / "test_navsys_abi_vocabulary.py",
    TEST_ROOT / "test_tests_runner.py",
    TEST_ROOT / "test_wrapper_coverage.py",
)


def build_command(framework: str, unit_only: bool, quiet: bool) -> list[str]:
    """Build the child test-runner command without invoking it."""
    if framework == "pytest":
        targets = GENERATOR_TESTS if unit_only else (TEST_ROOT,)
        command = [sys.executable, "-m", "pytest"]
        command.extend(str(path) for path in targets)
        command.append("-q" if quiet else "-v")
        return command

    patterns = (
        ("test_generate_wrapper_abi.py", "test_header_parser.py")
        if unit_only
        else ("test_*.py",)
    )
    # unittest accepts one discovery pattern at a time.  Multiple unit-only
    # patterns are executed by main() as separate commands.
    return [
        sys.executable,
        "-m",
        "unittest",
        "discover",
        "-s",
        str(TEST_ROOT),
        "-p",
        patterns[0],
        "-q" if quiet else "-v",
    ]


def build_commands(framework: str, unit_only: bool, quiet: bool) -> tuple[list[str], ...]:
    if framework != "unittest" or not unit_only:
        return (build_command(framework, unit_only, quiet),)
    return tuple(
        [
            sys.executable,
            "-m",
            "unittest",
            "discover",
            "-s",
            str(TEST_ROOT),
            "-p",
            path.name,
            "-q" if quiet else "-v",
        ]
        for path in GENERATOR_TESTS
    )


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run all BYUL Python wrapper tests.",
    )
    parser.add_argument(
        "--framework",
        choices=("pytest", "unittest"),
        default="pytest",
        help="test runner to use (default: pytest)",
    )
    parser.add_argument(
        "--unit-only",
        action="store_true",
        help="run only generator and header-parser tests that do not load BYUL",
    )
    parser.add_argument(
        "--library",
        type=Path,
        help="path to byul.dll, libbyul.so, or libbyul.dylib",
    )
    parser.add_argument(
        "--quiet",
        action="store_true",
        help="reduce test-runner output",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="print commands without running tests",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    environment = os.environ.copy()

    if args.library is not None:
        library = args.library.expanduser().resolve()
        if not library.is_file():
            print(f"[ERROR] BYUL shared library not found: {library}", file=sys.stderr)
            return 2
        environment["BYUL_LIBRARY_PATH"] = str(library)

    commands = build_commands(args.framework, args.unit_only, args.quiet)
    for command in commands:
        print("[TEST]", subprocess.list2cmdline(command), flush=True)
        if args.dry_run:
            continue
        result = subprocess.run(command, cwd=ROOT, env=environment, check=False)
        if result.returncode != 0:
            return result.returncode
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
