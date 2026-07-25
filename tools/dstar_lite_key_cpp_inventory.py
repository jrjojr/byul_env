#!/usr/bin/env python3
"""Generate/check the D* Lite C++ key-helper consumer inventory."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import re
import sys
import tempfile


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
HEADER = Path("byul/navsys/dstar_lite/dstar_lite_key.hpp")
MODULE_CMAKE = Path("byul/navsys/dstar_lite/CMakeLists.txt")
ROLE_MANIFEST = Path(
    "docs/ko/todo/header-refactor-current/header-role-manifest.json"
)
DEFAULT_OUTPUT = Path(
    "docs/ko/todo/navsys/dstar-lite-key-cpp-consumer-inventory.json"
)
INSTALL_PATH = Path("include/byul/dstar_lite_key.hpp")
SOURCE_SUFFIXES = {".c", ".cc", ".cpp", ".cxx", ".h", ".hh", ".hpp"}
SYMBOL_PATTERNS = {
    "DstarLiteKeyHash": re.compile(r"\bDstarLiteKeyHash\b"),
    "DstarLiteKeyEqual": re.compile(r"\bDstarLiteKeyEqual\b"),
    "DstarLiteKeyLess": re.compile(r"\bDstarLiteKeyLess\b"),
    "std::hash<dstar_lite_key_t>": re.compile(
        r"\bhash\s*<\s*dstar_lite_key_t\s*>"
    ),
    "dstar_lite_key_ptr_less": re.compile(
        r"\bdstar_lite_key_ptr_less\b"
    ),
}
HEADER_INCLUDE = re.compile(r"#\s*include\s*[<\"]dstar_lite_key\.hpp[>\"]")


def relative(path: Path) -> str:
    return path.relative_to(REPOSITORY_ROOT).as_posix()


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def source_files() -> list[Path]:
    files: list[Path] = []
    for root_name in ("byul", "external"):
        root = REPOSITORY_ROOT / root_name
        for path in root.rglob("*"):
            if not path.is_file() or path.suffix.lower() not in SOURCE_SUFFIXES:
                continue
            if any(part in {".git", "build", "__pycache__"} for part in path.parts):
                continue
            files.append(path)
    return sorted(files)


def classify_consumer(path: Path) -> str:
    name = relative(path)
    if name.startswith("external/"):
        return "external"
    if "/tests/" in name or path.name.startswith("test_"):
        return "test-only"
    return "internal-production"


def matching_lines(path: Path, pattern: re.Pattern[str]) -> list[int]:
    try:
        text = path.read_text(encoding="utf-8", errors="ignore")
    except OSError:
        return []
    return [
        number
        for number, line in enumerate(text.splitlines(), 1)
        if pattern.search(line)
    ]


def load_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def build_inventory(install_root: Path) -> dict:
    header_path = REPOSITORY_ROOT / HEADER
    files = source_files()
    symbol_rows = []
    all_consumers: set[tuple[str, str]] = set()
    include_consumers = []
    for path in files:
        if path == header_path:
            continue
        for line in matching_lines(path, HEADER_INCLUDE):
            category = classify_consumer(path)
            include_consumers.append(
                {
                    "path": relative(path),
                    "line": line,
                    "category": category,
                }
            )
            all_consumers.add((relative(path), category))
    for name, pattern in SYMBOL_PATTERNS.items():
        declarations = matching_lines(header_path, pattern)
        consumers = []
        for path in files:
            if path == header_path:
                continue
            for line in matching_lines(path, pattern):
                category = classify_consumer(path)
                row = {
                    "path": relative(path),
                    "line": line,
                    "category": category,
                }
                consumers.append(row)
                all_consumers.add((row["path"], category))
        symbol_rows.append(
            {
                "name": name,
                "declaration_lines": declarations,
                "consumers": consumers,
            }
        )

    manifest = load_json(REPOSITORY_ROOT / ROLE_MANIFEST)
    role = next(
        row
        for row in manifest["headers"]
        if row["current_path"] == HEADER.as_posix()
    )
    cmake_text = (REPOSITORY_ROOT / MODULE_CMAKE).read_text(encoding="utf-8")
    installed = install_root.resolve() / INSTALL_PATH
    categories = {
        category: sorted(path for path, value in all_consumers if value == category)
        for category in ("internal-production", "test-only", "external")
    }
    return {
        "schema_version": 1,
        "header": {
            "path": HEADER.as_posix(),
            "sha256": sha256(header_path),
            "declared_helpers": len(SYMBOL_PATTERNS),
        },
        "summary": {
            "unique_consumers": len(all_consumers),
            "internal_production": len(categories["internal-production"]),
            "test_only": len(categories["test-only"]),
            "external": len(categories["external"]),
            "old_sdk_source_fixture_present": bool(categories["external"]),
        },
        "consumers": categories,
        "header_includes": include_consumers,
        "symbols": symbol_rows,
        "build_and_distribution": {
            "module_public_header_inventory": (
                f"/dstar_lite_key.hpp" in cmake_text.replace("\\", "/")
            ),
            "install_path": INSTALL_PATH.as_posix(),
            "clean_install_present": installed.is_file(),
            "clean_install_sha256": sha256(installed) if installed.is_file() else None,
            "manifest_exported_symbols": role["source_evidence"]["exported_symbols"],
            "wrapper_registered": role["source_evidence"]["wrapper_modules_registered"],
            "wrapper_disposition": role["wrapper"],
        },
        "approved_boundary": {
            "manifest": ROLE_MANIFEST.as_posix(),
            "decision_status": role["decision_status"],
            "primary_role": role["primary_role"],
            "abi_1_x_install": role["approved_install"],
            "naming": role["naming"],
            "forwarding_required": True,
            "forwarding_reason": (
                "The helper shipped in the ABI 1.x SDK; retain the legacy include "
                "path through 1.x even though no external source fixture was found."
            ),
        },
    }


def write_json_atomic(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
        "w", encoding="utf-8", newline="\n", dir=path.parent, delete=False
    ) as stream:
        json.dump(payload, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
        temporary = Path(stream.name)
    temporary.replace(path)


def validate(payload: dict) -> list[str]:
    errors = []
    if payload["header"]["declared_helpers"] != 5:
        errors.append("the helper declaration inventory is not exactly five")
    summary = payload["summary"]
    if summary["internal_production"] != 1:
        errors.append("expected exactly one internal production consumer")
    if summary["test_only"] or summary["external"]:
        errors.append("unexpected test or external C++ helper consumer")
    build = payload["build_and_distribution"]
    if not build["module_public_header_inventory"]:
        errors.append("the ABI 1.x CMake header inventory no longer contains the helper")
    if not build["clean_install_present"]:
        errors.append("the clean ABI 1.x install does not contain the helper")
    if build["clean_install_sha256"] != payload["header"]["sha256"]:
        errors.append("the clean installed helper differs from the source header")
    if build["manifest_exported_symbols"] or build["wrapper_registered"]:
        errors.append("the C++ helper leaked into exports or the wrapper manifest")
    boundary = payload["approved_boundary"]
    if boundary["decision_status"] != "approved":
        errors.append("the header boundary decision is not approved")
    if boundary["primary_role"] != "compatibility-forwarder":
        errors.append("the ABI 1.x header is not classified as a compatibility forwarder")
    if boundary["naming"]["canonical_install"]:
        errors.append("the internal canonical helper is incorrectly installable")
    return errors


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--apply", action="store_true")
    mode.add_argument("--check", action="store_true")
    parser.add_argument("--install-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    output = args.output if args.output.is_absolute() else REPOSITORY_ROOT / args.output
    payload = build_inventory(args.install_root)
    errors = validate(payload)
    if errors:
        for error in errors:
            print(f"[ERROR] {error}", file=sys.stderr)
        return 1
    if args.apply:
        write_json_atomic(output, payload)
        print(f"[WRITTEN] {output} helpers=5 consumers=1 external=0")
        return 0
    if not output.is_file() or load_json(output) != payload:
        print(f"[ERROR] stale inventory: {output}", file=sys.stderr)
        return 1
    print(f"[OK] {output} helpers=5 consumers=1 external=0")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
