#!/usr/bin/env python3
"""Freeze the Navsys dependency and handwritten-wrapper stage-0 boundary."""

from __future__ import annotations

import argparse
import importlib.util
import json
import os
from pathlib import Path
import re
import sys
import tempfile
from typing import Any


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
NAVSYS_ROOT = REPOSITORY_ROOT / "byul/navsys"
WRAPPER_ROOT = REPOSITORY_ROOT / "tools/python/byul_wrapper"
WRAPPER_MODULE_ROOT = WRAPPER_ROOT / "byul_wrapper"
DEFAULT_OUTPUT = (
    REPOSITORY_ROOT
    / "docs/ko/todo/navsys/navsys-stage0-boundary-baseline.json"
)
DEFAULT_EXPORT_SNAPSHOT = (
    REPOSITORY_ROOT
    / "docs/ko/todo/header-refactor-current/msvc-release-build.json"
)

ORCHESTRATION_TODO = "docs/ko/todo/navsys/todo-리팩토링-navsys-headers.org"
OWNER_RULES = (
    (
        "byul/navsys/dstar_lite/dstar_lite_key",
        "docs/ko/todo/navsys/todo-navsys-dstar-lite-dstar-lite-key.org",
        5,
    ),
    (
        "byul/navsys/dstar_lite/dstar_lite_pqueue",
        "docs/ko/todo/navsys/todo-navsys-dstar-lite-dstar-lite-pqueue.org",
        5,
    ),
    (
        "byul/navsys/dstar_lite/dstar_lite_tick",
        "docs/ko/todo/navsys/todo-navsys-dstar-lite-dstar-lite-tick.org",
        5,
    ),
    (
        "byul/navsys/dstar_lite/",
        "docs/ko/todo/navsys/todo-navsys-dstar-lite-dstar-lite.org",
        5,
    ),
    (
        "byul/navsys/route_finder/dfs",
        "docs/ko/todo/navsys/todo-navsys-route-finder-dfs.org",
        4,
    ),
    (
        "byul/navsys/route_finder/weighted_astar",
        "docs/ko/todo/navsys/todo-navsys-route-finder-weighted-astar.org",
        4,
    ),
    (
        "byul/navsys/route_finder/",
        "docs/ko/todo/navsys/todo-navsys-route-finder-route-finder.org",
        4,
    ),
    (
        "byul/navsys/route/",
        "docs/ko/todo/navsys/todo-navsys-route-route.org",
        2,
    ),
    (
        "byul/navsys/coord/",
        "docs/ko/todo/navsys/todo-navsys-coord-coord.org",
        1,
    ),
    (
        "byul/navsys/navgrid/",
        "docs/ko/todo/navsys/todo-navsys-navgrid-navgrid.org",
        2,
    ),
    (
        "byul/navsys/obstacle/",
        "docs/ko/todo/navsys/todo-navsys-obstacle-obstacle.org",
        2,
    ),
    (
        "byul/navsys/maze/",
        "docs/ko/todo/navsys/todo-navsys-maze-maze.org",
        3,
    ),
)

WRAPPER_OWNERS = {
    "coord.py": "docs/ko/todo/navsys/todo-navsys-coord-coord.org",
    "coord_hash.py": "docs/ko/todo/navsys/todo-navsys-coord-coord-hash.org",
    "coord_list.py": "docs/ko/todo/navsys/todo-navsys-coord-coord-list.org",
    "cost_coord_pq.py": "docs/ko/todo/navsys/todo-navsys-coord-cost-coord-pq.org",
    "dstar_lite.py": "docs/ko/todo/navsys/todo-navsys-dstar-lite-dstar-lite.org",
    "dstar_lite_key.py": (
        "docs/ko/todo/navsys/todo-navsys-dstar-lite-dstar-lite-key.org"
    ),
    "dstar_lite_pqueue.py": (
        "docs/ko/todo/navsys/todo-navsys-dstar-lite-dstar-lite-pqueue.org"
    ),
    "navgrid.py": "docs/ko/todo/navsys/todo-navsys-navgrid-navgrid.org",
    "navsys_status.py": (
        "docs/ko/todo/navsys/todo-navsys-industry-api-baseline.org"
    ),
    "route.py": "docs/ko/todo/navsys/todo-navsys-route-route.org",
    "route_finder.py": (
        "docs/ko/todo/navsys/todo-navsys-route-finder-route-finder.org"
    ),
    "route_finder_common.py": (
        "docs/ko/todo/navsys/todo-navsys-route-finder-route-finder-core.org"
    ),
}


def load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def atomic_write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=path.parent
    )
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8", newline="\n") as stream:
            json.dump(payload, stream, ensure_ascii=False, indent=2)
            stream.write("\n")
        os.replace(temporary, path)
    except BaseException:
        Path(temporary).unlink(missing_ok=True)
        raise


def strip_c_comments(source: str) -> str:
    source = re.sub(r"/\*.*?\*/", "", source, flags=re.DOTALL)
    return re.sub(r"//[^\n]*", "", source)


def dependency_owner(path: str) -> tuple[str, int]:
    for prefix, owner, stage in OWNER_RULES:
        if path.startswith(prefix):
            return owner, stage
    return ORCHESTRATION_TODO, 7


def dependency_edges() -> list[dict[str, Any]]:
    patterns = (
        (
            "scalar-include",
            re.compile(r'^\s*#include\s*[<"]scalar\.h[>"]', re.MULTILINE),
        ),
        ("scalar-equal-call", re.compile(r"\bscalar_equal\s*\(")),
        ("m-pi-token", re.compile(r"\bM_PI\b")),
    )
    edges: list[dict[str, Any]] = []
    source_paths = sorted(
        path
        for path in NAVSYS_ROOT.rglob("*")
        if path.is_file() and path.suffix in {".c", ".cpp", ".h", ".hpp"}
    )
    for path in source_paths:
        relative = path.relative_to(REPOSITORY_ROOT).as_posix()
        source = strip_c_comments(path.read_text(encoding="utf-8", errors="replace"))
        owner, stage = dependency_owner(relative)
        for kind, pattern in patterns:
            count = len(pattern.findall(source))
            if count:
                edges.append(
                    {
                        "kind": kind,
                        "path": relative,
                        "occurrences": count,
                        "owner_todo": owner,
                        "target_stage": stage,
                        "disposition": "remove with the owning child integration gate",
                    }
                )

    for path in sorted(NAVSYS_ROOT.rglob("CMakeLists.txt")):
        relative = path.relative_to(REPOSITORY_ROOT).as_posix()
        source = re.sub(
            r"(?m)^\s*#.*$", "", path.read_text(encoding="utf-8", errors="replace")
        )
        owner, stage = dependency_owner(relative)
        for kind, pattern in (
            (
                "numal-include-directory",
                re.compile(r"\$\{CMAKE_SOURCE_DIR\}/numal"),
            ),
            ("numal-link", re.compile(r"(?m)^\s*numal\s*$")),
        ):
            count = len(pattern.findall(source))
            if count:
                edges.append(
                    {
                        "kind": kind,
                        "path": relative,
                        "occurrences": count,
                        "owner_todo": owner,
                        "target_stage": stage,
                        "disposition": "remove after source-level Numal use is eliminated",
                    }
                )
    return sorted(edges, key=lambda row: (row["path"], row["kind"]))


def load_wrapper_generator() -> Any:
    sys.path.insert(0, str(WRAPPER_ROOT))
    try:
        path = WRAPPER_ROOT / "generate_wrapper_abi.py"
        spec = importlib.util.spec_from_file_location("generate_wrapper_abi", path)
        if spec is None or spec.loader is None:
            raise RuntimeError(f"cannot load wrapper generator: {path}")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module
    finally:
        sys.path.pop(0)


def cdef_symbols(generator: Any) -> tuple[set[str], set[str]]:
    sys.path.insert(0, str(WRAPPER_ROOT))
    try:
        from byul_wrapper_generator import parse_header

        callable_symbols = {
            declaration.name
            for relative in generator.registered_headers()
            for declaration in parse_header(REPOSITORY_ROOT / "byul" / relative)
        }
        symbols = set(callable_symbols)
        for headers in generator.MODULE_HEADERS.values():
            cdef = generator.generated_block(headers)
            for body in re.findall(
                r"(?:typedef\s+)?enum(?:\s+[A-Za-z_][A-Za-z0-9_]*)?"
                r"\s*\{(.*?)\}",
                cdef,
                flags=re.DOTALL,
            ):
                symbols.update(
                    match.group(1)
                    for match in re.finditer(
                        r"(?:^|,)\s*([A-Za-z_][A-Za-z0-9_]*)"
                        r"(?:\s*=\s*[^,]+)?",
                        body,
                    )
                )
        return symbols, callable_symbols
    finally:
        sys.path.pop(0)


def wrapper_references() -> dict[str, list[str]]:
    references: dict[str, set[str]] = {}
    for module_name in sorted(WRAPPER_OWNERS):
        path = WRAPPER_MODULE_ROOT / module_name
        source = path.read_text(encoding="utf-8", errors="replace")
        for symbol in re.findall(r"\bC\.([A-Za-z_][A-Za-z0-9_]*)", source):
            references.setdefault(symbol, set()).add(module_name)
    return {
        symbol: sorted(modules)
        for symbol, modules in sorted(references.items())
    }


def wrapper_owner(modules: list[str]) -> str:
    owners = sorted({WRAPPER_OWNERS[module] for module in modules})
    return owners[0] if len(owners) == 1 else ORCHESTRATION_TODO


def wrapper_resolution(export_snapshot: Path) -> dict[str, Any]:
    generator = load_wrapper_generator()
    declared, callable_symbols = cdef_symbols(generator)
    references = wrapper_references()
    exports = {
        row["name"] for row in load_json(export_snapshot).get("exports", [])
    }

    def findings(symbols: set[str], disposition: str) -> list[dict[str, Any]]:
        return [
            {
                "symbol": symbol,
                "modules": references[symbol],
                "owner_todo": wrapper_owner(references[symbol]),
                "disposition": disposition,
            }
            for symbol in sorted(symbols)
        ]

    referenced = set(references)
    return {
        "reference_symbols": len(referenced),
        "cdef_symbols": len(declared),
        "root_export_symbols": len(exports),
        "missing_cdef": findings(
            referenced - declared,
            "add canonical declaration or remove the unsupported wrapper method",
        ),
        "missing_export": findings(
            (referenced & callable_symbols) - exports,
            "restore the root export or migrate the wrapper to a supported symbol",
        ),
    }


def build_payload(export_snapshot: Path = DEFAULT_EXPORT_SNAPSHOT) -> dict[str, Any]:
    edges = dependency_edges()
    wrapper = wrapper_resolution(export_snapshot)
    return {
        "schema_version": 1,
        "scope": "byul/navsys-stage-0",
        "export_snapshot": export_snapshot.relative_to(REPOSITORY_ROOT).as_posix(),
        "dependency_edges": edges,
        "wrapper_symbol_resolution": wrapper,
        "summary": {
            "dependency_edges": len(edges),
            "unclassified_dependency_edges": sum(
                not row["owner_todo"] or not row["disposition"] for row in edges
            ),
            "wrapper_reference_symbols": wrapper["reference_symbols"],
            "missing_cdef": len(wrapper["missing_cdef"]),
            "missing_export": len(wrapper["missing_export"]),
            "unclassified_wrapper_findings": sum(
                not row["owner_todo"] or not row["disposition"]
                for key in ("missing_cdef", "missing_export")
                for row in wrapper[key]
            ),
        },
    }


def validate_payload(payload: dict[str, Any]) -> list[str]:
    errors: list[str] = []
    summary = payload.get("summary", {})
    if summary.get("unclassified_dependency_edges") != 0:
        errors.append("unclassified-dependency-edge")
    if summary.get("unclassified_wrapper_findings") != 0:
        errors.append("unclassified-wrapper-finding")
    for row in payload.get("dependency_edges", []):
        if not (REPOSITORY_ROOT / row.get("owner_todo", "")).is_file():
            errors.append(f"missing-owner-todo:{row.get('owner_todo', '')}")
    for key in ("missing_cdef", "missing_export"):
        for row in payload.get("wrapper_symbol_resolution", {}).get(key, []):
            if not (REPOSITORY_ROOT / row.get("owner_todo", "")).is_file():
                errors.append(f"missing-owner-todo:{row.get('owner_todo', '')}")
    return sorted(set(errors))


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--check", action="store_true")
    mode.add_argument("--apply", action="store_true")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument(
        "--export-snapshot", type=Path, default=DEFAULT_EXPORT_SNAPSHOT
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    output = args.output.resolve()
    export_snapshot = args.export_snapshot.resolve()
    payload = build_payload(export_snapshot)
    errors = validate_payload(payload)
    if errors:
        for error in errors:
            print(f"[ERROR] {error}", file=sys.stderr)
        return 2

    if args.apply:
        atomic_write_json(output, payload)
        print(f"[WRITTEN] {output.relative_to(REPOSITORY_ROOT)}")
    else:
        if not output.is_file() or load_json(output) != payload:
            print(
                f"[ERROR] stale stage-0 baseline: "
                f"{output.relative_to(REPOSITORY_ROOT)}",
                file=sys.stderr,
            )
            return 1
        print(f"[OK] {output.relative_to(REPOSITORY_ROOT)}")

    summary = payload["summary"]
    print(
        "[SUMMARY] "
        f"dependencies={summary['dependency_edges']} "
        f"wrapper_refs={summary['wrapper_reference_symbols']} "
        f"missing_cdef={summary['missing_cdef']} "
        f"missing_export={summary['missing_export']}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
