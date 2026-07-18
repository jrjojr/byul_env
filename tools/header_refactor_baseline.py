"""BYUL header 리팩터링의 재현 가능한 최초 기준선을 생성한다."""

from __future__ import annotations

import argparse
from collections import defaultdict
from datetime import datetime, timezone
import hashlib
from importlib.util import module_from_spec, spec_from_file_location
import json
import os
from pathlib import Path
import platform
import re
import shutil
import subprocess
import sys
import tempfile
from typing import Any


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT_DIR = (
    REPOSITORY_ROOT / "docs" / "ko" / "todo" / "header-refactor-baseline"
)
HEADER_SUFFIXES = {".h", ".hpp"}
SOURCE_SUFFIXES = {".c", ".cc", ".cpp", ".cxx"}
DISPOSITION_VALUES = {
    "keep",
    "rename",
    "forward",
    "split",
    "internalize",
    "generated-regenerate",
    "tool-move",
    "decision-required",
}
GENERIC_COMPONENT_STEMS = {
    "astar",
    "bfs",
    "dfs",
    "dijkstra",
    "fast_marching",
    "fringe_search",
    "greedy_best_first",
    "ida_star",
    "rta_star",
    "sma_star",
    "weighted_astar",
}


def run_command(
    arguments: list[str], *, cwd: Path = REPOSITORY_ROOT
) -> dict[str, Any]:
    """명령을 shell 없이 실행하고 재현 가능한 결과 구조를 반환한다."""
    try:
        completed = subprocess.run(
            arguments,
            cwd=cwd,
            check=False,
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
        )
    except OSError as error:
        return {
            "command": arguments,
            "available": False,
            "exit_code": None,
            "stdout": "",
            "stderr": str(error),
            "output": str(error),
        }
    stdout = completed.stdout.strip()
    stderr = completed.stderr.strip()
    output = "\n".join(
        part for part in (stdout, stderr) if part
    )
    return {
        "command": arguments,
        "available": True,
        "exit_code": completed.returncode,
        "stdout": stdout,
        "stderr": stderr,
        "output": output,
    }


def git_lines(*arguments: str) -> tuple[str, ...]:
    result = run_command(["git", *arguments])
    if result["exit_code"] != 0:
        raise RuntimeError(f"git {' '.join(arguments)} failed: {result['output']}")
    return tuple(line for line in result["stdout"].splitlines() if line)


def tracked_headers() -> tuple[Path, ...]:
    """현재 worktree에서 version 관리 대상인 byul/tools header를 반환한다."""
    paths = []
    for relative in git_lines(
        "ls-files",
        "--cached",
        "--others",
        "--exclude-standard",
        "--",
        "byul",
        "tools",
    ):
        path = REPOSITORY_ROOT / relative
        if path.is_file() and path.suffix.lower() in HEADER_SUFFIXES:
            paths.append(path)
    return tuple(sorted(paths, key=lambda item: item.as_posix()))


def tracked_sources() -> tuple[Path, ...]:
    paths = []
    for relative in git_lines(
        "ls-files",
        "--cached",
        "--others",
        "--exclude-standard",
        "--",
        "byul",
        "tools",
    ):
        path = REPOSITORY_ROOT / relative
        if path.is_file() and path.suffix.lower() in SOURCE_SUFFIXES:
            paths.append(path)
    return tuple(sorted(paths, key=lambda item: item.as_posix()))


def relative(path: Path) -> str:
    return path.relative_to(REPOSITORY_ROOT).as_posix()


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    digest.update(path.read_bytes())
    return digest.hexdigest()


def detect_license(source: str) -> str:
    prefix = "\n".join(source.splitlines()[:30]).lower()
    if "spdx-license-identifier:" in prefix:
        match = re.search(r"spdx-license-identifier:\s*([^\s*]+)", prefix)
        return f"SPDX:{match.group(1)}" if match else "SPDX:unknown"
    if "mit license" in prefix:
        return "MIT"
    if "apache license" in prefix:
        return "Apache"
    if "copyright" in prefix:
        return "copyright-present-license-unclassified"
    return "missing-or-unclassified"


def detect_guard(source: str) -> str | None:
    match = re.search(r"^\s*#ifndef\s+([A-Za-z_][A-Za-z0-9_]*)", source, re.MULTILINE)
    return match.group(1) if match else None


def representative_types(source: str) -> tuple[str, ...]:
    patterns = (
        r"typedef\s+(?:struct|enum|union)\s+[A-Za-z_][A-Za-z0-9_]*"
        r"\s*(?:\{.*?\})?\s*([A-Za-z_][A-Za-z0-9_]*)\s*;",
        r"typedef\s+[^;{}]+\s+([A-Za-z_][A-Za-z0-9_]*_t)\s*;",
    )
    found: list[str] = []
    for pattern in patterns:
        for match in re.finditer(pattern, source, re.DOTALL):
            name = match.group(1)
            if name not in found:
                found.append(name)
    return tuple(found)


def exported_symbols(source: str) -> tuple[str, ...]:
    pattern = re.compile(
        r"\bBYUL_API\b[^;{}]*?\b([A-Za-z_][A-Za-z0-9_]*)\s*\(",
        re.DOTALL,
    )
    return tuple(dict.fromkeys(match.group(1) for match in pattern.finditer(source)))


def cmake_header_groups() -> dict[str, dict[str, Any]]:
    groups: dict[str, dict[str, Any]] = {}
    for cmake_path in sorted((REPOSITORY_ROOT / "byul").rglob("CMakeLists.txt")):
        source = cmake_path.read_text(encoding="utf-8", errors="replace")
        project_match = re.search(
            r"(?im)^\s*project\(\s*([A-Za-z0-9_]+)", source
        )
        set_match = re.search(
            r"set\(\s*\$\{PROJECT_NAME\}_HEADERS(?P<body>.*?)"
            r"CACHE\s+INTERNAL",
            source,
            re.DOTALL | re.IGNORECASE,
        )
        if not project_match or not set_match:
            continue
        name = project_match.group(1)
        body = set_match.group("body")
        literals = []
        for item in re.findall(
            r"\$\{CMAKE_CURRENT_SOURCE_DIR\}/([^\s\)]+)", body
        ):
            candidate = (cmake_path.parent / item).resolve()
            if candidate.suffix.lower() in HEADER_SUFFIXES:
                literals.append(relative(candidate))
        references = tuple(
            ref
            for ref in re.findall(r"\$\{([A-Za-z0-9_]+)_HEADERS\}", body)
            if ref != "PROJECT_NAME"
        )
        groups[name] = {
            "cmake": relative(cmake_path),
            "literal_headers": tuple(sorted(set(literals))),
            "references": references,
        }
    return groups


def resolve_group_headers(
    name: str,
    groups: dict[str, dict[str, Any]],
    active: tuple[str, ...] = (),
) -> set[str]:
    if name in active:
        raise ValueError(f"CMake header inventory cycle: {' -> '.join((*active, name))}")
    group = groups.get(name)
    if group is None:
        return set()
    result = set(group["literal_headers"])
    for reference in group["references"]:
        result.update(resolve_group_headers(reference, groups, (*active, name)))
    return result


def wrapper_headers() -> set[str]:
    wrapper_root = (
        REPOSITORY_ROOT / "tools" / "python" / "byul_wrapper"
    )
    script = wrapper_root / "generate_wrapper_abi.py"
    spec = spec_from_file_location("_byul_generate_wrapper_abi", script)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load wrapper manifest: {relative(script)}")
    module = module_from_spec(spec)
    inserted = str(wrapper_root) not in sys.path
    if inserted:
        sys.path.insert(0, str(wrapper_root))
    try:
        spec.loader.exec_module(module)
        registered = module.registered_headers()
    finally:
        if inserted:
            sys.path.remove(str(wrapper_root))
    return {
        f"byul/{path}"
        for path in registered
        if (REPOSITORY_ROOT / "byul" / path).is_file()
    }


def header_include_consumers(
    headers: tuple[Path, ...],
) -> dict[str, tuple[str, ...]]:
    by_basename: dict[str, list[str]] = defaultdict(list)
    for path in headers:
        by_basename[path.name].append(relative(path))
    consumers: dict[str, set[str]] = defaultdict(set)
    for consumer in headers:
        source = consumer.read_text(encoding="utf-8", errors="replace")
        for include in re.findall(r'^\s*#include\s*[<"]([^>"]+)[>"]', source, re.MULTILINE):
            matches = by_basename.get(Path(include).name, [])
            if len(matches) == 1:
                consumers[matches[0]].add(relative(consumer))
    return {
        path: tuple(sorted(items))
        for path, items in consumers.items()
    }


def source_definitions(
    header_symbols: dict[str, tuple[str, ...]],
    sources: tuple[Path, ...],
) -> dict[str, tuple[str, ...]]:
    requested = {
        symbol for symbols in header_symbols.values() for symbol in symbols
    }
    symbol_sources: dict[str, set[str]] = defaultdict(set)
    call_pattern = re.compile(r"\b([A-Za-z_][A-Za-z0-9_]*)\s*\(")
    for path in sources:
        text = path.read_text(encoding="utf-8", errors="replace")
        present = {
            match.group(1) for match in call_pattern.finditer(text)
        } & requested
        for symbol in present:
            symbol_sources[symbol].add(relative(path))
    return {
        header: tuple(
            sorted(
                {
                    source
                    for symbol in symbols
                    for source in symbol_sources.get(symbol, ())
                }
            )
        )
        for header, symbols in header_symbols.items()
    }


def owner_and_boundary(path: Path) -> tuple[str, str | None]:
    parts = path.relative_to(REPOSITORY_ROOT).parts
    if parts[0] == "tools":
        owner = parts[1] if len(parts) > 1 else "tools"
        return owner, None
    owner = parts[1] if len(parts) > 1 else "byul"
    if owner == "navsys" and len(parts) > 3:
        return owner, parts[2]
    return owner, owner


def classify_role(path: Path, source: str, owner: str) -> str:
    stem = path.stem
    if {"internal", "detail"} & set(path.parts):
        return "private-implementation"
    if relative(path).startswith("tools/unit_1003/"):
        return "reference"
    if relative(path).startswith("tools/"):
        return "tool-only"
    if "generated" in source[:2000].lower() or "base_primes" in stem:
        return "generated"
    if path.suffix.lower() == ".hpp":
        return "cpp-facade"
    if stem in {"byul", owner}:
        return "public-module-entry"
    if "BYUL_API" in source:
        return "public-component"
    return "decision-required"


def disposition_candidate(
    path: Path,
    role: str,
    representative: str | None,
    boundary: str | None,
) -> str:
    stem = path.stem
    if role == "generated":
        return "generated-regenerate"
    if role == "private-implementation":
        return "internalize"
    if role in {"tool-only", "reference"}:
        return "tool-move"
    if role == "cpp-facade":
        return "decision-required"
    type_stem = representative.removesuffix("_t") if representative else None
    if role == "public-component" and type_stem and stem == type_stem:
        return "keep"
    if role == "public-component" and (
        stem in GENERIC_COMPONENT_STEMS
        or (boundary and not stem.startswith(f"{boundary}_") and not type_stem)
    ):
        return "rename"
    if role in {"public-module-entry", "public-component"}:
        return "keep"
    return "decision-required"


def build_manifest() -> dict[str, Any]:
    headers = tracked_headers()
    sources = tracked_sources()
    groups = cmake_header_groups()
    resolved_groups = {
        name: resolve_group_headers(name, groups) for name in groups
    }
    root_headers = resolved_groups.get("byul", set())
    wrappers = wrapper_headers()
    consumers = header_include_consumers(headers)
    header_sources = {
        relative(path): exported_symbols(
            path.read_text(encoding="utf-8", errors="replace")
        )
        for path in headers
    }
    definitions = source_definitions(header_sources, sources)
    rows = []
    for path in headers:
        path_text = relative(path)
        source = path.read_text(encoding="utf-8", errors="replace")
        owner, boundary = owner_and_boundary(path)
        types = representative_types(source)
        representative = next(
            (item for item in types if item.removesuffix("_t") == path.stem),
            types[0] if types else None,
        )
        role = classify_role(path, source, owner)
        disposition = disposition_candidate(path, role, representative, boundary)
        rows.append(
            {
                "asset_id": f"initial:{path_text}",
                "current_path": path_text,
                "sha256": sha256(path),
                "line_count": len(source.splitlines()),
                "language": "c++-header" if path.suffix.lower() == ".hpp" else "c-header",
                "license": detect_license(source),
                "include_guard": detect_guard(source),
                "role_candidate": role,
                "current_owner": owner,
                "public_module_boundary_candidate": boundary,
                "responsibility": path.stem,
                "representative_type_candidate": representative,
                "declared_type_candidates": list(types),
                "naming_disposition": disposition,
                "compatibility_path": None,
                "successor_headers": [],
                "canonical_owner": None,
                "design_todo": None,
                "naming_exception": None,
                "module_header_groups": sorted(
                    name
                    for name, group_headers in resolved_groups.items()
                    if path_text in group_headers
                ),
                "root_header_inventory": path_text in root_headers,
                "install_inventory": path_text in root_headers,
                "umbrella_consumers": list(consumers.get(path_text, ())),
                "wrapper_modules_registered": path_text in wrappers,
                "exported_symbols": list(header_sources[path_text]),
                "source_definition_candidates": list(definitions[path_text]),
                "decision_status": (
                    "decision-required"
                    if disposition == "decision-required"
                    or role == "decision-required"
                    else "candidate-only"
                ),
            }
        )
    if len(rows) != len({row["asset_id"] for row in rows}):
        raise ValueError("header asset_id is not unique")
    return {
        "schema_version": 1,
        "source_scope": ["byul/**/*.h{,pp}", "tools/**/*.h{,pp}"],
        "disposition_values": sorted(DISPOSITION_VALUES),
        "summary": {
            "headers": len(rows),
            "byul_headers": sum(row["current_path"].startswith("byul/") for row in rows),
            "tool_headers": sum(row["current_path"].startswith("tools/") for row in rows),
            "root_inventory_headers": sum(row["root_header_inventory"] for row in rows),
            "wrapper_registered_headers": sum(
                row["wrapper_modules_registered"] for row in rows
            ),
            "decision_required": sum(
                row["decision_status"] == "decision-required" for row in rows
            ),
        },
        "headers": rows,
    }


def build_naming_report(manifest: dict[str, Any]) -> dict[str, Any]:
    installed = [
        row for row in manifest["headers"] if row["install_inventory"]
    ]
    basename_groups: dict[str, list[str]] = defaultdict(list)
    for row in installed:
        basename_groups[Path(row["current_path"]).name].append(row["current_path"])
    collisions = [
        {"basename": name, "paths": sorted(paths)}
        for name, paths in sorted(basename_groups.items())
        if len(paths) > 1
    ]
    repeated_chain = []
    type_mismatch = []
    generic = []
    for row in manifest["headers"]:
        path = Path(row["current_path"])
        stem = path.stem
        parts = path.parts[:-1]
        repeated = [
            part for part in parts if stem.startswith(f"{part}_")
        ]
        if stem.startswith("byul_") or len(repeated) >= 2:
            repeated_chain.append(
                {"path": row["current_path"], "repeated_segments": repeated}
            )
        representative = row["representative_type_candidate"]
        if representative and stem != representative.removesuffix("_t"):
            type_mismatch.append(
                {
                    "path": row["current_path"],
                    "representative_type_candidate": representative,
                }
            )
        if row["root_header_inventory"] and stem in GENERIC_COMPONENT_STEMS:
            generic.append(row["current_path"])
    return {
        "schema_version": 1,
        "sdk_public_basename_collisions": collisions,
        "physical_directory_chain_repetition_candidates": repeated_chain,
        "representative_type_stem_mismatch_candidates": type_mismatch,
        "generic_public_component_basename_candidates": sorted(generic),
        "counts": {
            "sdk_public_basename_collisions": len(collisions),
            "physical_directory_chain_repetition_candidates": len(repeated_chain),
            "representative_type_stem_mismatch_candidates": len(type_mismatch),
            "generic_public_component_basename_candidates": len(generic),
        },
    }


def first_line(result: dict[str, Any]) -> str | None:
    return result["output"].splitlines()[0] if result["output"] else None


def build_environment_snapshot() -> dict[str, Any]:
    tools = {}
    commands = {
        "git": ["git", "--version"],
        "cmake": ["cmake", "--version"],
        "python": [sys.executable, "--version"],
        "cl": ["cl"],
        "gcc": ["gcc", "--version"],
        "clang": ["clang", "--version"],
    }
    for name, command in commands.items():
        result = run_command(command)
        tools[name] = {
            "available": result["available"] and result["exit_code"] == 0,
            "version": first_line(result),
        }
    rules_path = REPOSITORY_ROOT / "external" / "rules"
    rules_sha = run_command(
        [
            "git",
            "-c",
            f"safe.directory={rules_path.as_posix()}",
            "-C",
            str(rules_path),
            "rev-parse",
            "HEAD",
        ]
    )
    return {
        "schema_version": 1,
        "captured_at_utc": datetime.now(timezone.utc).isoformat(),
        "git_head": git_lines("rev-parse", "HEAD")[0],
        "git_branch": git_lines("branch", "--show-current")[0],
        "git_status_porcelain": list(git_lines("status", "--porcelain")),
        "rules_submodule_sha": rules_sha["output"].strip(),
        "platform": {
            "system": platform.system(),
            "release": platform.release(),
            "machine": platform.machine(),
        },
        "tools": tools,
        "python_source": (
            "BYUL_PYTHON"
            if os.environ.get("BYUL_PYTHON")
            else "current-interpreter"
        ),
    }


def json_text(payload: dict[str, Any]) -> str:
    return json.dumps(payload, ensure_ascii=False, indent=2) + "\n"


def ensure_output_dir(path: Path) -> Path:
    resolved = path.resolve()
    try:
        resolved.relative_to(REPOSITORY_ROOT.resolve())
    except ValueError as error:
        raise ValueError("output directory must be inside the repository") from error
    return resolved


def atomic_write(path: Path, content: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        dir=path.parent, prefix=f".{path.name}.", suffix=".tmp", text=True
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8", newline="\n") as stream:
            stream.write(content)
            stream.flush()
            os.fsync(stream.fileno())
        temporary.replace(path)
    finally:
        if temporary.exists():
            temporary.unlink()


def check_output(path: Path, expected: dict[str, Any]) -> bool:
    if not path.is_file():
        print(f"[ERROR] missing baseline output: {relative(path)}", file=sys.stderr)
        return False
    try:
        actual = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        print(f"[ERROR] invalid baseline output {relative(path)}: {error}", file=sys.stderr)
        return False
    if actual != expected:
        print(f"[ERROR] stale baseline output: {relative(path)}", file=sys.stderr)
        return False
    return True


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "BYUL의 tracked header 135개를 기준으로 inventory, naming report와 "
            "실행 환경 snapshot을 생성한다."
        )
    )
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument(
        "--check",
        action="store_true",
        help="현재 inventory/naming 산출물이 source와 일치하는지만 검사한다.",
    )
    mode.add_argument(
        "--apply",
        action="store_true",
        help="검증된 baseline JSON 세 파일을 원자적으로 갱신한다.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=DEFAULT_OUTPUT_DIR,
        help="산출물 directory (기본: docs/ko/todo/header-refactor-baseline)",
    )
    args = parser.parse_args()

    output_dir = ensure_output_dir(args.output_dir)
    manifest = build_manifest()
    naming = build_naming_report(manifest)
    inventory_path = output_dir / "header-inventory.json"
    naming_path = output_dir / "naming-report.json"

    print(f"[MODE] {'check' if args.check else 'apply' if args.apply else 'dry-run'}")
    print("[SOURCE] tracked byul/tools headers")
    print(f"[OUTPUT] {relative(output_dir)}")
    print(
        "[SUMMARY] "
        f"headers={manifest['summary']['headers']} "
        f"byul={manifest['summary']['byul_headers']} "
        f"tools={manifest['summary']['tool_headers']} "
        f"root={manifest['summary']['root_inventory_headers']} "
        f"decision-required={manifest['summary']['decision_required']}"
    )

    if manifest["summary"]["headers"] != 135:
        print(
            f"[ERROR] expected 135 tracked headers, found {manifest['summary']['headers']}",
            file=sys.stderr,
        )
        return 1

    if args.check:
        valid = check_output(inventory_path, manifest)
        valid = check_output(naming_path, naming) and valid
        return 0 if valid else 1
    if not args.apply:
        print("[DRY-RUN] no files were written; use --apply to update outputs.")
        return 0

    environment = build_environment_snapshot()
    outputs = {
        inventory_path: manifest,
        naming_path: naming,
        output_dir / "environment.json": environment,
    }
    for path, payload in outputs.items():
        atomic_write(path, json_text(payload))
        print(f"[WRITTEN] {relative(path)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
