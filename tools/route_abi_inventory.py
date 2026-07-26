#!/usr/bin/env python3
"""Generate or check the Route ABI and consumer inventory."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys
import tempfile
from typing import Any


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
HEADER = Path("byul/navsys/route/route.h")
SOURCE = Path("byul/navsys/route/route.cpp")
MODULE_CMAKE = Path("byul/navsys/route/CMakeLists.txt")
ROLE_MANIFEST = Path(
    "docs/ko/todo/header-refactor-current/header-role-manifest.json"
)
ABI_INVENTORY = Path("docs/ko/todo/navsys/navsys-current-abi-inventory.json")
DEFAULT_OUTPUT = Path("docs/ko/todo/navsys/route-abi-consumer-inventory.json")
INSTALL_PATH = Path("include/byul/route.h")
SOURCE_SUFFIXES = {".c", ".cc", ".cpp", ".cxx", ".h", ".hh", ".hpp", ".py"}

ROUTE_DIR_VALUES = {
    "ROUTE_DIR_UNKNOWN": 0,
    "ROUTE_DIR_RIGHT": 1,
    "ROUTE_DIR_UP_RIGHT": 2,
    "ROUTE_DIR_UP": 3,
    "ROUTE_DIR_UP_LEFT": 4,
    "ROUTE_DIR_LEFT": 5,
    "ROUTE_DIR_DOWN_LEFT": 6,
    "ROUTE_DIR_DOWN": 7,
    "ROUTE_DIR_DOWN_RIGHT": 8,
    "ROUTE_DIR_COUNT": 9,
}
ROUTE_COMPLETION_VALUES = {
    "ROUTE_COMPLETION_NONE": 0,
    "ROUTE_COMPLETION_COMPLETE": 1,
    "ROUTE_COMPLETION_PARTIAL": 2,
}
ROUTE_JOIN_POLICY_VALUES = {
    "ROUTE_JOIN_KEEP_ALL": 0,
    "ROUTE_JOIN_DEDUP_BOUNDARY": 1,
}
STRUCT_FIELDS = [
    ("coord_list_t*", "coords"),
    ("coord_list_t*", "visited_order"),
    ("coord_hash_t*", "visited_count"),
    ("float", "cost"),
    ("bool", "success"),
    ("int", "total_retry_count"),
    ("float", "avg_vec_x"),
    ("float", "avg_vec_y"),
    ("int", "vec_count"),
]
CANONICAL_SYMBOLS = {
    "route_clone_ex",
    "route_get_coord_count",
    "route_fetch_coord",
    "route_fetch_total_cost",
    "route_fetch_completion",
    "route_export_coords",
    "route_slice_ex",
    "route_reconstruct_ex",
    "route_builder_create",
    "route_builder_create_from_route",
    "route_builder_destroy",
    "route_builder_push_coord",
    "route_builder_insert_coord",
    "route_builder_remove_coord",
    "route_builder_append",
    "route_builder_assign_slice",
    "route_builder_set_total_cost",
    "route_builder_set_completion",
    "route_builder_finish",
    "navsys_search_trace_create",
    "navsys_search_trace_destroy",
    "navsys_search_trace_clone_ex",
    "navsys_search_trace_get_visit_count",
    "navsys_search_trace_fetch_visit",
    "navsys_search_trace_fetch_coord_visit_count",
    "navsys_search_trace_export_visits",
}


def relative(path: Path) -> str:
    return path.relative_to(REPOSITORY_ROOT).as_posix()


def normalized_sha256(path: Path) -> str:
    data = path.read_bytes().replace(b"\r\n", b"\n").replace(b"\r", b"\n")
    return hashlib.sha256(data).hexdigest()


def load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def source_files() -> list[Path]:
    result = subprocess.run(
        [
            "git",
            "ls-files",
            "-z",
            "--",
            "byul",
            "external",
            "tools/python/byul_wrapper",
        ],
        cwd=REPOSITORY_ROOT,
        check=True,
        capture_output=True,
    )
    paths = result.stdout.decode("utf-8").split("\0")
    return sorted(
        REPOSITORY_ROOT / path
        for path in paths
        if path and Path(path).suffix.lower() in SOURCE_SUFFIXES
    )


def classify(path: Path) -> str:
    name = relative(path)
    if name.startswith("external/"):
        return "external"
    if "/tests/" in name or path.name.startswith("test_"):
        return "test-only"
    if name.startswith("tools/python/byul_wrapper/byul_wrapper/"):
        return "wrapper"
    return "internal-production"


def read_text(path: Path) -> str:
    try:
        return path.read_text(encoding="utf-8", errors="ignore")
    except OSError:
        return ""


def matching_lines(text: str, pattern: re.Pattern[str]) -> list[int]:
    return [
        number
        for number, line in enumerate(text.splitlines(), 1)
        if pattern.search(line)
    ]


def parse_enum(text: str, tag: str) -> dict[str, int]:
    match = re.search(
        rf"typedef\s+enum\s+{re.escape(tag)}\s*\{{(?P<body>.*?)\}}\s*\w+\s*;",
        text,
        re.DOTALL,
    )
    if not match:
        return {}
    body = re.sub(r"/\*.*?\*/", "", match.group("body"), flags=re.DOTALL)
    value = -1
    result: dict[str, int] = {}
    for item in body.split(","):
        item = item.strip()
        if not item:
            continue
        item = re.sub(r"//.*", "", item).strip()
        if not item:
            continue
        if "=" in item:
            name, raw_value = (part.strip() for part in item.split("=", 1))
            value = int(raw_value, 0)
        else:
            name = item
            value += 1
        result[name] = value
    return result


def code_without_comments(text: str) -> str:
    text = re.sub(
        r"/\*.*?\*/",
        lambda match: "\n" * match.group(0).count("\n"),
        text,
        flags=re.DOTALL,
    )
    return re.sub(r"//[^\n]*", "", text)


def parse_struct_fields(text: str) -> list[tuple[str, str]]:
    match = re.search(r"struct\s+s_route\s*\{(?P<body>.*?)\};", text, re.DOTALL)
    if not match:
        return []
    fields: list[tuple[str, str]] = []
    for declaration in match.group("body").split(";"):
        declaration = " ".join(declaration.split())
        if not declaration:
            continue
        field_match = re.fullmatch(r"(?P<type>.+?\*?)\s*(?P<name>\w+)", declaration)
        if not field_match:
            continue
        field_type = field_match.group("type").replace(" *", "*")
        fields.append((field_type, field_match.group("name")))
    return fields


def build_consumer_matrix(symbols: list[str]) -> tuple[list[dict], list[dict]]:
    function_rows = {
        symbol: {category: [] for category in CONSUMER_CATEGORIES}
        for symbol in symbols
    }
    field_rows = {
        field: {category: [] for category in CONSUMER_CATEGORIES}
        for _, field in STRUCT_FIELDS
    }
    header_path = REPOSITORY_ROOT / HEADER
    source_path = REPOSITORY_ROOT / SOURCE
    symbol_patterns = {
        symbol: re.compile(rf"\b{re.escape(symbol)}\s*\(")
        for symbol in symbols
    }
    field_patterns = {
        field: re.compile(
            rf"(?P<receiver>[A-Za-z_]\w*)\s*(?:->|\.)\s*"
            rf"{re.escape(field)}\b(?!\s*\()"
        )
        for _, field in STRUCT_FIELDS
    }
    for path in source_files():
        if path in {header_path, source_path}:
            continue
        text = read_text(path)
        if not text:
            continue
        category = classify(path)
        path_name = relative(path)
        for symbol, pattern in symbol_patterns.items():
            lines = matching_lines(text, pattern)
            if lines:
                function_rows[symbol][category].append(
                    {"path": path_name, "lines": lines}
                )
        route_aware = re.search(r"\broute_t\b|\broute_[a-zA-Z0-9_]+\s*\(", text)
        if not route_aware:
            continue
        for field, pattern in field_patterns.items():
            if (
                path_name.startswith("byul/navsys/route/")
                and category == "internal-production"
            ):
                continue
            code = code_without_comments(text)
            lines = []
            for line_number, line in enumerate(code.splitlines(), start=1):
                matches = list(pattern.finditer(line))
                if any(
                    match.group("receiver") not in {
                        "grid",
                        "stats",
                        "out_stats",
                        "unchanged",
                        "nested_ex_stats",
                    }
                    for match in matches
                ):
                    lines.append(line_number)
            if lines:
                field_rows[field][category].append(
                    {"path": path_name, "lines": lines}
                )
    return (
        [
            {"name": symbol, "consumers": function_rows[symbol]}
            for symbol in symbols
        ],
        [
            {"name": field, "candidate_consumers": field_rows[field]}
            for _, field in STRUCT_FIELDS
        ],
    )


CONSUMER_CATEGORIES = (
    "internal-production",
    "test-only",
    "wrapper",
    "external",
)


def build_inventory(install_root: Path, build_snapshot: Path) -> dict[str, Any]:
    header_path = REPOSITORY_ROOT / HEADER
    header_text = read_text(header_path)
    abi = load_json(REPOSITORY_ROOT / ABI_INVENTORY)
    symbols = [
        row
        for row in abi["symbols"]
        if row["header"] == HEADER.as_posix()
    ]
    symbol_names = [row["name"] for row in symbols]
    legacy_symbols = [
        name for name in symbol_names if name not in CANONICAL_SYMBOLS
    ]
    function_consumers, field_consumers = build_consumer_matrix(symbol_names)

    manifest = load_json(REPOSITORY_ROOT / ROLE_MANIFEST)
    role = next(
        row
        for row in manifest["headers"]
        if row["current_path"] == HEADER.as_posix()
    )
    snapshot = load_json(build_snapshot)
    exports = {row["name"] for row in snapshot["exports"]}
    missing_exports = sorted(set(symbol_names) - exports)
    installed = install_root.resolve() / INSTALL_PATH
    cmake_text = read_text(REPOSITORY_ROOT / MODULE_CMAKE).replace("\\", "/")

    return {
        "schema_version": 1,
        "header": {
            "path": HEADER.as_posix(),
            "sha256": normalized_sha256(header_path),
            "function_count": len(symbol_names),
            "legacy_function_count": len(legacy_symbols),
            "canonical_function_count": len(CANONICAL_SYMBOLS),
            "functions": [
                {
                    "name": row["name"],
                    "signature": row["signature"],
                    "surface": (
                        "canonical-additive"
                        if row["name"] in CANONICAL_SYMBOLS
                        else "legacy-abi-1"
                    ),
                }
                for row in symbols
            ],
            "enums": {
                "route_dir_t": parse_enum(header_text, "e_route_dir"),
                "route_completion_t": parse_enum(
                    header_text, "e_route_completion"
                ),
                "route_join_policy_t": parse_enum(
                    header_text, "e_route_join_policy"
                ),
            },
            "struct": {
                "name": "route_t",
                "fields": [
                    {"type": field_type, "name": name}
                    for field_type, name in parse_struct_fields(header_text)
                ],
                "layout_profiles": {
                    "pointer-64": {
                        "size": 48,
                        "alignment": 8,
                        "offsets": {
                            "coords": 0,
                            "visited_order": 8,
                            "visited_count": 16,
                            "cost": 24,
                            "success": 28,
                            "total_retry_count": 32,
                            "avg_vec_x": 36,
                            "avg_vec_y": 40,
                            "vec_count": 44,
                        },
                    },
                    "pointer-32": {
                        "size": 36,
                        "alignment": 4,
                        "offsets": {
                            "coords": 0,
                            "visited_order": 4,
                            "visited_count": 8,
                            "cost": 12,
                            "success": 16,
                            "total_retry_count": 20,
                            "avg_vec_x": 24,
                            "avg_vec_y": 28,
                            "vec_count": 32,
                        },
                    },
                },
            },
        },
        "distribution": {
            "module_public_header_inventory": "/route.h" in cmake_text,
            "manifest_primary_role": role["primary_role"],
            "manifest_approved_install": role["approved_install"],
            "manifest_wrapper_mode": role["wrapper"]["mode"],
            "clean_install_present": installed.is_file(),
            "root_export_expected": len(symbol_names),
            "root_export_missing": missing_exports,
        },
        "consumer_matrix": {
            "functions": function_consumers,
            "direct_field_candidates": field_consumers,
        },
        "decisions": {
            "result_value": "coords+completion+cost",
            "builder": "separate-status-checked-owner",
            "search_trace_and_stats": "separate-from-route-value",
            "heading_tracker": "separate-state-owner",
            "empty": "zero-coordinates+completion-none",
            "start_equals_goal": "one-coordinate+completion-complete",
            "partial": "coordinates-present+completion-partial",
            "no_path": "zero-coordinates+completion-none",
        },
    }


def validate(payload: dict[str, Any]) -> list[str]:
    errors: list[str] = []
    header = payload["header"]
    if header["function_count"] != 70:
        errors.append("route.h function inventory is not exactly 70")
    if header["legacy_function_count"] != 44:
        errors.append("legacy Route function inventory is not exactly 44")
    if header["canonical_function_count"] != 26:
        errors.append("canonical Route function inventory is not exactly 26")
    if header["enums"]["route_dir_t"] != ROUTE_DIR_VALUES:
        errors.append("route_dir_t numeric values changed")
    if header["enums"]["route_completion_t"] != ROUTE_COMPLETION_VALUES:
        errors.append("route_completion_t numeric values changed")
    if header["enums"]["route_join_policy_t"] != ROUTE_JOIN_POLICY_VALUES:
        errors.append("route_join_policy_t numeric values changed")
    observed_fields = [
        (row["type"], row["name"]) for row in header["struct"]["fields"]
    ]
    if observed_fields != STRUCT_FIELDS:
        errors.append("route_t public field layout changed")
    distribution = payload["distribution"]
    if not distribution["module_public_header_inventory"]:
        errors.append("route.h is missing from the module public header inventory")
    if distribution["manifest_primary_role"] != "public-component":
        errors.append("route.h is not classified as a public component")
    if not distribution["manifest_approved_install"]:
        errors.append("route.h is not approved for SDK installation")
    if distribution["manifest_wrapper_mode"] != "generated":
        errors.append("route.h is not registered for generated wrapper input")
    if not distribution["clean_install_present"]:
        errors.append("route.h is missing from the clean SDK install")
    if distribution["root_export_missing"]:
        errors.append(
            "Route declarations are missing from the root export snapshot: "
            + ", ".join(distribution["root_export_missing"])
        )
    return errors


def write_json_atomic(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
        "w", encoding="utf-8", newline="\n", dir=path.parent, delete=False
    ) as stream:
        json.dump(payload, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
        temporary = Path(stream.name)
    temporary.replace(path)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--apply", action="store_true")
    mode.add_argument("--check", action="store_true")
    parser.add_argument("--install-root", type=Path, required=True)
    parser.add_argument("--build-snapshot", type=Path, required=True)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    output = (
        args.output
        if args.output.is_absolute()
        else REPOSITORY_ROOT / args.output
    )
    snapshot = (
        args.build_snapshot
        if args.build_snapshot.is_absolute()
        else REPOSITORY_ROOT / args.build_snapshot
    )
    payload = build_inventory(args.install_root, snapshot)
    errors = validate(payload)
    if errors:
        for error in errors:
            print(f"[ERROR] {error}", file=sys.stderr)
        return 1
    if args.apply:
        write_json_atomic(output, payload)
        print(f"[WRITTEN] {output} functions=70 legacy=44 canonical=26")
        return 0
    if not output.is_file() or load_json(output) != payload:
        print(f"[ERROR] stale inventory: {output}", file=sys.stderr)
        return 1
    print(f"[OK] {output} functions=70 legacy=44 canonical=26")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
