#!/usr/bin/env python3
"""Generate/check the current Navsys public-symbol return vocabulary."""

from __future__ import annotations

import argparse
from collections import Counter
import json
from pathlib import Path
import re
import sys
import tempfile


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
WRAPPER_ROOT = REPOSITORY_ROOT / "tools/python/byul_wrapper"
sys.path.insert(0, str(WRAPPER_ROOT))

from byul_wrapper_generator import audit_header, parse_header


DEFAULT_ROLE_MANIFEST = (
    REPOSITORY_ROOT
    / "docs/ko/todo/header-refactor-current/header-role-manifest.json"
)
DEFAULT_VOCABULARY = (
    REPOSITORY_ROOT / "docs/ko/todo/navsys/navsys-abi-vocabulary.json"
)
DEFAULT_OUTPUT = (
    REPOSITORY_ROOT / "docs/ko/todo/navsys/navsys-current-abi-inventory.json"
)
OWNER_TODO_OVERRIDES = {
    "byul/navsys/navsys_status.h":
        "docs/ko/todo/navsys/todo-navsys-industry-api-baseline.org",
    "byul/navsys/coord/internal/coord_ops.hpp":
        "docs/ko/todo/navsys/todo-navsys-coord-coord-cpp.org",
    "byul/navsys/dstar_lite/internal/dstar_lite_callback.hpp":
        "docs/ko/todo/navsys/todo-navsys-dstar-lite-dstar-lite.org",
    "byul/navsys/navgrid/internal/navgrid_callback.hpp":
        "docs/ko/todo/navsys/todo-navsys-navgrid-navgrid.org",
}


def load_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def classify_return(name: str, return_type: str) -> str:
    normalized = re.sub(r"\s+", " ", return_type).strip()
    if "*" in normalized:
        return "nullable_pointer"
    if normalized == "void":
        return "void_fallible_operation"
    if normalized == "bool":
        if re.search(r"(?:^|_)(?:is|has|equal|equals|contains)(?:_|$)", name):
            return "bool_predicate"
        return "bool_operation"
    if normalized in {"float", "double"}:
        return "floating_result"
    if normalized in {"size_t", "unsigned int", "unsigned long", "unsigned long long"}:
        return "int_count"
    if normalized == "int":
        if re.search(r"(?:^|_)(?:find|index)(?:_|$)", name):
            return "int_index_minus_one"
        if re.search(r"(?:^|_)(?:length|count|size)(?:_|$)", name):
            return "int_count"
        return "int_zero_nonzero_status"
    if normalized.endswith("_t"):
        return "enum_result"
    return "enum_result"


def classify_lifecycle(name: str) -> tuple[str, str] | None:
    """Return ``(resource, operation)`` for ownership vocabulary symbols."""
    create_or_init = re.match(r"^(.+)_(create|init)(?:_.+)?$", name)
    if create_or_init:
        return create_or_init.group(1), create_or_init.group(2)
    terminal = re.match(r"^(.+)_(copy|reset|free|destroy)$", name)
    if terminal:
        return terminal.group(1), terminal.group(2)
    return None


def classify_callback_binding(name: str) -> tuple[str, str, str, str] | None:
    """Return family, normalized slot, access and component for callback APIs."""
    match = re.match(r"^(.+?)_(set|get)_(.+)$", name)
    if not match:
        return None
    family, access, slot = match.groups()
    if not re.search(r"(?:func|fn)", slot):
        return None
    component = "userdata" if slot.endswith("_userdata") else "function"
    slot = re.sub(r"_userdata$", "", slot)
    slot = re.sub(r"(?:func|fn)", "callback", slot)
    return family, slot, access, component


def owner_todo_path(header_path: str) -> str:
    if header_path in OWNER_TODO_OVERRIDES:
        return OWNER_TODO_OVERRIDES[header_path]
    path = Path(header_path)
    parts = list(path.parts[2:-1])
    stem = path.stem
    if parts == ["maze"] and stem.startswith("maze_"):
        stem = stem.removeprefix("maze_")
    parts.append(stem)
    if path.suffix == ".hpp":
        parts.append("cpp")
    slug = "-".join(part.replace("_", "-") for part in parts)
    return f"docs/ko/todo/navsys/todo-navsys-{slug}.org"


def build_inventory(role_manifest: dict, vocabulary: dict) -> dict:
    header_rows = [
        row
        for row in role_manifest["headers"]
        if row["current_path"].replace("\\", "/").startswith("byul/navsys/")
    ]
    mappings = vocabulary["legacy_return_mapping"]
    symbols: list[dict] = []
    headers: list[dict] = []
    audit_debt: list[dict] = []
    lifecycle_operations: list[dict] = []
    callback_operations: list[dict] = []

    for header in sorted(header_rows, key=lambda row: row["current_path"]):
        relative = header["current_path"].replace("\\", "/")
        absolute = REPOSITORY_ROOT / relative
        declarations = parse_header(absolute)
        audit = audit_header(absolute)
        owner_todo = owner_todo_path(relative)
        if not (REPOSITORY_ROOT / owner_todo).is_file():
            raise ValueError(f"missing owner TODO for {relative}: {owner_todo}")
        header_symbols = []
        for declaration in declarations:
            return_form = classify_return(
                declaration.name,
                declaration.return_type,
            )
            symbol = {
                "header": relative,
                "name": declaration.name,
                "return_type": declaration.return_type,
                "return_form": return_form,
                "canonical": mappings[return_form]["canonical"],
            }
            symbols.append(symbol)
            header_symbols.append(declaration.name)
            lifecycle = classify_lifecycle(declaration.name)
            if lifecycle:
                resource, operation = lifecycle
                lifecycle_operations.append(
                    {
                        "header": relative,
                        "resource": resource,
                        "operation": operation,
                        "symbol": declaration.name,
                        "return_type": declaration.return_type,
                        "parameters": [
                            parameter.declaration
                            for parameter in declaration.parameters
                        ],
                    }
                )
            callback = classify_callback_binding(declaration.name)
            if callback:
                family, slot, access, component = callback
                callback_operations.append(
                    {
                        "header": relative,
                        "family": family,
                        "slot": slot,
                        "access": access,
                        "component": component,
                        "symbol": declaration.name,
                    }
                )
        for issue in audit.issues:
            audit_debt.append(
                {
                    "header": relative,
                    "owner_todo": owner_todo,
                    "severity": issue.severity,
                    "code": issue.code,
                    "symbol": issue.symbol,
                    "message": issue.message,
                    "disposition": "route-to-owner-child",
                }
            )
        headers.append(
            {
                "path": relative,
                "owner_todo": owner_todo,
                "primary_role": header["primary_role"],
                "wrapper_mode": header["wrapper"]["mode"],
                "symbol_count": len(header_symbols),
                "audit_issue_count": len(audit.issues),
                "symbols": header_symbols,
            }
        )

    form_counts = {
        form: sum(symbol["return_form"] == form for symbol in symbols)
        for form in mappings
    }
    duplicate_declarations = []
    for header in headers:
        counts = Counter(header["symbols"])
        duplicate_declarations.extend(
            {
                "header": header["path"],
                "symbol": name,
                "count": count,
                "disposition": "route-to-owner-child",
            }
            for name, count in sorted(counts.items())
            if count > 1
        )
    audit_codes = dict(
        sorted(Counter(issue["code"] for issue in audit_debt).items())
    )
    lifecycle_resources = []
    for resource in sorted({row["resource"] for row in lifecycle_operations}):
        operations = [
            row for row in lifecycle_operations if row["resource"] == resource
        ]
        lifecycle_resources.append(
            {
                "resource": resource,
                "operations": operations,
                "operation_kinds": sorted(
                    {row["operation"] for row in operations}
                ),
            }
        )
    callback_bindings = []
    callback_keys = sorted(
        {(row["family"], row["slot"]) for row in callback_operations}
    )
    for family, slot in callback_keys:
        operations = [
            row
            for row in callback_operations
            if row["family"] == family and row["slot"] == slot
        ]
        setters = {
            row["component"]
            for row in operations
            if row["access"] == "set"
        }
        callback_bindings.append(
            {
                "family": family,
                "slot": slot,
                "operations": operations,
                "setter_components": sorted(setters),
                "current_atomicity": (
                    "split-function-and-userdata-setters"
                    if setters == {"function", "userdata"}
                    else "partial-export-surface"
                ),
            }
        )
    return {
        "schema_version": 1,
        "source_role_manifest": str(
            DEFAULT_ROLE_MANIFEST.relative_to(REPOSITORY_ROOT)
        ).replace("\\", "/"),
        "source_vocabulary": str(
            DEFAULT_VOCABULARY.relative_to(REPOSITORY_ROOT)
        ).replace("\\", "/"),
        "summary": {
            "headers": len(headers),
            "symbols": len(symbols),
            "return_forms": form_counts,
            "unclassified": 0,
            "duplicate_declarations": len(duplicate_declarations),
            "audit_issues": len(audit_debt),
            "audit_errors": sum(
                issue["severity"] == "error" for issue in audit_debt
            ),
            "audit_codes": audit_codes,
            "lifecycle_resources": len(lifecycle_resources),
            "lifecycle_operations": len(lifecycle_operations),
            "callback_bindings": len(callback_bindings),
        },
        "known_debt": {
            "duplicate_declarations": duplicate_declarations,
            "header_audit": audit_debt,
        },
        "headers": headers,
        "symbols": symbols,
        "lifecycle": {
            "resources": lifecycle_resources,
            "operations": lifecycle_operations,
        },
        "callback_bindings": callback_bindings,
    }


def write_json_atomic(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    text = json.dumps(payload, ensure_ascii=False, indent=2) + "\n"
    with tempfile.NamedTemporaryFile(
        "w",
        encoding="utf-8",
        newline="\n",
        dir=path.parent,
        delete=False,
    ) as stream:
        stream.write(text)
        temporary = Path(stream.name)
    temporary.replace(path)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate/check the current Navsys ABI return inventory.",
    )
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--apply", action="store_true")
    mode.add_argument("--check", action="store_true")
    parser.add_argument("--role-manifest", type=Path, default=DEFAULT_ROLE_MANIFEST)
    parser.add_argument("--vocabulary", type=Path, default=DEFAULT_VOCABULARY)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    payload = build_inventory(
        load_json(args.role_manifest),
        load_json(args.vocabulary),
    )
    if args.apply:
        write_json_atomic(args.output, payload)
        print(
            f"[WRITTEN] {args.output} "
            f"headers={payload['summary']['headers']} "
            f"symbols={payload['summary']['symbols']}"
        )
        return 0

    if not args.output.is_file():
        print(f"[ERROR] missing inventory: {args.output}", file=sys.stderr)
        return 1
    current = load_json(args.output)
    if current != payload:
        print(f"[ERROR] stale inventory: {args.output}", file=sys.stderr)
        return 1
    print(
        f"[OK] {args.output} "
        f"headers={payload['summary']['headers']} "
        f"symbols={payload['summary']['symbols']}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
