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

from byul_wrapper_generator import parse_header


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


def build_inventory(role_manifest: dict, vocabulary: dict) -> dict:
    header_rows = [
        row
        for row in role_manifest["headers"]
        if row["current_path"].replace("\\", "/").startswith("byul/navsys/")
    ]
    mappings = vocabulary["legacy_return_mapping"]
    symbols: list[dict] = []
    headers: list[dict] = []

    for header in sorted(header_rows, key=lambda row: row["current_path"]):
        relative = header["current_path"].replace("\\", "/")
        declarations = parse_header(REPOSITORY_ROOT / relative)
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
        headers.append(
            {
                "path": relative,
                "primary_role": header["primary_role"],
                "wrapper_mode": header["wrapper"]["mode"],
                "symbol_count": len(header_symbols),
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
        },
        "known_debt": {
            "duplicate_declarations": duplicate_declarations,
        },
        "headers": headers,
        "symbols": symbols,
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
