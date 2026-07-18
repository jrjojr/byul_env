"""Installed BYUL headers의 public struct layout과 enum 값을 측정한다."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = (
    REPOSITORY_ROOT
    / "docs"
    / "ko"
    / "todo"
    / "header-refactor-baseline"
    / "msvc-release-abi.json"
)
DEFAULT_BUILD_ROOT = REPOSITORY_ROOT / "build_win_msvc" / "abi_probe"


@dataclass(frozen=True)
class StructDeclaration:
    header: str
    name: str
    fields: tuple[str, ...]
    skipped_fields: tuple[str, ...]


@dataclass(frozen=True)
class EnumDeclaration:
    header: str
    name: str
    enumerators: tuple[str, ...]


def strip_comments(source: str) -> str:
    source = re.sub(r"/\*.*?\*/", "", source, flags=re.DOTALL)
    return re.sub(r"//[^\n]*", "", source)


def complete_typedefs(source: str, keyword: str) -> list[tuple[str, str]]:
    """중첩 brace를 보존하며 complete typedef body와 alias를 찾는다."""
    pattern = re.compile(
        rf"typedef\s+{keyword}(?:\s+[A-Za-z_][A-Za-z0-9_]*)?\s*\{{"
    )
    result = []
    for match in pattern.finditer(source):
        opening = source.find("{", match.start())
        depth = 0
        closing = None
        for index in range(opening, len(source)):
            if source[index] == "{":
                depth += 1
            elif source[index] == "}":
                depth -= 1
                if depth == 0:
                    closing = index
                    break
        if closing is None:
            continue
        alias = re.match(
            r"\s*([A-Za-z_][A-Za-z0-9_]*)\s*;", source[closing + 1 :]
        )
        if alias:
            result.append((source[opening + 1 : closing], alias.group(1)))
    return result


def top_level_declarations(body: str) -> tuple[str, ...]:
    result = []
    start = 0
    brace_depth = 0
    for index, character in enumerate(body):
        if character == "{":
            brace_depth += 1
        elif character == "}":
            brace_depth -= 1
        elif character == ";" and brace_depth == 0:
            result.append(body[start:index])
            start = index + 1
    return tuple(result)


def field_names(body: str) -> tuple[tuple[str, ...], tuple[str, ...]]:
    fields: list[str] = []
    skipped: list[str] = []
    for raw in top_level_declarations(body):
        declaration = " ".join(
            line.strip()
            for line in raw.splitlines()
            if line.strip() and not line.lstrip().startswith("#")
        ).strip()
        if not declaration:
            continue
        function_pointer = re.search(
            r"\(\s*\*\s*([A-Za-z_][A-Za-z0-9_]*)\s*\)", declaration
        )
        if function_pointer:
            name = function_pointer.group(1)
        else:
            normal = re.search(
                r"([A-Za-z_][A-Za-z0-9_]*)\s*"
                r"(?:\[[^\]]*\])?\s*(?::\s*\d+)?\s*$",
                declaration,
            )
            if not normal:
                skipped.append(declaration)
                continue
            name = normal.group(1)
        if ":" in declaration:
            skipped.append(declaration)
            continue
        if name not in fields:
            fields.append(name)
    return tuple(fields), tuple(skipped)


def enum_names(body: str) -> tuple[str, ...]:
    result = []
    for item in body.split(","):
        cleaned = " ".join(
            line.strip()
            for line in item.splitlines()
            if line.strip() and not line.lstrip().startswith("#")
        )
        match = re.match(r"([A-Za-z_][A-Za-z0-9_]*)", cleaned)
        if match and match.group(1) not in result:
            result.append(match.group(1))
    return tuple(result)


def declarations(
    include_root: Path,
) -> tuple[list[StructDeclaration], list[EnumDeclaration], list[str]]:
    structs = []
    enums = []
    opaque = []
    for path in sorted(include_root.glob("*.h")):
        source = strip_comments(path.read_text(encoding="utf-8", errors="replace"))
        for body, name in complete_typedefs(source, "struct"):
            fields, skipped = field_names(body)
            structs.append(
                StructDeclaration(path.name, name, fields, skipped)
            )
        for body, name in complete_typedefs(source, "enum"):
            enums.append(
                EnumDeclaration(
                    path.name,
                    name,
                    enum_names(body),
                )
            )
        for match in re.finditer(
            r"typedef\s+(?:struct|union)\s+[A-Za-z_][A-Za-z0-9_]*\s+"
            r"([A-Za-z_][A-Za-z0-9_]*)\s*;",
            source,
        ):
            opaque.append(f"{path.name}:{match.group(1)}")
    unique_structs = {
        item.name: item for item in structs
    }
    unique_enums = {
        item.name: item for item in enums
    }
    return (
        [unique_structs[name] for name in sorted(unique_structs)],
        [unique_enums[name] for name in sorted(unique_enums)],
        sorted(set(opaque) - {f"{item.header}:{item.name}" for item in structs}),
    )


def generate_source(
    include_root: Path,
    structs: list[StructDeclaration],
    enums: list[EnumDeclaration],
) -> str:
    headers = ["byul.h"] + [
        path.name
        for path in sorted(include_root.glob("*.h"))
        if path.name != "byul.h"
    ]
    lines = [
        "#define BYUL_STATIC 1",
        "#include <cstddef>",
        "#include <cstdio>",
        *[f'#include "{header}"' for header in headers],
        "",
        "int main() {",
    ]
    for item in structs:
        lines.append(
            f'  std::printf("TYPE\\t{item.name}\\t%zu\\t%zu\\n", '
            f"sizeof({item.name}), alignof({item.name}));"
        )
        for field in item.fields:
            lines.append(
                f'  std::printf("FIELD\\t{item.name}\\t{field}\\t%zu\\n", '
                f"offsetof({item.name}, {field}));"
            )
    for item in enums:
        for enumerator in item.enumerators:
            lines.append(
                f'  std::printf("ENUM\\t{item.name}\\t{enumerator}\\t%lld\\n", '
                f"static_cast<long long>({enumerator}));"
            )
    lines.extend(("  return 0;", "}", ""))
    return "\n".join(lines)


def run(arguments: list[str], *, cwd: Path | None = None) -> str:
    completed = subprocess.run(
        arguments,
        cwd=cwd,
        check=False,
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
    )
    if completed.returncode != 0:
        raise RuntimeError(
            f"command failed ({completed.returncode}): {' '.join(arguments)}\n"
            f"{completed.stdout}\n{completed.stderr}"
        )
    return completed.stdout


def parse_probe(output: str) -> tuple[dict[str, dict[str, object]], dict[str, dict[str, int]]]:
    types: dict[str, dict[str, object]] = {}
    enums: dict[str, dict[str, int]] = {}
    for line in output.splitlines():
        parts = line.split("\t")
        if len(parts) == 4 and parts[0] == "TYPE":
            types[parts[1]] = {
                "size": int(parts[2]),
                "align": int(parts[3]),
                "fields": {},
            }
        elif len(parts) == 4 and parts[0] == "FIELD":
            types[parts[1]]["fields"][parts[2]] = int(parts[3])
        elif len(parts) == 4 and parts[0] == "ENUM":
            enums.setdefault(parts[1], {})[parts[2]] = int(parts[3])
    return types, enums


def capture(
    cmake: Path,
    install_root: Path,
    build_root: Path,
) -> dict[str, object]:
    include_root = install_root / "include" / "byul"
    if not include_root.is_dir():
        raise FileNotFoundError(f"installed include root not found: {include_root}")
    structs, enums, opaque = declarations(include_root)
    source_root = build_root / "source"
    binary_root = build_root / "build"
    source_root.mkdir(parents=True, exist_ok=True)
    source = generate_source(include_root, structs, enums)
    (source_root / "abi_probe.cpp").write_text(
        source, encoding="utf-8", newline="\n"
    )
    cmake_lists = f"""\
cmake_minimum_required(VERSION 3.20)
project(byul_abi_probe LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_executable(byul_abi_probe abi_probe.cpp)
target_include_directories(byul_abi_probe PRIVATE "{include_root.as_posix()}")
if(MSVC)
    target_compile_options(byul_abi_probe PRIVATE /utf-8)
endif()
"""
    (source_root / "CMakeLists.txt").write_text(
        cmake_lists, encoding="utf-8", newline="\n"
    )
    run(
        [
            str(cmake),
            "-S",
            str(source_root),
            "-B",
            str(binary_root),
            "--fresh",
            "-G",
            "Visual Studio 17 2022",
            "-A",
            "x64",
        ]
    )
    run([str(cmake), "--build", str(binary_root), "--config", "Release"])
    executable = binary_root / "Release" / "byul_abi_probe.exe"
    output = run([str(executable)])
    type_values, enum_values = parse_probe(output)
    if len(type_values) != len(structs) or len(enum_values) != len(enums):
        raise ValueError("probe output does not cover every parsed struct/enum")
    return {
        "schema_version": 1,
        "profile": "windows-msvc-x64-release",
        "types": [
            {
                "header": item.header,
                "name": item.name,
                **type_values[item.name],
                "skipped_field_declarations": list(item.skipped_fields),
            }
            for item in structs
        ],
        "enums": [
            {
                "header": item.header,
                "name": item.name,
                "values": enum_values[item.name],
            }
            for item in enums
        ],
        "forward_or_opaque_declarations": opaque,
        "summary": {
            "complete_types": len(structs),
            "fields": sum(len(item.fields) for item in structs),
            "skipped_field_declarations": sum(
                len(item.skipped_fields) for item in structs
            ),
            "enums": len(enums),
            "enumerators": sum(len(item.enumerators) for item in enums),
            "forward_or_opaque_declarations": len(opaque),
        },
    }


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


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Installed BYUL headers의 MSVC x64 layout/enum ABI를 측정한다."
    )
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--apply", action="store_true")
    mode.add_argument("--check", action="store_true")
    parser.add_argument("--cmake", type=Path, required=True)
    parser.add_argument("--install-root", type=Path, required=True)
    parser.add_argument("--build-root", type=Path, default=DEFAULT_BUILD_ROOT)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()

    snapshot = capture(
        args.cmake.resolve(),
        args.install_root.resolve(),
        args.build_root.resolve(),
    )
    text = json.dumps(snapshot, ensure_ascii=False, indent=2) + "\n"
    print("[SUMMARY] " + " ".join(
        f"{key}={value}" for key, value in snapshot["summary"].items()
    ))
    output = args.output.resolve()
    if args.check:
        if not output.is_file() or output.read_text(encoding="utf-8") != text:
            print(f"[ERROR] stale ABI snapshot: {output}", file=sys.stderr)
            return 1
        return 0
    atomic_write(output, text)
    print(f"[WRITTEN] {output.relative_to(REPOSITORY_ROOT)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
