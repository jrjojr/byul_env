"""Built BYUL DLL export와 clean install inventory를 snapshot한다."""

from __future__ import annotations

import argparse
import hashlib
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
    / "msvc-release-build.json"
)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    digest.update(path.read_bytes())
    return digest.hexdigest()


def parse_dumpbin_exports(output: str) -> list[dict[str, object]]:
    """dumpbin /exports ordinal table을 parse한다."""
    exports = []
    pattern = re.compile(
        r"^\s*(\d+)\s+([0-9A-Fa-f]+)\s+([0-9A-Fa-f]+)\s+(\S+)",
        re.MULTILINE,
    )
    for match in pattern.finditer(output):
        exports.append(
            {
                "ordinal": int(match.group(1)),
                "hint": match.group(2).upper(),
                "rva": match.group(3).upper(),
                "name": match.group(4),
            }
        )
    return exports


def run_dumpbin(dumpbin: Path, dll: Path) -> tuple[str, list[dict[str, object]]]:
    completed = subprocess.run(
        [str(dumpbin), "/exports", str(dll)],
        check=False,
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
    )
    if completed.returncode != 0:
        raise RuntimeError(
            f"dumpbin failed ({completed.returncode}): {completed.stderr.strip()}"
        )
    exports = parse_dumpbin_exports(completed.stdout)
    if not exports:
        raise ValueError("dumpbin output did not contain an export table")
    version = next(
        (line.strip() for line in completed.stdout.splitlines() if line.strip()),
        "unknown",
    )
    return version, exports


def parse_nm_exports(output: str) -> list[dict[str, object]]:
    """Parse POSIX ``nm -P`` output and retain defined external symbols."""
    exports: list[dict[str, object]] = []
    for line in output.splitlines():
        parts = line.split()
        if len(parts) < 2:
            continue
        name, symbol_type = parts[0], parts[1]
        if (
            symbol_type.upper() == "U"
            or not name
            or name.startswith("__imp_")
        ):
            continue
        exports.append({"name": name, "type": symbol_type})
    return exports


def run_nm(nm: Path, library: Path) -> tuple[str, list[dict[str, object]]]:
    environment = os.environ.copy()
    environment["PATH"] = (
        str(nm.parent) + os.pathsep + environment.get("PATH", "")
    )
    completed = subprocess.run(
        [
            str(nm),
            "-P",
            "--defined-only",
            "--extern-only",
            str(library),
        ],
        check=False,
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        env=environment,
    )
    if completed.returncode != 0:
        raise RuntimeError(
            f"nm failed ({completed.returncode}): {completed.stderr.strip()}"
        )
    exports = parse_nm_exports(completed.stdout)
    if not exports:
        raise ValueError("nm output did not contain defined external symbols")
    version_result = subprocess.run(
        [str(nm), "--version"],
        check=False,
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        env=environment,
    )
    version = next(
        (line.strip() for line in version_result.stdout.splitlines() if line.strip()),
        "unknown",
    )
    return version, exports


def installed_files(root: Path) -> list[dict[str, object]]:
    return [
        {
            "path": path.relative_to(root).as_posix(),
            "size": path.stat().st_size,
            "sha256": sha256(path),
        }
        for path in sorted(root.rglob("*"))
        if path.is_file()
    ]


def build_snapshot(
    export_tool: Path,
    library: Path,
    install_root: Path,
    *,
    tool_kind: str = "dumpbin",
    profile: str = "windows-msvc-release",
) -> dict[str, object]:
    for label, path in (
        (tool_kind, export_tool),
        ("library", library),
        ("install root", install_root),
    ):
        if not path.exists():
            raise FileNotFoundError(f"{label} not found: {path}")
    if tool_kind == "dumpbin":
        tool_version, exports = run_dumpbin(export_tool, library)
    elif tool_kind == "nm":
        tool_version, exports = run_nm(export_tool, library)
    else:
        raise ValueError(f"unsupported export tool: {tool_kind}")
    files = installed_files(install_root)
    headers = [
        item for item in files if str(item["path"]).startswith("include/byul/")
    ]
    tests = [
        item
        for item in files
        if re.fullmatch(r"bin/test_.*\.exe", str(item["path"]))
    ]
    names = [str(item["name"]) for item in exports]
    if len(names) != len(set(names)):
        raise ValueError("DLL export names are not unique")
    payload = {
        "schema_version": 1,
        "profile": profile,
        "dll": {
            "name": library.name,
            "size": library.stat().st_size,
            "sha256": sha256(library),
            "export_count": len(exports),
        },
        "exports": exports,
        "install": {
            "file_count": len(files),
            "header_count": len(headers),
            "test_executable_count": len(tests),
            "files": files,
        },
    }
    if tool_kind == "dumpbin":
        # Preserve the stage-1 MSVC snapshot schema byte-for-byte.
        payload["dumpbin_version"] = tool_version
        payload = {
            "schema_version": payload["schema_version"],
            "profile": payload["profile"],
            "dumpbin_version": payload["dumpbin_version"],
            "dll": payload["dll"],
            "exports": payload["exports"],
            "install": payload["install"],
        }
    else:
        payload["export_tool"] = tool_kind
        payload["export_tool_version"] = tool_version
    return payload


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
        description="MSVC Release BYUL DLL export와 clean install inventory를 snapshot한다."
    )
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--check", action="store_true", help="기존 JSON과 현재 build를 비교한다.")
    mode.add_argument("--apply", action="store_true", help="snapshot JSON을 원자적으로 갱신한다.")
    export_tools = parser.add_mutually_exclusive_group(required=True)
    export_tools.add_argument("--dumpbin", type=Path)
    export_tools.add_argument(
        "--nm",
        type=Path,
        help="GNU/LLVM nm; invoked with -P --defined-only --extern-only",
    )
    parser.add_argument("--dll", type=Path, required=True)
    parser.add_argument("--install-root", type=Path, required=True)
    parser.add_argument(
        "--profile",
        default="windows-msvc-release",
        help="host/target/toolchain/configuration identifier",
    )
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()

    output = args.output.resolve()
    try:
        output.relative_to(REPOSITORY_ROOT.resolve())
    except ValueError:
        parser.error("--output must be inside the repository")

    export_tool = args.dumpbin or args.nm
    assert export_tool is not None
    snapshot = build_snapshot(
        export_tool.resolve(),
        args.dll.resolve(),
        args.install_root.resolve(),
        tool_kind="dumpbin" if args.dumpbin else "nm",
        profile=args.profile,
    )
    text = json.dumps(snapshot, ensure_ascii=False, indent=2) + "\n"
    print(
        "[SUMMARY] "
        f"exports={snapshot['dll']['export_count']} "
        f"install-files={snapshot['install']['file_count']} "
        f"headers={snapshot['install']['header_count']} "
        f"tests={snapshot['install']['test_executable_count']}"
    )
    if args.check:
        if not output.is_file() or output.read_text(encoding="utf-8") != text:
            print(f"[ERROR] stale build snapshot: {output}", file=sys.stderr)
            return 1
        return 0
    if not args.apply:
        print("[DRY-RUN] no files were written; use --apply.")
        return 0
    atomic_write(output, text)
    print(f"[WRITTEN] {output.relative_to(REPOSITORY_ROOT)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
