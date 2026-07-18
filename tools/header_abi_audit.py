#!/usr/bin/env python3
"""Cross-check BYUL header, ABI, install and wrapper ownership.

The stage-1 audit debt is an explicit baseline, not a reason to disable the
gate.  Set invariants always fail closed.  Header lint fingerprints may be
grandfathered only when they are present in the reviewed baseline; touched
headers can be made strict with ``--strict-header``.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile
from typing import Any, Iterable


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
CURRENT_DIR = REPOSITORY_ROOT / "docs/ko/todo/header-refactor-current"
DEFAULT_MANIFEST = CURRENT_DIR / "header-role-manifest.json"
DEFAULT_BUILD_SNAPSHOT = CURRENT_DIR / "msvc-release-build.json"
DEFAULT_ABI_SNAPSHOT = CURRENT_DIR / "msvc-release-abi.json"
DEFAULT_METADATA_BASELINE = CURRENT_DIR / "header-audit.json"
DEFAULT_DEBT_BASELINE = CURRENT_DIR / "header-abi-audit-debt.json"
DEFAULT_EXCLUSIONS = CURRENT_DIR / "header-abi-audit-exclusions.json"
LICENSE_PATH = REPOSITORY_ROOT / "docs/ko/byul-sdk/header-license-block.txt"
PUBLIC_ROLES = {
    "public-foundation",
    "public-module-entry",
    "public-aggregate",
    "public-component",
}


@dataclass(frozen=True)
class Finding:
    code: str
    subject: str
    message: str

    @property
    def fingerprint(self) -> str:
        return f"{self.code}|{self.subject}|{self.message}"


def load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def atomic_write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    text = json.dumps(payload, ensure_ascii=False, indent=2) + "\n"
    descriptor, temporary = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=path.parent
    )
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8", newline="\n") as stream:
            stream.write(text)
        os.replace(temporary, path)
    except BaseException:
        Path(temporary).unlink(missing_ok=True)
        raise


def repository_header_paths() -> set[str]:
    roots = (
        REPOSITORY_ROOT / "byul",
        REPOSITORY_ROOT / "tools/gpu_comp_tester",
        REPOSITORY_ROOT / "tools/unit_1003",
    )
    paths: set[str] = set()
    for root in roots:
        if not root.is_dir():
            continue
        for suffix in ("*.h", "*.hpp"):
            for path in root.rglob(suffix):
                paths.add(path.relative_to(REPOSITORY_ROOT).as_posix())
    return paths


def manifest_findings(manifest: dict[str, Any]) -> list[Finding]:
    findings: list[Finding] = []
    headers = manifest.get("headers", [])
    current_paths = [row.get("current_path") for row in headers]
    asset_ids = [row.get("asset_id") for row in headers]

    for label, values in (("current path", current_paths), ("asset id", asset_ids)):
        duplicates = sorted(
            value for value in set(values) if value is None or values.count(value) > 1
        )
        for value in duplicates:
            findings.append(
                Finding("duplicate-manifest-row", str(value), f"duplicate {label}")
            )

    actual = repository_header_paths()
    approved = set(current_paths)
    for path in sorted(actual - approved):
        findings.append(
            Finding("unmanifested-header", path, "repository header is not in manifest")
        )
    for path in sorted(approved - actual):
        findings.append(
            Finding("missing-manifest-header", path, "manifest header does not exist")
        )

    successor_ids = {
        row.get("asset_id") for row in manifest.get("created_successors", [])
    }
    for row in headers:
        path = row.get("current_path", "<unknown>")
        if row.get("decision_status") != "approved":
            findings.append(
                Finding("unapproved-header", path, "header decision is not approved")
            )
        wrapper = row.get("wrapper", {})
        if wrapper.get("mode") not in {"generated", "manual", "excluded"}:
            findings.append(
                Finding("invalid-wrapper-mode", path, "wrapper mode is not terminal")
            )
        if not wrapper.get("reason"):
            findings.append(
                Finding("missing-wrapper-reason", path, "wrapper reason is empty")
            )
        naming = row.get("naming", {})
        if not naming.get("canonical_path"):
            findings.append(
                Finding("missing-canonical-path", path, "canonical path is empty")
            )
        if naming.get("disposition") == "forward":
            if naming.get("compatibility_path") != path:
                findings.append(
                    Finding(
                        "invalid-compatibility-path",
                        path,
                        "forwarder does not preserve the current include path",
                    )
                )
            if not naming.get("deprecated_since") or not naming.get("remove_in"):
                findings.append(
                    Finding(
                        "missing-deprecation-window",
                        path,
                        "forwarder lacks deprecation or removal version",
                    )
                )
        for successor in row.get("successor_headers", []):
            if successor not in successor_ids:
                findings.append(
                    Finding(
                        "unknown-successor",
                        path,
                        f"successor asset is not registered: {successor}",
                    )
                )

    future_paths: list[str] = []
    for row in headers:
        if not row.get("approved_install"):
            continue
        naming = row["naming"]
        if naming.get("canonical_install"):
            future_paths.append(naming["canonical_path"])
        if naming.get("compatibility_path"):
            future_paths.append(naming["compatibility_path"])
    for successor in manifest.get("created_successors", []):
        future_paths.append(successor["canonical_path"])
        todo = REPOSITORY_ROOT / successor.get("design_todo", "")
        if not todo.is_file():
            findings.append(
                Finding(
                    "missing-successor-design",
                    successor.get("asset_id", "<unknown>"),
                    "successor design TODO does not exist",
                )
            )
    for migration in manifest.get("public_resource_migrations", []):
        future_paths.append(migration["abi_2_0"]["canonical_header"])
        todo = REPOSITORY_ROOT / migration.get("design_todo", "")
        if not todo.is_file():
            findings.append(
                Finding(
                    "missing-resource-design",
                    migration.get("resource", "<unknown>"),
                    "resource migration design TODO does not exist",
                )
            )

    by_basename: dict[str, list[str]] = {}
    for path in sorted(set(future_paths)):
        by_basename.setdefault(Path(path).name, []).append(path)
    for basename, paths in by_basename.items():
        if len(paths) > 1:
            findings.append(
                Finding(
                    "sdk-basename-collision",
                    basename,
                    f"future SDK paths collide: {', '.join(paths)}",
                )
            )
    if manifest.get("remaining_stage_2_design_gate"):
        findings.append(
            Finding(
                "open-boundary-gate",
                "manifest",
                "stage-2 design gate remains open",
            )
        )
    return findings


def _expected_guard(path: Path) -> str:
    stem = re.sub(r"[^A-Za-z0-9]+", "_", path.stem).upper()
    return f"BYUL_{stem}_H"


def structural_findings(path: Path) -> list[Finding]:
    relative = path.relative_to(REPOSITORY_ROOT).as_posix()
    source = path.read_text(encoding="utf-8", errors="replace")
    findings: list[Finding] = []
    license_text = LICENSE_PATH.read_text(encoding="utf-8").rstrip("\r\n")
    if not source.startswith(license_text + "\n\n"):
        findings.append(
            Finding(
                "header-license",
                relative,
                "header does not begin with the canonical license block and one blank line",
            )
        )
    if not re.search(
        rf"/\*\*.*?@file\s+{re.escape(path.name)}(?:\s|$).*?\*/",
        source,
        re.DOTALL,
    ):
        findings.append(
            Finding("file-doxygen", relative, "canonical @file Doxygen block is missing")
        )
    guard = _expected_guard(path)
    if not re.search(rf"^\s*#ifndef\s+{re.escape(guard)}\s*$", source, re.MULTILINE):
        findings.append(
            Finding("include-guard", relative, f"expected include guard {guard}")
        )
    if "BYUL_API" in source or re.search(r"typedef\s+.*\(\s*\*", source):
        if not re.search(r'#ifdef\s+__cplusplus\s+extern\s+"C"\s*\{', source, re.DOTALL):
            findings.append(
                Finding("c-linkage", relative, 'extern "C" boundary is missing')
            )
    extern = source.find('extern "C"')
    if extern >= 0 and re.search(r"^\s*#include\b", source[extern:], re.MULTILINE):
        findings.append(
            Finding(
                "include-inside-c-linkage",
                relative,
                "an include appears after the extern C boundary begins",
            )
        )
    return findings


def _load_header_parser() -> Any:
    wrapper_root = REPOSITORY_ROOT / "tools/python/byul_wrapper"
    sys.path.insert(0, str(wrapper_root))
    try:
        from byul_wrapper_generator import audit_header, parse_header
    finally:
        sys.path.pop(0)
    return audit_header, parse_header


def public_header_rows(manifest: dict[str, Any]) -> list[dict[str, Any]]:
    return [
        row
        for row in manifest["headers"]
        if row["primary_role"] in PUBLIC_ROLES
        and row["approved_install"]
        and row["current_path"].endswith(".h")
    ]


def metadata_findings(
    manifest: dict[str, Any],
) -> tuple[list[Finding], set[str]]:
    audit_header, parse_header = _load_header_parser()
    findings: list[Finding] = []
    symbols: set[str] = set()
    for row in public_header_rows(manifest):
        path = REPOSITORY_ROOT / row["current_path"]
        report = audit_header(path)
        symbols.update(item.name for item in parse_header(path))
        for issue in report.issues:
            findings.append(
                Finding(
                    f"metadata:{issue.code}",
                    f"{row['current_path']}:{issue.symbol or '-'}",
                    issue.message,
                )
            )
    return findings, symbols


def wrapper_registered_headers() -> set[str]:
    script = REPOSITORY_ROOT / "tools/python/byul_wrapper/generate_wrapper_abi.py"
    wrapper_root = script.parent
    sys.path.insert(0, str(wrapper_root))
    try:
        spec = importlib.util.spec_from_file_location("generate_wrapper_abi", script)
        if spec is None or spec.loader is None:
            raise RuntimeError(f"cannot load wrapper generator: {script}")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return {f"byul/{path}" for path in module.registered_headers()}
    finally:
        sys.path.pop(0)


def wrapper_findings(manifest: dict[str, Any]) -> list[Finding]:
    expected = {
        row["current_path"]
        for row in manifest["headers"]
        if row["wrapper"]["mode"] == "generated"
    }
    actual = wrapper_registered_headers()
    findings: list[Finding] = []
    for path in sorted(actual - expected):
        findings.append(
            Finding(
                "wrapper-row-missing",
                path,
                "generator header is not approved as wrapper generated",
            )
        )
    for path in sorted(expected - actual):
        findings.append(
            Finding(
                "wrapper-registration-missing",
                path,
                "approved generated header is absent from wrapper generator",
            )
        )
    return findings


def export_findings(symbols: set[str], snapshot: dict[str, Any]) -> list[Finding]:
    exports = {row["name"] for row in snapshot.get("exports", [])}
    return [
        Finding(
            "missing-root-export",
            symbol,
            "BYUL_API declaration is absent from the root shared export snapshot",
        )
        for symbol in sorted(symbols - exports)
    ]


def abi_snapshot_findings(
    snapshot: dict[str, Any],
) -> tuple[list[Finding], str]:
    findings: list[Finding] = []
    types = snapshot.get("types", [])
    enums = snapshot.get("enums", [])
    type_keys = [(row.get("header"), row.get("name")) for row in types]
    enum_keys = [(row.get("header"), row.get("name")) for row in enums]
    if len(type_keys) != len(set(type_keys)):
        findings.append(
            Finding("duplicate-abi-type", "abi-snapshot", "type keys are duplicated")
        )
    if len(enum_keys) != len(set(enum_keys)):
        findings.append(
            Finding("duplicate-abi-enum", "abi-snapshot", "enum keys are duplicated")
        )
    for row in types:
        subject = f"{row.get('header')}:{row.get('name')}"
        if not isinstance(row.get("size"), int) or row["size"] <= 0:
            findings.append(
                Finding("invalid-abi-size", subject, "sizeof must be positive")
            )
        if not isinstance(row.get("align"), int) or row["align"] <= 0:
            findings.append(
                Finding("invalid-abi-align", subject, "alignof must be positive")
            )
        for field, offset in row.get("fields", {}).items():
            if not isinstance(offset, int) or offset < 0:
                findings.append(
                    Finding(
                        "invalid-abi-offset",
                        f"{subject}:{field}",
                        "offsetof must be a non-negative integer",
                    )
                )
    summary = snapshot.get("summary", {})
    expected = {
        "complete_types": len(types),
        "fields": sum(len(row.get("fields", {})) for row in types),
        "skipped_field_declarations": sum(
            len(row.get("skipped_field_declarations", [])) for row in types
        ),
        "enums": len(enums),
        "enumerators": sum(len(row.get("values", {})) for row in enums),
        "forward_or_opaque_declarations": len(
            snapshot.get("forward_or_opaque_declarations", [])
        ),
    }
    for key, value in expected.items():
        if summary.get(key) != value:
            findings.append(
                Finding(
                    "abi-summary-mismatch",
                    key,
                    f"summary={summary.get(key)!r}, measured={value}",
                )
            )
    fingerprint_input = {
        "profile": snapshot.get("profile"),
        "types": types,
        "enums": enums,
        "forward_or_opaque_declarations": snapshot.get(
            "forward_or_opaque_declarations", []
        ),
    }
    encoded = json.dumps(
        fingerprint_input,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    return findings, hashlib.sha256(encoded).hexdigest()


def standalone_source(header_path: str) -> str:
    return f'#include "{header_path}"\nint main(void) {{ return 0; }}\n'


def _compiler_command(
    compiler: Path,
    language: str,
    source: Path,
    include_dirs: tuple[Path, ...],
) -> list[str]:
    name = compiler.name.lower()
    if name in {"cl", "cl.exe"}:
        standard = "/std:c17" if language == "c" else "/std:c++17"
        mode = "/TC" if language == "c" else "/TP"
        command = [
            str(compiler),
            "/nologo",
            standard,
            mode,
            "/Zs",
        ]
        command.extend(f"/I{path}" for path in include_dirs)
        command.append(str(source))
        return command
    standard = "-std=c17" if language == "c" else "-std=c++17"
    command = [
        str(compiler),
        standard,
        "-fsyntax-only",
    ]
    for path in include_dirs:
        command.extend(("-I", str(path)))
    command.append(str(source))
    return command


def compile_findings(
    manifest: dict[str, Any],
    c_compiler: Path,
    cxx_compiler: Path,
    output_dir: Path,
) -> tuple[list[Finding], dict[str, Any]]:
    output_dir.mkdir(parents=True, exist_ok=True)
    findings: list[Finding] = []
    results: list[dict[str, Any]] = []
    public_rows = public_header_rows(manifest)
    include_dirs = tuple(
        sorted(
            {REPOSITORY_ROOT, REPOSITORY_ROOT / "byul"}
            | {
                (REPOSITORY_ROOT / row["current_path"]).parent
                for row in manifest["headers"]
                if row["current_path"].startswith("byul/")
            },
            key=str,
        )
    )
    for row in public_rows:
        header = row["current_path"]
        for language, compiler, suffix in (
            ("c", c_compiler, ".c"),
            ("c++", cxx_compiler, ".cpp"),
        ):
            safe_name = re.sub(r"[^A-Za-z0-9]+", "_", header).strip("_")
            source = output_dir / f"{safe_name}{suffix}"
            source.write_text(
                standalone_source(header), encoding="utf-8", newline="\n"
            )
            command = _compiler_command(
                compiler, language, source, include_dirs
            )
            environment = os.environ.copy()
            environment["PATH"] = (
                str(compiler.parent)
                + os.pathsep
                + environment.get("PATH", "")
            )
            completed = subprocess.run(
                command,
                cwd=REPOSITORY_ROOT,
                env=environment,
                capture_output=True,
                text=True,
                encoding="utf-8",
                errors="replace",
                check=False,
            )
            result = {
                "header": header,
                "language": language,
                "compiler": str(compiler),
                "returncode": completed.returncode,
                "command": command,
                "stdout": completed.stdout,
                "stderr": completed.stderr,
            }
            results.append(result)
            if completed.returncode:
                findings.append(
                    Finding(
                        "standalone-compile",
                        f"{header}:{language}",
                        f"compiler exited {completed.returncode}",
                    )
                )
    return findings, {
        "schema_version": 1,
        "checks": len(results),
        "passed": sum(row["returncode"] == 0 for row in results),
        "failed": sum(row["returncode"] != 0 for row in results),
        "results": results,
    }


def apply_invariant_exclusions(
    findings: list[Finding], exclusions: dict[str, Any]
) -> tuple[list[Finding], list[Finding]]:
    approved: dict[tuple[str, str], dict[str, Any]] = {}
    validation: list[Finding] = []
    for row in exclusions.get("exclusions", []):
        key = (row.get("code"), row.get("subject"))
        if key in approved:
            validation.append(
                Finding(
                    "duplicate-invariant-exclusion",
                    str(key),
                    "invariant exclusion is duplicated",
                )
            )
        if not row.get("reason") or not row.get("owner") or not row.get("target_stage"):
            validation.append(
                Finding(
                    "incomplete-invariant-exclusion",
                    str(key),
                    "exclusion requires reason, owner and target_stage",
                )
            )
        approved[key] = row

    current = {(item.code, item.subject) for item in findings}
    for key in sorted(set(approved) - current):
        validation.append(
            Finding(
                "stale-invariant-exclusion",
                f"{key[0]}:{key[1]}",
                "approved exclusion no longer matches a current invariant failure",
            )
        )
    remaining = [
        item for item in findings if (item.code, item.subject) not in approved
    ]
    grandfathered = [
        item for item in findings if (item.code, item.subject) in approved
    ]
    return remaining + validation, grandfathered


def metadata_baseline_fingerprints(payload: dict[str, Any]) -> set[str]:
    result: set[str] = set()
    for issue in payload.get("issues", []):
        subject = f"byul/{issue['header']}:{issue.get('symbol') or '-'}"
        result.add(
            Finding(
                f"metadata:{issue['code']}", subject, issue["message"]
            ).fingerprint
        )
    return result


def debt_payload(findings: Iterable[Finding]) -> dict[str, Any]:
    rows = sorted(
        [
            asdict(item) | {"fingerprint": item.fingerprint}
            for item in findings
        ],
        key=lambda row: row["fingerprint"],
    )
    return {
        "schema_version": 1,
        "policy": "Only exact reviewed fingerprints are grandfathered; new or changed findings fail.",
        "findings": rows,
    }


def run_audit(
    manifest: dict[str, Any],
    build_snapshot: dict[str, Any],
    abi_snapshot: dict[str, Any],
    metadata_baseline: dict[str, Any],
    debt_baseline: dict[str, Any] | None,
    exclusions: dict[str, Any],
    strict_headers: set[str],
) -> tuple[dict[str, Any], int]:
    invariants = manifest_findings(manifest)
    invariants.extend(wrapper_findings(manifest))
    metadata, symbols = metadata_findings(manifest)
    invariants.extend(export_findings(symbols, build_snapshot))
    abi_findings, abi_fingerprint = abi_snapshot_findings(abi_snapshot)
    invariants.extend(abi_findings)
    invariants, excluded_invariants = apply_invariant_exclusions(
        invariants, exclusions
    )

    structural: list[Finding] = []
    for row in public_header_rows(manifest):
        structural.extend(structural_findings(REPOSITORY_ROOT / row["current_path"]))

    known_metadata = metadata_baseline_fingerprints(metadata_baseline)
    known_structural = {
        row["fingerprint"] for row in (debt_baseline or {}).get("findings", [])
    }
    new_debt = [
        item
        for item in metadata
        if item.fingerprint not in known_metadata
        or item.subject.split(":", 1)[0] in strict_headers
    ]
    new_debt.extend(
        item
        for item in structural
        if item.fingerprint not in known_structural or item.subject in strict_headers
    )
    violations = invariants + new_debt
    payload = {
        "schema_version": 1,
        "summary": {
            "manifest_headers": len(manifest.get("headers", [])),
            "public_c_headers": len(public_header_rows(manifest)),
            "public_declarations": len(symbols),
            "abi_fingerprint_sha256": abi_fingerprint,
            "invariant_errors": len(invariants),
            "approved_invariant_exclusions": len(excluded_invariants),
            "metadata_debt": len(metadata),
            "structural_debt": len(structural),
            "new_or_strict_debt": len(new_debt),
            "errors": len(violations),
        },
        "violations": [asdict(item) for item in violations],
        "approved_invariant_debt": [
            asdict(item) for item in excluded_invariants
        ],
        "known_debt": [asdict(item) for item in metadata + structural],
    }
    return payload, 1 if violations else 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Audit BYUL header roles, ABI exports and wrapper coverage."
    )
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument(
        "--build-snapshot", type=Path, default=DEFAULT_BUILD_SNAPSHOT
    )
    parser.add_argument("--abi-snapshot", type=Path, default=DEFAULT_ABI_SNAPSHOT)
    parser.add_argument(
        "--metadata-baseline", type=Path, default=DEFAULT_METADATA_BASELINE
    )
    parser.add_argument("--debt-baseline", type=Path, default=DEFAULT_DEBT_BASELINE)
    parser.add_argument("--exclusions", type=Path, default=DEFAULT_EXCLUSIONS)
    parser.add_argument("--json-output", type=Path)
    parser.add_argument(
        "--strict-header",
        action="append",
        default=[],
        help="repository-relative public header whose grandfathered debt must fail",
    )
    parser.add_argument(
        "--write-debt-baseline",
        action="store_true",
        help="write the current structural debt fingerprints after invariant checks pass",
    )
    parser.add_argument("--c-compiler", type=Path)
    parser.add_argument("--cxx-compiler", type=Path)
    parser.add_argument(
        "--compile-output-dir",
        type=Path,
        default=REPOSITORY_ROOT / "build_header_abi_audit",
    )
    parser.add_argument("--compile-report", type=Path)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    manifest = load_json(args.manifest)
    build = load_json(args.build_snapshot)
    abi_snapshot = load_json(args.abi_snapshot)
    metadata_baseline = load_json(args.metadata_baseline)
    debt = load_json(args.debt_baseline) if args.debt_baseline.is_file() else None
    exclusions = load_json(args.exclusions) if args.exclusions.is_file() else {}
    payload, result = run_audit(
        manifest,
        build,
        abi_snapshot,
        metadata_baseline,
        debt,
        exclusions,
        set(args.strict_header),
    )
    if (args.c_compiler is None) != (args.cxx_compiler is None):
        print(
            "[ERROR] --c-compiler and --cxx-compiler must be provided together",
            file=sys.stderr,
        )
        return 2
    if args.c_compiler is not None and args.cxx_compiler is not None:
        compile_errors, compile_report = compile_findings(
            manifest,
            args.c_compiler,
            args.cxx_compiler,
            args.compile_output_dir,
        )
        payload["summary"]["standalone_compile_checks"] = compile_report["checks"]
        payload["summary"]["standalone_compile_failures"] = compile_report["failed"]
        payload["violations"].extend(asdict(item) for item in compile_errors)
        payload["summary"]["errors"] += len(compile_errors)
        if compile_errors:
            result = 1
        if args.compile_report:
            atomic_write_json(args.compile_report, compile_report)
    else:
        payload["summary"]["standalone_compile_checks"] = 0
        payload["summary"]["standalone_compile_failures"] = 0
    if args.write_debt_baseline:
        if payload["summary"]["invariant_errors"]:
            print("[ERROR] refusing to baseline invariant failures", file=sys.stderr)
            return 1
        structural = [
            Finding(**row)
            for row in payload["known_debt"]
            if not row["code"].startswith("metadata:")
        ]
        atomic_write_json(args.debt_baseline, debt_payload(structural))
        print(f"[WRITTEN] {args.debt_baseline.relative_to(REPOSITORY_ROOT)}")
        # The newly written baseline is allowed in this same invocation.
        result = 0
        payload["summary"]["new_or_strict_debt"] = 0
        payload["summary"]["errors"] = 0
        payload["violations"] = []
    if args.json_output:
        atomic_write_json(args.json_output, payload)
    summary = payload["summary"]
    print(
        "[HEADER-ABI-AUDIT] "
        f"headers={summary['manifest_headers']} public-c={summary['public_c_headers']} "
        f"declarations={summary['public_declarations']} "
        f"invariants={summary['invariant_errors']} "
        f"excluded-invariants={summary['approved_invariant_exclusions']} "
        f"known-debt={summary['metadata_debt'] + summary['structural_debt']} "
        f"compile={summary['standalone_compile_checks'] - summary['standalone_compile_failures']}/"
        f"{summary['standalone_compile_checks']} "
        f"errors={summary['errors']}"
    )
    for item in payload["violations"][:50]:
        print(
            f"[ERROR] {item['code']} {item['subject']}: {item['message']}",
            file=sys.stderr,
        )
    return result


if __name__ == "__main__":
    raise SystemExit(main())
