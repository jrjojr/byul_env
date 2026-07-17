#!/usr/bin/env python3
"""Validate and publish BYUL Org documents as generated Markdown files."""

from __future__ import annotations

import argparse
import hashlib
import os
import re
import shutil
import stat
import subprocess
import sys
import tempfile
import urllib.parse
from dataclasses import dataclass
from pathlib import Path, PurePosixPath
from typing import Dict, Iterable, List, Mapping, Optional, Sequence, Tuple


MANIFEST_DEFAULT = "tools/docs/document-map.org"
MAP_TABLE_NAME = "byul-document-map-v1"
MAP_COLUMNS = ("key", "source", "output", "slug", "kind", "language", "access")
GENERATOR_PATH = "tools/docs/publish_docs.py"
EXPORTER_PATH = "tools/docs/publish.el"
SOURCE_FORMAT_VERSION = "1"
DOCUMENT_MAP_VERSION = "1"

IDENTIFIER_RE = re.compile(r"^[a-z0-9](?:[a-z0-9-]*[a-z0-9])?$")
METADATA_RE = re.compile(r"^#\+([A-Z0-9_]+):\s*(.*?)\s*$", re.IGNORECASE)
HEADING_RE = re.compile(r"^\*+\s+")
ORG_SEPARATOR_RE = re.compile(r"^\s*\|[-+]+\|\s*$")
HASH_RE = r"[0-9a-f]{64}"
MARKER_RE = re.compile(
    r"\A<!--\n"
    r"GENERATED FILE - DO NOT EDIT\n"
    r"Source: (?P<source>[^\n]+)\n"
    r"Source-SHA256: (?P<source_hash>" + HASH_RE + r")\n"
    r"Content-SHA256: (?P<content_hash>" + HASH_RE + r")\n"
    r"Generator: (?P<generator>[^\n]+)\n"
    r"-->\n\n"
)
BEGIN_SRC_RE = re.compile(r"^\s*#\+begin_src(?:\s|$)", re.IGNORECASE)
END_SRC_RE = re.compile(r"^\s*#\+end_src\s*$", re.IGNORECASE)
FILE_LINK_RE = re.compile(r"\[\[file:([^\]]+)\](?:\[[^\]]*\])?\]", re.IGNORECASE)

REQUIRED_SOURCE_METADATA = (
    "TITLE",
    "LANGUAGE",
    "DOCUMENT_KIND",
    "AUDIENCE",
    "ACCESS",
    "SECTION",
    "SLUG",
    "SOURCE_FORMAT_VERSION",
    "GENERATED_TARGETS",
)
ALLOWED_KINDS = {
    "rule",
    "todo",
    "configuration",
    "help",
    "architecture",
    "diagnostic",
    "release-note",
    "reference",
}
ALLOWED_LANGUAGES = {"ko", "en"}
ALLOWED_ACCESS = {"public", "protected"}


class PublishError(RuntimeError):
    """An expected validation or publication failure."""


@dataclass(frozen=True)
class Document:
    key: str
    source: str
    output: str
    slug: str
    kind: str
    language: str
    access: str


@dataclass(frozen=True)
class OutputState:
    document: Document
    kind: str
    detail: str
    current_bytes: Optional[bytes]


def canonical_text(text: str) -> str:
    """Return text with platform line endings normalized to LF."""
    return text.replace("\r\n", "\n").replace("\r", "\n")


def text_digest(text: str) -> str:
    return hashlib.sha256(canonical_text(text).encode("utf-8")).hexdigest()


def repository_root() -> Path:
    root = Path(__file__).resolve().parents[2]
    required = (
        root / "byul" / "CMakeLists.txt",
        root / "tools" / "docs" / "document-map.org",
        root / "docs" / "org",
    )
    missing = [path for path in required if not path.exists()]
    if missing:
        rendered = ", ".join(str(path) for path in missing)
        raise PublishError(
            f"Cannot identify the BYUL repository root from {__file__}; "
            f"missing repository-owned path(s): {rendered}"
        )
    return root.resolve()


def relative_display(path: Path, root: Path) -> str:
    try:
        return path.resolve(strict=False).relative_to(root).as_posix()
    except ValueError:
        return str(path)


def ensure_inside_root(path: Path, root: Path, label: str) -> Path:
    resolved = path.resolve(strict=False)
    try:
        resolved.relative_to(root)
    except ValueError as exc:
        raise PublishError(f"{label} escapes the repository root: {path}") from exc
    return resolved


def read_utf8(path: Path, label: str) -> str:
    if not path.exists():
        raise PublishError(f"Missing {label}: {path}")
    if not path.is_file():
        raise PublishError(f"{label} is not a regular file: {path}")
    try:
        text = path.read_text(encoding="utf-8")
    except UnicodeDecodeError as exc:
        raise PublishError(f"{label} is not valid UTF-8: {path}: {exc}") from exc
    except OSError as exc:
        raise PublishError(f"Cannot read {label}: {path}: {exc}") from exc
    if "\x00" in text:
        raise PublishError(f"{label} contains a NUL byte: {path}")
    return text


def parse_preamble_metadata(text: str, label: str) -> Dict[str, str]:
    metadata: Dict[str, str] = {}
    duplicates: List[str] = []
    for line in canonical_text(text).split("\n"):
        if HEADING_RE.match(line):
            break
        match = METADATA_RE.match(line)
        if not match:
            continue
        key = match.group(1).upper()
        value = match.group(2).strip()
        if key in metadata:
            duplicates.append(key)
        else:
            metadata[key] = value
    if duplicates:
        names = ", ".join(sorted(set(duplicates)))
        raise PublishError(f"Duplicate preamble metadata in {label}: {names}")
    return metadata


def parse_org_row(line: str) -> Tuple[str, ...]:
    stripped = line.strip()
    if not (stripped.startswith("|") and stripped.endswith("|")):
        raise PublishError(f"Invalid Org table row: {line}")
    return tuple(cell.strip() for cell in stripped[1:-1].split("|"))


def validate_manifest_relative_path(value: str, suffix: str, prefix: str, label: str) -> str:
    if not value or "\\" in value or ":" in value:
        raise PublishError(f"{label} must be a POSIX repository-relative path: {value!r}")
    pure = PurePosixPath(value)
    normalized = pure.as_posix()
    if pure.is_absolute() or normalized != value or any(part in {"", ".", ".."} for part in pure.parts):
        raise PublishError(f"{label} is not a normalized repository-relative path: {value!r}")
    if not normalized.startswith(prefix) or pure.suffix != suffix:
        raise PublishError(f"{label} must be under {prefix} and end in {suffix}: {value!r}")
    return normalized


def load_manifest(root: Path, manifest_path: Path) -> List[Document]:
    manifest_path = ensure_inside_root(manifest_path, root, "Manifest")
    text = read_utf8(manifest_path, "Org document map")
    metadata = parse_preamble_metadata(text, relative_display(manifest_path, root))
    version = metadata.get("DOCUMENT_MAP_VERSION")
    if version != DOCUMENT_MAP_VERSION:
        raise PublishError(
            f"DOCUMENT_MAP_VERSION must be {DOCUMENT_MAP_VERSION!r}, got {version!r}"
        )

    lines = canonical_text(text).split("\n")
    name_line = f"#+NAME: {MAP_TABLE_NAME}".casefold()
    name_indexes = [index for index, line in enumerate(lines) if line.strip().casefold() == name_line]
    if len(name_indexes) != 1:
        raise PublishError(
            f"Manifest must contain exactly one #+NAME: {MAP_TABLE_NAME} table; "
            f"found {len(name_indexes)}"
        )

    index = name_indexes[0] + 1
    while index < len(lines) and not lines[index].strip():
        index += 1
    if index >= len(lines):
        raise PublishError("The named document map has no header row")
    header = parse_org_row(lines[index])
    if header != MAP_COLUMNS:
        raise PublishError(
            "Document map columns must be exactly: " + " | ".join(MAP_COLUMNS)
        )

    index += 1
    while index < len(lines) and not lines[index].strip():
        index += 1
    if index >= len(lines) or not ORG_SEPARATOR_RE.match(lines[index]):
        raise PublishError("The document map header must be followed by an Org separator row")

    documents: List[Document] = []
    index += 1
    while index < len(lines):
        line = lines[index]
        if not line.strip().startswith("|"):
            break
        if ORG_SEPARATOR_RE.match(line):
            raise PublishError("Unexpected separator inside the document map body")
        cells = parse_org_row(line)
        if len(cells) != len(MAP_COLUMNS):
            raise PublishError(
                f"Document map row {index + 1} has {len(cells)} cells; "
                f"expected {len(MAP_COLUMNS)}"
            )
        if not all(cells):
            raise PublishError(f"Document map row {index + 1} contains an empty cell")
        row = dict(zip(MAP_COLUMNS, cells))
        source = validate_manifest_relative_path(
            row["source"], ".org", "docs/org/", f"source in row {index + 1}"
        )
        output = validate_manifest_relative_path(
            row["output"], ".md", "docs/", f"output in row {index + 1}"
        )
        if not IDENTIFIER_RE.fullmatch(row["key"]):
            raise PublishError(f"Invalid key in row {index + 1}: {row['key']!r}")
        if not IDENTIFIER_RE.fullmatch(row["slug"]):
            raise PublishError(f"Invalid slug in row {index + 1}: {row['slug']!r}")
        if row["kind"] not in ALLOWED_KINDS:
            raise PublishError(f"Unknown document kind in row {index + 1}: {row['kind']!r}")
        if row["language"] not in ALLOWED_LANGUAGES:
            raise PublishError(f"Unknown language in row {index + 1}: {row['language']!r}")
        if row["access"] not in ALLOWED_ACCESS:
            raise PublishError(f"Unknown access in row {index + 1}: {row['access']!r}")

        source_path = ensure_inside_root(root / Path(source), root, "Mapped source")
        output_path = ensure_inside_root(root / Path(output), root, "Mapped output")
        if source_path == output_path:
            raise PublishError(f"Source and output are the same path in row {index + 1}")
        documents.append(
            Document(
                key=row["key"],
                source=source,
                output=output,
                slug=row["slug"],
                kind=row["kind"],
                language=row["language"],
                access=row["access"],
            )
        )
        index += 1

    if not documents:
        raise PublishError("The document map is empty")
    validate_unique_manifest_fields(documents)
    return documents


def validate_unique_manifest_fields(documents: Sequence[Document]) -> None:
    fields = ("key", "slug", "source", "output")
    errors: List[str] = []
    for field in fields:
        owners: Dict[str, str] = {}
        for document in documents:
            value = getattr(document, field)
            folded = value.casefold()
            previous = owners.get(folded)
            if previous is not None:
                errors.append(f"duplicate {field} {value!r}: {previous}, {document.key}")
            else:
                owners[folded] = document.key
    if errors:
        raise PublishError("Manifest identity collisions:\n  - " + "\n  - ".join(errors))


def is_generated_by_this_publisher(text: str) -> bool:
    normalized = canonical_text(text)
    if not normalized.startswith("<!--\nGENERATED FILE - DO NOT EDIT\n"):
        return False
    header_lines = normalized.split("\n", 20)[:20]
    return f"Generator: {GENERATOR_PATH}" in header_lines


def validate_document_inventory(root: Path, documents: Sequence[Document]) -> None:
    source_root = ensure_inside_root(root / "docs" / "org", root, "Org source directory")
    docs_root = ensure_inside_root(root / "docs", root, "Document directory")
    if not source_root.is_dir():
        raise PublishError("Missing canonical Org source directory: docs/org")

    mapped_sources = {document.source.casefold() for document in documents}
    mapped_outputs = {document.output.casefold() for document in documents}
    orphan_sources: List[str] = []
    orphan_outputs: List[str] = []

    for candidate in source_root.rglob("*"):
        if not candidate.is_file() or candidate.suffix.casefold() != ".org":
            continue
        resolved = ensure_inside_root(candidate, root, "Discovered Org source")
        relative = relative_display(resolved, root)
        if relative.casefold() not in mapped_sources:
            orphan_sources.append(relative)

    for candidate in docs_root.rglob("*"):
        if not candidate.is_file() or candidate.suffix.casefold() != ".md":
            continue
        resolved = ensure_inside_root(candidate, root, "Discovered Markdown output")
        relative = relative_display(resolved, root)
        try:
            text = resolved.read_bytes().decode("utf-8", errors="replace")
        except OSError:
            continue
        if is_generated_by_this_publisher(text) and relative.casefold() not in mapped_outputs:
            orphan_outputs.append(relative)

    errors: List[str] = []
    if orphan_sources:
        errors.append(
            "unmapped Org source(s): " + ", ".join(sorted(orphan_sources, key=str.casefold))
        )
    if orphan_outputs:
        errors.append(
            "unmapped generated Markdown output(s): "
            + ", ".join(sorted(orphan_outputs, key=str.casefold))
        )
    if errors:
        raise PublishError("Document inventory is incomplete:\n  - " + "\n  - ".join(errors))


def validate_org_structure_and_links(root: Path, document: Document, text: str) -> None:
    source_path = ensure_inside_root(root / Path(document.source), root, "Org source")
    source_block_line: Optional[int] = None
    errors: List[str] = []

    for line_number, line in enumerate(canonical_text(text).split("\n"), start=1):
        if BEGIN_SRC_RE.match(line):
            if source_block_line is not None:
                errors.append(
                    f"line {line_number}: nested #+begin_src; "
                    f"previous block began at line {source_block_line}"
                )
            else:
                source_block_line = line_number
            continue
        if END_SRC_RE.match(line):
            if source_block_line is None:
                errors.append(f"line {line_number}: #+end_src without #+begin_src")
            else:
                source_block_line = None
            continue
        if source_block_line is not None:
            continue

        for match in FILE_LINK_RE.finditer(line):
            raw_target = match.group(1).strip()
            path_part = raw_target.split("::", 1)[0].strip()
            decoded = urllib.parse.unquote(path_part)
            if not decoded:
                errors.append(f"line {line_number}: empty file: link target")
                continue
            if (
                "\\" in decoded
                or decoded.startswith("~")
                or PurePosixPath(decoded).is_absolute()
                or re.match(r"^[A-Za-z]:", decoded)
            ):
                errors.append(
                    f"line {line_number}: file: link must be a POSIX relative path: {raw_target!r}"
                )
                continue
            target = (source_path.parent / Path(PurePosixPath(decoded))).resolve(strict=False)
            try:
                target.relative_to(root)
            except ValueError:
                errors.append(
                    f"line {line_number}: file: link escapes the repository: {raw_target!r}"
                )
                continue
            if not target.exists():
                errors.append(
                    f"line {line_number}: missing file: link target {raw_target!r}"
                )

    if source_block_line is not None:
        errors.append(f"line {source_block_line}: #+begin_src has no matching #+end_src")
    if errors:
        raise PublishError(
            f"Invalid Org structure or links in {document.source}:\n    - "
            + "\n    - ".join(errors)
        )


def validate_source(root: Path, document: Document) -> str:
    source_path = ensure_inside_root(root / Path(document.source), root, "Source")
    text = read_utf8(source_path, f"Org source for {document.key}")
    metadata = parse_preamble_metadata(text, document.source)
    missing = [key for key in REQUIRED_SOURCE_METADATA if not metadata.get(key)]
    if missing:
        raise PublishError(
            f"Missing source metadata in {document.source}: {', '.join(missing)}"
        )

    expected = {
        "LANGUAGE": document.language,
        "DOCUMENT_KIND": document.kind,
        "ACCESS": document.access,
        "SLUG": document.slug,
        "SOURCE_FORMAT_VERSION": SOURCE_FORMAT_VERSION,
    }
    mismatches = [
        f"{key}={metadata.get(key)!r} (expected {value!r})"
        for key, value in expected.items()
        if metadata.get(key) != value
    ]
    if metadata["GENERATED_TARGETS"] != document.output:
        mismatches.append(
            f"GENERATED_TARGETS={metadata['GENERATED_TARGETS']!r} "
            f"(expected {document.output!r})"
        )
    for key in ("AUDIENCE", "SECTION"):
        if not IDENTIFIER_RE.fullmatch(metadata[key]):
            mismatches.append(f"{key}={metadata[key]!r} (expected a lowercase identifier)")
    if mismatches:
        raise PublishError(
            f"Invalid source metadata in {document.source}: " + "; ".join(mismatches)
        )
    validate_org_structure_and_links(root, document, text)
    return text_digest(text)


def validate_all_sources(root: Path, documents: Sequence[Document]) -> Dict[Document, str]:
    hashes: Dict[Document, str] = {}
    errors: List[str] = []
    for document in documents:
        try:
            hashes[document] = validate_source(root, document)
        except PublishError as exc:
            errors.append(str(exc))
    if errors:
        raise PublishError("Source validation failed:\n  - " + "\n  - ".join(errors))
    return hashes


def build_marker(document: Document, source_hash: str, content_hash: str) -> str:
    return (
        "<!--\n"
        "GENERATED FILE - DO NOT EDIT\n"
        f"Source: {document.source}\n"
        f"Source-SHA256: {source_hash}\n"
        f"Content-SHA256: {content_hash}\n"
        f"Generator: {GENERATOR_PATH}\n"
        "-->\n\n"
    )


def inspect_output(root: Path, document: Document, source_hash: str) -> OutputState:
    output_path = ensure_inside_root(root / Path(document.output), root, "Output")
    if not output_path.exists():
        return OutputState(document, "missing", "output does not exist", None)
    if not output_path.is_file():
        return OutputState(document, "unmanaged", "output is not a regular file", None)
    try:
        current_bytes = output_path.read_bytes()
        text = current_bytes.decode("utf-8")
    except UnicodeDecodeError as exc:
        return OutputState(
            document, "unmanaged", f"output is not valid UTF-8: {exc}", current_bytes
        )
    except OSError as exc:
        return OutputState(document, "unmanaged", f"cannot read output: {exc}", None)

    normalized = canonical_text(text)
    marker = MARKER_RE.match(normalized)
    if marker is None:
        return OutputState(document, "unmanaged", "generated marker is missing or malformed", current_bytes)
    if marker.group("source") != document.source:
        return OutputState(
            document,
            "unmanaged",
            f"marker source is {marker.group('source')!r}, expected {document.source!r}",
            current_bytes,
        )
    if marker.group("generator") != GENERATOR_PATH:
        return OutputState(
            document,
            "unmanaged",
            f"marker generator is {marker.group('generator')!r}, expected {GENERATOR_PATH!r}",
            current_bytes,
        )
    body = normalized[marker.end() :]
    if not body.strip():
        return OutputState(document, "modified-output", "generated Markdown body is empty", current_bytes)
    actual_content_hash = text_digest(body)
    if marker.group("content_hash") != actual_content_hash:
        return OutputState(
            document,
            "modified-output",
            "Markdown body hash differs from the generated marker",
            current_bytes,
        )
    if marker.group("source_hash") != source_hash:
        return OutputState(
            document,
            "stale-source",
            "Org source hash differs from the generated marker",
            current_bytes,
        )
    return OutputState(document, "valid", "marker and hashes match", current_bytes)


def inspect_all_outputs(
    root: Path, documents: Sequence[Document], source_hashes: Mapping[Document, str]
) -> List[OutputState]:
    return [inspect_output(root, document, source_hashes[document]) for document in documents]


def check_destination(root: Path, document: Document) -> None:
    output = ensure_inside_root(root / Path(document.output), root, "Output destination")
    if output.exists() and not output.is_file():
        raise PublishError(f"Output destination is not a regular file: {document.output}")
    ancestor = output.parent
    while not ancestor.exists():
        if ancestor == root:
            break
        ancestor = ancestor.parent
    ensure_inside_root(ancestor, root, "Output parent")
    if not ancestor.is_dir():
        raise PublishError(f"Output parent is not a directory: {ancestor}")
    if not os.access(str(ancestor), os.W_OK):
        raise PublishError(f"Output parent is not writable: {ancestor}")


def resolve_executable(candidate: str) -> Optional[str]:
    if not candidate:
        return None
    expanded = os.path.expandvars(os.path.expanduser(candidate))
    if os.path.isabs(expanded) or os.sep in expanded or (os.altsep and os.altsep in expanded):
        path = Path(expanded)
        return str(path.resolve()) if path.is_file() else None
    return shutil.which(expanded)


def discover_emacs(explicit: Optional[str]) -> Tuple[Optional[str], str]:
    if explicit:
        resolved = resolve_executable(explicit)
        if resolved is None:
            raise PublishError(f"--emacs does not name an executable file or command: {explicit}")
        return resolved, "--emacs"
    configured = os.environ.get("BYUL_EMACS", "").strip()
    if configured:
        resolved = resolve_executable(configured)
        if resolved is not None:
            return resolved, "BYUL_EMACS"
        return None, f"BYUL_EMACS is configured but unavailable: {configured}"
    resolved = resolve_executable("emacs") or resolve_executable("emacs.exe")
    if resolved:
        return resolved, "PATH"
    return None, "Emacs was not found"


def export_one(
    root: Path,
    document: Document,
    source_hash: str,
    emacs: str,
    staging_root: Path,
    timeout: int,
) -> bytes:
    source = ensure_inside_root(root / Path(document.source), root, "Org source")
    output = ensure_inside_root(staging_root / Path(document.output), staging_root, "Staging output")
    output.parent.mkdir(parents=True, exist_ok=True)
    exporter = ensure_inside_root(root / EXPORTER_PATH, root, "Emacs exporter")
    if not exporter.is_file():
        raise PublishError(f"Missing repository-owned Emacs exporter: {EXPORTER_PATH}")

    environment = os.environ.copy()
    environment.update(
        {
            "BYUL_DOCS_SOURCE": str(source),
            "BYUL_DOCS_OUTPUT": str(output),
            "BYUL_DOCS_STAGING_ROOT": str(staging_root),
        }
    )
    command = [
        emacs,
        "--batch",
        "--quick",
        "-l",
        str(exporter),
        "--funcall",
        "byul-docs-export-from-environment",
    ]
    try:
        result = subprocess.run(
            command,
            cwd=str(root),
            env=environment,
            check=False,
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
            timeout=timeout,
        )
    except OSError as exc:
        raise PublishError(f"Cannot run Emacs for {document.source}: {exc}") from exc
    except subprocess.TimeoutExpired as exc:
        raise PublishError(
            f"Emacs export timed out after {timeout}s for {document.source}"
        ) from exc
    if result.returncode != 0:
        detail = (result.stderr or result.stdout or "no diagnostic output").strip()
        raise PublishError(
            f"Emacs export failed for {document.source} with exit code "
            f"{result.returncode}: {detail}"
        )

    raw = read_utf8(output, f"staged Markdown for {document.key}")
    body = canonical_text(raw)
    if not body.strip():
        raise PublishError(f"Emacs created an empty Markdown body for {document.source}")
    if MARKER_RE.match(body):
        raise PublishError(
            f"Emacs output unexpectedly contains a generated marker: {document.source}"
        )
    if not body.endswith("\n"):
        body += "\n"
    content_hash = text_digest(body)
    generated = build_marker(document, source_hash, content_hash) + body
    generated_bytes = generated.encode("utf-8")

    # Validate exactly what would be committed, including the marker and hashes.
    output.write_bytes(generated_bytes)
    staged_state = inspect_output(staging_root, document, source_hash)
    if staged_state.kind != "valid":
        raise PublishError(
            f"Staged output validation failed for {document.output}: {staged_state.detail}"
        )
    return generated_bytes


def export_all(
    root: Path,
    documents: Sequence[Document],
    source_hashes: Mapping[Document, str],
    emacs: str,
    staging_root: Path,
    timeout: int,
) -> Dict[Document, bytes]:
    generated: Dict[Document, bytes] = {}
    for document in documents:
        print(f"[EXPORT] {document.source} -> {document.output}")
        generated[document] = export_one(
            root, document, source_hashes[document], emacs, staging_root, timeout
        )
    return generated


def atomic_replace_bytes(path: Path, data: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    existing_mode = stat.S_IMODE(path.stat().st_mode) if path.exists() else 0o644
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent)
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        os.chmod(str(temporary), existing_mode)
        os.replace(str(temporary), str(path))
    finally:
        if temporary.exists():
            temporary.unlink()


def commit_outputs(
    root: Path, changes: Sequence[Tuple[Document, bytes, Optional[bytes]]]
) -> None:
    originals: Dict[Path, Optional[bytes]] = {}
    committed: List[Path] = []
    try:
        for document, generated, expected_current in changes:
            target = ensure_inside_root(root / Path(document.output), root, "Commit target")
            current = target.read_bytes() if target.exists() else None
            if current != expected_current:
                raise PublishError(
                    f"Output changed after validation; refusing to overwrite: {document.output}"
                )
            originals[target] = current
            atomic_replace_bytes(target, generated)
            committed.append(target)
    except Exception as exc:
        rollback_errors: List[str] = []
        for target in reversed(committed):
            try:
                original = originals[target]
                if original is None:
                    target.unlink(missing_ok=True)
                else:
                    atomic_replace_bytes(target, original)
            except OSError as rollback_exc:
                rollback_errors.append(f"{relative_display(target, root)}: {rollback_exc}")
        detail = f"Commit failed and prior replacements were rolled back: {exc}"
        if rollback_errors:
            detail += "\nRollback also failed for:\n  - " + "\n  - ".join(rollback_errors)
        raise PublishError(detail) from exc


def print_context(
    mode: str,
    root: Path,
    manifest: Path,
    documents: Sequence[Document],
    emacs: Optional[str],
    emacs_source: str,
) -> None:
    print(f"Mode: {mode}")
    print(f"Repository: {root}")
    print(f"Manifest: {relative_display(manifest, root)}")
    print(f"Documents: {len(documents)}")
    if emacs:
        print(f"Emacs: {emacs} ({emacs_source})")
    else:
        print(f"Emacs: unavailable ({emacs_source})")


def validate_existing_output_policy(
    mode: str, states: Sequence[OutputState], force: bool
) -> None:
    errors: List[str] = []
    for state in states:
        if mode == "check" and state.kind != "valid":
            errors.append(f"{state.document.output}: {state.kind}: {state.detail}")
        elif mode != "check" and state.kind in {"unmanaged", "modified-output"} and not force:
            errors.append(
                f"{state.document.output}: {state.kind}: {state.detail}; "
                "review the file and use --force to replace it"
            )
    if errors:
        raise PublishError("Output validation failed:\n  - " + "\n  - ".join(errors))


def planned_action(state: OutputState, generated: Optional[bytes]) -> str:
    if generated is None:
        if state.kind == "missing":
            return "CREATE"
        if state.kind in {"unmanaged", "modified-output"}:
            return "FORCE-REPLACE"
        if state.kind == "stale-source":
            return "UPDATE"
        return "VERIFY/UPDATE"
    if state.current_bytes is None:
        return "CREATE"
    if state.current_bytes == generated:
        return "UNCHANGED"
    if state.kind in {"unmanaged", "modified-output"}:
        return "FORCE-REPLACE"
    return "UPDATE"


def execute(args: argparse.Namespace) -> int:
    root = repository_root()
    manifest_input = Path(args.manifest)
    manifest = manifest_input if manifest_input.is_absolute() else root / manifest_input
    manifest = ensure_inside_root(manifest, root, "Manifest")
    documents = load_manifest(root, manifest)
    validate_document_inventory(root, documents)
    source_hashes = validate_all_sources(root, documents)
    states = inspect_all_outputs(root, documents, source_hashes)
    validate_existing_output_policy(args.mode, states, args.force)

    if args.mode != "check":
        for document in documents:
            check_destination(root, document)
    emacs, emacs_source = discover_emacs(args.emacs)
    print_context(args.mode, root, manifest, documents, emacs, emacs_source)

    if args.mode == "check":
        if emacs is not None:
            staging_parent = ensure_inside_root(
                root / "tools" / "docs", root, "Staging parent"
            )
            with tempfile.TemporaryDirectory(
                prefix=".publish-check-", dir=str(staging_parent)
            ) as temporary:
                staging_root = Path(temporary).resolve()
                generated = export_all(
                    root, documents, source_hashes, emacs, staging_root, args.timeout
                )
                drift = [
                    state.document.output
                    for state in states
                    if state.current_bytes != generated[state.document]
                ]
                if drift:
                    raise PublishError(
                        "Generated Markdown differs from a fresh Emacs export:\n  - "
                        + "\n  - ".join(drift)
                    )
        for state in states:
            print(f"[OK] {state.document.output}: marker and hashes match")
        if emacs is None:
            print("Exporter comparison: skipped because Emacs is unavailable; hashes were verified.")
        else:
            print("Exporter comparison: current outputs match a fresh staging export.")
        print("Result: mapping, metadata, outputs and hashes are consistent; no official files changed.")
        return 0

    if emacs is None:
        if args.mode == "apply":
            raise PublishError(
                "--apply requires Emacs. Install Emacs or set BYUL_EMACS to its executable."
            )
        for state in states:
            action = planned_action(state, None)
            print(
                f"[{action}] {state.document.output} <- {state.document.source} "
                "(exact export diff requires Emacs)"
            )
        print("Result: dry-run validation completed; no files changed.")
        return 0

    staging_parent = ensure_inside_root(root / "tools" / "docs", root, "Staging parent")
    with tempfile.TemporaryDirectory(prefix=".publish-stage-", dir=str(staging_parent)) as temporary:
        staging_root = Path(temporary).resolve()
        generated = export_all(
            root, documents, source_hashes, emacs, staging_root, args.timeout
        )
        changes: List[Tuple[Document, bytes, Optional[bytes]]] = []
        for state in states:
            data = generated[state.document]
            action = planned_action(state, data)
            print(f"[{action}] {state.document.output} <- {state.document.source}")
            if state.current_bytes != data:
                changes.append((state.document, data, state.current_bytes))

        if args.mode == "dry-run":
            print(f"Result: dry-run completed; {len(changes)} file(s) would change.")
            return 0
        if not changes:
            print("Result: all generated Markdown files are already current; no files changed.")
            return 0

        print("Commit order:")
        for document, _, _ in changes:
            print(f"  - {document.output}")
        commit_outputs(root, changes)
        print(f"Result: applied {len(changes)} generated Markdown file(s).")
        return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Validate the Org document map and publish BYUL Org sources as generated Markdown. "
            "The default mode is a write-free dry-run."
        )
    )
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument(
        "--dry-run",
        dest="mode",
        action="store_const",
        const="dry-run",
        help="validate and preview changes without replacing official outputs (default)",
    )
    modes.add_argument(
        "--check",
        dest="mode",
        action="store_const",
        const="check",
        help="fail when a mapping, source, output, marker or hash is inconsistent",
    )
    modes.add_argument(
        "--apply",
        dest="mode",
        action="store_const",
        const="apply",
        help="export to staging and atomically replace changed Markdown outputs",
    )
    parser.set_defaults(mode="dry-run")
    parser.add_argument(
        "--force",
        action="store_true",
        help="allow reviewed unmanaged or manually modified outputs to be replaced",
    )
    parser.add_argument(
        "--manifest",
        default=MANIFEST_DEFAULT,
        metavar="PATH",
        help=f"Org document map relative to the repository root (default: {MANIFEST_DEFAULT})",
    )
    parser.add_argument(
        "--emacs",
        metavar="PATH_OR_COMMAND",
        help="Emacs executable; otherwise BYUL_EMACS and PATH are checked",
    )
    parser.add_argument(
        "--timeout",
        type=int,
        default=120,
        metavar="SECONDS",
        help="per-document Emacs export timeout (default: 120)",
    )
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.timeout <= 0:
        parser.error("--timeout must be greater than zero")
    if args.force and args.mode == "check":
        parser.error("--force cannot be combined with --check")
    try:
        return execute(args)
    except PublishError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("ERROR: interrupted", file=sys.stderr)
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
