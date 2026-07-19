"""Validation helpers for the Navsys conformance evidence contract."""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path, PurePosixPath
import re


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
POLICY_PATH = (
    REPOSITORY_ROOT
    / "docs/ko/todo/navsys/navsys-conformance-policy.json"
)
INVENTORY_PATH = (
    REPOSITORY_ROOT
    / "docs/ko/todo/navsys/navsys-current-abi-inventory.json"
)
EVIDENCE_ROOT = (
    REPOSITORY_ROOT / "docs/ko/todo/navsys/evidence"
)
EXPECTED_CATEGORIES = {"correctness", "abi", "performance"}
EXPECTED_LAYERS = {"module_static", "root_shared", "wrapper", "installed_sdk"}
EXPECTED_VERDICTS = {"pass", "fail", "not-run", "not-applicable"}
EXPECTED_LAYER_CATEGORIES = {
    "module_static": {"correctness", "performance"},
    "root_shared": {"correctness", "abi", "performance"},
    "wrapper": {"correctness", "abi"},
    "installed_sdk": {"correctness", "abi"},
}
EXPECTED_RECORD_FIELDS = {
    "header",
    "owner_todo",
    "category",
    "layer",
    "revision",
    "host",
    "target",
    "toolchain",
    "configuration",
    "command",
    "artifacts",
    "verdict",
    "assertions",
    "limitations",
}
EXPECTED_RECORD_RULES = {
    "pass-requires-artifact",
    "fail-requires-artifact",
    "not-run-requires-limitation",
    "not-applicable-requires-limitation",
    "artifact-paths-are-repository-relative",
    "category-and-layer-must-be-compatible",
    "owner-todo-must-match-inventory",
}
EXPECTED_ARTIFACT_ROOT = PurePosixPath("docs/ko/todo/navsys/evidence")
REVISION_PATTERN = re.compile(r"^[0-9a-f]{7,40}$")


@dataclass(frozen=True)
class PolicyFinding:
    code: str
    subject: str
    message: str


def _relative_path(value: str) -> PurePosixPath | None:
    path = PurePosixPath(value)
    if path.is_absolute() or ".." in path.parts or "\\" in value:
        return None
    return path


def _command_matches(template: list[str], command: list[str]) -> bool:
    if len(template) != len(command):
        return False
    for expected, actual in zip(template, command):
        parts = re.split(r"(<[^>]+>)", expected)
        pattern = "".join(
            r".+" if part.startswith("<") and part.endswith(">")
            else re.escape(part)
            for part in parts
        )
        if re.fullmatch(pattern, actual) is None:
            return False
    return True


def validate_policy(policy: dict, inventory: dict) -> list[PolicyFinding]:
    findings: list[PolicyFinding] = []

    if policy.get("schema_version") != 1:
        findings.append(PolicyFinding(
            "schema-version", "policy", "schema_version must be 1"
        ))
    if inventory.get("schema_version", 0) < 2:
        findings.append(PolicyFinding(
            "inventory-schema",
            "inventory",
            "signature-aware inventory schema 2 or later is required",
        ))
    if policy.get("source_inventory") != str(
        INVENTORY_PATH.relative_to(REPOSITORY_ROOT)
    ).replace("\\", "/"):
        findings.append(PolicyFinding(
            "source-inventory",
            str(policy.get("source_inventory")),
            "policy must reference the canonical Navsys inventory",
        ))

    categories = set(policy.get("categories", {}))
    if categories != EXPECTED_CATEGORIES:
        findings.append(PolicyFinding(
            "category-set",
            "categories",
            "correctness, abi and performance must be separate categories",
        ))
    for category, row in policy.get("categories", {}).items():
        if (
            not isinstance(row.get("verdict_source"), str)
            or not row["verdict_source"]
            or not isinstance(row.get("must_not_use"), list)
            or not row["must_not_use"]
        ):
            findings.append(PolicyFinding(
                "category-contract",
                category,
                "category requires a verdict source and exclusion rules",
            ))
    layers = set(policy.get("layers", {}))
    if layers != EXPECTED_LAYERS:
        findings.append(PolicyFinding(
            "layer-set",
            "layers",
            "module_static, root_shared, wrapper and installed_sdk are required",
        ))

    for layer, row in policy.get("layers", {}).items():
        layer_categories = set(row.get("categories", []))
        if layer_categories != EXPECTED_LAYER_CATEGORIES.get(layer, set()):
            findings.append(PolicyFinding(
                "layer-category", layer, "layer has invalid category coverage"
            ))
        templates = row.get("command_templates", [])
        if not templates or any(
            not command
            or not all(isinstance(token, str) and token for token in command)
            for command in templates
        ):
            findings.append(PolicyFinding(
                "command-template", layer, "layer command templates are incomplete"
            ))

    coverage = policy.get("coverage", {})
    headers = inventory.get("headers", [])
    owners = {row["owner_todo"] for row in headers}
    baseline_owner = coverage.get("baseline_owner")
    child_owners = owners - {baseline_owner}
    actual_coverage = {
        "headers": len(headers),
        "owner_todos": len(owners),
        "child_owner_todos": len(child_owners),
    }
    for field, actual in actual_coverage.items():
        if coverage.get(field) != actual:
            findings.append(PolicyFinding(
                "coverage-mismatch",
                field,
                f"policy records {coverage.get(field)!r}, inventory has {actual}",
            ))
    if baseline_owner not in owners:
        findings.append(PolicyFinding(
            "baseline-owner",
            str(baseline_owner),
            "baseline owner is not present in the Navsys inventory",
        ))
    for owner in sorted(owners):
        if not (REPOSITORY_ROOT / owner).is_file():
            findings.append(PolicyFinding(
                "missing-owner-todo", owner, "owner TODO does not exist"
            ))

    record_schema = policy.get("record_schema", {})
    required_fields = record_schema.get("required_fields", [])
    if (
        len(required_fields) != len(set(required_fields))
        or set(required_fields) != EXPECTED_RECORD_FIELDS
    ):
        findings.append(PolicyFinding(
            "record-field-set",
            "record_schema",
            "required evidence fields are missing, duplicated or unexpected",
        ))
    if set(record_schema.get("verdicts", [])) != EXPECTED_VERDICTS:
        findings.append(PolicyFinding(
            "verdict-set", "record_schema", "record verdict vocabulary changed"
        ))
    if set(record_schema.get("rules", [])) != EXPECTED_RECORD_RULES:
        findings.append(PolicyFinding(
            "record-rule-set",
            "record_schema",
            "fail-closed evidence rules changed",
        ))
    artifact_root = _relative_path(record_schema.get("artifact_root", ""))
    if artifact_root != EXPECTED_ARTIFACT_ROOT:
        findings.append(PolicyFinding(
            "artifact-root",
            "record_schema",
            "artifact root must be the canonical repository-relative evidence path",
        ))
    return findings


def validate_record(
    policy: dict,
    inventory: dict,
    record: dict,
) -> list[PolicyFinding]:
    findings: list[PolicyFinding] = []
    schema = policy["record_schema"]
    for field in schema["required_fields"]:
        if field not in record:
            findings.append(PolicyFinding(
                "missing-record-field", field, "required evidence field is absent"
            ))

    header_rows = {
        row["path"]: row for row in inventory.get("headers", [])
    }
    header = record.get("header")
    if header not in header_rows:
        findings.append(PolicyFinding(
            "unknown-header", str(header), "header is absent from the inventory"
        ))
    elif record.get("owner_todo") != header_rows[header]["owner_todo"]:
        findings.append(PolicyFinding(
            "owner-mismatch",
            str(header),
            "record owner does not match the inventory owner",
        ))

    category = record.get("category")
    layer = record.get("layer")
    if category not in policy.get("categories", {}):
        findings.append(PolicyFinding(
            "unknown-category", str(category), "unknown result category"
        ))
    if layer not in policy.get("layers", {}):
        findings.append(PolicyFinding(
            "unknown-layer", str(layer), "unknown conformance layer"
        ))
    elif category not in policy["layers"][layer]["categories"]:
        findings.append(PolicyFinding(
            "layer-category",
            f"{layer}:{category}",
            "category is not valid for this layer",
        ))

    if not REVISION_PATTERN.fullmatch(str(record.get("revision", ""))):
        findings.append(PolicyFinding(
            "revision", str(record.get("revision")), "revision must be a Git hex id"
        ))
    for field in (
        "host",
        "target",
        "toolchain",
        "configuration",
    ):
        if not isinstance(record.get(field), str) or not record[field]:
            findings.append(PolicyFinding(
                "record-type", field, "field must be a non-empty string"
            ))
    command = record.get("command")
    if not isinstance(command, list) or not command or not all(
        isinstance(token, str) and token for token in command
    ):
        findings.append(PolicyFinding(
            "command", str(header), "command must be a non-empty argument list"
        ))
    elif layer in policy.get("layers", {}) and not any(
        _command_matches(template, command)
        for template in policy["layers"][layer]["command_templates"]
    ):
        findings.append(PolicyFinding(
            "command-template-mismatch",
            str(layer),
            "record command does not match a template for its layer",
        ))

    artifacts = record.get("artifacts")
    artifact_root = PurePosixPath(schema["artifact_root"])
    if not isinstance(artifacts, list):
        findings.append(PolicyFinding(
            "artifacts", str(header), "artifacts must be a list"
        ))
        artifacts = []
    for artifact in artifacts:
        path = _relative_path(artifact) if isinstance(artifact, str) else None
        if path is None or not path.is_relative_to(artifact_root):
            findings.append(PolicyFinding(
                "artifact-path",
                str(artifact),
                "artifact must be repository-relative and under artifact_root",
            ))

    verdict = record.get("verdict")
    if verdict not in EXPECTED_VERDICTS:
        findings.append(PolicyFinding(
            "verdict", str(verdict), "unknown evidence verdict"
        ))
    if verdict in {"pass", "fail"} and not artifacts:
        findings.append(PolicyFinding(
            "missing-artifact",
            str(header),
            f"{verdict} evidence requires at least one artifact",
        ))
    if verdict in {"not-run", "not-applicable"} and not record.get("limitations"):
        findings.append(PolicyFinding(
            "missing-limitation",
            str(header),
            f"{verdict} evidence requires a limitation",
        ))
    assertions = record.get("assertions")
    if not isinstance(assertions, int) or isinstance(assertions, bool) or assertions < 0:
        findings.append(PolicyFinding(
            "record-type",
            "assertions",
            "assertions must be a non-negative integer",
        ))
    limitations = record.get("limitations")
    if not isinstance(limitations, list) or not all(
        isinstance(item, str) and item for item in limitations
    ):
        findings.append(PolicyFinding(
            "record-type",
            "limitations",
            "limitations must be a list of non-empty strings",
        ))
    return findings


def validate_artifact_payload(
    record: dict,
    artifact: str,
    payload: dict,
) -> list[PolicyFinding]:
    findings: list[PolicyFinding] = []
    if (
        "revision" in payload
        and payload["revision"] != record.get("revision")
    ):
        findings.append(PolicyFinding(
            "artifact-revision-mismatch",
            artifact,
            "artifact revision does not match its evidence record",
        ))
    if "header" in payload and payload["header"] != record.get("header"):
        findings.append(PolicyFinding(
            "artifact-header-mismatch",
            artifact,
            "artifact header does not match its evidence record",
        ))
    return findings


def validate_evidence_tree(
    policy: dict,
    inventory: dict,
    evidence_root: Path = EVIDENCE_ROOT,
) -> tuple[list[PolicyFinding], int]:
    """Validate discovered ``*.record.json`` files and referenced artifacts."""
    findings = validate_policy(policy, inventory)
    record_paths = sorted(evidence_root.rglob("*.record.json"))
    if not record_paths:
        findings.append(PolicyFinding(
            "no-evidence-record",
            str(evidence_root),
            "evidence tree contains no record files",
        ))
        return findings, 0

    identities: dict[tuple, Path] = {}
    for record_path in record_paths:
        try:
            record = json.loads(record_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as error:
            findings.append(PolicyFinding(
                "invalid-record-json",
                str(record_path),
                str(error),
            ))
            continue

        findings.extend(validate_record(policy, inventory, record))
        identity = tuple(record.get(field) for field in (
            "header",
            "category",
            "layer",
            "revision",
            "target",
            "toolchain",
            "configuration",
        ))
        if identity in identities:
            findings.append(PolicyFinding(
                "duplicate-record",
                str(record_path),
                f"duplicates {identities[identity]}",
            ))
        else:
            identities[identity] = record_path

        for artifact in record.get("artifacts", []):
            if not isinstance(artifact, str) or _relative_path(artifact) is None:
                continue
            artifact_path = REPOSITORY_ROOT / PurePosixPath(artifact)
            if not artifact_path.is_file():
                findings.append(PolicyFinding(
                    "missing-artifact-file",
                    artifact,
                    "record references an artifact that does not exist",
                ))
                continue
            if artifact_path.suffix == ".json":
                try:
                    payload = json.loads(
                        artifact_path.read_text(encoding="utf-8")
                    )
                except (OSError, json.JSONDecodeError) as error:
                    findings.append(PolicyFinding(
                        "invalid-artifact-json",
                        artifact,
                        str(error),
                    ))
                else:
                    findings.extend(validate_artifact_payload(
                        record,
                        artifact,
                        payload,
                    ))
    return findings, len(record_paths)
