"""Parse and lint the machine-readable part of BYUL public C headers.

The normative rules live in ``docs/ko/byul-sdk``.  This module implements the
parts needed by the wrapper generator; it deliberately does not copy the rule
documents or infer ownership from prose.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
import re


KNOWN_BYUL_TAGS = frozenset(
    {
        "blocking",
        "buffer",
        "callback_failure",
        "callback_thread",
        "capacity",
        "copy_semantics",
        "count",
        "encoding",
        "error",
        "invalidates",
        "lifetime",
        "memory",
        "nullable",
        "preserves",
        "range",
        "reentrant",
        "requires",
        "requires_init",
        "side_effect",
        "storage",
        "termination",
        "thread_safety",
        "unit",
        "value_semantics",
        "zero_valid",
    }
)

_COMMENT_AND_DECLARATION = re.compile(
    r"(?P<comment>/\*\*.*?\*/)?\s*"
    r"BYUL_API\s+(?P<declaration>.*?;)",
    re.DOTALL,
)
_FUNCTION = re.compile(
    r"^(?P<return_type>.+?)\s+"
    r"(?P<name>[A-Za-z_]\w*)\s*\((?P<parameters>.*)\)\s*;$",
    re.DOTALL,
)
_PARAM_TAG = re.compile(
    r"@param(?:\[(?P<direction>in|out|in,out)\])?\s+"
    r"(?P<name>[A-Za-z_]\w*)\s*(?P<description>.*)"
)
_BYUL_TAG = re.compile(r"@byul\.(?P<name>[A-Za-z_]\w*)\s*(?P<value>.*)")


@dataclass(frozen=True)
class Parameter:
    declaration: str
    name: str
    pointer: bool
    const: bool


@dataclass
class ApiDeclaration:
    name: str
    return_type: str
    parameters: tuple[Parameter, ...]
    declaration: str
    comment: str | None
    brief: str | None = None
    parameter_docs: dict[str, str] = field(default_factory=dict)
    parameter_directions: dict[str, str] = field(default_factory=dict)
    has_return_doc: bool = False
    retval_docs: tuple[str, ...] = ()
    byul_tags: dict[str, tuple[str, ...]] = field(default_factory=dict)

    @property
    def returns_pointer(self) -> bool:
        return "*" in self.return_type


@dataclass(frozen=True)
class LintIssue:
    severity: str
    code: str
    message: str
    symbol: str | None = None


@dataclass(frozen=True)
class HeaderAudit:
    path: Path
    declarations: tuple[ApiDeclaration, ...]
    issues: tuple[LintIssue, ...]

    @property
    def error_count(self) -> int:
        return sum(issue.severity == "error" for issue in self.issues)


def _clean_declaration(text: str) -> str:
    return re.sub(r"\s+", " ", text).strip()


def _split_parameters(text: str) -> tuple[str, ...]:
    text = text.strip()
    if not text or text == "void":
        return ()
    parts: list[str] = []
    start = 0
    depth = 0
    for index, char in enumerate(text):
        if char == "(":
            depth += 1
        elif char == ")":
            depth -= 1
        elif char == "," and depth == 0:
            parts.append(text[start:index].strip())
            start = index + 1
    parts.append(text[start:].strip())
    return tuple(part for part in parts if part)


def _parse_parameter(declaration: str, index: int) -> Parameter:
    declaration = _clean_declaration(declaration)
    callback_name = re.search(r"\(\s*\*\s*([A-Za-z_]\w*)\s*\)", declaration)
    if callback_name:
        name = callback_name.group(1)
    else:
        name_match = re.search(r"([A-Za-z_]\w*)\s*(?:\[[^]]*\])?$", declaration)
        if not name_match:
            name = f"__unnamed_{index}"
        else:
            name = name_match.group(1)
    return Parameter(
        declaration=declaration,
        name=name,
        pointer="*" in declaration or "[" in declaration,
        const=bool(re.search(r"\bconst\b", declaration)),
    )


def _comment_lines(comment: str) -> tuple[str, ...]:
    body = comment.removeprefix("/**").removesuffix("*/")
    return tuple(re.sub(r"^\s*\*\s?", "", line).strip() for line in body.splitlines())


def _apply_comment(declaration: ApiDeclaration) -> None:
    if declaration.comment is None:
        return
    retval_docs: list[str] = []
    tags: dict[str, list[str]] = {}
    for line in _comment_lines(declaration.comment):
        if line.startswith("@brief"):
            declaration.brief = line.removeprefix("@brief").strip()
            continue
        parameter = _PARAM_TAG.match(line)
        if parameter:
            name = parameter.group("name")
            declaration.parameter_docs[name] = parameter.group("description").strip()
            if parameter.group("direction"):
                declaration.parameter_directions[name] = parameter.group("direction")
            continue
        if line.startswith("@return"):
            declaration.has_return_doc = True
            continue
        if line.startswith("@retval"):
            retval_docs.append(line.removeprefix("@retval").strip())
            continue
        byul = _BYUL_TAG.match(line)
        if byul:
            tags.setdefault(byul.group("name"), []).append(byul.group("value").strip())
    declaration.retval_docs = tuple(retval_docs)
    declaration.byul_tags = {name: tuple(values) for name, values in tags.items()}


def parse_header(path: Path) -> tuple[ApiDeclaration, ...]:
    """Return public function declarations and their adjacent Doxygen blocks."""
    source = path.read_text(encoding="utf-8")
    # Preserve adjacent Doxygen blocks but remove ordinary block and line
    # comments before looking for BYUL_API. Commented-out declarations are
    # historical text, not public ABI.
    source = re.sub(r"/\*(?!\*).*?\*/", "", source, flags=re.DOTALL)
    source = re.sub(r"//[^\n]*", "", source)
    declarations: list[ApiDeclaration] = []
    for match in _COMMENT_AND_DECLARATION.finditer(source):
        raw = _clean_declaration(match.group("declaration"))
        function = _FUNCTION.match(raw)
        if not function:
            continue
        parameters = tuple(
            _parse_parameter(item, index)
            for index, item in enumerate(
                _split_parameters(function.group("parameters")), start=1
            )
        )
        declaration = ApiDeclaration(
            name=function.group("name"),
            return_type=function.group("return_type").strip(),
            parameters=parameters,
            declaration=raw,
            comment=match.group("comment"),
        )
        _apply_comment(declaration)
        declarations.append(declaration)
    return tuple(declarations)


def _lint(declaration: ApiDeclaration) -> list[LintIssue]:
    issues: list[LintIssue] = []

    def add(severity: str, code: str, message: str) -> None:
        issues.append(LintIssue(severity, code, message, declaration.name))

    if declaration.comment is None:
        add("error", "missing-doxygen", "public function has no adjacent Doxygen block")
        return issues
    if not declaration.brief:
        add("error", "missing-brief", "public function has no @brief")

    parameter_names = {parameter.name for parameter in declaration.parameters}
    for parameter in declaration.parameters:
        if parameter.name.startswith("__unnamed_"):
            add("error", "unnamed-parameter", f"public parameter has no name: {parameter.declaration}")
            continue
        if parameter.name not in declaration.parameter_docs:
            add("error", "missing-param", f"parameter '{parameter.name}' has no @param")
        elif parameter.name not in declaration.parameter_directions:
            add("error", "missing-param-direction", f"parameter '{parameter.name}' has no direction")
        if parameter.pointer:
            nullable_targets = {
                value.split()[0]
                for value in declaration.byul_tags.get("nullable", ())
                if value
            }
            if parameter.name not in nullable_targets:
                add("error", "missing-nullable", f"pointer parameter '{parameter.name}' has no @byul.nullable")

    for documented in declaration.parameter_docs:
        if documented not in parameter_names:
            add("error", "unknown-param", f"@param references unknown parameter '{documented}'")

    if declaration.return_type != "void" and not declaration.has_return_doc:
        add("error", "missing-return", "non-void function has no @return")
    if declaration.returns_pointer:
        nullable = declaration.byul_tags.get("nullable", ())
        lifetime = declaration.byul_tags.get("lifetime", ())
        if not any(value.startswith("return ") for value in nullable):
            add("error", "missing-return-nullable", "pointer return has no @byul.nullable return")
        if not any(value.startswith("return ") for value in lifetime):
            add("error", "missing-return-lifetime", "pointer return has no @byul.lifetime return")

    for tag in declaration.byul_tags:
        if tag not in KNOWN_BYUL_TAGS:
            add("error", "unknown-byul-tag", f"unknown @byul.{tag} tag")
    return issues


def audit_header(path: Path) -> HeaderAudit:
    declarations = parse_header(path)
    issues = tuple(issue for declaration in declarations for issue in _lint(declaration))
    return HeaderAudit(path=path, declarations=declarations, issues=issues)
