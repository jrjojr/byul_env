import re
from pathlib import Path

from byul_wrapper.ffi_core import C
from byul_wrapper_generator import parse_header


REPOSITORY_ROOT = Path(__file__).resolve().parents[4]
HEADER = REPOSITORY_ROOT / "byul" / "navsys" / "coord" / "coord_list.h"
SOURCE = REPOSITORY_ROOT / "byul" / "navsys" / "coord" / "coord_list.cpp"
WRAPPER = (
    REPOSITORY_ROOT
    / "tools"
    / "python"
    / "byul_wrapper"
    / "byul_wrapper"
    / "coord_list.py"
)


def test_coord_list_declaration_definition_export_wrapper_intersection():
    declarations = {item.name for item in parse_header(HEADER)}
    source = SOURCE.read_text(encoding="utf-8")
    wrapper = WRAPPER.read_text(encoding="utf-8")
    cdef = wrapper.split('ffi.cdef("""', 1)[1].split('""")', 1)[0]

    missing_definitions = {
        name
        for name in declarations
        if not re.search(
            rf"\b{re.escape(name)}\s*\([^;{{}}]*\)\s*\{{",
            source,
            re.MULTILINE | re.DOTALL,
        )
    }
    wrapper_declarations = set(
        re.findall(r"\b(coord_list_\w+)\s*\(", cdef)
    )
    missing_exports = {
        name for name in declarations if not hasattr(C, name)
    }

    assert not missing_definitions
    assert declarations <= wrapper_declarations
    assert not missing_exports
