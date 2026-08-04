import json
from pathlib import Path
import re


REPOSITORY_ROOT = Path(__file__).resolve().parents[4]
CONTRACT_PATH = (
    REPOSITORY_ROOT
    / "docs/ko/todo/navsys/maze-dispatcher-stage2-contract.json"
)
HEADER_PATH = REPOSITORY_ROOT / "byul/navsys/maze/maze.h"
SOURCE_PATH = REPOSITORY_ROOT / "byul/navsys/maze/maze.cpp"
TODO_ROOT = REPOSITORY_ROOT / "docs/ko/todo/navsys"


def _contract():
    return json.loads(CONTRACT_PATH.read_text(encoding="utf-8"))


def _header_enums():
    header = HEADER_PATH.read_text(encoding="utf-8")
    match = re.search(
        r"typedef\s+enum(?:\s+e_maze_type)?\s*\{"
        r"(?P<body>.*?)\}\s*maze_type_t\s*;",
        header,
        re.DOTALL,
    )
    assert match is not None
    return re.findall(r"\b(MAZE_TYPE_[A-Z_]+)\b", match.group("body"))


def _dispatcher_cases():
    source = SOURCE_PATH.read_text(encoding="utf-8")
    cases = dict(re.findall(
        r"case\s+(MAZE_TYPE_[A-Z_]+)\s*:\s*"
        r"maze\s*=\s*([a-zA-Z0-9_]+)\s*\(",
        source,
    ))
    fallback = re.search(
        r"default\s*:\s*maze\s*=\s*([a-zA-Z0-9_]+)\s*\(",
        source,
    )
    assert fallback is not None
    return cases, fallback.group(1)


def test_capability_matrix_matches_dispatcher_enum_and_source():
    contract = _contract()
    capabilities = contract["capabilities"]
    header_enums = _header_enums()
    cases, fallback = _dispatcher_cases()

    assert len(capabilities) == 11
    assert [row["enum"] for row in capabilities] == header_enums
    assert [row["value"] for row in capabilities] == list(range(11))
    assert {row["enum"]: row["implementation"] for row in capabilities} == cases
    assert fallback == "maze_make_kruskal"
    assert all((TODO_ROOT / row["owner_todo"]).is_file() for row in capabilities)


def test_generation_contract_is_advertised_and_logically_consistent():
    contract = _contract()
    rng = contract["rng_contract"]
    assert contract["rng_contract"]["replay_scope"].startswith(
        "cross-platform"
    )
    assert rng["algorithm"] == "pcg32-xsh-rr-v1"
    assert rng["multiplier"] == "6364136223846793005"
    assert len(rng["initialization"]) == 5
    assert len(rng["next_u32"]) == 5
    assert "threshold" in rng["bounded_sampling"]
    assert "standard-library" in rng["forbidden_sampling"]
    assert contract["termination_contract"]["failure_atomicity"].startswith(
        "no partial maze"
    )
    assert contract["support_policy"]["current_advertised_algorithms"] == [
        row["enum"] for row in contract["capabilities"]
    ]

    for row in contract["capabilities"]:
        dimensions = row["dimensions"]
        topology = row["topology"]
        assert dimensions["min_width"] >= 3
        assert dimensions["min_height"] >= 3
        assert dimensions["parity"] in {"odd", "any"}
        assert row["default_step_multiplier"] > 0
        assert row["advertisable_supported"] is True
        assert row["support_gate"]
        if topology["uniform_spanning_tree"]:
            assert topology["perfect"] is True
        if topology["perfect"] is True:
            assert topology["connected"] is True
            assert topology["acyclic"] is True

    by_enum = {row["enum"]: row for row in contract["capabilities"]}
    any_parity = [
        row["enum"]
        for row in contract["capabilities"]
        if row["dimensions"]["parity"] == "any"
    ]
    ust_algorithms = [
        row["enum"]
        for row in contract["capabilities"]
        if row["topology"]["uniform_spanning_tree"]
    ]
    assert any_parity == ["MAZE_TYPE_ROOM_BLEND"]
    assert ust_algorithms == ["MAZE_TYPE_ALDOUS_BRODER", "MAZE_TYPE_WILSON"]
    assert by_enum["MAZE_TYPE_RECURSIVE_DIVISION"]["topology"] == {
        "connected": True,
        "acyclic": True,
        "perfect": True,
        "uniform_spanning_tree": False,
        "border_blocked": True,
    }
