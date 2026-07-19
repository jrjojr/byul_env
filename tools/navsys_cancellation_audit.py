#!/usr/bin/env python3
"""Check route-finder cooperative-cancellation polling coverage."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
import re


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SOURCE_ROOT = REPOSITORY_ROOT / "byul/navsys/route_finder"
EXPECTED_POLL_COUNTS = {
    "astar.cpp": 1,
    "bfs.cpp": 1,
    "dfs.cpp": 1,
    "dijkstra.cpp": 1,
    "fast_marching.cpp": 2,
    "fringe_search.cpp": 2,
    "greedy_best_first.cpp": 1,
    "ida_star.cpp": 2,
    "rta_star.cpp": 1,
    "sma_star.cpp": 1,
    "weighted_astar.cpp": 1,
}
POLL_PATTERN = re.compile(r"\broute_finder_poll_cancel_internal\s*\(")


@dataclass(frozen=True)
class CancellationFinding:
    code: str
    subject: str
    message: str


def load_algorithm_sources(source_root: Path) -> dict[str, str]:
    """Load the expected algorithm implementation sources."""
    return {
        name: (source_root / name).read_text(encoding="utf-8")
        for name in EXPECTED_POLL_COUNTS
        if (source_root / name).is_file()
    }


def cancellation_findings(
    sources: dict[str, str],
) -> list[CancellationFinding]:
    """Return missing-source and cancellation-poll coverage findings."""
    findings: list[CancellationFinding] = []
    for name, expected_count in EXPECTED_POLL_COUNTS.items():
        if name not in sources:
            findings.append(CancellationFinding(
                "missing-algorithm-source",
                name,
                "expected dispatcher algorithm source is absent",
            ))
            continue

        implementation = re.sub(
            r"^\s*extern\s+bool\s+"
            r"route_finder_poll_cancel_internal\s*\(\s*void\s*\)\s*;\s*$",
            "",
            sources[name],
            flags=re.MULTILINE,
        )
        actual_count = len(POLL_PATTERN.findall(implementation))
        if actual_count != expected_count:
            findings.append(CancellationFinding(
                "cancel-poll-count",
                name,
                (
                    f"expected {expected_count} cancellation polling points, "
                    f"found {actual_count}"
                ),
            ))
    return findings


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Check that every route-finder dispatcher algorithm retains its "
            "approved cooperative-cancellation polling points."
        )
    )
    parser.add_argument(
        "--source-root",
        type=Path,
        default=DEFAULT_SOURCE_ROOT,
        help="route_finder source directory (default: repository source tree)",
    )
    parser.add_argument(
        "--quiet",
        action="store_true",
        help="suppress the success summary",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    source_root = args.source_root.resolve()
    sources = load_algorithm_sources(source_root)
    findings = cancellation_findings(sources)
    for finding in findings:
        print(
            f"[{finding.code}] {finding.subject}: {finding.message}"
        )
    if findings:
        return 1
    if not args.quiet:
        print(
            "[NAVSYS-CANCELLATION-AUDIT] "
            f"algorithms={len(EXPECTED_POLL_COUNTS)} "
            f"polling-points={sum(EXPECTED_POLL_COUNTS.values())} errors=0"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
