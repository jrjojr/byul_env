"""BYUL developer entry point for external CMake Starter."""

from __future__ import annotations

import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
STARTER_SOURCE = PROJECT_ROOT / "external" / "cmake_starter_env" / "src"

if not STARTER_SOURCE.is_dir():
    raise SystemExit(
        "Missing external/cmake_starter_env. Initialize the external project first."
    )

sys.path.insert(0, str(STARTER_SOURCE))

from cmake_starter import main  # noqa: E402


if __name__ == "__main__":
    raise SystemExit(main())
