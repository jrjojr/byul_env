"""Prepare the packaging environment and build the BYUL Grid application."""

from __future__ import annotations

import argparse
from datetime import datetime
import hashlib
import os
from pathlib import Path
import platform
import shutil
import subprocess
import sys


ROOT = Path(__file__).resolve().parent
DEFAULT_VENV = ROOT / ".venv-package"
REQUIREMENTS = ROOT / "requirements-package.txt"
STAMP_NAME = ".requirements-package.sha256"


def platform_tag() -> str:
    system_names = {"Windows": "windows", "Linux": "linux", "Darwin": "macos"}
    system = system_names.get(platform.system(), platform.system().lower())
    machine_names = {"AMD64": "x86_64", "x86_64": "x86_64", "arm64": "arm64", "aarch64": "arm64"}
    machine = machine_names.get(platform.machine(), platform.machine().lower())
    return f"{system}-{machine}"


PLATFORM_BUILD_ROOT = ROOT / "build" / platform_tag()
DEFAULT_OUTPUT = PLATFORM_BUILD_ROOT / "byul_grid"
LEGACY_PACKAGE_ROOT = ROOT / "build" / "byul_grid"


class LegacyOutputLockedError(RuntimeError):
    def __init__(self, path: Path) -> None:
        super().__init__(str(path))
        self.path = path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Create the packaging environment and build BYUL Grid."
    )
    parser.add_argument(
        "--venv",
        type=Path,
        default=DEFAULT_VENV,
        help="packaging virtual environment (default: .venv-package)",
    )
    parser.add_argument(
        "--force-dependencies",
        action="store_true",
        help="reinstall packaging dependencies even when requirements are unchanged",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=DEFAULT_OUTPUT,
        help=f"output directory (default: {DEFAULT_OUTPUT.relative_to(ROOT)})",
    )
    parser.add_argument(
        "--on-locked",
        choices=("ask", "new", "cancel"),
        default="ask",
        help="action when the output directory is locked (default: ask)",
    )
    return parser.parse_args()


def venv_python(venv: Path) -> Path:
    if sys.platform == "win32":
        return venv / "Scripts" / "python.exe"
    return venv / "bin" / "python"


def run(command: list[str]) -> None:
    print(f"[INFO] Running: {' '.join(command)}", flush=True)
    subprocess.run(command, cwd=ROOT, check=True)


def requirements_digest() -> str:
    digest = hashlib.sha256()
    for path in (REQUIREMENTS, ROOT / "requirements.txt"):
        digest.update(path.read_bytes())
    return digest.hexdigest()


def prepare_environment(venv: Path, force_dependencies: bool) -> Path:
    python = venv_python(venv)
    if not python.is_file():
        print(f"[INFO] Creating packaging virtual environment: {venv}", flush=True)
        run([sys.executable, "-m", "venv", str(venv)])

    stamp = venv / STAMP_NAME
    expected_digest = requirements_digest()
    installed_digest = stamp.read_text(encoding="utf-8").strip() if stamp.is_file() else ""

    if force_dependencies or installed_digest != expected_digest:
        print("[INFO] Installing packaging dependencies...", flush=True)
        run([str(python), "-m", "pip", "install", "-r", str(REQUIREMENTS)])
        stamp.write_text(expected_digest + "\n", encoding="utf-8")

    return python


def directory_is_locked(path: Path) -> bool:
    """Check a directory lock without deleting or changing its contents."""
    if not path.exists():
        return False

    probe = path.with_name(f".{path.name}.lock-check-{os.getpid()}")
    renamed = False
    try:
        path.rename(probe)
        renamed = True
        probe.rename(path)
    except OSError:
        if renamed and probe.exists() and not path.exists():
            try:
                probe.rename(path)
            except OSError:
                pass
        return True
    return False


def alternate_output(path: Path) -> Path:
    timestamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    candidate = path.with_name(f"{path.name}-{timestamp}")
    sequence = 2
    while candidate.exists():
        candidate = path.with_name(f"{path.name}-{timestamp}-{sequence}")
        sequence += 1
    return candidate


def select_output(path: Path, on_locked: str) -> Path | None:
    if not directory_is_locked(path):
        return path

    print(f"[ERROR] Output directory is locked: {path}", file=sys.stderr, flush=True)
    print(
        "[ERROR] Another process is using the directory or a file inside it. "
        "Close BYUL Grid and any program holding the folder, then try again.",
        file=sys.stderr,
        flush=True,
    )
    action = on_locked
    if action == "ask":
        if not sys.stdin.isatty():
            print("[ERROR] Cannot prompt in a non-interactive session.", file=sys.stderr)
            return None
        answer = input("Create a new output directory? [Y/n]: ").strip().lower()
        action = "new" if answer in ("", "y", "yes") else "cancel"

    if action == "new":
        new_path = alternate_output(path)
        print(f"[INFO] Using a new output directory: {new_path}", flush=True)
        return new_path

    print("[ERROR] Packaging cancelled because the output directory is locked.", file=sys.stderr)
    return None


def clean_output(path: Path) -> None:
    if path.exists():
        print(f"[INFO] Cleaning previous package: {path}", flush=True)
        shutil.rmtree(path)


def clean_legacy_layout() -> None:
    """Remove outputs created with the old build/byul_grid/<platform> layout."""
    if not LEGACY_PACKAGE_ROOT.is_dir():
        return

    if directory_is_locked(LEGACY_PACKAGE_ROOT):
        raise LegacyOutputLockedError(LEGACY_PACKAGE_ROOT)

    print(f"[INFO] Cleaning legacy package layout: {LEGACY_PACKAGE_ROOT}", flush=True)
    shutil.rmtree(LEGACY_PACKAGE_ROOT)


def clean_flat_platform_layout() -> None:
    """Remove files from the former build/<platform> application layout."""
    if not PLATFORM_BUILD_ROOT.is_dir():
        return

    markers = (
        PLATFORM_BUILD_ROOT / "byul_grid.exe",
        PLATFORM_BUILD_ROOT / "byul_grid",
        PLATFORM_BUILD_ROOT / "frozen_application_license.txt",
    )
    if not any(marker.is_file() for marker in markers):
        return

    targets = [
        child
        for child in PLATFORM_BUILD_ROOT.iterdir()
        if not (child.name == "byul_grid" and child.is_dir())
    ]
    for target in targets:
        if directory_is_locked(target):
            raise LegacyOutputLockedError(target)

    print(f"[INFO] Cleaning flat platform layout: {PLATFORM_BUILD_ROOT}", flush=True)
    for target in targets:
        if target.is_dir():
            shutil.rmtree(target)
        else:
            target.unlink()


def native_library_name() -> str:
    names = {
        "Windows": "byul.dll",
        "Linux": "libbyul.so",
        "Darwin": "libbyul.dylib",
    }
    try:
        return names[platform.system()]
    except KeyError as error:
        raise RuntimeError(f"Unsupported platform: {platform.system()}") from error


def arrange_package(output: Path) -> None:
    if platform.system() != "Windows":
        library_name = native_library_name()
        source = output / library_name
        destination_directory = output / "lib"
        destination_directory.mkdir(parents=True, exist_ok=True)
        if source.is_file():
            source.replace(destination_directory / library_name)

    licenses = output / "licenses"
    licenses.mkdir(parents=True, exist_ok=True)

    frozen_license = output / "frozen_application_license.txt"
    if frozen_license.is_file():
        frozen_license.replace(licenses / "cx_Freeze-license.txt")

    repository_root = ROOT.parents[1]
    for name in ("LICENSE", "THIRD_PARTY_NOTICES.md"):
        source = repository_root / name
        if source.is_file():
            shutil.copy2(source, licenses / name)


def main() -> int:
    args = parse_args()
    venv = args.venv.expanduser().resolve()
    requested_output = args.output.expanduser().resolve()
    output = select_output(requested_output, args.on_locked)
    if output is None:
        return 2

    try:
        clean_legacy_layout()
        clean_flat_platform_layout()
        clean_output(output)
        python = prepare_environment(venv, args.force_dependencies)
        run(
            [
                str(python),
                str(ROOT / "setup_freeze.py"),
                "build_exe",
                "--build-exe",
                str(output),
            ]
        )
        arrange_package(output)
    except LegacyOutputLockedError as error:
        print(f"[ERROR] Legacy package output is locked: {error.path}", file=sys.stderr)
        print(
            "[ERROR] Close BYUL Grid or any process using this file, then run "
            "the build again. Nothing was deleted.",
            file=sys.stderr,
        )
        return 2
    except (OSError, subprocess.CalledProcessError) as error:
        print(f"[ERROR] BYUL Grid packaging failed: {error}", file=sys.stderr)
        return 1

    print(f"[INFO] BYUL Grid package: {output}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
