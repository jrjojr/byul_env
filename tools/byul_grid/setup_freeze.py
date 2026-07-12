from pathlib import Path
import importlib.util
import os
import platform
import sys

from cx_Freeze import Executable, setup


ROOT = Path(__file__).resolve().parent
REPOSITORY_ROOT = ROOT.parents[1]
SYSTEM = platform.system()
SYSTEM_NAME = {"Windows": "windows", "Linux": "linux", "Darwin": "macos"}.get(
    SYSTEM, SYSTEM.lower()
)
MACHINE_NAME = {
    "AMD64": "x86_64",
    "x86_64": "x86_64",
    "arm64": "arm64",
    "aarch64": "arm64",
}.get(platform.machine(), platform.machine().lower())
PLATFORM_TAG = f"{SYSTEM_NAME}-{MACHINE_NAME}"
SOURCE_ROOT = ROOT / "byul_grid"
MODULE_PATHS = [
    SOURCE_ROOT,
    SOURCE_ROOT / "gui",
    SOURCE_ROOT / "wrapper" / "modules",
]

# Internal modules currently use top-level imports. Expose the same lookup
# roots to cx_Freeze's module finder that the source launcher uses at runtime.
for module_path in reversed(MODULE_PATHS):
    module_path_string = str(module_path)
    if module_path_string not in sys.path:
        sys.path.insert(0, module_path_string)

configured_library_path = os.environ.get("BYUL_LIBRARY_PATH")

if SYSTEM == "Windows":
    library_name = "byul.dll"
    default_library_path = (
        REPOSITORY_ROOT
        / "build_win_msvc_release"
        / "bin"
        / "Release"
        / library_name
    )
elif SYSTEM == "Linux":
    library_name = "libbyul.so"
    default_library_path = REPOSITORY_ROOT / "build_release" / "lib" / library_name
elif SYSTEM == "Darwin":
    library_name = "libbyul.dylib"
    default_library_path = REPOSITORY_ROOT / "build_macos_release" / "lib" / library_name
else:
    raise RuntimeError(f"Unsupported platform: {SYSTEM}")

library_path = (
    Path(configured_library_path).expanduser().resolve()
    if configured_library_path
    else default_library_path
)

pycparser_spec = importlib.util.find_spec("pycparser")
if pycparser_spec is None or pycparser_spec.submodule_search_locations is None:
    raise ModuleNotFoundError("pycparser is required for packaging")
pycparser_path = Path(next(iter(pycparser_spec.submodule_search_locations)))

include_files = [
    (ROOT / "assets" / "images", "assets/images"),
    (pycparser_path, "lib/pycparser"),
]
if not library_path.is_file():
    raise FileNotFoundError(
        f"BYUL native library was not found: {library_path}. "
        "Build the BYUL Release target before packaging."
    )
include_files.append((library_path, library_name))

build_options = {
    "build_exe": str(ROOT / "build" / PLATFORM_TAG / "byul_grid"),
    "excludes": ["pycparser"],
    "include_files": include_files,
    "includes": ["cffi", "pandas", "psutil", "pyqtgraph", "PySide6"],
    "include_msvcr": SYSTEM == "Windows",
    "path": sys.path,
}

base = "gui" if SYSTEM == "Windows" else None
target_name = "byul_grid.exe" if SYSTEM == "Windows" else "byul_grid"
icon = str(ROOT / "assets" / "icons" / "byul_grid.ico") if SYSTEM == "Windows" else None

setup(
    name="BYUL Grid",
    version="0.1.0",
    description="Grid-based navigation visualization and simulation tool",
    options={"build_exe": build_options},
    executables=[
        Executable(
            str(ROOT / "byul_grid" / "byul_grid.py"),
            base=base,
            icon=icon,
            target_name=target_name,
        )
    ],
)
