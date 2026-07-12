from cffi import FFI
from pathlib import Path
import os
import platform
import sys


ffi = FFI()

ffi.cdef("""
#define TRUE 1
#define FALSE 0
""")


def _library_name(system: str) -> str:
    if system == "Windows":
        return "byul.dll"
    if system == "Linux":
        return "libbyul.so"
    if system == "Darwin":
        return "libbyul.dylib"
    raise RuntimeError(f"Unsupported platform: {system}")


def _find_library() -> Path:
    system = platform.system()
    library_name = _library_name(system)
    package_root = Path(__file__).resolve().parent
    tool_root = Path(__file__).resolve().parents[1]
    repository_root = tool_root.parents[2]

    candidates = []
    if os.environ.get("BYUL_LIBRARY_PATH"):
        candidates.append(Path(os.environ["BYUL_LIBRARY_PATH"]).expanduser())
    candidates.append(package_root / "bin" / library_name)
    if getattr(sys, "frozen", False):
        executable_dir = Path(sys.executable).resolve().parent
        if system == "Windows":
            candidates.append(executable_dir / library_name)
        else:
            candidates.append(executable_dir / "lib" / library_name)

    if system == "Windows":
        candidates.append(
            repository_root
            / "build_win_msvc"
            / "bin"
            / "Release"
            / library_name
        )
    elif system == "Linux":
        candidates.append(repository_root / "build_release" / "lib" / library_name)
    else:
        candidates.append(
            repository_root / "build_macos_release" / "lib" / library_name
        )

    candidates.append(tool_root / "bin" / library_name)

    for candidate in candidates:
        if candidate.is_file():
            return candidate.resolve()

    searched = "\n  ".join(str(path) for path in candidates)
    raise RuntimeError(
        f"BYUL native library was not found for {system}.\n"
        f"Build BYUL in this repository or set BYUL_LIBRARY_PATH. Searched:\n"
        f"  {searched}"
    )


routefinder_path = _find_library()

if platform.system() == "Windows":
    if hasattr(os, "add_dll_directory"):
        os.add_dll_directory(str(routefinder_path.parent))
        mingw_bin = os.environ.get("MINGW_BIN_PATH")
        if mingw_bin:
            os.add_dll_directory(str(Path(mingw_bin).expanduser()))
    else:
        os.environ["PATH"] = (
            str(routefinder_path.parent) + os.pathsep + os.environ.get("PATH", "")
        )

try:
    C = ffi.dlopen(str(routefinder_path))
except OSError as error:
    raise RuntimeError(
        f"Failed to load BYUL native library: {routefinder_path}\n{error}"
    ) from error
