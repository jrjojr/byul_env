@echo off
setlocal EnableDelayedExpansion

set "PROJECT_ROOT=%~dp0"
set "ENTRYPOINT=%PROJECT_ROOT%byul_grid\byul_grid.py"
set "VENV_PYTHON=%PROJECT_ROOT%.venv\Scripts\python.exe"
set "BYUL_SOURCE=%PROJECT_ROOT%..\..\byul"
set "BYUL_BUILD=%PROJECT_ROOT%..\..\build_win_msvc_release"
set "BYUL_DLL=%BYUL_BUILD%\bin\Release\byul.dll"
set "CMAKE_EXE=cmake"

if not exist "%BYUL_SOURCE%\CMakeLists.txt" (
    echo [ERROR] byul_env source was not found: 1>&2
    echo         "%BYUL_SOURCE%\CMakeLists.txt" 1>&2
    set "EXIT_CODE=1"
    goto :done
)

where cmake >nul 2>&1
if errorlevel 1 (
    set "CMAKE_EXE=%ProgramFiles%\Microsoft Visual Studio\2022\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe"
    if not exist "!CMAKE_EXE!" (
        echo [ERROR] CMake was not found. Install CMake or add it to PATH. 1>&2
        set "EXIT_CODE=1"
        goto :done
    )
)

if not exist "%BYUL_DLL%" (
    echo [INFO] byul.dll was not found. Preparing BYUL Grid...

    if not exist "%BYUL_BUILD%\CMakeCache.txt" (
        echo [INFO] Configuring byul_env...
        "!CMAKE_EXE!" -S "%BYUL_SOURCE%" -B "%BYUL_BUILD%" -G "Visual Studio 17 2022" -A x64 -DCMAKE_BUILD_TYPE=Release
        if errorlevel 1 (
            echo [ERROR] byul_env configuration failed. 1>&2
            set "EXIT_CODE=1"
            goto :done
        )
    )

    echo [INFO] Building the current BYUL Release target...
    "!CMAKE_EXE!" --build "%BYUL_BUILD%" --target byul --config Release
    if errorlevel 1 (
        echo [ERROR] byul_env build failed. 1>&2
        set "EXIT_CODE=1"
        goto :done
    )

)

if not exist "%BYUL_DLL%" (
    echo [ERROR] Built byul.dll was not found: "%BYUL_DLL%" 1>&2
    set "EXIT_CODE=1"
    goto :done
)

set "BYUL_LIBRARY_PATH=%BYUL_DLL%"

if exist "%VENV_PYTHON%" (
    "%VENV_PYTHON%" "%ENTRYPOINT%" %*
    set "EXIT_CODE=%ERRORLEVEL%"
    if not "!EXIT_CODE!"=="9009" goto :done
)

where py >nul 2>&1
if not errorlevel 1 (
    py -3 "%ENTRYPOINT%" %*
    set "EXIT_CODE=%ERRORLEVEL%"
    goto :done
)

where python >nul 2>&1
if not errorlevel 1 (
    python "%ENTRYPOINT%" %*
    set "EXIT_CODE=%ERRORLEVEL%"
    goto :done
)

echo [ERROR] Python 3 executable was not found. 1>&2
echo Create .venv or install Python, then try again. 1>&2
set "EXIT_CODE=1"

:done
endlocal & exit /b %EXIT_CODE%
