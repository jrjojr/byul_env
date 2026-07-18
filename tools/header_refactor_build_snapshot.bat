@echo off
setlocal
set "SCRIPT_DIR=%~dp0"
set "PYTHONUTF8=1"

if defined BYUL_PYTHON goto run_byul_python
if exist "%SCRIPT_DIR%python\.venv\Scripts\python.exe" goto run_venv
python --version >nul 2>&1
if not errorlevel 1 goto run_python
py -3 --version >nul 2>&1
if not errorlevel 1 goto run_py

echo ERROR: Python 3 is unavailable; set BYUL_PYTHON or create tools/python/.venv. 1>&2
exit /b 2

:run_byul_python
"%BYUL_PYTHON%" "%SCRIPT_DIR%header_refactor_build_snapshot.py" %*
exit /b %ERRORLEVEL%

:run_venv
"%SCRIPT_DIR%python\.venv\Scripts\python.exe" "%SCRIPT_DIR%header_refactor_build_snapshot.py" %*
exit /b %ERRORLEVEL%

:run_python
python "%SCRIPT_DIR%header_refactor_build_snapshot.py" %*
exit /b %ERRORLEVEL%

:run_py
py -3 "%SCRIPT_DIR%header_refactor_build_snapshot.py" %*
exit /b %ERRORLEVEL%
