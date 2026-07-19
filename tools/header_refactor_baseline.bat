@echo off
setlocal
set "SCRIPT_DIR=%~dp0"
set "PYTHONUTF8=1"

if defined BYUL_PYTHON goto run_byul_python

if exist "%SCRIPT_DIR%python\.venv\Scripts\python.exe" goto validate_venv

goto try_python

:run_byul_python
"%BYUL_PYTHON%" -c "import sys; raise SystemExit(0 if sys.version_info >= (3, 10) else 1)" >nul 2>&1
if errorlevel 1 goto invalid_byul_python
"%BYUL_PYTHON%" "%SCRIPT_DIR%header_refactor_baseline.py" %*
exit /b %ERRORLEVEL%

:validate_venv
"%SCRIPT_DIR%python\.venv\Scripts\python.exe" -c "import sys; raise SystemExit(0 if sys.version_info >= (3, 10) else 1)" >nul 2>&1
if not errorlevel 1 goto run_venv
echo WARNING: Ignoring broken repository Python environment: "%SCRIPT_DIR%python\.venv\Scripts\python.exe" 1>&2

:try_python
py -3 -c "import sys; raise SystemExit(0 if sys.version_info >= (3, 10) else 1)" >nul 2>&1
if not errorlevel 1 goto run_py

python -c "import sys; raise SystemExit(0 if sys.version_info >= (3, 10) else 1)" >nul 2>&1
if not errorlevel 1 goto run_python

echo ERROR: Python 3.10 or newer is unavailable; set BYUL_PYTHON or recreate tools/python/.venv. 1>&2
exit /b 2

:invalid_byul_python
echo ERROR: BYUL_PYTHON is not an executable Python 3.10 or newer: "%BYUL_PYTHON%" 1>&2
exit /b 2

:run_venv
"%SCRIPT_DIR%python\.venv\Scripts\python.exe" "%SCRIPT_DIR%header_refactor_baseline.py" %*
exit /b %ERRORLEVEL%

:run_python
python "%SCRIPT_DIR%header_refactor_baseline.py" %*
exit /b %ERRORLEVEL%

:run_py
py -3 "%SCRIPT_DIR%header_refactor_baseline.py" %*
exit /b %ERRORLEVEL%
