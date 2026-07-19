@echo off
setlocal

set "ROOT=%~dp0"
set "VENV_PYTHON=%ROOT%..\.venv\Scripts\python.exe"
set "PYTHONUTF8=1"

if defined BYUL_PYTHON goto validate_byul_python
if exist "%VENV_PYTHON%" goto validate_venv
goto try_py

:validate_byul_python
"%BYUL_PYTHON%" -c "import sys; raise SystemExit(0 if sys.version_info >= (3, 10) else 1)" >nul 2>&1
if errorlevel 1 goto invalid_byul_python
goto run_byul_python

:validate_venv
"%VENV_PYTHON%" -c "import sys; raise SystemExit(0 if sys.version_info >= (3, 10) else 1)" >nul 2>&1
if not errorlevel 1 goto run_venv
echo WARNING: Ignoring broken repository Python environment: "%VENV_PYTHON%" 1>&2

:try_py
py -3 -c "import sys; raise SystemExit(0 if sys.version_info >= (3, 10) else 1)" >nul 2>&1
if not errorlevel 1 goto run_py

python -c "import sys; raise SystemExit(0 if sys.version_info >= (3, 10) else 1)" >nul 2>&1
if not errorlevel 1 goto run_python

echo [ERROR] Python 3.10 or newer was not found. Run tools\python\setup_env.bat first. 1>&2
exit /b 2

:invalid_byul_python
echo [ERROR] BYUL_PYTHON is not an executable Python 3.10 or newer: "%BYUL_PYTHON%" 1>&2
exit /b 2

:run_byul_python
"%BYUL_PYTHON%" "%ROOT%generate_wrapper_abi.py" %*
exit /b %ERRORLEVEL%

:run_venv
"%VENV_PYTHON%" "%ROOT%generate_wrapper_abi.py" %*
exit /b %ERRORLEVEL%

:run_py
py -3 "%ROOT%generate_wrapper_abi.py" %*
exit /b %ERRORLEVEL%

:run_python
python "%ROOT%generate_wrapper_abi.py" %*
exit /b %ERRORLEVEL%
