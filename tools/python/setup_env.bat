@echo off
setlocal

set "PYTHON_ROOT=%~dp0"
set "VENV=%PYTHON_ROOT%.venv"
set "VENV_PYTHON=%VENV%\Scripts\python.exe"
set "APPROVED_WIN11_PYTHON=C:\Users\critl\AppData\Local\Programs\Python\Python313\python.exe"
set "PYTHONUTF8=1"
pushd "%PYTHON_ROOT%"

if defined BYUL_PYTHON goto validate_byul_python
if exist "%VENV_PYTHON%" goto validate_repository_venv
goto try_py

:validate_repository_venv
"%VENV_PYTHON%" -c "import sys; raise SystemExit(0 if sys.version_info >= (3, 10) else 1)" >nul 2>&1
if not errorlevel 1 goto install_dependencies
echo [INFO] Shared Python environment is broken and requires recreation: "%VENV%"
set "VENV_BROKEN=1"
goto try_py

:validate_byul_python
"%BYUL_PYTHON%" -c "import sys; raise SystemExit(0 if sys.version_info >= (3, 10) else 1)" >nul 2>&1
if errorlevel 1 goto invalid_byul_python
set "BOOTSTRAP_KIND=byul"
goto create_or_repair

:try_py
if exist "%APPROVED_WIN11_PYTHON%" goto validate_approved_win11_python
goto try_launcher

:validate_approved_win11_python
"%APPROVED_WIN11_PYTHON%" -c "import sys; raise SystemExit(0 if sys.version_info >= (3, 10) else 1)" >nul 2>&1
if errorlevel 1 goto invalid_approved_win11_python
set "BOOTSTRAP_KIND=approved_win11"
goto create_or_repair

:invalid_approved_win11_python
echo [WARNING] Approved Win11 Python is not executable or is older than 3.10: "%APPROVED_WIN11_PYTHON%" 1>&2

:try_launcher
py -3 -c "import sys; raise SystemExit(0 if sys.version_info >= (3, 10) else 1)" >nul 2>&1
if not errorlevel 1 goto use_py

python -c "import sys; raise SystemExit(0 if sys.version_info >= (3, 10) else 1)" >nul 2>&1
if not errorlevel 1 goto use_python

echo [ERROR] Python 3.10 or newer was not found. Set BYUL_PYTHON or install Python. 1>&2
popd
exit /b 2

:invalid_byul_python
echo [ERROR] BYUL_PYTHON is not an executable Python 3.10 or newer: "%BYUL_PYTHON%" 1>&2
popd
exit /b 2

:use_py
set "BOOTSTRAP_KIND=py"
goto create_or_repair

:use_python
set "BOOTSTRAP_KIND=python"
goto create_or_repair

:create_or_repair
if defined VENV_BROKEN goto recreate_venv
if exist "%VENV_PYTHON%" goto validate_existing_venv
echo [INFO] Creating shared Python environment: "%VENV%"
goto create_venv

:validate_existing_venv
"%VENV_PYTHON%" -c "import sys; raise SystemExit(0 if sys.version_info >= (3, 10) else 1)" >nul 2>&1
if not errorlevel 1 goto install_dependencies
echo [INFO] Recreating broken shared Python environment: "%VENV%"
goto recreate_venv

:create_venv
if "%BOOTSTRAP_KIND%"=="byul" goto create_with_byul
if "%BOOTSTRAP_KIND%"=="approved_win11" goto create_with_approved_win11
if "%BOOTSTRAP_KIND%"=="py" goto create_with_py
goto create_with_python

:recreate_venv
if "%BOOTSTRAP_KIND%"=="byul" goto recreate_with_byul
if "%BOOTSTRAP_KIND%"=="approved_win11" goto recreate_with_approved_win11
if "%BOOTSTRAP_KIND%"=="py" goto recreate_with_py
goto recreate_with_python

:create_with_byul
"%BYUL_PYTHON%" -m venv "%VENV%"
set "RESULT=%ERRORLEVEL%"
goto check_bootstrap

:create_with_approved_win11
"%APPROVED_WIN11_PYTHON%" -m venv "%VENV%"
set "RESULT=%ERRORLEVEL%"
goto check_bootstrap

:create_with_py
py -3 -m venv "%VENV%"
set "RESULT=%ERRORLEVEL%"
goto check_bootstrap

:create_with_python
python -m venv "%VENV%"
set "RESULT=%ERRORLEVEL%"
goto check_bootstrap

:recreate_with_byul
"%BYUL_PYTHON%" -m venv --clear "%VENV%"
set "RESULT=%ERRORLEVEL%"
goto check_bootstrap

:recreate_with_approved_win11
"%APPROVED_WIN11_PYTHON%" -m venv --clear "%VENV%"
set "RESULT=%ERRORLEVEL%"
goto check_bootstrap

:recreate_with_py
py -3 -m venv --clear "%VENV%"
set "RESULT=%ERRORLEVEL%"
goto check_bootstrap

:recreate_with_python
python -m venv --clear "%VENV%"
set "RESULT=%ERRORLEVEL%"

:check_bootstrap
if not "%RESULT%"=="0" goto bootstrap_failed

:install_dependencies
"%VENV_PYTHON%" -m pip install --upgrade pip
set "RESULT=%ERRORLEVEL%"
if not "%RESULT%"=="0" goto install_failed

"%VENV_PYTHON%" -m pip install -r requirements-dev.txt -r requirements-package.txt
set "RESULT=%ERRORLEVEL%"
if not "%RESULT%"=="0" goto install_failed

echo [OK] Shared Python environment is ready: "%VENV%"
popd
exit /b 0

:bootstrap_failed
echo [ERROR] Failed to create the shared Python environment. 1>&2
popd
exit /b %RESULT%

:install_failed
echo [ERROR] Failed to install shared Python dependencies. 1>&2
popd
exit /b %RESULT%
