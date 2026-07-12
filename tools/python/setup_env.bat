@echo off
setlocal

set "PYTHON_ROOT=%~dp0"
set "VENV=%PYTHON_ROOT%.venv"
set "DEFAULT_PYTHON=C:\Users\critl\AppData\Local\Programs\Python\Python313\python.exe"
pushd "%PYTHON_ROOT%"

if defined BYUL_PYTHON if exist "%BYUL_PYTHON%" (
    set BOOTSTRAP="%BYUL_PYTHON%"
    goto :create
)

if exist "%DEFAULT_PYTHON%" (
    set BOOTSTRAP="%DEFAULT_PYTHON%"
    goto :create
)

where py >nul 2>&1
if not errorlevel 1 (
    set "BOOTSTRAP=py -3"
    goto :create
)

where python >nul 2>&1
if not errorlevel 1 (
    set "BOOTSTRAP=python"
    goto :create
)

echo [ERROR] Python 3 was not found. Install Python 3.10 or newer. 1>&2
exit /b 1

:create
if not exist "%VENV%\Scripts\python.exe" (
    echo [INFO] Creating shared Python environment: "%VENV%"
    %BOOTSTRAP% -m venv "%VENV%"
    if errorlevel 1 exit /b %ERRORLEVEL%
)

"%VENV%\Scripts\python.exe" -m pip install --upgrade pip
if errorlevel 1 exit /b %ERRORLEVEL%

"%VENV%\Scripts\python.exe" -m pip install -r requirements-dev.txt -r requirements-package.txt
if errorlevel 1 exit /b %ERRORLEVEL%

echo [OK] Shared Python environment is ready: "%VENV%"
popd
exit /b 0
