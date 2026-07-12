@echo off
setlocal

set "ROOT=%~dp0"
set "VENV_PYTHON=%ROOT%..\.venv\Scripts\python.exe"
set "DEFAULT_PYTHON=C:\Users\critl\AppData\Local\Programs\Python\Python313\python.exe"

if defined BYUL_PYTHON if exist "%BYUL_PYTHON%" (
    "%BYUL_PYTHON%" "%ROOT%run_tests.py" %*
    exit /b %ERRORLEVEL%
)

if exist "%VENV_PYTHON%" (
    "%VENV_PYTHON%" "%ROOT%run_tests.py" %*
    exit /b %ERRORLEVEL%
)

if exist "%DEFAULT_PYTHON%" (
    "%DEFAULT_PYTHON%" "%ROOT%run_tests.py" %*
    exit /b %ERRORLEVEL%
)

where py >nul 2>&1
if not errorlevel 1 (
    py -3 "%ROOT%run_tests.py" %*
    exit /b %ERRORLEVEL%
)

where python >nul 2>&1
if not errorlevel 1 (
    python "%ROOT%run_tests.py" %*
    exit /b %ERRORLEVEL%
)

echo [ERROR] Python 3 was not found. Run tools\python\setup_env.bat first. 1>&2
exit /b 1
