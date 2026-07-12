@echo off
setlocal

set "DEFAULT_PYTHON=C:\Users\critl\AppData\Local\Programs\Python\Python313\python.exe"

if defined BYUL_PYTHON if exist "%BYUL_PYTHON%" (
    "%BYUL_PYTHON%" "%~dp0generate_wrapper_abi.py" %*
    exit /b %ERRORLEVEL%
)

if exist "%DEFAULT_PYTHON%" (
    "%DEFAULT_PYTHON%" "%~dp0generate_wrapper_abi.py" %*
    exit /b %ERRORLEVEL%
)

where py >nul 2>&1
if not errorlevel 1 (
    py -3 "%~dp0generate_wrapper_abi.py" %*
    exit /b %ERRORLEVEL%
)

python "%~dp0generate_wrapper_abi.py" %*
exit /b %ERRORLEVEL%
