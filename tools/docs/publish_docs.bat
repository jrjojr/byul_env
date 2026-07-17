@echo off
setlocal

set "SCRIPT_DIR=%~dp0"
set "REPOSITORY_ROOT=%SCRIPT_DIR%..\.."

if defined BYUL_PYTHON goto check_byul_python

set "PYTHON=%REPOSITORY_ROOT%\tools\python\.venv\Scripts\python.exe"
if not exist "%PYTHON%" goto check_py_launcher
"%PYTHON%" --version >nul 2>nul
if not errorlevel 1 goto run_python

:check_py_launcher
where py >nul 2>nul
if errorlevel 1 goto check_python
py -3 --version >nul 2>nul
if not errorlevel 1 goto run_py_launcher

:check_python
where python >nul 2>nul
if errorlevel 1 goto no_python
python --version >nul 2>nul
if errorlevel 1 goto no_python
set "PYTHON=python"
goto run_python

:no_python
echo No Python 3 interpreter was found. Set BYUL_PYTHON. 1>&2
exit /b 127

:check_byul_python
"%BYUL_PYTHON%" --version >nul 2>nul
if not errorlevel 1 goto run_byul_python
echo BYUL_PYTHON is not an executable Python interpreter: %BYUL_PYTHON% 1>&2
exit /b 127

:run_byul_python
"%BYUL_PYTHON%" "%SCRIPT_DIR%publish_docs.py" %*
exit /b %ERRORLEVEL%

:run_py_launcher
py -3 "%SCRIPT_DIR%publish_docs.py" %*
exit /b %ERRORLEVEL%

:run_python
"%PYTHON%" "%SCRIPT_DIR%publish_docs.py" %*
exit /b %ERRORLEVEL%
