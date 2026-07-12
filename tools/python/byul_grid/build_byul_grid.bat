@echo off
setlocal

set "DEFAULT_PYTHON=C:\Users\critl\AppData\Local\Programs\Python\Python313\python.exe"

if defined BYUL_PYTHON if exist "%BYUL_PYTHON%" (
    "%BYUL_PYTHON%" "%~dp0build_byul_grid.py" %*
    endlocal & exit /b %ERRORLEVEL%
)

if exist "%DEFAULT_PYTHON%" (
    "%DEFAULT_PYTHON%" "%~dp0build_byul_grid.py" %*
    endlocal & exit /b %ERRORLEVEL%
)

where py >nul 2>&1
if errorlevel 1 goto use_python
py -3 "%~dp0build_byul_grid.py" %*
endlocal & exit /b %ERRORLEVEL%

:use_python
where python >nul 2>&1
if errorlevel 1 (
    echo [ERROR] Python 3 was not found. 1>&2
    exit /b 1
)
python "%~dp0build_byul_grid.py" %*
endlocal & exit /b %ERRORLEVEL%
