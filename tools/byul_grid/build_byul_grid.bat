@echo off
setlocal

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
