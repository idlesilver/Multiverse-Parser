@echo off
setlocal enabledelayedexpansion

for /f %%a in ('powershell -Command "[int](Get-Date -UFormat %%s)"') do set START_TIME=%%a

cd /d "%~dp0"

set "EXT_DIR=%CD%\src\multiverse_parser\external"
if not exist "%EXT_DIR%" (
  set "BLENDER_DIR=%EXT_DIR%\blender\windows"
  mkdir "!BLENDER_DIR%!
  xcopy /e /i /y ext\blender\* "!BLENDER_DIR!\"

  set "USD_DIR=%EXT_DIR%\USD\windows"
  mkdir "!USD_DIR!"
  xcopy /e /i /y USD\windows "!USD_DIR!\windows"
)
@REM python -m build

set "end_time=%TIME%"

for /f %%a in ('powershell -Command "[int](Get-Date -UFormat %%s)"') do set END_TIME=%%a
set /a ELAPSED=%END_TIME% - %START_TIME%

echo Setup completed in %ELAPSED% seconds
