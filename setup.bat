@echo off
setlocal enabledelayedexpansion

for /f %%a in ('powershell -Command "[int](Get-Date -UFormat %%s)"') do set START_TIME=%%a

cd /d "%~dp0"

set "USD_ENABLED=false"

:parse_args
if "%~1"=="" goto after_parse
if "%~1"=="--usd" (
  set "USD_ENABLED=true"
  ) else (
  echo Unknown option: %~1
  exit /b 1
)
shift
goto parse_args
:after_parse

set "EXT_DIR=%CD%\ext"
set "ZIP_DIR=%EXT_DIR%\7zip"
set "ZIP_EXECUTABLE=%ZIP_DIR%\7za.exe"
if not exist "%ZIP_DIR%" (
  mkdir "%ZIP_DIR%"
  set "ZIP_FILE=7z2409-extra.7z"
  powershell -NoProfile -Command "C:\Windows\System32\curl.exe --ssl-no-revoke -L -o %EXT_DIR%\7zr.exe https://www.7-zip.org/a/7zr.exe"
  powershell -NoProfile -Command "C:\Windows\System32\curl.exe --ssl-no-revoke -L -o %EXT_DIR%\!ZIP_FILE! https://www.7-zip.org/a/!ZIP_FILE!"
  powershell -NoProfile -Command "%EXT_DIR%\7zr.exe x '%EXT_DIR%\!ZIP_FILE!' -o'%ZIP_DIR%'"
  del "%EXT_DIR%\!ZIP_FILE!"
  del "%EXT_DIR%\7zr.exe"
)

set "BLENDER_DIR=%EXT_DIR%\blender"
if not exist "%BLENDER_DIR%" (
  set "BLENDER_VERSION=4.5"
  set "BLENDER=blender-!BLENDER_VERSION!.1-windows-x64"
  set "BLENDER_ZIP_FILE=!BLENDER!.zip"
  powershell -NoProfile -Command "C:\Windows\System32\curl.exe --ssl-no-revoke -L -o '%EXT_DIR%\!BLENDER_ZIP_FILE!' https://download.blender.org/release/Blender!BLENDER_VERSION!/!BLENDER_ZIP_FILE!"
  powershell -NoProfile -Command "%ZIP_EXECUTABLE% x '%EXT_DIR%\!BLENDER_ZIP_FILE!' -o'%EXT_DIR%'"
  move "%EXT_DIR%\!BLENDER!" "%EXT_DIR%\blender"
  del "%EXT_DIR%\!BLENDER_ZIP_FILE!"
  
  %BLENDER_DIR%\!BLENDER_VERSION!\python\bin\python.exe -m pip install --upgrade pip build --no-warn-script-location
  %BLENDER_DIR%\!BLENDER_VERSION!\python\bin\python.exe -m pip install bpy Pillow --no-warn-script-location
)

if /i "%USD_ENABLED%"=="true" (
  set "USD_SRC_DIR=%EXT_DIR%\OpenUSD"
  if not exist "!USD_SRC_DIR!" (
    git clone https://github.com/PixarAnimationStudios/OpenUSD.git --depth 1 --branch v25.02 "!USD_SRC_DIR!"
  )
  
  set "BUILD_DIR=%CD%\build"
  set "USD_BUILD_DIR=!BUILD_DIR!\USD"
  
  if not exist "!USD_BUILD_DIR!" (
    set "PYTHON_EXECUTABLE=python.exe"

    mkdir "!USD_BUILD_DIR!"
    
    for /f "usebackq delims=" %%i in (`"%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do (
      call "%%i\VC\Auxiliary\Build\vcvarsall.bat" x64
    )
    !PYTHON_EXECUTABLE! "!USD_SRC_DIR!\build_scripts\build_usd.py" "!USD_BUILD_DIR!" ^
    --no-tests ^
    --no-examples ^
    --no-tutorials ^
    --no-tools ^
    --no-docs ^
    --no-python-docs ^
    --python ^
    --prefer-speed-over-safety ^
    --no-debug-python ^
    --no-imaging ^
    --no-ptex ^
    --no-openvdb ^
    --no-usdview ^
    --no-embree ^
    --no-openimageio ^
    --no-opencolorio ^
    --no-alembic ^
    --no-hdf5 ^
    --no-draco ^
    --no-materialx ^
    --no-onetbb ^
    --no-usdValidation ^
    --no-mayapy-tests ^
    --no-animx-tests
    
    set "PATH=%PATH%;!USD_BUILD_DIR!\bin;!USD_BUILD_DIR!\lib"
    set "PYTHONPATH=!USD_BUILD_DIR!\lib\python"
    call install.bat
    
    set "USD_BIN_DIR=%CD%\USD\windows"
    if not exist "!USD_BIN_DIR!" (
      mkdir "!USD_BIN_DIR!"
    )
    xcopy /E /I /Y "!USD_BUILD_DIR!\bin\*" "!USD_BIN_DIR!\bin\"
    xcopy /E /I /Y "!USD_BUILD_DIR!\lib\*" "!USD_BIN_DIR!\lib\"
    xcopy /E /I /Y "!USD_BUILD_DIR!\plugin\*" "!USD_BIN_DIR!\plugin\"
  )
)

set "end_time=%TIME%"

for /f %%a in ('powershell -Command "[int](Get-Date -UFormat %%s)"') do set END_TIME=%%a
set /a ELAPSED=%END_TIME% - %START_TIME%

echo Setup completed in %ELAPSED% seconds
