@echo off
setlocal enabledelayedexpansion

if not defined USD_SRC_DIR (
    echo USD_SRC_DIR is unset. >&2
    exit /b 1
)
if not defined USD_BUILD_DIR (
    echo USD_BUILD_DIR is unset. >&2
    exit /b 1
)

echo USD_SRC_DIR is set to: !USD_SRC_DIR!
echo USD_BUILD_DIR is set to: !USD_BUILD_DIR!

xcopy /E /I /Y "USD\plugin" "!USD_SRC_DIR!\plugin"

for /d %%G in ("!USD_SRC_DIR!\plugin\*") do (
    if exist "%%G\schema.usda" (
        pushd "%%G"
        call "!USD_BUILD_DIR!\bin\usdGenSchema.cmd" schema.usda
        popd
    )
)

set "USD_CMAKE_PATH=!USD_SRC_DIR!\CMakeLists.txt"
set "LINE_TO_ADD=add_subdirectory(plugin)"

findstr /x /c:"!LINE_TO_ADD!" "!USD_CMAKE_PATH!" >nul
if errorlevel 1 (
    echo.>>"!USD_CMAKE_PATH!"
    echo !LINE_TO_ADD!>>"!USD_CMAKE_PATH!"
)

for /f "usebackq delims=" %%i in (`"%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do (
    call "%%i\VC\Auxiliary\Build\vcvarsall.bat" x64
)

"!PYTHON_EXECUTABLE!" "!USD_SRC_DIR!\build_scripts\build_usd.py" "!USD_BUILD_DIR!" ^
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

endlocal
