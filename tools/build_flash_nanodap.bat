@echo off
setlocal

set "PROJECT_DIR=%~dp0.."
set "OPENOCD=D:\ti\xpack-openocd-0.12.0-7\bin\openocd.exe"
set "OPENOCD_SCRIPTS=D:\ti\xpack-openocd-0.12.0-7\openocd\scripts"
set "ELF=%PROJECT_DIR:\=/%/Debug/line_follower.out"
set "PROGRAM_COMMAND=program %ELF% verify reset exit"
if /I "%~1"=="/halt" set "PROGRAM_COMMAND=program %ELF% verify exit"

echo [1/2] Building project...
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0build_project.ps1"
if errorlevel 1 (
    echo Build failed. Flash was not changed.
    exit /b 1
)

echo [2/2] Programming with nanoDAP...
"%OPENOCD%" ^
    -s "%OPENOCD_SCRIPTS%" ^
    -f interface/cmsis-dap.cfg ^
    -c "cmsis-dap backend hid" ^
    -c "cmsis-dap quirk enable" ^
    -c "transport select swd" ^
    -c "adapter speed 50" ^
    -f target/ti_mspm0.cfg ^
    -c "%PROGRAM_COMMAND%"

if errorlevel 1 (
    echo Programming failed.
    echo Check target power, GND, 3.3V/VTref, SWDIO, SWCLK, and nRST.
    echo Make sure both wireless blue LEDs are solid before retrying.
    exit /b 1
)

echo Programming completed successfully.
exit /b 0
