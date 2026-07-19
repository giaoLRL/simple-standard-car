$ErrorActionPreference = "Stop"

$projectDir = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$buildDir = Join-Path $projectDir "Debug"
$objDir = Join-Path $buildDir "obj"
$syscfgDir = Join-Path $buildDir "syscfg"
$sdkDir = "D:\ti\ccs2051\mspm0_sdk_2_10_00_04"
$syscfgCli = "D:\ti\ccs2051\sysconfig_1.26.2\sysconfig_cli.bat"
$compilerDir = "D:\ti\ccs2051\ccs\tools\compiler\ti-cgt-armllvm_4.0.4.LTS"
$compiler = Join-Path $compilerDir "bin\tiarmclang.exe"

foreach ($path in @($sdkDir, $syscfgCli, $compiler)) {
    if (-not (Test-Path -LiteralPath $path)) {
        throw "Required build dependency not found: $path"
    }
}

New-Item -ItemType Directory -Force -Path $objDir, $syscfgDir | Out-Null

& $syscfgCli `
    -s (Join-Path $sdkDir ".metadata\product.json") `
    --script (Join-Path $projectDir "empty_mspm0g3507.syscfg") `
    -o $syscfgDir `
    --compiler ticlang
if ($LASTEXITCODE -ne 0) {
    throw "SysConfig generation failed with exit code $LASTEXITCODE"
}

$includeArgs = @(
    "-I$projectDir",
    "-I$syscfgDir",
    "-I$(Join-Path $sdkDir 'source')",
    "-I$(Join-Path $sdkDir 'source\third_party\CMSIS\Core\Include')"
)
$compileArgs = @(
    "-c",
    "-march=thumbv6m",
    "-mcpu=cortex-m0plus",
    "-mfloat-abi=soft",
    "-mlittle-endian",
    "-mthumb",
    "-O2",
    "-gdwarf-3",
    "-D__MSPM0G3507__",
    "-D__USE_SYSCONFIG__"
) + $includeArgs

$sources = @(
    @{ Source = "main.cpp"; Object = "main.o" },
    @{ Source = "modules\common\buzzer.cpp"; Object = "buzzer.o" },
    @{ Source = "modules\common\timebase.cpp"; Object = "timebase.o" },
    @{ Source = "modules\common\uart_debug.cpp"; Object = "uart_debug.o" },
    @{ Source = "modules\common\uart_protocol.cpp"; Object = "uart_protocol.o" },
    @{ Source = "modules\common\config.cpp"; Object = "config.o" },
    @{ Source = "modules\control\turn_state_machine.cpp"; Object = "turn_state_machine.o" },
    @{ Source = "modules\encoder\encoder.cpp"; Object = "encoder.o" },
    @{ Source = "modules\line_sensor\line_sensor.cpp"; Object = "line_sensor.o" },
    @{ Source = "modules\motor\motor.cpp"; Object = "motor.o" },
    @{ Source = (Join-Path $syscfgDir "ti_msp_dl_config.c"); Object = "ti_msp_dl_config.o" },
    @{
        Source = (Join-Path $sdkDir "source\ti\devices\msp\m0p\startup_system_files\ticlang\startup_mspm0g350x_ticlang.c")
        Object = "startup_mspm0g350x_ticlang.o"
    }
)

$objects = foreach ($item in $sources) {
    $source = $item.Source
    if (-not [System.IO.Path]::IsPathRooted($source)) {
        $source = Join-Path $projectDir $source
    }
    $object = Join-Path $objDir $item.Object
    & $compiler @compileArgs -o $object $source
    if ($LASTEXITCODE -ne 0) {
        throw "Compilation failed: $source"
    }
    $object
}

$output = Join-Path $buildDir "line_follower.out"
$map = Join-Path $buildDir "line_follower.map"
$linkArgs = @(
    "-march=thumbv6m",
    "-mcpu=cortex-m0plus",
    "-mfloat-abi=soft",
    "-mlittle-endian",
    "-mthumb",
    "-O2",
    "-gdwarf-3",
    "-Wl,-m$map",
    "-Wl,-i$(Join-Path $sdkDir 'source')",
    "-Wl,-i$syscfgDir",
    "-Wl,-i$(Join-Path $compilerDir 'lib')",
    "-Wl,--diag_wrap=off",
    "-Wl,--display_error_number",
    "-Wl,--warn_sections",
    "-Wl,--rom_model",
    "-o",
    $output
) + $objects + @(
    "-Wl,-ldevice_linker.cmd",
    "-Wl,-ldevice.cmd.genlibs",
    "-Wl,-llibc.a"
)

& $compiler @linkArgs
if ($LASTEXITCODE -ne 0) {
    throw "Link failed with exit code $LASTEXITCODE"
}

Write-Host "Build completed: $output"
