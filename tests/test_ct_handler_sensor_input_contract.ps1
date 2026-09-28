$ErrorActionPreference = 'Stop'

$root = Split-Path -Parent $PSScriptRoot
$header = Get-Content -Raw (Join-Path $root 'App\HW_Device\CT_Handler.h')
$source = Get-Content -Raw (Join-Path $root 'App\HW_Device\CT_Handler.c')

function Assert-Contains([string] $text, [string] $pattern, [string] $message) {
    if ($text -notmatch $pattern) { throw $message }
}

$expectedPins = @(
    @('CT_CAP_GRIP_OPEN_SENSOR_PIN', '0'),
    @('CT_CAP_GRIP_CLOSE_SENSOR_PIN', '1'),
    @('CT_BODY_MIDDLE_GRIP_OPEN_SENSOR_PIN', '2'),
    @('CT_BODY_MIDDLE_GRIP_CLOSE_SENSOR_PIN', '3'),
    @('CT_BODY_TOP_GRIP_OPEN_SENSOR_PIN', '4'),
    @('CT_BODY_TOP_GRIP_CLOSE_SENSOR_PIN', '5'),
    @('Z2_H_LIMIT_SENSOR_PIN', '6'),
    @('Z2_L_LIMIT_SENSOR_PIN', '7'),
    @('Z_L_LIMIT_SENSOR_PIN', '8'),
    @('Y_H_LIMIT_SENSOR_PIN', '9'),
    @('Y_L_LIMIT_SENSOR_PIN', '10')
)

foreach ($pin in $expectedPins) {
    Assert-Contains $header ("#define\s+" + $pin[0] + "\s+\(" + $pin[1] + "U\)") "Missing physical input pin definition: $($pin[0])"
}

$functionBody = [regex]::Match($source, '(?s)void\s+CT_Handler_Sensor_Update\s*\(void\)\s*\{.*?\n\}').Value
if ([string]::IsNullOrWhiteSpace($functionBody)) { throw 'CT_Handler_Sensor_Update was not found.' }

$expectedReads = @(
    @('CT_Cap_Grip_Open', 'CT_CAP_GRIP_OPEN_SENSOR_PIN'),
    @('CT_Cap_Grip_Close', 'CT_CAP_GRIP_CLOSE_SENSOR_PIN'),
    @('CT_Body_Middle_Grip_Open', 'CT_BODY_MIDDLE_GRIP_OPEN_SENSOR_PIN'),
    @('CT_Body_Middle_Grip_Close', 'CT_BODY_MIDDLE_GRIP_CLOSE_SENSOR_PIN'),
    @('CT_Body_Top_Grip_Open', 'CT_BODY_TOP_GRIP_OPEN_SENSOR_PIN'),
    @('CT_Body_Top_Grip_Close', 'CT_BODY_TOP_GRIP_CLOSE_SENSOR_PIN'),
    @('Z2_H_Limit', 'Z2_H_LIMIT_SENSOR_PIN'),
    @('Z2_L_Limit', 'Z2_L_LIMIT_SENSOR_PIN'),
    @('Z_L_Limit', 'Z_L_LIMIT_SENSOR_PIN'),
    @('Y_H_Limit', 'Y_H_LIMIT_SENSOR_PIN'),
    @('Y_L_Limit', 'Y_L_LIMIT_SENSOR_PIN')
)

foreach ($read in $expectedReads) {
    Assert-Contains $functionBody ("Decapper\." + $read[0] + ".*IOEXP_ReadIObit\(READ_IN,\s*" + $read[1] + "\)") "CT_Handler_Sensor_Update does not read $($read[0]) from $($read[1])."
}

if ($header -match 'xSLecapping_Sensor' -or $header -match 'CDecapping_Sensor') {
    throw 'Sensor state must be stored directly in tsXSL_Decapper.'
}

Write-Output 'CT Handler sensor input contract passed.'
