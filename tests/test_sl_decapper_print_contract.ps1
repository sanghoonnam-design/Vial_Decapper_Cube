$ErrorActionPreference = 'Stop'

$sourcePath = Join-Path $PSScriptRoot '..\App\Task\_10_XCommand_Module.c'
$source = Get-Content -LiteralPath $sourcePath -Raw

$start = $source.IndexOf('void CMD_Handle_Print_SL')
$end = $source.IndexOf('void CMD_Handle_Print_CD', $start)

if ($start -lt 0 -or $end -lt 0) {
    throw 'Could not isolate CMD_Handle_Print_SL().'
}

$function = $source.Substring($start, $end - $start)
$requiredOutput = @(
    '[ Common ]',
    'xSL.Time',
    'xSL.isBusy',
    'xSL.isError',
    'xSL.isEnable',
    'xSL.isHomed',
    'xSL.errorCode',
    'sizeof(tsXStateList)',
    '[ xSL.Decapper State ]',
    'xSL.Decapper.Cap_is',
    'xSL.Decapper.Body_is',
    'xSL.Decapper.Z2_HL_isError',
    'xSL.Decapper.Y_HL_isError',
    'xSL.Decapper.CT_Cap_Grip_isError',
    'xSL.Decapper.CT_Body_Middle_Grip_isError',
    'xSL.Decapper.CT_Body_Top_Grip_isError',
    '[ xSL.Decapper Sensor ]',
    'xSL.Decapper.CT_Cap_Grip_Open',
    'xSL.Decapper.CT_Cap_Grip_Close',
    'xSL.Decapper.CT_Body_Middle_Grip_Open',
    'xSL.Decapper.CT_Body_Middle_Grip_Close',
    'xSL.Decapper.CT_Body_Top_Grip_Open',
    'xSL.Decapper.CT_Body_Top_Grip_Close',
    'xSL.Decapper.Z2_H_Limit',
    'xSL.Decapper.Z2_L_Limit',
    'xSL.Decapper.Z_L_Limit',
    'xSL.Decapper.Y_H_Limit',
    'xSL.Decapper.Y_L_Limit',
    '[ xSL.Decapper Diagnosis ]',
    '[ Motor Run ]',
    'xSL.xZ_Motor_Run.Motor_Run',
    'xSL.xR_Motor_Run.Motor_Run'
)

foreach ($text in $requiredOutput) {
    if (-not $function.Contains($text)) {
        throw "CMD_Handle_Print_SL() is missing output for: $text"
    }
}

Write-Output 'PASS: CMD_Handle_Print_SL prints every Decapper state-list field.'
