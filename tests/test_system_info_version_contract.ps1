$ErrorActionPreference = 'Stop'

$repoRoot = Split-Path -Parent $PSScriptRoot
$moduleDef = Get-Content -Raw (Join-Path $repoRoot 'App/System/XSystemInfo_ModuleDef.h')
$systemInfoHeader = Get-Content -Raw (Join-Path $repoRoot 'App/System/XSystemInfo.h')
$versionDef = Get-Content -Raw (Join-Path $repoRoot 'App/Version.h')
$systemInfo = Get-Content -Raw (Join-Path $repoRoot 'App/System/XSystemInfo.c')

function Require-Match([string]$Text, [string]$Pattern, [string]$Message) {
    if ($Text -notmatch $Pattern) {
        throw $Message
    }
}

Require-Match $moduleDef '#define\s+SYSTEM_TYPE[^\r\n]*SYS_HEM_CT_Handler' 'Default system type must be SYS_HEM_CT_Handler.'
Require-Match $moduleDef '#define\s+MODEL_TYPE[^\r\n]*MODEL_TUBE' 'Default model must be MODEL_TUBE.'
Require-Match $moduleDef '#define\s+SYS_HEM_CT_Handler[^\r\n]*\(1\)' 'HEM CT Handler system identifier is missing.'
Require-Match $moduleDef '#define\s+MODEL_TUBE[^\r\n]*\(1\)' 'MODEL_TUBE must have numeric value 1.'
Require-Match $moduleDef '#define\s+MODEL_50ML[^\r\n]*\(50\)' 'MODEL_50ML must have numeric value 50.'
Require-Match $systemInfo 'case\s+MODEL_TUBE\s*:[\s\S]*?return\s+"TUBE"' 'Tube model must be displayed as TUBE.'
Require-Match $systemInfo 'case\s+MODEL_50ML\s*:[\s\S]*?return\s+"50mL"' '50 mL model must be displayed as 50mL.'
Require-Match $moduleDef '#define\s+SYSTEM_TYPE_STR\s+.*"HEM_CT_Handler"' 'System type string must be HEM_CT_Handler.'
Require-Match $systemInfoHeader '#define\s+MODEL_NAME_STR\s+\("HEM_CT_Handler"\)' 'Model name string must be HEM_CT_Handler.'
Require-Match $systemInfoHeader 'char\s+cd_FWVersion_str\[20\]' 'Firmware version text buffer must hold the longest formatted version.'
Require-Match $versionDef '#define\s+FW_VERSION[^\r\n]*\(10000\)' 'Initial firmware version must be 1.0.0.'
Require-Match $systemInfo '"%d\.%d\.%dA%02u"' 'Version formatter must produce an A-prefixed, two-digit model suffix.'

foreach ($model in 1, 50) {
    $formatted = '1.0.0A{0:D2}' -f $model
    $expected = switch ($model) {
        1  { '1.0.0A01' }
        50 { '1.0.0A50' }
    }
    if ($formatted -ne $expected) {
        throw "Unexpected formatted version for model ${model}: $formatted"
    }
}

Write-Host 'System info version contract passed.'
