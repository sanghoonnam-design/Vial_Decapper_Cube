$ErrorActionPreference = 'Stop'

$repoRoot = Split-Path -Parent $PSScriptRoot

if (-not (Test-Path (Join-Path $repoRoot 'App/HW_Device/CT_Handler.c'))) {
    throw 'CT_Handler.c must exist.'
}

if (-not (Test-Path (Join-Path $repoRoot 'App/HW_Device/CT_Handler.h'))) {
    throw 'CT_Handler.h must exist.'
}

if (Test-Path (Join-Path $repoRoot 'App/HW_Device/CDecap.c')) {
    throw 'Legacy CDecap.c must not remain.'
}

if (Test-Path (Join-Path $repoRoot 'App/HW_Device/CDecap.h')) {
    throw 'Legacy CDecap.h must not remain.'
}

$references = rg -l '^\s*#include\s*[<"]CDecap\.(h|c)[>"]' $repoRoot/App $repoRoot/tests
if ($LASTEXITCODE -eq 0 -and $references) {
    throw "Legacy CDecap include remains: $($references -join ', ')"
}

Write-Host 'CT Handler file rename contract passed.'
