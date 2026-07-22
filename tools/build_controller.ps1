# =============================================================================
# build_controller.ps1 — PowerShell wrapper for tools/build_controller.sh
#
# Rebuilds the Webots controller binary from source (Webots does not always do
# this on world reload, leaving stale controller code running). Run after
# editing controller source, then reload the world in Webots.
#
#   .\tools\build_controller.ps1
# =============================================================================
$ErrorActionPreference = "Stop"

$candidates = @(
    "C:\Program Files\Git\bin\bash.exe",
    "C:\Program Files\Git\usr\bin\bash.exe",
    "${env:ProgramFiles}\Git\bin\bash.exe",
    "${env:LOCALAPPDATA}\Programs\Git\bin\bash.exe"
)
$bash = $candidates | Where-Object { Test-Path $_ } | Select-Object -First 1
if (-not $bash) {
    Write-Error "Git Bash not found. Install Git for Windows, or run tools/build_controller.sh from a Git Bash terminal."
    exit 1
}

$root = Split-Path -Parent $PSScriptRoot
Push-Location $root
try {
    & $bash "tools/build_controller.sh" @args
    exit $LASTEXITCODE
} finally {
    Pop-Location
}
