# =============================================================================
# apply_scene.ps1 — PowerShell wrapper for tools/apply_scene.sh
#
# PowerShell cannot run .sh scripts directly (typing ./x.sh makes Windows ask
# which app to open the file with). This finds Git Bash and runs the real script
# through it, forwarding any arguments.
#
#   .\tools\apply_scene.ps1                     # regenerate the walk world
#   .\tools\apply_scene.ps1 scene_stress.json   # regenerate the stress world
#
# NOTE: use Git Bash, NOT the Windows Store / WSL bash (C:\Windows\System32\
# bash.exe) — the scripts rely on Git Bash's /c/... path convention.
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
    Write-Error "Git Bash not found. Install Git for Windows, or run the script from a Git Bash terminal."
    exit 1
}

# Run from the project root (this script lives in tools/).
$root = Split-Path -Parent $PSScriptRoot
Push-Location $root
try {
    & $bash "tools/apply_scene.sh" @args
    exit $LASTEXITCODE
} finally {
    Pop-Location
}
