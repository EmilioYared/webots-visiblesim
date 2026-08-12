# =============================================================================
# build_and_test.ps1 -- PowerShell wrapper for reactivebuild/cpp/build_and_test.sh
#
# PowerShell can't run .sh directly; this invokes it via Git Bash.
#   .\reactivebuild\cpp\build_and_test.ps1
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
    Write-Error "Git Bash not found. Install Git for Windows, or run reactivebuild/cpp/build_and_test.sh from a Git Bash terminal."
    exit 1
}

& $bash "reactivebuild/cpp/build_and_test.sh" @args
exit $LASTEXITCODE
