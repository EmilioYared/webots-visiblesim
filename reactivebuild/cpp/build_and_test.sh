#!/usr/bin/env bash
# =============================================================================
# build_and_test.sh -- compile and run every reactivebuild/cpp/tests/test_*.cpp.
#
# Uses Webots' bundled MinGW g++ and the Eigen headers vendored in VisibleSim.
# Toolchain notes (from prior work in this repo):
#   * the MinGW bin MUST be on PATH or cc1plus dies silently,
#   * TMP/TEMP must be a writable Windows dir (we use the local .build).
# Run from anywhere:  bash reactivebuild/cpp/build_and_test.sh
# =============================================================================
set -u

WEBOTS_MINGW="${WEBOTS_MINGW:-/c/Program Files/Webots/msys64/mingw64/bin}"
GXX="$WEBOTS_MINGW/g++.exe"
[ -x "$GXX" ] || { echo "ERROR: g++ not found at $GXX (set WEBOTS_MINGW)"; exit 1; }
export PATH="$WEBOTS_MINGW:$PATH"

CPP="$(cd "$(dirname "$0")" && pwd)"           # reactivebuild/cpp
ROOT="$(cd "$CPP/../.." && pwd)"               # repo root
EIGEN="$ROOT/visible_sim/simulatorCore/src/deps"
[ -d "$EIGEN/Eigen" ] || { echo "ERROR: Eigen not found at $EIGEN/Eigen"; exit 1; }

BUILD="$CPP/.build"
mkdir -p "$BUILD"
export TMP="$BUILD" TEMP="$BUILD"

INC=(-I"$CPP/include" -I"$CPP/tests" -I"$EIGEN")
FLAGS=(-std=c++17 -O2 -w)

fail=0
shopt -s nullglob
for t in "$CPP"/tests/test_*.cpp; do
    name=$(basename "$t" .cpp)
    exe="$BUILD/$name.exe"
    printf 'compiling %-16s ... ' "$name"
    if ! "$GXX" "${FLAGS[@]}" "${INC[@]}" "$t" -o "$exe" 2> "$BUILD/$name.log"; then
        echo "COMPILE FAILED"; sed 's/^/    /' "$BUILD/$name.log"; fail=1; continue
    fi
    echo "ok"
    if ! "$exe"; then fail=1; fi
done

echo "----------------------------------------"
if [ $fail -eq 0 ]; then echo "ALL C++ TESTS PASSED"; else echo "SOME C++ TESTS FAILED"; fi
exit $fail
