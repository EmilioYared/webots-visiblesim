#!/usr/bin/env bash
# =============================================================================
# build_and_run_app.sh -- compile and run one reactivebuild/cpp/apps/*.cpp.
# Same toolchain setup as build_and_test.sh (Webots MinGW g++ + vendored Eigen).
#
#   bash reactivebuild/cpp/build_and_run_app.sh rb_tower [args...]
# e.g.
#   bash reactivebuild/cpp/build_and_run_app.sh rb_tower 100 10 3 5 0
# =============================================================================
set -u

APP="${1:-rb_tower}"; shift || true

WEBOTS_MINGW="${WEBOTS_MINGW:-/c/Program Files/Webots/msys64/mingw64/bin}"
GXX="$WEBOTS_MINGW/g++.exe"
[ -x "$GXX" ] || { echo "ERROR: g++ not found at $GXX (set WEBOTS_MINGW)"; exit 1; }
export PATH="$WEBOTS_MINGW:$PATH"

CPP="$(cd "$(dirname "$0")" && pwd)"           # reactivebuild/cpp
ROOT="$(cd "$CPP/../.." && pwd)"               # repo root
EIGEN="$ROOT/visible_sim/simulatorCore/src/deps"
[ -d "$EIGEN/Eigen" ] || { echo "ERROR: Eigen not found at $EIGEN/Eigen"; exit 1; }

SRC="$CPP/apps/$APP.cpp"
[ -f "$SRC" ] || { echo "ERROR: no app at $SRC"; exit 1; }

BUILD="$CPP/.build"
mkdir -p "$BUILD"
export TMP="$BUILD" TEMP="$BUILD"
mkdir -p "$ROOT/reactivebuild/results"

EXE="$BUILD/$APP.exe"
echo "compiling $APP ..."
if ! "$GXX" -std=c++17 -O2 -w -I"$CPP/include" -I"$EIGEN" "$SRC" -o "$EXE" 2> "$BUILD/$APP.log"; then
    echo "COMPILE FAILED"; sed 's/^/    /' "$BUILD/$APP.log"; exit 1
fi

# Run from the repo root so the default "reactivebuild/results" out_dir resolves.
cd "$ROOT"
"$EXE" "$@"
