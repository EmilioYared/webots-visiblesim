#!/usr/bin/env bash
# =============================================================================
# build_controller.sh — compile the Webots controller into a fresh binary.
#
# Webots is supposed to rebuild the controller when you load a world, but it
# does not always do so — a stale controllers/catom3d_controller.exe then keeps
# running the OLD code even though the sources changed (symptom: your controller
# edits have no effect, e.g. the physics still "pops"). Run this after editing
# any controller source, then reload the world in Webots.
#
# It links WITHOUT the -s strip step (which needs a writable temp dir the
# sandbox denies); the resulting binary is larger but behaves identically.
# =============================================================================
set -u

WEBOTS_HOME="${WEBOTS_HOME:-C:/Program Files/Webots}"
WEBOTS_MINGW="${WEBOTS_MINGW:-/c/Program Files/Webots/msys64/mingw64/bin}"
GXX="$WEBOTS_MINGW/g++.exe"
[ -x "$GXX" ] || { echo "ERROR: g++ not found at $GXX"; exit 1; }
[ -d "$WEBOTS_HOME/lib/controller" ] || { echo "ERROR: Webots libs not at $WEBOTS_HOME/lib/controller (set WEBOTS_HOME)"; exit 1; }

export PATH="$WEBOTS_MINGW:$PATH"
cd "$(dirname "$0")/.."                        # project root
mkdir -p tools/.build
export TMP="$(pwd -W 2>/dev/null || pwd)/tools/.build" TEMP="$TMP"

CTRL=controllers/catom3d_controller
# Arrays keep the "Program Files" space intact through word-splitting.
FLAGS=(-std=c++17 -O1 -pipe -w)
INC=(-I"$WEBOTS_HOME/include/controller/cpp" -I"$WEBOTS_HOME/include/controller/c")
LIBS=(-L"$WEBOTS_HOME/lib/controller" -lCppController -lController)

echo "compiling catom3d_controller.cpp ..."
if ! "$GXX" "${FLAGS[@]}" "${INC[@]}" -c "$CTRL/catom3d_controller.cpp" -o tools/.build/catom3d_controller.o; then
    echo "COMPILE FAILED"; exit 1
fi
echo "linking catom3d_controller.exe ..."
if ! "$GXX" -pipe tools/.build/catom3d_controller.o -o "$CTRL/catom3d_controller.exe" "${LIBS[@]}"; then
    echo "LINK FAILED"; exit 1
fi

ls -la --time-style=+"%Y-%m-%d %H:%M" "$CTRL/catom3d_controller.exe"
echo "OK — controller rebuilt. Reload the world in Webots to run it."
