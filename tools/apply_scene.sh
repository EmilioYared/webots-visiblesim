#!/usr/bin/env bash
# =============================================================================
# apply_scene.sh — apply scene.json to the generated Webots world.
#
# Run this after editing scene.json. It regenerates the proto + world from the
# current config and re-verifies collisions and traffic spacing, so physical
# changes (mass, bounding radius, contacts, floor, bonds, gravity, ...) take
# effect. Then reload the world in Webots to pick them up.
#
# NOT needed for runtime-only fields (playback.*, analysis.*): those are read
# live by the controller, so a plain Reload World suffices.
#
# To change algorithm.movingCatomRadius's actual MOTION (not just its verified
# value), edit visible_sim/.../config.xml too and regenerate trace.csv in
# VisibleSim — this script only re-checks the trace on disk.
# =============================================================================
set -u

WEBOTS_MINGW="${WEBOTS_MINGW:-/c/Program Files/Webots/msys64/mingw64/bin}"
GXX="$WEBOTS_MINGW/g++.exe"
[ -x "$GXX" ] || { echo "ERROR: g++ not found at $GXX (set WEBOTS_MINGW)"; exit 1; }

export PATH="$WEBOTS_MINGW:$PATH"
cd "$(dirname "$0")/.."            # project root
mkdir -p tools/.build
export TMP="$(pwd -W 2>/dev/null || pwd)/tools/.build" TEMP="$TMP"
FLAGS="-std=c++17 -O1 -pipe -w"

build() {  # build <name> from tools/<name>.cpp
    printf 'building %-16s ' "$1"
    if "$GXX" $FLAGS -o "tools/$1.exe" "tools/$1.cpp" 2> "tools/.build/$1.log"; then
        echo ok
    else
        echo BUILD FAILED; cat "tools/.build/$1.log"; exit 1
    fi
}

# Optional config argument (default the walk scene). Pass scene_stress.json to
# regenerate the cantilever stress world instead.
CONFIG="${1:-scene.json}"
[ -f "$CONFIG" ] || { echo "ERROR: config '$CONFIG' not found"; exit 1; }

build gen_world
build check_collisions

echo
echo "--- regenerating world/proto from $CONFIG ---"
./tools/gen_world.exe "$CONFIG" || { echo "gen_world FAILED"; exit 1; }

# The collision + spacing checks replay the walk trace, so they only apply to the
# walk scenario. Stress scenarios report their structure at generation instead.
if grep -qE '"type"[[:space:]]*:[[:space:]]*"cantilever"' "$CONFIG"; then
    echo
    echo "OK — stress world regenerated (see structure above)."
else
    echo
    echo "--- verifying collisions + traffic spacing ---"
    ./tools/check_collisions.exe controllers/catom3d_controller/trace_resolved.csv \
        | grep -E "worst pen|movingCatomRadius|along-track gap|spacing RESULT|RESULT"
    if ! ./tools/check_collisions.exe controllers/catom3d_controller/trace_resolved.csv > /dev/null 2>&1; then
        echo
        echo "WARNING: collision or spacing check FAILED for $CONFIG."
        echo "         The world was still regenerated; review the numbers above."
        exit 1
    fi
fi

echo
echo "OK — now Reload the World in Webots (Ctrl+Shift+R) to see the changes."
