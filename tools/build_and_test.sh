#!/usr/bin/env bash
# =============================================================================
# Builds and runs the offline (non-Webots) tools and tests.
#
#   ./tools/build_and_test.sh            build + run everything
#   ./tools/build_and_test.sh test_json  build + run one target
#
# Uses the MinGW toolchain bundled with Webots. Two environment quirks matter:
#   * the toolchain's own bin/ must be on PATH or cc1plus fails to load its DLLs
#     and exits silently with no diagnostics;
#   * TMP/TEMP must point at a writable Windows-style path.
# =============================================================================
set -u

WEBOTS_MINGW="${WEBOTS_MINGW:-/c/Program Files/Webots/msys64/mingw64/bin}"
GXX="$WEBOTS_MINGW/g++.exe"
[ -x "$GXX" ] || { echo "ERROR: g++ not found at $GXX (set WEBOTS_MINGW)"; exit 1; }

export PATH="$WEBOTS_MINGW:$PATH"
TMPDIR_WIN="${TMPDIR_WIN:-$(cd "$(dirname "$0")/.." && pwd -W 2>/dev/null || pwd)/tools/.build}"
mkdir -p "$TMPDIR_WIN"
export TMP="$TMPDIR_WIN" TEMP="$TMPDIR_WIN"

cd "$(dirname "$0")/.."          # project root
FLAGS="-std=c++17 -O1 -pipe -w"

TARGETS=(test_json test_scene_config test_scenario gen_world test_gen_world)
[ $# -gt 0 ] && TARGETS=("$@")

# Resolve link selection FIRST (before gen_world consumes the resolved trace):
# apply VisibleSim's blocking rule + a clearance tiebreak offline and bake the
# chosen arc into trace_resolved.csv, so the runtime player is collision-free.
if [ $# -eq 0 ]; then
    printf 'building %-18s ' "check_collisions"
    if "$GXX" $FLAGS -o tools/check_collisions.exe \
            tools/check_collisions.cpp 2> tools/.build/check_collisions.log; then
        echo "ok"
        ./tools/check_collisions.exe --emit controllers/catom3d_controller/trace_resolved.csv \
            > tools/.build/resolve.log 2>&1 && grep -E "emitted" tools/.build/resolve.log
    else
        echo "BUILD FAILED"; cat tools/.build/check_collisions.log; exit 1
    fi
fi

fail=0
for t in "${TARGETS[@]}"; do
    src="tools/$t.cpp"
    [ -f "$src" ] || { echo "SKIP $t (no $src)"; continue; }
    printf 'building %-18s ' "$t"
    if ! "$GXX" $FLAGS -o "tools/$t.exe" "$src" 2> "tools/.build/$t.log"; then
        echo "BUILD FAILED"; cat "tools/.build/$t.log"; fail=1; continue
    fi
    echo "ok"
done
[ $fail -ne 0 ] && { echo "=== build failures ==="; exit 1; }

# gen_world is a tool, not a test: run it so the generated world is current.
echo
for t in "${TARGETS[@]}"; do
    [ -f "tools/$t.exe" ] || continue
    case "$t" in
        test_*)    echo "--- running $t ---"; ./tools/"$t".exe || fail=1 ;;
        gen_world) echo "--- running gen_world ---"; ./tools/gen_world.exe || fail=1 ;;
    esac
done

if [ $# -eq 0 ]; then
    # Regenerate the stress-demo world (separate proto so it can't disturb the
    # walk world) and report the structure the analyst will see.
    echo
    echo "--- generating stress demo world (scene_stress.json) ---"
    ./tools/gen_world.exe scene_stress.json | grep -E "scenario|structure|wrote|bonds" || fail=1

    # Collision regression: the resolved trace, replayed as the controller will,
    # must keep every module pair essentially tangent (worst overlap <= 2 mm).
    echo
    echo "--- running check_collisions (collision + spacing regression, resolved trace) ---"
    ./tools/check_collisions.exe controllers/catom3d_controller/trace_resolved.csv \
        | grep -E "closest approach|worst pen|steps with|movingCatomRadius|along-track gap|spacing RESULT|RESULT" || fail=1
    ./tools/check_collisions.exe controllers/catom3d_controller/trace_resolved.csv \
        > /dev/null 2>&1 || { echo "COLLISION/SPACING REGRESSION FAILED"; fail=1; }

    # Fidelity regression: the resolved trace must still replay cell-for-cell.
    echo "--- running verify_trace (fidelity regression, resolved trace) ---"
    if "$GXX" $FLAGS -o tools/verify_trace.exe \
            controllers/catom3d_controller/verify_trace.cpp 2> tools/.build/verify_trace.log; then
        ./tools/verify_trace.exe controllers/catom3d_controller/trace_resolved.csv | tail -4 || fail=1
    else
        cat tools/.build/verify_trace.log; fail=1
    fi
fi

echo
[ $fail -eq 0 ] && echo "ALL GREEN" || echo "FAILURES ABOVE"
exit $fail
