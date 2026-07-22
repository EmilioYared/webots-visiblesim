// =============================================================================
// scenario.hpp — non-walk demo structures (stress scenarios).
//
// The default "walk" scenario gets its layout from the VisibleSim trace. Stress
// scenarios instead build a fixed structure geometrically so the structural-
// integrity checks and physics have something to actually work on: a flat sheet
// on a floor never sags, tips, or breaks a bond.
//
// Shared by tools/gen_world.cpp (to place the modules) and the tests (to assert
// the structure has the intended properties).
// =============================================================================
#ifndef SCENARIO_HPP
#define SCENARIO_HPP

#include "catom3d_core.hpp"
#include "scene_config.hpp"
#include <vector>

// Cantilever: a solid rectangular pedestal (well supported, resting on the
// ground plane) with a single-line horizontal arm extending past the pedestal
// edge in +x. The arm has nothing directly beneath it, so it overhangs; each
// interior arm cell is a cut-vertex (single point of failure); and a long
// enough arm pushes the centre of mass beyond the pedestal footprint. The root
// bond carries the whole arm's weight — the first thing to break under gravity.
inline std::vector<GridPos> buildCantilever(const SceneConfig& c) {
    std::vector<GridPos> cells;
    const int bw = std::max(1, c.cantBaseW), bd = std::max(1, c.cantBaseD),
              bh = std::max(1, c.cantBaseH);
    // Pedestal: solid bw x bd x bh block, lowest layer on the ground (z=0).
    for (int z = 0; z < bh; ++z)
        for (int y = c.cantY0; y < c.cantY0 + bd; ++y)
            for (int x = c.cantX0; x < c.cantX0 + bw; ++x)
                cells.push_back({x, y, z});
    // Arm: one line at the pedestal's top layer, running +x past its edge.
    const int armZ = bh - 1, armY = c.cantY0;
    for (int i = 0; i < c.cantArmLen; ++i)
        cells.push_back({c.cantX0 + bw + i, armY, armZ});
    return cells;
}

// Cells for a non-walk scenario; empty for "walk" (which uses the trace).
inline std::vector<GridPos> buildScenarioCells(const SceneConfig& c) {
    if (c.scenarioType == "cantilever") return buildCantilever(c);
    return {};
}

#endif // SCENARIO_HPP
