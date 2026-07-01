// =============================================================================
// verify_trace.cpp  (offline, no Webots)
//
// Replays the VisibleSim move trace through the SAME geometry/motion core that
// the Webots controller uses (catom3d_core.hpp) and checks, cell-for-cell, that
// each module reproduces every move: it starts where the trace says, rolls
// around the recorded pivot, and lands exactly on the recorded target cell.
//
// Usage: verify_trace [trace.csv]
// =============================================================================

#include "catom3d_core.hpp"
#include <tuple>
#include <map>

using Key = std::tuple<int,int,int>;
static Key key(GridPos g){ return std::make_tuple(g.x,g.y,g.z); }

int main(int argc, char** argv) {
    const char* path = (argc > 1) ? argv[1] : "trace.csv";
    buildMotionLinks();

    std::vector<Move> all = loadTraceFrom(path, nullptr);
    if (all.empty()) {
        std::cerr << "ERROR: no moves loaded from " << path << std::endl;
        return 1;
    }

    // Group moves by originating module (its initial cell).
    std::map<Key, std::vector<Move>> byModule;
    for (const Move& m : all) byModule[key(m.init)].push_back(m);
    for (auto& kv : byModule)
        std::sort(kv.second.begin(), kv.second.end(),
                  [](const Move& a, const Move& b){ return a.t_us < b.t_us; });

    int totalMoves = 0, reproduced = 0, fromMismatch = 0, landMismatch = 0, noLink = 0;
    int modulesOK = 0;
    std::vector<GridPos> finalCells;

    for (auto& kv : byModule) {
        GridPos init{std::get<0>(kv.first), std::get<1>(kv.first), std::get<2>(kv.first)};
        double snap[3]; gridToWorld(init, snap);
        Mat4 myMat = buildCatomMat(snap, 0);
        GridPos myGrid = init;
        bool moduleOK = true;

        for (const Move& m : kv.second) {
            totalMoves++;

            // The trace says this module rolls from m.from; we should be there.
            if (myGrid != m.from) {
                fromMismatch++; moduleOK = false;
                std::cerr << "module(" << init.x << "," << init.y << "," << init.z
                          << ") at (" << myGrid.x << "," << myGrid.y << "," << myGrid.z
                          << ") but trace from (" << m.from.x << "," << m.from.y << ","
                          << m.from.z << ")\n";
            }

            RotationAnim anim;
            if (!findAnimForMove(myMat, m.pivot, m.to, anim)) {
                noLink++; moduleOK = false;
                std::cerr << "module(" << init.x << "," << init.y << "," << init.z
                          << "): no link from (" << myGrid.x << "," << myGrid.y << ","
                          << myGrid.z << ") pivot (" << m.pivot.x << "," << m.pivot.y << ","
                          << m.pivot.z << ") -> (" << m.to.x << "," << m.to.y << ","
                          << m.to.z << ")\n";
                continue; // can't advance this module reliably
            }

            // Run the animation to completion and check the landing cell.
            Mat4 stepMat; bool done = false; int guard = 0;
            while (!done && guard++ < 4*N_STEPS) done = rotationNextStep(anim, stepMat);
            myMat = anim.finalMat;
            double fp[3]; matToPos(myMat, fp);
            GridPos landed = worldToGrid(fp);

            if (landed != m.to) {
                landMismatch++; moduleOK = false;
                std::cerr << "module(" << init.x << "," << init.y << "," << init.z
                          << "): landed (" << landed.x << "," << landed.y << ","
                          << landed.z << ") expected (" << m.to.x << "," << m.to.y << ","
                          << m.to.z << ")\n";
            } else {
                reproduced++;
            }
            myGrid = landed;
        }

        finalCells.push_back(myGrid);
        if (moduleOK) modulesOK++;
    }

    std::sort(finalCells.begin(), finalCells.end(),
              [](GridPos a, GridPos b){ return key(a) < key(b); });

    std::cout << "================ trace verification ================\n";
    std::cout << "modules:                 " << byModule.size() << " (" << modulesOK << " fully OK)\n";
    std::cout << "total moves:             " << totalMoves << "\n";
    std::cout << "reproduced exactly:      " << reproduced << "\n";
    std::cout << "from-cell mismatches:    " << fromMismatch << "\n";
    std::cout << "no-link failures:        " << noLink << "\n";
    std::cout << "landing mismatches:      " << landMismatch << "\n";
    std::cout << "final cells (" << finalCells.size() << "):\n   ";
    for (size_t i = 0; i < finalCells.size(); ++i) {
        std::cout << "(" << finalCells[i].x << "," << finalCells[i].y << ","
                  << finalCells[i].z << ") ";
        if ((i+1) % 5 == 0) std::cout << "\n   ";
    }
    std::cout << "\n";
    // Structural-integrity sanity check on the start and end configurations.
    std::vector<GridPos> initialCells;
    for (auto& kv : byModule)
        initialCells.push_back({std::get<0>(kv.first), std::get<1>(kv.first),
                                std::get<2>(kv.first)});
    StructuralReport ri = analyzeStructure(initialCells);
    StructuralReport rf = analyzeStructure(finalCells);
    auto printRep = [](const char* tag, const StructuralReport& r){
        std::cout << tag << ": modules=" << r.modules << " components=" << r.components
                  << " articulation=" << r.articulation << " supported=" << r.supported
                  << " overhanging=" << r.overhanging << " CoM-in-support="
                  << (r.comInSupport?"yes":"no") << "\n";
    };
    std::cout << "---------------- structural analysis ----------------\n";
    printRep("initial (Sheet A)", ri);
    printRep("final   (Sheet B)", rf);

    bool pass = (fromMismatch == 0 && noLink == 0 && landMismatch == 0);
    std::cout << (pass ? "RESULT: PASS — every move reproduced exactly\n"
                       : "RESULT: FAIL — see mismatches above\n");
    return pass ? 0 : 2;
}
