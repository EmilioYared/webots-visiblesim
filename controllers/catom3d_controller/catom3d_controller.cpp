// =============================================================================
// catom3d_controller.cpp
// Webots C++ controller for Catom3D modular robot — TRACE PLAYER.
//
// Visualizes the real VisibleSim lightWalkCatoms3D algorithm in Webots. The
// algorithm runs offline in VisibleSim (headless) and exports an exact per-move
// trace (trace.csv). Each Webots catom identifies itself by its initial FCC
// cell, replays only its own moves at the recorded sim-times, and animates each
// roll with the faithful port of Catoms3DRotation::nextStep. Because the trace
// already encodes the distributed coordination, no messaging, reservation, or
// neighbor table is needed here.
//
// All geometry/motion math lives in catom3d_core.hpp (shared with the offline
// verifier verify_trace.cpp).
// =============================================================================

#include <webots/Supervisor.hpp>
#include <webots/GPS.hpp>
#include <webots/Node.hpp>
#include <webots/Field.hpp>

#include <functional>
#include <tuple>

#include "catom3d_core.hpp"

using namespace webots;

// Load my moves, trying the locations the trace may live in relative to the
// controller's working directory.
static std::vector<Move> loadMyTrace(GridPos myInit) {
    const char* candidates[] = {
        "trace.csv",
        "../trace.csv",
        "controllers/catom3d_controller/trace.csv"
    };
    for (const char* c : candidates) {
        std::vector<Move> mv = loadTraceFrom(c, &myInit);
        std::ifstream probe(c);
        if (probe.is_open()) {
            std::cerr << "[trace] opened " << c << std::endl;
            return mv;
        }
    }
    std::cerr << "[trace] ERROR: could not open trace.csv" << std::endl;
    return {};
}

// =============================================================================
// STRUCTURAL INTEGRITY ANALYSIS  (Phase 2)
// One designated catom (ANALYST_NAME) reads the whole configuration every few
// steps, builds the FCC adjacency graph, and reports structural-integrity
// indicators:
//   * connected components  (a split structure has fallen apart)
//   * articulation points   (modules whose loss disconnects the structure = SPOFs)
//   * unsupported/overhang   (no module or floor beneath them)
//   * centre-of-mass over the support footprint (tip-over check)
// Read-only on poses, so it works in BOTH the kinematic and physics worlds.
// =============================================================================

static const char* ANALYST_NAME = "r0c0";

// Read every catom's current pose from the scene tree (analyst is a Supervisor)
// and return only the cells of modules that are actually AT a lattice cell.
// A module mid-roll is "in flight": its pose is far from any cell center, so
// rounding it would fabricate a bogus (disconnected) cell — exclude it instead
// and report the count via *inFlight. *total is the module count.
static std::vector<GridPos> collectPlacedCatomCells(Supervisor* robot,
                                                    int* inFlight, int* total) {
    std::vector<GridPos> cells; int nf=0, tot=0;
    Node* root = robot->getRoot();
    if(root){
        Field* ch = root->getField("children");
        if(ch){
            int n = ch->getCount();
            for(int i=0;i<n;++i){
                Node* nd = ch->getMFNode(i);
                if(!nd) continue;
                std::string tn = nd->getTypeName();
                if(tn!="Catom3D" && tn!="Catom3DPhysics") continue;
                Field* tf = nd->getField("translation");
                if(!tf) continue;
                const double* t = tf->getSFVec3f();
                double w[3]={t[0],t[1],t[2]};
                tot++;
                GridPos g = worldToGrid(w);
                double c[3]; gridToWorld(g,c);
                double dx=w[0]-c[0],dy=w[1]-c[1],dz=w[2]-c[2];
                if(dx*dx+dy*dy+dz*dz < 0.0004) cells.push_back(g); // within 2 cm: placed
                else nf++;
            }
        }
    }
    *inFlight=nf; *total=tot;
    return cells;
}

// How much VisibleSim sim-time one Webots timestep represents during playback.
// One roll = 2*N_STEPS = 40 animation steps; a VisibleSim move spans ~400000 us,
// so 400000/40 = 10000 us/step keeps consecutive moves of one module back-to-back
// while preserving the relative timing (and thus parallelism) across modules.
static const long long US_PER_STEP = 10000;

int main() {
    std::cerr << "[catom3d] trace player main() entered" << std::endl;
    buildMotionLinks();
    std::cerr << "[catom3d] links built: " << g_links.size() << std::endl;

    Supervisor *robot = new Supervisor();
    int timestep = (int)robot->getBasicTimeStep();
    std::string myName = robot->getName();

    GPS *gps = robot->getGPS("gps");
    if (!gps) { std::cerr << "[" << myName << "] ERROR: GPS 'gps' not found\n"; return 1; }
    gps->enable(timestep);

    Node  *self       = robot->getSelf();
    Field *transField = self ? self->getField("translation") : nullptr;
    Field *rotField   = self ? self->getField("rotation")    : nullptr;
    if (!self || !transField || !rotField) {
        std::cerr << "[" << myName << "] ERROR: self/translation/rotation not found\n";
        return 1;
    }

    // Physics present on this node? (Catom3DPhysics proto sets a Physics node.)
    Field* physField = self->getField("physics");
    bool hasPhysics = physField && physField->getSFNode() != NULL;

    // The designated analyst computes structural-integrity indicators.
    bool isAnalyst = (myName == ANALYST_NAME);
    std::ofstream structCsv;
    StructuralReport lastRep;
    if (isAnalyst) {
        structCsv.open("structural_metrics.csv", std::ios::out | std::ios::trunc);
        if (structCsv.is_open())
            structCsv << "step,total_modules,in_flight,placed,components,"
                         "articulation_points,supported,overhanging,"
                         "com_x,com_y,com_in_support\n";
        std::cerr << "[" << myName << "] structural analyst active (physics="
                  << (hasPhysics?"on":"off") << ")\n";
    }

    // ---- Startup: read GPS, determine my initial cell ----
    robot->step(timestep);
    const double* gv = gps->getValues();
    double p0[3] = {gv[0], gv[1], gv[2]};
    GridPos initCell = worldToGrid(p0);

    // Snap to the canonical VisibleSim initial condition: this cell, orientation 0.
    // This guarantees the first move's connector geometry matches the algorithm.
    double snap[3]; gridToWorld(initCell, snap);
    Mat4 myMat = buildCatomMat(snap, 0);
    transField->setSFVec3f(snap);
    double aa0[4]; matToAxisAngle(myMat, aa0); rotField->setSFRotation(aa0);

    GridPos myGrid = initCell;
    std::vector<Move> moves = loadMyTrace(initCell);
    std::cout << "[" << myName << "] init cell (" << initCell.x << ","
              << initCell.y << "," << initCell.z << "), "
              << moves.size() << " moves to play" << std::endl;

    // ---- Playback loop ----
    long long stepCount = 0;
    size_t    mi        = 0;
    bool      animating = false;
    RotationAnim anim;

    while (robot->step(timestep) != -1) {
        // Start the next move if it is due and we are idle.
        if (!animating && mi < moves.size() &&
            stepCount * US_PER_STEP >= moves[mi].t_us) {
            if (findAnimForMove(myMat, moves[mi].pivot, moves[mi].to, anim)) {
                animating = true;
            } else {
                std::cerr << "[" << myName << "] WARN: cannot reproduce move to ("
                          << moves[mi].to.x << "," << moves[mi].to.y << ","
                          << moves[mi].to.z << ") pivot (" << moves[mi].pivot.x << ","
                          << moves[mi].pivot.y << "," << moves[mi].pivot.z << ")\n";
                double tw[3]; gridToWorld(moves[mi].to, tw);
                myMat = buildCatomMat(tw, oriFromMat(myMat));
                transField->setSFVec3f(tw);
                double faa[4]; matToAxisAngle(myMat, faa); rotField->setSFRotation(faa);
                myGrid = moves[mi].to;
                mi++;
            }
        }

        // Advance the current animation by one step.
        if (animating) {
            if (hasPhysics) self->resetPhysics();  // keep the mover kinematic under physics
            Mat4 stepMat;
            bool done = rotationNextStep(anim, stepMat);
            double np[3]; matToPos(stepMat, np);
            transField->setSFVec3f(np);
            double aa[4]; matToAxisAngle(stepMat, aa);
            rotField->setSFRotation(aa);

            if (done) {
                double fp[3]; matToPos(anim.finalMat, fp);
                transField->setSFVec3f(fp);
                double faa[4]; matToAxisAngle(anim.finalMat, faa);
                rotField->setSFRotation(faa);
                myMat  = anim.finalMat;
                myGrid = moves[mi].to;
                std::cout << "[" << myName << "] move " << (mi+1) << "/" << moves.size()
                          << " -> (" << myGrid.x << "," << myGrid.y << "," << myGrid.z
                          << ")" << std::endl;
                mi++;
                animating = false;
            }
        }

        // Structural-integrity analysis (analyst only, throttled). In-flight
        // modules are excluded so metrics reflect the actual placed structure.
        if (isAnalyst && stepCount % 8 == 0) {
            int inFlight=0, total=0;
            std::vector<GridPos> cells = collectPlacedCatomCells(robot, &inFlight, &total);
            StructuralReport rep = analyzeStructure(cells);
            if (structCsv.is_open()) {
                structCsv << stepCount << "," << total << "," << inFlight << ","
                          << rep.modules << "," << rep.components << ","
                          << rep.articulation << "," << rep.supported << ","
                          << rep.overhanging << "," << rep.comX << "," << rep.comY
                          << "," << (rep.comInSupport?1:0) << "\n";
                structCsv.flush();
            }
            if (rep.components != lastRep.components ||
                rep.articulation != lastRep.articulation ||
                rep.overhanging  != lastRep.overhanging  ||
                rep.comInSupport != lastRep.comInSupport) {
                std::cout << "[STRUCT] step " << stepCount << ": placed=" << rep.modules
                          << " inFlight=" << inFlight
                          << " components=" << rep.components
                          << " articulation=" << rep.articulation
                          << " overhanging=" << rep.overhanging
                          << " CoM-in-support=" << (rep.comInSupport?"yes":"NO")
                          << std::endl;
                lastRep = rep;
            }
        }

        stepCount++;
    }

    delete robot;
    return 0;
}
