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
// Physics behaviour is not hardcoded: scene.json drives both the generated
// world (see tools/gen_world.cpp) and the runtime knobs read here.
//
// All geometry/motion math lives in catom3d_core.hpp (shared with the offline
// verifier verify_trace.cpp).
// =============================================================================

#include <webots/Supervisor.hpp>
#include <webots/GPS.hpp>
#include <webots/Node.hpp>
#include <webots/Field.hpp>
#include <webots/Connector.hpp>

#include <functional>
#include <tuple>
#include <array>

#include "catom3d_core.hpp"
#include "scene_config.hpp"

using namespace webots;

// Load my moves, trying the locations the trace may live in relative to the
// controller's working directory.
static std::vector<Move> loadMyTrace(GridPos myInit, const std::string& configured) {
    std::string candidates[] = {
        configured,
        "../" + configured,
        "controllers/catom3d_controller/" + configured
    };
    for (size_t i = 0; i < sizeof(candidates)/sizeof(candidates[0]); ++i) {
        std::ifstream probe(candidates[i].c_str());
        if (!probe.is_open()) continue;
        std::cerr << "[trace] opened " << candidates[i] << std::endl;
        return loadTraceFrom(candidates[i].c_str(), &myInit);
    }
    std::cerr << "[trace] ERROR: could not open " << configured << std::endl;
    return {};
}

// =============================================================================
// STRUCTURAL INTEGRITY ANALYSIS  (Phase 2)
// One designated catom (analysis.analyst in scene.json) reads the whole
// configuration every few steps, builds the FCC adjacency graph, and reports
// structural-integrity indicators:
//   * connected components  (a split structure has fallen apart)
//   * articulation points   (modules whose loss disconnects the structure = SPOFs)
//   * unsupported/overhang   (no module or floor beneath them)
//   * centre-of-mass over the support footprint (tip-over check)
// Read-only on poses, so it works in BOTH the kinematic and physics worlds.
// =============================================================================

// Every catom proto shares the "Catom3D" prefix (Catom3D, Catom3DPhysics,
// Catom3DStress), so match on that rather than exact type names.
static bool isCatomType(const std::string& tn) {
    return tn.rfind("Catom3D", 0) == 0;
}

// A catom as seen by the analyst: its scene node, its lattice cell, and whether
// it is actually AT that cell. A module mid-roll is "in flight": its pose is far
// from any cell center, so rounding it would fabricate a bogus (disconnected)
// cell — flag it instead so the analysis can exclude it.
struct CatomView { Node* node; GridPos cell; bool placed; };

static std::vector<CatomView> collectCatomViews(Supervisor* robot, double tolSq) {
    std::vector<CatomView> out;
    Node* root = robot->getRoot(); if(!root) return out;
    Field* ch = root->getField("children"); if(!ch) return out;
    int n = ch->getCount();
    for(int i=0;i<n;++i){
        Node* nd = ch->getMFNode(i); if(!nd) continue;
        if(!isCatomType(nd->getTypeName())) continue;
        Field* tf = nd->getField("translation"); if(!tf) continue;
        const double* t = tf->getSFVec3f();
        double w[3]={t[0],t[1],t[2]};
        GridPos g = worldToGrid(w);
        double c[3]; gridToWorld(g,c);
        double dx=w[0]-c[0],dy=w[1]-c[1],dz=w[2]-c[2];
        out.push_back({nd, g, (dx*dx+dy*dy+dz*dz < tolSq)});
    }
    return out;
}

// Every catom's raw world position (including modules mid-roll), for the live
// collision monitor. Two solid modules overlap when their centers are closer
// than 2*boundingRadius.
static std::vector<std::array<double,3>> collectAllCatomPositions(Supervisor* robot) {
    std::vector<std::array<double,3>> out;
    Node* root = robot->getRoot(); if(!root) return out;
    Field* ch = root->getField("children"); if(!ch) return out;
    int n = ch->getCount();
    for(int i=0;i<n;++i){
        Node* nd = ch->getMFNode(i); if(!nd) continue;
        if(!isCatomType(nd->getTypeName())) continue;
        Field* tf = nd->getField("translation"); if(!tf) continue;
        const double* t = tf->getSFVec3f();
        out.push_back({t[0],t[1],t[2]});
    }
    return out;
}

int main(int argc, char** argv) {
    std::cerr << "[catom3d] trace player main() entered" << std::endl;
    buildMotionLinks();
    std::cerr << "[catom3d] links built: " << g_links.size() << std::endl;

    Supervisor *robot = new Supervisor();
    int timestep = (int)robot->getBasicTimeStep();
    std::string myName = robot->getName();

    // ---- Configuration (shared with the world generator) ----
    // controllerArgs from the world: the scene-config basename, and optionally
    // the literal "dynamic" marking this module as physics-driven (it may sag,
    // fall, or break free) rather than a held part of the static substrate.
    std::string cfgName = "scene.json";
    bool selfDynamic = false, gotCfg = false;
    for (int i = 1; i < argc; ++i) {
        std::string a = (argv[i] ? argv[i] : "");
        if (a == "dynamic") selfDynamic = true;
        else if (!gotCfg && !a.empty()) { cfgName = a; gotCfg = true; }
    }
    SceneConfig cfg;
    std::string cfgPath = findConfigNamed(cfgName);
    SceneConfigLoad cfgLoad = loadSceneConfig(cfgPath.empty() ? cfgName : cfgPath, cfg);
    bool isAnalyst = (myName == cfg.analystName);
    if (isAnalyst) {                       // one voice only: 20 copies would spam
        std::cout << "[config] " << (cfgPath.empty() ? "defaults (no scene.json found)"
                                                     : cfgPath) << std::endl;
        for (size_t i = 0; i < cfgLoad.warnings.size(); ++i)
            std::cout << "[config] WARNING: " << cfgLoad.warnings[i] << std::endl;
    }
    if (!cfgLoad.ok) {
        std::cerr << "[" << myName << "] ERROR: " << cfgLoad.error << std::endl;
        return 1;
    }
    const double placedTolSq = cfg.placedToleranceM * cfg.placedToleranceM;

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

    // Physics present on this node? (the generated proto adds it when
    // module.physicsEnabled is true).
    Field* physField = self->getField("physics");
    bool hasPhysics = physField && physField->getSFNode() != NULL;

    // ---- Lattice bonds -----------------------------------------------------
    // The generated proto carries one Connector per FCC direction. Lock state is
    // driven from here rather than by autoLock: a module about to roll releases
    // its bonds first, so it is never grabbed by the modules it passes, and
    // re-bonds once it has landed. Absent in the kinematic world — handled.
    // Enumerate rather than probe by name: asking for a device that does not
    // exist logs a warning per call, and the kinematic world has none.
    std::vector<Connector*> bonds;
    for (int i = 0; i < robot->getNumberOfDevices(); ++i) {
        Device* d = robot->getDeviceByIndex(i);
        if (d && d->getNodeType() == Node::CONNECTOR)
            bonds.push_back(robot->getConnector(d->getName()));
    }
    const bool hasBonds = !bonds.empty();
    auto lockBonds   = [&]() { for (size_t i = 0; i < bonds.size(); ++i) bonds[i]->lock();   };
    auto unlockBonds = [&]() { for (size_t i = 0; i < bonds.size(); ++i) bonds[i]->unlock(); };

    std::ofstream structCsv;
    StructuralReport lastRep;
    const double solidDiam = 2.0 * cfg.boundingRadius;  // centers closer than this overlap
    double monitorWorstPen = 0.0;                       // deepest overlap reported so far
    if (isAnalyst) {
        structCsv.open(cfg.metricsCsv.c_str(), std::ios::out | std::ios::trunc);
        if (structCsv.is_open())
            structCsv << "step,total_modules,in_flight,placed,components,"
                         "articulation_points,supported,overhanging,"
                         "com_x,com_y,com_in_support\n";
        std::cerr << "[" << myName << "] structural analyst active (physics="
                  << (hasPhysics?"on":"off") << ", bonds="
                  << (hasBonds?"on":"off") << ", movingCatomRadius="
                  << cfg.movingCatomRadius << " [verified offline])\n";
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

    // Bond to the neighbours we start adjacent to.
    if (hasBonds) lockBonds();

    GridPos myGrid = initCell;
    std::vector<Move> moves = loadMyTrace(initCell, cfg.tracePath);
    std::cout << "[" << myName << "] init cell (" << initCell.x << ","
              << initCell.y << "," << initCell.z << "), "
              << moves.size() << " moves to play" << std::endl;

    // ---- Playback loop ----
    long long stepCount = 0;
    size_t    mi        = 0;
    bool      animating = false;
    int       releasing = 0;      // steps to wait after unlocking, before moving
    bool      drifted   = false;  // physics pulled us off our cell while idle
    RotationAnim anim;

    while (robot->step(timestep) != -1) {
        // Start the next move if it is due and we are idle. With bonds present,
        // release them first and let a step pass: detaching a joint and
        // teleporting the solid within one step makes the solver spike.
        if (!animating && releasing == 0 && mi < moves.size() &&
            stepCount * cfg.usPerStep >= moves[mi].t_us) {
            if (findAnimForMove(myMat, moves[mi].pivot, moves[mi].to, anim,
                                moves[mi].linkTo, moves[mi].linkFace)) {
                if (hasBonds) { unlockBonds(); releasing = 2; }
                else          { animating = true; }
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

        // Bonds released — start rolling.
        if (releasing > 0 && --releasing == 0) animating = true;

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
                if (hasPhysics) self->resetPhysics();
                if (hasBonds)   lockBonds();       // re-bond at the new cell
                std::cout << "[" << myName << "] move " << (mi+1) << "/" << moves.size()
                          << " -> (" << myGrid.x << "," << myGrid.y << "," << myGrid.z
                          << ")" << std::endl;
                mi++;
                animating = false;
            }
        }
        // Hold idle substrate modules kinematically. Without this, a resting
        // structure of modules bonded in every direction is an over-constrained
        // rigid network that ODE resolves explosively ("popcorn"). Modules
        // marked dynamic are left to physics so they can sag / fall / break free.
        else if (!selfDynamic && cfg.holdWhenIdle) {
            if (hasPhysics) self->resetPhysics();
            double hp[3];  matToPos(myMat, hp);        transField->setSFVec3f(hp);
            double haa[4]; matToAxisAngle(myMat, haa); rotField->setSFRotation(haa);
        }

        // Idle drift check: a HELD module that leaves its cell means the pin
        // failed (e.g. holdWhenIdle off); a dynamic module is expected to move,
        // so it is excluded here.
        if (selfDynamic) { /* dynamic modules move under physics by design */ }
        else
        if (hasPhysics && !animating && releasing == 0) {
            const double* t = transField->getSFVec3f();
            double home[3]; gridToWorld(myGrid, home);
            double dx=t[0]-home[0], dy=t[1]-home[1], dz=t[2]-home[2];
            double d2 = dx*dx + dy*dy + dz*dz;
            if (d2 > placedTolSq && !drifted) {
                drifted = true;
                std::cout << "[PHYSICS] " << myName << " drifted "
                          << std::sqrt(d2) << " m off cell (" << myGrid.x << ","
                          << myGrid.y << "," << myGrid.z << ") at step " << stepCount
                          << std::endl;
            } else if (d2 <= placedTolSq) {
                drifted = false;
            }
        }

        // Structural-integrity analysis (analyst only, throttled). In-flight
        // modules are excluded so metrics reflect the actual placed structure.
        if (isAnalyst && stepCount % cfg.analysisEveryNSteps == 0) {
            // Live collision monitor: the algorithm is discrete-cell, so a roll
            // arc can still graze a neighbour. Report the deepest overlap seen
            // (once per new worst, to avoid spamming) so the physics violation
            // is visible in Webots, not only offline.
            std::vector<std::array<double,3>> P = collectAllCatomPositions(robot);
            double closest = 1e9; int ci=-1, cj=-1;
            for (size_t a=0;a<P.size();++a)
                for (size_t b=a+1;b<P.size();++b){
                    double dx=P[a][0]-P[b][0], dy=P[a][1]-P[b][1], dz=P[a][2]-P[b][2];
                    double d=std::sqrt(dx*dx+dy*dy+dz*dz);
                    if(d<closest){closest=d;ci=(int)a;cj=(int)b;}
                }
            double pen = solidDiam - closest;
            if (pen > 0.001 && pen > monitorWorstPen + 0.0005) {  // new worst, >1 mm
                monitorWorstPen = pen;
                std::cout << "[COLLISION] step " << stepCount << ": modules "
                          << ci << " and " << cj << " overlap by "
                          << pen*1000.0 << " mm (centers " << closest << " m < "
                          << solidDiam << " m)" << std::endl;
            }

            std::vector<CatomView> views = collectCatomViews(robot, placedTolSq);
            std::vector<GridPos> cells; std::vector<Node*> nodes;
            int inFlight=0, total=(int)views.size();
            for (const CatomView& v : views) {
                if (v.placed) { cells.push_back(v.cell); nodes.push_back(v.node); }
                else inFlight++;
            }
            StructuralReport rep = analyzeStructure(cells);

            // Live risk colouring: recolour each placed module by its structural
            // role so single-points-of-failure and overhangs are visible in the
            // 3D view. Indices line up with `cells`, hence with `nodes`.
            if (cfg.colorByRisk) {
                for (size_t i = 0; i < nodes.size(); ++i) {
                    bool art = rep.cellArticulation[i], ovh = rep.cellOverhang[i];
                    double col[3];
                    if      (art && ovh) { col[0]=1.0; col[1]=0.0; col[2]=0.0; } // red   = SPOF + overhang
                    else if (art)        { col[0]=0.8; col[1]=0.0; col[2]=0.8; } // purple= SPOF
                    else if (ovh)        { col[0]=1.0; col[1]=0.55;col[2]=0.0; } // orange= overhang
                    else                 { col[0]=0.2; col[1]=0.7; col[2]=0.2; } // green = healthy
                    Field* cf = nodes[i]->getField("color");
                    if (cf) cf->setSFColor(col);
                }
            }
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
