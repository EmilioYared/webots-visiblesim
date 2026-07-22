// =============================================================================
// check_collisions.cpp  (offline, no Webots)
//
// Replays the whole VisibleSim trace with the SAME timing the Webots controller
// uses (US_PER_STEP, 40-step rolls, start-when-due) and, at every step, measures
// the center-to-center distance between every pair of modules — including ones
// mid-roll. Two modules whose bounding spheres (radius r) overlap have centers
// closer than 2r.
//
// This turns "I saw two catoms overlap" into a measured, located, timestamped
// fact, and separates two very different causes:
//   * sphere-level interpenetration  -> the algorithm/playback drives a solid
//     module through another: a real fidelity bug to fix;
//   * centers never below 2r          -> the collision shape is respected and
//     any overlap the eye caught is the boxy visual mesh at its corners.
//
//   Usage: check_collisions [trace.csv] [scene.json]
// =============================================================================

#include "../controllers/catom3d_controller/catom3d_core.hpp"
#include "../controllers/catom3d_controller/scene_config.hpp"

#include <iostream>
#include <iomanip>
#include <algorithm>
#include <array>
#include <map>
#include <set>
#include <tuple>

using Key = std::tuple<int,int,int>;
static Key key(GridPos g){ return std::make_tuple(g.x,g.y,g.z); }

struct Mod {
    GridPos init;
    std::vector<Move> moves;
    Mat4 mat;
    GridPos grid;
    size_t mi = 0;
    bool animating = false;
    int  releasing = 0;
    RotationAnim anim;
    double pos[3];       // world center this step
    bool inFlight = false;
};

static double worldDist(const double a[3], const double b[3]){
    double dx=a[0]-b[0], dy=a[1]-b[1], dz=a[2]-b[2];
    return std::sqrt(dx*dx+dy*dy+dz*dz);
}

// ---- lightWalk traffic-spacing rule (ports VisibleSim's intervalFreeCells /
// movementReservationsConflict) --------------------------------------------
// Free cells strictly between two 1-D intervals (0 if they touch/overlap).
static int intervalFreeCells(int minA, int maxA, int minB, int maxB){
    if (maxA < minB) return minB - maxA - 1;
    if (maxB < minA) return minA - maxB - 1;
    return 0;
}
// Along-track (x) free-cell gap between two moves, but ONLY when they share a
// y-line neighbourhood (no free line between their y-intervals) — otherwise the
// spacing rule does not constrain them, so return a large value. Matches
// movementReservationsConflict: conflict at radius R <=> this gap < R.
static int movingSpacingGap(GridPos fromA, GridPos toA, GridPos fromB, GridPos toB){
    int minLA=std::min(fromA.y,toA.y), maxLA=std::max(fromA.y,toA.y);
    int minLB=std::min(fromB.y,toB.y), maxLB=std::max(fromB.y,toB.y);
    if (intervalFreeCells(minLA,maxLA,minLB,maxLB) >= 1) return 1000000; // different lines
    int minXA=std::min(fromA.x,toA.x), maxXA=std::max(fromA.x,toA.x);
    int minXB=std::min(fromB.x,toB.x), maxXB=std::max(fromB.x,toB.x);
    return intervalFreeCells(minXA,maxXA,minXB,maxXB);
}

// The lattice cell that connector `con` of a catom (pose myMat, center c) points
// at. CONNECTOR_POS entries are unit vectors; the neighbour centre is one
// lattice spacing out along the rotated connector direction.
static GridPos neighborCellForConnector(const Mat4& myMat, const double c[3], int con){
    Vec3 local(CONNECTOR_POS[con][0], CONNECTOR_POS[con][1], CONNECTOR_POS[con][2], 0.0);
    Vec3 w = myMat * local;                                  // w=0 -> rotation only
    double nb[3] = { c[0]+w[0]*LATTICE_SCALE, c[1]+w[1]*LATTICE_SCALE, c[2]+w[2]*LATTICE_SCALE };
    return worldToGrid(nb);
}

// Diagnostics for the blocking-aware experiment. g_countMoves gates the counters
// to the measurement pass so the state-dump pass does not inflate them.
static long long g_movesTotal = 0, g_movesClear = 0, g_movesNoClear = 0;
static bool g_countMoves = true;

// VisibleSim only performs a rotation whose blocking cells are empty. Among the
// links that land on the target, prefer the one whose blocking cells are clear
// in the current occupancy — that is the arc VisibleSim actually took. Falls
// back to landing-closest when none qualify (and records it).
// Largest sphere interpenetration (0 if none) between an arc and the occupied
// neighbour centres, evaluated continuously over the 40 sub-steps.
static double arcMaxPenetration(RotationAnim anim, const std::vector<std::array<double,3>>& obst,
                                double solidD){
    double worst = 0.0; Mat4 s;
    for (int k = 0; k < 4*N_STEPS; ++k) {
        bool done = rotationNextStep(anim, s);
        double p[3]; matToPos(s, p);
        for (const auto& o : obst) {
            double d = worldDist(p, o.data());
            if (solidD - d > worst) worst = solidD - d;
        }
        if (done) break;
    }
    return worst;
}

static bool g_clearanceTiebreak = false;

static bool chooseAnimBlocking(const Mat4& myMat, const double myCenter[3], GridPos myGrid,
                               GridPos pivotCell, GridPos toCell,
                               const std::set<Key>& occ, double solidD, RotationAnim& out,
                               int* outTo = nullptr, int* outFace = nullptr){
    double pivW[3]; gridToWorld(pivotCell, pivW);
    double tgtW[3]; gridToWorld(toCell, tgtW);
    int con = connectorForNeighbor(myMat, pivW);
    if (con < 0) return false;
    Mat4 pivMat; pivMat.setTranslation(pivW[0], pivW[1], pivW[2]);

    // Occupied neighbour centres, for the continuous-clearance tiebreak.
    std::vector<std::array<double,3>> obst;
    if (g_clearanceTiebreak)
        for (const Key& k : occ) {
            GridPos g{std::get<0>(k), std::get<1>(k), std::get<2>(k)};
            if (g == myGrid) continue;
            double w[3]; gridToWorld(g, w);
            obst.push_back({w[0], w[1], w[2]});
        }

    double bestAny = 1e9;  RotationAnim animAny;   bool foundAny = false;
    int anyTo=-1, anyFace=-1;
    // Among blocking-clear arcs, prefer least penetration, then least landing error.
    double bestPen = 1e9, bestClearD = 1e9; RotationAnim animClear; bool foundClear = false;
    int clearTo=-1, clearFace=-1;
    for (const MotionLink& lnk : g_links) {
        if (lnk.from != con) continue;
        RotationAnim anim = buildRotationAnim(myMat, pivMat, lnk);
        Vec3 fp = anim.finalMat.getPosition();
        double dx=fp[0]-tgtW[0], dy=fp[1]-tgtW[1], dz=fp[2]-tgtW[2];
        double d = dx*dx+dy*dy+dz*dz;
        if (d < bestAny) { bestAny=d; animAny=anim; foundAny=true; anyTo=lnk.to; anyFace=(int)lnk.faceType; }
        if (d < 0.002) {
            bool clear = true;
            for (int b : lnk.blocking) {
                GridPos nb = neighborCellForConnector(myMat, myCenter, b);
                if (nb == myGrid) continue;               // our own cell is vacating
                if (occ.count(key(nb))) { clear = false; break; }
            }
            if (!clear) continue;
            double pen = g_clearanceTiebreak ? arcMaxPenetration(anim, obst, solidD) : 0.0;
            if (pen < bestPen - 1e-9 || (std::abs(pen-bestPen) <= 1e-9 && d < bestClearD)) {
                bestPen = pen; bestClearD = d; animClear = anim; foundClear = true;
                clearTo = lnk.to; clearFace = (int)lnk.faceType;
            }
        }
    }
    if (g_countMoves) g_movesTotal++;
    if (foundClear) { if(g_countMoves)g_movesClear++;   out = animClear;
                      if(outTo)*outTo=clearTo; if(outFace)*outFace=clearFace; return true; }
    if (foundAny)   { if(g_countMoves)g_movesNoClear++; out = animAny;
                      if(outTo)*outTo=anyTo; if(outFace)*outFace=anyFace; return true; }
    return false;
}

int main(int argc, char** argv) {
    buildMotionLinks();
    const char* tracePath = "controllers/catom3d_controller/trace.csv";
    const char* cfgArg    = nullptr;
    bool blockingMode = false;
    std::string emitPath;                 // --emit <file>: bake resolved arcs
    double maxPenMm = 2.0;                 // grazing below this passes (soft contacts absorb it)
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--blocking") blockingMode = true;
        else if (a == "--clearance") { blockingMode = true; g_clearanceTiebreak = true; }
        else if (a == "--emit") { blockingMode = true; g_clearanceTiebreak = true;
                                  if (i+1 < argc) emitPath = argv[++i]; }
        else if (a == "--max-pen") { if (i+1 < argc) maxPenMm = std::atof(argv[++i]); }
        else if (tracePath == std::string("controllers/catom3d_controller/trace.csv") &&
                 a.rfind("--",0) != 0) tracePath = argv[i];
        else if (!cfgArg && a.rfind("--",0) != 0) cfgArg = argv[i];
    }

    SceneConfig cfg;
    { std::string p = cfgArg ? cfgArg : findSceneConfigPath();
      if (!p.empty()) loadSceneConfig(p, cfg); }
    const double r        = cfg.boundingRadius;
    const double solidD   = 2.0 * r;                 // centers closer than this overlap
    const long long US    = cfg.usPerStep;
    const bool releaseDelay = cfg.bondsEnabled;      // physics world staggers by 2 steps

    std::vector<Move> all = loadTraceFrom(tracePath, nullptr);
    if (all.empty()) { std::cerr << "ERROR: no moves in " << tracePath << "\n"; return 1; }

    // One Mod per originating module.
    std::map<Key, Mod> byInit;
    for (const Move& m : all) {
        Mod& md = byInit[key(m.init)];
        md.init = m.init;
        md.moves.push_back(m);
    }
    std::vector<Mod> proto;
    for (auto& kv : byInit) {
        Mod md = kv.second;
        std::sort(md.moves.begin(), md.moves.end(),
                  [](const Move& a, const Move& b){ return a.t_us < b.t_us; });
        proto.push_back(md);
    }
    const int N = (int)proto.size();
    auto cell = [&](int i){ return proto[i].init; };

    std::cout << "modules " << N << ", radius " << r << " m, solid diameter "
              << solidD << " m, usPerStep " << US
              << ", releaseDelay " << (releaseDelay?"on":"off")
              << ", selection " << (blockingMode?"BLOCKING-AWARE":"landing-only") << "\n";

    double globalMin = 1e9; long long minStep = -1; int minA=-1, minB=-1;
    long long overlapSteps = 0;                 // steps with >=1 penetrating pair
    double worstPen = 0.0; long long worstPenStep=-1; int worstA=-1, worstB=-1;
    long long b1=0, b2=0, b5=0, b10=0;          // >0.1/1/2/5 mm penetration samples
    long long lastStep = 0;

    // Resolved arc per move (key: init cell + start time), captured in pass 1.
    std::map<std::pair<Key,long long>, std::pair<int,int>> resolved;
    bool capture = !emitPath.empty();

    // Traffic-spacing observation: the smallest along-track gap seen between any
    // two concurrently-moving catoms on the same/adjacent lines. That value is
    // the largest movingCatomRadius the trace actually guarantees.
    const int R = cfg.movingCatomRadius;
    int   minSpacingGap = 1000000; long long spacingStep=-1; int spacingA=-1, spacingB=-1;
    long long spacingViolations = 0;

    // One simulation pass. When dumpStep >= 0, print full state at that step.
    auto simulate = [&](long long dumpStep) {
        g_countMoves = (dumpStep < 0);            // count only the measurement pass
        std::vector<Mod> mods = proto;
        for (int i = 0; i < N; ++i) {
            double w[3]; gridToWorld(mods[i].init, w);
            mods[i].mat  = buildCatomMat(w, 0);
            mods[i].grid = mods[i].init;
            mods[i].pos[0]=w[0]; mods[i].pos[1]=w[1]; mods[i].pos[2]=w[2];
            mods[i].mi=0; mods[i].animating=false; mods[i].releasing=0; mods[i].inFlight=false;
        }
        long long step = 0;
        const long long STEP_CAP = 2000000;
        for (;;) {
            bool anyPending = false;

            // Occupancy = cells held by modules that are not mid-roll. Recomputed
            // each step so the blocking check sees the current configuration.
            std::set<Key> occ;
            if (blockingMode)
                for (int i = 0; i < N; ++i)
                    if (!mods[i].animating && mods[i].releasing == 0)
                        occ.insert(key(mods[i].grid));

            for (int i = 0; i < N; ++i) {
                Mod& md = mods[i];
                if (!md.animating && md.releasing == 0 && md.mi < md.moves.size() &&
                    step * US >= md.moves[md.mi].t_us) {
                    double myC[3]; gridToWorld(md.grid, myC);
                    int rTo=-1, rFace=-1;
                    bool ok = blockingMode
                        ? chooseAnimBlocking(md.mat, myC, md.grid, md.moves[md.mi].pivot,
                                             md.moves[md.mi].to, occ, solidD, md.anim, &rTo, &rFace)
                        : findAnimForMove(md.mat, md.moves[md.mi].pivot,
                                          md.moves[md.mi].to, md.anim,
                                          md.moves[md.mi].linkTo, md.moves[md.mi].linkFace);
                    if (ok && capture && dumpStep < 0)
                        resolved[{key(md.init), md.moves[md.mi].t_us}] = {rTo, rFace};
                    if (ok) {
                        if (releaseDelay) md.releasing = 2;
                        else              md.animating = true;
                    } else {
                        double tw[3]; gridToWorld(md.moves[md.mi].to, tw);
                        md.mat  = buildCatomMat(tw, oriFromMat(md.mat));
                        md.grid = md.moves[md.mi].to;
                        md.mi++;
                    }
                }
                if (md.releasing > 0 && --md.releasing == 0) md.animating = true;
                if (md.animating) {
                    Mat4 stepMat;
                    bool done = rotationNextStep(md.anim, stepMat);
                    matToPos(stepMat, md.pos);
                    md.inFlight = true;
                    if (done) {
                        matToPos(md.anim.finalMat, md.pos);
                        md.mat  = md.anim.finalMat;
                        md.grid = md.moves[md.mi].to;
                        md.mi++; md.animating=false; md.inFlight=false;
                    }
                } else if (md.releasing == 0) {
                    double w[3]; gridToWorld(md.grid, w);
                    md.pos[0]=w[0]; md.pos[1]=w[1]; md.pos[2]=w[2];
                    md.inFlight = false;
                }
                if (md.mi < md.moves.size() || md.animating || md.releasing > 0)
                    anyPending = true;
            }

            // Traffic-spacing check: among modules currently mid-roll, every
            // pair must satisfy the algorithm's movingCatomRadius rule.
            if (dumpStep < 0 && R > 0) {
                for (int i = 0; i < N; ++i) {
                    if (!mods[i].animating || mods[i].mi >= mods[i].moves.size()) continue;
                    for (int j = i+1; j < N; ++j) {
                        if (!mods[j].animating || mods[j].mi >= mods[j].moves.size()) continue;
                        const Move& mi = mods[i].moves[mods[i].mi];
                        const Move& mj = mods[j].moves[mods[j].mi];
                        int gap = movingSpacingGap(mi.from, mi.to, mj.from, mj.to);
                        if (gap < minSpacingGap) {
                            minSpacingGap = gap; spacingStep = step; spacingA=i; spacingB=j;
                        }
                        if (gap < R) spacingViolations++;
                    }
                }
            }

            double stepMin = 1e9;
            for (int i = 0; i < N; ++i)
                for (int j = i+1; j < N; ++j) {
                    double d = worldDist(mods[i].pos, mods[j].pos);
                    if (d < stepMin)   stepMin = d;
                    if (dumpStep < 0 && d < globalMin) { globalMin=d; minStep=step; minA=i; minB=j; }
                    double pen = solidD - d;
                    if (dumpStep < 0 && pen > 1e-4) {
                        if (pen > worstPen){worstPen=pen;worstPenStep=step;worstA=i;worstB=j;}
                        if (pen>0.0001)b1++; if (pen>0.001)b2++; if (pen>0.002)b5++; if (pen>0.005)b10++;
                    }
                }
            if (dumpStep < 0 && solidD - stepMin > 1e-4) overlapSteps++;

            if (step == dumpStep) {
                std::cout << "\n---- state at step " << step << " ----\n";
                for (int i = 0; i < N; ++i) {
                    const Mod& md = mods[i];
                    std::cout << "  @(" << md.init.x << "," << md.init.y << "," << md.init.z << ")"
                              << (md.animating ? " ROLL" : (md.releasing?" REL ":"     "))
                              << " grid(" << md.grid.x << "," << md.grid.y << "," << md.grid.z << ")"
                              << " pos[" << std::setprecision(3) << md.pos[0] << ","
                              << md.pos[1] << "," << md.pos[2] << "]";
                    if (md.animating && md.mi < md.moves.size())
                        std::cout << " ->("<< md.moves[md.mi].to.x << "," << md.moves[md.mi].to.y
                                  << "," << md.moves[md.mi].to.z << ")";
                    std::cout << "\n";
                }
                return;
            }
            step++;
            lastStep = step;
            if (!anyPending) break;
            if (step > STEP_CAP) { std::cerr << "WARN: step cap hit\n"; break; }
        }
        // settled check on the last pass only
        if (dumpStep < 0) {
            double settledMin = 1e9; int sa=-1, sb=-1;
            for (int i = 0; i < N; ++i)
                for (int j = i+1; j < N; ++j) {
                    double d = worldDist(mods[i].pos, mods[j].pos);
                    if (d < settledMin) { settledMin=d; sa=i; sb=j; }
                }
            std::cout << std::fixed << std::setprecision(4)
                      << "settled closest pair:        " << settledMin << " m (@("
                      << cell(sa).x << "," << cell(sa).y << "," << cell(sa).z << ") - @("
                      << cell(sb).x << "," << cell(sb).y << "," << cell(sb).z << "))\n";
        }
    };

    g_movesTotal = g_movesClear = g_movesNoClear = 0;   // count pass 1 only
    simulate(-1);
    long long step = lastStep;

    // --emit: re-read the trace in original order, append the resolved arc hint
    // (link-to connector, face type) to each row.
    if (!emitPath.empty()) {
        std::ifstream in(tracePath);
        std::ofstream em(emitPath.c_str());
        if (!in.is_open() || !em.is_open()) {
            std::cerr << "ERROR: cannot emit to " << emitPath << "\n"; return 1;
        }
        std::string line; std::getline(in, line);           // header
        em << line << ",linkTo,linkFace\n";
        int written=0, unresolved=0;
        while (std::getline(in, line)) {
            if (line.empty()) continue;
            std::istringstream ss(line); std::string t; long long v[14]; int k=0;
            while (k<14 && std::getline(ss,t,',')) { try{v[k]=std::stoll(t);++k;}catch(...){break;} }
            if (k<14) continue;
            Key ik = std::make_tuple((int)v[2],(int)v[3],(int)v[4]);
            auto it = resolved.find({ik, v[0]});
            int to = (it!=resolved.end()) ? it->second.first  : -1;
            int fc = (it!=resolved.end()) ? it->second.second : -1;
            if (to < 0) unresolved++;
            em << line << "," << to << "," << fc << "\n";
            written++;
        }
        std::cout << "emitted " << emitPath << ": " << written << " rows ("
                  << unresolved << " unresolved)\n";
    }

    std::cout << std::fixed << std::setprecision(4);
    std::cout << "\n================ collision report ================\n";
    std::cout << "steps simulated:            " << step << "\n";
    std::cout << "closest approach (any step): " << globalMin << " m";
    if (minStep>=0)
        std::cout << "  at step " << minStep << " between module@("
                  << cell(minA).x << "," << cell(minA).y << "," << cell(minA).z << ") and @("
                  << cell(minB).x << "," << cell(minB).y << "," << cell(minB).z << ")";
    std::cout << "\n";
    std::cout << "solid diameter (2r):        " << solidD << " m\n";
    std::cout << "steps with interpenetration: " << overlapSteps << "\n";
    std::cout << "pair-samples penetrating >0.1mm/>1mm/>2mm/>5mm: "
              << b1 << "/" << b2 << "/" << b5 << "/" << b10 << "\n";
    if (worstPenStep>=0)
        std::cout << "worst penetration:          " << worstPen*1000.0 << " mm at step "
                  << worstPenStep << " between @(" << cell(worstA).x << "," << cell(worstA).y
                  << "," << cell(worstA).z << ") and @(" << cell(worstB).x << ","
                  << cell(worstB).y << "," << cell(worstB).z << ")\n";

    // Dump the full configuration at the closest-approach step to reveal whether
    // it is mover-vs-mover or a roll path sweeping through a settled neighbour.
    if (minStep >= 0) simulate(minStep);

    if (blockingMode)
        std::cout << "\nblocking-aware selection: " << g_movesClear << " moves took a clear arc, "
                  << g_movesNoClear << " had no clear arc (of " << g_movesTotal << ")\n";

    // ---- traffic-spacing (movingCatomRadius) result ----
    bool spacingOK = true;
    std::cout << "\n---- traffic spacing (movingCatomRadius) ----\n";
    if (R <= 0) {
        std::cout << "movingCatomRadius = 0 (spacing disabled) — nothing to enforce\n";
    } else {
        int observed = (minSpacingGap >= 1000000) ? -1 : minSpacingGap;
        std::cout << "configured movingCatomRadius: " << R << " cells\n";
        if (observed < 0)
            std::cout << "no two catoms ever roll on the same/adjacent lines "
                         "concurrently — spacing trivially satisfied\n";
        else {
            std::cout << "observed smallest along-track gap between concurrent movers: "
                      << observed << " cells";
            if (spacingStep >= 0)
                std::cout << "  at step " << spacingStep << " between @("
                          << cell(spacingA).x << "," << cell(spacingA).y << "," << cell(spacingA).z
                          << ") and @(" << cell(spacingB).x << "," << cell(spacingB).y << ","
                          << cell(spacingB).z << ")";
            std::cout << "\n";
            std::cout << "spacing-rule violations (gap < radius): " << spacingViolations << "\n";
        }
        spacingOK = (spacingViolations == 0);
        std::cout << "spacing RESULT: "
                  << (spacingOK ? "PASS — trace honours movingCatomRadius\n"
                                : "FAIL — concurrent movers violate movingCatomRadius\n");
    }

    bool collisionOK = (worstPen * 1000.0 <= maxPenMm);
    std::cout << "\nRESULT: ";
    if (worstPen <= 1e-9)
        std::cout << "NO sphere interpenetration — collision shape is respected\n";
    else if (collisionOK)
        std::cout << "within tolerance — worst " << worstPen*1000.0 << " mm <= "
                  << maxPenMm << " mm (soft contacts absorb it)\n";
    else
        std::cout << "SPHERE INTERPENETRATION — worst " << worstPen*1000.0 << " mm > "
                  << maxPenMm << " mm: modules pass through each other\n";
    return (collisionOK && spacingOK) ? 0 : 3;
}
