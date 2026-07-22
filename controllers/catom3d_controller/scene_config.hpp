// =============================================================================
// scene_config.hpp — typed view of scene.json.
//
// One source of truth for the physics setup, read by two consumers:
//   * tools/gen_world.cpp — bakes the values Webots can only accept at world
//     load time (boundingObject, ContactProperties, basicTimeStep, Connector
//     strengths) into the generated .proto and .wbt;
//   * the controller — reads the values it applies at runtime (playback rate,
//     trace path, analysis cadence, tolerances).
//
// Every field has a default, so a partial (or absent) scene.json still yields a
// working scene. Unknown keys are reported as warnings rather than ignored,
// because a silently-misspelled key looks exactly like a setting that "did not
// take effect".
//
// Depends only on json_min.hpp — no Webots, no lattice math.
// =============================================================================
#ifndef SCENE_CONFIG_HPP
#define SCENE_CONFIG_HPP

#include "json_min.hpp"
#include <string>
#include <vector>
#include <sstream>

struct SceneConfig {
    // ---- world ----
    std::string title         = "Catom3D Sheet Walk - generated from scene.json";
    double      basicTimeStep = 8.0;          // ms
    // WorldInfo.gravity is an SFFloat in Webots: a magnitude along the down
    // axis (-Z under the default ENU coordinate system), not a vector.
    double      gravity       = 9.81;         // m/s^2
    double      worldCFM      = 1e-5;         // ODE global constraint force mixing
    double      worldERP      = 0.2;          // ODE global error reduction
    // Resting bodies are put to sleep by ODE; without this, 20 stacked spheres
    // accumulate micro-jitter indefinitely.
    double      physicsDisableTime             = 1.0;    // s
    double      physicsDisableLinearThreshold  = 0.01;   // m/s
    double      physicsDisableAngularThreshold = 0.01;   // rad/s

    // ---- floor ----
    bool        floorEnabled  = true;
    bool        floorAutoFit  = true;         // fit XY extent to the visited cells
    double      floorMargin   = 0.15;         // m, added around the fitted extent
    double      floorThickness= 0.01;         // m
    double      floorSize[3]  = {1.3, 0.45, 0.01};   // used when autoFit = false
    double      floorCenter[3]= {1.0, 0.2, 0.12178}; // used when autoFit = false
    bool        floorTopZAuto = true;         // sit the floor under the ZLINE layer
    double      floorTopZ     = 0.12678;      // used when floorTopZAuto = false
    std::string floorMaterial = "floor";

    // ---- module ----
    bool        physicsEnabled = true;        // false -> kinematic world, no Physics
    double      mass           = 0.1;         // kg
    double      boundingRadius = 0.05;        // m
    double      linearDamping  = 0.2;
    double      angularDamping = 0.2;
    std::string moduleMaterial = "catom";
    // A resting sheet of modules rigidly bonded in every direction is an
    // over-constrained closed-loop system that ODE resolves explosively
    // ("popcorn"). Holding idle modules kinematically at their cell keeps the
    // substrate rock-stable while physics still runs for modules explicitly
    // marked dynamic (e.g. a cantilever arm that must sag and break free).
    bool        holdWhenIdle   = true;

    // ---- contact ----
    // Webots' ContactProperties defaults are wrong for this scene: bounce
    // defaults to 0.5, which makes resting modules jitter, and rollingFriction
    // defaults to 0, which lets spherical modules roll away indefinitely.
    double      coulombFriction    = 0.8;
    double      bounce             = 0.0;
    double      bounceVelocity     = 0.01;
    double      rollingFriction    = 0.05;    // emitted as the 3 ODE components
    double      forceDependentSlip = 0.0;
    double      softCFM            = 1e-5;
    double      softERP            = 0.2;
    int         maxContactJoints   = 10;

    // ---- bonds (Webots Connector nodes; -1 strength = unbreakable) ----
    bool        bondsEnabled      = true;
    double      tensileStrength   = 15.0;     // N
    double      shearStrength     = 10.0;     // N
    double      distanceTolerance = 0.02;     // m
    double      axisTolerance     = 0.3;      // rad
    double      rotationTolerance = 0.4;      // rad
    int         numberOfRotations = 0;        // 0 = rotational alignment unchecked

    // ---- scenario ----
    // "walk" (default): modules replay the lightWalk trace. "cantilever": a
    // static stress structure (supported base + overhanging arm) that exercises
    // the structural-integrity checks and physics (sag / bond load / tip-over).
    std::string scenarioType = "walk";
    int cantBaseW   = 2;      // base pedestal width  (x cells)
    int cantBaseD   = 2;      // base pedestal depth  (y cells)
    int cantBaseH   = 3;      // base pedestal height (z layers)
    int cantArmLen  = 8;      // overhanging arm length (cells, +x)
    int cantX0      = 5;      // base origin x
    int cantY0      = 1;      // base origin y (arm runs along this line)

    // ---- algorithm ----
    // The lightWalk traffic-spacing radius the trace was generated with. Two
    // concurrently-moving catoms on the same/adjacent y-lines must keep at least
    // this many free cells between their x-travel intervals (VisibleSim's
    // movementReservationsConflict rule). This is a property of the TRACE: to
    // change how the modules actually move, set the same value in VisibleSim's
    // applicationsBin/.../config.xml and regenerate the trace. Declared here so
    // playback can VERIFY the trace still honours it. 0 disables spacing.
    // (radius=1 livelocks this scenario; use 0 or 2.)
    int         movingCatomRadius = 2;

    // ---- playback ----
    long long   usPerStep = 10000;            // VisibleSim us per Webots step
    std::string tracePath = "trace.csv";

    // ---- analysis ----
    std::string analystName        = "r0c0";
    int         analysisEveryNSteps= 8;
    double      placedToleranceM   = 0.02;    // within this of a cell centre = placed
    std::string metricsCsv         = "structural_metrics.csv";
    bool        colorByRisk        = false;   // recolour modules by structural risk

    // ---- generator output ----
    std::string outProto       = "protos/Catom3DPhysics.proto";
    std::string outWorld       = "worlds/catom3d_world_physics.wbt";
    std::string protoName      = "Catom3DPhysics";
    std::string controllerName = "catom3d_controller";
};

// Every path the schema recognises. Anything else in the file is a typo.
inline const char* const* sceneConfigKnownKeys(size_t* n) {
    static const char* keys[] = {
        "world.title", "world.basicTimeStep", "world.gravity",
        "world.CFM", "world.ERP", "world.physicsDisableTime",
        "world.physicsDisableLinearThreshold", "world.physicsDisableAngularThreshold",
        "floor.enabled", "floor.autoFit", "floor.margin", "floor.thickness",
        "floor.size", "floor.center", "floor.topZAuto", "floor.topZ",
        "floor.contactMaterial",
        "module.physicsEnabled", "module.mass", "module.boundingRadius",
        "module.linearDamping", "module.angularDamping", "module.contactMaterial",
        "module.holdWhenIdle",
        "contact.coulombFriction", "contact.bounce", "contact.bounceVelocity",
        "contact.rollingFriction", "contact.forceDependentSlip",
        "contact.softCFM", "contact.softERP", "contact.maxContactJoints",
        "bonds.enabled", "bonds.tensileStrength", "bonds.shearStrength",
        "bonds.distanceTolerance", "bonds.axisTolerance",
        "bonds.rotationTolerance", "bonds.numberOfRotations",
        "scenario.type", "scenario.baseW", "scenario.baseD", "scenario.baseH",
        "scenario.armLength", "scenario.x0", "scenario.y0",
        "algorithm.movingCatomRadius",
        "playback.usPerStep", "playback.trace",
        "analysis.analyst", "analysis.everyNSteps", "analysis.placedTolerance",
        "analysis.metricsCsv", "analysis.colorByRisk",
        "output.proto", "output.world", "output.protoName", "output.controller"
    };
    *n = sizeof(keys) / sizeof(keys[0]);
    return keys;
}

struct SceneConfigLoad {
    bool                     ok = false;
    std::string              error;       // set when the file is unusable
    std::vector<std::string> warnings;    // typos, out-of-range values
};

// Parse `text` into `cfg`. Missing keys keep their defaults.
inline SceneConfigLoad parseSceneConfig(const std::string& text, SceneConfig& cfg) {
    using namespace jsonm;
    SceneConfigLoad res;

    Result r = parse(text);
    if (!r.ok)             { res.error = "scene.json: " + r.error; return res; }
    if (!r.value.isObject()){ res.error = "scene.json: top level must be an object"; return res; }
    const Value& v = r.value;

    // ---- unknown-key detection ----
    {
        std::vector<std::string> present;
        collectPaths(v, "", present);
        size_t nKnown = 0;
        const char* const* known = sceneConfigKnownKeys(&nKnown);
        for (size_t i = 0; i < present.size(); ++i) {
            bool found = false;
            for (size_t k = 0; k < nKnown && !found; ++k)
                if (present[i] == known[k]) found = true;
            if (!found) res.warnings.push_back("unknown key \"" + present[i] + "\" (ignored)");
        }
    }

    cfg.title         = stringOr(v, "world.title",         cfg.title);
    cfg.basicTimeStep = numberOr(v, "world.basicTimeStep", cfg.basicTimeStep);
    // Webots takes gravity as a magnitude along the down axis. A vector here is
    // a natural mistake and would otherwise be silently ignored.
    {
        const jsonm::Value* g = v.at("world.gravity");
        if (g && g->isArray()) {
            res.warnings.push_back("world.gravity must be a number (magnitude along the "
                                   "down axis), not an array - using "
                                   + std::to_string(cfg.gravity));
        } else {
            cfg.gravity = numberOr(v, "world.gravity", cfg.gravity);
        }
    }
    cfg.worldCFM = numberOr(v, "world.CFM", cfg.worldCFM);
    cfg.worldERP = numberOr(v, "world.ERP", cfg.worldERP);
    cfg.physicsDisableTime             = numberOr(v, "world.physicsDisableTime",             cfg.physicsDisableTime);
    cfg.physicsDisableLinearThreshold  = numberOr(v, "world.physicsDisableLinearThreshold",  cfg.physicsDisableLinearThreshold);
    cfg.physicsDisableAngularThreshold = numberOr(v, "world.physicsDisableAngularThreshold", cfg.physicsDisableAngularThreshold);

    cfg.floorEnabled   = boolOr  (v, "floor.enabled",   cfg.floorEnabled);
    cfg.floorAutoFit   = boolOr  (v, "floor.autoFit",   cfg.floorAutoFit);
    cfg.floorMargin    = numberOr(v, "floor.margin",    cfg.floorMargin);
    cfg.floorThickness = numberOr(v, "floor.thickness", cfg.floorThickness);
    vec3Into(v, "floor.size",   cfg.floorSize);
    vec3Into(v, "floor.center", cfg.floorCenter);
    cfg.floorTopZAuto  = boolOr  (v, "floor.topZAuto",  cfg.floorTopZAuto);
    cfg.floorTopZ      = numberOr(v, "floor.topZ",      cfg.floorTopZ);
    cfg.floorMaterial  = stringOr(v, "floor.contactMaterial", cfg.floorMaterial);

    cfg.physicsEnabled = boolOr  (v, "module.physicsEnabled", cfg.physicsEnabled);
    cfg.mass           = numberOr(v, "module.mass",           cfg.mass);
    cfg.boundingRadius = numberOr(v, "module.boundingRadius", cfg.boundingRadius);
    cfg.linearDamping  = numberOr(v, "module.linearDamping",  cfg.linearDamping);
    cfg.angularDamping = numberOr(v, "module.angularDamping", cfg.angularDamping);
    cfg.moduleMaterial = stringOr(v, "module.contactMaterial",cfg.moduleMaterial);
    cfg.holdWhenIdle   = boolOr  (v, "module.holdWhenIdle",    cfg.holdWhenIdle);

    cfg.coulombFriction    = numberOr(v, "contact.coulombFriction",    cfg.coulombFriction);
    cfg.bounce             = numberOr(v, "contact.bounce",             cfg.bounce);
    cfg.bounceVelocity     = numberOr(v, "contact.bounceVelocity",     cfg.bounceVelocity);
    cfg.rollingFriction    = numberOr(v, "contact.rollingFriction",    cfg.rollingFriction);
    cfg.forceDependentSlip = numberOr(v, "contact.forceDependentSlip", cfg.forceDependentSlip);
    cfg.softCFM            = numberOr(v, "contact.softCFM",            cfg.softCFM);
    cfg.softERP            = numberOr(v, "contact.softERP",            cfg.softERP);
    cfg.maxContactJoints   = (int)numberOr(v, "contact.maxContactJoints", cfg.maxContactJoints);

    cfg.bondsEnabled      = boolOr  (v, "bonds.enabled",           cfg.bondsEnabled);
    cfg.tensileStrength   = numberOr(v, "bonds.tensileStrength",   cfg.tensileStrength);
    cfg.shearStrength     = numberOr(v, "bonds.shearStrength",     cfg.shearStrength);
    cfg.distanceTolerance = numberOr(v, "bonds.distanceTolerance", cfg.distanceTolerance);
    cfg.axisTolerance     = numberOr(v, "bonds.axisTolerance",     cfg.axisTolerance);
    cfg.rotationTolerance = numberOr(v, "bonds.rotationTolerance", cfg.rotationTolerance);
    cfg.numberOfRotations = (int)numberOr(v, "bonds.numberOfRotations", cfg.numberOfRotations);

    cfg.scenarioType = stringOr(v, "scenario.type", cfg.scenarioType);
    cfg.cantBaseW   = (int)numberOr(v, "scenario.baseW",     cfg.cantBaseW);
    cfg.cantBaseD   = (int)numberOr(v, "scenario.baseD",     cfg.cantBaseD);
    cfg.cantBaseH   = (int)numberOr(v, "scenario.baseH",     cfg.cantBaseH);
    cfg.cantArmLen  = (int)numberOr(v, "scenario.armLength", cfg.cantArmLen);
    cfg.cantX0      = (int)numberOr(v, "scenario.x0",        cfg.cantX0);
    cfg.cantY0      = (int)numberOr(v, "scenario.y0",        cfg.cantY0);

    cfg.movingCatomRadius = (int)numberOr(v, "algorithm.movingCatomRadius", cfg.movingCatomRadius);

    cfg.usPerStep = (long long)numberOr(v, "playback.usPerStep", (double)cfg.usPerStep);
    cfg.tracePath = stringOr(v, "playback.trace", cfg.tracePath);

    cfg.analystName         = stringOr(v, "analysis.analyst",     cfg.analystName);
    cfg.analysisEveryNSteps = (int)numberOr(v, "analysis.everyNSteps", cfg.analysisEveryNSteps);
    cfg.placedToleranceM    = numberOr(v, "analysis.placedTolerance", cfg.placedToleranceM);
    cfg.metricsCsv          = stringOr(v, "analysis.metricsCsv",  cfg.metricsCsv);
    cfg.colorByRisk         = boolOr  (v, "analysis.colorByRisk",  cfg.colorByRisk);

    cfg.outProto       = stringOr(v, "output.proto",      cfg.outProto);
    cfg.outWorld       = stringOr(v, "output.world",      cfg.outWorld);
    cfg.protoName      = stringOr(v, "output.protoName",  cfg.protoName);
    cfg.controllerName = stringOr(v, "output.controller", cfg.controllerName);

    // ---- validation ----
    // Fatal: values that would produce a world Webots cannot simulate.
    std::ostringstream fatal;
    if (cfg.mass <= 0.0)           fatal << "module.mass must be > 0 (got " << cfg.mass << "); ";
    if (cfg.boundingRadius <= 0.0) fatal << "module.boundingRadius must be > 0 (got " << cfg.boundingRadius << "); ";
    if (cfg.basicTimeStep <= 0.0)  fatal << "world.basicTimeStep must be > 0 (got " << cfg.basicTimeStep << "); ";
    if (!fatal.str().empty()) { res.error = "scene.json: " + fatal.str(); return res; }

    // Non-fatal: clamp and say so.
    if (cfg.coulombFriction < 0.0)   { res.warnings.push_back("contact.coulombFriction < 0, clamped to 0"); cfg.coulombFriction = 0.0; }
    if (cfg.rollingFriction < 0.0)   { res.warnings.push_back("contact.rollingFriction < 0, clamped to 0"); cfg.rollingFriction = 0.0; }
    if (cfg.maxContactJoints < 1)    { res.warnings.push_back("contact.maxContactJoints < 1, clamped to 1"); cfg.maxContactJoints = 1; }
    if (cfg.gravity < 0.0)           { res.warnings.push_back("world.gravity is a magnitude along the down axis; negative value points it up"); }
    if (cfg.scenarioType != "walk" && cfg.scenarioType != "cantilever")
        res.warnings.push_back("scenario.type \"" + cfg.scenarioType + "\" unknown; using \"walk\"");
    if (cfg.cantArmLen < 0) { res.warnings.push_back("scenario.armLength < 0, clamped to 0"); cfg.cantArmLen = 0; }
    if (cfg.cantBaseW < 1 || cfg.cantBaseD < 1 || cfg.cantBaseH < 1)
        res.warnings.push_back("scenario base dimensions must be >= 1");
    if (cfg.movingCatomRadius < 0)   { res.warnings.push_back("algorithm.movingCatomRadius < 0, clamped to 0"); cfg.movingCatomRadius = 0; }
    if (cfg.movingCatomRadius == 1)  { res.warnings.push_back("algorithm.movingCatomRadius = 1 is known to livelock this scenario (use 0 or 2)"); }
    if (cfg.bounce < 0.0)            { res.warnings.push_back("contact.bounce < 0, clamped to 0"); cfg.bounce = 0.0; }
    if (cfg.bounce > 1.0)            { res.warnings.push_back("contact.bounce > 1, clamped to 1"); cfg.bounce = 1.0; }
    if (cfg.analysisEveryNSteps < 1) { res.warnings.push_back("analysis.everyNSteps < 1, clamped to 1"); cfg.analysisEveryNSteps = 1; }
    if (cfg.usPerStep < 1)           { res.warnings.push_back("playback.usPerStep < 1, clamped to 1"); cfg.usPerStep = 1; }
    if (cfg.placedToleranceM <= 0.0) { res.warnings.push_back("analysis.placedTolerance <= 0, reset to 0.02"); cfg.placedToleranceM = 0.02; }
    if (cfg.linearDamping  < 0.0 || cfg.linearDamping  > 1.0) { res.warnings.push_back("module.linearDamping outside [0,1]"); }
    if (cfg.angularDamping < 0.0 || cfg.angularDamping > 1.0) { res.warnings.push_back("module.angularDamping outside [0,1]"); }

    res.ok = true;
    return res;
}

// Same, reading from disk. A missing file is NOT an error: defaults apply and a
// warning says so, so the simulation still runs out of the box.
inline SceneConfigLoad loadSceneConfig(const std::string& path, SceneConfig& cfg) {
    std::ifstream f(path.c_str());
    if (!f.is_open()) {
        SceneConfigLoad res;
        res.ok = true;
        res.warnings.push_back("could not open " + path + " - using built-in defaults");
        return res;
    }
    std::ostringstream ss;
    ss << f.rdbuf();
    return parseSceneConfig(ss.str(), cfg);
}

// Locate a named config from either the project root or a controller working
// directory (Webots runs the controller from controllers/<name>/). Returns ""
// when nothing is found.
inline std::string findConfigNamed(const std::string& name) {
    const std::string prefixes[] = { "", "../", "../../", "../../../" };
    for (const std::string& p : prefixes) {
        std::string cand = p + name;
        std::ifstream probe(cand.c_str());
        if (probe.is_open()) return cand;
    }
    return "";
}
inline std::string findSceneConfigPath() { return findConfigNamed("scene.json"); }

#endif // SCENE_CONFIG_HPP
