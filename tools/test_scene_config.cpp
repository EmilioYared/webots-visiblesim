// =============================================================================
// test_scene_config.cpp — unit tests for scene_config.hpp.
// Covers: defaults, partial override, full override, typo detection,
// validation (fatal vs clamped), and the real shipped scene.json.
// =============================================================================
#include "../controllers/catom3d_controller/scene_config.hpp"
#include <iostream>
#include <cmath>

static int g_fail = 0, g_pass = 0;

static void check(bool cond, const std::string& what) {
    if (cond) ++g_pass;
    else { ++g_fail; std::cout << "  FAIL: " << what << "\n"; }
}
static void checkNear(double got, double want, const std::string& what) {
    bool ok = std::fabs(got - want) < 1e-9;
    if (!ok) std::cout << "  (got " << got << ", want " << want << ")\n";
    check(ok, what);
}
static bool hasWarning(const SceneConfigLoad& r, const std::string& needle) {
    for (size_t i = 0; i < r.warnings.size(); ++i)
        if (r.warnings[i].find(needle) != std::string::npos) return true;
    return false;
}

int main() {
    // ---- empty document keeps every default ----
    {
        SceneConfig c;
        SceneConfigLoad r = parseSceneConfig("{}", c);
        check(r.ok, "empty object parses: " + r.error);
        check(r.warnings.empty(), "no warnings for empty config");
        checkNear(c.mass, 0.1,           "default mass");
        checkNear(c.boundingRadius, 0.05,"default radius");
        checkNear(c.basicTimeStep, 8.0,  "default timestep");
        checkNear(c.gravity, 9.81,       "default gravity magnitude");
        checkNear(c.worldCFM, 1e-5,      "default world CFM");
        checkNear(c.worldERP, 0.2,       "default world ERP");
        checkNear(c.physicsDisableTime, 1.0, "default physicsDisableTime");
        checkNear(c.rollingFriction, 0.05,   "default rollingFriction");
        check(c.maxContactJoints == 10,      "default maxContactJoints");
        check(c.bondsEnabled,            "bonds default on");
        check(c.physicsEnabled,          "physics default on");
        check(c.usPerStep == 10000,      "default usPerStep");
        check(c.analystName == "r0c0",   "default analyst");
        check(c.movingCatomRadius == 2,  "default movingCatomRadius");
        check(c.scenarioType == "walk",  "default scenario is walk");
        check(!c.colorByRisk,            "colorByRisk default off");
        check(c.holdWhenIdle,            "holdWhenIdle default on");
    }

    // ---- partial override touches only what it names ----
    {
        SceneConfig c;
        SceneConfigLoad r = parseSceneConfig("{\"module\":{\"mass\":0.25}}", c);
        check(r.ok, "partial parses: " + r.error);
        checkNear(c.mass, 0.25,           "mass overridden");
        checkNear(c.boundingRadius, 0.05, "radius still default");
        checkNear(c.coulombFriction, 0.8, "friction still default");
    }

    // ---- full override: every field actually lands ----
    {
        SceneConfig c;
        const char* full =
        "{\"world\":{\"title\":\"T\",\"basicTimeStep\":4,\"gravity\":1.62,\"CFM\":0.002,"
        "           \"ERP\":0.5,\"physicsDisableTime\":2.5,"
        "           \"physicsDisableLinearThreshold\":0.03,"
        "           \"physicsDisableAngularThreshold\":0.04},"
        " \"floor\":{\"enabled\":false,\"autoFit\":false,\"margin\":0.5,\"thickness\":0.02,"
        "           \"size\":[2,3,4],\"center\":[5,6,7],\"topZAuto\":false,\"topZ\":0.9,"
        "           \"contactMaterial\":\"fm\"},"
        " \"module\":{\"physicsEnabled\":false,\"mass\":0.3,\"boundingRadius\":0.06,"
        "            \"linearDamping\":0.4,\"angularDamping\":0.5,\"contactMaterial\":\"mm\","
        "            \"holdWhenIdle\":false},"
        " \"contact\":{\"coulombFriction\":1.2,\"bounce\":0.3,\"bounceVelocity\":0.02,"
        "             \"rollingFriction\":0.11,\"forceDependentSlip\":0.22,"
        "             \"softCFM\":0.001,\"softERP\":0.4,\"maxContactJoints\":6},"
        " \"bonds\":{\"enabled\":false,\"tensileStrength\":-1,\"shearStrength\":-1,"
        "           \"distanceTolerance\":0.03,\"axisTolerance\":0.2,"
        "           \"rotationTolerance\":0.1,\"numberOfRotations\":4},"
        " \"scenario\":{\"type\":\"cantilever\",\"baseW\":3,\"baseD\":2,\"baseH\":4,"
        "              \"armLength\":6,\"x0\":2,\"y0\":1},"
        " \"algorithm\":{\"movingCatomRadius\":0},"
        " \"playback\":{\"usPerStep\":5000,\"trace\":\"t.csv\"},"
        " \"analysis\":{\"analyst\":\"r1c1\",\"everyNSteps\":16,\"placedTolerance\":0.01,"
        "              \"metricsCsv\":\"m.csv\",\"colorByRisk\":true},"
        " \"output\":{\"proto\":\"p.proto\",\"world\":\"w.wbt\",\"protoName\":\"PN\","
        "            \"controller\":\"ctrl\"}}";
        SceneConfigLoad r = parseSceneConfig(full, c);
        check(r.ok, "full parses: " + r.error);
        check(r.warnings.empty(), "full config produces no warnings");

        check(c.title == "T",                "world.title");
        checkNear(c.basicTimeStep, 4,        "world.basicTimeStep");
        checkNear(c.gravity, 1.62,           "world.gravity");
        checkNear(c.worldCFM, 0.002,         "world.CFM");
        checkNear(c.worldERP, 0.5,           "world.ERP");
        checkNear(c.physicsDisableTime, 2.5, "world.physicsDisableTime");
        checkNear(c.physicsDisableLinearThreshold, 0.03,  "world.physicsDisableLinearThreshold");
        checkNear(c.physicsDisableAngularThreshold, 0.04, "world.physicsDisableAngularThreshold");
        check(!c.floorEnabled,               "floor.enabled");
        check(!c.floorAutoFit,               "floor.autoFit");
        checkNear(c.floorMargin, 0.5,        "floor.margin");
        checkNear(c.floorThickness, 0.02,    "floor.thickness");
        checkNear(c.floorSize[1], 3,         "floor.size");
        checkNear(c.floorCenter[2], 7,       "floor.center");
        check(!c.floorTopZAuto,              "floor.topZAuto");
        checkNear(c.floorTopZ, 0.9,          "floor.topZ");
        check(c.floorMaterial == "fm",       "floor.contactMaterial");
        check(!c.physicsEnabled,             "module.physicsEnabled");
        checkNear(c.mass, 0.3,               "module.mass");
        checkNear(c.boundingRadius, 0.06,    "module.boundingRadius");
        checkNear(c.linearDamping, 0.4,      "module.linearDamping");
        checkNear(c.angularDamping, 0.5,     "module.angularDamping");
        check(c.moduleMaterial == "mm",      "module.contactMaterial");
        check(!c.holdWhenIdle,               "module.holdWhenIdle");
        checkNear(c.coulombFriction, 1.2,    "contact.coulombFriction");
        checkNear(c.bounce, 0.3,             "contact.bounce");
        checkNear(c.bounceVelocity, 0.02,    "contact.bounceVelocity");
        checkNear(c.softCFM, 0.001,          "contact.softCFM");
        checkNear(c.softERP, 0.4,            "contact.softERP");
        checkNear(c.rollingFriction, 0.11,   "contact.rollingFriction");
        checkNear(c.forceDependentSlip, 0.22,"contact.forceDependentSlip");
        check(c.maxContactJoints == 6,       "contact.maxContactJoints");
        check(!c.bondsEnabled,               "bonds.enabled");
        checkNear(c.tensileStrength, -1,     "bonds.tensileStrength (-1 = unbreakable)");
        checkNear(c.shearStrength, -1,       "bonds.shearStrength");
        checkNear(c.distanceTolerance, 0.03, "bonds.distanceTolerance");
        checkNear(c.axisTolerance, 0.2,      "bonds.axisTolerance");
        checkNear(c.rotationTolerance, 0.1,  "bonds.rotationTolerance");
        check(c.numberOfRotations == 4,      "bonds.numberOfRotations");
        check(c.scenarioType == "cantilever","scenario.type");
        check(c.cantBaseW == 3,              "scenario.baseW");
        check(c.cantBaseD == 2,              "scenario.baseD");
        check(c.cantBaseH == 4,              "scenario.baseH");
        check(c.cantArmLen == 6,             "scenario.armLength");
        check(c.cantX0 == 2,                 "scenario.x0");
        check(c.cantY0 == 1,                 "scenario.y0");
        check(c.colorByRisk,                 "analysis.colorByRisk");
        check(c.movingCatomRadius == 0,      "algorithm.movingCatomRadius");
        check(c.usPerStep == 5000,           "playback.usPerStep");
        check(c.tracePath == "t.csv",        "playback.trace");
        check(c.analystName == "r1c1",       "analysis.analyst");
        check(c.analysisEveryNSteps == 16,   "analysis.everyNSteps");
        checkNear(c.placedToleranceM, 0.01,  "analysis.placedTolerance");
        check(c.metricsCsv == "m.csv",       "analysis.metricsCsv");
        check(c.outProto == "p.proto",       "output.proto");
        check(c.outWorld == "w.wbt",         "output.world");
        check(c.protoName == "PN",           "output.protoName");
        check(c.controllerName == "ctrl",    "output.controller");
    }

    // ---- typos are surfaced, not swallowed ----
    {
        SceneConfig c;
        SceneConfigLoad r = parseSceneConfig(
            "{\"module\":{\"masss\":0.5},\"bonds\":{\"tensile\":3},\"nope\":{\"x\":1}}", c);
        check(r.ok, "typo config still loads");
        check(hasWarning(r, "module.masss"),   "warns on module.masss");
        check(hasWarning(r, "bonds.tensile"),  "warns on bonds.tensile");
        check(hasWarning(r, "nope.x"),         "warns on unknown section");
        checkNear(c.mass, 0.1, "typo did not change mass");
    }

    // ---- gravity given as a vector: Webots takes a scalar, so warn loudly ----
    {
        SceneConfig c;
        SceneConfigLoad r = parseSceneConfig("{\"world\":{\"gravity\":[0,0,-9.81]}}", c);
        check(r.ok, "vector gravity still loads");
        check(hasWarning(r, "must be a number"), "vector gravity is warned about");
        checkNear(c.gravity, 9.81, "vector gravity falls back to the default magnitude");
    }

    // ---- fatal validation ----
    {
        SceneConfig c;
        check(!parseSceneConfig("{\"module\":{\"mass\":0}}", c).ok,           "mass 0 is fatal");
        check(!parseSceneConfig("{\"module\":{\"mass\":-1}}", c).ok,          "negative mass is fatal");
        check(!parseSceneConfig("{\"module\":{\"boundingRadius\":0}}", c).ok, "radius 0 is fatal");
        check(!parseSceneConfig("{\"world\":{\"basicTimeStep\":0}}", c).ok,   "timestep 0 is fatal");
        check(!parseSceneConfig("{\"module\":", c).ok,                        "malformed JSON is fatal");
        check(!parseSceneConfig("[1,2]", c).ok,                               "non-object top level is fatal");
    }

    // ---- non-fatal clamping ----
    {
        SceneConfig c;
        SceneConfigLoad r = parseSceneConfig(
            "{\"contact\":{\"coulombFriction\":-1,\"bounce\":2},"
            " \"analysis\":{\"everyNSteps\":0,\"placedTolerance\":0},"
            " \"playback\":{\"usPerStep\":0}}", c);
        check(r.ok, "clampable values still load");
        checkNear(c.coulombFriction, 0.0,  "friction clamped to 0");
        checkNear(c.bounce, 1.0,           "bounce clamped to 1");
        check(c.analysisEveryNSteps == 1,  "everyNSteps clamped to 1");
        check(c.usPerStep == 1,            "usPerStep clamped to 1");
        checkNear(c.placedToleranceM, 0.02,"placedTolerance reset");
        check(r.warnings.size() >= 5,      "each clamp warned");
    }

    // ---- movingCatomRadius validation ----
    {
        SceneConfig c;
        SceneConfigLoad r = parseSceneConfig("{\"algorithm\":{\"movingCatomRadius\":1}}", c);
        check(r.ok, "radius 1 still loads");
        check(hasWarning(r, "livelock"), "radius 1 warns about livelock");
        SceneConfig c2;
        SceneConfigLoad r2 = parseSceneConfig("{\"algorithm\":{\"movingCatomRadius\":-3}}", c2);
        check(r2.ok, "negative radius still loads");
        check(c2.movingCatomRadius == 0, "negative radius clamped to 0");
        check(hasWarning(r2, "clamped to 0"), "negative radius warns");
    }

    // ---- a missing file is survivable, not fatal ----
    {
        SceneConfig c;
        SceneConfigLoad r = loadSceneConfig("no_such_scene_98765.json", c);
        check(r.ok, "missing file is not an error");
        check(hasWarning(r, "built-in defaults"), "missing file warns about defaults");
        checkNear(c.mass, 0.1, "defaults intact after missing file");
    }

    // ---- the real shipped scene.json must be valid and typo-free ----
    {
        SceneConfig c;
        std::string path = findSceneConfigPath();
        check(!path.empty(), "findSceneConfigPath locates scene.json");
        SceneConfigLoad r = loadSceneConfig(path, c);
        check(r.ok, "shipped scene.json is valid: " + r.error);
        for (size_t i = 0; i < r.warnings.size(); ++i)
            std::cout << "  shipped scene.json warning: " << r.warnings[i] << "\n";
        check(r.warnings.empty(), "shipped scene.json has no unknown keys");
        check(c.physicsEnabled, "shipped config enables physics");
        check(c.bondsEnabled,   "shipped config enables bonds");
    }

    std::cout << "test_scene_config: " << g_pass << " passed, " << g_fail << " failed\n";
    return g_fail == 0 ? 0 : 1;
}
