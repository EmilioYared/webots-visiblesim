// =============================================================================
// gen_world.cpp — scene.json (+ trace.csv) -> generated Webots .proto and .wbt
//
// Webots fixes most physics at world-load time: boundingObject geometry,
// ContactProperties, basicTimeStep and Connector strengths cannot be changed by
// a Supervisor once the simulation is running. So the JSON has to be baked into
// the scene description, which is what this tool does.
//
// The module layout is not hardcoded: it is derived from the VisibleSim trace,
// so the generated world always matches the algorithm being replayed.
//
//   Usage: gen_world [scene.json]
// =============================================================================

#include "../controllers/catom3d_controller/catom3d_core.hpp"
#include "../controllers/catom3d_controller/scene_config.hpp"
#include "../controllers/catom3d_controller/scenario.hpp"

#include <iostream>
#include <iomanip>
#include <algorithm>
#include <set>
#include <tuple>

using Key = std::tuple<int,int,int>;
static Key key(GridPos g) { return std::make_tuple(g.x, g.y, g.z); }

// Look for the trace next to the config, then in the controller directory
// (where Webots' working directory puts it at run time). The module LAYOUT is
// identical whether or not the arc hint has been resolved, so fall back to the
// raw trace.csv if the configured (resolved) trace does not exist yet.
static std::string findTrace(const std::string& configured) {
    std::string names[] = { configured, "trace.csv" };
    for (const std::string& nm : names) {
        std::string candidates[] = {
            nm,
            "controllers/catom3d_controller/" + nm,
            "../controllers/catom3d_controller/" + nm,
            "../" + nm
        };
        for (size_t i = 0; i < sizeof(candidates)/sizeof(candidates[0]); ++i) {
            std::ifstream probe(candidates[i].c_str());
            if (probe.is_open()) return candidates[i];
        }
    }
    return "";
}

// Deterministic, well-spread palette so neighbouring modules stay tellable apart.
static void moduleColor(int i, int n, double rgb[3]) {
    double h = (n > 0) ? (double)i / (double)n : 0.0;      // hue in [0,1)
    double s = 0.75, v = 0.90;
    double hh = h * 6.0;
    int    sect = (int)std::floor(hh) % 6;
    double f = hh - std::floor(hh);
    double p = v * (1 - s), q = v * (1 - s * f), t = v * (1 - s * (1 - f));
    switch (sect) {
        case 0: rgb[0]=v; rgb[1]=t; rgb[2]=p; break;
        case 1: rgb[0]=q; rgb[1]=v; rgb[2]=p; break;
        case 2: rgb[0]=p; rgb[1]=v; rgb[2]=t; break;
        case 3: rgb[0]=p; rgb[1]=q; rgb[2]=v; break;
        case 4: rgb[0]=t; rgb[1]=p; rgb[2]=v; break;
        default:rgb[0]=v; rgb[1]=p; rgb[2]=q; break;
    }
}

// Keep "-0" out of the generated files; it is legal VRML but reads as noise.
static double z0(double v) { return (v == 0.0) ? 0.0 : v; }

// Rotation taking Webots' connector axis (+z) onto the outward direction u.
// CONNECTOR_POS entries are already unit vectors.
static void axisAngleToward(const double u[3], double out[4]) {
    const double uz = u[2];
    if (uz > 1.0 - 1e-12)  { out[0]=0; out[1]=1; out[2]=0; out[3]=0;      return; } // +z
    if (uz < -1.0 + 1e-12) { out[0]=1; out[1]=0; out[2]=0; out[3]=M_PI;   return; } // -z
    double ax = -u[1], ay = u[0], az = 0.0;                 // z x u
    double len = std::sqrt(ax*ax + ay*ay + az*az);
    out[0] = ax/len; out[1] = ay/len; out[2] = az/len;
    out[3] = std::acos(uz);
}

int main(int argc, char** argv) {
    buildMotionLinks();

    // ---------------------------------------------------------- config ----
    SceneConfig cfg;
    std::string cfgPath = (argc > 1) ? argv[1] : findSceneConfigPath();
    if (cfgPath.empty()) {
        std::cerr << "ERROR: scene.json not found (run from the project root)\n";
        return 1;
    }
    SceneConfigLoad load = loadSceneConfig(cfgPath, cfg);
    for (size_t i = 0; i < load.warnings.size(); ++i)
        std::cout << "WARNING: " << load.warnings[i] << "\n";
    if (!load.ok) { std::cerr << "ERROR: " << load.error << "\n"; return 1; }
    std::cout << "config: " << cfgPath << "\n";
    // The controller reads this same config at runtime; pass its basename so the
    // world is self-describing (walk world -> scene.json, stress -> scene_stress.json).
    std::string cfgBase = cfgPath;
    { size_t s = cfgBase.find_last_of("/\\"); if (s != std::string::npos) cfgBase = cfgBase.substr(s+1); }

    const bool isWalk = (cfg.scenarioType == "walk");

    // -------------------------------------------------------- layout ----
    // "walk": module positions come from the VisibleSim trace. Stress
    // scenarios ("cantilever", ...): positions come from scenario.hpp.
    std::vector<GridPos> initCells;
    std::vector<GridPos> allCells;              // for the floor footprint
    if (isWalk) {
        std::string tracePath = findTrace(cfg.tracePath);
        if (tracePath.empty()) {
            std::cerr << "ERROR: trace '" << cfg.tracePath << "' not found\n";
            return 1;
        }
        std::vector<Move> moves = loadTraceFrom(tracePath.c_str(), nullptr);
        if (moves.empty()) { std::cerr << "ERROR: no moves in " << tracePath << "\n"; return 1; }
        std::set<Key> initSet;
        for (size_t i = 0; i < moves.size(); ++i) {
            if (initSet.insert(key(moves[i].init)).second) initCells.push_back(moves[i].init);
            allCells.push_back(moves[i].init);
            allCells.push_back(moves[i].to);
        }
        std::sort(initCells.begin(), initCells.end(),
                  [](GridPos a, GridPos b){
                      if (a.y != b.y) return a.y < b.y;
                      if (a.x != b.x) return a.x < b.x;
                      return a.z < b.z;
                  });
        std::cout << "trace:  " << tracePath << " (" << moves.size() << " moves, "
                  << initCells.size() << " modules)\n";
    } else {
        initCells = buildScenarioCells(cfg);
        if (initCells.empty()) {
            std::cerr << "ERROR: scenario '" << cfg.scenarioType << "' produced no cells\n";
            return 1;
        }
        allCells = initCells;
        std::cout << "scenario: " << cfg.scenarioType << " (" << initCells.size() << " modules)\n";
    }

    int minX = initCells[0].x, minY = initCells[0].y;
    for (size_t i = 0; i < initCells.size(); ++i) {
        minX = std::min(minX, initCells[i].x);
        minY = std::min(minY, initCells[i].y);
    }

    // What the structural analyst will report for the initial configuration.
    {
        StructuralReport rep = analyzeStructure(initCells);
        std::cout << "  structure: components=" << rep.components
                  << " articulation=" << rep.articulation
                  << " supported=" << rep.supported
                  << " overhanging=" << rep.overhanging
                  << " CoM-in-support=" << (rep.comInSupport?"yes":"NO") << "\n";
    }

    // ------------------------------------------------ floor placement ----
    double lo[3], hi[3];
    { double w[3]; gridToWorld(allCells[0], w);
      for (int k = 0; k < 3; ++k) lo[k] = hi[k] = w[k]; }
    for (size_t i = 1; i < allCells.size(); ++i) {
        double w[3]; gridToWorld(allCells[i], w);
        for (int k = 0; k < 3; ++k) { lo[k] = std::min(lo[k], w[k]); hi[k] = std::max(hi[k], w[k]); }
    }

    double floorSize[3], floorCenter[3];
    double topZ = cfg.floorTopZAuto ? (lo[2] - cfg.boundingRadius) : cfg.floorTopZ;
    if (cfg.floorAutoFit) {
        double pad = cfg.boundingRadius + cfg.floorMargin;
        floorSize[0] = (hi[0] - lo[0]) + 2 * pad;
        floorSize[1] = (hi[1] - lo[1]) + 2 * pad;
        floorSize[2] = cfg.floorThickness;
        floorCenter[0] = 0.5 * (lo[0] + hi[0]);
        floorCenter[1] = 0.5 * (lo[1] + hi[1]);
        floorCenter[2] = topZ - 0.5 * cfg.floorThickness;
    } else {
        for (int k = 0; k < 3; ++k) { floorSize[k] = cfg.floorSize[k]; floorCenter[k] = cfg.floorCenter[k]; }
    }

    // ------------------------------------------------------ PROTO file ----
    {
        std::ofstream p(cfg.outProto.c_str());
        if (!p.is_open()) { std::cerr << "ERROR: cannot write " << cfg.outProto << "\n"; return 1; }
        p << std::setprecision(9);
        p << "#VRML_SIM R2025a utf8\n"
             "# GENERATED FILE - do not edit.\n"
             "# Produced by tools/gen_world.cpp from " << cfgPath << ".\n"
             "# Change the physics there and re-run tools/gen_world.exe.\n\n";

        p << "PROTO " << cfg.protoName << " [\n"
             "  field SFVec3f    translation    0 0 0\n"
             "  field SFRotation rotation       0 1 0 0\n"
             "  field SFString   controller     \"" << cfg.controllerName << "\"\n"
             "  field MFString   controllerArgs []\n"
             "  field SFString   name           \"catom3d\"\n"
             "  field SFColor    color          0.2 0.4 0.8\n"
             "  field SFFloat    mass           " << cfg.mass << "\n"
             "]\n{\n"
             "  Robot {\n"
             "    translation IS translation\n"
             "    rotation    IS rotation\n"
             "    name        IS name\n"
             "    controller  IS controller\n"
             "    controllerArgs IS controllerArgs\n"
             "    supervisor  TRUE\n"
             "    contactMaterial \"" << cfg.moduleMaterial << "\"\n\n"
             "    children [\n"
             "      # OBJ is in VisibleSim units (+-5); scale to a "
          << cfg.boundingRadius << " m radius.\n"
             "      Transform {\n"
             "        scale " << (cfg.boundingRadius / 5.0) << " "
                              << (cfg.boundingRadius / 5.0) << " "
                              << (cfg.boundingRadius / 5.0) << "\n"
             "        children [\n"
             "          Shape {\n"
             "            appearance PBRAppearance {\n"
             "              baseColor IS color\n"
             "              roughness 0.4\n"
             "              metalness 0.15\n"
             "            }\n"
             "            geometry Mesh {\n"
             "              url \"meshes/catom3DV2.obj\"\n"
             "            }\n"
             "          }\n"
             "        ]\n"
             "      }\n\n"
             "      GPS {\n"
             "        name \"gps\"\n"
             "        accuracy 0.0\n"
             "      }\n\n"
             "      Emitter {\n"
             "        name \"emitter\"\n"
             "        range 1.0\n"
             "        channel 1\n"
             "      }\n\n"
             "      Receiver {\n"
             "        name \"receiver\"\n"
             "        channel 1\n"
             "      }\n";

        if (cfg.bondsEnabled) {
            p << "\n      # --- lattice bonds -------------------------------------------\n"
                 "      # One Connector per FCC connector direction, on the module\n"
                 "      # surface, +z pointing at the neighbouring cell. Adjacent\n"
                 "      # modules present exactly antiparallel, coincident connectors.\n"
                 "      # autoLock/unilateralLock are FALSE so lock state stays under\n"
                 "      # controller control: a rolling module unlocks first and so\n"
                 "      # cannot be grabbed by the modules it passes.\n"
                 "      # Bonds break above tensileStrength/shearStrength (-1 = never).\n";
            for (int i = 0; i < 12; ++i) {
                const double* u = CONNECTOR_POS[i];
                double aa[4]; axisAngleToward(u, aa);
                p << "      Connector {\n"
                     "        translation " << z0(u[0]*cfg.boundingRadius) << " "
                                            << z0(u[1]*cfg.boundingRadius) << " "
                                            << z0(u[2]*cfg.boundingRadius) << "\n"
                     "        rotation " << z0(aa[0]) << " " << z0(aa[1]) << " "
                                         << z0(aa[2]) << " " << z0(aa[3]) << "\n"
                     "        name \"con" << i << "\"\n"
                     "        model \"" << cfg.moduleMaterial << "\"\n"
                     "        type \"symmetric\"\n"
                     // Locked at world load so the initial sheet is bonded from
                     // step 0, before any controller has run.
                     "        isLocked TRUE\n"
                     "        autoLock FALSE\n"
                     "        unilateralLock FALSE\n"
                     "        unilateralUnlock TRUE\n"
                     "        snap FALSE\n"
                     "        distanceTolerance " << cfg.distanceTolerance << "\n"
                     "        axisTolerance " << cfg.axisTolerance << "\n"
                     "        rotationTolerance " << cfg.rotationTolerance << "\n"
                     "        numberOfRotations " << cfg.numberOfRotations << "\n"
                     "        tensileStrength " << cfg.tensileStrength << "\n"
                     "        shearStrength " << cfg.shearStrength << "\n"
                     "      }\n";
            }
        }

        p << "    ]\n";

        if (cfg.physicsEnabled) {
            p << "\n    boundingObject Sphere {\n"
                 "      radius " << cfg.boundingRadius << "\n"
                 "      subdivision 2\n"
                 "    }\n"
                 "    physics Physics {\n"
                 "      density -1\n"
                 "      mass IS mass\n"
                 "      damping Damping {\n"
                 "        linear " << cfg.linearDamping << "\n"
                 "        angular " << cfg.angularDamping << "\n"
                 "      }\n"
                 "    }\n";
        } else {
            p << "\n    # module.physicsEnabled = false -> kinematic module\n";
        }

        p << "  }\n}\n";
        p.close();
        std::cout << "wrote " << cfg.outProto << (cfg.bondsEnabled ? " (12 bonds/module)" : " (no bonds)") << "\n";
    }

    // ------------------------------------------------------ WORLD file ----
    {
        std::ofstream w(cfg.outWorld.c_str());
        if (!w.is_open()) { std::cerr << "ERROR: cannot write " << cfg.outWorld << "\n"; return 1; }
        w << std::setprecision(9);
        w << "#VRML_SIM R2025a utf8\n"
             "# GENERATED FILE - do not edit.\n"
             "# Produced by tools/gen_world.cpp from " << cfgPath << ".\n"
             "# Change the physics there and re-run tools/gen_world.exe.\n\n"
             "EXTERNPROTO \"https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/backgrounds/protos/TexturedBackground.proto\"\n"
             "EXTERNPROTO \"https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/backgrounds/protos/TexturedBackgroundLight.proto\"\n"
             "EXTERNPROTO \"../" << cfg.outProto << "\"\n\n";

        // WorldInfo.gravity is an SFFloat: a magnitude along the down axis
        // (-Z under the default ENU coordinate system), NOT a vector.
        w << "WorldInfo {\n"
             "  title \"" << cfg.title << "\"\n"
             "  basicTimeStep " << cfg.basicTimeStep << "\n"
             "  gravity " << cfg.gravity << "\n";
        if (cfg.physicsEnabled) {
            w << "  CFM " << cfg.worldCFM << "\n"
                 "  ERP " << cfg.worldERP << "\n"
                 "  physicsDisableTime " << cfg.physicsDisableTime << "\n"
                 "  physicsDisableLinearThreshold " << cfg.physicsDisableLinearThreshold << "\n"
                 "  physicsDisableAngularThreshold " << cfg.physicsDisableAngularThreshold << "\n";
            const char* pairs[2][2] = {
                { cfg.moduleMaterial.c_str(), cfg.moduleMaterial.c_str() },
                { cfg.moduleMaterial.c_str(), cfg.floorMaterial.c_str()  }
            };
            w << "  contactProperties [\n";
            for (int i = 0; i < 2; ++i) {
                w << "    ContactProperties {\n"
                     "      material1 \"" << pairs[i][0] << "\"\n"
                     "      material2 \"" << pairs[i][1] << "\"\n"
                     "      coulombFriction [ " << cfg.coulombFriction << " ]\n"
                     // Spherical bounding objects roll forever without this.
                     "      rollingFriction " << cfg.rollingFriction << " "
                                              << cfg.rollingFriction << " "
                                              << cfg.rollingFriction << "\n"
                     "      bounce " << cfg.bounce << "\n"
                     "      bounceVelocity " << cfg.bounceVelocity << "\n"
                     "      forceDependentSlip [ " << cfg.forceDependentSlip << " ]\n"
                     "      softCFM " << cfg.softCFM << "\n"
                     "      softERP " << cfg.softERP << "\n"
                     "      maxContactJoints " << cfg.maxContactJoints << "\n"
                     "    }\n";
            }
            w << "  ]\n";
        }
        w << "}\n";

        // Frame the structure from the same angle as the hand-built world.
        w << "Viewpoint {\n"
             "  orientation -0.08047166804442894 0.3531457038730418 0.9321010795392102 1.2961945422273482\n"
             "  position " << (0.5*(lo[0]+hi[0]) - 0.15) << " "
                           << (0.5*(lo[1]+hi[1]) - 1.78) << " "
                           << (0.5*(lo[2]+hi[2]) + 0.98) << "\n"
             "}\n"
             "TexturedBackground {\n}\n"
             "TexturedBackgroundLight {\n}\n";

        if (cfg.floorEnabled) {
            w << "Solid {\n"
                 "  translation " << floorCenter[0] << " " << floorCenter[1] << " " << floorCenter[2] << "\n"
                 "  children [\n"
                 "    Shape {\n"
                 "      appearance PBRAppearance {\n"
                 "        baseColor 0.6 0.6 0.6\n"
                 "        roughness 1\n"
                 "        metalness 0\n"
                 "      }\n"
                 "      geometry Box {\n"
                 "        size " << floorSize[0] << " " << floorSize[1] << " " << floorSize[2] << "\n"
                 "      }\n"
                 "    }\n"
                 "  ]\n"
                 "  boundingObject Box {\n"
                 "    size " << floorSize[0] << " " << floorSize[1] << " " << floorSize[2] << "\n"
                 "  }\n"
                 "  contactMaterial \"" << cfg.floorMaterial << "\"\n"
                 "  name \"floor\"\n"
                 "}\n";
        }

        // In the cantilever, the pedestal is a held anchor and the arm is
        // dynamic (it sags and eventually breaks free). The arm cells follow the
        // pedestal cells in buildCantilever's output.
        const int pedestalCount = cfg.cantBaseW * cfg.cantBaseD * cfg.cantBaseH;
        for (size_t i = 0; i < initCells.size(); ++i) {
            double pos[3]; gridToWorld(initCells[i], pos);
            double rgb[3]; moduleColor((int)i, (int)initCells.size(), rgb);
            // Walk cells are one-per-(x,y) so r#c# is unique and readable; stress
            // scenarios stack cells in z, so name them by index instead.
            std::string nm = isWalk
                ? ("r" + std::to_string(initCells[i].y - minY) + "c" + std::to_string(initCells[i].x - minX))
                : ("m" + std::to_string(i));
            bool dynamic = (cfg.scenarioType == "cantilever" && (int)i >= pedestalCount);
            w << cfg.protoName << " {\n"
                 "  translation " << pos[0] << " " << pos[1] << " " << pos[2] << "\n"
                 "  rotation 0 1 0 0\n"
                 "  name \"" << nm << "\"\n"
                 "  controllerArgs [ \"" << cfgBase << "\"" << (dynamic ? ", \"dynamic\"" : "") << " ]\n"
                 "  color " << rgb[0] << " " << rgb[1] << " " << rgb[2] << "\n"
                 "  mass " << cfg.mass << "\n"
                 "}\n";
        }
        w.close();
        std::cout << "wrote " << cfg.outWorld << "\n";
    }

    std::cout << "  modules       " << initCells.size()
              << " (physics " << (cfg.physicsEnabled ? "on" : "off")
              << ", mass " << cfg.mass << " kg, r " << cfg.boundingRadius << " m)\n";
    std::cout << "  bonds         " << (cfg.bondsEnabled
              ? ("on, tensile " + std::to_string(cfg.tensileStrength)
                 + " N / shear " + std::to_string(cfg.shearStrength) + " N")
              : std::string("off")) << "\n";
    if (cfg.floorEnabled)
        std::cout << "  floor         " << floorSize[0] << " x " << floorSize[1]
                  << " m, top at z=" << topZ << (cfg.floorAutoFit ? " (auto-fitted)" : "") << "\n";
    std::cout << "  analyst       " << cfg.analystName << "\n";
    return 0;
}
