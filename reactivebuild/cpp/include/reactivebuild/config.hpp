// config.hpp -- ReactiveBuild parameters (Phase 0, C++ port).
//
// Every constant traces to Swissler & Rubenstein (2022), "ReactiveBuild:
// Environment-Adaptive Self-Assembly of Amorphous Structures". Section markers refer
// to that paper; see REACTIVEBUILD_PLAN.md section 1 for the decoded fidelity spec.
//
// This is the canonical C++ implementation. The frozen Python package under
// reactivebuild/*.py is retained only as an independent cross-check oracle.
#pragma once

#include <Eigen/Dense>

#include <array>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <string>

namespace rb {

using Vec3 = Eigen::Vector3d;

// --- Algorithm parameter sweeps (section 4) ---------------------------------
inline const std::array<double, 4> F_SWEEP = {1.0, 2.5, 5.0, 25.0};
inline const std::array<double, 4> B_SWEEP = {1.5, 3.0, 6.0, 10.0};
inline const std::array<int, 4> J_SWEEP = {1, 2, 5, 1000};

// Defaults: B, J fixed across the F-sweeps; F defaults to the growth-illustration value.
constexpr double DEFAULT_F = 10.0;
constexpr double DEFAULT_B = 3.0;
constexpr int DEFAULT_J = 5;

// --- FEM model (section 3, Fig. 3) ------------------------------------------
constexpr double STIFFNESS_IN_SPHERE = 5e10;
constexpr double STIFFNESS_ROBOT_STRUCTURE = 2e9;
constexpr double STIFFNESS_CONNECTION = 1e10;
constexpr double SPHERE_NODE_LOAD = 0.25;   // per sphere node; 4 nodes -> 1.0/sphere
constexpr int NODES_PER_SPHERE = 4;
constexpr int NODES_PER_CENTER = 4;
constexpr double CONTACT_STRESS_RADIUS = 0.5;

// --- Robot geometry (section 2, section 4) ----------------------------------
constexpr int NUM_SPHERES = 3;
constexpr double SPHERE_RADIUS = 1.0;
constexpr double TETRA_SCALE = 0.1;         // intra-sphere tetra circumradius / radius

// --- Experiment framing (section 4) -----------------------------------------
constexpr int N_RUNS = 100;
constexpr int N_ROBOTS = 100;
constexpr int MATURATION_N = 25;
inline const std::array<double, 3> BRIDGE_GAPS = {20.0, 25.0, 30.0};
constexpr int BRIDGE_N_CAP = 200;
constexpr int BRIDGE_SUCCESS_THRESHOLD = 100;

enum class ExperimentType { TOWER, CHAIN, CANTILEVER, BRIDGE };
enum class GoalReference { BASE, EDGE };

inline const char* to_string(ExperimentType t) {
    switch (t) {
        case ExperimentType::TOWER: return "tower";
        case ExperimentType::CHAIN: return "chain";
        case ExperimentType::CANTILEVER: return "cantilever";
        case ExperimentType::BRIDGE: return "bridge";
    }
    return "?";
}
inline const char* to_string(GoalReference r) {
    return r == GoalReference::BASE ? "base" : "edge";
}

// --- parameter groups -------------------------------------------------------

struct AlgorithmParams {
    double F = DEFAULT_F;
    double B = DEFAULT_B;
    int J = DEFAULT_J;

    // recruit = min(floor(B^(sensed/F - 1)), J), clamped at 0.  B is an EXPONENT base
    // (section 4), not a linear multiplier. Overflow-safe: since the result caps at J, we
    // short-circuit before B^exponent can overflow.
    int recruit_value(double sensed_force) const {
        if (J <= 0) return 0;
        double exponent = sensed_force / F - 1.0;
        if (exponent > 0.0 && B > 1.0 &&
            exponent * std::log(B) >= std::log(static_cast<double>(J))) {
            return J;
        }
        double raw = std::pow(B, exponent);
        int v = static_cast<int>(std::floor(raw));
        if (v < 0) v = 0;
        if (v > J) v = J;
        return v;
    }
};

struct FEMParams {
    double k_in_sphere = STIFFNESS_IN_SPHERE;
    double k_robot_structure = STIFFNESS_ROBOT_STRUCTURE;
    double k_connection = STIFFNESS_CONNECTION;
    double sphere_node_load = SPHERE_NODE_LOAD;
    int nodes_per_sphere = NODES_PER_SPHERE;
    int nodes_per_center = NODES_PER_CENTER;
    double contact_stress_radius = CONTACT_STRESS_RADIUS;
    // Q7: treat the class stiffness as axial rigidity EA and use k=EA/L (true), or as a
    // length-independent k (false). Only rescales magnitudes; ordering is unchanged.
    bool length_normalized = true;

    double sphere_weight() const { return sphere_node_load * nodes_per_sphere; }
};

struct RobotParams {
    int num_spheres = NUM_SPHERES;
    double sphere_radius = SPHERE_RADIUS;
    double tetra_scale = TETRA_SCALE;

    // 3 spheres * 1.0 (center nodes unloaded, Q3)
    double robot_weight() const {
        return num_spheres * SPHERE_NODE_LOAD * NODES_PER_SPHERE;
    }
};

struct ExperimentParams {
    ExperimentType type = ExperimentType::TOWER;
    Vec3 goal_offset = Vec3(0.0, 0.0, 65.0);
    GoalReference goal_reference = GoalReference::BASE;
    int n_robots = N_ROBOTS;
    int n_runs = N_RUNS;
    int maturation_n = MATURATION_N;
    // bridge-only (gap_width < 0 == not a bridge)
    double gap_width = -1.0;
    int n_cap = -1;
    int success_threshold = -1;

    bool is_bridge() const { return gap_width >= 0.0; }
};

struct Params {
    AlgorithmParams algorithm;
    FEMParams fem;
    RobotParams robot;
    ExperimentParams experiment;
    Vec3 gravity = Vec3(0.0, 0.0, -1.0);
    int seed = 0;

    std::string describe() const {
        const auto& a = algorithm;
        const auto& e = experiment;
        std::ostringstream os;
        os << "ReactiveBuild config\n"
           << "  experiment : " << to_string(e.type) << "\n"
           << "  goal       : offset=(" << e.goal_offset.x() << ", " << e.goal_offset.y()
           << ", " << e.goal_offset.z() << ") from " << to_string(e.goal_reference) << "\n"
           << "  runs       : " << e.n_runs << " runs x " << e.n_robots
           << " robots (maturation N=" << e.maturation_n << ")\n";
        if (e.is_bridge()) {
            os << "  bridge     : gap=" << e.gap_width << " n_cap=" << e.n_cap
               << " success_if_N<" << e.success_threshold << "\n";
        }
        os << "  algorithm  : F=" << a.F << " B=" << a.B << " J=" << a.J << "\n"
           << "  robot      : " << robot.num_spheres << " spheres, radius "
           << robot.sphere_radius << ", weight " << robot.robot_weight() << "\n"
           << "  fem        : k(in-sphere)=" << fem.k_in_sphere
           << " k(structure)=" << fem.k_robot_structure
           << " k(connection)=" << fem.k_connection << "\n"
           << "  fem loads  : " << fem.sphere_node_load << " / sphere-node x "
           << fem.nodes_per_sphere << " nodes = " << fem.sphere_weight() << " / sphere\n"
           << "  gravity    : (" << gravity.x() << ", " << gravity.y() << ", "
           << gravity.z() << ")\n"
           << "  seed       : " << seed;
        return os.str();
    }
};

// --- presets (section 4.1 - 4.4) --------------------------------------------

inline Params tower(double F = DEFAULT_F, double B = DEFAULT_B, int J = DEFAULT_J,
                    int n_runs = N_RUNS, int n_robots = N_ROBOTS, int seed = 0) {
    Params p;
    p.algorithm = {F, B, J};
    p.experiment.type = ExperimentType::TOWER;
    p.experiment.goal_offset = Vec3(0.0, 0.0, 65.0);
    p.experiment.goal_reference = GoalReference::BASE;
    p.experiment.n_runs = n_runs;
    p.experiment.n_robots = n_robots;
    p.seed = seed;
    return p;
}

inline Params chain(double F = DEFAULT_F, double B = DEFAULT_B, int J = DEFAULT_J,
                    int n_runs = N_RUNS, int n_robots = N_ROBOTS, int seed = 0) {
    Params p;
    p.algorithm = {F, B, J};
    p.experiment.type = ExperimentType::CHAIN;
    p.experiment.goal_offset = Vec3(0.0, 0.0, -600.0);
    p.experiment.goal_reference = GoalReference::EDGE;
    p.experiment.n_runs = n_runs;
    p.experiment.n_robots = n_robots;
    p.seed = seed;
    return p;
}

inline Params cantilever(double F = DEFAULT_F, double B = DEFAULT_B, int J = DEFAULT_J,
                         int n_runs = N_RUNS, int n_robots = N_ROBOTS, int seed = 0) {
    Params p;
    p.algorithm = {F, B, J};
    p.experiment.type = ExperimentType::CANTILEVER;
    p.experiment.goal_offset = Vec3(45.0, 0.0, 10.0);
    p.experiment.goal_reference = GoalReference::EDGE;
    p.experiment.n_runs = n_runs;
    p.experiment.n_robots = n_robots;
    p.seed = seed;
    return p;
}

inline Params bridge(double gap_width = BRIDGE_GAPS[0], double F = DEFAULT_F,
                     double B = DEFAULT_B, int J = DEFAULT_J, int n_runs = N_RUNS,
                     int seed = 0) {
    Params p;
    p.algorithm = {F, B, J};
    p.experiment.type = ExperimentType::BRIDGE;
    p.experiment.goal_offset = Vec3(gap_width, 0.0, 0.0);
    p.experiment.goal_reference = GoalReference::EDGE;
    p.experiment.n_runs = n_runs;
    p.experiment.n_robots = BRIDGE_N_CAP;
    p.experiment.gap_width = gap_width;
    p.experiment.n_cap = BRIDGE_N_CAP;
    p.experiment.success_threshold = BRIDGE_SUCCESS_THRESHOLD;
    p.seed = seed;
    return p;
}

// name -> default preset ("tower"|"chain"|"cantilever"|"bridge"); throws on unknown.
inline Params preset(const std::string& name) {
    if (name == "tower") return tower();
    if (name == "chain") return chain();
    if (name == "cantilever") return cantilever();
    if (name == "bridge") return bridge();
    throw std::invalid_argument("unknown preset: " + name);
}

}  // namespace rb
