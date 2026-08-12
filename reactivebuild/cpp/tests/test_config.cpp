// test_config.cpp -- Phase 0 tests for config.hpp (mirrors reference_python test_config.py).
#include "reactivebuild/config.hpp"
#include "rb_test.hpp"

using namespace rb;
using rbtest::check;
using rbtest::checkNear;

int main() {
    // --- paper constants are exactly the published values ---
    {
        FEMParams fem;
        checkNear(fem.k_in_sphere, 5e10, "k_in_sphere", 1.0);
        checkNear(fem.k_robot_structure, 2e9, "k_robot_structure", 1.0);
        checkNear(fem.k_connection, 1e10, "k_connection", 1.0);
        checkNear(fem.sphere_node_load, 0.25, "sphere_node_load");
        check(fem.nodes_per_sphere == 4, "nodes_per_sphere == 4");
        checkNear(fem.sphere_weight(), 1.0, "sphere_weight == 1.0");
        checkNear(fem.contact_stress_radius, 0.5, "contact radius 0.5");
        check(fem.length_normalized, "length_normalized default true");
    }
    {
        RobotParams r;
        check(r.num_spheres == 3, "3 spheres");
        checkNear(r.sphere_radius, 1.0, "radius 1.0");
        checkNear(r.robot_weight(), 3.0, "robot weight 3.0");
    }
    {
        AlgorithmParams a;
        checkNear(a.F, 10.0, "default F");
        checkNear(a.B, 3.0, "default B");
        check(a.J == 5, "default J");
    }
    check(MATURATION_N == 25, "maturation N=25");
    check(N_RUNS == 100 && N_ROBOTS == 100, "100 runs x 100 robots");
    check(F_SWEEP[0] == 1.0 && F_SWEEP[3] == 25.0, "F sweep endpoints");
    check(B_SWEEP[0] == 1.5 && B_SWEEP[3] == 10.0, "B sweep endpoints");
    check(J_SWEEP[0] == 1 && J_SWEEP[3] == 1000, "J sweep endpoints");
    check(BRIDGE_GAPS[0] == 20.0 && BRIDGE_GAPS[2] == 30.0, "bridge gaps");

    // --- recruit-value formula: min(floor(B^(sensed/F - 1)), J) ---
    {
        AlgorithmParams a{5.0, 3.0, 5};
        check(a.recruit_value(0.0) == 0, "recruit 0 well below F");
        check(a.recruit_value(2.5) == 0, "recruit 0 below F (3^-0.5 floors to 0)");
        check(a.recruit_value(5.0) == 1, "recruit begins at F (B^0=1)");
        check(a.recruit_value(10.0) == 3, "recruit exponential: 3^1=3");
        check(a.recruit_value(15.0) == 5, "recruit saturates at J (3^2=9 -> 5)");
        check(a.recruit_value(1e6) == 5, "recruit no overflow, caps at J");
        // monotonic non-decreasing
        bool mono = true;
        int prev = 0;
        for (int s = 0; s < 40; ++s) {
            int v = AlgorithmParams{2.5, 3.0, 5}.recruit_value((double)s);
            if (v < prev) mono = false;
            prev = v;
        }
        check(mono, "recruit monotonic non-decreasing");
    }

    // --- presets encode the four experiments ---
    {
        Params p = tower();
        check(p.experiment.type == ExperimentType::TOWER, "tower type");
        check(p.experiment.goal_offset.isApprox(Vec3(0, 0, 65)), "tower goal 65 up");
        check(p.experiment.goal_reference == GoalReference::BASE, "tower goal ref base");
    }
    {
        Params p = chain();
        check(p.experiment.type == ExperimentType::CHAIN, "chain type");
        check(p.experiment.goal_offset.isApprox(Vec3(0, 0, -600)), "chain goal 600 down");
        check(p.experiment.goal_reference == GoalReference::EDGE, "chain goal ref edge");
    }
    {
        Params p = cantilever();
        check(p.experiment.type == ExperimentType::CANTILEVER, "cantilever type");
        check(p.experiment.goal_offset.isApprox(Vec3(45, 0, 10)), "cantilever goal 45/0/10");
    }
    {
        Params p = bridge(25.0);
        check(p.experiment.type == ExperimentType::BRIDGE, "bridge type");
        checkNear(p.experiment.gap_width, 25.0, "bridge gap 25");
        check(p.experiment.n_cap == 200, "bridge n_cap 200");
        check(p.experiment.success_threshold == 100, "bridge success threshold 100");
        check(p.experiment.is_bridge(), "is_bridge true");
    }
    check(preset("tower").experiment.type == ExperimentType::TOWER, "preset dispatch tower");
    check(preset("bridge").experiment.type == ExperimentType::BRIDGE, "preset dispatch bridge");
    {
        bool threw = false;
        try { preset("nope"); } catch (const std::invalid_argument&) { threw = true; }
        check(threw, "preset unknown name throws");
    }

    // describe() smoke
    check(tower().describe().find("ReactiveBuild config") != std::string::npos,
          "describe has header");
    check(tower().describe().find("F=10") != std::string::npos, "describe has F=10");

    return rbtest::summary("test_config");
}
