// test_simulator.cpp -- Phase 6: the end-to-end add-one-robot-at-a-time loop.
// Confirms a tower run completes, metrics are monotonic where expected, recruitment engages,
// output serializes to CSV, and the loop is deterministic under a fixed seed.
#include "reactivebuild/simulator.hpp"
#include "rb_test.hpp"

#include <sstream>

using namespace rb;
using rbtest::check;
using rbtest::checkClose;

static int countLines(const std::string& s) {
    int n = 0;
    for (char c : s) if (c == '\n') ++n;
    return n;
}

int main() {
    const int N = 30;

    // A tower run completes and produces one metrics row per robot.
    {
        Params p = tower(DEFAULT_F, DEFAULT_B, DEFAULT_J, 1, N, 0);
        Simulator sim(p, 2024);
        SimResult r = sim.run(N);
        check(static_cast<int>(r.robots.size()) == N, "all N robots placed on the plane");
        check(static_cast<int>(r.steps.size()) == N, "one metrics row per robot");
        for (int i = 0; i < N; ++i) check(r.steps[i].n == i + 1, "step n counts up");

        // Monotonic where expected: the structure only grows, so height and contact count
        // never decrease.
        bool h_mono = true, c_mono = true;
        for (int i = 1; i < N; ++i) {
            if (r.steps[i].height < r.steps[i - 1].height - 1e-9) h_mono = false;
            if (r.steps[i].n_contacts < r.steps[i - 1].n_contacts) c_mono = false;
        }
        check(h_mono, "height is non-decreasing");
        check(c_mono, "contact count is non-decreasing");

        // Sanity: a tower rises and develops real forces (amorphous packing).
        check(r.steps.back().height > 3.0 * p.robot.sphere_radius, "tower rises above the base");
        check(r.steps.back().max_sensed > 1.0, "structure develops real sensed forces");
        check(r.steps.back().n_anchors >= 3, "structure stays anchored to the plane");
        check(r.steps[0].max_sensed < 1e-3, "first robot alone senses ~0 (physics insight)");
    }

    // CSV serialization: header + one row per step / per sphere.
    {
        Params p = tower(DEFAULT_F, DEFAULT_B, DEFAULT_J, 1, N, 0);
        Simulator sim(p, 7);
        SimResult r = sim.run(N);

        std::ostringstream mcsv;
        write_metrics_csv(mcsv, r);
        std::string ms = mcsv.str();
        check(ms.rfind("n,height,peak_stress", 0) == 0, "metrics CSV has header");
        check(countLines(ms) == N + 1, "metrics CSV: header + N rows");

        std::ostringstream pcsv;
        write_positions_csv(pcsv, r);
        std::string ps = pcsv.str();
        check(ps.rfind("robot,sphere,x,y,z", 0) == 0, "positions CSV has header");
        check(countLines(ps) == 3 * N + 1, "positions CSV: header + 3 spheres * N robots");
    }

    // Recruitment engages at a low threshold force.
    {
        Params p = tower(2.0, DEFAULT_B, DEFAULT_J, 1, N, 0);  // F=2
        Simulator sim(p, 2024);
        SimResult r = sim.run(N);
        int max_recruit = 0;
        bool any_recruited = false;
        for (const auto& m : r.steps) {
            max_recruit = std::max(max_recruit, m.max_recruit);
            any_recruited = any_recruited || m.recruited;
        }
        check(max_recruit > 0, "low F -> recruitment values propagate through the structure");
        check(max_recruit <= p.algorithm.J, "recruit values never exceed J");
        check(any_recruited, "low F -> at least one robot joins via recruitment");
    }

    // Determinism: identical seed -> identical structure and metrics.
    {
        Params p = tower(DEFAULT_F, DEFAULT_B, DEFAULT_J, 1, N, 0);
        Simulator a(p, 99), b(p, 99);
        SimResult ra = a.run(N), rb = b.run(N);
        bool same_pos = ra.robots.size() == rb.robots.size();
        for (size_t i = 0; same_pos && i < ra.robots.size(); ++i)
            same_pos = (ra.robots[i].position - rb.robots[i].position).norm() < 1e-12;
        check(same_pos, "same seed -> identical final positions");
        bool same_metrics = true;
        for (int i = 0; i < N; ++i)
            same_metrics = same_metrics &&
                           std::fabs(ra.steps[i].peak_stress - rb.steps[i].peak_stress) < 1e-9 &&
                           ra.steps[i].n_contacts == rb.steps[i].n_contacts;
        check(same_metrics, "same seed -> identical metrics");
    }

    // Different seeds -> different structures (the RNG actually varies placement).
    {
        Params p = tower(DEFAULT_F, DEFAULT_B, DEFAULT_J, 1, N, 0);
        Simulator a(p, 1), b(p, 2);
        SimResult ra = a.run(N), rb = b.run(N);
        double diff = 0.0;
        for (int i = 0; i < N; ++i) diff += (ra.robots[i].position - rb.robots[i].position).norm();
        check(diff > 1.0, "different seeds -> different structures");
    }

    // Chain and cantilever are now wired (edge spawn) and run end to end.
    {
        for (const char* which : {"chain", "cantilever"}) {
            Params p = std::string(which) == "chain" ? chain(DEFAULT_F, DEFAULT_B, DEFAULT_J, 1, 20, 0)
                                                     : cantilever(DEFAULT_F, DEFAULT_B, DEFAULT_J, 1, 20, 0);
            Simulator sim(p, 5);
            SimResult r = sim.run(20);
            check(!r.robots.empty(), std::string(which) + " runs and places robots");
        }
        // A cantilever should extend past the edge (reach > 0) once robots stack outward.
        Params pc = cantilever(DEFAULT_F, DEFAULT_B, DEFAULT_J, 1, 40, 0);
        Simulator scant(pc, 5);
        SimResult rc = scant.run(40);
        double max_reach = 0.0;
        for (const auto& m : rc.steps) max_reach = std::max(max_reach, m.reach);
        check(max_reach > 0.0, "cantilever reaches past the edge");
    }

    // Chain descends SUSTAINABLY via the docking climber (Phase 10). The old yaw-only climber
    // stalled at ~9 robots / depth ~15 then reverted to building up; docking hangs a thin line
    // that keeps extending. Grow N=40 and require it to descend well past the old stall, that
    // depth grows monotonically to the end (no revert), and that no two spheres interpenetrate.
    {
        Params p = chain(DEFAULT_F, DEFAULT_B, DEFAULT_J, 1, 40, 0);
        Simulator sim(p, 7);
        SimResult r = sim.run(40);
        check(r.robots.size() >= 38, "chain seats nearly all robots");
        double last_depth = r.steps.back().depth;
        check(last_depth > 60.0, "chain descends far past the old ~15 stall");
        // monotone non-decreasing depth over the second half (no reverting to build up)
        bool monotone = true;
        for (std::size_t i = r.steps.size() / 2; i + 1 < r.steps.size(); ++i)
            if (r.steps[i + 1].depth < r.steps[i].depth - 1e-6) monotone = false;
        check(monotone, "chain depth never reverts once descending (no stall-and-build-up)");
        // penetration-free: every pair of structural spheres is >= 2r apart (minus eps)
        std::vector<geom::Vec3> sc;
        for (const auto& rb : r.robots)
            for (const auto& c : rb.sphere_centers()) sc.push_back(c);
        double mind = 1e300;
        for (std::size_t i = 0; i < sc.size(); ++i)
            for (std::size_t j = i + 1; j < sc.size(); ++j)
                mind = std::min(mind, (sc[i] - sc[j]).norm());
        check(mind >= 2.0 * SPHERE_RADIUS - 1e-3, "chain is penetration-free");
    }

    // Bridge (Phase 10): two arms dock toward each other from opposite lips and SPAN the gap.
    {
        Params p = bridge(BRIDGE_GAPS[0]);  // gap = 20
        Simulator sim(p, 3);
        SimResult r = sim.run();            // up to BRIDGE_N_CAP robots
        check(r.bridge_spanned, "bridge spans the gap (the two arms meet)");
        check(r.bridge_span_n > 0 && r.bridge_span_n <= BRIDGE_N_CAP,
              "bridge records the robot count at span");
        // The spanning structure reaches across: some sphere left of the gap and some right.
        double max_x = -1e300, min_x = 1e300;
        for (const auto& rb : r.robots)
            for (const auto& c : rb.sphere_centers()) {
                max_x = std::max(max_x, c.x());
                min_x = std::min(min_x, c.x());
            }
        check(max_x > 0.5 * p.experiment.gap_width - SPHERE_RADIUS &&
                  min_x < -0.5 * p.experiment.gap_width + SPHERE_RADIUS,
              "bridge structure reaches both lips");
        // penetration-free
        std::vector<geom::Vec3> sc;
        for (const auto& rb : r.robots)
            for (const auto& c : rb.sphere_centers()) sc.push_back(c);
        double mind = 1e300;
        for (std::size_t i = 0; i < sc.size(); ++i)
            for (std::size_t j = i + 1; j < sc.size(); ++j)
                mind = std::min(mind, (sc[i] - sc[j]).norm());
        check(mind >= 2.0 * SPHERE_RADIUS - 1e-3, "bridge is penetration-free");
    }

    return rbtest::summary("test_simulator");
}
