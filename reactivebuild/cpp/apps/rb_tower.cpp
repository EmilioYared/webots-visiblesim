// rb_tower.cpp -- Phase 6 demo: grow a tower end to end and write metrics + final sphere
// positions to CSV. This is the "a full tower run produces a metrics CSV + final positions"
// artifact. Phase 7 generalizes this into rb_experiment.cpp (all four structures + sweeps).
//
// Usage: rb_tower [N] [F] [B] [J] [seed] [out_dir]
#include "reactivebuild/simulator.hpp"

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>

using namespace rb;

int main(int argc, char** argv) {
    int N = argc > 1 ? std::atoi(argv[1]) : N_ROBOTS;
    double F = argc > 2 ? std::atof(argv[2]) : DEFAULT_F;
    double B = argc > 3 ? std::atof(argv[3]) : DEFAULT_B;
    int J = argc > 4 ? std::atoi(argv[4]) : DEFAULT_J;
    unsigned long seed = argc > 5 ? std::strtoul(argv[5], nullptr, 10) : 0;
    std::string out_dir = argc > 6 ? argv[6] : "reactivebuild/results";

    Params p = tower(F, B, J, 1, N, static_cast<int>(seed));
    Simulator sim(p, seed);
    SimResult r = sim.run(N);

    char tag[128];
    std::snprintf(tag, sizeof(tag), "tower_F%g_B%g_J%d_s%lu", F, B, J, seed);

    std::string mpath = out_dir + "/" + tag + "_metrics.csv";
    std::string ppath = out_dir + "/" + tag + "_positions.csv";
    std::ofstream mf(mpath), pf(ppath);
    if (!mf || !pf) {
        std::fprintf(stderr, "ERROR: cannot write to %s (does the directory exist?)\n",
                     out_dir.c_str());
        return 1;
    }
    write_metrics_csv(mf, r);
    write_positions_csv(pf, r);

    const StepMetrics& last = r.steps.back();
    std::printf("tower F=%g B=%g J=%d seed=%lu : %zu robots\n", F, B, J, seed, r.robots.size());
    std::printf("  final height      = %.3f\n", last.height);
    std::printf("  peak conn. stress = %.3f\n", last.peak_stress);
    std::printf("  max sensed force  = %.3f\n", last.max_sensed);
    std::printf("  max recruit value = %d\n", last.max_recruit);
    std::printf("  wrote %s\n  wrote %s\n", mpath.c_str(), ppath.c_str());
    return 0;
}
