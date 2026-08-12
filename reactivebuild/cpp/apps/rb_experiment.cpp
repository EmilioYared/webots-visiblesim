// rb_experiment.cpp -- Phase 7 experiment runner. Runs `runs` independent simulations of one
// (experiment, F, B, J, N) configuration and writes run-tagged CSVs for Python analysis:
//   <tag>_metrics.csv : one row per (run, robot-count) -- the full growth trajectory.
//   <tag>_spheres.csv : final sphere world positions per run (shape / cross-section analysis).
//
// Usage: rb_experiment <tower|chain|cantilever> F B J N runs seed_base [out_dir]
#include "reactivebuild/simulator.hpp"

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>

using namespace rb;

static Params make_params(const std::string& exp, double F, double B, int J, int N) {
    if (exp == "tower") return tower(F, B, J, 1, N, 0);
    if (exp == "chain") return chain(F, B, J, 1, N, 0);
    if (exp == "cantilever") return cantilever(F, B, J, 1, N, 0);
    throw std::runtime_error("rb_experiment: unknown/unwired experiment '" + exp +
                             "' (tower|chain|cantilever; bridge is separate)");
}

int main(int argc, char** argv) {
    if (argc < 8) {
        std::fprintf(stderr,
                     "usage: %s <tower|chain|cantilever> F B J N runs seed_base [out_dir]\n",
                     argv[0]);
        return 2;
    }
    std::string exp = argv[1];
    double F = std::atof(argv[2]);
    double B = std::atof(argv[3]);
    int J = std::atoi(argv[4]);
    int N = std::atoi(argv[5]);
    int runs = std::atoi(argv[6]);
    unsigned long seed_base = std::strtoul(argv[7], nullptr, 10);
    std::string out_dir = argc > 8 ? argv[8] : "reactivebuild/results";

    char tag[160];
    std::snprintf(tag, sizeof(tag), "%s_F%g_B%g_J%d", exp.c_str(), F, B, J);

    std::string mpath = out_dir + "/" + tag + "_metrics.csv";
    std::string spath = out_dir + "/" + tag + "_spheres.csv";
    std::ofstream mf(mpath), sf(spath);
    if (!mf || !sf) {
        std::fprintf(stderr, "ERROR: cannot write to %s\n", out_dir.c_str());
        return 1;
    }
    mf << "run,";
    write_metrics_header(mf);
    sf << "run,robot,sphere,x,y,z\n";

    double sum_final_height = 0.0, sum_peak = 0.0;
    int completed = 0;
    for (int run = 0; run < runs; ++run) {
        Params p = make_params(exp, F, B, J, N);
        Simulator sim(p, seed_base + run);
        SimResult r = sim.run(N);

        for (const auto& m : r.steps) {
            mf << run << ',';
            write_metrics_row(mf, m);
        }
        for (const auto& rb : r.robots) {
            auto c = rb.sphere_centers();
            for (int s = 0; s < static_cast<int>(c.size()); ++s)
                sf << run << ',' << rb.id << ',' << s << ',' << c[s].x() << ',' << c[s].y()
                   << ',' << c[s].z() << '\n';
        }
        if (!r.steps.empty()) {
            sum_final_height += r.steps.back().height;
            sum_peak += r.steps.back().peak_stress;
            ++completed;
        }
        if ((run + 1) % 10 == 0 || run + 1 == runs)
            std::printf("  %s F=%g B=%g J=%d : %d/%d runs\n", exp.c_str(), F, B, J, run + 1, runs);
    }

    std::printf("%s F=%g B=%g J=%d N=%d runs=%d : mean final height=%.3f  mean peak stress=%.3f\n",
                exp.c_str(), F, B, J, N, runs, completed ? sum_final_height / completed : 0.0,
                completed ? sum_peak / completed : 0.0);
    std::printf("  wrote %s\n  wrote %s\n", mpath.c_str(), spath.c_str());
    return 0;
}
