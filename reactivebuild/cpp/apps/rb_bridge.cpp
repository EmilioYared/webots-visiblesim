// rb_bridge.cpp -- Phase 10 bridge runner (section 4.4). Runs the two-sided docking bridge for
// each gap width over several independent seeds and reports, per gap: the success rate (arms
// met before the N cap), the mean robot count at span, and the mean peak stress at span. Also
// writes:
//   bridge_summary.csv : one row per (gap, run) -- spanned, span_n, peak_stress_at_span.
//   bridge_g<gap>_spheres.csv : final sphere positions of the first spanning run (for viz).
//
// Usage: rb_bridge [runs] [seed_base] [out_dir]
#include "reactivebuild/simulator.hpp"

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>

using namespace rb;

int main(int argc, char** argv) {
    int runs = argc > 1 ? std::atoi(argv[1]) : 10;
    unsigned long seed_base = argc > 2 ? std::strtoul(argv[2], nullptr, 10) : 1000;
    std::string out_dir = argc > 3 ? argv[3] : "reactivebuild/results";

    std::ofstream sum(out_dir + "/bridge_summary.csv");
    if (!sum) { std::fprintf(stderr, "ERROR: cannot write to %s\n", out_dir.c_str()); return 1; }
    sum << "gap,run,spanned,span_n,peak_stress_at_span\n";

    for (double gap : BRIDGE_GAPS) {
        int n_span = 0, n_total = 0;
        double sum_span_n = 0.0, sum_peak = 0.0;
        bool wrote_spheres = false;
        for (int run = 0; run < runs; ++run) {
            Params p = bridge(gap);
            Simulator sim(p, seed_base + run);
            SimResult r = sim.run();
            ++n_total;

            // peak stress at the span step (or final if it never spanned)
            double peak_at = 0.0;
            int idx = r.bridge_spanned ? r.bridge_span_n - 1 : static_cast<int>(r.steps.size()) - 1;
            if (idx >= 0 && idx < static_cast<int>(r.steps.size())) peak_at = r.steps[idx].peak_stress;

            sum << gap << ',' << run << ',' << (r.bridge_spanned ? 1 : 0) << ','
                << r.bridge_span_n << ',' << peak_at << '\n';

            if (r.bridge_spanned) { ++n_span; sum_span_n += r.bridge_span_n; sum_peak += peak_at; }

            // dump the first spanning run's geometry for the Webots viewer
            if (r.bridge_spanned && !wrote_spheres) {
                char sp[256];
                std::snprintf(sp, sizeof(sp), "%s/bridge_g%g_spheres.csv", out_dir.c_str(), gap);
                std::ofstream sf(sp);
                sf << "robot,sphere,x,y,z\n";
                for (const auto& rb : r.robots) {
                    auto c = rb.sphere_centers();
                    for (int s = 0; s < static_cast<int>(c.size()); ++s)
                        sf << rb.id << ',' << s << ',' << c[s].x() << ',' << c[s].y() << ','
                           << c[s].z() << '\n';
                }
                wrote_spheres = true;
                std::printf("  wrote %s\n", sp);
            }
        }
        std::printf("gap=%.0f : spanned %d/%d  mean span_n=%.1f  mean peak stress@span=%.1f\n",
                    gap, n_span, n_total, n_span ? sum_span_n / n_span : 0.0,
                    n_span ? sum_peak / n_span : 0.0);
    }
    std::printf("  wrote %s/bridge_summary.csv\n", out_dir.c_str());
    return 0;
}
