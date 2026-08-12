// test_sensing.cpp -- Phase 3 sensing tests (mirrors reference_python test_sensing.py).
// The golden sensed_force (1.1547005384) must match the Python oracle -- two independent
// implementations agreeing is the "is our implementation right?" cross-check.
#include "reactivebuild/sensing.hpp"
#include "rb_test.hpp"

using namespace rb;
using rbtest::check;
using rbtest::checkClose;
using rbtest::checkNear;

// A TrussResult carrying only chosen axial forces (for direct reconstruction tests).
static TrussResult fakeResult(std::vector<double> axial) {
    TrussResult r;
    r.axial.resize(axial.size());
    for (int i = 0; i < (int)axial.size(); ++i) r.axial(i) = axial[i];
    return r;
}

static TrussModel bundleModel(double d, double L) {
    TrussModel m;
    m.nodes.resize(4, 3);
    m.nodes << 0, d, 0, 0, -d, 0, L, d, 0, L, -d, 0;  // a0,a1 (spheres), b0,b1 (center)
    m.elements = {{0, 2}, {1, 3}};
    return m;
}

static void cantilever(std::vector<FireAnt3D>& robots, TrussModel& model, TrussResult& res) {
    robots = {FireAnt3D(0, geom::Vec3(0, 0, 1.0))};
    model = build_robot_fem(robots, {}, {{0, 0}}, 0.0);  // anchor only sphere 0
    res = solve_truss(model);
}

int main() {
    // pure couple -> pure bending 2dP
    {
        double d = 0.5, L = 4.0, P = 30.0;
        TrussModel m = bundleModel(d, L);
        auto r = bundle_resultant(m, fakeResult({+P, -P}), {0, 1}, {2, 3});
        check(std::fabs(r.axial) < 1e-9, "couple: axial 0");
        check(std::fabs(r.shear) < 1e-9, "couple: shear 0");
        check(std::fabs(r.torsion) < 1e-9, "couple: torsion 0");
        checkClose(r.bending, 2 * d * P, "couple: bending 2dP");
    }
    // symmetric tension -> pure axial 2P, no bending
    {
        double d = 0.5, L = 4.0, P = 30.0;
        TrussModel m = bundleModel(d, L);
        auto r = bundle_resultant(m, fakeResult({P, P}), {0, 1}, {2, 3});
        checkClose(std::fabs(r.axial), 2 * P, "sym tension: |axial| 2P");
        check(std::fabs(r.bending) < 1e-9, "sym tension: bending 0");
        check(std::fabs(r.shear) < 1e-9, "sym tension: shear 0");
    }
    // tension axial sign: force on b points back toward a (negative along a->b)
    {
        TrussModel m;
        m.nodes.resize(2, 3); m.nodes << 0, 0, 0, 2, 0, 0;
        m.elements = {{0, 1}};
        auto r = bundle_resultant(m, fakeResult({+10.0}), {0}, {1});
        checkClose(r.axial, -10.0, "tension pulls groups together (axial<0)");
    }

    // sensed_force zero under zero gravity
    {
        std::vector<FireAnt3D> robots = {FireAnt3D(0, geom::Vec3(0, 0, 1.0))};
        auto gc = ground_contacts(robots, 0.0);
        auto m = build_robot_fem(robots, {}, gc, 0.0, FEMParams(), geom::Vec3(0, 0, 0));
        auto res = solve_truss(m);
        check(std::fabs(robot_sensed_force(m, res, 0)) < 1e-9, "sensed 0 under zero gravity");
    }
    // sensed_force positive under gravity
    {
        std::vector<FireAnt3D> robots = {FireAnt3D(0, geom::Vec3(0, 0, 1.0))};
        auto gc = ground_contacts(robots, 0.0);
        auto m = build_robot_fem(robots, {}, gc, 0.0);
        auto res = solve_truss(m);
        check(robot_sensed_force(m, res, 0) > 0, "sensed > 0 under gravity");
    }

    // cantilever statics: shear 2.0 / 1.0 / 1.0
    {
        std::vector<FireAnt3D> robots; TrussModel m; TrussResult res;
        cantilever(robots, m, res);
        auto center = center_node_indices(0);
        std::vector<int> cg(center.begin(), center.end());
        double sh[3];
        for (int s = 0; s < 3; ++s) {
            auto sph = sphere_node_indices(0, s);
            std::vector<int> sg(sph.begin(), sph.end());
            sh[s] = bundle_resultant(m, res, sg, cg).shear;
        }
        checkClose(sh[0], 2.0, "anchored connection shear = 2 (carries 2 free spheres)", 1e-6);
        checkClose(sh[1], 1.0, "free connection 1 shear = 1", 1e-6);
        checkClose(sh[2], 1.0, "free connection 2 shear = 1", 1e-6);
    }
    // cantilever mirror symmetry (conn 1 == conn 2)
    {
        std::vector<FireAnt3D> robots; TrussModel m; TrussResult res;
        cantilever(robots, m, res);
        auto center = center_node_indices(0);
        std::vector<int> cg(center.begin(), center.end());
        auto contrib = [&](int s) {
            auto sph = sphere_node_indices(0, s);
            std::vector<int> sg(sph.begin(), sph.end());
            auto r = bundle_resultant(m, res, sg, cg);
            return std::fabs(r.axial) + std::fabs(r.bending);
        };
        checkClose(contrib(1), contrib(2), "cantilever mirror symmetry conn1==conn2", 1e-6);
    }
    // GOLDEN regression: cantilever sensed_force == 1.1547005384 (matches Python oracle)
    {
        std::vector<FireAnt3D> robots; TrussModel m; TrussResult res;
        cantilever(robots, m, res);
        checkClose(robot_sensed_force(m, res, 0), 1.1547005384, "golden sensed_force", 1e-6);
    }

    // all_sensed_forces shape + stack ordering
    {
        std::vector<FireAnt3D> robots;
        for (int i = 0; i < 4; ++i) robots.push_back(FireAnt3D(i, geom::Vec3(0, 0, 1.0 + 2.0 * i)));
        auto contacts = all_contacts(robots);
        auto gc = ground_contacts(robots, 0.0);
        auto m = build_robot_fem(robots, contacts, gc, 0.0);
        auto res = solve_truss(m);
        auto s = all_sensed_forces(m, res, robots);
        check(s.size() == 4, "all_sensed size 4");
        double mx = s[0], mn = s[0];
        for (double v : s) { mx = std::max(mx, v); mn = std::min(mn, v); }
        check(s[0] == mx && s[3] == mn, "bottom feels most, top least");
    }
    // connection stress zero/positive + empty
    {
        std::vector<FireAnt3D> robots = {FireAnt3D(0, geom::Vec3(0, 0, 1.0))};
        auto gc = ground_contacts(robots, 0.0);
        auto m0 = build_robot_fem(robots, {}, gc, 0.0, FEMParams(), geom::Vec3(0, 0, 0));
        auto res0 = solve_truss(m0);
        auto bundles = connection_bundles(robots, {}, gc);
        check(std::fabs(peak_connection_stress(m0, res0, bundles)) < 1e-9, "stress 0 zero-gravity");

        auto mg = build_robot_fem(robots, {}, gc, 0.0);
        auto resg = solve_truss(mg);
        check(peak_connection_stress(mg, resg, bundles) > 0, "stress > 0 under gravity");
        check(peak_connection_stress(mg, resg, {}) == 0.0, "empty bundles -> 0 stress");
    }
    return rbtest::summary("test_sensing");
}
