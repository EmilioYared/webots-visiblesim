// test_fem.cpp -- Phase 2 truss FEM validation (mirrors reference_python test_fem.py).
// Analytic cases checked to relative 1e-9.
#include "reactivebuild/fem.hpp"
#include "rb_test.hpp"

#include <stdexcept>

using namespace rb;
using rbtest::check;
using rbtest::checkClose;

static std::vector<std::array<bool, 3>> fixedMask(int n,
                                                  std::vector<std::pair<int, int>> dofs) {
    std::vector<std::array<bool, 3>> m(n, {false, false, false});
    for (auto& d : dofs) m[d.first][d.second] = true;
    return m;
}

static int countClass(const TrussModel& m, int cls) {
    int c = 0;
    for (int e : m.element_class) if (e == cls) ++c;
    return c;
}

int main() {
    // 1. single axial element: u = PL/EA, N = P (tension)
    {
        double EA = 2e6, L = 3.0, P = 500.0;
        TrussModel m;
        m.nodes.resize(2, 3); m.nodes << 0, 0, 0, L, 0, 0;
        m.elements = {{0, 1}};
        m.rigidity = {EA};
        m.fixed = fixedMask(2, {{0, 0}, {0, 1}, {0, 2}, {1, 1}, {1, 2}});
        m.loads = Eigen::MatrixXd::Zero(2, 3); m.loads(1, 0) = P;
        auto r = solve_truss(m);
        checkClose(r.displacements(1, 0), P * L / EA, "single element u = PL/EA");
        checkClose(r.axial(0), P, "single element N = P (tension)");
    }
    // compression sign
    {
        double EA = 1e6, L = 2.0, P = -300.0;
        TrussModel m;
        m.nodes.resize(2, 3); m.nodes << 0, 0, 0, L, 0, 0;
        m.elements = {{0, 1}}; m.rigidity = {EA};
        m.fixed = fixedMask(2, {{0, 0}, {0, 1}, {0, 2}, {1, 1}, {1, 2}});
        m.loads = Eigen::MatrixXd::Zero(2, 3); m.loads(1, 0) = P;
        auto r = solve_truss(m);
        check(r.axial(0) < 0, "compression negative");
    }
    // 2. symmetric two-bar truss: N=5P/8, dz=-125P/(32EA), dx=0
    {
        double EA = 1e7, P = 800.0;
        TrussModel m;
        m.nodes.resize(3, 3); m.nodes << 0, 0, 0, -3, 0, 4, 3, 0, 4;
        m.elements = {{0, 1}, {0, 2}}; m.rigidity = {EA, EA};
        m.fixed = fixedMask(3, {{0, 1}, {1, 0}, {1, 1}, {1, 2}, {2, 0}, {2, 1}, {2, 2}});
        m.loads = Eigen::MatrixXd::Zero(3, 3); m.loads(0, 2) = -P;
        auto r = solve_truss(m);
        checkClose(r.axial(0), 5 * P / 8, "two-bar N0 = 5P/8");
        checkClose(r.axial(1), 5 * P / 8, "two-bar N1 = 5P/8");
        check(std::fabs(r.displacements(0, 0)) < 1e-9, "apex dx == 0 (symmetry)");
        checkClose(r.displacements(0, 2), -125 * P / (32 * EA), "apex dz");
        check(r.displacements(0, 2) < 0, "apex sags down");
        // 3. global equilibrium
        Eigen::RowVector3d rsum = r.reactions.colwise().sum();
        Eigen::RowVector3d lsum = m.loads.colwise().sum();
        check((rsum + lsum).norm() < 1e-6, "sum(reactions) == -sum(loads)");
        checkClose(rsum(2), P, "reactions hold up the load");
    }
    // 4. length_normalized flag
    {
        double EA = 2e6, L = 4.0, P = 500.0;
        TrussModel m;
        m.nodes.resize(2, 3); m.nodes << 0, 0, 0, L, 0, 0;
        m.elements = {{0, 1}}; m.rigidity = {EA};
        m.fixed = fixedMask(2, {{0, 0}, {0, 1}, {0, 2}, {1, 1}, {1, 2}});
        m.loads = Eigen::MatrixXd::Zero(2, 3); m.loads(1, 0) = P;
        auto rn = solve_truss(m, true);
        auto rr = solve_truss(m, false);
        checkClose(rn.displacements(1, 0), P * L / EA, "k = EA/L branch");
        checkClose(rr.displacements(1, 0), P / EA, "k = EA branch");
    }
    // 5. under-constrained system reported
    {
        TrussModel m;
        m.nodes.resize(2, 3); m.nodes << 0, 0, 0, 1, 0, 0;
        m.elements = {{0, 1}}; m.rigidity = {1e6};
        m.fixed = fixedMask(2, {{0, 0}, {0, 1}, {0, 2}});  // node1 free in all 3
        m.loads = Eigen::MatrixXd::Zero(2, 3); m.loads(1, 1) = 10.0;  // transverse, unresistable
        bool threw = false;
        try { solve_truss(m); } catch (const std::exception&) { threw = true; }
        check(threw, "under-constrained solve throws");
    }

    // --- build_robot_fem ---
    {
        std::vector<FireAnt3D> robots = {FireAnt3D(0, geom::Vec3(0, 0, 1.0))};
        auto gc = ground_contacts(robots, 0.0);
        auto m = build_robot_fem(robots, {}, gc, 0.0);
        int na = static_cast<int>(gc.size());
        check(m.n_nodes() == 16 + 4 * na, "node count 16 + 4*anchors");
        check(countClass(m, IN_SPHERE) == 4 * 6 + 6 * na, "in-sphere count");
        check(countClass(m, ROBOT_STRUCTURE) == 3 * 16, "robot-structure count");
        check(countClass(m, CONNECTION) == 16 * na, "connection count");

        auto r = solve_truss(m);
        bool zeroFixed = true;
        for (int n = 0; n < m.n_nodes(); ++n)
            if (m.fixed[n][0])
                if (r.displacements.row(n).norm() > 1e-12) zeroFixed = false;
        check(zeroFixed, "fixed nodes have zero displacement");
        checkClose(m.loads.colwise().sum()(2), -3.0, "total load = -3 (weight)");
        checkClose(r.reactions.colwise().sum()(2), 3.0, "reactions carry the weight");
    }
    // stacked: top sags more than bottom
    {
        std::vector<FireAnt3D> robots = {FireAnt3D(0, geom::Vec3(0, 0, 1.0)),
                                         FireAnt3D(1, geom::Vec3(0, 0, 3.0))};
        auto contacts = all_contacts(robots);
        auto gc = ground_contacts(robots, 0.0);
        auto m = build_robot_fem(robots, contacts, gc, 0.0);
        auto r = solve_truss(m);
        double bottomZ = 0, topZ = 0;
        for (int k = 12; k < 16; ++k) bottomZ += r.displacements(k, 2) / 4.0;
        for (int k = 16 + 12; k < 16 + 16; ++k) topZ += r.displacements(k, 2) / 4.0;
        check(topZ < bottomZ && bottomZ <= 1e-12, "top center sags more than bottom");
    }
    return rbtest::summary("test_fem");
}
