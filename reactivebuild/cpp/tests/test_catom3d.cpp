// test_catom3d.cpp -- Catoms3D internal-force analysis (domain transfer of the validated FEM).
// Checks the load-bearing physical invariants: static equilibrium (support reactions balance
// gravity exactly), solvability of supported structures, and that an unsupported structure is
// correctly reported as an under-constrained mechanism rather than silently returning garbage.
#include "reactivebuild/catom3d_fem.hpp"
#include "rb_test.hpp"

using namespace rb;
using rbtest::check;
using rbtest::checkClose;

// A vertical column of `n` modules spaced by `d`, the bottom one fixed to the floor.
static void column(int n, double d, std::vector<geom::Vec3>& centers,
                   std::vector<std::array<int, 2>>& bonds, std::vector<char>& fixed) {
    centers.clear(); bonds.clear(); fixed.clear();
    for (int i = 0; i < n; ++i) {
        centers.push_back(geom::Vec3(0, 0, i * d));
        fixed.push_back(i == 0 ? 1 : 0);
        if (i > 0) bonds.push_back({i - 1, i});
    }
}

int main() {
    Catom3DFEMParams p;  // module_weight = 0.981 N, tensile 15 / shear 10
    const double w = p.module_weight;

    // Equilibrium: a supported column's vertical support reaction equals the total weight, and
    // the lateral reactions vanish (nothing pushes sideways).
    {
        for (int n : {1, 3, 8}) {
            std::vector<geom::Vec3> c; std::vector<std::array<int, 2>> b; std::vector<char> fx;
            column(n, 0.10, c, b, fx);
            Catom3DForces F = catom3d_forces(c, b, fx, p);
            check(F.solved, "supported column solves");
            checkClose(F.reaction_sum.z(), n * w, "reaction balances gravity (sum Rz = N*weight)", 1e-6);
            check(std::fabs(F.reaction_sum.x()) < 1e-6 && std::fabs(F.reaction_sum.y()) < 1e-6,
                  "no spurious lateral reaction under pure gravity");
            checkClose(F.total_weight, n * w, "total weight = N * module_weight", 1e-9);
        }
    }

    // The single vertical bond in a 2-stack carries the WHOLE upper module's weight, and nothing
    // else: |axial| == weight (the load is purely along the bond axis) and the shear is ~0.
    {
        std::vector<geom::Vec3> c; std::vector<std::array<int, 2>> b; std::vector<char> fx;
        column(2, 0.10, c, b, fx);
        Catom3DForces F = catom3d_forces(c, b, fx, p);
        check(F.solved && F.bonds.size() == 1, "2-stack has one bond, solved");
        checkClose(std::fabs(F.bonds[0].axial), w, "vertical bond carries the upper module's weight", 1e-6);
        checkClose(F.bonds[0].shear, 0.0, "pure vertical load -> no shear in a vertical bond", 1e-6);
    }

    // A horizontally-supported pair (both on the floor) is still in equilibrium.
    {
        std::vector<geom::Vec3> c = {geom::Vec3(0, 0, 0), geom::Vec3(0.10, 0, 0)};
        std::vector<std::array<int, 2>> b = {{0, 1}};
        std::vector<char> fx = {1, 1};
        Catom3DForces F = catom3d_forces(c, b, fx, p);
        check(F.solved, "supported pair solves");
        checkClose(F.reaction_sum.z(), 2 * w, "pair: sum Rz = 2*weight", 1e-6);
    }

    // Mechanism detection: a single UNSUPPORTED module is a free rigid body -> the linear solve
    // is singular -> reported as unsolved (a structural verdict), not a crash or silent zero.
    {
        std::vector<geom::Vec3> c = {geom::Vec3(0, 0, 0)};
        std::vector<std::array<int, 2>> b;
        std::vector<char> fx = {0};
        Catom3DForces F = catom3d_forces(c, b, fx, p);
        check(!F.solved, "unsupported (floating) structure reported as under-constrained");
        check(!F.note.empty(), "under-constrained case carries an explanatory note");
    }

    // A fixed single module is trivially in equilibrium (reaction = its own weight).
    {
        std::vector<geom::Vec3> c = {geom::Vec3(0, 0, 0)};
        std::vector<std::array<int, 2>> b;
        std::vector<char> fx = {1};
        Catom3DForces F = catom3d_forces(c, b, fx, p);
        check(F.solved, "fixed single module solves");
        checkClose(F.reaction_sum.z(), w, "fixed single module: Rz = weight", 1e-6);
    }

    return rbtest::summary("test_catom3d");
}
