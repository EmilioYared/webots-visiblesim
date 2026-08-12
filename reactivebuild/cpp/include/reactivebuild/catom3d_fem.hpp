// catom3d_fem.hpp -- internal-force analysis for Catoms3D structures (domain transfer).
//
// Reuses the SAME validated truss-FEM the ReactiveBuild replication is built on (fem.hpp +
// sensing.hpp, cross-validated to 1e-6 against the frozen Python oracle) to answer, for a
// static snapshot of a Catoms3D lattice: which bonds carry the load, and how close each is to
// its tensile / shear limit. This is the "statics oracle" that runs ALONGSIDE the Catoms3D
// controller (which drives the movement via Webots/ODE) -- it does not move anything.
//
// Modelling (identical pattern to build_robot_fem, which is why the numerics are already
// validated): each module is a rigid 4-node tetrahedron, and each bond between two adjacent
// modules is a 16-bar bipartite bundle. A single node per module would make a thin arm / chain
// a pin-jointed MECHANISM (singular); the tetra+bundle transmits force AND moment, so a
// cantilever arm is solvable and its bending shows up -- exactly as in the FireAnt3D chain.
//
// This header is deliberately independent of the Webots controller: it takes plain module
// centres + a bond list, so it has no dependency on catom3d_core.hpp / Webots. The caller
// (apps/catom3d_forces.cpp) turns FCC cells into centres+bonds via the controller's own
// gridToWorld / fccNeighbor and hands them here.
#pragma once

#include <Eigen/Dense>

#include <array>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include "reactivebuild/fem.hpp"
#include "reactivebuild/sensing.hpp"

namespace rb {

struct Catom3DFEMParams {
    double tetra_radius = 0.05;                   // module radius (m): the bond moment lever arm
    double module_weight = 0.1 * 9.81;            // N per module (scene.json mass 0.1 kg * g)
    double tensile_strength = 15.0;               // N: bond pull-apart limit (scene.json)
    double shear_strength = 10.0;                 // N: bond shear limit (scene.json)
    double k_module = STIFFNESS_IN_SPHERE;        // 5e10: near-rigid intra-module bars
    double k_bond = STIFFNESS_CONNECTION;         // 1e10: bond bars
    geom::Vec3 gravity = geom::Vec3(0, 0, -1);    // -z is down (world z is up, ENU)
};

// Module m owns tetra nodes [4m .. 4m+3].
inline std::array<int, 4> catom_nodes(int m) {
    int b = 4 * m;
    return {b, b + 1, b + 2, b + 3};
}

// Build the Catoms3D truss: a rigid tetra per module, a 16-bar bundle per bond. `fixed_module`
// (parallel to centres) pins a module's nodes -- the modules resting on the floor.
inline TrussModel build_catom3d_fem(const std::vector<geom::Vec3>& centers,
                                    const std::vector<std::array<int, 2>>& bonds,
                                    const std::vector<char>& fixed_module,
                                    const Catom3DFEMParams& p = Catom3DFEMParams()) {
    const int n_mod = static_cast<int>(centers.size());
    const int N = 4 * n_mod;
    TrussModel model;
    model.nodes.resize(N, 3);
    auto tet = geom::tetra_offsets(p.tetra_radius);
    for (int m = 0; m < n_mod; ++m)
        for (int k = 0; k < 4; ++k)
            model.nodes.row(4 * m + k) = (centers[m] + tet[k]).transpose();

    auto& elems = model.elements;
    auto& eclass = model.element_class;
    auto addTetra = [&](int base) {
        static const int e[6][2] = {{0, 1}, {0, 2}, {0, 3}, {1, 2}, {1, 3}, {2, 3}};
        for (auto& q : e) { elems.push_back({base + q[0], base + q[1]}); eclass.push_back(IN_SPHERE); }
    };
    auto addBundle = [&](const std::array<int, 4>& A, const std::array<int, 4>& B) {
        for (int a = 0; a < 4; ++a)
            for (int b = 0; b < 4; ++b) { elems.push_back({A[a], B[b]}); eclass.push_back(CONNECTION); }
    };
    for (int m = 0; m < n_mod; ++m) addTetra(4 * m);
    for (const auto& bd : bonds) addBundle(catom_nodes(bd[0]), catom_nodes(bd[1]));

    model.rigidity.resize(elems.size());
    for (std::size_t m = 0; m < elems.size(); ++m)
        model.rigidity[m] = (eclass[m] == IN_SPHERE) ? p.k_module : p.k_bond;

    model.loads = Eigen::MatrixXd::Zero(N, 3);
    for (int m = 0; m < n_mod; ++m)
        for (int k = 0; k < 4; ++k)
            model.loads.row(4 * m + k) = (0.25 * p.module_weight * p.gravity).transpose();

    model.fixed.assign(N, {false, false, false});
    for (int m = 0; m < n_mod; ++m)
        if (m < static_cast<int>(fixed_module.size()) && fixed_module[m])
            for (int k = 0; k < 4; ++k) model.fixed[4 * m + k] = {true, true, true};
    return model;
}

struct BondForce {
    int a = 0, b = 0;             // module indices
    double axial = 0;            // + tension (pull apart), - compression, along the bond
    double shear = 0;
    double bending = 0;
    double util_tension = 0;     // max(axial, 0) / tensile_strength   (>= 1 => bond breaks)
    double util_shear = 0;       // shear / shear_strength
};

struct Catom3DForces {
    bool solved = false;
    std::string note;
    std::vector<BondForce> bonds;
    double max_util_tension = 0, max_util_shear = 0;
    int worst_tension_bond = -1, worst_shear_bond = -1;
    geom::Vec3 reaction_sum = geom::Vec3::Zero();  // sum of support reactions (== +total weight)
    double total_weight = 0;
    std::vector<double> module_load;               // sum of |bond force| meeting each module
};

// Solve the snapshot and report per-bond force + utilisation. On an under-constrained
// (mechanism / floating) structure the linear solve is singular; we report that rather than
// throw, since it is itself a structural verdict (a pin-jointed version would collapse).
inline Catom3DForces catom3d_forces(const std::vector<geom::Vec3>& centers,
                                    const std::vector<std::array<int, 2>>& bonds,
                                    const std::vector<char>& fixed_module,
                                    const Catom3DFEMParams& p = Catom3DFEMParams()) {
    Catom3DForces out;
    const int n_mod = static_cast<int>(centers.size());
    out.total_weight = n_mod * p.module_weight;
    out.module_load.assign(n_mod, 0.0);
    if (n_mod == 0) { out.note = "empty structure"; return out; }

    TrussModel model = build_catom3d_fem(centers, bonds, fixed_module, p);
    TrussResult sol;
    try {
        sol = solve_truss(model, /*length_normalized=*/true);
    } catch (const std::exception& e) {
        out.note = std::string("unsolved: ") + e.what() +
                   " (structure is under-constrained -- would need a support / more bonds)";
        return out;
    }
    out.solved = true;

    for (std::size_t bi = 0; bi < bonds.size(); ++bi) {
        auto A = catom_nodes(bonds[bi][0]), B = catom_nodes(bonds[bi][1]);
        std::vector<int> ga(A.begin(), A.end()), gb(B.begin(), B.end());
        ForceResultant r = bundle_resultant(model, sol, ga, gb);
        BondForce bf;
        bf.a = bonds[bi][0];
        bf.b = bonds[bi][1];
        bf.axial = r.axial;
        bf.shear = r.shear;
        bf.bending = r.bending;
        bf.util_tension = std::max(r.axial, 0.0) / p.tensile_strength;
        bf.util_shear = r.shear / p.shear_strength;
        double mag = std::sqrt(r.axial * r.axial + r.shear * r.shear);
        out.module_load[bf.a] += mag;
        out.module_load[bf.b] += mag;
        if (bf.util_tension > out.max_util_tension) {
            out.max_util_tension = bf.util_tension;
            out.worst_tension_bond = static_cast<int>(bi);
        }
        if (bf.util_shear > out.max_util_shear) {
            out.max_util_shear = bf.util_shear;
            out.worst_shear_bond = static_cast<int>(bi);
        }
        out.bonds.push_back(bf);
    }

    geom::Vec3 rsum = geom::Vec3::Zero();
    for (int m = 0; m < n_mod; ++m)
        if (m < static_cast<int>(fixed_module.size()) && fixed_module[m])
            for (int k : catom_nodes(m)) rsum += sol.reactions.row(k).transpose();
    out.reaction_sum = rsum;
    return out;
}

}  // namespace rb
