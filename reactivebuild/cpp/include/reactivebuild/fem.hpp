// fem.hpp -- linear 3D truss finite-element solver (Phase 2, C++ port, Eigen sparse).
//
// Generic pin-jointed truss (3 translational DOF/node) via the direct stiffness method,
// solved with Eigen's SimplicialLDLT, plus build_robot_fem() which turns a scene of
// FireAnt3D robots + contacts + ground anchors into the paper's truss model (section 3,
// Fig. 3): each sphere/center is a 4-node tetra (IN_SPHERE edges), each sphere fully
// connects to the center (ROBOT_STRUCTURE), contacting spheres and sphere<->environment
// fully connect (CONNECTION), env-tetra nodes are fixed, 0.25 load per sphere node.
#pragma once

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <array>
#include <stdexcept>
#include <string>
#include <vector>

#include "reactivebuild/config.hpp"
#include "reactivebuild/robot.hpp"

namespace rb {

enum ElementClass { IN_SPHERE = 0, ROBOT_STRUCTURE = 1, CONNECTION = 2 };

// --- node-layout helpers (single source of truth for build_robot_fem + sensing) ---
constexpr int STRIDE = 16;  // sphere0..2 (4 each) + center (4)

inline std::array<int, 4> sphere_node_indices(int robot_index, int sphere_index) {
    int base = robot_index * STRIDE + 4 * sphere_index;
    return {base, base + 1, base + 2, base + 3};
}
inline std::array<int, 4> center_node_indices(int robot_index) {
    int base = robot_index * STRIDE + 12;
    return {base, base + 1, base + 2, base + 3};
}
inline int env_anchor_base(int n_robots, int anchor_index) {
    return n_robots * STRIDE + 4 * anchor_index;
}

struct TrussModel {
    Eigen::MatrixXd nodes;                       // N x 3
    std::vector<std::array<int, 2>> elements;    // M
    std::vector<double> rigidity;                // M (EA, or k if !length_normalized)
    std::vector<std::array<bool, 3>> fixed;      // N x 3 (true == DOF constrained to 0)
    Eigen::MatrixXd loads;                       // N x 3
    std::vector<int> element_class;              // M

    int n_nodes() const { return static_cast<int>(nodes.rows()); }
    int n_elements() const { return static_cast<int>(elements.size()); }
};

struct TrussResult {
    Eigen::MatrixXd displacements;  // N x 3
    Eigen::VectorXd axial;          // M (tension positive)
    Eigen::VectorXd lengths;        // M
    Eigen::MatrixXd reactions;      // N x 3 (0 at free DOFs)
    Eigen::VectorXd k;              // M (per-element axial stiffness used)
};

inline TrussResult solve_truss(const TrussModel& model, bool length_normalized = true) {
    using geom::Vec3;
    const int N = model.n_nodes();
    const int M = model.n_elements();
    const int ndof = 3 * N;

    std::vector<Eigen::Triplet<double>> trips;
    trips.reserve(static_cast<size_t>(M) * 36);
    Eigen::VectorXd L(M), kvec(M);
    std::vector<Vec3> axes(M);

    for (int m = 0; m < M; ++m) {
        int i = model.elements[m][0], j = model.elements[m][1];
        Vec3 d = (model.nodes.row(j) - model.nodes.row(i)).transpose();
        double len = d.norm();
        if (len == 0.0) throw std::runtime_error("degenerate (zero-length) truss element");
        Vec3 c = d / len;
        double k = length_normalized ? model.rigidity[m] / len : model.rigidity[m];
        L(m) = len; kvec(m) = k; axes[m] = c;
        Eigen::Matrix3d B = k * (c * c.transpose());
        int di = 3 * i, dj = 3 * j;
        for (int a = 0; a < 3; ++a)
            for (int b = 0; b < 3; ++b) {
                double v = B(a, b);
                trips.emplace_back(di + a, di + b, v);
                trips.emplace_back(di + a, dj + b, -v);
                trips.emplace_back(dj + a, di + b, -v);
                trips.emplace_back(dj + a, dj + b, v);
            }
    }
    Eigen::SparseMatrix<double> K(ndof, ndof);
    K.setFromTriplets(trips.begin(), trips.end());

    Eigen::VectorXd f(ndof);
    for (int n = 0; n < N; ++n)
        for (int a = 0; a < 3; ++a) f(3 * n + a) = model.loads(n, a);

    // free DOF list
    std::vector<int> freeDofs;
    freeDofs.reserve(ndof);
    for (int n = 0; n < N; ++n)
        for (int a = 0; a < 3; ++a) {
            bool fx = !model.fixed.empty() && model.fixed[n][a];
            if (!fx) freeDofs.push_back(3 * n + a);
        }
    const int nf = static_cast<int>(freeDofs.size());

    Eigen::VectorXd u = Eigen::VectorXd::Zero(ndof);
    if (nf > 0) {
        Eigen::SparseMatrix<double> P(nf, ndof);
        std::vector<Eigen::Triplet<double>> pt;
        pt.reserve(nf);
        for (int r = 0; r < nf; ++r) pt.emplace_back(r, freeDofs[r], 1.0);
        P.setFromTriplets(pt.begin(), pt.end());

        Eigen::SparseMatrix<double> Kff = P * K * P.transpose();
        Kff.makeCompressed();
        Eigen::VectorXd ff = P * f;

        Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;
        solver.compute(Kff);
        if (solver.info() != Eigen::Success)
            throw std::runtime_error("truss solve: factorization failed (singular / "
                                     "under-constrained)");
        Eigen::VectorXd uf = solver.solve(ff);
        if (solver.info() != Eigen::Success || !uf.allFinite())
            throw std::runtime_error("truss solve: non-finite result (singular)");
        u = P.transpose() * uf;
    }

    Eigen::VectorXd axial(M);
    for (int m = 0; m < M; ++m) {
        int i = model.elements[m][0], j = model.elements[m][1];
        Vec3 du(u(3 * j) - u(3 * i), u(3 * j + 1) - u(3 * i + 1), u(3 * j + 2) - u(3 * i + 2));
        axial(m) = kvec(m) * du.dot(axes[m]);
    }

    Eigen::VectorXd R = K * u - f;
    for (int d : freeDofs) R(d) = 0.0;

    TrussResult res;
    res.displacements.resize(N, 3);
    res.reactions.resize(N, 3);
    for (int n = 0; n < N; ++n)
        for (int a = 0; a < 3; ++a) {
            res.displacements(n, a) = u(3 * n + a);
            res.reactions(n, a) = R(3 * n + a);
        }
    res.axial = axial;
    res.lengths = L;
    res.k = kvec;
    return res;
}

// --- robot scene -> TrussModel (section 3, Fig. 3) --------------------------

inline TrussModel build_robot_fem(
    const std::vector<FireAnt3D>& robots,
    const std::vector<Contact>& robot_contacts = {},
    const std::vector<std::pair<int, int>>& ground_anchors = {},
    double plane_z = 0.0, const FEMParams& fem = FEMParams(),
    const geom::Vec3& gravity = geom::Vec3(0, 0, -1)) {
    const int n_robots = static_cast<int>(robots.size());
    const int n_env = static_cast<int>(ground_anchors.size());
    const int N = n_robots * STRIDE + 4 * n_env;

    TrussModel model;
    model.nodes.resize(N, 3);
    for (int ri = 0; ri < n_robots; ++ri) {
        auto an = robots[ri].all_nodes();
        for (int l = 0; l < 16; ++l) model.nodes.row(ri * STRIDE + l) = an[l].transpose();
    }
    for (int a = 0; a < n_env; ++a) {
        int ri = ground_anchors[a].first, si = ground_anchors[a].second;
        geom::Vec3 contact = robots[ri].sphere_centers()[si];
        contact.z() = plane_z;
        auto tet = geom::tetra_offsets(robots[ri].params.tetra_scale *
                                       robots[ri].params.sphere_radius);
        int base = env_anchor_base(n_robots, a);
        for (int k = 0; k < 4; ++k) model.nodes.row(base + k) = (contact + tet[k]).transpose();
    }

    auto& elems = model.elements;
    auto& eclass = model.element_class;
    auto addTetra = [&](int base, int cls) {
        static const int e[6][2] = {{0, 1}, {0, 2}, {0, 3}, {1, 2}, {1, 3}, {2, 3}};
        for (auto& p : e) { elems.push_back({base + p[0], base + p[1]}); eclass.push_back(cls); }
    };
    auto addBipartite = [&](const std::array<int, 4>& A, const std::array<int, 4>& B, int cls) {
        for (int a = 0; a < 4; ++a)
            for (int b = 0; b < 4; ++b) { elems.push_back({A[a], B[b]}); eclass.push_back(cls); }
    };

    for (int ri = 0; ri < n_robots; ++ri) {
        for (int si = 0; si < robots[ri].params.num_spheres; ++si)
            addTetra(ri * STRIDE + 4 * si, IN_SPHERE);
        addTetra(ri * STRIDE + 12, IN_SPHERE);  // center tetra
        for (int si = 0; si < robots[ri].params.num_spheres; ++si)
            addBipartite(sphere_node_indices(ri, si), center_node_indices(ri), ROBOT_STRUCTURE);
    }
    for (const auto& c : robot_contacts)
        addBipartite(sphere_node_indices(c.ri, c.si), sphere_node_indices(c.rj, c.sj), CONNECTION);
    for (int a = 0; a < n_env; ++a) {
        int ri = ground_anchors[a].first, si = ground_anchors[a].second;
        int base = env_anchor_base(n_robots, a);
        std::array<int, 4> env = {base, base + 1, base + 2, base + 3};
        addBipartite(sphere_node_indices(ri, si), env, CONNECTION);
        addTetra(base, IN_SPHERE);
    }

    model.rigidity.resize(elems.size());
    for (size_t m = 0; m < elems.size(); ++m) {
        switch (eclass[m]) {
            case IN_SPHERE: model.rigidity[m] = fem.k_in_sphere; break;
            case ROBOT_STRUCTURE: model.rigidity[m] = fem.k_robot_structure; break;
            default: model.rigidity[m] = fem.k_connection; break;
        }
    }

    model.loads = Eigen::MatrixXd::Zero(N, 3);
    for (int ri = 0; ri < n_robots; ++ri)
        for (int si = 0; si < robots[ri].params.num_spheres; ++si)
            for (int nd : sphere_node_indices(ri, si))
                model.loads.row(nd) = (fem.sphere_node_load * gravity).transpose();

    model.fixed.assign(N, {false, false, false});
    for (int a = 0; a < n_env; ++a) {
        int base = env_anchor_base(n_robots, a);
        for (int k = 0; k < 4; ++k) model.fixed[base + k] = {true, true, true};
    }
    return model;
}

struct Bundle {
    std::vector<int> group_a, group_b;
    std::string kind;
};

inline std::vector<Bundle> connection_bundles(
    const std::vector<FireAnt3D>& robots, const std::vector<Contact>& robot_contacts = {},
    const std::vector<std::pair<int, int>>& ground_anchors = {}) {
    const int n = static_cast<int>(robots.size());
    std::vector<Bundle> out;
    for (const auto& c : robot_contacts) {
        auto A = sphere_node_indices(c.ri, c.si), B = sphere_node_indices(c.rj, c.sj);
        out.push_back({{A.begin(), A.end()}, {B.begin(), B.end()}, "robot-robot"});
    }
    for (int a = 0; a < static_cast<int>(ground_anchors.size()); ++a) {
        int ri = ground_anchors[a].first, si = ground_anchors[a].second;
        int base = env_anchor_base(n, a);
        auto A = sphere_node_indices(ri, si);
        out.push_back({{A.begin(), A.end()}, {base, base + 1, base + 2, base + 3}, "ground"});
    }
    return out;
}

}  // namespace rb
