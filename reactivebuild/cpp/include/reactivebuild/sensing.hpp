// sensing.hpp -- force-sensor and connection-stress reconstruction (Phase 3, C++ port).
//
// A bundle of axial-only truss elements spanning two 4-node groups transmits a resultant
// force AND moment; bundle_resultant() decomposes it into axial/shear/bending/torsion.
// From the 3 sphere<->center bundles we form sensed_force = mean(|axial|+|bending|)
// (section 3), and from CONNECTION bundles a von-Mises connection stress (circular contact
// radius 0.5). Interpretation flags: magnitudes in the sensed sum (Q2), moment about the
// bundle midpoint (Q2b).
#pragma once

#include <Eigen/Dense>

#include <cmath>
#include <string>
#include <vector>

#include "reactivebuild/fem.hpp"

namespace rb {

struct ForceResultant {
    double axial;    // signed, along axis (+ == tension / pull apart)
    double shear;    // perpendicular force magnitude
    double bending;  // perpendicular moment magnitude
    double torsion;  // along-axis moment (signed)
    geom::Vec3 force;
    geom::Vec3 moment;
};

// Resultant transmitted from group_a to group_b through the truss elements linking them.
// axis defaults to (centroid_a -> centroid_b); moment reference defaults to their midpoint.
inline ForceResultant bundle_resultant(const TrussModel& model, const TrussResult& result,
                                       const std::vector<int>& group_a,
                                       const std::vector<int>& group_b,
                                       const geom::Vec3* ref_point = nullptr,
                                       const geom::Vec3* axis_in = nullptr) {
    using geom::Vec3;
    std::vector<char> inA(model.n_nodes(), 0), inB(model.n_nodes(), 0);
    for (int i : group_a) inA[i] = 1;
    for (int i : group_b) inB[i] = 1;

    Vec3 ca = Vec3::Zero(), cb = Vec3::Zero();
    for (int i : group_a) ca += model.nodes.row(i).transpose();
    ca /= static_cast<double>(group_a.size());
    for (int i : group_b) cb += model.nodes.row(i).transpose();
    cb /= static_cast<double>(group_b.size());

    Vec3 axis = axis_in ? *axis_in : (cb - ca);
    double na = axis.norm();
    Vec3 axis_hat = na > 0 ? (axis / na) : Vec3(0, 0, 1);
    Vec3 ref = ref_point ? *ref_point : (0.5 * (ca + cb));

    Vec3 F = Vec3::Zero(), M = Vec3::Zero();
    for (int m = 0; m < model.n_elements(); ++m) {
        int i = model.elements[m][0], j = model.elements[m][1];
        int a_node, b_node;
        if (inA[i] && inB[j]) { a_node = i; b_node = j; }
        else if (inA[j] && inB[i]) { a_node = j; b_node = i; }
        else continue;
        double N = result.axial(m);
        Vec3 pa = model.nodes.row(a_node).transpose();
        Vec3 pb = model.nodes.row(b_node).transpose();
        Vec3 c = (pb - pa).normalized();
        Vec3 f_on_b = -N * c;               // tension pulls b toward a
        F += f_on_b;
        M += (pb - ref).cross(f_on_b);
    }
    double axial = F.dot(axis_hat);
    double shear = (F - axial * axis_hat).norm();
    double torsion = M.dot(axis_hat);
    double bending = (M - torsion * axis_hat).norm();
    return {axial, shear, bending, torsion, F, M};
}

inline double robot_sensed_force(const TrussModel& model, const TrussResult& result,
                                 int robot_index, int num_spheres = 3) {
    auto center = center_node_indices(robot_index);
    std::vector<int> cg(center.begin(), center.end());
    double total = 0.0;
    for (int s = 0; s < num_spheres; ++s) {
        auto sph = sphere_node_indices(robot_index, s);
        std::vector<int> sg(sph.begin(), sph.end());
        ForceResultant r = bundle_resultant(model, result, sg, cg);
        total += std::fabs(r.axial) + std::fabs(r.bending);
    }
    return total / num_spheres;
}

inline std::vector<double> all_sensed_forces(const TrussModel& model,
                                             const TrussResult& result,
                                             const std::vector<FireAnt3D>& robots) {
    std::vector<double> out(robots.size());
    for (int ri = 0; ri < static_cast<int>(robots.size()); ++ri)
        out[ri] = robot_sensed_force(model, result, ri, robots[ri].params.num_spheres);
    return out;
}

inline double connection_stress(const TrussModel& model, const TrussResult& result,
                                const std::vector<int>& group_a,
                                const std::vector<int>& group_b,
                                double contact_radius = 0.5) {
    ForceResultant r = bundle_resultant(model, result, group_a, group_b);
    double rad = contact_radius;
    double area = geom::PI * rad * rad;
    double I = geom::PI * std::pow(rad, 4) / 4.0;
    double Jp = geom::PI * std::pow(rad, 4) / 2.0;
    double sigma = std::fabs(r.axial) / area + std::fabs(r.bending) * rad / I;
    double tau = std::fabs(r.shear) / area + std::fabs(r.torsion) * rad / Jp;
    return std::sqrt(sigma * sigma + 3.0 * tau * tau);
}

inline double peak_connection_stress(const TrussModel& model, const TrussResult& result,
                                     const std::vector<Bundle>& bundles,
                                     double contact_radius = 0.5) {
    double mx = 0.0;
    bool any = false;
    for (const auto& b : bundles) {
        double s = connection_stress(model, result, b.group_a, b.group_b, contact_radius);
        if (!any || s > mx) { mx = s; any = true; }
    }
    return any ? mx : 0.0;
}

}  // namespace rb
