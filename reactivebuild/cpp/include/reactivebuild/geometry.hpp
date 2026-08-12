// geometry.hpp -- FireAnt3D geometry primitives (Phase 1, C++ port).
//
// Three spheres in a coplanar equilateral triangle (mutually tangent, side 2r), robot
// center at the centroid. Each sphere and the center is a 4-node regular tetra -> 16
// nodes/robot (this is also why the 0.25/node load gives 1.0/sphere). See plan Q3/Q6.
#pragma once

#include <Eigen/Dense>

#include <array>
#include <cmath>

#include "reactivebuild/config.hpp"

namespace rb {
namespace geom {

using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;

constexpr double PI = 3.14159265358979323846;
constexpr double CONTACT_TOL = 1e-6;  // "touching" slack

// Body-frame centers of the 3 spheres (equilateral triangle in xy, mutually tangent).
inline std::array<Vec3, 3> sphere_offsets(double sphere_radius) {
    const double side = 2.0 * sphere_radius;
    const double R = side / std::sqrt(3.0);            // circumradius
    const double deg[3] = {90.0, 210.0, 330.0};
    std::array<Vec3, 3> out;
    for (int i = 0; i < 3; ++i) {
        double a = deg[i] * PI / 180.0;
        out[i] = Vec3(R * std::cos(a), R * std::sin(a), 0.0);
    }
    return out;
}

// Body-frame node offsets of a 4-node regular tetra with the given circumradius.
inline std::array<Vec3, 4> tetra_offsets(double circumradius) {
    static const double v[4][3] = {{1, 1, 1}, {1, -1, -1}, {-1, 1, -1}, {-1, -1, 1}};
    const double s = circumradius / std::sqrt(3.0);
    std::array<Vec3, 4> out;
    for (int i = 0; i < 4; ++i) out[i] = Vec3(v[i][0] * s, v[i][1] * s, v[i][2] * s);
    return out;
}

inline Mat3 rotation_from_axis_angle(const Vec3& axis, double angle) {
    double n = axis.norm();
    if (n == 0.0) return Mat3::Identity();
    return Eigen::AngleAxisd(angle, axis / n).toRotationMatrix();
}

// world = position + rotation * body
inline Vec3 apply_pose(const Vec3& body, const Vec3& position, const Mat3& rotation) {
    return position + rotation * body;
}

inline std::array<Vec3, 3> sphere_centers(const Vec3& position, const Mat3& rotation,
                                          const RobotParams& rp) {
    auto offs = sphere_offsets(rp.sphere_radius);
    std::array<Vec3, 3> out;
    for (int i = 0; i < 3; ++i) out[i] = apply_pose(offs[i], position, rotation);
    return out;
}

struct RobotNodes {
    std::array<std::array<Vec3, 4>, 3> sphere_nodes;  // [sphere][node]
    std::array<Vec3, 4> center_nodes;
};

inline RobotNodes robot_node_positions(const Vec3& position, const Mat3& rotation,
                                       const RobotParams& rp) {
    const double tetra_r = rp.tetra_scale * rp.sphere_radius;
    auto offs = sphere_offsets(rp.sphere_radius);
    auto tet = tetra_offsets(tetra_r);
    RobotNodes rn;
    for (int s = 0; s < 3; ++s)
        for (int k = 0; k < 4; ++k)
            rn.sphere_nodes[s][k] = apply_pose(offs[s] + tet[k], position, rotation);
    for (int k = 0; k < 4; ++k) rn.center_nodes[k] = apply_pose(tet[k], position, rotation);
    return rn;
}

// All 16 nodes ordered sphere0(4), sphere1(4), sphere2(4), center(4).
// This ordering is the contract the FEM assembly relies on.
inline std::array<Vec3, 16> all_node_positions(const Vec3& position, const Mat3& rotation,
                                               const RobotParams& rp) {
    RobotNodes rn = robot_node_positions(position, rotation, rp);
    std::array<Vec3, 16> out;
    int idx = 0;
    for (int s = 0; s < 3; ++s)
        for (int k = 0; k < 4; ++k) out[idx++] = rn.sphere_nodes[s][k];
    for (int k = 0; k < 4; ++k) out[idx++] = rn.center_nodes[k];
    return out;
}

inline bool spheres_contact(const Vec3& c1, double r1, const Vec3& c2, double r2,
                            double tol = CONTACT_TOL) {
    return (c1 - c2).norm() <= r1 + r2 + tol;
}

inline bool sphere_plane_contact(const Vec3& center, double radius, double plane_z = 0.0,
                                 double tol = CONTACT_TOL) {
    return center.z() - radius <= plane_z + tol;
}

}  // namespace geom
}  // namespace rb
