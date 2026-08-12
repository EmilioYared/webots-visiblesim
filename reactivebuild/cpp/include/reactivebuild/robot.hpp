// robot.hpp -- the FireAnt3D robot object and scene-level contact detection (Phase 1).
#pragma once

#include <Eigen/Dense>

#include <array>
#include <utility>
#include <vector>

#include "reactivebuild/config.hpp"
#include "reactivebuild/geometry.hpp"

namespace rb {

enum class Role { MOVING, STRUCTURAL };

struct FireAnt3D {
    int id = 0;
    geom::Vec3 position = geom::Vec3::Zero();
    geom::Mat3 rotation = geom::Mat3::Identity();
    Role role = Role::MOVING;
    RobotParams params;
    std::vector<int> comm_in;   // per contact zone (per sphere)
    std::vector<int> comm_out;

    FireAnt3D() { init_comm(); }
    FireAnt3D(int id_, const geom::Vec3& pos,
              const geom::Mat3& rot = geom::Mat3::Identity(),
              Role role_ = Role::MOVING, RobotParams p = RobotParams())
        : id(id_), position(pos), rotation(rot), role(role_), params(p) {
        init_comm();
    }

    void init_comm() {
        comm_in.assign(params.num_spheres, 0);
        comm_out.assign(params.num_spheres, 0);
    }

    int num_zones() const { return params.num_spheres; }
    double radius() const { return params.sphere_radius; }
    std::array<geom::Vec3, 3> sphere_centers() const {
        return geom::sphere_centers(position, rotation, params);
    }
    geom::RobotNodes node_positions() const {
        return geom::robot_node_positions(position, rotation, params);
    }
    std::array<geom::Vec3, 16> all_nodes() const {
        return geom::all_node_positions(position, rotation, params);
    }
    double total_weight() const { return params.robot_weight(); }
    double sphere_node_loads(const FEMParams& fem = FEMParams()) const {
        return params.num_spheres * fem.nodes_per_sphere * fem.sphere_node_load;
    }
};

// robot i sphere si <-> robot j sphere sj
struct Contact {
    int ri, si, rj, sj;
};

inline std::vector<std::pair<int, int>> robot_sphere_contacts(
    const FireAnt3D& a, const FireAnt3D& b, double tol = geom::CONTACT_TOL) {
    auto ca = a.sphere_centers();
    auto cb = b.sphere_centers();
    std::vector<std::pair<int, int>> out;
    for (int i = 0; i < static_cast<int>(ca.size()); ++i)
        for (int j = 0; j < static_cast<int>(cb.size()); ++j)
            if (geom::spheres_contact(ca[i], a.radius(), cb[j], b.radius(), tol))
                out.emplace_back(i, j);
    return out;
}

inline std::vector<Contact> all_contacts(const std::vector<FireAnt3D>& robots,
                                         double tol = geom::CONTACT_TOL) {
    std::vector<Contact> out;
    for (int i = 0; i < static_cast<int>(robots.size()); ++i)
        for (int j = i + 1; j < static_cast<int>(robots.size()); ++j)
            for (auto& p : robot_sphere_contacts(robots[i], robots[j], tol))
                out.push_back({i, p.first, j, p.second});
    return out;
}

inline std::vector<std::pair<int, int>> ground_contacts(
    const std::vector<FireAnt3D>& robots, double plane_z = 0.0,
    double tol = geom::CONTACT_TOL) {
    std::vector<std::pair<int, int>> out;
    for (int i = 0; i < static_cast<int>(robots.size()); ++i) {
        auto c = robots[i].sphere_centers();
        for (int s = 0; s < static_cast<int>(c.size()); ++s)
            if (geom::sphere_plane_contact(c[s], robots[i].radius(), plane_z, tol))
                out.emplace_back(i, s);
    }
    return out;
}

}  // namespace rb
