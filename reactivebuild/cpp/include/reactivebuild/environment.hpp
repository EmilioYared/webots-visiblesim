// environment.hpp -- the static environment a structure grows on/from (Phase 5).
//
// The paper's four experiments differ mainly by environment + goal (section 4):
//   TOWER      -> flat plane,           goal 65 up from the base.
//   CHAIN      -> plane ending at an edge (+cylinder lip), goal 600 down from the edge.
//   CANTILEVER -> plane ending at an edge (+cylinder lip), goal 45 out / 10 up from the edge.
//   BRIDGE     -> two plane halves across a gap,           goals on the opposite side.
//
// The environment provides (a) the support surface the climber walks/settles on and (b) the
// FIXED anchor contacts handed to the FEM (build_robot_fem's ground_anchors). v1 implements
// the flat/half plane exactly; the cylinder lip is a viewing detail (Phase 8) and does not
// affect statics, so it is omitted here. The GAP kind supports the bridge's two-sided spawn.
#pragma once

#include <Eigen/Dense>

#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include "reactivebuild/config.hpp"
#include "reactivebuild/robot.hpp"

namespace rb {

enum class EnvKind { PLANE, EDGE, GAP };

struct Environment {
    EnvKind kind = EnvKind::PLANE;
    double plane_z = 0.0;
    // EDGE: the plane exists only where x <= edge_x (robots chain/cantilever out past it).
    double edge_x = std::numeric_limits<double>::infinity();
    // GAP (bridge): plane on both sides, absent for -gap/2 < x < +gap/2.
    double gap = 0.0;

    static constexpr double NO_SUPPORT = -std::numeric_limits<double>::infinity();

    // Is there solid environment directly below (x, y)?
    bool has_support(double x, double /*y*/) const {
        switch (kind) {
            case EnvKind::PLANE: return true;
            case EnvKind::EDGE: return x <= edge_x;
            case EnvKind::GAP: return x <= -0.5 * gap || x >= 0.5 * gap;
        }
        return true;
    }

    // Height of the support surface below (x, y), or NO_SUPPORT over a void.
    double support_z(double x, double y) const {
        return has_support(x, y) ? plane_z : NO_SUPPORT;
    }

    // Does a sphere rest on (touch) the environment plane?
    bool sphere_touches(const geom::Vec3& c, double r, double tol = geom::CONTACT_TOL) const {
        return has_support(c.x(), c.y()) && (c.z() - r <= plane_z + tol);
    }

    // (robot, sphere) pairs whose sphere touches the environment -> FEM fixed anchors.
    std::vector<std::pair<int, int>> anchor_contacts(const std::vector<FireAnt3D>& robots,
                                                     double tol = geom::CONTACT_TOL) const {
        std::vector<std::pair<int, int>> out;
        for (int i = 0; i < static_cast<int>(robots.size()); ++i) {
            auto c = robots[i].sphere_centers();
            for (int s = 0; s < static_cast<int>(c.size()); ++s)
                if (sphere_touches(c[s], robots[i].radius(), tol)) out.emplace_back(i, s);
        }
        return out;
    }
};

// Build the environment implied by an experiment. The edge sits at x = 0 by convention.
inline Environment make_environment(const ExperimentParams& e) {
    Environment env;
    env.plane_z = 0.0;
    switch (e.type) {
        case ExperimentType::TOWER:
            env.kind = EnvKind::PLANE;
            break;
        case ExperimentType::CHAIN:
        case ExperimentType::CANTILEVER:
            env.kind = EnvKind::EDGE;
            env.edge_x = 0.0;
            break;
        case ExperimentType::BRIDGE:
            env.kind = EnvKind::GAP;
            env.gap = e.gap_width;
            break;
    }
    return env;
}

// Goal position in world coordinates. BASE reference = origin on the plane; EDGE reference =
// the plane edge at (edge_x, 0, plane_z) (origin when the edge is not finite).
inline geom::Vec3 goal_position(const ExperimentParams& e, const Environment& env) {
    geom::Vec3 ref(0.0, 0.0, env.plane_z);
    if (e.goal_reference == GoalReference::EDGE) {
        double ex = std::isfinite(env.edge_x) ? env.edge_x : 0.0;
        ref = geom::Vec3(ex, 0.0, env.plane_z);
    }
    return ref + e.goal_offset;
}

}  // namespace rb
