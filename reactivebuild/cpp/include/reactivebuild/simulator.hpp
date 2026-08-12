// simulator.hpp -- the ReactiveBuild add-one-robot-at-a-time loop (Phase 6).
//
// Ties together every prior layer into the paper's simulation loop (section 3): add one
// moving robot, let it climb and join, re-solve the FEM, propagate recruitment, record
// metrics, repeat. Only one moving robot exists at a time (the paper's simplifying
// assumption). Each robot addition is:
//
//   spawn (Phase 5) -> climb (Phase 5) -> join as structural
//     -> all_contacts + Environment anchors (Phase 1/5)
//     -> build_robot_fem + solve_truss (Phase 2)
//     -> all_sensed_forces + peak_connection_stress (Phase 3)
//     -> propagate_recruitment updates comm_out (Phase 4)  [used by the NEXT climber]
//     -> record StepMetrics
//
// v1 drives the TOWER (flat plane) end to end; chain/cantilever/bridge need their own spawn
// (behind the furthest-back robot / alternating sides) and are added in Phase 7.
#pragma once

#include <Eigen/Dense>

#include <algorithm>
#include <cstdint>
#include <ostream>
#include <stdexcept>
#include <vector>

#include "reactivebuild/algorithm.hpp"
#include "reactivebuild/climbing.hpp"
#include "reactivebuild/config.hpp"
#include "reactivebuild/environment.hpp"
#include "reactivebuild/fem.hpp"
#include "reactivebuild/robot.hpp"
#include "reactivebuild/sensing.hpp"

namespace rb {

// One row per robot addition.
struct StepMetrics {
    int n = 0;               // robots in the structure after this addition
    double height = 0.0;     // tallest sphere top above the plane
    double peak_stress = 0.0;  // peak connection von-Mises stress
    double max_sensed = 0.0;
    double mean_sensed = 0.0;
    int n_contacts = 0;      // robot-robot sphere contacts
    int n_anchors = 0;       // environment anchor contacts
    int max_recruit = 0;     // largest comm_out anywhere in the structure
    bool recruited = false;  // the robot just added joined via recruitment
    double reach = 0.0;      // furthest sphere past the edge toward the goal (cantilever)
    double depth = 0.0;      // furthest sphere below the plane (chain)
    double x = 0.0, y = 0.0, z = 0.0;  // pose of the robot just added
    int climb_steps = 0;
};

struct SimResult {
    std::vector<StepMetrics> steps;
    std::vector<FireAnt3D> robots;  // final structure
    bool bridge_spanned = false;    // bridge only: the two arms met before the N cap
    int bridge_span_n = -1;         // bridge only: robot count when the gap was first spanned
};

class Simulator {
public:
    Simulator(const Params& params, std::uint64_t seed)
        : params_(params),
          env_(make_environment(params.experiment)),
          goal_(goal_position(params.experiment, env_)),
          rng_(seed) {
        climber_.cfg = ClimbConfig();
        // The chain grows a thin hanging line: it needs full-orientation docking, not the
        // yaw-only surface-walk (which the tower/cantilever use). See climbing.hpp dock_climb.
        use_docking_ = (params.experiment.type == ExperimentType::CHAIN);
    }

    const std::vector<FireAnt3D>& robots() const { return robots_; }
    const Environment& environment() const { return env_; }
    const geom::Vec3& goal() const { return goal_; }

    // Grow the structure to n_robots (default: params.experiment.n_robots). Returns per-step
    // metrics + the final structure. max_spawn_retry guards against a spawn that keeps
    // falling into a void (edge/gap experiments); on TOWER a robot always finds support.
    SimResult run(int n_robots = -1, int max_spawn_retry = 25) {
        if (n_robots < 0) n_robots = params_.experiment.n_robots;
        if (env_.kind == EnvKind::GAP) return runBridge(n_robots, max_spawn_retry);
        SimResult out;
        for (int i = 0; i < n_robots; ++i) {
            ClimbResult cr;
            bool placed = false;
            for (int attempt = 0; attempt < max_spawn_retry; ++attempt) {
                Support sup = build_support(robots_);
                if (use_docking_) {
                    cr = dock_climb(robots_, sup, env_, goal_, params_.robot.sphere_radius, rng_,
                                    dock_cfg_);
                } else {
                    geom::Vec3 start = spawn(sup);
                    geom::Mat3 rot = random_yaw(rng_);
                    cr = climber_.run(start, rot, sup, env_, goal_, params_.robot.sphere_radius);
                }
                if (!cr.lost) { placed = true; break; }
            }
            if (!placed) break;  // could not seat a robot; stop growing

            FireAnt3D robot(i, cr.position, cr.rotation, Role::STRUCTURAL, params_.robot);
            robots_.push_back(robot);
            out.steps.push_back(joinAndMeasure(cr));
        }
        out.robots = robots_;
        return out;
    }

    // Bridge (section 4.4): two arms grow toward each other from opposite lips of the gap,
    // alternating one robot per side. Each arm docks onto ITS OWN spheres (build_support_side)
    // and targets the OPPOSITE lip, so the arms extend across the void via full-orientation
    // docking. The bridge is "spanned" the first time a newly placed robot touches a sphere of
    // the other arm (the arms meet); we record the robot count at that moment. Both arms stay
    // anchored to their own solid ground throughout, so the FEM is well-posed even before they
    // connect. Wider gaps need longer arms -> more robots and higher peak stress to span.
    SimResult runBridge(int n_cap, int max_spawn_retry = 25) {
        SimResult out;
        const double r = params_.robot.sphere_radius;
        const double half = 0.5 * env_.gap;
        const double lipL = -half, lipR = half;
        const geom::Vec3 goalL(lipR, 0.0, env_.plane_z);  // left arm -> right lip
        const geom::Vec3 goalR(lipL, 0.0, env_.plane_z);  // right arm -> left lip
        std::vector<int> side;                            // 0 = left, 1 = right, per robot
        bool spanned = false;

        for (int i = 0; i < n_cap && !spanned; ++i) {
            int s = i % 2;
            Support sup = build_support_side(robots_, side, s);   // dock onto our own arm
            Support all = build_support(robots_);                 // but never overlap any sphere
            geom::Vec3 goal = (s == 0) ? goalL : goalR;
            double lip = (s == 0) ? lipL : lipR;
            double gdir = (s == 0) ? -1.0 : 1.0;          // which side of the lip is solid
            ClimbResult cr;
            bool placed = false;
            for (int attempt = 0; attempt < max_spawn_retry; ++attempt) {
                cr = dock_climb(robots_, sup, env_, goal, r, rng_, dock_cfg_, lip, gdir, &all);
                if (!cr.lost) { placed = true; break; }
            }
            if (!placed) continue;  // this arm is stuck this round; let the other one grow
            robots_.push_back(FireAnt3D(i, cr.position, cr.rotation, Role::STRUCTURAL,
                                        params_.robot));
            side.push_back(s);
            out.steps.push_back(joinAndMeasure(cr));
            if (touchesOtherSide(side, s)) { spanned = true; out.bridge_span_n = i + 1; }
        }
        out.bridge_spanned = spanned;
        out.robots = robots_;
        return out;
    }

private:
    // Have the arms met? The just-added robot (robots_.back(), on side s) is within LATCH of a
    // sphere on the other arm. Penetration (< 2r) is forbidden by pose_ok, so the two tips can
    // approach to at best ~2r; a robot actively connects ("latches") across a small remaining
    // gap, so we declare a span when opposite spheres come within 2r + latch (latch = 0.5r).
    bool touchesOtherSide(const std::vector<int>& side, int s) const {
        const auto& nu = robots_.back();
        const double latch = 0.5 * nu.radius();
        auto cn = nu.sphere_centers();
        for (int i = 0; i + 1 < static_cast<int>(robots_.size()); ++i) {
            if (side[i] == s) continue;
            auto ci = robots_[i].sphere_centers();
            for (const auto& a : cn)
                for (const auto& b : ci)
                    if (geom::spheres_contact(a, nu.radius(), b, robots_[i].radius(), latch))
                        return true;
        }
        return false;
    }

    // Experiment-specific spawn.
    //   TOWER (plane): random angle just outside the footprint, on the plane.
    //   CHAIN / CANTILEVER (edge): behind the furthest-back robot, on the plane.
    geom::Vec3 spawn(const Support& sup) {
        const double r = params_.robot.sphere_radius;
        if (env_.kind == EnvKind::PLANE)
            return spawn_on_plane(sup, env_, r, goal_, rng_, climber_.cfg);
        if (env_.kind == EnvKind::EDGE) {
            if (params_.experiment.type == ExperimentType::CHAIN)
                return spawn_chain(sup, env_, r, rng_, climber_.cfg);  // hang from the tip
            return spawn_behind_edge(sup, env_, r, rng_, climber_.cfg);  // cantilever: behind
        }
        throw std::runtime_error("simulator: bridge spawn (two-sided, alternating) not yet wired");
    }

    // Solve the FEM on the current structure, propagate recruitment, and produce metrics.
    StepMetrics joinAndMeasure(const ClimbResult& cr) {
        const int n = static_cast<int>(robots_.size());
        auto contacts = all_contacts(robots_);
        auto anchors = env_.anchor_contacts(robots_);

        TrussModel model = build_robot_fem(robots_, contacts, anchors, env_.plane_z,
                                           params_.fem, params_.gravity);
        TrussResult sol = solve_truss(model, params_.fem.length_normalized);
        auto sensed = all_sensed_forces(model, sol, robots_);
        auto bundles = connection_bundles(robots_, contacts, anchors);
        double peak = peak_connection_stress(model, sol, bundles, params_.fem.contact_stress_radius);

        // recruitment field for the NEXT climber
        propagate_recruitment(robots_, contacts, sensed, params_.algorithm);

        StepMetrics m;
        m.n = n;
        m.height = structureHeight();
        m.peak_stress = peak;
        m.n_contacts = static_cast<int>(contacts.size());
        m.n_anchors = static_cast<int>(anchors.size());
        double sum = 0.0, mx = 0.0;
        for (double v : sensed) { sum += v; mx = std::max(mx, v); }
        m.max_sensed = mx;
        m.mean_sensed = n > 0 ? sum / n : 0.0;
        int mr = 0;
        for (const auto& r : robots_)
            for (int v : r.comm_out) mr = std::max(mr, v);
        m.max_recruit = mr;
        m.recruited = cr.recruited;
        reachAndDepth(m.reach, m.depth);
        m.x = cr.position.x();
        m.y = cr.position.y();
        m.z = cr.position.z();
        m.climb_steps = cr.steps;
        return m;
    }

    double structureHeight() const {
        double top = env_.plane_z;
        const double r = params_.robot.sphere_radius;
        for (const auto& rb : robots_) {
            auto c = rb.sphere_centers();
            for (const auto& sc : c) top = std::max(top, sc.z() + r);
        }
        return top - env_.plane_z;
    }

    // reach = furthest sphere past the edge toward the goal (cantilever horizontal extent);
    // depth = furthest sphere below the plane (chain drop). Both floored at 0.
    void reachAndDepth(double& reach, double& depth) const {
        double edge = std::isfinite(env_.edge_x) ? env_.edge_x : 0.0;
        double max_x = edge, min_z = env_.plane_z;
        for (const auto& rb : robots_) {
            auto c = rb.sphere_centers();
            for (const auto& sc : c) {
                max_x = std::max(max_x, sc.x());
                min_z = std::min(min_z, sc.z());
            }
        }
        reach = std::max(0.0, max_x - edge);
        depth = std::max(0.0, env_.plane_z - min_z);
    }

    Params params_;
    Environment env_;
    geom::Vec3 goal_;
    std::mt19937_64 rng_;
    std::vector<FireAnt3D> robots_;
    GreedySurfaceWalk climber_;
    DockConfig dock_cfg_;
    bool use_docking_ = false;
};

// --- CSV output -------------------------------------------------------------

inline void write_metrics_header(std::ostream& os) {
    os << "n,height,peak_stress,max_sensed,mean_sensed,n_contacts,n_anchors,max_recruit,"
          "recruited,reach,depth,x,y,z,climb_steps\n";
}
inline void write_metrics_row(std::ostream& os, const StepMetrics& m) {
    os << m.n << ',' << m.height << ',' << m.peak_stress << ',' << m.max_sensed << ','
       << m.mean_sensed << ',' << m.n_contacts << ',' << m.n_anchors << ',' << m.max_recruit
       << ',' << (m.recruited ? 1 : 0) << ',' << m.reach << ',' << m.depth << ',' << m.x << ','
       << m.y << ',' << m.z << ',' << m.climb_steps << '\n';
}
inline void write_metrics_csv(std::ostream& os, const SimResult& r) {
    write_metrics_header(os);
    for (const auto& m : r.steps) write_metrics_row(os, m);
}

// Final sphere world positions (one row per sphere) for shape analysis (cross-section, etc.).
inline void write_positions_csv(std::ostream& os, const SimResult& r) {
    os << "robot,sphere,x,y,z\n";
    for (const auto& rb : r.robots) {
        auto c = rb.sphere_centers();
        for (int s = 0; s < static_cast<int>(c.size()); ++s)
            os << rb.id << ',' << s << ',' << c[s].x() << ',' << c[s].y() << ',' << c[s].z()
               << '\n';
    }
}

}  // namespace rb
