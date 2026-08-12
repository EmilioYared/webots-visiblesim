// climbing.hpp -- greedy surface-walk locomotion for moving robots (Phase 5).
//
// This is the paper's least-specified part (locomotion is deferred to ref [22], which we do
// not have) and our highest-risk modelling choice (plan Q1, DECIDED = greedy surface-walk).
// It is deliberately behind a small, swappable interface (GreedySurfaceWalk) so it can be
// refined once tower/chain shapes are compared to the figures.
//
// The v1 model:
//   * A robot keeps a FIXED orientation while walking -- a random yaw about the up axis,
//     assigned at spawn. Random yaw is what breaks lattice alignment: without it, identical
//     robots stacked on-axis nest into aligned columns whose load bypasses the robot centre,
//     giving ~0 sensed force and no recruitment (the Phase-3 physics insight). Random yaw
//     makes spheres rest in each other's pockets -> amorphous, offset packing -> real forces.
//   * settle(): rigid vertical drop along -up; the robot stops at the HIGHEST contact of any
//     of its spheres against existing spheres or the environment (exact, deterministic).
//   * step: sample n_dirs horizontal directions, settle each candidate, move to the one that
//     most reduces distance-to-goal (rise/*fall* per step capped so it walks the surface
//     rather than teleporting up/down a cliff). Join at the local closest point (Alg. 1's
//     "would reverse") or as soon as a contacted sphere carries a recruit signal >= 1.
//
// Assumes up = +z (gravity = -z). Non-vertical gravity would need a rotated settle; out of
// scope for the four experiments here.
#pragma once

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <random>
#include <unordered_map>
#include <vector>

#include "reactivebuild/config.hpp"
#include "reactivebuild/environment.hpp"
#include "reactivebuild/robot.hpp"

namespace rb {

// Flattened support surface: every structural sphere (world centre) + the recruit signal it
// carries (that zone's comm_out) + the shared radius. A coarse xy grid (built once per robot
// addition) answers "which spheres are near (x,y)?" in ~O(1) so the climber's many settle
// queries stay cheap as the structure grows.
struct Support {
    std::vector<geom::Vec3> centers;
    std::vector<int> recruit;  // parallel to centers
    double radius = SPHERE_RADIUS;
    double cell = 2.0 * SPHERE_RADIUS;
    std::unordered_map<std::int64_t, std::vector<int>> grid;

    static std::int64_t key(int ix, int iy) {
        return (static_cast<std::int64_t>(ix) << 32) ^ (static_cast<std::uint32_t>(iy));
    }
    void build_grid() {
        cell = 2.0 * radius;
        grid.clear();
        for (int i = 0; i < static_cast<int>(centers.size()); ++i) {
            int ix = static_cast<int>(std::floor(centers[i].x() / cell));
            int iy = static_cast<int>(std::floor(centers[i].y() / cell));
            grid[key(ix, iy)].push_back(i);
        }
    }
    // Append indices of spheres whose xy is within `rad` of (x,y) (over-approximate by cell).
    void near(double x, double y, double rad, std::vector<int>& out) const {
        if (grid.empty()) {  // no grid -> linear
            for (int i = 0; i < static_cast<int>(centers.size()); ++i) out.push_back(i);
            return;
        }
        int span = static_cast<int>(std::ceil(rad / cell)) + 1;
        int cx = static_cast<int>(std::floor(x / cell)), cy = static_cast<int>(std::floor(y / cell));
        for (int ix = cx - span; ix <= cx + span; ++ix)
            for (int iy = cy - span; iy <= cy + span; ++iy) {
                auto it = grid.find(key(ix, iy));
                if (it != grid.end()) out.insert(out.end(), it->second.begin(), it->second.end());
            }
    }
};

inline Support build_support(const std::vector<FireAnt3D>& robots) {
    Support s;
    if (!robots.empty()) s.radius = robots.front().radius();
    for (const auto& r : robots) {
        if (r.role != Role::STRUCTURAL) continue;
        auto c = r.sphere_centers();
        for (int si = 0; si < static_cast<int>(c.size()); ++si) {
            s.centers.push_back(c[si]);
            s.recruit.push_back(si < static_cast<int>(r.comm_out.size()) ? r.comm_out[si] : 0);
        }
    }
    s.build_grid();
    return s;
}

// Support built from only the robots whose side[i] == want (the bridge grows two arms; each
// arm docks onto its own spheres only, so they stay separate until they physically meet).
inline Support build_support_side(const std::vector<FireAnt3D>& robots,
                                  const std::vector<int>& side, int want) {
    Support s;
    if (!robots.empty()) s.radius = robots.front().radius();
    for (int i = 0; i < static_cast<int>(robots.size()); ++i) {
        if (robots[i].role != Role::STRUCTURAL || side[i] != want) continue;
        auto c = robots[i].sphere_centers();
        for (int si = 0; si < static_cast<int>(c.size()); ++si) {
            s.centers.push_back(c[si]);
            s.recruit.push_back(si < static_cast<int>(robots[i].comm_out.size())
                                    ? robots[i].comm_out[si] : 0);
        }
    }
    s.build_grid();
    return s;
}

struct ClimbConfig {
    int n_dirs = 12;             // horizontal probe directions per step
    double step = 0.5;           // horizontal step length (sphere radii)
    // Max |dz| per step (~2 sphere diameters). FireAnt3D can climb the floor, wall, AND
    // ceiling of its peers (paper §2), so steep ascents are within capability; a lower cap
    // trapped climbers on the shallow flank and produced short, over-wide piles.
    double max_step_rise = 4.0;
    int max_steps = 4000;
    double progress_eps = 1e-6;
    double spawn_margin = 3.0;   // spawn this far beyond the current footprint
    double stabilize_roll = 1.0; // nestle cap (in radii) for a normal join -- just stabilize
    double reinforce_roll = 6.0; // nestle cap (in radii) for a recruited join -- buttress
};

struct ClimbResult {
    geom::Vec3 position = geom::Vec3::Zero();
    geom::Mat3 rotation = geom::Mat3::Identity();
    bool joined = false;     // became structural (reversed or recruited or step cap)
    bool recruited = false;  // joined because a contacted sphere had recruit >= 1
    bool reversed = false;   // joined at the local closest point to the goal
    bool lost = false;       // never found support (fell into a void)
    int steps = 0;
};

// Highest resting centre-z for a robot at horizontal (cx, cy) with rotation rot, dropped
// straight down onto the support spheres + environment. Returns false if unsupported.
inline bool settle_z(double cx, double cy, const geom::Mat3& rot, double r,
                     const Support& sup, const Environment& env, double& out_z) {
    const double two_r = 2.0 * r;
    const double two_r2 = two_r * two_r;
    auto offs = geom::sphere_offsets(r);
    double best = Environment::NO_SUPPORT;
    for (int j = 0; j < 3; ++j) {
        geom::Vec3 o = rot * offs[j];  // body offset -> world (yaw keeps o.z == 0)
        double sx = cx + o.x(), sy = cy + o.y();
        if (env.has_support(sx, sy))  // sphere centre at plane_z + r
            best = std::max(best, env.plane_z + r - o.z());
        for (const auto& ck : sup.centers) {
            double dx = sx - ck.x(), dy = sy - ck.y();
            double dh2 = dx * dx + dy * dy;
            if (dh2 <= two_r2)  // rest on top of ck: centre distance == 2r
                best = std::max(best, ck.z() + std::sqrt(two_r2 - dh2) - o.z());
        }
    }
    if (best == Environment::NO_SUPPORT) return false;
    out_z = best;
    return true;
}

// Feasible vertical attachment heights at (cx, cy): centre-z values where the robot touches
// >=1 support (RESTING ON TOP or HANGING BELOW a sphere) or the plane, and penetrates nothing.
// Resting-on-top builds towers/cantilevers (goal above/out); hanging-below builds chains (goal
// below). The climber then picks whichever candidate is closest to the goal, so the same rule
// grows upward or downward depending only on the goal. Uses the support grid for locality.
inline std::vector<double> settle_candidates(double cx, double cy, const geom::Mat3& rot,
                                             double r, const Support& sup,
                                             const Environment& env) {
    const double two_r = 2.0 * r, two_r2 = two_r * two_r, tol = 1e-6;
    auto offb = geom::sphere_offsets(r);
    std::array<geom::Vec3, 3> o = {rot * offb[0], rot * offb[1], rot * offb[2]};

    std::vector<int> local;
    for (int j = 0; j < 3; ++j) sup.near(cx + o[j].x(), cy + o[j].y(), two_r, local);
    std::sort(local.begin(), local.end());
    local.erase(std::unique(local.begin(), local.end()), local.end());

    std::vector<double> cand;
    bool anyPlane = false;
    double min_oz = std::min({o[0].z(), o[1].z(), o[2].z()});
    for (int j = 0; j < 3; ++j)
        if (env.has_support(cx + o[j].x(), cy + o[j].y())) anyPlane = true;
    if (anyPlane) cand.push_back(env.plane_z + r - min_oz);  // rest lowest sphere on plane
    for (int k : local) {
        const auto& ck = sup.centers[k];
        for (int j = 0; j < 3; ++j) {
            double dx = cx + o[j].x() - ck.x(), dy = cy + o[j].y() - ck.y();
            double dh2 = dx * dx + dy * dy;
            if (dh2 <= two_r2) {
                double s = std::sqrt(two_r2 - dh2);
                cand.push_back(ck.z() + s - o[j].z());  // rest on top of k
                cand.push_back(ck.z() - s - o[j].z());  // hang below k
            }
        }
    }

    auto feasible = [&](double cz) {
        bool touch = false;
        for (int j = 0; j < 3; ++j) {
            geom::Vec3 sc(cx + o[j].x(), cy + o[j].y(), cz + o[j].z());
            if (env.has_support(sc.x(), sc.y())) {
                double gap = (sc.z() - r) - env.plane_z;
                if (gap < -tol) return false;
                if (gap <= tol) touch = true;
            }
            for (int k : local) {
                double d = (sc - sup.centers[k]).norm() - two_r;
                if (d < -tol) return false;
                if (d <= tol) touch = true;
            }
        }
        return touch;
    };

    std::vector<double> out;
    for (double cz : cand)
        if (feasible(cz)) out.push_back(cz);
    return out;
}

// Does the robot (at cx,cy,cz with rotation rot) touch any sphere carrying a recruit signal?
inline bool climber_recruited(double cx, double cy, double cz, const geom::Mat3& rot, double r,
                              const Support& sup) {
    auto offs = geom::sphere_offsets(r);
    for (int j = 0; j < 3; ++j) {
        geom::Vec3 sc = geom::Vec3(cx, cy, cz) + rot * offs[j];
        for (std::size_t k = 0; k < sup.centers.size(); ++k)
            if (sup.recruit[k] >= 1 && geom::spheres_contact(sc, r, sup.centers[k], sup.radius))
                return true;
    }
    return false;
}

// Roll the robot downhill into a stable, well-supported pocket (a local minimum of settled
// z), capped at max_roll horizontal displacement from the start. A small cap just STABILISES
// a summit/flank rest (removes single-contact perches -> cleaner stress); a larger cap turns
// a recruited robot into a BUTTRESS -- it rolls down toward the base perimeter and wedges into
// a ground-reaching pocket, providing an alternative load path that reinforces (reduces peak
// stress), which is what recruitment is meant to do (paper Fig. 2g).
inline geom::Vec3 nestle(double cx, double cy, const geom::Mat3& rot, double r,
                         const Support& sup, const Environment& env, double max_roll,
                         int rounds = 30) {
    double z;
    if (!settle_z(cx, cy, rot, r, sup, env, z)) return geom::Vec3(cx, cy, env.plane_z + r);
    const double ox = cx, oy = cy;
    const int NDIR = 16;
    double probe = 0.5 * r;
    for (int it = 0; it < rounds; ++it) {
        double bx = cx, by = cy, bz = z;
        bool improved = false;
        for (int d = 0; d < NDIR; ++d) {
            double ang = 2.0 * geom::PI * d / NDIR;
            double nx = cx + probe * std::cos(ang), ny = cy + probe * std::sin(ang), nz;
            if (std::hypot(nx - ox, ny - oy) > max_roll) continue;  // cap total roll
            if (!settle_z(nx, ny, rot, r, sup, env, nz)) continue;
            if (nz < bz - 1e-9) { bz = nz; bx = nx; by = ny; improved = true; }
        }
        if (improved) { cx = bx; cy = by; z = bz; }
        else { probe *= 0.5; if (probe < 1e-3 * r) break; }
    }
    return geom::Vec3(cx, cy, z);
}

// Greedy surface-walk from a settled start toward the goal.
inline ClimbResult climb(const geom::Vec3& start_center, const geom::Mat3& rot,
                         const Support& sup, const Environment& env, const geom::Vec3& goal,
                         double r, const ClimbConfig& cfg = ClimbConfig()) {
    ClimbResult res;
    res.rotation = rot;
    auto dist = [&](double x, double y, double z) {
        return (goal - geom::Vec3(x, y, z)).norm();
    };
    // Initial attachment: nearest-goal feasible height at the spawn (usually just the plane).
    double cx = start_center.x(), cy = start_center.y(), cz = 0.0;
    {
        auto czs = settle_candidates(cx, cy, rot, r, sup, env);
        if (czs.empty()) {
            res.lost = true;
            res.position = start_center;
            return res;
        }
        double bd = 1e300;
        for (double z : czs) { double d = dist(cx, cy, z); if (d < bd) { bd = d; cz = z; } }
    }
    double cur = dist(cx, cy, cz);

    for (res.steps = 0; res.steps < cfg.max_steps; ++res.steps) {
        if (climber_recruited(cx, cy, cz, rot, r, sup)) {
            res.recruited = true;
            break;
        }
        double bx = cx, by = cy, bz = cz, bd = cur;
        bool moved = false;
        for (int d = 0; d < cfg.n_dirs; ++d) {
            double ang = 2.0 * geom::PI * d / cfg.n_dirs;
            double nx = cx + cfg.step * std::cos(ang);
            double ny = cy + cfg.step * std::sin(ang);
            // Each direction offers several attachment heights (rest-on-top / hang-below);
            // consider those reachable within one step's rise and take the best.
            for (double nz : settle_candidates(nx, ny, rot, r, sup, env)) {
                if (std::fabs(nz - cz) > cfg.max_step_rise) continue;  // too steep to walk
                double nd = dist(nx, ny, nz);
                if (nd < bd - cfg.progress_eps) {
                    bd = nd; bx = nx; by = ny; bz = nz; moved = true;
                }
            }
        }
        if (!moved) {  // no reachable step reduces distance -> passed the closest point
            res.reversed = true;
            break;
        }
        cx = bx; cy = by; cz = bz; cur = bd;
    }
    // Seat the robot. Only nestle when it is resting on top (towers/cantilevers): a recruited
    // robot rolls into a reinforcing buttress, an ordinary join just stabilises. A HANGING robot
    // (chain) is left where it grabbed -- rolling it "downhill" via the top-rest surface is wrong.
    double top_z;
    bool on_top = settle_z(cx, cy, rot, r, sup, env, top_z) && std::fabs(cz - top_z) < 1e-6;
    if (on_top) {
        double roll = (res.recruited ? cfg.reinforce_roll : cfg.stabilize_roll) * r;
        res.position = nestle(cx, cy, rot, r, sup, env, roll);
    } else {
        res.position = geom::Vec3(cx, cy, cz);
    }
    res.joined = true;  // reversed, recruited, or step-cap all end as a join
    return res;
}

// --- spawn helpers ----------------------------------------------------------

// Farthest horizontal distance of any support sphere from (ox, oy).
inline double footprint_radius(const Support& sup, double ox, double oy) {
    double R = 0.0;
    for (const auto& c : sup.centers) {
        double dx = c.x() - ox, dy = c.y() - oy;
        R = std::max(R, std::sqrt(dx * dx + dy * dy));
    }
    return R;
}

inline geom::Mat3 random_yaw(std::mt19937_64& rng) {
    std::uniform_real_distribution<double> U(0.0, 2.0 * geom::PI);
    return geom::rotation_from_axis_angle(geom::Vec3(0, 0, 1), U(rng));
}

// Spawn a robot on the plane at a random angle, just outside the current footprint (measured
// around the goal's horizontal position -- the tower/pile centre). Returns the settled centre.
inline geom::Vec3 spawn_on_plane(const Support& sup, const Environment& env, double r,
                                 const geom::Vec3& goal, std::mt19937_64& rng,
                                 const ClimbConfig& cfg = ClimbConfig()) {
    std::uniform_real_distribution<double> U(0.0, 2.0 * geom::PI);
    double R = footprint_radius(sup, goal.x(), goal.y()) + cfg.spawn_margin + 2.0 * r;
    double a = U(rng);
    double x = goal.x() + R * std::cos(a);
    double y = goal.y() + R * std::sin(a);
    double z = env.plane_z + r;
    settle_z(x, y, geom::Mat3::Identity(), r, sup, env, z);
    return geom::Vec3(x, y, z);
}

// Spawn behind the furthest-back part of the structure, on the plane (chain / cantilever).
// "Furthest back" = smallest x (the -x direction, away from the edge/goal); a fresh run
// starts just behind the edge. Random distance behind + random y, per the paper.
inline geom::Vec3 spawn_behind_edge(const Support& sup, const Environment& env, double r,
                                    std::mt19937_64& rng, const ClimbConfig& cfg = ClimbConfig()) {
    double min_x = std::isfinite(env.edge_x) ? env.edge_x : 0.0;
    for (const auto& c : sup.centers) min_x = std::min(min_x, c.x());
    std::uniform_real_distribution<double> Ux(1.0, cfg.spawn_margin + 2.0 * r);
    std::uniform_real_distribution<double> Uy(-cfg.spawn_margin, cfg.spawn_margin);
    double x = min_x - Ux(rng);  // strictly behind the furthest-back point
    if (x > env.edge_x) x = env.edge_x - 1.0;
    double y = Uy(rng);
    double z = env.plane_z + r;
    settle_z(x, y, geom::Mat3::Identity(), r, sup, env, z);
    return geom::Vec3(x, y, z);
}

// Spawn at the growing END of a hanging chain: the current lowest sphere (the tip). A moving
// robot introduced here hangs below the tip (goal far below), extending the chain downward. A
// fresh run seeds one robot at the edge. This is a pragmatic proxy for "the robot has climbed
// to the active growth front"; the greedy walk alone cannot climb-then-commit-to-descend past
// its rest-on-top preference, so we introduce it at the front (random yaw still applies).
inline geom::Vec3 spawn_chain(const Support& sup, const Environment& env, double r,
                              std::mt19937_64& /*rng*/, const ClimbConfig& /*cfg*/ = ClimbConfig()) {
    if (sup.centers.empty())
        return geom::Vec3((std::isfinite(env.edge_x) ? env.edge_x : 0.0) - 0.5, 0.0,
                          env.plane_z + r);
    geom::Vec3 tip = sup.centers.front();
    for (const auto& c : sup.centers)
        if (c.z() < tip.z()) tip = c;
    return tip;
}

// Thin swappable wrapper -- Phase 6's simulator holds one of these and calls run().
struct GreedySurfaceWalk {
    ClimbConfig cfg;
    ClimbResult run(const geom::Vec3& start, const geom::Mat3& rot, const Support& sup,
                    const Environment& env, const geom::Vec3& goal, double r) const {
        return climb(start, rot, sup, env, goal, r, cfg);
    }
};

// =====================================================================================
// Docking climber (Phase 10) -- full-orientation attachment for chains & bridges.
//
// The greedy surface-walk above keeps a robot at a FIXED (yaw-only) orientation and only
// translates it. That is right for the tower (robots pack onto a growing pile) but it is
// exactly why the chain stalled: a horizontal triangle cannot extend a THIN hanging tip --
// docking its lead sphere below the tip forces its other two spheres to splay sideways at the
// same height, where they penetrate the robots just above, so the hang becomes infeasible and
// the climber falls back to building up (RESULTS.md, "Chain").
//
// The fix is to let the robot choose its FULL 3-D orientation when docking. A triangle whose
// plane is roughly VERTICAL with its apex pointing down hangs cleanly below a tip: the lead
// sphere docks, the other two trail *downward*, away from the structure above, penetrating
// nothing. Orientation is still sampled randomly (amorphous packing is preserved -- the paper's
// premise), but among the sampled poses we keep the penetration-free dock that advances the
// structure's frontier furthest toward the goal. The same rule descends a chain (goal below),
// extends a cantilever, or spans a bridge gap (goal across) -- it depends only on the goal.
//
// This is a placement search, not a walk: the robot is assumed to have reached the growth front
// (as the paper's robots locomote to the goal region) and we solve for how it seats there.
// =====================================================================================

// A uniformly-distributed random rotation (random unit quaternion). Unlike random_yaw this
// samples all of SO(3), so the triangle can tilt/hang, not just spin about the up axis.
inline geom::Mat3 random_rotation(std::mt19937_64& rng) {
    std::normal_distribution<double> N(0.0, 1.0);
    Eigen::Quaterniond q(N(rng), N(rng), N(rng), N(rng));
    if (q.norm() < 1e-12) return geom::Mat3::Identity();
    q.normalize();
    return q.toRotationMatrix();
}

struct DockConfig {
    int n_orient = 64;          // random orientations per (anchor, contact-direction)
    int n_anchors = 5;          // frontier support spheres tried as dock anchors
    int n_contact_dirs = 5;     // contact directions sampled in a cone around the goal dir
    double contact_cone = 1.0;  // rad: half-angle of that cone (allows off-axis docking)
    int n_ground = 40;          // ground/edge seed positions (a sphere resting on the plane)
    double ground_margin = 4.0; // sample ground seeds within this many radii behind the edge
    double tol = 1e-6;
};

// Is pose (pos, rot) penetration-free against the structure + environment, and does it TOUCH at
// least one of them (dock to a structural sphere or rest on the plane)? A floating pose (no
// touch) is rejected -- every robot must connect to the structure so the FEM stays anchored.
//
// `blockers`, when given, is the set checked for penetration + touch (use ALL spheres so a
// robot never overlaps another). `sup` alone is used when blockers == nullptr. For the bridge
// the caller docks anchors onto one arm (via `sup` in dock_climb) but passes blockers = every
// sphere, so the arm that closes the gap seats flush against the other arm instead of into it.
inline bool pose_ok(const geom::Vec3& pos, const geom::Mat3& rot, double r, const Support& sup,
                    const Environment& env, double tol = 1e-6,
                    const Support* blockers = nullptr) {
    const Support& B = blockers ? *blockers : sup;
    auto offb = geom::sphere_offsets(r);
    const double two_r = 2.0 * r;
    bool touch = false;
    std::vector<int> local;
    for (int j = 0; j < 3; ++j) {
        geom::Vec3 sc = pos + rot * offb[j];
        if (env.has_support(sc.x(), sc.y())) {          // over solid ground
            double gap = (sc.z() - r) - env.plane_z;
            if (gap < -tol) return false;               // pushed through the plane
            if (gap <= tol) touch = true;               // resting on the plane
        }
        local.clear();
        B.near(sc.x(), sc.y(), two_r, local);
        for (int k : local) {
            double d = (sc - B.centers[k]).norm();
            if (d < two_r - tol) return false;          // penetrates a structural sphere
            if (d <= two_r + tol) touch = true;         // docks onto it
        }
    }
    return touch;
}

// Place one robot at the growth front by full-orientation docking. Samples penetration-free
// poses (docking onto frontier spheres, and resting/overhanging seeds near the edge/lip) and
// returns the feasible one whose nearest sphere gets closest to `goal`. res.lost if none seat.
//
// lip_x / ground_dir control where the ground seeds are sampled: seeds rest on the solid plane
// within `ground_margin` radii of x == lip_x, on the side given by ground_dir (< 0 = solid is
// x < lip, the chain/left case; > 0 = solid is x > lip, the bridge's right lip). Defaults (NaN
// lip -> env.edge_x, dir = -1) reproduce the chain unchanged. `sup` already restricts docking to
// the correct arm (the caller passes a same-side support for the bridge).
inline ClimbResult dock_climb(const std::vector<FireAnt3D>& /*robots*/, const Support& sup,
                              const Environment& env, const geom::Vec3& goal, double r,
                              std::mt19937_64& rng, const DockConfig& cfg = DockConfig(),
                              double lip_x = std::numeric_limits<double>::quiet_NaN(),
                              double ground_dir = -1.0, const Support* blockers = nullptr) {
    ClimbResult res;
    auto offb = geom::sphere_offsets(r);
    const double two_r = 2.0 * r;

    // frontier metric: how close does this pose's nearest sphere get to the goal (minimise)
    auto goaldist = [&](const geom::Vec3& pos, const geom::Mat3& rot) {
        double best = 1e300;
        for (int j = 0; j < 3; ++j) best = std::min(best, (goal - (pos + rot * offb[j])).norm());
        return best;
    };

    double bestd = 1e300;
    geom::Vec3 bestpos = geom::Vec3::Zero();
    geom::Mat3 bestrot = geom::Mat3::Identity();
    bool found = false;
    auto consider = [&](const geom::Vec3& pos, const geom::Mat3& rot) {
        if (!pose_ok(pos, rot, r, sup, env, cfg.tol, blockers)) return;
        double d = goaldist(pos, rot);
        if (d < bestd) { bestd = d; bestpos = pos; bestrot = rot; found = true; }
    };

    // (a) dock onto the frontier: the support spheres already closest to the goal.
    if (!sup.centers.empty()) {
        std::vector<int> idx(sup.centers.size());
        for (int i = 0; i < static_cast<int>(idx.size()); ++i) idx[i] = i;
        int K = std::min<int>(cfg.n_anchors, static_cast<int>(idx.size()));
        std::partial_sort(idx.begin(), idx.begin() + K, idx.end(), [&](int a, int b) {
            return (goal - sup.centers[a]).squaredNorm() < (goal - sup.centers[b]).squaredNorm();
        });
        std::uniform_real_distribution<double> Uaz(0.0, 2.0 * geom::PI);
        std::uniform_real_distribution<double> Ucone(0.0, cfg.contact_cone);
        for (int ai = 0; ai < K; ++ai) {
            geom::Vec3 anchor = sup.centers[idx[ai]];
            geom::Vec3 gdir = goal - anchor;
            double gn = gdir.norm();
            gdir = (gn < 1e-9) ? geom::Vec3(0, 0, -1) : (gdir / gn);
            geom::Vec3 up = std::fabs(gdir.z()) < 0.9 ? geom::Vec3(0, 0, 1) : geom::Vec3(1, 0, 0);
            geom::Vec3 e1 = gdir.cross(up).normalized();
            geom::Vec3 e2 = gdir.cross(e1).normalized();
            for (int c = 0; c < cfg.n_contact_dirs; ++c) {
                double az = Uaz(rng), po = Ucone(rng);
                geom::Vec3 u = std::cos(po) * gdir +
                               std::sin(po) * (std::cos(az) * e1 + std::sin(az) * e2);
                geom::Vec3 dock_center = anchor + two_r * u;  // new sphere touches the anchor
                for (int o = 0; o < cfg.n_orient; ++o) {
                    geom::Mat3 rot = random_rotation(rng);
                    for (int j = 0; j < 3; ++j)          // try each sphere as the docking one
                        consider(dock_center - rot * offb[j], rot);
                }
            }
        }
    }

    // (b) ground/edge seeds: rest a sphere on the plane just behind the edge, letting the
    // triangle tilt so another sphere overhangs the void toward the goal. This roots the chain
    // (FEM anchor) and lets the first robots reach past the edge to start descending.
    {
        double lip = std::isnan(lip_x) ? (std::isfinite(env.edge_x) ? env.edge_x : 0.0) : lip_x;
        double xa = lip, xb = lip + ground_dir * cfg.ground_margin * r;
        std::uniform_real_distribution<double> Ux(std::min(xa, xb), std::max(xa, xb));
        std::uniform_real_distribution<double> Uy(-cfg.ground_margin * r, cfg.ground_margin * r);
        int per = std::max(1, cfg.n_orient / 2);
        for (int g = 0; g < cfg.n_ground; ++g) {
            double gx = Ux(rng), gy = Uy(rng);
            for (int o = 0; o < per; ++o) {
                geom::Mat3 rot = random_rotation(rng);
                int jmin = 0; double mz = 1e300;         // lowest sphere -> rest it on the plane
                for (int j = 0; j < 3; ++j) {
                    double z = (rot * offb[j]).z();
                    if (z < mz) { mz = z; jmin = j; }
                }
                geom::Vec3 gsphere(gx, gy, env.plane_z + r);
                consider(gsphere - rot * offb[jmin], rot);
            }
        }
    }

    if (!found) {
        res.lost = true;
        double ex = std::isfinite(env.edge_x) ? env.edge_x : 0.0;
        res.position = geom::Vec3(ex, 0.0, env.plane_z + r);
        return res;
    }
    res.position = bestpos;
    res.rotation = bestrot;
    res.recruited = climber_recruited(bestpos.x(), bestpos.y(), bestpos.z(), bestrot, r, sup);
    res.reversed = true;   // docked at the closest feasible approach to the goal
    res.joined = true;
    return res;
}

}  // namespace rb
