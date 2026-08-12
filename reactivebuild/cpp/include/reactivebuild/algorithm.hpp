// algorithm.hpp -- the ReactiveBuild algorithm (Phase 4): Alg. 1 (moving robots) and
// Alg. 2 (structural robots) from Swissler & Rubenstein (2022), section 2.
//
// The heart of the paper (Fig. 2) is the RECRUIT-VALUE PROPAGATION: a structural robot
// computes a recruit value from its sensed force and relays it to neighbours, decrementing
// by 1 per hop. "Both inter-robot contacts and intra-robot contact zones are a hop distance
// of 1 apart" (section 2) -- which is exactly Alg. 2's -1 (same zone) / -2 (other zones):
//
//   Algorithm 2 (per structural robot, per contact zone z):
//     recruit_value = min(floor(B^(sensed/F - 1)), J)              (see AlgorithmParams)
//     in_neighbor   = max(comm_in[z])            - 1               (inter-robot hop)
//     in_others     = max(comm_in[other zones])  - 2               (inter- + intra-robot hop)
//     comm_out[z]   = max(recruit_value, in_neighbor, in_others)
//
//   Algorithm 1 (moving robot): step toward the goal; become structural (permanently) when
//     the next step would reverse (passed the closest point to goal) OR when recruited
//     (max(comm_in) >= 1). The geometric climbing/sense-direction model is Phase 5 (Q1);
//     this header provides only the role-transition logic that consumes those signals.
//
// recruit_value itself lives in config.hpp (AlgorithmParams::recruit_value) so the constant
// and the formula stay in one place. This header adds the multi-robot propagation and the
// moving-robot state machine.
#pragma once

#include <algorithm>
#include <limits>
#include <vector>

#include "reactivebuild/config.hpp"
#include "reactivebuild/robot.hpp"

namespace rb {

// --- Algorithm 2: recruit values --------------------------------------------

// Per-robot recruit value from per-robot sensed force (batch of AlgorithmParams::recruit_value).
inline std::vector<int> recruit_values(const std::vector<double>& sensed_forces,
                                       const AlgorithmParams& algo) {
    std::vector<int> out(sensed_forces.size());
    for (std::size_t i = 0; i < sensed_forces.size(); ++i)
        out[i] = algo.recruit_value(sensed_forces[i]);
    return out;
}

// --- Algorithm 2: recruit-value propagation (Fig. 2) ------------------------

// Propagate recruit values across the structure to a fixed point, given each robot's OWN
// recruit value. Mutates robots' comm_in / comm_out. This is the pure integer core Fig. 2
// illustrates -- it needs only the contact graph, not the geometry, so it is testable in
// isolation.
//
// Semantics:
//   - comm_in[R][z]  = max comm_out over all spheres CONTACTING sphere z of robot R
//                      (inter-robot only; intra-robot flow is the -2 term below).
//   - Only STRUCTURAL robots relay/emit; a MOVING robot climbing over the structure never
//     outputs a signal (comm_out stays 0) -- it only reads comm_in to check recruitment.
//   - Synchronous (Jacobi) relaxation: each sweep reads the previous sweep's comm_out, so
//     the result is order-independent. Converges in O(graph diameter) sweeps.
// Returns the number of sweeps performed (including the final no-change sweep).
inline int propagate(std::vector<FireAnt3D>& robots,
                     const std::vector<Contact>& contacts,
                     const std::vector<int>& recruit,
                     int max_iters = 1000) {
    const int n = static_cast<int>(robots.size());
    for (auto& r : robots) {
        std::fill(r.comm_in.begin(), r.comm_in.end(), 0);
        std::fill(r.comm_out.begin(), r.comm_out.end(), 0);
    }

    int iters = 0;
    for (; iters < max_iters; ++iters) {
        // 1. Gather comm_in from neighbours' comm_out (snapshot of previous sweep).
        std::vector<std::vector<int>> in(n);
        for (int i = 0; i < n; ++i) in[i].assign(robots[i].num_zones(), 0);
        for (const auto& c : contacts) {
            int from_j = robots[c.rj].comm_out[c.sj];
            int from_i = robots[c.ri].comm_out[c.si];
            if (from_j > in[c.ri][c.si]) in[c.ri][c.si] = from_j;
            if (from_i > in[c.rj][c.sj]) in[c.rj][c.sj] = from_i;
        }

        // 2. Recompute comm_out per Alg. 2.
        bool changed = false;
        for (int i = 0; i < n; ++i) {
            const int Z = robots[i].num_zones();
            const bool structural = (robots[i].role == Role::STRUCTURAL);
            const int rec = structural ? recruit[i] : 0;
            for (int z = 0; z < Z; ++z) {
                int out = 0;
                if (structural) {
                    int in_neighbor = in[i][z] - 1;
                    int in_others = std::numeric_limits<int>::min();
                    for (int z2 = 0; z2 < Z; ++z2)
                        if (z2 != z) in_others = std::max(in_others, in[i][z2]);
                    if (in_others != std::numeric_limits<int>::min()) in_others -= 2;
                    out = std::max({rec, in_neighbor, in_others});
                    if (out < 0) out = 0;
                }
                if (out != robots[i].comm_out[z]) changed = true;
                robots[i].comm_out[z] = out;
            }
        }

        // 3. Commit comm_in for readers (Alg. 1 recruitment check).
        for (int i = 0; i < n; ++i) robots[i].comm_in = in[i];

        if (!changed) { ++iters; break; }
    }
    return iters;
}

// Convenience: compute recruit values from sensed forces, then propagate.
inline int propagate_recruitment(std::vector<FireAnt3D>& robots,
                                 const std::vector<Contact>& contacts,
                                 const std::vector<double>& sensed_forces,
                                 const AlgorithmParams& algo,
                                 int max_iters = 1000) {
    return propagate(robots, contacts, recruit_values(sensed_forces, algo), max_iters);
}

// --- Algorithm 1: moving-robot role transition ------------------------------

// A moving robot is recruited when any incoming contact zone carries a recruit signal >= 1.
inline int max_comm_in(const std::vector<int>& comm_in) {
    int m = 0;
    for (int v : comm_in) m = std::max(m, v);
    return m;
}
inline bool is_recruited(const std::vector<int>& comm_in) { return max_comm_in(comm_in) >= 1; }

// Two unit direction vectors are "the same sensed direction" (Alg. 1's next==last test).
inline bool same_direction(const Vec3& a, const Vec3& b, double tol = 1e-9) {
    return (a - b).norm() <= tol;
}

// Greedy surface-walk join predicate (Q1 = greedy, DECIDED): the robot has passed the
// closest point to the goal when the best reachable next position does not strictly reduce
// distance-to-goal. Phase 5's climbing model supplies the distances; used in place of the
// literal direction test when walking a continuous surface.
inline bool passed_closest_point(double best_next_dist, double current_dist,
                                  double tol = 1e-12) {
    return best_next_dist >= current_dist - tol;
}

// Result of one moving-robot step.
struct MovingResult {
    bool still_moving = true;
    bool joined_reversed = false;   // next sensed direction == direction just moved
    bool joined_recruited = false;  // max(comm_in) >= 1
};

// Literal Algorithm 1 as a small state machine. Drive it once per climbing step with the
// freshly sensed goal direction and the robot's current comm_in. It mirrors the pseudocode:
// step, record the direction moved, re-sense; if the new sensed direction equals the one
// just moved it would reverse (join); otherwise if recruited, join; otherwise keep moving.
// The actual Sense()/StepTowards() geometry is Phase 5.
class MovingRobotFSM {
    bool have_last_ = false;
    Vec3 last_ = Vec3::Zero();

public:
    MovingResult update(const Vec3& sensed_dir, const std::vector<int>& comm_in,
                        double tol = 1e-9) {
        MovingResult r;
        if (have_last_ && same_direction(sensed_dir, last_, tol)) {
            r.still_moving = false;
            r.joined_reversed = true;
            return r;
        }
        if (is_recruited(comm_in)) {
            r.still_moving = false;
            r.joined_recruited = true;
            return r;
        }
        last_ = sensed_dir;
        have_last_ = true;
        return r;
    }
    void reset() { have_last_ = false; last_ = Vec3::Zero(); }
};

}  // namespace rb
