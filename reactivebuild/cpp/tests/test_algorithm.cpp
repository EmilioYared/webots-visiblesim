// test_algorithm.cpp -- Phase 4: ReactiveBuild algorithm (Alg. 1 & 2, Fig. 2).
//
// The correctness gate the plan calls for is Fig. 2: recruit values propagate through the
// structure, decrementing by 1 per hop, and the recruitment region shrinks when the source
// stress (recruit value) drops. Fig. 2 is an abstract "spheres with numbers" illustration,
// so we test the propagation on hand-built contact graphs (geometry-independent) and check
// the exact hop-decrement cascade -- inter-robot hop = -1, intra-robot zone hop = -1.
#include "reactivebuild/algorithm.hpp"
#include "rb_test.hpp"

using namespace rb;
using rbtest::check;
using rbtest::checkClose;

// Build n robots (positions irrelevant to propagation) with 3 zones each, all structural.
static std::vector<FireAnt3D> chainRobots(int n) {
    std::vector<FireAnt3D> robots;
    for (int i = 0; i < n; ++i) {
        robots.push_back(FireAnt3D(i, geom::Vec3(0, 0, 1.0 + 3.0 * i)));
        robots.back().role = Role::STRUCTURAL;
    }
    return robots;
}

int main() {
    // recruit_values batch matches AlgorithmParams::recruit_value (F=10,B=3,J=5 default).
    {
        AlgorithmParams a;  // F=10, B=3, J=5
        std::vector<double> sensed = {0.0, 10.0, 20.0, 30.0, 1e9};
        auto rv = recruit_values(sensed, a);
        check(rv.size() == 5, "recruit_values size");
        check(rv[0] == 0, "recruit 0 below F");
        check(rv[1] == a.recruit_value(10.0), "recruit at F matches scalar");
        check(rv[1] == 1, "recruit == 1 at sensed==F (B^0)");
        check(rv[4] == a.J, "recruit capped at J for huge force");
    }

    // Fig. 2 cascade: chain R0.z0-R1.z1, R1.z2-R2.z0, R2.z1-R3.z2. Source R0 recruits 3.
    // Golden values verified against an independent Python prototype of Alg. 2.
    std::vector<Contact> contacts = {{0, 0, 1, 1}, {1, 2, 2, 0}, {2, 1, 3, 2}};
    {
        auto robots = chainRobots(4);
        std::vector<int> recruit = {3, 0, 0, 0};
        int sweeps = propagate(robots, contacts, recruit);
        check(sweeps > 0 && sweeps < 1000, "propagation converges");

        // Source robot outputs its recruit value on every zone.
        check(robots[0].comm_out == std::vector<int>({3, 3, 3}), "R0 emits 3 on all zones");
        // 1 inter-robot hop -> 2 ; +1 intra-robot hop -> 1.
        check(robots[1].comm_out == std::vector<int>({1, 2, 1}), "R1 relays 2 (1 hop) / 1 (2 hops)");
        // 3 hops from source -> 0: region ends here.
        check(robots[2].comm_out == std::vector<int>({0, 0, 0}), "R2 at 3 hops -> 0 (region ends)");
        check(robots[3].comm_out == std::vector<int>({0, 0, 0}), "R3 beyond region -> 0");
        // R2 still RECEIVES a 1 (from R1.z2) even though it re-emits 0.
        check(robots[2].comm_in[0] == 1, "R2 receives 1 on the contacting zone");
    }

    // Panels (e)->(h): source stress drops (recruit 3 -> 2); recruitment region shrinks by
    // one hop.
    {
        auto robots = chainRobots(4);
        std::vector<int> recruit = {2, 0, 0, 0};
        propagate(robots, contacts, recruit);
        check(robots[0].comm_out == std::vector<int>({2, 2, 2}), "R0 emits 2");
        check(robots[1].comm_out == std::vector<int>({0, 1, 0}), "region shrank: R1 only z1==1");
        check(robots[2].comm_out == std::vector<int>({0, 0, 0}), "R2 -> 0 after shrink");
    }

    // Hop accounting in isolation: one inter-robot contact = -1; crossing to another zone of
    // the same robot = an extra -1 (total -2), exactly Alg. 2's terms.
    {
        auto robots = chainRobots(2);
        std::vector<Contact> c2 = {{0, 0, 1, 0}};  // R0.z0 <-> R1.z0
        std::vector<int> recruit = {4, 0};
        propagate(robots, c2, recruit);
        check(robots[1].comm_out[0] == 3, "inter-robot hop decrements by 1 (4->3)");
        check(robots[1].comm_out[1] == 2, "same robot, other zone: extra -1 (4->2)");
        check(robots[1].comm_out[2] == 2, "same robot, other zone: extra -1 (4->2)");
    }

    // A MOVING robot climbing over the structure never relays a signal, but does receive one.
    {
        auto robots = chainRobots(2);
        robots[1].role = Role::MOVING;
        std::vector<Contact> c2 = {{0, 0, 1, 0}};
        std::vector<int> recruit = {5, 0};
        propagate(robots, c2, recruit);
        check(robots[1].comm_out == std::vector<int>({0, 0, 0}), "moving robot emits nothing");
        check(robots[1].comm_in[0] == 5, "moving robot still receives (for recruitment check)");
        check(is_recruited(robots[1].comm_in), "moving robot is recruited (comm_in>=1)");
    }

    // Idempotence: re-running propagation from the converged state gives the same result.
    {
        auto a = chainRobots(4);
        auto b = chainRobots(4);
        std::vector<int> recruit = {3, 0, 0, 0};
        propagate(a, contacts, recruit);
        propagate(b, contacts, recruit);
        bool same = true;
        for (int i = 0; i < 4; ++i) same = same && (a[i].comm_out == b[i].comm_out);
        check(same, "propagation is deterministic / idempotent");
    }

    // A robot with recruit 0 and no non-zero neighbours emits nothing.
    {
        auto robots = chainRobots(4);
        std::vector<int> recruit = {0, 0, 0, 0};
        propagate(robots, contacts, recruit);
        bool allZero = true;
        for (auto& r : robots)
            for (int v : r.comm_out) allZero = allZero && (v == 0);
        check(allZero, "no stress anywhere -> no recruitment");
    }

    // --- Algorithm 1: moving-robot transition ---
    {
        check(!is_recruited({0, 0, 0}), "not recruited when all comm_in 0");
        check(is_recruited({0, 1, 0}), "recruited when any comm_in >= 1");
        check(is_recruited({3, 0, 2}), "recruited (max >= 1)");
        check(max_comm_in({0, 4, 2}) == 4, "max_comm_in");
    }

    // greedy surface-walk join predicate: passed the closest point when no step improves.
    {
        check(passed_closest_point(5.0, 4.0), "no forward progress -> join");
        check(passed_closest_point(4.0, 4.0), "equal distance -> join (local min)");
        check(!passed_closest_point(3.0, 4.0), "step reduces distance -> keep moving");
    }

    // MovingRobotFSM: keep moving while the sensed direction changes and not recruited;
    // join on repeated (reversed) direction; join on recruitment.
    {
        MovingRobotFSM fsm;
        geom::Vec3 dA(1, 0, 0), dB(0, 1, 0);
        std::vector<int> none = {0, 0, 0};
        auto r1 = fsm.update(dB, none);  // first sense: record, move
        check(r1.still_moving, "FSM: first step keeps moving");
        auto r2 = fsm.update(dA, none);  // direction changed: keep moving
        check(r2.still_moving, "FSM: changed direction keeps moving");
        auto r3 = fsm.update(dA, none);  // same direction as just moved: reverse -> join
        check(!r3.still_moving && r3.joined_reversed, "FSM: repeated direction -> join (reversed)");
    }
    {
        MovingRobotFSM fsm;
        geom::Vec3 dA(1, 0, 0), dB(0, 1, 0);
        fsm.update(dA, {0, 0, 0});                    // move
        auto r = fsm.update(dB, {0, 2, 0});           // recruited before reversing
        check(!r.still_moving && r.joined_recruited, "FSM: recruitment -> join");
    }

    return rbtest::summary("test_algorithm");
}
