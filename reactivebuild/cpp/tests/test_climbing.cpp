// test_climbing.cpp -- Phase 5: greedy surface-walk (settle + climb + spawn), and the
// end-to-end check that amorphous (offset) packing produces real sensed forces while an
// aligned stack does not (the Phase-3 physics insight, and the reason random yaw matters).
#include "reactivebuild/algorithm.hpp"
#include "reactivebuild/climbing.hpp"
#include "reactivebuild/fem.hpp"
#include "reactivebuild/sensing.hpp"
#include "rb_test.hpp"

using namespace rb;
using rbtest::check;
using rbtest::checkClose;

static Support supportFrom(const std::vector<FireAnt3D>& robots) { return build_support(robots); }

int main() {
    const double r = SPHERE_RADIUS;
    Environment plane;  // PLANE, plane_z = 0

    // settle onto the bare plane: centre z = plane_z + r, anywhere.
    {
        Support empty;
        double z;
        check(settle_z(3, 4, geom::Mat3::Identity(), r, empty, plane, z), "plane always supports");
        checkClose(z, plane.plane_z + r, "settle on plane -> z = plane_z + r");
    }

    // settle resting on top of one sphere placed directly under robot sphere 0.
    // Sphere 0 body offset is (0, 2r/sqrt3, 0); put a support sphere there at z=5.
    {
        Support sup;
        auto off0 = geom::sphere_offsets(r)[0];
        sup.centers.push_back(geom::Vec3(off0.x(), off0.y(), 5.0));
        sup.recruit.push_back(0);
        sup.radius = r;
        double z;
        check(settle_z(0, 0, geom::Mat3::Identity(), r, sup, plane, z), "supported on sphere");
        checkClose(z, 5.0 + 2.0 * r, "rest directly on top of a sphere -> z = z_k + 2r");
    }

    // settle returns false over a void with no spheres (EDGE env, x past the edge).
    {
        Environment edge;
        edge.kind = EnvKind::EDGE;
        edge.edge_x = 0.0;
        Support empty;
        double z;
        check(!settle_z(5.0, 0.0, geom::Mat3::Identity(), r, empty, edge, z),
              "unsupported over a void -> settle fails");
    }

    // A lone robot on the plane with a goal straight above walks to under the goal and joins
    // at the closest point (it cannot climb on a bare plane).
    {
        Support empty;
        geom::Vec3 goal(0, 0, 20);
        auto res = climb(geom::Vec3(6, 0, r), geom::Mat3::Identity(), empty, plane, goal, r);
        check(res.joined && res.reversed, "lone robot joins at local closest point");
        check(!res.lost, "lone robot not lost");
        checkClose(res.position.z(), plane.plane_z + r, "stays on plane (cannot climb nothing)", 1e-6);
        check(std::hypot(res.position.x(), res.position.y()) < 1.0, "walks under the goal");
    }

    // A robot climbs UP an existing base toward a goal above (final height > base height).
    {
        std::vector<FireAnt3D> base = {FireAnt3D(0, geom::Vec3(0, 0, r), geom::Mat3::Identity(),
                                                 Role::STRUCTURAL)};
        Support sup = supportFrom(base);
        geom::Vec3 goal(0, 0, 20);
        std::mt19937_64 rng(7);
        auto start = spawn_on_plane(sup, plane, r, goal, rng, ClimbConfig());
        auto rot = random_yaw(rng);
        auto res = climb(start, rot, sup, plane, goal, r);
        check(res.joined && !res.lost, "climber joins");
        check(res.position.z() > 2.0 * r, "climber ends above the base (climbed up)");
    }

    // Recruitment: a sphere carrying recruit>=1 in the path makes the climber join early.
    {
        // Base robot on the plane, mark its spheres as recruiting.
        FireAnt3D b(0, geom::Vec3(0, 0, r), geom::Mat3::Identity(), Role::STRUCTURAL);
        b.comm_out.assign(b.num_zones(), 3);  // strong recruit signal
        std::vector<FireAnt3D> base = {b};
        Support sup = supportFrom(base);
        check(sup.recruit.size() == 3 && sup.recruit[0] == 3, "support carries recruit signal");
        geom::Vec3 goal(0, 0, 20);
        auto res = climb(geom::Vec3(6, 0, r), geom::Mat3::Identity(), sup, plane, goal, r);
        check(res.recruited, "climber is recruited on contact with a recruiting sphere");
    }

    // Determinism: identical seed -> identical spawn + climb result.
    {
        auto run = [&](uint64_t seed) {
            std::vector<FireAnt3D> robots;
            geom::Vec3 goal(0, 0, 20);
            std::mt19937_64 rng(seed);
            geom::Vec3 last;
            for (int i = 0; i < 5; ++i) {
                Support sup = supportFrom(robots);
                auto start = spawn_on_plane(sup, plane, r, goal, rng, ClimbConfig());
                auto rot = random_yaw(rng);
                auto res = climb(start, rot, sup, plane, goal, r);
                robots.push_back(FireAnt3D(i, res.position, res.rotation, Role::STRUCTURAL));
                last = res.position;
            }
            return last;
        };
        auto a = run(42), b = run(42);
        check((a - b).norm() < 1e-12, "same seed -> identical structure (deterministic)");
    }

    // PHYSICS INSIGHT: aligned stack -> ~0 sensed force; offset placement -> real force.
    {
        // Aligned: robot B exactly above robot A (identical yaw) -> spheres nest vertically,
        // load bypasses B's centre.
        std::vector<FireAnt3D> aligned = {
            FireAnt3D(0, geom::Vec3(0, 0, r), geom::Mat3::Identity(), Role::STRUCTURAL),
            FireAnt3D(1, geom::Vec3(0, 0, r + 2 * r), geom::Mat3::Identity(), Role::STRUCTURAL)};
        auto ac = all_contacts(aligned);
        auto aa = plane.anchor_contacts(aligned);
        check(ac.size() == 3, "aligned stack: 3 sphere-sphere contacts");
        auto am = build_robot_fem(aligned, ac, aa, plane.plane_z);
        auto asol = solve_truss(am);
        auto asf = all_sensed_forces(am, asol, aligned);
        check(asf[1] < 1e-3, "aligned upper robot senses ~0 (load bypasses centre)");

        // Offset: robot B rests on the pile shifted, via a real settle + a random yaw.
        std::vector<FireAnt3D> off = {
            FireAnt3D(0, geom::Vec3(0, 0, r), geom::Mat3::Identity(), Role::STRUCTURAL)};
        Support sup = supportFrom(off);
        std::mt19937_64 rng(3);
        auto rot = random_yaw(rng);
        double z;
        // place B's centre over A's centre but with a rotated triangle -> offset nesting.
        check(settle_z(0.4, 0.2, rot, r, sup, plane, z), "offset placement settles");
        off.push_back(FireAnt3D(1, geom::Vec3(0.4, 0.2, z), rot, Role::STRUCTURAL));
        auto oc = all_contacts(off);
        auto oa = plane.anchor_contacts(off);
        auto om = build_robot_fem(off, oc, oa, plane.plane_z);
        auto osol = solve_truss(om);
        auto osf = all_sensed_forces(om, osol, off);
        check(osf[1] > 0.1, "offset upper robot senses real force (amorphous packing)");
    }

    // End-to-end: grow a small tower and confirm it is solvable, offset, and rising.
    {
        std::vector<FireAnt3D> robots;
        geom::Vec3 goal(0, 0, 20);
        std::mt19937_64 rng(2024);
        double max_z = 0.0, max_sensed = 0.0;
        bool solvable = true;
        for (int i = 0; i < 6; ++i) {
            Support sup = supportFrom(robots);
            auto start = spawn_on_plane(sup, plane, r, goal, rng, ClimbConfig());
            auto rot = random_yaw(rng);
            auto res = climb(start, rot, sup, plane, goal, r);
            if (res.lost) continue;
            robots.push_back(FireAnt3D(i, res.position, res.rotation, Role::STRUCTURAL));
            max_z = std::max(max_z, res.position.z());
            auto c = all_contacts(robots);
            auto an = plane.anchor_contacts(robots);
            try {
                auto m = build_robot_fem(robots, c, an, plane.plane_z);
                auto sol = solve_truss(m);
                auto sf = all_sensed_forces(m, sol, robots);
                for (double v : sf) max_sensed = std::max(max_sensed, v);
            } catch (const std::exception&) {
                solvable = false;
            }
        }
        check(robots.size() >= 5, "most spawned robots joined the structure");
        check(solvable, "grown tower stays FEM-solvable at every step");
        check(max_z > 3.0 * r, "tower rises above the base");
        check(max_sensed > 1.0, "amorphous packing generates real sensed forces");
    }

    // --- Docking climber (Phase 10) ------------------------------------------

    // pose_ok: a pose whose sphere 0 rests exactly on top of a support sphere is feasible
    // (touches, no penetration); pushing it 0.1 lower penetrates and is rejected; lifting it
    // clear of everything floats and is rejected.
    {
        Support sup;
        auto off0 = geom::sphere_offsets(r)[0];
        sup.centers.push_back(geom::Vec3(off0.x(), off0.y(), 5.0));
        sup.recruit.push_back(0);
        sup.radius = r;
        sup.build_grid();
        Environment edge;  // EDGE at x=0 so "float in the void" is possible
        edge.kind = EnvKind::EDGE;
        edge.edge_x = 0.0;
        geom::Mat3 I = geom::Mat3::Identity();
        geom::Vec3 rest(0, 0, 5.0 + 2.0 * r);                    // sphere0 sits on the support
        check(pose_ok(rest, I, r, sup, edge), "dock: resting on a sphere is feasible");
        check(!pose_ok(rest - geom::Vec3(0, 0, 0.2), I, r, sup, edge),
              "dock: pushed into the sphere is rejected (penetration)");
        check(!pose_ok(geom::Vec3(50, 50, 50), I, r, sup, edge),
              "dock: floating clear of everything is rejected (no contact)");
    }

    // dock_climb hangs BELOW the frontier toward a downward goal, penetration-free. One support
    // sphere in the void; goal far below; the docked robot must place a sphere lower than the
    // anchor (it descended) and touch it without overlapping.
    {
        Environment edge;
        edge.kind = EnvKind::EDGE;
        edge.edge_x = 0.0;
        Support sup;
        sup.centers.push_back(geom::Vec3(2.0, 0.0, -3.0));       // a tip out in the void
        sup.recruit.push_back(0);
        sup.radius = r;
        sup.build_grid();
        geom::Vec3 goal(0.0, 0.0, -600.0);                      // straight down
        std::mt19937_64 rng(123);
        DockConfig dc;
        auto res = dock_climb({}, sup, edge, goal, r, rng, dc);
        check(res.joined && !res.lost, "dock: seats a robot onto the tip");
        check(pose_ok(res.position, res.rotation, r, sup, edge),
              "dock: chosen pose is penetration-free and touching");
        double lowest = 1e300;
        for (auto& sc : {res.position + res.rotation * geom::sphere_offsets(r)[0],
                         res.position + res.rotation * geom::sphere_offsets(r)[1],
                         res.position + res.rotation * geom::sphere_offsets(r)[2]})
            lowest = std::min(lowest, sc.z());
        check(lowest < -3.0, "dock: extends below the tip (descends toward the goal)");
    }

    return rbtest::summary("test_climbing");
}
