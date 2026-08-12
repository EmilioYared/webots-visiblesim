// test_robot.cpp -- Phase 1 robot + contact tests (mirrors reference_python test_robot.py).
#include "reactivebuild/robot.hpp"
#include "rb_test.hpp"

#include <algorithm>

using namespace rb;
using rbtest::check;
using rbtest::checkNear;

int main() {
    // construction defaults
    {
        FireAnt3D r(0, geom::Vec3(0, 0, 0));
        check(r.role == Role::MOVING, "default role moving");
        check(r.num_zones() == 3, "num_zones 3");
        checkNear(r.radius(), 1.0, "radius 1.0");
        check(r.rotation.isApprox(geom::Mat3::Identity()), "default rotation identity");
    }
    // comm buffers initialised per zone
    {
        FireAnt3D r(1, geom::Vec3(0, 0, 0));
        check((int)r.comm_in.size() == 3 && (int)r.comm_out.size() == 3, "comm buffers size 3");
        check(r.comm_in[0] == 0 && r.comm_out[2] == 0, "comm buffers zeroed");
    }
    // node views
    {
        FireAnt3D r(0, geom::Vec3(1, 2, 3));
        check(r.sphere_centers().size() == 3, "3 sphere centers");
        check(r.all_nodes().size() == 16, "16 nodes");
    }
    // weight bookkeeping
    {
        FireAnt3D r(0, geom::Vec3(0, 0, 0));
        checkNear(r.total_weight(), 3.0, "total weight 3.0");
        checkNear(r.sphere_node_loads(), 3.0, "sphere node loads sum 3.0");
    }
    // role transition
    {
        FireAnt3D r(0, geom::Vec3(0, 0, 0));
        r.role = Role::STRUCTURAL;
        check(r.role == Role::STRUCTURAL, "role settable to structural");
    }
    // stacked robot -> 3 aligned contacts
    {
        FireAnt3D a(0, geom::Vec3(0, 0, 0));
        FireAnt3D b(1, geom::Vec3(0, 0, 2.0));  // 2r above
        auto pairs = robot_sphere_contacts(a, b);
        std::sort(pairs.begin(), pairs.end());
        check(pairs.size() == 3, "3 contacts when stacked");
        bool aligned = pairs.size() == 3 && pairs[0] == std::make_pair(0, 0) &&
                       pairs[1] == std::make_pair(1, 1) && pairs[2] == std::make_pair(2, 2);
        check(aligned, "aligned sphere contacts (0,0)(1,1)(2,2)");
    }
    // distant robots -> no contact
    {
        FireAnt3D a(0, geom::Vec3(0, 0, 0));
        FireAnt3D b(1, geom::Vec3(100, 0, 0));
        check(robot_sphere_contacts(a, b).empty(), "distant no contact");
    }
    // all_contacts scene ordering
    {
        std::vector<FireAnt3D> robots = {
            FireAnt3D(0, geom::Vec3(0, 0, 0)),
            FireAnt3D(1, geom::Vec3(0, 0, 2.0)),
            FireAnt3D(2, geom::Vec3(100, 0, 0)),
        };
        auto contacts = all_contacts(robots);
        check(contacts.size() == 3, "scene has 3 contacts");
        bool ok = true;
        for (auto& c : contacts) {
            if (!(c.ri < c.rj)) ok = false;      // canonical order
            if (!(c.ri == 0 && c.rj == 1)) ok = false;  // only 0-1 touch
            if (c.si != c.sj) ok = false;        // aligned
        }
        check(ok, "contacts ordered i<j, only 0-1, aligned");
    }
    // ground contacts
    {
        std::vector<FireAnt3D> robots = {
            FireAnt3D(0, geom::Vec3(0, 0, 0.0)),
            FireAnt3D(1, geom::Vec3(0, 0, 50.0)),
        };
        auto hits = ground_contacts(robots, 0.0);
        bool onlyBottom = !hits.empty();
        for (auto& h : hits) if (h.first != 0) onlyBottom = false;
        check(onlyBottom, "only bottom robot touches ground");
    }
    return rbtest::summary("test_robot");
}
