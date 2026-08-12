// test_environment.cpp -- Phase 5: static environment (plane / edge / gap) + goal placement.
#include "reactivebuild/environment.hpp"
#include "rb_test.hpp"

using namespace rb;
using rbtest::check;
using rbtest::checkClose;

int main() {
    // make_environment maps each experiment to the right support geometry.
    {
        auto tower = make_environment(ExperimentParams{});  // defaults to TOWER
        check(tower.kind == EnvKind::PLANE, "tower -> PLANE");

        ExperimentParams ce;
        ce.type = ExperimentType::CANTILEVER;
        auto cant = make_environment(ce);
        check(cant.kind == EnvKind::EDGE, "cantilever -> EDGE");
        check(cant.edge_x == 0.0, "edge at x=0");

        ExperimentParams be;
        be.type = ExperimentType::BRIDGE;
        be.gap_width = 20.0;
        auto br = make_environment(be);
        check(br.kind == EnvKind::GAP, "bridge -> GAP");
        checkClose(br.gap, 20.0, "gap width propagated");
    }

    // has_support / support_z.
    {
        Environment plane;  // PLANE, plane_z=0
        check(plane.has_support(100, -50), "plane supports everywhere");
        checkClose(plane.support_z(3, 4), 0.0, "plane support_z = plane_z");

        Environment edge;
        edge.kind = EnvKind::EDGE;
        edge.edge_x = 0.0;
        check(edge.has_support(-1, 0), "edge: x<0 supported");
        check(!edge.has_support(1, 0), "edge: x>0 unsupported");
        check(edge.support_z(1, 0) == Environment::NO_SUPPORT, "edge void -> NO_SUPPORT");

        Environment gap;
        gap.kind = EnvKind::GAP;
        gap.gap = 20.0;
        check(gap.has_support(-15, 0) && gap.has_support(15, 0), "gap: both banks supported");
        check(!gap.has_support(0, 0), "gap: middle unsupported");
    }

    // sphere_touches.
    {
        Environment plane;
        check(plane.sphere_touches(geom::Vec3(0, 0, 1.0), 1.0), "sphere resting on plane touches");
        check(!plane.sphere_touches(geom::Vec3(0, 0, 3.0), 1.0), "lifted sphere does not touch");
    }

    // anchor_contacts collects env-touching spheres.
    {
        Environment plane;
        std::vector<FireAnt3D> robots = {FireAnt3D(0, geom::Vec3(0, 0, 1.0)),
                                         FireAnt3D(1, geom::Vec3(0, 0, 5.0))};
        auto a = plane.anchor_contacts(robots);
        check(a.size() == 3, "robot on plane anchors its 3 spheres");
        for (auto& p : a) check(p.first == 0, "only the grounded robot anchors");
    }

    // goal_position: BASE vs EDGE reference.
    {
        ExperimentParams tower;  // TOWER, goal (0,0,65), BASE
        auto env = make_environment(tower);
        auto g = goal_position(tower, env);
        checkClose(g.z(), 65.0, "tower goal 65 up from base");

        ExperimentParams cant;
        cant.type = ExperimentType::CANTILEVER;
        cant.goal_offset = geom::Vec3(45, 0, 10);
        cant.goal_reference = GoalReference::EDGE;
        auto cenv = make_environment(cant);
        auto cg = goal_position(cant, cenv);
        checkClose(cg.x(), 45.0, "cantilever goal 45 out from edge (x=0)");
        checkClose(cg.z(), 10.0, "cantilever goal 10 up from edge");
    }

    return rbtest::summary("test_environment");
}
