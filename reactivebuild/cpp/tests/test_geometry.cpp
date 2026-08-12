// test_geometry.cpp -- Phase 1 geometry tests (mirrors reference_python test_geometry.py).
#include "reactivebuild/geometry.hpp"
#include "rb_test.hpp"

using namespace rb;
using namespace rb::geom;
using rbtest::check;
using rbtest::checkClose;
using rbtest::checkNear;

static double pdist_maxdiff(const std::array<Vec3, 16>& a, const std::array<Vec3, 16>& b) {
    double worst = 0.0;
    for (int i = 0; i < 16; ++i)
        for (int j = i + 1; j < 16; ++j) {
            double da = (a[i] - a[j]).norm(), db = (b[i] - b[j]).norm();
            worst = std::max(worst, std::fabs(da - db));
        }
    return worst;
}

int main() {
    // three spheres coplanar + centroid at origin
    {
        auto offs = sphere_offsets(1.0);
        Vec3 c = (offs[0] + offs[1] + offs[2]) / 3.0;
        check(std::fabs(offs[0].z()) < 1e-12 && std::fabs(offs[1].z()) < 1e-12 &&
              std::fabs(offs[2].z()) < 1e-12, "spheres coplanar (z=0)");
        check(c.norm() < 1e-12, "sphere centroid at origin");
    }
    // mutually tangent, equilateral, side 2r
    {
        auto o = sphere_offsets(1.0);
        double d01 = (o[0] - o[1]).norm(), d12 = (o[1] - o[2]).norm(), d20 = (o[2] - o[0]).norm();
        checkClose(d01, 2.0, "sphere side == 2r");
        checkClose(d01, d12, "equilateral 01==12");
        checkClose(d12, d20, "equilateral 12==20");
    }
    // regular tetra, circumradius == scale, equal edges
    {
        double scale = 0.1;
        auto tet = tetra_offsets(scale);
        Vec3 c = (tet[0] + tet[1] + tet[2] + tet[3]) / 4.0;
        check(c.norm() < 1e-12, "tetra centroid at origin");
        for (int i = 0; i < 4; ++i) checkClose(tet[i].norm(), scale, "tetra circumradius");
        double e0 = (tet[0] - tet[1]).norm();
        bool eq = true;
        for (int i = 0; i < 4; ++i)
            for (int j = i + 1; j < 4; ++j)
                if (std::fabs((tet[i] - tet[j]).norm() - e0) > 1e-9) eq = false;
        check(eq, "tetra edges all equal (regular)");
    }
    // 16 nodes grouped; clusters centered correctly
    {
        RobotParams rp;
        Vec3 pos(2, -3, 5);
        auto rn = robot_node_positions(pos, Mat3::Identity(), rp);
        auto centers = sphere_centers(pos, Mat3::Identity(), rp);
        for (int s = 0; s < 3; ++s) {
            Vec3 mean = (rn.sphere_nodes[s][0] + rn.sphere_nodes[s][1] +
                         rn.sphere_nodes[s][2] + rn.sphere_nodes[s][3]) / 4.0;
            check((mean - centers[s]).norm() < 1e-9, "sphere cluster centered on center");
        }
        Vec3 cmean = (rn.center_nodes[0] + rn.center_nodes[1] + rn.center_nodes[2] +
                      rn.center_nodes[3]) / 4.0;
        check((cmean - pos).norm() < 1e-9, "center cluster at robot center");
        auto all = all_node_positions(pos, Mat3::Identity(), rp);
        check(all.size() == 16, "16 nodes total");
    }
    // rigid pose: all pairwise distances preserved under rotation + translation
    {
        RobotParams rp;
        auto base = all_node_positions(Vec3(0, 0, 0), Mat3::Identity(), rp);
        Mat3 R = rotation_from_axis_angle(Vec3(0, 0, 1), 0.7);
        auto moved = all_node_positions(Vec3(10, -4, 3), R, rp);
        check(pdist_maxdiff(base, moved) < 1e-9, "pose is rigid (distances preserved)");
    }
    // rotation about z by 90 deg
    {
        Mat3 R = rotation_from_axis_angle(Vec3(0, 0, 1), PI / 2.0);
        Vec3 r = R * Vec3(1, 0, 0);
        check((r - Vec3(0, 1, 0)).norm() < 1e-12, "rot z 90 maps x->y");
    }
    // contact primitives
    {
        check(spheres_contact(Vec3(0, 0, 0), 1.0, Vec3(2, 0, 0), 1.0), "tangent contact");
        check(spheres_contact(Vec3(0, 0, 0), 1.0, Vec3(1.5, 0, 0), 1.0), "overlap contact");
        check(!spheres_contact(Vec3(0, 0, 0), 1.0, Vec3(2.5, 0, 0), 1.0), "apart no contact");
        check(sphere_plane_contact(Vec3(0, 0, 1.0), 1.0, 0.0), "sphere resting on plane");
        check(sphere_plane_contact(Vec3(0, 0, 0.5), 1.0), "sphere penetrating plane");
        check(!sphere_plane_contact(Vec3(0, 0, 2.0), 1.0), "sphere floating");
    }
    return rbtest::summary("test_geometry");
}
