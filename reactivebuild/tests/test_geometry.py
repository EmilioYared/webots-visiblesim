"""Phase 1 tests: FireAnt3D geometry primitives."""

import numpy as np
import pytest

from reactivebuild import geometry as G
from reactivebuild.config import RobotParams


def test_three_spheres_coplanar_and_centroid_at_origin():
    offs = G.sphere_offsets(1.0)
    assert offs.shape == (3, 3)
    assert np.allclose(offs[:, 2], 0.0)                 # coplanar in xy
    assert np.allclose(offs.mean(axis=0), 0.0)          # centroid == robot center


def test_spheres_mutually_tangent_side_is_two_radii():
    r = 1.0
    offs = G.sphere_offsets(r)
    d01 = np.linalg.norm(offs[0] - offs[1])
    d12 = np.linalg.norm(offs[1] - offs[2])
    d20 = np.linalg.norm(offs[2] - offs[0])
    assert d01 == pytest.approx(2 * r)                  # tangent
    assert d01 == pytest.approx(d12) == pytest.approx(d20)  # equilateral


def test_tetra_is_regular_with_given_circumradius():
    scale = 0.1
    tet = G.tetra_offsets(scale)
    assert tet.shape == (4, 3)
    assert np.allclose(tet.mean(axis=0), 0.0)           # centroid at origin
    assert np.allclose(np.linalg.norm(tet, axis=1), scale)  # circumradius == scale
    dists = [np.linalg.norm(tet[i] - tet[j]) for i in range(4) for j in range(i + 1, 4)]
    assert np.allclose(dists, dists[0])                 # all 6 edges equal (regular)


def test_robot_has_16_nodes_grouped_correctly():
    rp = RobotParams()
    sphere_nodes, center_nodes = G.robot_node_positions([0, 0, 0], np.eye(3), rp)
    assert sphere_nodes.shape == (3, 4, 3)
    assert center_nodes.shape == (4, 3)
    assert G.all_node_positions([0, 0, 0], np.eye(3), rp).shape == (16, 3)


def test_sphere_node_clusters_centered_on_sphere_centers():
    rp = RobotParams()
    pos = np.array([2.0, -3.0, 5.0])
    sphere_nodes, center_nodes = G.robot_node_positions(pos, np.eye(3), rp)
    centers = G.sphere_centers(pos, np.eye(3), rp)
    for i in range(3):
        assert np.allclose(sphere_nodes[i].mean(axis=0), centers[i])
    assert np.allclose(center_nodes.mean(axis=0), pos)  # center cluster at robot center


def test_pose_is_rigid_distances_preserved():
    rp = RobotParams()
    base = G.all_node_positions([0, 0, 0], np.eye(3), rp)
    R = G.rotation_from_axis_angle([0, 0, 1], 0.7)
    moved = G.all_node_positions([10, -4, 3], R, rp)
    # all pairwise distances identical under rotation + translation
    def pdist(x):
        d = x[:, None, :] - x[None, :, :]
        return np.linalg.norm(d, axis=2)
    assert np.allclose(pdist(base), pdist(moved))


def test_rotation_about_z_90deg():
    R = G.rotation_from_axis_angle([0, 0, 1], np.pi / 2)
    assert np.allclose(R @ np.array([1, 0, 0]), [0, 1, 0], atol=1e-12)


def test_spheres_contact_primitive():
    assert G.spheres_contact([0, 0, 0], 1.0, [2.0, 0, 0], 1.0)        # tangent
    assert G.spheres_contact([0, 0, 0], 1.0, [1.5, 0, 0], 1.0)        # overlapping
    assert not G.spheres_contact([0, 0, 0], 1.0, [2.5, 0, 0], 1.0)    # apart


def test_sphere_plane_contact_primitive():
    assert G.sphere_plane_contact([0, 0, 1.0], 1.0, plane_z=0.0)      # resting on plane
    assert G.sphere_plane_contact([0, 0, 0.5], 1.0)                   # penetrating
    assert not G.sphere_plane_contact([0, 0, 2.0], 1.0)               # floating
