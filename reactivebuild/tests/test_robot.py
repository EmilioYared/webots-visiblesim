"""Phase 1 tests: the FireAnt3D object and scene-level contact detection."""

import numpy as np
import pytest

from reactivebuild import geometry as G
from reactivebuild import robot as RB
from reactivebuild.robot import FireAnt3D, Role


def test_construction_defaults():
    r = FireAnt3D(id=0, position=[0, 0, 0])
    assert r.role is Role.MOVING
    assert r.num_zones == 3
    assert r.radius == 1.0
    assert r.position.shape == (3,)
    assert r.rotation.shape == (3, 3)
    assert np.allclose(r.rotation, np.eye(3))


def test_comm_buffers_initialised_per_zone():
    r = FireAnt3D(id=1, position=[0, 0, 0])
    assert r.comm_in.shape == (3,) and r.comm_out.shape == (3,)
    assert np.all(r.comm_in == 0) and np.all(r.comm_out == 0)


def test_node_and_center_views():
    r = FireAnt3D(id=0, position=[1, 2, 3])
    assert r.sphere_centers().shape == (3, 3)
    assert r.all_nodes().shape == (16, 3)
    sn, cn = r.node_positions()
    assert sn.shape == (3, 4, 3) and cn.shape == (4, 3)


def test_weight_bookkeeping():
    r = FireAnt3D(id=0, position=[0, 0, 0])
    assert r.total_weight == pytest.approx(3.0)         # 3 spheres * 1.0
    assert r.sphere_node_loads() == pytest.approx(3.0)  # 12 sphere nodes * 0.25


def test_role_transition_is_settable():
    r = FireAnt3D(id=0, position=[0, 0, 0])
    r.role = Role.STRUCTURAL
    assert r.role is Role.STRUCTURAL


def test_stacked_robot_makes_three_aligned_contacts():
    # Placing robot B exactly 2r above robot A stacks each sphere i directly over
    # sphere i (distance 2r == tangent); cross-index pairs are farther apart.
    a = FireAnt3D(id=0, position=[0, 0, 0])
    b = FireAnt3D(id=1, position=[0, 0, 2.0])  # 2 * radius
    pairs = RB.robot_sphere_contacts(a, b)
    assert sorted(pairs) == [(0, 0), (1, 1), (2, 2)]


def test_distant_robots_have_no_contact():
    a = FireAnt3D(id=0, position=[0, 0, 0])
    b = FireAnt3D(id=1, position=[100, 0, 0])
    assert RB.robot_sphere_contacts(a, b) == []


def test_all_contacts_scene_indices_ordered():
    robots = [
        FireAnt3D(id=0, position=[0, 0, 0]),
        FireAnt3D(id=1, position=[0, 0, 2.0]),   # contacts robot 0
        FireAnt3D(id=2, position=[100, 0, 0]),   # isolated
    ]
    contacts = RB.all_contacts(robots)
    assert len(contacts) == 3
    for (i, sa, j, sb) in contacts:
        assert i < j                              # canonical ordering
        assert (i, j) == (0, 1)                   # only 0-1 pair touches
        assert sa == sb                           # aligned stack


def test_ground_contacts():
    robots = [
        FireAnt3D(id=0, position=[0, 0, 0.0]),    # spheres straddle z=0 -> touching
        FireAnt3D(id=1, position=[0, 0, 50.0]),   # high up -> no ground contact
    ]
    hits = RB.ground_contacts(robots, plane_z=0.0)
    assert all(i == 0 for (i, s) in hits)         # only robot 0 touches ground
    assert len(hits) >= 1
