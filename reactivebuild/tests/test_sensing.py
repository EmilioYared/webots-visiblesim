"""Phase 3 tests: sensed_force and connection-stress reconstruction.

The bundle_resultant tests use hand-built models + hand-chosen axial forces (independent
of the solver) so the force/moment decomposition is checked against closed-form answers.
The integration tests then exercise the full build -> solve -> sense pipeline, including a
golden-value regression sentinel so an accidental change to the model is caught.
"""

import numpy as np
import pytest

from reactivebuild import fem, sensing
from reactivebuild.fem import (TrussModel, TrussResult, build_robot_fem,
                               connection_bundles, solve_truss)
from reactivebuild.robot import FireAnt3D, all_contacts, ground_contacts


def _fake_result(axial):
    """A TrussResult carrying only chosen axial forces (other fields unused here)."""
    axial = np.asarray(axial, float)
    z = np.zeros((1, 3))
    return TrussResult(displacements=z, axial=axial, lengths=np.ones(len(axial)),
                       reactions=z, k=np.ones(len(axial)))


# --- direct reconstruction: a pure couple -> pure bending --------------------

def test_bundle_pure_couple_is_bending():
    # Two parallel elements along +x, offset +-d in y, carrying opposite axial forces.
    # Net force cancels; the offset forces make a couple -> bending 2*d*P, no axial/shear.
    d, L, P = 0.5, 4.0, 30.0
    nodes = np.array([[0, +d, 0], [0, -d, 0],       # sphere-side (group_a: 0,1)
                      [L, +d, 0], [L, -d, 0]], float)  # center-side (group_b: 2,3)
    elements = np.array([[0, 2], [1, 3]])
    model = TrussModel(nodes, elements, np.array([1.0, 1.0]),
                       np.zeros((4, 3), bool), np.zeros((4, 3)))
    res = _fake_result([+P, -P])
    r = sensing.bundle_resultant(model, res, [0, 1], [2, 3])
    assert r.axial == pytest.approx(0.0, abs=1e-9)
    assert r.shear == pytest.approx(0.0, abs=1e-9)
    assert r.torsion == pytest.approx(0.0, abs=1e-9)
    assert r.bending == pytest.approx(2 * d * P, rel=1e-9)


def test_bundle_symmetric_tension_is_pure_axial():
    # Same geometry, both elements in equal tension: net axial 2P (pull toward sphere),
    # no bending (symmetric), no shear/torsion.
    d, L, P = 0.5, 4.0, 30.0
    nodes = np.array([[0, +d, 0], [0, -d, 0], [L, +d, 0], [L, -d, 0]], float)
    elements = np.array([[0, 2], [1, 3]])
    model = TrussModel(nodes, elements, np.array([1.0, 1.0]),
                       np.zeros((4, 3), bool), np.zeros((4, 3)))
    r = sensing.bundle_resultant(model, _fake_result([P, P]), [0, 1], [2, 3])
    assert abs(r.axial) == pytest.approx(2 * P, rel=1e-9)
    assert r.bending == pytest.approx(0.0, abs=1e-9)
    assert r.shear == pytest.approx(0.0, abs=1e-9)


def test_bundle_axial_sign_tension_pulls_groups_together():
    # Single element along +x in tension: force on the b-side points back toward a (-x),
    # so the axial component (along a->b = +x) is negative.
    nodes = np.array([[0, 0, 0], [2.0, 0, 0]], float)
    model = TrussModel(nodes, np.array([[0, 1]]), np.array([1.0]),
                       np.zeros((2, 3), bool), np.zeros((2, 3)))
    r = sensing.bundle_resultant(model, _fake_result([+10.0]), [0], [1])
    assert r.axial == pytest.approx(-10.0, rel=1e-9)


# --- sensed_force integration ------------------------------------------------

def _single_robot_model(gravity=(0, 0, -1.0)):
    r = FireAnt3D(id=0, position=[0, 0, 1.0])
    gc = ground_contacts([r], plane_z=0.0)
    model = build_robot_fem([r], ground_anchors=gc, plane_z=0.0, gravity=gravity)
    return [r], model, gc


def test_sensed_force_zero_gravity_is_zero():
    robots, model, _ = _single_robot_model(gravity=(0, 0, 0))
    res = solve_truss(model)
    assert sensing.robot_sensed_force(model, res, 0) == pytest.approx(0.0, abs=1e-9)


def test_sensed_force_positive_under_gravity():
    robots, model, _ = _single_robot_model()
    res = solve_truss(model)
    s = sensing.robot_sensed_force(model, res, 0)
    assert s > 0 and np.isfinite(s)


def test_cantilever_connections_mirror_symmetric():
    # In the one-sphere-anchored cantilever, spheres 1 and 2 sit symmetrically about the
    # plane through sphere 0 and the center, so their connections must carry identical
    # bending and identical |axial|+|bending| (robust O(1) symmetry, unlike the near-zero
    # 3-anchor case which sits at the solver noise floor).
    _, model, res = _cantilever_robot()
    center = fem.center_node_indices(0)
    def contrib(s):
        r = sensing.bundle_resultant(model, res, fem.sphere_node_indices(0, s), center)
        return r.bending, abs(r.axial) + abs(r.bending)
    b1, c1 = contrib(1)
    b2, c2 = contrib(2)
    assert b1 == pytest.approx(b2, rel=1e-6)
    assert c1 == pytest.approx(c2, rel=1e-6)


def test_all_sensed_forces_shape_and_stack_monotonic():
    robots = [FireAnt3D(id=i, position=[0, 0, 1.0 + 2.0 * i]) for i in range(4)]
    contacts = all_contacts(robots)
    gc = ground_contacts(robots, plane_z=0.0)
    model = build_robot_fem(robots, robot_contacts=contacts, ground_anchors=gc,
                            plane_z=0.0)
    res = solve_truss(model)
    s = sensing.all_sensed_forces(model, res, robots)
    assert s.shape == (4,)
    # bottom robot supports the whole stack -> feels the most; top robot the least.
    assert s[0] == max(s) and s[-1] == min(s)


def _cantilever_robot():
    # Anchor only sphere 0: spheres 1 and 2 cantilever, forcing load through the center
    # bundles (a non-degenerate, statically clean case -> robust golden value).
    r = FireAnt3D(id=0, position=[0, 0, 1.0])
    model = build_robot_fem([r], ground_anchors=[(0, 0)], plane_z=0.0)
    return r, model, solve_truss(model)


def test_cantilever_shear_matches_statics():
    # Transverse (shear) load carried at each sphere-center connection must equal the
    # weight it supports: the anchored connection carries the 2 free spheres (2.0), each
    # free connection carries its own sphere (1.0). Pure statics -- no tolerance games.
    _, model, res = _cantilever_robot()
    shear = [sensing.bundle_resultant(model, res, fem.sphere_node_indices(0, s),
                                      fem.center_node_indices(0)).shear for s in range(3)]
    assert shear[0] == pytest.approx(2.0, rel=1e-6)
    assert shear[1] == pytest.approx(1.0, rel=1e-6)
    assert shear[2] == pytest.approx(1.0, rel=1e-6)


def test_sensed_force_golden_regression():
    # Sentinel: this fixed scene must keep producing the same sensed_force.
    # Captured 2026-07-29 (== 2/sqrt(3), set by geometry x weight). If it changes, the
    # FEM/sensing model changed -- confirm the change was intentional before updating it.
    _, model, res = _cantilever_robot()
    s = sensing.robot_sensed_force(model, res, 0)
    assert s == pytest.approx(1.1547005384, rel=1e-6)


# --- connection stress -------------------------------------------------------

def test_connection_stress_zero_and_positive():
    robots, model_zero, gc = _single_robot_model(gravity=(0, 0, 0))
    res0 = solve_truss(model_zero)
    bundles = connection_bundles(robots, None, gc)
    assert sensing.peak_connection_stress(model_zero, res0, bundles) == pytest.approx(0.0, abs=1e-9)

    robots, model_g, gc = _single_robot_model()
    resg = solve_truss(model_g)
    bundles = connection_bundles(robots, None, gc)
    assert sensing.peak_connection_stress(model_g, resg, bundles) > 0


def test_peak_connection_stress_empty_is_zero():
    robots, model, _ = _single_robot_model()
    res = solve_truss(model)
    assert sensing.peak_connection_stress(model, res, []) == 0.0
