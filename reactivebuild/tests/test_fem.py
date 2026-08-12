"""Phase 2 tests: the 3D truss FEM solver.

Validation against closed-form / textbook cases (not just smoke tests), per the plan's
Phase 2 done-criterion "matches analytic cases to <1e-6 relative".
"""

import numpy as np
import pytest

from reactivebuild import fem
from reactivebuild.fem import ElementClass, TrussModel, build_robot_fem, solve_truss
from reactivebuild.config import FEMParams
from reactivebuild.robot import FireAnt3D, all_contacts, ground_contacts


def _fixed_mask(n, fixed_dofs):
    m = np.zeros((n, 3), dtype=bool)
    for (node, dof) in fixed_dofs:
        m[node, dof] = True
    return m


# --- 1. single axial element: u = P L / EA, N = P (tension) ------------------

def test_single_element_axial():
    EA, L, P = 2.0e6, 3.0, 500.0
    nodes = np.array([[0.0, 0, 0], [L, 0, 0]])
    elements = np.array([[0, 1]])
    rigidity = np.array([EA])
    # fix node 0 fully; node 1 free in x, fixed in y,z (no transverse stiffness)
    fixed = _fixed_mask(2, [(0, 0), (0, 1), (0, 2), (1, 1), (1, 2)])
    loads = np.zeros((2, 3))
    loads[1, 0] = P
    r = solve_truss(TrussModel(nodes, elements, rigidity, fixed, loads))
    assert r.displacements[1, 0] == pytest.approx(P * L / EA, rel=1e-9)
    assert r.axial[0] == pytest.approx(P, rel=1e-9)          # tension positive


def test_single_element_compression_sign():
    EA, L, P = 1.0e6, 2.0, -300.0                            # push node 1 toward node 0
    nodes = np.array([[0.0, 0, 0], [L, 0, 0]])
    fixed = _fixed_mask(2, [(0, 0), (0, 1), (0, 2), (1, 1), (1, 2)])
    loads = np.zeros((2, 3)); loads[1, 0] = P
    r = solve_truss(TrussModel(nodes, np.array([[0, 1]]), np.array([EA]), fixed, loads))
    assert r.axial[0] < 0                                    # compression negative


# --- 2. symmetric two-bar truss (determinate, hand solution) -----------------
# Apex at origin, supports at (+-3,0,4). Downward load P at apex.
# N_each = 5P/8; apex z-disp = -125 P /(32 EA); apex x-disp = 0 by symmetry.

def test_two_bar_symmetric_truss():
    EA, P = 1.0e7, 800.0
    nodes = np.array([[0.0, 0, 0], [-3.0, 0, 4.0], [3.0, 0, 4.0]])
    elements = np.array([[0, 1], [0, 2]])
    rigidity = np.array([EA, EA])
    # apex (node 0): free in x,z, fixed in y (planar). Supports fully fixed.
    fixed = _fixed_mask(3, [(0, 1), (1, 0), (1, 1), (1, 2), (2, 0), (2, 1), (2, 2)])
    loads = np.zeros((3, 3)); loads[0, 2] = -P
    r = solve_truss(TrussModel(nodes, elements, rigidity, fixed, loads))

    assert r.axial[0] == pytest.approx(5 * P / 8, rel=1e-9)
    assert r.axial[1] == pytest.approx(5 * P / 8, rel=1e-9)     # equal by symmetry
    assert r.displacements[0, 0] == pytest.approx(0.0, abs=1e-9)   # apex x == 0
    assert r.displacements[0, 2] == pytest.approx(-125 * P / (32 * EA), rel=1e-9)
    assert r.displacements[0, 2] < 0                              # downward


# --- 3. global equilibrium: sum of reactions == -sum of applied loads --------

def test_global_equilibrium_reactions():
    EA, P = 1.0e7, 800.0
    nodes = np.array([[0.0, 0, 0], [-3.0, 0, 4.0], [3.0, 0, 4.0]])
    elements = np.array([[0, 1], [0, 2]])
    fixed = _fixed_mask(3, [(0, 1), (1, 0), (1, 1), (1, 2), (2, 0), (2, 1), (2, 2)])
    loads = np.zeros((3, 3)); loads[0, 2] = -P
    r = solve_truss(TrussModel(nodes, elements, np.array([EA, EA]), fixed, loads))
    assert np.allclose(r.reactions.sum(axis=0), -loads.sum(axis=0), atol=1e-6)
    assert r.reactions.sum(axis=0)[2] == pytest.approx(P, rel=1e-9)  # holds up the load


# --- 4. length_normalized flag: k=EA/L vs raw k ------------------------------

def test_length_normalized_flag():
    EA, L, P = 2.0e6, 4.0, 500.0
    nodes = np.array([[0.0, 0, 0], [L, 0, 0]])
    fixed = _fixed_mask(2, [(0, 0), (0, 1), (0, 2), (1, 1), (1, 2)])
    loads = np.zeros((2, 3)); loads[1, 0] = P
    m = TrussModel(nodes, np.array([[0, 1]]), np.array([EA]), fixed, loads)
    u_norm = solve_truss(m, length_normalized=True).displacements[1, 0]
    u_raw = solve_truss(m, length_normalized=False).displacements[1, 0]
    assert u_norm == pytest.approx(P * L / EA, rel=1e-9)   # k = EA/L
    assert u_raw == pytest.approx(P / EA, rel=1e-9)        # k = EA (length ignored)


# --- 5. singular / under-constrained system is reported ----------------------

@pytest.mark.filterwarnings("ignore::scipy.sparse.linalg.MatrixRankWarning")
def test_underconstrained_raises():
    # one element, node 1 free in all 3 DOFs -> no transverse stiffness -> singular
    nodes = np.array([[0.0, 0, 0], [1.0, 0, 0]])
    fixed = _fixed_mask(2, [(0, 0), (0, 1), (0, 2)])
    loads = np.zeros((2, 3)); loads[1, 1] = 10.0   # transverse load, unresistable
    with pytest.raises((np.linalg.LinAlgError, Exception)):
        solve_truss(TrussModel(nodes, np.array([[0, 1]]), np.array([1e6]), fixed, loads))


# =============================================================================
# build_robot_fem (section 3, Fig. 3)
# =============================================================================

def test_robot_fem_element_counts_single_robot():
    r = FireAnt3D(id=0, position=[0, 0, 1.0])              # resting: sphere bottoms at z=0
    gc = ground_contacts([r], plane_z=0.0)                # all 3 spheres touch
    model = build_robot_fem([r], ground_anchors=gc, plane_z=0.0)

    n_anchors = len(gc)
    # nodes: 16 per robot + 4 per env anchor
    assert model.n_nodes == 16 + 4 * n_anchors
    # in-sphere: 4 tetras * 6 edges (robot) + 6 per env tetra
    in_sphere = np.sum(model.element_class == int(ElementClass.IN_SPHERE))
    assert in_sphere == 4 * 6 + 6 * n_anchors
    # robot-structure: 3 spheres * 16
    structure = np.sum(model.element_class == int(ElementClass.ROBOT_STRUCTURE))
    assert structure == 3 * 16
    # connection: 16 per ground anchor (no robot-robot contacts here)
    connection = np.sum(model.element_class == int(ElementClass.CONNECTION))
    assert connection == 16 * n_anchors


def test_robot_fem_fixed_nodes_have_zero_displacement():
    r = FireAnt3D(id=0, position=[0, 0, 1.0])
    gc = ground_contacts([r], plane_z=0.0)
    model = build_robot_fem([r], ground_anchors=gc, plane_z=0.0)
    res = solve_truss(model)
    fixed = model.fixed.any(axis=1)
    assert np.allclose(res.displacements[fixed], 0.0)
    # some robot node actually moves under gravity
    assert np.any(np.abs(res.displacements[~fixed]) > 0)


def test_robot_fem_loads_and_equilibrium():
    r = FireAnt3D(id=0, position=[0, 0, 1.0])
    gc = ground_contacts([r], plane_z=0.0)
    model = build_robot_fem([r], ground_anchors=gc, plane_z=0.0,
                            gravity=(0, 0, -1.0))
    # total applied load == robot weight downward (3 spheres * 1.0)
    assert model.loads.sum(axis=0)[2] == pytest.approx(-3.0, rel=1e-12)
    res = solve_truss(model)
    # reactions carry the whole weight
    assert res.reactions.sum(axis=0)[2] == pytest.approx(3.0, rel=1e-6)


def test_stacked_robots_top_sags_more_than_bottom():
    # bottom robot anchored to ground; top robot rests directly above (2r).
    bottom = FireAnt3D(id=0, position=[0, 0, 1.0])
    top = FireAnt3D(id=1, position=[0, 0, 3.0])           # 2r above bottom
    robots = [bottom, top]
    contacts = all_contacts(robots)
    gc = ground_contacts(robots, plane_z=0.0)             # only bottom touches ground
    assert all(i == 0 for (i, s) in gc)
    model = build_robot_fem(robots, robot_contacts=contacts, ground_anchors=gc,
                            plane_z=0.0, gravity=(0, 0, -1.0))
    res = solve_truss(model)
    # center node z-displacement: top center should sag more (more negative) than bottom
    bottom_center_z = res.displacements[12:16, 2].mean()   # robot 0 center nodes
    top_center_z = res.displacements[16 + 12:16 + 16, 2].mean()  # robot 1 center nodes
    assert top_center_z < bottom_center_z <= 0
