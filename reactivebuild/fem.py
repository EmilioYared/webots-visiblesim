"""fem.py -- linear 3D truss finite-element solver (Phase 2).

Two layers:

  1. A generic pin-jointed 3D truss solver (``TrussModel`` + ``solve_truss``): direct
     stiffness method, 3 translational DOF/node, sparse assembly and solve, recovery of
     element axial forces and support reactions. Validated against closed-form cases.

  2. ``build_robot_fem``: turns a scene of FireAnt3D robots + their contacts + ground
     anchors into a ``TrussModel`` exactly per the paper (section 3, Fig. 3; plan section 1.2):
       * each sphere and the center = a 4-node tetra, its 6 internal edges = IN_SPHERE,
       * each sphere's 4 nodes fully connected to the center's 4 nodes = ROBOT_STRUCTURE,
       * contacting spheres (robot-robot or sphere<->environment) fully connected = CONNECTION,
       * environmental-contact tetra nodes are fixed supports,
       * a 0.25 gravitational load on every robot sphere node (center nodes unloaded, Q3).

Stiffness values come from FEMParams; whether they are EA (k=EA/L) or raw k is the
``length_normalized`` flag (Q7).
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
from itertools import combinations, product
from typing import List, Optional, Sequence, Tuple

import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.linalg import spsolve

from . import geometry
from .config import FEMParams
from .robot import FireAnt3D


class ElementClass(IntEnum):
    IN_SPHERE = 0        # within a sphere/center tetra (near-rigid)
    ROBOT_STRUCTURE = 1  # sphere <-> center (the robot's flexible frame)
    CONNECTION = 2       # contacting spheres, robot-robot or sphere<->environment


# --- node-layout helpers (single source of truth for build_robot_fem + sensing) ---
STRIDE = 16  # nodes per robot: sphere0..2 (4 each) + center (4)


def sphere_node_indices(robot_index: int, sphere_index: int) -> List[int]:
    """Global indices of a robot's sphere tetra (4 nodes)."""
    base = robot_index * STRIDE + 4 * sphere_index
    return [base, base + 1, base + 2, base + 3]


def center_node_indices(robot_index: int) -> List[int]:
    """Global indices of a robot's center tetra (4 nodes)."""
    base = robot_index * STRIDE + 12
    return [base, base + 1, base + 2, base + 3]


def env_anchor_base(n_robots: int, anchor_index: int) -> int:
    """Global index of the first node of the anchor'th environment tetra."""
    return n_robots * STRIDE + 4 * anchor_index


# =============================================================================
# Generic truss model + solver
# =============================================================================

@dataclass
class TrussModel:
    nodes: np.ndarray            # (N, 3) positions
    elements: np.ndarray         # (M, 2) int node-index pairs
    rigidity: np.ndarray         # (M,) EA (or k if length_normalized=False)
    fixed: np.ndarray            # (N, 3) bool -- True == that DOF is constrained to 0
    loads: np.ndarray            # (N, 3) nodal forces
    element_class: Optional[np.ndarray] = None  # (M,) ElementClass values

    @property
    def n_nodes(self) -> int:
        return len(self.nodes)

    @property
    def n_elements(self) -> int:
        return len(self.elements)


@dataclass
class TrussResult:
    displacements: np.ndarray    # (N, 3)
    axial: np.ndarray            # (M,) element axial force, tension positive
    lengths: np.ndarray          # (M,) element rest lengths
    reactions: np.ndarray        # (N, 3) support reactions (0 at free DOFs)
    k: np.ndarray                # (M,) per-element axial stiffness used


def _assemble(nodes: np.ndarray, elements: np.ndarray, rigidity: np.ndarray,
              length_normalized: bool):
    """Assemble the global stiffness (CSR) plus per-element length, unit axis and k."""
    n_dof = 3 * len(nodes)
    p1 = nodes[elements[:, 0]]
    p2 = nodes[elements[:, 1]]
    d = p2 - p1
    L = np.linalg.norm(d, axis=1)
    if np.any(L == 0.0):
        raise ValueError("degenerate (zero-length) truss element")
    axis = d / L[:, None]
    k = rigidity / L if length_normalized else np.asarray(rigidity, float)

    rows: List[int] = []
    cols: List[int] = []
    data: List[float] = []
    for m in range(len(elements)):
        c = axis[m]
        B = np.outer(c, c) * k[m]           # 3x3 block
        i, j = int(elements[m, 0]), int(elements[m, 1])
        di = (3 * i, 3 * i + 1, 3 * i + 2)
        dj = (3 * j, 3 * j + 1, 3 * j + 2)
        for a in range(3):
            for b in range(3):
                v = B[a, b]
                rows += [di[a], di[a], dj[a], dj[a]]
                cols += [di[b], dj[b], di[b], dj[b]]
                data += [v, -v, -v, v]
    K = coo_matrix((data, (rows, cols)), shape=(n_dof, n_dof)).tocsr()
    return K, L, axis, k


def solve_truss(model: TrussModel, length_normalized: bool = True) -> TrussResult:
    """Solve K u = f with the fixed DOFs constrained to zero.

    Returns displacements, element axial forces (tension positive), rest lengths and
    support reactions. Raises if the free system is singular (under-constrained).
    """
    K, L, axis, k = _assemble(model.nodes, model.elements, model.rigidity,
                              length_normalized)
    n_dof = 3 * model.n_nodes
    f = model.loads.reshape(-1).astype(float)
    free = ~model.fixed.reshape(-1)

    u = np.zeros(n_dof)
    if free.any():
        Kff = K[free][:, free].tocsc()
        uf = spsolve(Kff, f[free])
        if not np.all(np.isfinite(uf)):
            raise np.linalg.LinAlgError(
                "truss solve produced non-finite displacements (singular / "
                "under-constrained system)")
        u[free] = uf
    U = u.reshape(-1, 3)

    # Element axial force: N = k * (relative displacement projected on the axis).
    du = U[model.elements[:, 1]] - U[model.elements[:, 0]]
    elongation = np.einsum("ij,ij->i", du, axis)
    axial = k * elongation

    # Reactions at supports: R = K u - f, zeroed on the free DOFs.
    R = K @ u - f
    R[free] = 0.0

    return TrussResult(displacements=U, axial=axial, lengths=L,
                       reactions=R.reshape(-1, 3), k=k)


# =============================================================================
# Robot scene  ->  TrussModel  (section 3, Fig. 3)
# =============================================================================

def _tetra_edges(base: int) -> List[Tuple[int, int]]:
    """The 6 fully-connecting edges of a 4-node tetra starting at global index `base`."""
    return [(base + a, base + b) for a, b in combinations(range(4), 2)]


def _full_bipartite(nodes_a: Sequence[int], nodes_b: Sequence[int]) -> List[Tuple[int, int]]:
    """All 16 pairs connecting one 4-node group to another (fully connect)."""
    return [(a, b) for a, b in product(nodes_a, nodes_b)]


def build_robot_fem(
    robots: List[FireAnt3D],
    *,
    robot_contacts: Optional[Sequence[Tuple[int, int, int, int]]] = None,
    ground_anchors: Optional[Sequence[Tuple[int, int]]] = None,
    plane_z: float = 0.0,
    fem: Optional[FEMParams] = None,
    gravity=(0.0, 0.0, -1.0),
) -> TrussModel:
    """Assemble the paper's truss FEM for a scene of robots.

    Args:
        robots:         the scene, in a fixed order (global node block = 16 * index).
        robot_contacts: (i, sphere_i, j, sphere_j) tuples, e.g. from robot.all_contacts.
        ground_anchors: (robot_index, sphere_index) spheres anchored to the ground; each
                        gets a fixed 4-node environment tetra at its ground contact point.
        plane_z:        ground height for anchor placement.
        fem:            FEMParams (stiffnesses, loads).
        gravity:        unit gravity direction for the sphere-node loads.
    """
    fem = fem or FEMParams()
    robot_contacts = list(robot_contacts or [])
    ground_anchors = list(ground_anchors or [])
    gravity = np.asarray(gravity, float).reshape(3)

    n_robots = len(robots)
    stride = STRIDE  # nodes per robot (sphere0..2 x4, center x4)

    # --- nodes: robot blocks first, then one fixed env tetra per ground anchor ---
    node_blocks = [r.all_nodes() for r in robots]
    env_nodes: List[np.ndarray] = []
    env_base_for_anchor: List[int] = []
    running = n_robots * stride
    for (ri, si) in ground_anchors:
        contact_pt = robots[ri].sphere_centers()[si].copy()
        contact_pt[2] = plane_z  # project to the ground plane
        tetra = contact_pt[None, :] + geometry.tetra_offsets(
            robots[ri].params.tetra_scale * robots[ri].params.sphere_radius)
        env_nodes.append(tetra)
        env_base_for_anchor.append(running)
        running += 4
    nodes = np.vstack(node_blocks + env_nodes) if env_nodes else np.vstack(node_blocks)

    elems: List[Tuple[int, int]] = []
    eclass: List[int] = []

    def add(pairs: List[Tuple[int, int]], cls: ElementClass) -> None:
        elems.extend(pairs)
        eclass.extend([int(cls)] * len(pairs))

    # in-sphere (each of the 3 sphere tetras + the center tetra) and robot-structure
    for ri in range(n_robots):
        for si in range(robots[ri].params.num_spheres):
            add(_tetra_edges(ri * stride + 4 * si), ElementClass.IN_SPHERE)
        add(_tetra_edges(ri * stride + 12), ElementClass.IN_SPHERE)  # center tetra
        for si in range(robots[ri].params.num_spheres):
            add(_full_bipartite(sphere_node_indices(ri, si), center_node_indices(ri)),
                ElementClass.ROBOT_STRUCTURE)

    # connections between contacting spheres (robot-robot)
    for (i, sa, j, sb) in robot_contacts:
        add(_full_bipartite(sphere_node_indices(i, sa), sphere_node_indices(j, sb)),
            ElementClass.CONNECTION)

    # environment contacts: fixed tetra <-> sphere (connection) + its internal edges
    for a, (ri, si) in enumerate(ground_anchors):
        base = env_base_for_anchor[a]
        env_group = [base, base + 1, base + 2, base + 3]
        add(_full_bipartite(sphere_node_indices(ri, si), env_group), ElementClass.CONNECTION)
        add(_tetra_edges(base), ElementClass.IN_SPHERE)

    elements = np.asarray(elems, dtype=int)
    element_class = np.asarray(eclass, dtype=int)

    # rigidity per element from its class
    rig_by_class = {
        int(ElementClass.IN_SPHERE): fem.k_in_sphere,
        int(ElementClass.ROBOT_STRUCTURE): fem.k_robot_structure,
        int(ElementClass.CONNECTION): fem.k_connection,
    }
    rigidity = np.array([rig_by_class[c] for c in element_class], dtype=float)

    # loads: 0.25 on every robot sphere node along gravity; center + env nodes unloaded
    loads = np.zeros((len(nodes), 3))
    for ri in range(n_robots):
        for si in range(robots[ri].params.num_spheres):
            for nd in sphere_node_indices(ri, si):
                loads[nd] = fem.sphere_node_load * gravity

    # fixed DOFs: all env-tetra nodes fully fixed
    fixed = np.zeros((len(nodes), 3), dtype=bool)
    for base in env_base_for_anchor:
        fixed[base:base + 4] = True

    return TrussModel(nodes=nodes, elements=elements, rigidity=rigidity,
                      fixed=fixed, loads=loads, element_class=element_class)


def connection_bundles(
    robots: List[FireAnt3D],
    robot_contacts: Optional[Sequence[Tuple[int, int, int, int]]] = None,
    ground_anchors: Optional[Sequence[Tuple[int, int]]] = None,
) -> List[Tuple[List[int], List[int], str]]:
    """Node-index groups for every CONNECTION bundle, matching build_robot_fem's layout.

    Returns (group_a, group_b, kind) with kind in {"robot-robot", "ground"}. Used by
    sensing to compute per-connection stresses without re-deriving node indices.
    """
    robot_contacts = list(robot_contacts or [])
    ground_anchors = list(ground_anchors or [])
    n = len(robots)
    bundles: List[Tuple[List[int], List[int], str]] = []
    for (i, sa, j, sb) in robot_contacts:
        bundles.append((sphere_node_indices(i, sa), sphere_node_indices(j, sb),
                        "robot-robot"))
    for a, (ri, si) in enumerate(ground_anchors):
        base = env_anchor_base(n, a)
        bundles.append((sphere_node_indices(ri, si), [base, base + 1, base + 2, base + 3],
                        "ground"))
    return bundles
