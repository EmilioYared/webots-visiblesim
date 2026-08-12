"""robot.py -- the FireAnt3D robot object and scene-level contact detection (Phase 1).

A FireAnt3D robot is three spheres + a center, with a rigid pose (position + rotation).
Its three spheres are the algorithm's contact zones (section 2). This module wraps the
geometry primitives and adds contact detection between robots and against the ground.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import List, Optional, Tuple

import numpy as np

from . import geometry
from .config import FEMParams, RobotParams


class Role(str, Enum):
    """A robot is either climbing toward the goal or frozen into the structure (section 2)."""
    MOVING = "moving"
    STRUCTURAL = "structural"


@dataclass
class FireAnt3D:
    """One FireAnt3D robot.

    Attributes:
        id:       stable identifier (assignment order).
        position: (3,) world position of the robot center.
        rotation: (3,3) rotation matrix (world = R @ body); defaults to identity.
        role:     MOVING or STRUCTURAL.
        params:   geometry parameters.
        comm_in / comm_out: per-contact-zone (per-sphere) integer message buffers,
                  used by the ReactiveBuild algorithm in a later phase.
    """
    id: int
    position: np.ndarray
    rotation: np.ndarray = field(default_factory=lambda: np.eye(3))
    role: Role = Role.MOVING
    params: RobotParams = field(default_factory=RobotParams)
    comm_in: Optional[np.ndarray] = None
    comm_out: Optional[np.ndarray] = None

    def __post_init__(self) -> None:
        self.position = np.asarray(self.position, float).reshape(3)
        self.rotation = np.asarray(self.rotation, float).reshape(3, 3)
        n = self.params.num_spheres
        if self.comm_in is None:
            self.comm_in = np.zeros(n, dtype=int)
        if self.comm_out is None:
            self.comm_out = np.zeros(n, dtype=int)

    # -- geometry views -------------------------------------------------------
    @property
    def num_zones(self) -> int:
        """Number of contact zones == number of spheres."""
        return self.params.num_spheres

    @property
    def radius(self) -> float:
        return self.params.sphere_radius

    def sphere_centers(self) -> np.ndarray:
        """World centers of the spheres (shape (num_spheres, 3))."""
        return geometry.sphere_centers(self.position, self.rotation, self.params)

    def node_positions(self):
        """(sphere_nodes (num_spheres,4,3), center_nodes (4,3)) in world frame."""
        return geometry.robot_node_positions(self.position, self.rotation, self.params)

    def all_nodes(self) -> np.ndarray:
        """All 16 node positions, ordered sphere0..2 (4 each) then center (4)."""
        return geometry.all_node_positions(self.position, self.rotation, self.params)

    @property
    def total_weight(self) -> float:
        return self.params.robot_weight

    def sphere_node_loads(self, fem: Optional[FEMParams] = None) -> float:
        """Total gravitational load carried by this robot's sphere nodes."""
        fem = fem or FEMParams()
        return self.params.num_spheres * fem.nodes_per_sphere * fem.sphere_node_load


# --- contact detection -------------------------------------------------------

# A robot-robot contact: (robot_a_index, sphere_a, robot_b_index, sphere_b).
Contact = Tuple[int, int, int, int]


def robot_sphere_contacts(a: FireAnt3D, b: FireAnt3D,
                          tol: float = geometry.CONTACT_TOL) -> List[Tuple[int, int]]:
    """Sphere-index pairs (sphere_a, sphere_b) where a sphere of `a` touches one of `b`."""
    ca, cb = a.sphere_centers(), b.sphere_centers()
    ra, rb = a.radius, b.radius
    pairs: List[Tuple[int, int]] = []
    for i in range(len(ca)):
        for j in range(len(cb)):
            if geometry.spheres_contact(ca[i], ra, cb[j], rb, tol):
                pairs.append((i, j))
    return pairs


def all_contacts(robots: List[FireAnt3D],
                 tol: float = geometry.CONTACT_TOL) -> List[Contact]:
    """All robot-robot sphere contacts in a scene, as (i, sphere_i, j, sphere_j) with
    i < j into `robots`. Used to build FEM connection elements and comm links."""
    contacts: List[Contact] = []
    for i in range(len(robots)):
        for j in range(i + 1, len(robots)):
            for (sa, sb) in robot_sphere_contacts(robots[i], robots[j], tol):
                contacts.append((i, sa, j, sb))
    return contacts


def ground_contacts(robots: List[FireAnt3D], plane_z: float = 0.0,
                    tol: float = geometry.CONTACT_TOL) -> List[Tuple[int, int]]:
    """(robot_index, sphere_index) pairs whose sphere touches the ground plane z=plane_z."""
    hits: List[Tuple[int, int]] = []
    for i, r in enumerate(robots):
        for s, c in enumerate(r.sphere_centers()):
            if geometry.sphere_plane_contact(c, r.radius, plane_z, tol):
                hits.append((i, s))
    return hits
