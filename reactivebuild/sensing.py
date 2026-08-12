"""sensing.py -- force-sensor and connection-stress reconstruction (Phase 3).

The truss carries only axial forces, but a *bundle* of axial elements spanning two
4-node groups (spatially distributed) transmits a resultant force AND moment. This module
reconstructs, for any such bundle:

    axial, shear   (force components parallel / perpendicular to the connection axis),
    bending, torsion (moment components perpendicular / parallel to the axis),

and from those computes the two quantities the paper defines (section 3):

  * sensed_force (per robot): the mean, over the robot's 3 sphere<->center connections
    (the ROBOT_STRUCTURE bundles), of (|axial| + |bending|). This is what the ReactiveBuild
    algorithm reacts to; robots can measure only axial force and bending moment.

  * connection stress (per contact): a combined (von-Mises) stress from all forces and
    moments at a CONNECTION bundle, assuming a circular contact of radius 0.5.

Interpretation flags (see plan): magnitudes are used for the sensed sum (Q2), and the
moment is taken about the bundle midpoint (Q2b). Both are isolated here so they are easy
to revisit against the paper's F thresholds once experiments run.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import numpy as np

from . import fem as _fem
from .fem import TrussModel, TrussResult


@dataclass
class ForceResultant:
    """Resultant transmitted from group_a to group_b, decomposed about the axis."""
    axial: float      # force component along the axis (signed: + == tension/pull apart)
    shear: float      # force magnitude perpendicular to the axis (>= 0)
    bending: float    # moment magnitude perpendicular to the axis (>= 0)
    torsion: float    # moment component along the axis (signed)
    force: np.ndarray   # (3,) net force on group_b
    moment: np.ndarray  # (3,) net moment on group_b about the reference point


def bundle_resultant(model: TrussModel, result: TrussResult,
                     group_a: Sequence[int], group_b: Sequence[int],
                     ref_point: Optional[np.ndarray] = None,
                     axis: Optional[np.ndarray] = None) -> ForceResultant:
    """Resultant force/moment transmitted from group_a to group_b through the truss
    elements linking them.

    The axis defaults to (centroid_a -> centroid_b); the moment reference defaults to the
    midpoint of the two centroids.
    """
    A = set(int(i) for i in group_a)
    B = set(int(i) for i in group_b)
    ca = model.nodes[list(group_a)].mean(axis=0)
    cb = model.nodes[list(group_b)].mean(axis=0)
    if axis is None:
        axis = cb - ca
    axis = np.asarray(axis, float)
    na = np.linalg.norm(axis)
    axis_hat = axis / na if na > 0 else np.array([0.0, 0.0, 1.0])
    if ref_point is None:
        ref_point = 0.5 * (ca + cb)
    ref_point = np.asarray(ref_point, float)

    F = np.zeros(3)
    M = np.zeros(3)
    for e_idx, (i, j) in enumerate(model.elements):
        i, j = int(i), int(j)
        if i in A and j in B:
            a_node, b_node = i, j
        elif j in A and i in B:
            a_node, b_node = j, i
        else:
            continue
        n_ax = result.axial[e_idx]                      # tension positive
        pa, pb = model.nodes[a_node], model.nodes[b_node]
        d = pb - pa
        c = d / np.linalg.norm(d)
        # In tension the element pulls b toward a: force on b = -N * c.
        f_on_b = -n_ax * c
        F += f_on_b
        M += np.cross(pb - ref_point, f_on_b)

    axial = float(F @ axis_hat)
    shear = float(np.linalg.norm(F - axial * axis_hat))
    torsion = float(M @ axis_hat)
    bending = float(np.linalg.norm(M - torsion * axis_hat))
    return ForceResultant(axial=axial, shear=shear, bending=bending, torsion=torsion,
                          force=F, moment=M)


# --- sensed_force (per robot) ------------------------------------------------

def robot_sensed_force(model: TrussModel, result: TrussResult, robot_index: int,
                       num_spheres: int = 3) -> float:
    """sensed_force for one robot = mean over its sphere<->center bundles of
    (|axial| + |bending|) (section 3)."""
    center = _fem.center_node_indices(robot_index)
    total = 0.0
    for s in range(num_spheres):
        sphere = _fem.sphere_node_indices(robot_index, s)
        r = bundle_resultant(model, result, sphere, center)
        total += abs(r.axial) + abs(r.bending)
    return total / num_spheres


def all_sensed_forces(model: TrussModel, result: TrussResult,
                      robots: Sequence) -> np.ndarray:
    """sensed_force for every robot (shape (n_robots,))."""
    return np.array([
        robot_sensed_force(model, result, ri, robots[ri].params.num_spheres)
        for ri in range(len(robots))
    ])


# --- connection stress (per contact), for reporting/plots --------------------

def connection_stress(model: TrussModel, result: TrussResult,
                      group_a: Sequence[int], group_b: Sequence[int],
                      contact_radius: float = 0.5) -> float:
    """Combined (von-Mises) stress at a connection, circular contact of `contact_radius`
    (section 3). Uses all four resultants: axial+bending -> normal stress, shear+torsion ->
    shear stress; sigma_vm = sqrt(sigma^2 + 3 tau^2)."""
    r = bundle_resultant(model, result, group_a, group_b)
    rad = contact_radius
    area = np.pi * rad ** 2
    I = np.pi * rad ** 4 / 4.0        # second moment of area
    Jp = np.pi * rad ** 4 / 2.0       # polar moment
    sigma = abs(r.axial) / area + abs(r.bending) * rad / I
    tau = abs(r.shear) / area + abs(r.torsion) * rad / Jp
    return float(np.sqrt(sigma ** 2 + 3.0 * tau ** 2))


def peak_connection_stress(model: TrussModel, result: TrussResult,
                           bundles: Sequence[Tuple[List[int], List[int], str]],
                           contact_radius: float = 0.5) -> float:
    """Maximum connection stress across a list of CONNECTION bundles (from
    fem.connection_bundles). Returns 0.0 for an empty list."""
    if not bundles:
        return 0.0
    return max(connection_stress(model, result, ga, gb, contact_radius)
               for (ga, gb, _kind) in bundles)
