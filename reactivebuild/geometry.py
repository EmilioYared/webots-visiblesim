"""geometry.py -- FireAnt3D geometry primitives (Phase 1).

Pure-array geometry, independent of the FireAnt3D object in robot.py:

  * body-frame layout of the three sphere centers (equilateral triangle, mutually
    tangent) and the robot center (their centroid),
  * the 4-node tetrahedron used to represent each sphere and the center in the FEM
    (a single pin node has no rotational stiffness, so each is a small regular
    tetrahedron -- 3 spheres x 4 + center x 4 = 16 nodes/robot; this is also why the
    0.25/node load gives 1.0/sphere, section 3),
  * rigid pose transforms (position + rotation), and
  * contact primitives (sphere<->sphere, sphere<->ground-plane).

Assumptions (see plan Q3/Q6): sphere_radius = 1 "unit"; the three spheres are coplanar
and mutually tangent; the tetra circumradius is tetra_scale * sphere_radius (small).
"""

from __future__ import annotations

import numpy as np

from .config import RobotParams

# Body-frame angles of the 3 sphere centers in the xy-plane (120 deg apart).
_SPHERE_ANGLES = np.deg2rad(np.array([90.0, 210.0, 330.0]))

# Unit regular tetrahedron: 4 vertices, circumradius 1, centroid at the origin.
_TETRA_UNIT = np.array(
    [[1.0, 1.0, 1.0],
     [1.0, -1.0, -1.0],
     [-1.0, 1.0, -1.0],
     [-1.0, -1.0, 1.0]]
) / np.sqrt(3.0)

CONTACT_TOL = 1e-6  # default absolute slack for "in contact" (touching counts)


# --- body-frame layouts ------------------------------------------------------

def sphere_offsets(sphere_radius: float) -> np.ndarray:
    """Body-frame centers of the 3 spheres (shape (3, 3)).

    Equilateral triangle in the xy-plane, mutually tangent (side = 2 * radius), so the
    circumradius is 2*radius/sqrt(3); centroid at the origin (== the robot center).
    """
    side = 2.0 * sphere_radius
    circumradius = side / np.sqrt(3.0)
    return np.stack(
        [circumradius * np.cos(_SPHERE_ANGLES),
         circumradius * np.sin(_SPHERE_ANGLES),
         np.zeros(3)],
        axis=1,
    )


def tetra_offsets(circumradius: float) -> np.ndarray:
    """Body-frame node offsets of a 4-node regular tetra (shape (4, 3))."""
    return _TETRA_UNIT * circumradius


# --- pose transforms ---------------------------------------------------------

def rotation_from_axis_angle(axis, angle: float) -> np.ndarray:
    """3x3 rotation matrix for a right-handed rotation of `angle` rad about `axis`."""
    axis = np.asarray(axis, float)
    n = np.linalg.norm(axis)
    if n == 0.0:
        return np.eye(3)
    x, y, z = axis / n
    c, s = np.cos(angle), np.sin(angle)
    C = 1.0 - c
    return np.array(
        [[c + x * x * C, x * y * C - z * s, x * z * C + y * s],
         [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
         [z * x * C - y * s, z * y * C + x * s, c + z * z * C]]
    )


def _apply_pose(points_body: np.ndarray, position, rotation) -> np.ndarray:
    """world = position + points_body @ rotation.T  (rotation: world = R @ body)."""
    position = np.asarray(position, float).reshape(3)
    rotation = np.asarray(rotation, float).reshape(3, 3)
    return position + np.asarray(points_body, float) @ rotation.T


def sphere_centers(position, rotation, robot: RobotParams) -> np.ndarray:
    """World-frame centers of the 3 spheres (shape (3, 3))."""
    return _apply_pose(sphere_offsets(robot.sphere_radius), position, rotation)


def robot_node_positions(position, rotation, robot: RobotParams):
    """World-frame node positions for one robot.

    Returns (sphere_nodes, center_nodes) with shapes (num_spheres, 4, 3) and (4, 3).
    """
    tetra_r = robot.tetra_scale * robot.sphere_radius
    offs = sphere_offsets(robot.sphere_radius)       # (num_spheres, 3)
    tet = tetra_offsets(tetra_r)                      # (4, 3)

    sphere_nodes = np.empty((robot.num_spheres, 4, 3))
    for i in range(robot.num_spheres):
        sphere_nodes[i] = _apply_pose(offs[i][None, :] + tet, position, rotation)
    center_nodes = _apply_pose(tet, position, rotation)  # center sits at body origin
    return sphere_nodes, center_nodes


def all_node_positions(position, rotation, robot: RobotParams) -> np.ndarray:
    """All 16 node positions, ordered sphere0(4), sphere1(4), sphere2(4), center(4).

    This ordering is the contract the FEM assembly (later phase) relies on.
    """
    sphere_nodes, center_nodes = robot_node_positions(position, rotation, robot)
    return np.vstack([sphere_nodes.reshape(-1, 3), center_nodes])


# --- contact primitives ------------------------------------------------------

def spheres_contact(c1, r1: float, c2, r2: float, tol: float = CONTACT_TOL) -> bool:
    """True if two spheres touch or overlap (center distance <= r1 + r2 + tol)."""
    d = float(np.linalg.norm(np.asarray(c1, float) - np.asarray(c2, float)))
    return d <= r1 + r2 + tol


def sphere_plane_contact(center, radius: float, plane_z: float = 0.0,
                         tol: float = CONTACT_TOL) -> bool:
    """True if a sphere touches/penetrates a horizontal ground plane at z = plane_z.

    Minimal flat-plane primitive; full environments (edge+cylinder, gaps) arrive in a
    later phase (environment.py).
    """
    return float(np.asarray(center, float)[2]) - radius <= plane_z + tol
