"""config.py -- Parameters for the ReactiveBuild replication.

Every constant here traces to Swissler & Rubenstein (2022), "ReactiveBuild:
Environment-Adaptive Self-Assembly of Amorphous Structures". Section markers (e.g.
"(2)") refer to that paper; see REACTIVEBUILD_PLAN.md section 1 for the decoded
fidelity spec and the interpretation decisions (Q1..Q6).

The module exposes:
  * module-level paper constants (so tests can pin the exact values),
  * small frozen dataclasses grouping those constants
    (AlgorithmParams / FEMParams / RobotParams / ExperimentParams),
  * a top-level ``Params`` bundle, and
  * per-experiment presets: tower(), chain(), cantilever(), bridge().
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional, Tuple

Vec3 = Tuple[float, float, float]

# =============================================================================
# Paper constants  (module-level == the single source of truth for the numbers)
# =============================================================================

# --- Algorithm parameter sweeps (section 4) ----------------------------------
F_SWEEP: Tuple[float, ...] = (1.0, 2.5, 5.0, 25.0)   # threshold force F
B_SWEEP: Tuple[float, ...] = (1.5, 3.0, 6.0, 10.0)   # exponent base B
J_SWEEP: Tuple[int, ...] = (1, 2, 5, 1000)           # max recruit value J

# Defaults. B and J are the fixed values used throughout the F-sweeps (section 4);
# F defaults to 10, the value used for the growth illustrations (Figs. 4, 6, 8).
DEFAULT_F: float = 10.0
DEFAULT_B: float = 3.0
DEFAULT_J: int = 5

# --- FEM model (section 3, Fig. 3; plan section 1.2) --------------------------
STIFFNESS_IN_SPHERE: float = 5e10        # near-rigid, within a sphere-tetra (red)
STIFFNESS_ROBOT_STRUCTURE: float = 2e9   # sphere -> center, the robot's frame (green)
STIFFNESS_CONNECTION: float = 1e10       # contacting spheres / sphere<->env (blue)

SPHERE_NODE_LOAD: float = 0.25           # gravitational load per *sphere node*
NODES_PER_SPHERE: int = 4                # each sphere is a 4-node tetrahedron
NODES_PER_CENTER: int = 4                # the robot center is also a 4-node tetra
# => per-sphere weight = 0.25 * 4 = 1.0 unit (this is *why* spheres are 4 nodes).

CONTACT_STRESS_RADIUS: float = 0.5       # circular contact radius for stress calc

# --- Robot geometry (section 2, section 4; Q6) -------------------------------
NUM_SPHERES: int = 3                      # FireAnt3D = three spheres
SPHERE_RADIUS: float = 1.0               # distances are "in sphere radii" -> radius 1
# Intra-sphere tetrahedron edge, as a fraction of the sphere radius (Q3, tunable).
# Small -> in-sphere elements behave near-rigidly; mainly affects conditioning.
TETRA_SCALE: float = 0.1

# --- Experiment framing (section 4) -----------------------------------------
N_RUNS: int = 100                        # "100 structures ... each"
N_ROBOTS: int = 100                      # "... of 100 robots"
MATURATION_N: int = 25                   # scaling laws analysed after N = 25

# Goal offsets per structure type (section 4.1 - 4.4), in sphere radii.
GOAL_TOWER: Vec3 = (0.0, 0.0, 65.0)          # 65 units straight up
GOAL_CHAIN: Vec3 = (0.0, 0.0, -600.0)        # 600 units below the edge
GOAL_CANTILEVER: Vec3 = (45.0, 0.0, 10.0)    # 45 out horizontally, 10 up

# Bridge (section 4.4).
BRIDGE_GAPS: Tuple[float, ...] = (20.0, 25.0, 30.0)
BRIDGE_N_CAP: int = 200                   # stop after 200 robots
BRIDGE_SUCCESS_THRESHOLD: int = 100       # unsuccessful if N >= 100

# Gravity as a unit direction (plan section 1.2). Magnitude lives in the node loads.
GRAVITY_DIR: Vec3 = (0.0, 0.0, -1.0)


class ExperimentType(str, Enum):
    TOWER = "tower"
    CHAIN = "chain"
    CANTILEVER = "cantilever"
    BRIDGE = "bridge"


class GoalReference(str, Enum):
    """What the goal offset is measured from."""
    BASE = "base"    # the spawn-plane origin (tower)
    EDGE = "edge"    # the plane edge / cylinder (chain, cantilever, bridge)


# =============================================================================
# Parameter groups
# =============================================================================

@dataclass(frozen=True)
class AlgorithmParams:
    """ReactiveBuild control parameters (section 2, Alg. 2; section 4)."""
    F: float = DEFAULT_F    # threshold force: recruitment begins when sensed >= F
    B: float = DEFAULT_B    # EXPONENT base for recruit growth (section 4)
    J: int = DEFAULT_J      # max recruit value == jurisdiction radius in hops

    def recruit_value(self, sensed_force: float) -> int:
        """Recruit value for a given sensed force.

        recruit = min(floor(B ** (sensed/F - 1)), J), clamped at 0.

        B is an *exponent base* (paper: "exponent base B", section 4), NOT a linear
        multiplier. Consequences: below threshold (sensed < F) the exponent is
        negative so the value floors to 0; at sensed == F it is B**0 = 1 (recruitment
        just begins); it saturates at J.
        """
        exponent = sensed_force / self.F - 1.0
        if self.J <= 0:
            return 0
        # Result caps at J, so short-circuit before B**exponent can overflow a float
        # (B**exponent >= J  =>  floor(B**exponent) >= J). Only relevant for B > 1;
        # otherwise B**exponent <= 1 and cannot overflow.
        if exponent > 0.0 and self.B > 1.0 and exponent * math.log(self.B) >= math.log(self.J):
            return self.J
        raw = self.B ** exponent
        return max(0, min(int(math.floor(raw)), self.J))


@dataclass(frozen=True)
class FEMParams:
    """Truss-FEM stiffnesses, loads and contact geometry (section 3; plan section 1.2)."""
    k_in_sphere: float = STIFFNESS_IN_SPHERE
    k_robot_structure: float = STIFFNESS_ROBOT_STRUCTURE
    k_connection: float = STIFFNESS_CONNECTION
    sphere_node_load: float = SPHERE_NODE_LOAD
    nodes_per_sphere: int = NODES_PER_SPHERE
    nodes_per_center: int = NODES_PER_CENTER
    contact_stress_radius: float = CONTACT_STRESS_RADIUS
    # Q7: treat the per-class stiffness values as axial rigidity EA and use k=EA/L
    # (standard truss FEM). If False, use the value directly as the element stiffness k
    # (length-independent). Only rescales magnitudes; relative ordering is unchanged.
    length_normalized: bool = True

    @property
    def sphere_weight(self) -> float:
        """Total gravitational load per sphere (0.25 * 4 nodes = 1.0)."""
        return self.sphere_node_load * self.nodes_per_sphere


@dataclass(frozen=True)
class RobotParams:
    """FireAnt3D geometry (section 2; Q3, Q6)."""
    num_spheres: int = NUM_SPHERES
    sphere_radius: float = SPHERE_RADIUS
    tetra_scale: float = TETRA_SCALE

    @property
    def robot_weight(self) -> float:
        """Total robot weight = num_spheres * 1.0 (center nodes carry no load, Q3)."""
        return float(self.num_spheres) * SPHERE_NODE_LOAD * NODES_PER_SPHERE


@dataclass(frozen=True)
class ExperimentParams:
    """Environment, goal and run counts for one structure type (section 4)."""
    type: ExperimentType = ExperimentType.TOWER
    goal_offset: Vec3 = GOAL_TOWER
    goal_reference: GoalReference = GoalReference.BASE
    n_robots: int = N_ROBOTS
    n_runs: int = N_RUNS
    maturation_n: int = MATURATION_N
    # Bridge-only fields (None for the single-origin structures).
    gap_width: Optional[float] = None
    n_cap: Optional[int] = None
    success_threshold: Optional[int] = None


@dataclass(frozen=True)
class Params:
    """Top-level bundle passed around the simulator."""
    algorithm: AlgorithmParams = field(default_factory=AlgorithmParams)
    fem: FEMParams = field(default_factory=FEMParams)
    robot: RobotParams = field(default_factory=RobotParams)
    experiment: ExperimentParams = field(default_factory=ExperimentParams)
    gravity: Vec3 = GRAVITY_DIR
    seed: int = 0

    # -- convenience ----------------------------------------------------------
    def describe(self) -> str:
        a, e, fem, r = self.algorithm, self.experiment, self.fem, self.robot
        lines = [
            "ReactiveBuild config",
            "  experiment : {}".format(e.type.value),
            "  goal       : offset={} from {}".format(e.goal_offset, e.goal_reference.value),
            "  runs       : {} runs x {} robots (maturation N={})".format(
                e.n_runs, e.n_robots, e.maturation_n),
        ]
        if e.gap_width is not None:
            lines.append("  bridge     : gap={} n_cap={} success_if_N<{}".format(
                e.gap_width, e.n_cap, e.success_threshold))
        lines += [
            "  algorithm  : F={} B={} J={}".format(a.F, a.B, a.J),
            "  robot      : {} spheres, radius {}, weight {}".format(
                r.num_spheres, r.sphere_radius, r.robot_weight),
            "  fem        : k(in-sphere)={:.0e} k(structure)={:.0e} k(connection)={:.0e}".format(
                fem.k_in_sphere, fem.k_robot_structure, fem.k_connection),
            "  fem loads  : {} / sphere-node x {} nodes = {} / sphere".format(
                fem.sphere_node_load, fem.nodes_per_sphere, fem.sphere_weight),
            "  gravity    : {}".format(self.gravity),
            "  seed       : {}".format(self.seed),
        ]
        return "\n".join(lines)


# =============================================================================
# Presets  (section 4.1 - 4.4)
# =============================================================================

def tower(F: float = DEFAULT_F, B: float = DEFAULT_B, J: int = DEFAULT_J,
          *, n_runs: int = N_RUNS, n_robots: int = N_ROBOTS, seed: int = 0) -> Params:
    """Tower: goal 65 units above a flat plane (section 4.1)."""
    return Params(
        algorithm=AlgorithmParams(F=F, B=B, J=J),
        experiment=ExperimentParams(
            type=ExperimentType.TOWER, goal_offset=GOAL_TOWER,
            goal_reference=GoalReference.BASE, n_runs=n_runs, n_robots=n_robots),
        seed=seed,
    )


def chain(F: float = DEFAULT_F, B: float = DEFAULT_B, J: int = DEFAULT_J,
          *, n_runs: int = N_RUNS, n_robots: int = N_ROBOTS, seed: int = 0) -> Params:
    """Chain: goal 600 units below a plane edge (section 4.2)."""
    return Params(
        algorithm=AlgorithmParams(F=F, B=B, J=J),
        experiment=ExperimentParams(
            type=ExperimentType.CHAIN, goal_offset=GOAL_CHAIN,
            goal_reference=GoalReference.EDGE, n_runs=n_runs, n_robots=n_robots),
        seed=seed,
    )


def cantilever(F: float = DEFAULT_F, B: float = DEFAULT_B, J: int = DEFAULT_J,
               *, n_runs: int = N_RUNS, n_robots: int = N_ROBOTS, seed: int = 0) -> Params:
    """Cantilever: goal 45 out + 10 up from a plane edge (section 4.3)."""
    return Params(
        algorithm=AlgorithmParams(F=F, B=B, J=J),
        experiment=ExperimentParams(
            type=ExperimentType.CANTILEVER, goal_offset=GOAL_CANTILEVER,
            goal_reference=GoalReference.EDGE, n_runs=n_runs, n_robots=n_robots),
        seed=seed,
    )


def bridge(gap_width: float = BRIDGE_GAPS[0], F: float = DEFAULT_F,
           B: float = DEFAULT_B, J: int = DEFAULT_J,
           *, n_runs: int = N_RUNS, seed: int = 0) -> Params:
    """Bridge across a gap; robots alternate sides (section 4.4).

    Runs until a robot crosses or N reaches ``n_cap`` (200); a bridge counts as
    unsuccessful if it needs N >= ``success_threshold`` (100). The goal offset here
    is a placeholder marking the far side (x = gap_width); the two per-side goals
    are resolved by the bridge experiment in a later phase.
    """
    return Params(
        algorithm=AlgorithmParams(F=F, B=B, J=J),
        experiment=ExperimentParams(
            type=ExperimentType.BRIDGE, goal_offset=(gap_width, 0.0, 0.0),
            goal_reference=GoalReference.EDGE, n_runs=n_runs, n_robots=BRIDGE_N_CAP,
            gap_width=gap_width, n_cap=BRIDGE_N_CAP,
            success_threshold=BRIDGE_SUCCESS_THRESHOLD),
        seed=seed,
    )


PRESETS = {
    ExperimentType.TOWER: tower,
    ExperimentType.CHAIN: chain,
    ExperimentType.CANTILEVER: cantilever,
    ExperimentType.BRIDGE: bridge,
}


def preset(name: str) -> Params:
    """Build a default preset by name ('tower' | 'chain' | 'cantilever' | 'bridge')."""
    return PRESETS[ExperimentType(name)]()
