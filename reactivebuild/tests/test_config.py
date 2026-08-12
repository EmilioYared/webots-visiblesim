"""Phase 0 tests: the config holds the paper's exact constants and the presets
match the four experiments (section 4). Also pins the recruit-value formula, since the
"B is an exponent base" decision (Q4) is load-bearing for every later phase.
"""

import math

import pytest

from reactivebuild import config as C


# --- paper constants are exactly the published values ------------------------

def test_fem_stiffnesses():
    fem = C.FEMParams()
    assert fem.k_in_sphere == 5e10
    assert fem.k_robot_structure == 2e9
    assert fem.k_connection == 1e10


def test_load_bookkeeping_sphere_weight_is_one():
    # 0.25 load per node * 4 nodes per sphere == 1.0 unit weight (section 3).
    fem = C.FEMParams()
    assert fem.sphere_node_load == 0.25
    assert fem.nodes_per_sphere == 4
    assert fem.sphere_weight == pytest.approx(1.0)


def test_robot_geometry_and_weight():
    r = C.RobotParams()
    assert r.num_spheres == 3
    assert r.sphere_radius == 1.0
    # 3 spheres * 1.0 == 3.0 (center nodes unloaded, Q3).
    assert r.robot_weight == pytest.approx(3.0)


def test_contact_stress_radius():
    assert C.FEMParams().contact_stress_radius == 0.5


def test_sweeps_match_paper():
    assert C.F_SWEEP == (1.0, 2.5, 5.0, 25.0)
    assert C.B_SWEEP == (1.5, 3.0, 6.0, 10.0)
    assert C.J_SWEEP == (1, 2, 5, 1000)


def test_defaults():
    a = C.AlgorithmParams()
    assert (a.F, a.B, a.J) == (10.0, 3.0, 5)  # Figs. 4/6/8 illustration values


def test_maturation_and_run_counts():
    assert C.MATURATION_N == 25
    assert C.N_RUNS == 100
    assert C.N_ROBOTS == 100


# --- recruit-value formula: recruit = min(floor(B**(sensed/F - 1)), J) -------

def test_recruit_below_threshold_is_zero():
    a = C.AlgorithmParams(F=5.0, B=3.0, J=5)
    assert a.recruit_value(0.0) == 0
    assert a.recruit_value(2.5) == 0     # 3**(-0.5) = 0.577 -> floor 0


def test_recruit_begins_at_threshold():
    a = C.AlgorithmParams(F=5.0, B=3.0, J=5)
    assert a.recruit_value(5.0) == 1     # B**0 = 1


def test_recruit_is_exponential_not_linear():
    a = C.AlgorithmParams(F=5.0, B=3.0, J=5)
    # sensed=10 -> 3**(10/5-1)=3**1=3 ; a linear rule would give a different value.
    assert a.recruit_value(10.0) == 3
    assert a.recruit_value(10.0) == min(math.floor(3.0 ** (10.0 / 5.0 - 1.0)), 5)


def test_recruit_saturates_at_J():
    a = C.AlgorithmParams(F=5.0, B=3.0, J=5)
    assert a.recruit_value(15.0) == 5    # 3**2 = 9 -> capped at J=5
    assert a.recruit_value(1e6) == 5


def test_recruit_monotonic_nondecreasing():
    a = C.AlgorithmParams(F=2.5, B=3.0, J=5)
    vals = [a.recruit_value(s) for s in range(0, 40)]
    assert all(b >= x for x, b in zip(vals, vals[1:]))


# --- presets encode the four experiments (section 4.1 - 4.4) -----------------

def test_tower_preset():
    p = C.tower()
    assert p.experiment.type is C.ExperimentType.TOWER
    assert p.experiment.goal_offset == (0.0, 0.0, 65.0)
    assert p.experiment.goal_reference is C.GoalReference.BASE


def test_chain_preset():
    p = C.chain()
    assert p.experiment.type is C.ExperimentType.CHAIN
    assert p.experiment.goal_offset == (0.0, 0.0, -600.0)
    assert p.experiment.goal_reference is C.GoalReference.EDGE


def test_cantilever_preset():
    p = C.cantilever()
    assert p.experiment.type is C.ExperimentType.CANTILEVER
    assert p.experiment.goal_offset == (45.0, 0.0, 10.0)


def test_bridge_preset():
    p = C.bridge(gap_width=25.0)
    e = p.experiment
    assert e.type is C.ExperimentType.BRIDGE
    assert e.gap_width == 25.0
    assert e.n_cap == 200
    assert e.success_threshold == 100


def test_bridge_gaps_constant():
    assert C.BRIDGE_GAPS == (20.0, 25.0, 30.0)


def test_preset_dispatch_by_name():
    assert C.preset("tower").experiment.type is C.ExperimentType.TOWER
    assert C.preset("bridge").experiment.type is C.ExperimentType.BRIDGE
    with pytest.raises(ValueError):
        C.preset("nope")


# --- housekeeping ------------------------------------------------------------

def test_params_are_frozen():
    p = C.tower()
    with pytest.raises(Exception):
        p.algorithm.F = 1.0  # type: ignore[misc]  # frozen dataclass


def test_describe_is_nonempty_string():
    text = C.tower().describe()
    assert isinstance(text, str) and "ReactiveBuild config" in text
    assert "F=10.0" in text
