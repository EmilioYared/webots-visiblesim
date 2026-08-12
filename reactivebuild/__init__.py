"""ReactiveBuild replication package.

A bespoke Python simulator that reproduces the results of

    Swissler & Rubenstein (2022), "ReactiveBuild: Environment-Adaptive
    Self-Assembly of Amorphous Structures".

See ``REACTIVEBUILD_PLAN.md`` at the repo root for the full plan and the decoded
fidelity spec. This package is intentionally independent of the Webots/VisibleSim
Catoms3D code elsewhere in the repo; Webots is used only as an optional 3D viewer
for finished structures (see plan §0, §8).
"""

from . import config, fem, geometry, robot, sensing

__all__ = ["config", "fem", "geometry", "robot", "sensing"]
__version__ = "0.3.0"  # Phase 3: sensing (sensed_force + connection stress)
