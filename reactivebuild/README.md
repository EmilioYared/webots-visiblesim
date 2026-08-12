# reactivebuild

A bespoke Python simulator replicating **Swissler & Rubenstein (2022), *ReactiveBuild:
Environment-Adaptive Self-Assembly of Amorphous Structures*** (DARS).

This package is a **separate track** from the Webots/VisibleSim Catoms3D work in the rest
of the repo. ReactiveBuild uses FireAnt3D robots (three spheres each), builds *amorphous*
(non-latticed) structures, and is validated by a **linear 3D truss FEM** — not a physics
engine. Webots/ODE cannot reproduce its results, so we build the same kind of simulator the
authors did. Webots is kept only as an optional 3D viewer for finished structures.

Full plan and decoded fidelity spec: [`../REACTIVEBUILD_PLAN.md`](../REACTIVEBUILD_PLAN.md).

## Status

- **Phase 0 (done):** package scaffold + `config.py` (all paper constants + the four
  experiment presets) + tests.
- Phase 1+: geometry/robot model, truss FEM, sensing, algorithm, climbing, experiments,
  plots, Webots export — see the plan.

## Layout

```
reactivebuild/
  config.py        # Params + presets (tower/chain/cantilever/bridge); all paper constants
  __main__.py      # `python -m reactivebuild [tower|chain|cantilever|bridge]` prints a config
  tests/           # pytest
  results/         # generated CSVs/figures (gitignored)
```

## Quick start

```bash
# from the repo root
python -m reactivebuild tower          # print the tower config
python -m pytest reactivebuild -q      # run the tests
```

## Dependencies

Standard scientific Python, all already installed: `numpy`, `scipy`, `matplotlib`,
`pandas`, `pytest`. No new dependencies (the truss FEM is hand-rolled in numpy/scipy).
