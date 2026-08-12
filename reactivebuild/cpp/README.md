# reactivebuild / cpp

The **canonical C++ implementation** of the ReactiveBuild replication (Swissler &
Rubenstein 2022). Header-only, `namespace rb`, Eigen for linear algebra. See
[`../../REACTIVEBUILD_PLAN.md`](../../REACTIVEBUILD_PLAN.md) for the full plan.

Why C++ (not Python): to match the rest of the repo (Webots controller, VisibleSim), share
its toolchain and test convention, cross-check against our C++ work, and keep open the
option of running ReactiveBuild as a live Webots controller. The Python package under
`reactivebuild/*.py` is kept **frozen, as an independent cross-check oracle** — its golden
values (e.g. `sensed_force = 1.1547005384`) must match the C++.

## Build & test

```bash
bash reactivebuild/cpp/build_and_test.sh      # Git Bash
```
```powershell
.\reactivebuild\cpp\build_and_test.ps1        # PowerShell (wraps the above via Git Bash)
```

The script uses Webots' bundled MinGW g++ and the Eigen headers vendored at
`visible_sim/simulatorCore/src/deps`. It compiles and runs every `tests/test_*.cpp`.

## Status

- **Phase 0** `config.hpp` — Params, presets, recruit_value.  (`test_config` 45)
- **Phase 1** `geometry.hpp`, `robot.hpp` — FireAnt3D geometry + contacts.  (`test_geometry` 24, `test_robot` 17)
- **Phase 2** `fem.hpp` — 3D truss FEM (Eigen `SimplicialLDLT`) + `build_robot_fem`.  (`test_fem` 21)
- **Phase 3** `sensing.hpp` — `bundle_resultant`, `sensed_force`, connection stress.  (`test_sensing` 20)
- **Phase 4** `algorithm.hpp` — Alg. 2 recruit-value propagation (Fig. 2, −1/−2 hop decrements to a fixed point) + Alg. 1 moving-robot transition (`is_recruited`, `MovingRobotFSM`).  (`test_algorithm` 33)
- **Phase 5** `environment.hpp` (plane/edge/gap + goal + FEM anchors), `climbing.hpp` (greedy surface-walk: `settle_z`, `climb`, `spawn_on_plane`, `random_yaw`, `GreedySurfaceWalk`).  (`test_environment` 21, `test_climbing` 22)
- **Phase 6** `simulator.hpp` — `Simulator` add-one-at-a-time loop (spawn→climb→join→FEM→propagate), `StepMetrics`, `write_metrics_csv`/`write_positions_csv`; demo `apps/rb_tower.cpp`.  (`test_simulator` 52)
- **Phase 7** `apps/rb_experiment.cpp` (multi-run runner → run-tagged CSVs), spawn generalized (tower/chain/cantilever); Python `reactivebuild/analysis/` (scaling fits + figures). Tower reproduces 6/7 relationships; cantilever trend; chain partial — see [`../RESULTS.md`](../RESULTS.md).
- **Phase 8** `reactivebuild/viz_webots/export_wbt.py` → static `worlds/reactivebuild_*.wbt` for viewing in Webots (environment-aware ground: `--edge` for the chain void, `--gap` for the bridge).
- **Phase 9** [`../RESULTS.md`](../RESULTS.md) — honest scorecard vs the paper.
- **Phase 10** `climbing.hpp` `dock_climb` — full-orientation **docking** climber (samples all of SO(3), keeps the penetration-free dock closest to the goal). Closes the **chain** (sustained descent, was a ~9-robot stall) and wires the **bridge** (`Simulator::runBridge`: two arms dock toward each other from opposite lips, span-detected on latch). `apps/rb_bridge.cpp` sweeps the gaps. (`test_climbing` 28, `test_simulator` 59)

## Run the tower demo / experiments

```bash
bash reactivebuild/cpp/build_and_run_app.sh rb_tower 100 10 3 5 0   # N F B J seed
# experiment runner (multi-run): <tower|chain|cantilever> F B J N runs seed_base
bash reactivebuild/cpp/build_and_run_app.sh rb_experiment tower 5 3 5 100 30 1000
bash reactivebuild/cpp/build_and_run_app.sh rb_experiment chain 5 3 5 100 1 4000   # docking climber
# bridge runner: sweeps gaps {20,25,30}:  rb_bridge [runs] [seed_base]
bash reactivebuild/cpp/build_and_run_app.sh rb_bridge 10 2000
```
Then analyse (reads the CSVs, writes figures to `reactivebuild/results/`):
```bash
python -m reactivebuild.analysis.tower     # run from the repo root
```

## Layout

```
include/reactivebuild/*.hpp   # header-only core (namespace rb)
tests/test_*.cpp + rb_test.hpp
apps/                         # (Phase 7) CLI runners that emit results CSV
.build/                       # compiled test exes + logs (gitignored)
```
