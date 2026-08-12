# Replication Plan — Swissler & Rubenstein (2022), *ReactiveBuild*

**Paper:** *ReactiveBuild: Environment-Adaptive Self-Assembly of Amorphous Structures*, Petras Swissler & Michael Rubenstein, Northwestern University, DARS 2021/2022.
**PDF in repo:** `Swissler and Rubenstein - 2022 - ReactiveBuild Environment-Adaptive Self-Assembly of Amorphous Structures (1).pdf`
**Goal of this effort:** reproduce the paper's *results* — the four structure types (tower, chain, cantilever, bridge) and the quantitative scaling relationships and bridge success-rate table — so we can confirm our understanding and tooling are correct.

---

## 0. The one decision that shapes everything

ReactiveBuild is **not** the algorithm the current repo runs. The repo replays VisibleSim's lattice-based `lightWalkCatoms3D` (Catoms3D robots) inside **Webots (ODE physics)**. ReactiveBuild is:

- **A different robot** — FireAnt3D (three rigid spheres per robot), not Catoms3D.
- **Amorphous** — robots grab peers at arbitrary points; structures are *not* on a lattice.
- **Validated by a bespoke FEM (finite-element) stress simulator**, *not* a physics engine. The paper's entire result set (connection stresses, `sensed_force`, recruit propagation) comes out of a **linear 3D truss FEM** solved once per robot addition.

**Consequence:** Webots/ODE cannot produce these results. ODE is a rigid-body contact engine; it does not compute the small-deflection truss stresses the paper's algorithm reacts to. (This is the same limitation that produced the "popcorn" over-constraint problem in the Catoms3D work — ODE is the wrong tool for statics.) To replicate faithfully we must build the **same kind of simulator the authors built: a Python truss-FEM sim.**

> **Architecture — DECIDED (user, 2026-07-29):**
> 1. **Primary deliverable — a bespoke Python simulator** (`reactivebuild/`) that reproduces the paper's FEM + algorithm + experiments + plots. This is what actually "replicates the result."
> 2. **Webots = 3D viewer only.** After a structure is grown in the Python sim, export sphere positions to a generated `.wbt`/`.proto` so it can be *viewed* in Webots (honors the project's "must visualize in Webots" constraint) — but Webots plays no role in the science.
>
> Rejected alternative: implementing ReactiveBuild directly on ODE in Webots. It would not reproduce the stress-driven recruitment that is the heart of the paper, so it cannot replicate the results.

---

## 1. Reference model extracted from the paper (the fidelity spec)

Everything below is what our implementation must match. Section refs are to the paper.

### 1.1 Robot: FireAnt3D (§2, Fig. 1)
- Three rigid spheres arranged in a triangle, plus a robot "center". Distances are given **in sphere radii, measured to sphere centers** (§4).
- Capabilities the algorithm needs: (a) self-climb over peers / the environment, (b) send small-integer messages to *contacting* robots, (c) sense local force and the direction to a goal.
- **Contact zones = the 3 spheres** (spatial differentiation of messages). Algorithm still works with a single zone, but we replicate the 3-zone version.

### 1.2 FEM stress model (§3, Fig. 3) — the crux
Built/solved **after each robot joins**; one-step linear solve, small-deflection, **pin-jointed truss** (3 translational DOF/node, no rotational DOF).

**Nodes** — *each sphere and each robot center is itself a tetrahedron of 4 nodes* (confirmed by Fig. 3b and by the load bookkeeping below):
- Per robot: 3 spheres × 4 nodes + 1 center × 4 nodes = **16 nodes**.
- Each **environmental contact** = a tetrahedron of nodes that are **fixed** (all DOF constrained → supports).

**Elements** (linear truss, tension/compression only), three stiffness classes:
| Class | Where | Stiffness (sim units) | Colour in Fig 3c |
|---|---|---|---|
| **in-sphere** | fully connect the 4 nodes *within* each sphere-tetra (and center-tetra): 6 edges | **5e10** (near-rigid) | red |
| **robot-structure** | fully connect each sphere's 4 nodes to the center's 4 nodes (the robot's flexible frame) | **2e9** (softest → robot flexes here) | green |
| **connection** | fully connect the 4+4 nodes of two *contacting* spheres (robot–robot), or sphere↔environment-contact | **1e10** | blue |

**Loads:** a **0.25-unit gravitational load per robot *sphere node*** → 4 nodes × 0.25 = **1.0 unit weight per sphere** (this is exactly why spheres are 4-node tetrahedra). Direction = gravity (−z, or −goal-up axis). Center nodes: unloaded (see Open Question Q3).

**Force-sensor reconstruction (`sensed_force`)** (§3):
- From element (axial) forces, compute **axial & shear forces and bending & torsion moments** at each **sphere↔center connection** (the bundle of robot-structure elements between one sphere-tetra and the center-tetra). Because the bundle is spatially distributed, it transmits a resultant force **and moment** even though each element is axial-only.
- **`sensed_force` = mean, across a robot's 3 sphere-center connections, of (axial force + bending moment).** Robots can measure only axial force and bending moment (not shear/torsion).
- **Connection stress** (for reporting/plots) uses *all* forces+moments at a connection, assuming a **circular contact of radius 0.5**.

### 1.3 Algorithm (§2, Algorithms 1 & 2, Fig. 2)
**Roles:** every robot starts *moving*; transition to *structural* is **permanent**.

**Moving robot (Alg. 1):**
```
role = moving; next_dir = Sense(direction to goal)
while moving:
    StepTowards(next_dir)
    last_dir = next_dir
    next_dir = Sense(direction to goal)
    if next_dir == last_dir:            # would reverse / at closest point to goal
        role = structural
    if max(comm_in) >= 1:               # recruited by a structural neighbour
        role = structural
```
Interpretation of the transition (Fig. 2c): the robot climbs the surface toward the goal; when the greedy step no longer makes forward progress toward the goal (it has passed the closest point), it stops and joins. See Open Question Q1 for the exact climbing/step model.

**Structural robot (Alg. 2), every step, per contact zone:**
```
sensed = Sense(force)                                   # from FEM (§1.2)
recruit_value = min( floor( B ** (sensed/F - 1) ), J )  # NOTE: B is an EXPONENT base (§4)
for each contact zone z:
    in_neighbor = max(comm_in[z])       - 1             # same-zone hop out
    in_others   = max(comm_in[others])  - 2             # cross-zone (intra-robot +1) then out
    comm_out[z] = max(recruit_value, in_neighbor, in_others)
```
- **Recruit rule is exponential:** `recruit = min(floor(B^(sensed/F − 1)), J)`. At `sensed=F` → `B^0=1` → recruit 1 (begins recruiting); below F → 0; capped at J. ("exponent base B", §4). *Not* `B×(…)`.
- **Hop decrement = 1 per hop.** Inter-robot contact and intra-robot (zone-to-zone) contact are each 1 hop → same-zone passthrough costs −1, cross-zone costs −2.
- **F** = threshold force, **B** = exponent base (recruitment growth), **J** = max recruit value (jurisdiction radius in hops).

### 1.4 Simulation loop (§3)
- **One moving robot at a time.** Add a robot, let it climb & join, *then* add the next. (Multiple simultaneous movers is explicitly out of scope in the paper.)
- Re-solve the FEM after each join; propagate recruit values to quiescence between additions (or per the paper's stepping).

### 1.5 The four experiments (§4)
Distances in sphere radii; goals as offsets. 100 structures × 100 robots each unless noted.

| Structure | Env | Goal | Robot spawn |
|---|---|---|---|
| **Tower** (§4.1) | flat plane | 65 units straight up | random pos/orientation on plane, outside tower |
| **Chain** (§4.2) | plane ending in a cylinder at an edge | 600 units below the edge | random dist/orientation behind the highest/furthest-back robot |
| **Cantilever** (§4.3) | plane edge + cylinder | 45 out horizontally, 10 up | random dist/orientation behind furthest-back robot |
| **Bridge** (§4.4) | two cantilever envs across a gap (**20/25/30**) | left robots target right, right target left | added alternating L/R, one at a time |

**Parameter sweeps (§4):**
- F ∈ {1, 2.5, 5, 25} at B=3, J=5
- B ∈ {1.5, 3, 6, 10} at F ∈ {2.5, 5}, J=5
- J ∈ {1, 2, 5, 1000} at F ∈ {2.5, 5}, B=3
- (Tower/chain/cantilever figures also use F=10 for the growth illustration; Fig 4/6/8 use F=10,B=3,J=5.)

---

## 2. Target results checklist (definition of "replicated")

We are matching **trends and scaling laws** (the paper's own claims), not identical pixel plots — the climbing model and RNG differ. A claim counts as replicated if we reproduce the qualitative behaviour and the reported scaling/R² *direction and rough magnitude*.

**Tower (§4.1):**
- [ ] Higher F → taller, skinnier towers, higher peak connection stress.
- [ ] Peak stress ∝ F (paper R²=0.999).
- [ ] Final height ∝ √F (paper R²=1.000).
- [ ] After N=25: stress grows ~linearly with N; height ∝ √N.
- [ ] Cross-section (spheres within dist 1 of a height) ∝ (distance from top)² (mean R²≈0.992).
- [ ] Stress & height ∝ 1/B; taller/higher-stress as J decreases; J=5 ≈ J=1000.

**Chain (§4.2):**
- [ ] F ∝ stress (R²=0.998); shape ∝ dist_from_tip^1.75; height ∝ F^0.75.
- [ ] After N=25: stress & length ∝ log(N).
- [ ] B, J trends match tower.

**Cantilever (§4.3):**
- [ ] A support structure emerges behind the edge and grows into a small tower.
- [ ] Lengthening stalls after ~N=[5,17,20] for F=[1,2.5,5]; F=10,25 keep lengthening.

**Bridge (§4.4):**
- [ ] Reproduce Table 1 qualitatively: higher F → more reliable crossing of wider gaps; lower B → more reliable; lower J → higher success; the three bridge morphologies (two separated cantilevers / narrow connection / unified support) appear.

**Figures to regenerate:** Fig. 4/5 (tower), 6/7 (chain), 8/9 (cantilever), Table 1 (bridge), plus a Fig-3-style FEM visualization for sanity.

---

## 3. Tech stack & repo layout

**Language — C++ (DECIDED, user, 2026-07-29).** The algorithm + physics simulation are in
**C++**, to match the rest of the codebase (Webots controller, `catom3d_core.hpp`,
VisibleSim), share its toolchain and test convention, cross-check against our C++ work, and
keep open the option of running ReactiveBuild as a live Webots controller later. Python was
the wrong default; the only reason for it (a linear-algebra library for the FEM) is moot
because **Eigen is already vendored** at `visible_sim/simulatorCore/src/deps/Eigen`.

- **Compiler:** Webots' bundled MinGW g++ 14.2.0 (`C:/Program Files/Webots/msys64/mingw64/bin`).
  Toolchain gotchas (from prior work): that `bin` must be on PATH or `cc1plus` dies silently;
  `TMP`/`TEMP` must point to a writable Windows dir (we use the scratchpad).
- **FEM linear algebra:** Eigen `SparseMatrix` + `SimplicialLDLT` (verified working). Include
  with `-I visible_sim/simulatorCore/src/deps`, `#include <Eigen/Sparse>` / `<Eigen/Dense>`.
- **Core style:** header-only (like `catom3d_core.hpp`), namespace `rb`, Eigen `Vector3d`/
  `Matrix3d` throughout.
- **Tests:** C++ assert-style executables matching the existing `tools/test_*.cpp` convention
  (shared `rb_test.hpp` with `check`/`checkNear`), run by a build script. (User allowed Python
  tests, but C++ tests fit the codebase and need no language bridge.)
- **Analysis / plots (Phase 7 only):** Python (matplotlib/pandas/scipy) reads the C++ sim's
  CSV output. This produces no physics, so it doesn't violate "sim in C++"; it's just where
  scaling-law fits, figures, and the Weibull bridge table are far easier.
- **Reference oracle:** the working Python core from the first pass is **retained, frozen, as
  an independent second implementation** to cross-validate C++ golden values (e.g.
  `sensed_force = 1.1547005384` must match). Two implementations agreeing is strong evidence
  our physics is right — directly serving the "is our implementation correct?" goal.

```
reactivebuild/
  cpp/
    include/reactivebuild/
      config.hpp        # Params, presets, recruit_value  (F,B,J, stiffnesses, loads, goals)
      geometry.hpp      # sphere/tetra layout, pose transforms, contact primitives
      robot.hpp         # FireAnt3D, Role, contact detection
      fem.hpp           # TrussModel + solve_truss (Eigen sparse) + build_robot_fem
      sensing.hpp       # bundle_resultant, sensed_force, connection stress
      environment.hpp   # (Phase 5) plane / edge+cylinder / two-sided gap; goal; env anchors
      algorithm.hpp     # (Phase 4) Alg 1 + Alg 2 + recruit propagation
      climbing.hpp      # (Phase 5) greedy surface-walk
      simulator.hpp     # (Phase 6) add-one-at-a-time loop
    tests/
      rb_test.hpp       # shared check/checkNear + summary
      test_config.cpp  test_geometry.cpp  test_robot.cpp  test_fem.cpp  test_sensing.cpp
    apps/
      rb_experiment.cpp # (Phase 7) runs a scenario, writes results CSV
    build_and_test.sh   # sets PATH/TMP, -I Eigen, compiles + runs every test_*.cpp
    build_and_test.ps1  # PowerShell wrapper via Git Bash
  config.py geometry.py robot.py fem.py sensing.py __init__.py tests/  # FROZEN Python oracle
                        #   (kept in place so its pytest stays green; C++ is now canonical)
  analysis/             # (Phase 7) Python: scaling-law fits, plots, Weibull bridge table
  viz_webots/           # (Phase 8) OPTIONAL export of a final structure to a .wbt for viewing
  results/              # generated CSVs + figures (gitignored)
```

Keep this **entirely separate** from `controllers/`, `worlds/`, `visible_sim/` (aside from
including vendored Eigen) — it reuses none of the Catoms3D algorithm code.

---

## 4. Phased plan (subtasks + tests + done-criteria)

Each phase ends with passing tests and a demonstrable artifact. Build bottom-up (FEM & geometry before algorithm before experiments).

### Phase 0 — Scaffold & config *(0.5 day)*
- **0.1** Create `reactivebuild/` package, `README.md`, `pytest` setup, `results/` (gitignored).
- **0.2** `config.py`: `Params` dataclass with every paper constant (F, B, J, stiffnesses 5e10/2e9/1e10, load 0.25/node, sphere radius, goal offsets, env type, RNG seed). Provide the paper's named presets (tower/chain/cantilever/bridge defaults).
- **Test:** presets load; defaults equal paper values. **Done:** `pytest` green, `python -m reactivebuild` prints config.

### Phase 1 — Geometry & robot model *(1 day)*
- **1.1** `geometry.py`: given a robot pose (center + orientation), produce 3 sphere centers (triangle) and the 4-node tetrahedra for each sphere and the center. Fix tetra size/shape (small, relative to sphere radius) — flag as Q3.
- **1.2** Contact detection: two spheres contact when center distance ≤ 2·radius (+ tolerance); sphere↔environment contact test per env.
- **1.3** `robot.py`: FireAnt3D object (spheres, center, 3 contact zones, role, comm buffers).
- **Tests:** node counts (16/robot), sphere-sphere contact geometry, per-sphere load sums to 1.0, contact-zone assignment. **Done:** can place N robots and list all contacts.

### Phase 2 — Truss FEM solver *(2 days, critical)*
- **2.1** `fem.py`: assemble global stiffness `K` for a 3D truss (each element: `k·(local 6×6)` from direction cosines, area×E folded into the per-class stiffness constant). Sparse (`scipy.sparse`).
- **2.2** Boundary conditions: fix all DOF of environment-contact nodes.
- **2.3** Assemble load vector (0.25 per sphere node along gravity). Solve `K u = f` (sparse, one step). Recover element axial forces.
- **2.4** Element classification & stiffness assignment (in-sphere / robot-structure / connection) exactly per §1.2.
- **Tests (validation, not just unit):**
  - Single truss element under axial load → closed-form elongation.
  - A statically-determinate truss (e.g., simple 3-bar) vs hand/textbook solution.
  - Symmetry: symmetric structure+load → symmetric displacement.
  - Fixed env nodes have zero displacement; free nodes deflect downward under gravity.
- **Done:** solver matches analytic cases to <1e-6 relative.

### Phase 3 — Sensing & stress reconstruction *(1.5 days)*
- **3.1** `sensing.py`: for each sphere↔center bundle, sum member axial force vectors → net force; net moment about the connection point `Σ r×f`. Decompose into axial (along sphere→center), shear (perp), bending (moment ⟂ axis), torsion (moment ∥ axis).
- **3.2** `sensed_force` = mean over 3 spheres of (|axial| + |bending|).
- **3.3** Connection stress: from full force+moment set at a connection with circular contact r=0.5 (σ = axial/A + M·c/I with A=πr², etc.).
- **Tests:** pure vertical load on a 1-robot-on-ground case → sensed_force in a sane range; doubling load ~doubles axial term; a cantilevered robot shows bending > axial. **Done:** `sensed_force` and connection-stress are deterministic and monotonic in load. *(This is Open Question Q2 — validate against the paper's F thresholds once experiments run: recruitment should begin near sensed≈F.)*

### Phase 4 — ReactiveBuild algorithm *(1.5 days)* — **DONE (C++)**
- **4.1** `algorithm.hpp` Alg 2: `recruit_value = min(floor(B**(sensed/F−1)), J)` (in `config.hpp`); `propagate()` = per-zone `comm_out` with −1 (same zone) / −2 (other zones) hop decrements, synchronous Jacobi relaxation across the whole structure to a fixed point. `propagate_recruitment()` wraps sensed-force → recruit → propagate. Moving robots receive but never relay.
- **4.2** Alg 1 moving-robot transition: `is_recruited(comm_in)` (`max ≥ 1`), `passed_closest_point()` (greedy surface-walk join, Q1), and `MovingRobotFSM` (literal `next==last` reverse test + recruitment). Geometric `Sense()/StepTowards()` deferred to Phase 5.
- **Tests:** `test_algorithm.cpp` (33 checks) reproduces **Fig. 2**'s mechanism on hand-built contact graphs (geometry-independent, matching the paper's abstract "spheres with numbers"): source recruit 3 → cascade 3/2/1/0 by hop distance; source drops to 2 → recruitment region shrinks one hop (panels e→h); inter- vs intra-robot hop decrements checked in isolation; moving-robot non-relay; idempotence. Golden cascade cross-checked against an independent Python prototype of Alg. 2.
- **Done:** Fig. 2 recruit-value cascade + shrink reproduced; all 160 C++ checks pass.

### Phase 5 — Climbing / surface-walk model *(2–3 days, highest risk — Q1)* — **DONE (C++)**
- **5.1** `environment.hpp` — `Environment{PLANE|EDGE|GAP}` with `has_support`/`support_z`/`sphere_touches`/`anchor_contacts` (env-aware FEM fixed anchors, replaces the old `ground_contacts`), plus `make_environment(experiment)` and `goal_position(experiment, env)` (BASE vs EDGE reference). Flat/half-plane exact; the cylinder lip is a Phase-8 viewing detail (no effect on statics) and is omitted.
- **5.2** `climbing.hpp` — greedy surface-walk. `Support` = flattened structural spheres + their recruit signal; `settle_z()` = rigid vertical drop, robot stops at the highest contact (`z = max` over env + existing spheres); `climb()` = sample `n_dirs` horizontal steps, settle each, move to the greatest distance-to-goal reduction (per-step |Δz| capped so it walks, not teleports), join at the local closest point (Alg. 1 "reverse") or on recruitment (a contacted sphere with recruit ≥ 1). `spawn_on_plane()` (seeded, random angle just outside the footprint) + `random_yaw()`. **Key modelling choice:** each robot keeps a fixed *random yaw* while walking — this breaks lattice alignment so spheres nest in each other's pockets → amorphous/offset packing → real sensed forces (without it, on-axis stacks give ~0 force and never recruit). `GreedySurfaceWalk` is the swappable wrapper the Phase-6 simulator will hold.
- **5.3** Join = freeze pose + `Role::STRUCTURAL`; the add-to-FEM / re-solve / propagate loop is Phase 6 (this phase supplies the joined pose).
- **Tests:** `test_environment.cpp` (21) + `test_climbing.cpp` (22). settle exact values (plane → `plane_z+r`; on a sphere → `z_k+2r`; void → unsupported); lone robot walks under an overhead goal and joins; a climber rises above an existing base; recruitment forces an early join; determinism under a fixed seed; **physics-insight contrast: an aligned stack senses ~0 while an offset placement senses real force**; an end-to-end 6-robot tower stays FEM-solvable at every step, rises above the base, and develops `max_sensed > 1`. **Done:** towers grow upward with amorphous packing; all 203 C++ checks pass. *(Expect iteration on shape once Phase 7 compares to the figures — greedy walk can leave robots at the pile base; tune spawn/step there.)*

### Phase 6 — Simulator integration *(1 day)* — **DONE (C++)**
- **6.1** `simulator.hpp` — `Simulator(params, seed)` holds the environment, goal, RNG, structure, and a `GreedySurfaceWalk`. `run(n_robots)` = the paper's add-one-at-a-time loop: spawn → climb → join as structural → `all_contacts` + `Environment::anchor_contacts` → `build_robot_fem` + `solve_truss` → `all_sensed_forces` + `peak_connection_stress` → `propagate_recruitment` (updates `comm_out`, read by the NEXT climber) → record `StepMetrics`. One moving robot at a time (the paper's assumption). Lost-robot spawn retry guards edge/gap voids.
- **6.2** `StepMetrics` (n, height, peak_stress, max/mean sensed, contacts, anchors, max_recruit, recruited, pose, climb_steps) + `write_metrics_csv` / `write_positions_csv`. Demo app `apps/rb_tower.cpp` (+ `build_and_run_app.sh`) grows a tower and writes `results/tower_*_{metrics,positions}.csv`.
- **Tests:** `test_simulator.cpp` (49). A 30-robot tower completes with one metrics row per robot; height and contact count are non-decreasing; the tower rises and develops real sensed forces while the first robot alone senses ~0; low F engages recruitment (values propagate, ≤ J, robots join `recruited`); CSV serialization (header + row counts); determinism under a fixed seed and divergence under different seeds; non-tower spawn throws (Phase 7) rather than silently misbehaving. **Done:** full 100-robot tower runs in ~1.4 s and writes metrics + final positions CSV; all 252 C++ checks pass.
- *Known (Phase-7 tuning): the greedy walk yields a broad pile — height plateaus (~13) while the base widens as more robots fill the flanks. Correct trend (cross-section grows downward) but shape/height need tuning against Figs 4–5.*

### Phase 7 — Experiments & analysis *(3–4 days)* — **IN PROGRESS** (infra done; tower partially validated)
- **Infrastructure DONE (C++ + Python):** `apps/rb_experiment.cpp` runs `runs` sims of one (exp,F,B,J,N) and writes run-tagged `<tag>_metrics.csv` (full growth trajectory) + `<tag>_spheres.csv` (final positions). Spawn generalized for **tower** (plane) and **chain/cantilever** (`spawn_behind_edge`). Python `reactivebuild/analysis/` (`common.py` fit helpers with R², `tower.py` scaling fits + figures). Perf: ~1.4 s/run (100 robots), full FEM rebuild each step (fine at this scale).
- **7.1 Tower — 6 of 7 relationships reproduced** (F/B/J sweeps, N=100, 30 runs). See `reactivebuild/RESULTS.md`. **Reproduces (direction + rough magnitude):** cross-section ∝ dist² (R²≈0.97, exp 2.07–2.24); peak stress ∝ N post-mat. (R²≈0.99); peak stress ∝ F (monotonic, linear R²≈0.70); height ∝ √F (≈F^0.43); stress & height ∝ 1/B (both fall with B); smaller J → taller/higher-stress. **Miss:** height ∝ √N (plateaus — greedy climb reaches an equilibrium cone). **Two climbing fixes got here:** (1) recruited robots `nestle()` downhill into a ground-reaching **buttress** → recruitment now *reduces* stress (Fig. 2g), flipping peak-stress-∝-F to the correct direction and fixing B/J directions (verified isolating J at F=5: J=5 stress 2681→625); (2) `max_step_rise` 2.2→4.0 — climbers were trapped on the shallow flank (½ never climbed), fix lifted the natural tower 13→18 and height ∝ √F from F^0.34→F^0.43. Remaining magnitude saturation traces to the greedy-climb height ceiling, not the algorithm.
- **7.3 Cantilever — lengthening/stall trend reproduced (partial):** `reach`-vs-N sweep F∈{1,2.5,5,10,25} (`fig_cantilever_reach.png`). F=1 stalls immediately, higher F lengthens more, F=10/25 keep lengthening — matches the paper's ordering. Our stall threshold sits at lower F than the paper's (only F=1 clearly stalls). Direction ✓, exact stall-N ⚠️.
- **7.2 Chain — descended partially (superseded by Phase 10):** the yaw-only hang model descended ~9 robots (z≈−15) then stalled; **closed in Phase 10** by the full-orientation docking climber (sustained descent to depth ≈388/100 robots).
- **7.4 Bridge — not wired here (done in Phase 10):** now spans via two-sided docking + per-side goals.
- **Remaining Phase-7 core → done in Phase 10:** the chain/bridge "more capable climber" is now `dock_climb` (full-orientation docking). Tower/cantilever unchanged.
- **Done-criteria:** §2 checklist reproduces (trend + rough magnitude); figures in `results/`. **Tower 6/7 laws; cantilever trend; chain sustained descent (Phase 10); bridge spans (Phase 10) — documented in RESULTS.md.**

### Phase 8 — Webots visualization (honors project constraint) *(1 day)* — **DONE**
- **8.1** `viz_webots/export_wbt.py`: reads a grown structure's final sphere positions and emits a self-contained **static** `.wbt` (R2025a, z-up matching the sim; one coloured `Sphere` per robot sphere, coloured per robot so amorphous packing is visible; `TexturedBackground` like the existing worlds; NO `Physics` nodes — ODE can't reproduce the truss statics, so Webots is viewer-only). Computes a framing `Viewpoint` (look-at) automatically.
- **Generated:** `worlds/reactivebuild_tower.wbt` (100 robots, broad amorphous cone) + `worlds/reactivebuild_cantilever.wbt` (F=25, arm to x≈40 with a support cluster behind the edge). Brace/bracket-balanced; follows the exact node conventions of the existing `catom3d_*.wbt`. Non-Webots previews `results/fig_view_{tower,cantilever}.png` confirm the shapes are sane.
- **Done:** the replicated structures open in Webots (static view).

### Phase 9 — Write-up *(0.5 day)* — **DONE**
- `reactivebuild/RESULTS.md`: honest side-by-side scorecard of our numbers vs the paper's claims (tower 6/7 relationships; cantilever lengthening/stall trend; chain partial descent; bridge pending), each deviation's root cause traced to the Q1 climbing/attachment model (not the algorithm/physics), plus the two climbing fixes, the Webots visualisation, and how to regenerate every figure. `reactivebuild/cpp/README.md` covers the C++ build/run.

### Phase 10 — Full-orientation docking climber (closes chain + bridge) — **DONE (2026-08-12)**
- **Motivation:** the chain/bridge gaps all lived in one place — the yaw-only climber couldn't extend a *thin* hanging/spanning tip. Rather than adopt a physics engine (rejected: our statics are already cross-validated to 1e-6, and sphere packing is an exact distance check, so an engine adds an opaque dependency with unspecified friction/timestep knobs for no confidence gain), we built the missing locomotion ourselves.
- **10.1 `dock_climb` (`climbing.hpp`):** samples the full `SO(3)` (`random_rotation` = random unit quaternion), docks a sphere at 2r from a frontier support sphere (or rests one on the plane near the edge/lip), and keeps the **penetration-free** pose whose nearest sphere is closest to the goal. `pose_ok(...,blockers)` enforces no-overlap against *all* spheres while docking anchors come from the target arm. Amorphousness preserved (orientation still random); the goal alone decides up/down/across.
- **10.2 Chain — sustained descent ✓:** a vertical apex-down triangle hangs cleanly below the tip. Descends **monotonically to depth ≈388/100 robots** (~3.9/robot, no stall), thin (aspect ≈50:1), penetration-free (min dist 1.999). Simulator routes `CHAIN` → docking; tower/cantilever untouched.
- **10.3 Bridge — spans ✓ (`Simulator::runBridge`):** two arms alternate one robot/side, each docks onto its **own** spheres (`build_support_side`) toward the **opposite lip** (per-side goal), checked for penetration against *all* spheres; span recorded when a robot latches within 2r+0.5r of the other arm. Both arms stay ground-anchored → FEM well-posed pre-span. `apps/rb_bridge.cpp` sweeps gaps {20,25,30}: **spans all** (penetration-free, both lips reached); gap-dependence *direction* reproduced (robots-to-span 6.1/7.2/9.1; peak stress at span 513/859/1357). Exact Table 1 *rates* need a material-yield threshold the paper doesn't specify — we report the stress trend instead of a tuned Weibull.
- **10.4 Viz:** `export_wbt.py` env-aware (`--edge` chain void, `--gap` bridge platforms); generated `worlds/reactivebuild_{chain,bridge}.wbt`.
- **Tests:** `test_climbing` 22→28, `test_simulator` 52→59; suite **268** C++ checks pass. All four experiments now run end to end.

---

## 5. Open interpretation questions (my chosen defaults — flag for confirmation)

These are genuine ambiguities in the paper. I'll proceed with the default unless you say otherwise; each is isolated behind a config flag or a single module so it's cheap to change.

- **Q1 — Climbing/step model (biggest). DECIDED (user, 2026-07-29): greedy surface-walk.** The paper defers locomotion to ref [22] (which we don't have). *Chosen model:* greedy surface-walk minimizing Euclidean distance-to-goal; join at the first local minimum (or on recruitment); kept behind a swappable interface so we can refine once we see whether tower/chain shapes match the figures. The exact gait doesn't affect the FEM/algorithm results much, but it affects *which* surface positions get filled, hence structure shape — the main place our figures may diverge.
- **Q2 — `sensed_force` averaging.** "sum of axial forces and bending moments averaged across all sphere-center connections." *Default:* per-robot `mean over 3 spheres of (|axial|+|bending|)`. Alternative: sum not mean. We calibrate against the fact that recruitment should begin near `sensed≈F`.
- **Q3 — Tetrahedron size & center load.** Paper doesn't give the intra-sphere tetra dimensions or whether center nodes carry load. *Default:* small tetra (edge ≪ sphere radius) so in-sphere elements act near-rigid; center nodes unloaded (all robot weight on the 12 sphere nodes = 3 units/robot). Tetra size mainly sets numerical conditioning.
- **Q4 — Recruit formula.** Confirmed **exponential** `B^(sensed/F−1)` (from "exponent base B", §4), not linear. Locking this in.
- **Q5 — Environment-contact tetrahedra.** *Default:* each environmental contact = one fixed 4-node tetra co-located with the contacting sphere's footprint; fully connect to that sphere (connection stiffness 1e10). 
- **Q6 — Units/sphere radius.** *Default:* sphere radius = 1 unit (distances "in sphere radii"); gravity axis = −z (tower/cantilever) or as set by env.

---

## 6. Risks & mitigations
- **Climbing model divergence (Q1)** → keep it swappable; validate shape qualitatively; document deviations. Highest-risk item.
- **FEM conditioning** (stiff in-sphere 5e10 vs soft structure 2e9 = 25× spread, plus tiny tetra) → use `float64`, scipy sparse direct solver, scale check; unit-test against analytic cases (Phase 2).
- **Compute** (100 runs × 100 robots × ~12 param combos × 4 experiments, each with an incremental FEM solve) → sparse + multiprocessing; cache; if capping for time, log it.
- **Scope creep vs Catoms3D work** → ReactiveBuild lives only in `reactivebuild/`; touches nothing in the existing Webots/VisibleSim pipeline.

## 7. Suggested execution order (milestones)
1. **M1 (Phases 0–2):** working, validated truss FEM + geometry. *Proves the hard part.*
2. **M2 (Phases 3–4):** sensing + algorithm, with **Fig. 2 reproduced** — the cleanest correctness gate.
3. **M3 (Phases 5–6):** end-to-end tower grows.
4. **M4 (Phase 7):** tower results match §4.1 scaling laws → then chain, cantilever, bridge/Table 1.
5. **M5 (Phases 8–9):** Webots viewer + results write-up.

## 8. Effort estimate
~2–3 focused weeks for full replication of all four experiments + sweeps. First credible tower result (M3→M4 tower only): ~1 week.

---

### Immediate next step
Architecture (§0) and the climbing model (Q1) are both **decided**. Next: start **Phase 0 + Phase 1** — scaffold `reactivebuild/`, `config.py` with the paper's exact constants, and the geometry/robot model with tests.
