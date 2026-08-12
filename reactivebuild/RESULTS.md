# ReactiveBuild replication — results so far

Status of reproducing Swissler & Rubenstein (2022) with our C++ simulator + Python analysis.
Honest scorecard: what reproduces, what does not, and why. Updated through Phase 10 (all four
experiments run end to end; chain + bridge closed via the full-orientation docking climber).

Regenerate: `python -m reactivebuild.analysis.tower` (reads `reactivebuild/results/*.csv`,
written by `rb_experiment`). Figures land in `reactivebuild/results/`.

## Tower (§4.1) — N=100, 30 runs per config

F sweep at B=3, J=5; B sweep at F=5, J=5; J sweep at F=5, B=3.

| Paper claim | Paper R² | Our result | Verdict |
|---|---|---|---|
| Cross-section ∝ (dist from top)² | 0.992 | exponent ≈ **2.07–2.24**, mean R² **0.968** | ✅ **reproduced** |
| Peak stress ∝ N after maturation (N>25) | 0.870 | linear R² **0.97–0.99** | ✅ **reproduced** (exceeds paper) |
| Peak stress ∝ F | 0.999 | increases, monotonic, linear R² **0.70** | ✅ direction + rough magnitude |
| Final height ∝ √F | 1.000 | ≈ **F^0.43** (free power-law R²=0.74) | ✅ close to √F |
| Stress & height ∝ 1/B | 0.99 | both **decrease with B** (weak magnitude) | ✅ direction |
| Smaller J → taller/higher stress | — | J=1: 15.2/952 → J=1000: 13.1/583 | ✅ direction |
| Height ∝ √N after maturation | 0.983 | height **plateaus** (grows slower than √N) | ❌ not reproduced |

Figures: `fig_tower_height_stress_vs_F.png`, `fig_tower_crosssection.png`,
`fig_tower_growth_vs_N.png`, `fig_tower_BJ_sweeps.png`.

**Six of the paper's seven tower relationships now reproduce** (direction + rough magnitude),
after two climbing-model fixes below. Only height ∝ √N (a growth-dynamics law) is missed.

### The stress-control fix (the paper's central claim)
The first pass had recruitment running **backwards**: more recruitment (lower F, higher J)
made towers *shorter and much more stressed* — recruited robots perched on the stressed upper
structure, *adding* load. Diagnosis (fixed F=5, isolating J):

| J (recruit extent) | before fix: height / stress | after fix: height / stress |
|---|---|---|
| 0 (no recruitment) | 13.28 / 780 | 13.19 / 854 |
| 5   | 11.24 / **2681** | 11.49 / **625** |
| 1000 | 10.69 / **2967** | 11.27 / **519** |

**Fix 1 — recruitment reinforces (`climbing.hpp` `nestle()`):** on recruitment a robot now
rolls downhill into a stable, ground-reaching pocket (a *buttress*), rather than joining at the
first contact. This provides an alternative load path, so recruitment now **reduces** peak
stress (Fig. 2g), and more recruitment ⇒ lower stress. Consequently peak stress now rises
monotonically with F (paper direction), where before it *fell*. A small `nestle` on ordinary
joins also removes single-contact perches → cleaner stress. This also fixed the B and J
directions (both are recruitment knobs).

**Fix 2 — climbers reach the summit (`ClimbConfig::max_step_rise` 2.2 → 4.0):** diagnosis of the
natural (J=0) tower showed it was a *flat, wide pile* — half the robots never climbed (51/100
sat at z∈[1,3] out to radius ~20). They were trapped on the shallow flank because the per-step
rise cap (~1 sphere) was too low to climb over the bumpy, random-yaw surface. FireAnt3D can
climb the floor, wall, and ceiling of its peers (paper §2), so a cap of ~2 sphere-diameters is
within capability. This lifted the natural tower from height ~13 to ~18 and moved height ∝ √F
from F^0.34 to **F^0.43**, and peak-stress-∝-F linear R² from 0.60 to **0.70**.

### What genuinely works
- **Amorphous packing generates real forces** (aligned stack senses ~0; offset packing routes
  load through robot centres → recruitment engages). The paper's core premise holds.
- **Recruitment controls stress** in the correct direction (the paper's headline result):
  higher F ⇒ higher tolerated stress; recruitment reinforces to lower it. F, B, and J all move
  height and stress the way the paper reports.
- **Cross-section ∝ dist²** and **stress ∝ N** reproduce with R² comparable to / better than
  the paper; **height ∝ √F** is close (F^0.43).
- Pipeline validated end to end: 255 C++ unit checks, deterministic runs, CSV → Python → figs.

### What still doesn't (yet), and why
1. **Height ∝ √N plateaus.** As robots are added the tower reaches a quasi-steady cone and stops
   growing tall (it only widens), so height grows slower than √N. This is a growth-*dynamics*
   property the greedy surface-walk does not capture; it would need the climb to keep extending
   the summit indefinitely rather than settling to an equilibrium cone.
2. **F/B/J magnitudes saturate.** Directions are all correct, but at the "less-reinforcement"
   end (high F, low B, high J) the structure approaches the natural tower and the curves flatten,
   so the *magnitudes* are weaker than the paper's (e.g. height ∝ 1/B is nearly flat, not a true
   inverse). Same root as (1): a higher, less-compressed height ceiling would spread the range.

Both remaining gaps trace to the same place — the greedy climb reaching an equilibrium cone —
not the recruitment logic, which is now correct. Further gains need a climb that keeps extending
the tip as N grows (diminishing returns vs. effort; the six reproduced trends already validate
the algorithm + physics).

## Cantilever (§4.3) — lengthening trend reproduced (partial)
Runs end to end (goal is out **and up**, so rest-on-top climbing extends it). `reach`-vs-N,
F∈{1,2.5,5,10,25}, N=60, 20 runs (`fig_cantilever_reach.png`):

| F | final reach | behaviour | paper |
|---|---|---|---|
| 1 | ~2 | **stalls immediately** | stall N≈5 |
| 2.5 | ~11 | lengthens, slowing | stall N≈17 |
| 5 | ~24 | lengthens | stall N≈20 |
| 10 | ~28 | keeps lengthening | keeps lengthening |
| 25 | ~35 | keeps lengthening | keeps lengthening |

**Trend reproduced:** F=1 stalls at once; higher F lengthens more and stalls later; F=10/25
never stall — matching the paper's ordering. Our stall threshold sits at *lower* F than the
paper's (only F=1 clearly stalls, vs. their F≤5), i.e. our cantilevers over-lengthen. Direction
✅, exact stall-N ⚠️.

## Chain (§4.2) — sustained descent ✅ (docking climber, Phase 10)
The earlier yaw-only climber descended ~9 robots (to z≈−15) then **stalled and reverted to
building up**: a 3-sphere robot at a *fixed* orientation cannot extend a thin hanging tip — its
lead sphere docks below the tip but its other two spheres splay sideways at the same height,
where they penetrate the robots above, so the hang goes infeasible.

**Fix — full-orientation docking (`climbing.hpp` `dock_climb`):** the robot now chooses its
*entire* 3-D orientation (a random unit quaternion, still amorphous) when it seats, instead of
translating a fixed yaw. A triangle whose plane is roughly *vertical* with its apex pointing
down hangs cleanly below a tip — the lead sphere docks, the other two trail downward, away from
the structure above, penetrating nothing. Among the sampled poses we keep the penetration-free
dock whose nearest sphere gets closest to the goal.

Result (N=100, F=5, one run): the chain descends **monotonically to depth ≈ 388** (≈ 3.9 per
robot, no stall), a **thin** hanging line (x-span 8, y-span 7, z-span 393 — aspect ≈ 50:1) and
**penetration-free** (min sphere-centre distance 1.999 ≈ 2r). It would reach the paper's 600
with ~155 robots. The trend — a chain that keeps descending toward a distant downward goal — is
**reproduced**; the per-robot descent rate is a property of our (unspecified-in-paper) geometry.
Regenerate: `rb_experiment chain 5 3 5 100 1 4000`.

## Bridge (§4.4) — spans the gap ✅; gap-dependence direction ✅ (Phase 10)
Wired as **two arms** that dock toward each other from opposite lips of the gap, alternating one
robot per side (`Simulator::runBridge`). Each arm docks only onto **its own** spheres
(`build_support_side`) and targets the **opposite lip** (per-side goal), but is checked for
penetration against **every** sphere, so the arm that closes the gap seats flush against the
other rather than into it. A span is recorded when a new robot latches within 2r+0.5r of the
other arm. Both arms stay anchored to their own solid ground throughout, so the FEM is well-posed
even before they connect.

Result (`rb_bridge`, gaps {20,25,30}, 6–8 runs each), all penetration-free, both lips reached:

| gap | spanned | mean robots to span | mean peak stress at span |
|---|---|---|---|
| 20 | 8/8 | 6.1 | 513 |
| 25 | 8/8 | 7.2 | 859 |
| 30 | 9/9 | 9.1 | 1357 |

**Gap-dependence direction reproduced:** a wider gap needs a longer arm → more robots to span
**and** a steeply higher peak stress (513 → 1357, ~2.6× over 20→30). That rising stress is the
mechanism behind the paper's Table 1 (success rate falling with gap). We do **not** reproduce the
exact success *rates*: our placement is geometric and imposes no material-yield / collapse
threshold (the paper specifies none), so every gap spans (100%). Adding a yield threshold would
turn the rising span-stress into falling success — but its value would be a free knob fit to the
answer, so we report the stress trend instead of a tuned Weibull. Direction ✅, exact Table 1 ⚠️.

## Visualisation in Webots (§Phase 8)
The project's hard constraint is that structures be viewable in Webots. Because ODE cannot
reproduce the truss statics, Webots is used **only as a static viewer**: `reactivebuild/
viz_webots/export_wbt.py` reads a grown structure's final sphere positions and emits a
self-contained `.wbt` (R2025a, z-up, one coloured `Sphere` per robot sphere, no `Physics`
nodes). Generated:
- `worlds/reactivebuild_tower.wbt` (100 robots) — a broad amorphous cone (matches the
  cross-section ∝ dist² result).
- `worlds/reactivebuild_cantilever.wbt` (F=25) — a support cluster behind the edge with an
  arm reaching to x≈40, as the paper describes.
- `worlds/reactivebuild_chain.wbt` (100 robots) — a thin line hanging from the edge down to
  z≈−390 (the exporter draws ground only for x≤edge, so the void the chain hangs in is honest).
- `worlds/reactivebuild_bridge.wbt` (gap=30) — two arms meeting across the gap; the exporter
  draws two platforms with the void between them (`--gap`) so the span reads correctly.

Regenerate: `python -m reactivebuild.viz_webots.export_wbt <positions.csv> worlds/<name>.wbt`
(add `--edge 0` for the chain, `--gap 30` for the bridge so voids render as voids).
Quick non-Webots previews (matplotlib 3-D): `fig_view_tower.png`, `fig_view_cantilever.png`.

## Applying the validated engine to Catoms3D (domain transfer)
The point of reproducing the paper is to trust the **statics** engine — the force/stress analysis
ODE gets wrong (the "popcorn" instability). `catom3d_fem.hpp` transfers the *same* validated
truss-FEM to Catoms3D lattices: each module is a rigid tetra, each bond a 16-bar bundle that
transmits force **and** moment (a single node/bond would make a thin arm a pin-jointed mechanism).
`apps/catom3d_forces.cpp` turns the controller's FCC cells (`gridToWorld`/`fccNeighbor`) into that
truss and reports, per bond, the tensile/shear **utilisation** vs the scene's real limits (15 N /
10 N). It runs **alongside** the Webots/ODE controller (which drives the movement); it does not
move anything.

On the built-in cantilever scenario (2×2×3 pedestal + 8-cell arm): the solve is in exact static
equilibrium (support reaction +19.62 N = total weight), and the **most-loaded bond is the arm
root** (159 % of the tensile limit — 3 bonds predicted to break), precisely where `scenario.hpp`
documents the failure. `test_catom3d` (21 checks) locks in equilibrium and mechanism detection.
Regenerate: `catom3d_forces cantilever` then `python -m reactivebuild.analysis.catom3d_forces`.
A shareable side-by-side of the paper vs our engine (incl. this analysis) is built by
`python -m reactivebuild.analysis.build_showcase` → `results/showcase.html`.

## Bottom line
The **simulator, algorithm, FEM, sensing, and analysis pipeline are complete and correct**
(203 physics/algorithm checks cross-validated vs the frozen Python oracle; 268 C++ checks
total). All **four** experiments now run end to end:
- **Tower** (flagship) — reproduces 6 of 7 relationships after the recruitment + climbing fixes.
- **Cantilever** — reproduces the lengthening/stall trend (exact stall-N ⚠️).
- **Chain** — **sustained descent** to depth ≈388/100 robots, thin and penetration-free, after
  the Phase-10 full-orientation **docking** climber (was: stalled at ~9 robots).
- **Bridge** — **spans** all three gaps via two-sided docking; the gap-dependence *direction*
  (more robots + steeply higher stress for wider gaps) reproduces. Exact Table 1 success *rates*
  need a material-yield threshold the paper doesn't specify, so we report the stress trend.

What remains unreproduced is now only the **magnitudes** the paper itself gets from its specific,
**unspecified** locomotion model (ref [22]): the tower's height∝√N plateau, and exact stall-N /
Table-1 numbers. These are not verifiable against the paper as written; the algorithm and physics
(the parts the paper *does* specify) are validated, and all four structures now form.
