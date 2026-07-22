# Catom3D in Webots — Tutorial & Reference

This project runs the VisibleSim **lightWalk Catoms3D** modular-robot reconfiguration
algorithm inside **Webots**, faithfully, and layers **real physics and structural-integrity
checks** on top. This file is the practical guide: what's implemented, how to see it, and
how to configure it.

---

## 1. The big picture

```
scene.json  ─┐                         ┌─►  protos/*.proto      ┐
             ├─►  tools/gen_world  ─────┤                        ├─► open in Webots
trace.csv  ──┘   (bakes physics in)    └─►  worlds/*.wbt         ┘
     │
     └─► the controller reads the SAME scene.json at runtime for its live knobs
```

- **VisibleSim** is the source of truth for the algorithm. It runs offline and exports an
  exact move trace (`trace.csv`).
- **Webots** is the visualizer + physics engine. Each catom is a `Robot` that replays its
  moves from the trace and animates each roll with a faithful port of the rotation model.
- **`scene.json`** is the single source of truth for physics and behaviour. One designated
  catom (the *analyst*) reads the whole scene each step and reports structural integrity.

You never edit the `.wbt` or `.proto` by hand — they're generated. You edit `scene.json`.

---

## 2. Quick start

You are on Windows/PowerShell. `.sh` scripts need Git Bash; use the PowerShell wrappers.

```powershell
# Regenerate the walk world from scene.json, then verify collisions + spacing
.\tools\apply_scene.ps1

# Regenerate the cantilever stress world from scene_stress.json
.\tools\apply_scene.ps1 scene_stress.json
```

Then in Webots: **File ▸ Open World**, pick a `.wbt` below, and press **Play** ▶.
After changing `scene.json`, **Reload World** (Ctrl+Shift+R) to pick up changes.

> Why not `./tools/apply_scene.sh`? PowerShell can't execute `.sh` files — it asks Windows
> which app to open them with. The `.ps1` wrapper runs the real script through Git Bash.
> (Avoid plain `bash x.sh`: your PATH's `bash` is WSL, which mishandles the paths.)

---

## 3. The three worlds

| World file | Config | What it shows |
|---|---|---|
| `worlds/catom3d_world.wbt` | `scene.json` | **Kinematic walk** — the algorithm, no physics. Fast, clean, good for watching the reconfiguration. |
| `worlds/catom3d_world_physics.wbt` | `scene.json` | **Physics walk** — same walk with mass, bonds, contacts, gravity. |
| `worlds/catom3d_stress.wbt` | `scene_stress.json` | **Cantilever stress demo** — a static structure that overhangs, tips, and breaks a bond. This is where the integrity checks and risk colors come alive. |

Each world tells the controller which config to load (via `controllerArgs`), so the same
controller drives all three.

---

## 4. Physical constraints implemented

Each constraint below lists **what it is**, **how to see it**, and **how to configure it**.

### 4.1 Rigid bodies & non-overlap (collision)
- **What:** every catom has mass and a solid bounding sphere (radius 0.05 m). Two catoms
  cannot occupy the same space — centers can't get closer than 0.10 m.
- **See it:** in a physics world, catoms rest on each other and the floor instead of
  interpenetrating. The analyst prints `[COLLISION] step N: modules i and j overlap by X mm`
  if anything ever does.
- **Configure:** `module.mass`, `module.boundingRadius`, `module.physicsEnabled`.

### 4.1a Stability — held substrate vs. dynamic parts (important!)
- **What:** a resting sheet of modules bonded rigidly in every direction is an
  *over-constrained* system that ODE (Webots' physics engine) resolves explosively —
  everything jumps and pops. The fix: **hold idle modules kinematically** at their cell,
  and let physics act only on modules explicitly marked **dynamic** (the parts meant to
  sag, fall, or break free). This is standard practice for modular-robot sims — the
  reconfiguration is kinematic; physics is selective.
- **See it:** the walk world's sheet is rock-stable (no popcorn); the mover rolls over it.
  In the stress world, the pedestal holds firm while only the arm is dynamic.
- **Configure:** `module.holdWhenIdle` (default `true`). Which modules are dynamic is set
  by the world generator (the cantilever arm); the walk holds everything idle.
  Set `holdWhenIdle: false` only if you *want* full (unstable) dynamics of the whole sheet.

### 4.2 Contact properties (friction, bounce)
- **What:** how surfaces behave on contact. Webots' defaults are wrong for this scene
  (`bounce` defaults to 0.5, which makes resting modules jitter; spheres roll forever with
  zero rolling friction), so we set them explicitly, per material pair (catom–catom and
  catom–floor).
- **See it:** catoms settle and stay put instead of buzzing or rolling away.
- **Configure:** `contact.coulombFriction`, `contact.bounce`, `contact.rollingFriction`,
  `contact.bounceVelocity`, `contact.softCFM`, `contact.softERP`, `contact.forceDependentSlip`,
  `contact.maxContactJoints`.

### 4.3 Gravity & world solver
- **What:** gravity plus ODE solver globals and auto-sleep (idle bodies stop being
  simulated, which kills residual jitter and saves CPU).
- **See it:** things fall; resting structures go quiet.
- **Configure:** `world.gravity` (a **magnitude** along the down axis, not a vector),
  `world.basicTimeStep`, `world.CFM`, `world.ERP`, `world.physicsDisableTime`,
  `world.physicsDisableLinearThreshold`, `world.physicsDisableAngularThreshold`.

### 4.4 Lattice bonds (and bond breaking)
- **What:** each module carries 12 `Connector` nodes (one per lattice direction) with a
  **tensile** and **shear** breaking strength. Adjacent modules bond; when the load on a
  bond exceeds its strength, **Webots itself breaks it**. The controller drives lock/unlock
  so a rolling module releases its bonds first (and can't be grabbed by modules it passes),
  then re-bonds on landing.
- **See it:** the sheet holds together as one body; in the stress world, the overloaded
  root bond snaps and the arm falls. A break shows up as `components` jumping 1→2 in
  `[STRUCT]`.
- **Configure:** `bonds.enabled`, `bonds.tensileStrength`, `bonds.shearStrength` (`-1` =
  unbreakable), `bonds.distanceTolerance`, `bonds.axisTolerance`, `bonds.rotationTolerance`,
  `bonds.numberOfRotations`.

### 4.5 Terrain (floor)
- **What:** a static box floor the structure rests on, auto-fitted to the modules.
- **See it:** the grey slab under the catoms.
- **Configure:** `floor.enabled`, `floor.autoFit`, `floor.margin`, `floor.thickness`;
  or set `floor.autoFit: false` and give explicit `floor.size` / `floor.center`;
  `floor.topZAuto` / `floor.topZ` control its height.

### 4.6 Motion collision fidelity (no phasing through)
- **What:** the algorithm is a discrete-cell model, so a naive replay could sweep a rolling
  catom **through** a stationary neighbour. We fix this by choosing each roll's arc using
  VisibleSim's own feasibility rule (the rotation's *blocking cells* must be clear) plus a
  clearance tiebreak, resolved offline and baked into `trace_resolved.csv`.
- **See it:** catoms roll over each other's surfaces without interpenetrating. Verify it:
  ```powershell
  .\tools\apply_scene.ps1        # prints "worst penetration: 0.97 mm ... within tolerance"
  ```
- **Configure:** it's automatic. The resolver runs in `apply_scene` / the build. Threshold
  is 2 mm (sub-mm grazing is absorbed by soft contacts).

### 4.7 Traffic spacing (`movingCatomRadius`)
- **What:** the algorithm's rule that two catoms moving at the same time on the same/adjacent
  lines keep at least *R* free cells between them (along the walk direction).
- **See it:** verified during `apply_scene` — reports the smallest gap between concurrent
  movers and whether it honours the configured radius.
- **Configure:** `algorithm.movingCatomRadius`. **Important:** in `scene.json` this only
  *declares and verifies* the value. To actually change how modules move, set the same value
  in VisibleSim's `config.xml` and regenerate `trace.csv` (see §9). `1` livelocks this
  scenario — use `0` or `2`.

---

## 5. Structural-integrity analysis

One catom (the *analyst*) reads every module's position each analysis tick and computes:

| Indicator | Meaning |
|---|---|
| **components** | connected pieces. >1 means the structure has fallen apart. |
| **articulation points** | modules whose removal disconnects the structure — single points of failure (SPOFs). |
| **overhanging** | modules with nothing directly beneath them (unsupported). |
| **CoM-in-support** | is the center of mass over the base footprint? `NO` = tip-over risk. |
| **supported** | modules resting on the ground or another module. |

It's read-only on positions, so it runs in **every** world (kinematic or physics).

---

## 6. How to *see* everything

### 6.1 Live risk coloring (the colors) 🎨
When `analysis.colorByRisk: true`, the analyst recolors each placed module every few steps
by its structural role — **visible in the Webots 3D viewport**:

| Color | Meaning |
|---|---|
| 🟢 green | healthy (supported, not a SPOF) |
| 🟠 orange | overhanging (no support beneath) |
| 🟣 purple | articulation point (single point of failure) |
| 🔴 red | both — overhanging **and** a SPOF |

**Where to see it:** open `worlds/catom3d_stress.wbt` and press Play. `scene_stress.json`
has `colorByRisk: true`, so the cantilever arm turns orange/red immediately. The walk world
has it **off** by default (keeps distinct tracking colors); turn it on by adding
`"analysis": { "colorByRisk": true }` to `scene.json` and reloading.

### 6.2 Console messages
Open the Webots console. From the analyst and modules:

| Message | Meaning |
|---|---|
| `[config] scene_stress.json` | which config was loaded (+ any warnings) |
| `[STRUCT] step N: placed=.. components=.. articulation=.. overhanging=.. CoM-in-support=..` | structural report, printed whenever it changes |
| `[COLLISION] step N: modules i and j overlap by X mm` | two solids interpenetrated |
| `[PHYSICS] <name> drifted X m off cell` | an idle module slipped/was pushed off its cell |
| `... move N/M -> (x,y,z)` | a module completed a roll |

### 6.3 The metrics CSV
`controllers/catom3d_controller/structural_metrics.csv` logs the analysis over time:
`step, total_modules, in_flight, placed, components, articulation_points, supported,
overhanging, com_x, com_y, com_in_support`. Open it in Excel to chart integrity vs time.

---

## 7. Configuration reference (`scene.json`)

Every field has a default, so a partial file still works. Unknown keys and out-of-range
values produce warnings (printed by `gen_world` and in the Webots console).

**When does a change take effect?**

| Section | Effect | To apply |
|---|---|---|
| `world`, `floor`, `module`, `contact`, `bonds`, `scenario`, `output` | baked into the world/proto | `apply_scene` **+ Reload World** |
| `playback`, `analysis` | read live by the controller | **Reload World** only |
| `algorithm.movingCatomRadius` | declares + verifies (see §4.7) | edit `config.xml` + regen trace to change motion |

<details>
<summary><b>Full field list</b></summary>

```jsonc
{
  "world": {
    "title": "...", "basicTimeStep": 8, "gravity": 9.81,   // gravity = magnitude, not a vector
    "CFM": 1e-5, "ERP": 0.2,
    "physicsDisableTime": 1.0, "physicsDisableLinearThreshold": 0.01,
    "physicsDisableAngularThreshold": 0.01
  },
  "floor": {
    "enabled": true, "autoFit": true, "margin": 0.15, "thickness": 0.01,
    "size": [1.3,0.45,0.01], "center": [1.0,0.2,0.12],   // used when autoFit=false
    "topZAuto": true, "topZ": 0.12678, "contactMaterial": "floor"
  },
  "module": {
    "physicsEnabled": true, "mass": 0.1, "boundingRadius": 0.05,
    "linearDamping": 0.2, "angularDamping": 0.2, "contactMaterial": "catom"
  },
  "contact": {
    "coulombFriction": 0.8, "bounce": 0.0, "bounceVelocity": 0.01,
    "rollingFriction": 0.05, "forceDependentSlip": 0.0,
    "softCFM": 1e-5, "softERP": 0.2, "maxContactJoints": 10
  },
  "bonds": {
    "enabled": true, "tensileStrength": 15.0, "shearStrength": 10.0,   // -1 = unbreakable
    "distanceTolerance": 0.02, "axisTolerance": 0.3,
    "rotationTolerance": 0.4, "numberOfRotations": 0
  },
  "scenario": {
    "type": "walk",                                        // or "cantilever"
    "baseW": 2, "baseD": 2, "baseH": 3, "armLength": 8, "x0": 5, "y0": 1
  },
  "algorithm": { "movingCatomRadius": 2 },                 // 1 livelocks; use 0 or 2
  "playback": { "usPerStep": 10000, "trace": "trace_resolved.csv" },
  "analysis": {
    "analyst": "r0c0", "everyNSteps": 8, "placedTolerance": 0.02,
    "metricsCsv": "structural_metrics.csv", "colorByRisk": false
  },
  "output": {
    "proto": "protos/Catom3DPhysics.proto", "world": "worlds/catom3d_world_physics.wbt",
    "protoName": "Catom3DPhysics", "controller": "catom3d_controller"
  }
}
```
</details>

---

## 8. The stress demo (cantilever)

`scene_stress.json` builds a **supported pedestal + an overhanging arm**. The analyzer
reports `articulation=7, overhanging=7, CoM-in-support=NO`, and the bonds are tuned so the
root bond is overloaded and breaks.

**Watch it:** open `worlds/catom3d_stress.wbt`, Play. The arm shows orange/red (overhang +
SPOF) from the start; under gravity it sags; the root bond snaps and the arm drops; the
console reports `components` 1→2.

**Tune the failure** in `scene_stress.json`, then `.\tools\apply_scene.ps1 scene_stress.json`:

| Want | Change |
|---|---|
| Arm **holds** (no break) | raise `bonds.tensileStrength` / `bonds.shearStrength` (e.g. 20 / 15), or set `-1` |
| Arm **breaks sooner** | lower them (e.g. 3 / 2) |
| Longer / heavier arm (more tip-over) | raise `scenario.armLength` |
| Sturdier base | raise `scenario.baseW` / `baseD` / `baseH` |
| Heavier modules (more load) | raise `module.mass` |

---

## 9. Tools reference

Run via Git Bash (`.\tools\apply_scene.ps1` wraps the common one). All are offline, no Webots.

| Tool | Purpose |
|---|---|
| `tools/gen_world.exe [config]` | generate `.proto` + `.wbt` from a scene config |
| `tools/check_collisions.exe [trace] [config]` | verify collisions + traffic spacing; `--emit <file>` bakes the collision-free arc choice into a resolved trace |
| `tools/verify_trace.exe [trace]` | prove the trace replays cell-for-cell (fidelity) |
| `tools/build_and_test.sh` | build everything, run all unit tests + regressions, regenerate both worlds |
| `tools/apply_scene.sh [config]` | regenerate one world from a config and re-verify |
| `tools/build_controller.sh` (`.ps1`) | rebuild the controller binary (Webots doesn't always do this on reload) |

**Regenerating the VisibleSim trace** (only needed to change the algorithm itself, e.g.
`movingCatomRadius`): edit `visible_sim/applicationsBin/lightWalkCatoms3DmultiLines2/config.xml`,
build VisibleSim headless (Docker), run it to produce a new `trace.csv`, then
`.\tools\apply_scene.ps1` (which re-resolves it into `trace_resolved.csv`).

---

## 10. Common recipes — "I want to…"

| Goal | Do this |
|---|---|
| **See the risk colors** | open `worlds/catom3d_stress.wbt`, press Play |
| **See colors in the walk** | add `"analysis":{"colorByRisk":true}` to `scene.json`, Reload World |
| **Make bonds weaker/stronger** | edit `bonds.*`, `.\tools\apply_scene.ps1`, Reload |
| **Heavier modules** | edit `module.mass`, `.\tools\apply_scene.ps1`, Reload |
| **Turn physics off (kinematic)** | `module.physicsEnabled: false`, regen, Reload — or just open `catom3d_world.wbt` |
| **Change how often integrity is checked** | `analysis.everyNSteps`, Reload (no regen) |
| **Slow the playback down** | raise `playback.usPerStep`, Reload |
| **Build a different stress shape** | edit `scenario.*` in `scene_stress.json`, `.\tools\apply_scene.ps1 scene_stress.json` |
| **Change traffic spacing for real** | edit `config.xml` + regen trace (§9) |
| **Run all the checks** | `& "C:\Program Files\Git\bin\bash.exe" tools/build_and_test.sh` |

---

## 11. What's verified vs. what needs your eyes

**Proven offline (no Webots needed):**
- The trace replays cell-for-cell (121/121 moves).
- No module passes through another (worst < 1 mm, within tolerance).
- The trace honours `movingCatomRadius`.
- The cantilever is structurally a cantilever (overhang, SPOFs, CoM out of support).
- Every scene.json value lands correctly in the generated world.

**Only visible in Webots (please confirm on screen):**
- Whether catoms rest stably and roll cleanly under physics.
- Whether the cantilever's root bond actually breaks and the arm falls where predicted.
- The risk colors updating live.

---

## 12. Troubleshooting

| Symptom | Cause / fix |
|---|---|
| Windows asks *"how do you want to open this .sh file?"* | You ran a `.sh` in PowerShell. Use `.\tools\apply_scene.ps1` instead. |
| Config change had no effect | Physical fields need `apply_scene` **and** Reload World; runtime fields still need Reload World. |
| **Controller edits have no effect / physics still "pops"** | **Stale controller binary.** Webots does not always rebuild the controller on world reload, so old code keeps running. Fix: `.\tools\build_controller.ps1`, then Reload World. Confirm via the console startup lines (they name `movingCatomRadius`). |
| `cc1plus` fails silently when building tools | The Webots MinGW `bin` must be on PATH — the scripts handle this; run them, don't call `g++` directly. |
| Bond strengths look wrong in a world | The walk and stress worlds use **separate** protos (`Catom3DPhysics` vs `Catom3DStress`); regenerate the one you're opening. |
| Colors don't appear | `analysis.colorByRisk` must be `true` for that world; only **placed** (not mid-roll) modules are colored. |
```
