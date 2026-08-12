"""Visualise a Catoms3D internal-force analysis (from apps/catom3d_forces).

Reads the two CSVs the analyzer writes and renders a 3-D picture of the structure with every
bond coloured by how close it is to breaking (green safe -> red over the limit) and every module
sized by the total load it carries. This is the "show people" view of the validated statics
oracle: the overloaded bonds light up exactly where the structure is about to fail.

Run from the repo root:
    python -m reactivebuild.analysis.catom3d_forces
"""
from __future__ import annotations

import os
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

RESULTS = os.path.join("reactivebuild", "results")


def _util(row):
    return max(row["util_tension"], row["util_shear"])


def main():
    bonds = pd.read_csv(os.path.join(RESULTS, "catom3d_bond_forces.csv"))
    mods = pd.read_csv(os.path.join(RESULTS, "catom3d_module_load.csv"))

    fig = plt.figure(figsize=(11, 6))
    ax = fig.add_subplot(111, projection="3d")

    # bonds coloured by utilisation: green < 0.5, orange 0.5-1, red >= 1 (predicted to break)
    def col(u):
        if u >= 1.0:
            return "#d62728"
        if u >= 0.5:
            return "#ff7f0e"
        return "#2ca02c"

    broken = 0
    for _, b in bonds.iterrows():
        u = _util(b)
        if u >= 1.0:
            broken += 1
        ax.plot([b.xa, b.xb], [b.ya, b.yb], [b.za, b.zb], color=col(u),
                lw=1.0 + 3.0 * min(u, 1.5), alpha=0.9, zorder=1)

    # modules: size by load, base (fixed) drawn as black squares
    load = mods["load"].to_numpy()
    smax = load.max() if load.max() > 0 else 1.0
    base = mods["fixed"] == 1
    ax.scatter(mods.x[~base], mods.y[~base], mods.z[~base], s=30 + 120 * load[~base] / smax,
               c="#1f77b4", edgecolors="k", linewidths=0.4, depthshade=True, zorder=2,
               label="module (size = load)")
    ax.scatter(mods.x[base], mods.y[base], mods.z[base], s=90, marker="s", c="k",
               label="fixed base (on floor)", zorder=3)

    maxu = bonds.apply(_util, axis=1).max()
    ax.set_title(f"Catoms3D internal forces (validated truss-FEM)\n"
                 f"max bond utilisation = {maxu:.2f}  ->  {broken} bond(s) predicted to break "
                 f"(green safe / orange loaded / red over limit)")
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)"); ax.set_zlabel("z (m)")

    # equal aspect
    P = mods[["x", "y", "z"]].to_numpy()
    c = P.mean(0)
    r = max(P.max(0) - P.min(0)) / 2 + 1e-6
    ax.set_xlim(c[0] - r, c[0] + r); ax.set_ylim(c[1] - r, c[1] + r); ax.set_zlim(c[2] - r, c[2] + r)
    ax.view_init(elev=18, azim=-72)
    ax.legend(loc="upper left", fontsize=8)

    out = os.path.join(RESULTS, "fig_catom3d_forces.png")
    fig.tight_layout()
    fig.savefig(out, dpi=120)
    print(f"wrote {out}  (max utilisation {maxu:.2f}, {broken} bond(s) over the limit)")


if __name__ == "__main__":
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except Exception:
        pass
    main()
