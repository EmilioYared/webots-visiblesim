"""Cantilever analysis (paper section 4.3).

Paper's key result: horizontal lengthening (`reach`) stalls after about N=[5, 17, 20] for
F=[1, 2.5, 5], while F=10 and F=25 keep lengthening for the whole run. A support structure
grows behind the edge and eventually becomes the primary build area, diminishing lengthening.

Run from the repo root:  python -m reactivebuild.analysis.cantilever
"""
from __future__ import annotations

import os
import sys

try:
    sys.stdout.reconfigure(encoding="utf-8")
except Exception:
    pass

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from .common import RESULTS_DIR, load_metrics

F_SWEEP = [1.0, 2.5, 5.0, 10.0, 25.0]
PAPER_STALL = {1.0: 5, 2.5: 17, 5.0: 20}  # F -> stall N (paper); F=10,25 keep lengthening
B, J = 3, 5


def reach_by_n(F):
    m = load_metrics("cantilever", F, B, J)
    g = m.groupby("n")["reach"].mean()
    return g.index.to_numpy(), g.to_numpy()


def stall_n(n, reach, frac=0.9):
    """Smallest N reaching `frac` of the final reach (the knee of the lengthening curve)."""
    final = reach[-1]
    if final <= 1e-9:
        return int(n[0])
    hit = np.where(reach >= frac * final)[0]
    return int(n[hit[0]]) if len(hit) else int(n[-1])


def still_growing(n, reach, tail=10, thresh=0.05):
    """True if reach grew by > thresh (fractional) over the last `tail` robots (no stall)."""
    if len(reach) <= tail or reach[-1] <= 1e-9:
        return False
    return (reach[-1] - reach[-1 - tail]) / reach[-1] > thresh


def main():
    fig, ax = plt.subplots(figsize=(7, 4.5))
    print("--- Cantilever lengthening (paper: stall N≈[5,17,20] for F=[1,2.5,5]; "
          "F=10,25 keep lengthening) ---")
    print(f"{'F':>5} {'final reach':>12} {'stall N (ours)':>15} {'paper':>7}  {'status':>18}")
    for F in F_SWEEP:
        try:
            n, r = reach_by_n(F)
        except FileNotFoundError:
            continue
        grow = still_growing(n, r)
        sN = "-" if grow else stall_n(n, r)
        paper = PAPER_STALL.get(F, "keeps lengthening")
        status = "keeps lengthening" if grow else "stalled"
        print(f"{F:>5g} {r[-1]:>12.1f} {str(sN):>15} {str(paper):>7}  {status:>18}")
        ax.plot(n, r, "-", label=f"F={F:g}" + ("" if grow else f" (stall≈N{sN})"))
    ax.set_xlabel("N robots"); ax.set_ylabel("reach past the edge")
    ax.set_title("Cantilever lengthening vs N"); ax.legend(fontsize=8)
    fig.tight_layout()
    out = os.path.join(RESULTS_DIR, "fig_cantilever_reach.png")
    fig.savefig(out, dpi=120); plt.close(fig)
    print(f"  wrote {out}")


if __name__ == "__main__":
    main()
