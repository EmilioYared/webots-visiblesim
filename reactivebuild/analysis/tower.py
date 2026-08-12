"""Tower scaling-law analysis (paper section 4.1).

Reproduces / tests the paper's tower claims from the C++ simulator's CSVs:
  * peak connection stress proportional to F        (paper R^2 = 0.999)
  * final height proportional to sqrt(F)            (paper R^2 = 1.000)
  * cross-section (spheres near a height) proportional to (dist-from-top)^2  (paper R^2 ~ 0.992)
  * after maturation (N > 25): stress ~ linear in N, height ~ sqrt(N)

Run from the repo root:  python -m reactivebuild.analysis.tower
Writes figures to reactivebuild/results/.
"""
from __future__ import annotations

import os
import sys

try:  # Windows consoles default to cp1252; the summary uses ∝ √ ² symbols.
    sys.stdout.reconfigure(encoding="utf-8")
except Exception:
    pass

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from .common import (RESULTS_DIR, final_rows, linear_fit, load_metrics, load_spheres,
                     power_fit, r2_against)

F_SWEEP = [1.0, 2.5, 5.0, 25.0]
B, J = 3, 5


def _present(exp="tower"):
    fs = []
    for F in F_SWEEP:
        try:
            load_metrics(exp, F, B, J)
            fs.append(F)
        except FileNotFoundError:
            pass
    return fs


def height_and_stress_vs_F(fs):
    Fv, h_mean, h_std, s_mean, s_std = [], [], [], [], []
    for F in fs:
        fin = final_rows(load_metrics("tower", F, B, J))
        Fv.append(F)
        h_mean.append(fin["height"].mean()); h_std.append(fin["height"].std())
        s_mean.append(fin["peak_stress"].mean()); s_std.append(fin["peak_stress"].std())
    Fv = np.array(Fv); h_mean = np.array(h_mean); s_mean = np.array(s_mean)

    # height ~ sqrt(F): fit a free power law AND test the fixed 0.5 exponent.
    a, b, r2 = power_fit(Fv, h_mean)
    k = np.mean(h_mean / np.sqrt(Fv))
    r2_sqrt = r2_against(Fv, h_mean, lambda F: k * np.sqrt(F))
    # peak stress ~ F: linear fit through the data.
    slope, intercept, r2_lin = linear_fit(Fv, s_mean)

    print("--- Tower vs F (paper: height ∝ √F R²=1.000 ; peak stress ∝ F R²=0.999) ---")
    print(f"  height  = {a:.3g}·F^{b:.3f}   (free power-law R²={r2:.3f})")
    print(f"  height  vs k·√F                 (R²={r2_sqrt:.3f})")
    print(f"  stress  = {slope:.3g}·F + {intercept:.3g}   (linear R²={r2_lin:.3f})")

    fig, ax = plt.subplots(1, 2, figsize=(10, 4))
    ax[0].errorbar(Fv, h_mean, yerr=h_std, fmt="o-", capsize=3, label="sim (mean±std)")
    xf = np.linspace(min(Fv), max(Fv), 100)
    ax[0].plot(xf, k * np.sqrt(xf), "--", label=f"k·√F fit (R²={r2_sqrt:.3f})")
    ax[0].set_xlabel("F"); ax[0].set_ylabel("final height"); ax[0].set_title("Height vs F")
    ax[0].legend()
    ax[1].errorbar(Fv, s_mean, yerr=s_std, fmt="o-", capsize=3, label="sim (mean±std)")
    ax[1].plot(xf, slope * xf + intercept, "--", label=f"linear (R²={r2_lin:.3f})")
    ax[1].set_xlabel("F"); ax[1].set_ylabel("peak connection stress")
    ax[1].set_title("Peak stress vs F"); ax[1].legend()
    fig.tight_layout()
    out = os.path.join(RESULTS_DIR, "fig_tower_height_stress_vs_F.png")
    fig.savefig(out, dpi=120); plt.close(fig)
    print(f"  wrote {out}")


def cross_section(fs):
    """Spheres within distance 1 of a height, vs distance from the top, pooled over runs."""
    fig, ax = plt.subplots(figsize=(6, 4))
    print("--- Tower cross-section (paper: count ∝ dist_from_top^2, mean R²≈0.992) ---")
    r2s = []
    for F in fs:
        sph = load_spheres("tower", F, B, J)
        z = sph["z"].to_numpy()
        top = z.max()
        # sample heights from just below the top down to the base
        heights = np.arange(top - 0.5, z.min() + 1.0, -1.0)
        dist = top - heights
        count = np.array([np.sum(np.abs(z - h) <= 1.0) for h in heights], float)
        m = dist > 0.5  # ignore the very tip (few spheres, noisy)
        a, b, r2 = power_fit(dist[m], count[m])
        r2s.append(r2)
        ax.plot(dist[m], count[m], "o-", ms=3, label=f"F={F:g} (exp={b:.2f}, R²={r2:.2f})")
        print(f"  F={F:<4g}: count ∝ dist^{b:.2f}  (R²={r2:.3f})")
    if r2s:
        print(f"  mean R² = {np.nanmean(r2s):.3f}")
    ax.set_xlabel("distance from top"); ax.set_ylabel("spheres within ±1 of height")
    ax.set_title("Tower cross-section"); ax.legend(fontsize=8)
    fig.tight_layout()
    out = os.path.join(RESULTS_DIR, "fig_tower_crosssection.png")
    fig.savefig(out, dpi=120); plt.close(fig)
    print(f"  wrote {out}")


def growth_vs_N(fs):
    """Post-maturation growth: height ~ sqrt(N), stress ~ N for N > 25 (paper section 4.1)."""
    print("--- Tower growth vs N, N>25 (paper: height∝√N R²=0.983 ; stress∝N R²=0.870) ---")
    fig, ax = plt.subplots(1, 2, figsize=(10, 4))
    for F in fs:
        met = load_metrics("tower", F, B, J)
        g = met.groupby("n")
        n = g["height"].mean().index.to_numpy()
        h = g["height"].mean().to_numpy()
        s = g["peak_stress"].mean().to_numpy()
        mask = n > 25
        if mask.sum() >= 2:
            _, _, r2h = power_fit(n[mask], h[mask])
            kh = np.mean(h[mask] / np.sqrt(n[mask]))
            r2h_sqrt = r2_against(n[mask], h[mask], lambda N: kh * np.sqrt(N))
            sl, ic, r2s = linear_fit(n[mask], s[mask])
            print(f"  F={F:<4g}: height vs √N R²={r2h_sqrt:.3f} ; stress vs N (linear) R²={r2s:.3f}")
        ax[0].plot(n, h, label=f"F={F:g}")
        ax[1].plot(n, s, label=f"F={F:g}")
    ax[0].axvline(25, ls=":", c="gray"); ax[1].axvline(25, ls=":", c="gray")
    ax[0].set_xlabel("N robots"); ax[0].set_ylabel("height"); ax[0].set_title("Height vs N")
    ax[0].legend(fontsize=8)
    ax[1].set_xlabel("N robots"); ax[1].set_ylabel("peak stress"); ax[1].set_title("Peak stress vs N")
    ax[1].legend(fontsize=8)
    fig.tight_layout()
    out = os.path.join(RESULTS_DIR, "fig_tower_growth_vs_N.png")
    fig.savefig(out, dpi=120); plt.close(fig)
    print(f"  wrote {out}")


def _sweep_series(param, values, F, fixedB, fixedJ):
    xs, h, s = [], [], []
    for v in values:
        b = v if param == "B" else fixedB
        j = v if param == "J" else fixedJ
        try:
            fin = final_rows(load_metrics("tower", F, b, j))
        except FileNotFoundError:
            continue
        xs.append(v); h.append(fin["height"].mean()); s.append(fin["peak_stress"].mean())
    return np.array(xs, float), np.array(h, float), np.array(s, float)


def bj_sweeps(F=5.0):
    """Paper §4.1: stress & height ∝ 1/B; smaller J → taller/higher-stress, J=5 ≈ J=1000."""
    Bv, Bh, Bs = _sweep_series("B", [1.5, 3, 6, 10], F, B, J)
    Jv, Jh, Js = _sweep_series("J", [1, 2, 5, 1000], F, B, J)

    fig, ax = plt.subplots(1, 2, figsize=(10, 4))
    if len(Bv) >= 2:
        # height/stress ∝ 1/B: test R² of the fixed 1/B model.
        kh = np.mean(Bh * Bv); ks = np.mean(Bs * Bv)
        r2h = r2_against(Bv, Bh, lambda b: kh / b)
        r2s = r2_against(Bv, Bs, lambda b: ks / b)
        _, ph, _ = power_fit(Bv, Bh); _, ps, _ = power_fit(Bv, Bs)
        print(f"--- Tower vs B, F={F:g} (paper: stress & height ∝ 1/B) ---")
        print(f"  height ∝ B^{ph:.2f} (want −1); vs 1/B R²={r2h:.3f}")
        print(f"  stress ∝ B^{ps:.2f} (want −1); vs 1/B R²={r2s:.3f}")
        ax[0].plot(Bv, Bh, "o-", label="height")
        ax[0].plot(Bv, Bs / Bs.max() * Bh.max(), "s--", label="stress (scaled)")
        ax[0].set_xlabel("B"); ax[0].set_ylabel("final height"); ax[0].set_title(f"vs B (F={F:g})")
        ax[0].legend(fontsize=8)
    if len(Jv) >= 2:
        print(f"--- Tower vs J, F={F:g} (paper: smaller J → taller/higher stress; J=5 ≈ J=1000) ---")
        for v, hh, ss in zip(Jv, Jh, Js):
            print(f"  J={int(v):<5d}: height={hh:6.2f}  peak_stress={ss:8.2f}")
        ax[1].plot(np.log10(Jv), Jh, "o-", label="height")
        ax[1].set_xlabel("log10(J)"); ax[1].set_ylabel("final height"); ax[1].set_title(f"vs J (F={F:g})")
        ax[1].legend(fontsize=8)
    fig.tight_layout()
    out = os.path.join(RESULTS_DIR, "fig_tower_BJ_sweeps.png")
    fig.savefig(out, dpi=120); plt.close(fig)
    print(f"  wrote {out}")


def main():
    fs = _present("tower")
    if not fs:
        print("No tower CSVs in", RESULTS_DIR, "- run rb_experiment first.")
        return
    print(f"Tower analysis over F = {fs}  (B={B}, J={J})\n")
    height_and_stress_vs_F(fs)
    print()
    cross_section(fs)
    print()
    growth_vs_N(fs)
    print()
    bj_sweeps()


if __name__ == "__main__":
    main()
