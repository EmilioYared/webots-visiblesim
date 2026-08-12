"""Shared analysis helpers for the ReactiveBuild replication (Phase 7).

The C++ simulator (reactivebuild/cpp) writes the physics; this package only READS its CSVs
and computes scaling-law fits + plots. It produces no simulation results itself.

CSV layout (written by rb_experiment):
  <exp>_F<F>_B<B>_J<J>_metrics.csv : run, n, height, peak_stress, max_sensed, mean_sensed,
                                     n_contacts, n_anchors, max_recruit, recruited, reach,
                                     depth, x, y, z, climb_steps
  <exp>_F<F>_B<B>_J<J>_spheres.csv : run, robot, sphere, x, y, z   (final positions)
"""
from __future__ import annotations

import glob
import os

import numpy as np
import pandas as pd

RESULTS_DIR = os.path.join(os.path.dirname(__file__), "..", "results")
RESULTS_DIR = os.path.abspath(RESULTS_DIR)


def _tag(exp: str, F: float, B: float, J: int) -> str:
    # Match the C++ %g formatting (e.g. 2.5 -> "2.5", 3 -> "3").
    def g(x):
        s = f"{x:g}"
        return s
    return f"{exp}_F{g(F)}_B{g(B)}_J{J}"


def load_metrics(exp: str, F: float, B: float = 3, J: int = 5) -> pd.DataFrame:
    path = os.path.join(RESULTS_DIR, _tag(exp, F, B, J) + "_metrics.csv")
    return pd.read_csv(path)


def load_spheres(exp: str, F: float, B: float = 3, J: int = 5) -> pd.DataFrame:
    path = os.path.join(RESULTS_DIR, _tag(exp, F, B, J) + "_spheres.csv")
    return pd.read_csv(path)


def available_configs(exp: str) -> list[str]:
    return sorted(glob.glob(os.path.join(RESULTS_DIR, f"{exp}_*_metrics.csv")))


def final_rows(metrics: pd.DataFrame) -> pd.DataFrame:
    """One row per run: the last (largest-n) metrics row."""
    idx = metrics.groupby("run")["n"].idxmax()
    return metrics.loc[idx].reset_index(drop=True)


def power_fit(x, y):
    """Fit y = a * x^b in log-log space. Returns (a, b, r2). NaN if too few points."""
    x = np.asarray(x, float)
    y = np.asarray(y, float)
    m = (x > 0) & (y > 0) & np.isfinite(x) & np.isfinite(y)
    if m.sum() < 2:
        return float("nan"), float("nan"), float("nan")
    lx, ly = np.log(x[m]), np.log(y[m])
    b, la = np.polyfit(lx, ly, 1)
    pred = la + b * lx
    ss_res = np.sum((ly - pred) ** 2)
    ss_tot = np.sum((ly - ly.mean()) ** 2)
    r2 = 1 - ss_res / ss_tot if ss_tot > 0 else float("nan")
    return float(np.exp(la)), float(b), float(r2)


def linear_fit(x, y):
    """Fit y = m*x + c. Returns (slope, intercept, r2)."""
    x = np.asarray(x, float)
    y = np.asarray(y, float)
    m = np.isfinite(x) & np.isfinite(y)
    if m.sum() < 2:
        return float("nan"), float("nan"), float("nan")
    slope, intercept = np.polyfit(x[m], y[m], 1)
    pred = slope * x[m] + intercept
    ss_res = np.sum((y[m] - pred) ** 2)
    ss_tot = np.sum((y[m] - y[m].mean()) ** 2)
    r2 = 1 - ss_res / ss_tot if ss_tot > 0 else float("nan")
    return float(slope), float(intercept), float(r2)


def r2_against(x, y, predict):
    """R^2 of an arbitrary model predict(x) vs y (proportionality tests, fixed exponents)."""
    x = np.asarray(x, float)
    y = np.asarray(y, float)
    pred = np.asarray(predict(x), float)
    ss_res = np.sum((y - pred) ** 2)
    ss_tot = np.sum((y - y.mean()) ** 2)
    return 1 - ss_res / ss_tot if ss_tot > 0 else float("nan")
