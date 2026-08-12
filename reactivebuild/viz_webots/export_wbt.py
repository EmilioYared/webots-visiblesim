"""Export a grown ReactiveBuild structure to a Webots world for viewing (Phase 8).

Honours the project's "must visualise in Webots" constraint: after the C++ simulator grows a
structure, its final sphere positions are written to a static `.wbt` (no ODE dynamics -- ODE
cannot reproduce the truss statics; this is purely a viewer). Each FireAnt3D robot's three
spheres share a colour so the amorphous packing is visible.

Both z-up (our sim and Webots R2025a default/ENU share z-up, so coordinates map directly).

Usage (from repo root):
    python -m reactivebuild.viz_webots.export_wbt <positions.csv> <out.wbt> [--run N] [--scale S]

Input CSV columns: robot,sphere,x,y,z  (rb_tower positions) or run,robot,sphere,x,y,z
(rb_experiment spheres -- pick one run with --run).
"""
from __future__ import annotations

import argparse
import colorsys
import os

import numpy as np
import pandas as pd

WEBOTS_VERSION = "R2025a"
PROTO_BASE = f"https://raw.githubusercontent.com/cyberbotics/webots/{WEBOTS_VERSION}/projects/objects/backgrounds/protos"


def _mat_to_axis_angle(R):
    """Rotation matrix -> Webots (ax, ay, az, angle). Assumes angle not near pi."""
    angle = float(np.arccos(np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)))
    if angle < 1e-8:
        return (0.0, 0.0, 1.0, 0.0)
    ax = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    n = np.linalg.norm(ax)
    if n < 1e-8:  # ~180 deg (won't happen for our look-at); fall back to up axis
        return (0.0, 0.0, 1.0, angle)
    ax = ax / n
    return (float(ax[0]), float(ax[1]), float(ax[2]), angle)


def _look_at(eye, target, up=(0.0, 0.0, 1.0)):
    """Webots Viewpoint orientation for a camera at `eye` looking at `target` (camera looks
    down its local -Z, local +Y up)."""
    eye = np.asarray(eye, float); target = np.asarray(target, float); up = np.asarray(up, float)
    f = target - eye; f /= np.linalg.norm(f)
    r = np.cross(f, up); r /= np.linalg.norm(r)
    u = np.cross(r, f)
    R = np.column_stack([r, u, -f])  # camera axes as columns
    return _mat_to_axis_angle(R)


def _robot_color(rid):
    h = (rid * 0.6180339887498949) % 1.0  # golden-ratio hue hopping -> distinct colours
    return colorsys.hsv_to_rgb(h, 0.62, 0.95)


def _platform(cx, cy, sx, sy, z, r):
    """A grey ground slab: Solid holding a Box centred at (cx,cy,z) of size (sx,sy,thin)."""
    return [
        "Solid {",
        f"  translation {cx:.3f} {cy:.3f} {z:.4f}",
        "  children [",
        "    Shape {",
        "      appearance PBRAppearance { baseColor 0.55 0.55 0.58 roughness 1 metalness 0 }",
        f"      geometry Box {{ size {sx:.3f} {sy:.3f} {0.02 * r:.4f} }}",
        "    }",
        "  ]",
        '  name "ground"',
        "}",
    ]


def export(csv_path, out_path, run=None, scale=1.0, radius=1.0, title=None,
           edge=None, gap=None):
    df = pd.read_csv(csv_path)
    if "run" in df.columns:
        run = df["run"].iloc[0] if run is None else run
        df = df[df["run"] == run]
    xyz = df[["x", "y", "z"]].to_numpy() * scale
    robots = df["robot"].to_numpy()
    r = radius * scale

    lo = xyz.min(axis=0); hi = xyz.max(axis=0)
    centroid = 0.5 * (lo + hi)
    extent = float(np.linalg.norm(hi - lo)) + 4 * r
    # camera: off to the -y, elevated, framing the whole structure
    eye = centroid + np.array([0.35, -1.0, 0.55]) * extent * 1.1
    ax, ay, az, ang = _look_at(eye, centroid)

    title = title or os.path.splitext(os.path.basename(out_path))[0]
    fx = max(hi[0] - lo[0], 4 * r) + 6 * r
    fy = max(hi[1] - lo[1], 4 * r) + 6 * r

    lines = []
    lines.append(f"#VRML_SIM {WEBOTS_VERSION} utf8")
    lines.append("# GENERATED FILE - do not edit.")
    lines.append("# Produced by reactivebuild/viz_webots/export_wbt.py from a grown structure.")
    lines.append("# Static view only (no ODE dynamics): ReactiveBuild statics come from the")
    lines.append("# C++ truss FEM, which ODE cannot reproduce. See reactivebuild/RESULTS.md.")
    lines.append("")
    lines.append(f'EXTERNPROTO "{PROTO_BASE}/TexturedBackground.proto"')
    lines.append(f'EXTERNPROTO "{PROTO_BASE}/TexturedBackgroundLight.proto"')
    lines.append("")
    lines.append("WorldInfo {")
    lines.append(f'  title "{title}"')
    lines.append("  basicTimeStep 16")
    lines.append("}")
    lines.append("Viewpoint {")
    lines.append(f"  orientation {ax:.6f} {ay:.6f} {az:.6f} {ang:.6f}")
    lines.append(f"  position {eye[0]:.4f} {eye[1]:.4f} {eye[2]:.4f}")
    lines.append("}")
    lines.append("TexturedBackground {\n}")
    lines.append("TexturedBackgroundLight {\n}")
    # ground at the plane (z=0 in sim units). Draw it to match the ENVIRONMENT so voids read
    # honestly: a bridge shows two platforms with the gap between them; an edge/chain shows a
    # single platform ending at the edge (the chain hangs into the void past it); a tower shows
    # a full floor. Without this the viewer would paint solid ground under a hanging structure.
    fz = -0.01 * r
    pad = 6 * r
    if gap is not None:  # bridge: two platforms, void of width `gap` centred at x=0
        g = gap * scale
        left_hi = -0.5 * g
        right_lo = 0.5 * g
        left_lo = min(lo[0] - pad, left_hi - pad)
        right_hi = max(hi[0] + pad, right_lo + pad)
        lines += _platform(0.5 * (left_lo + left_hi), centroid[1], left_hi - left_lo, fy, fz, r)
        lines += _platform(0.5 * (right_lo + right_hi), centroid[1], right_hi - right_lo, fy, fz, r)
    elif edge is not None:  # chain/cantilever: platform for x <= edge, void beyond
        e = edge * scale
        x_lo = min(lo[0] - pad, e - pad)
        lines += _platform(0.5 * (x_lo + e), centroid[1], e - x_lo, fy, fz, r)
    else:  # tower: full floor under the footprint
        lines += _platform(centroid[0], centroid[1], fx, fy, fz, r)
    # the structure: one Solid holding a coloured Sphere per robot sphere
    lines.append("Solid {")
    lines.append("  children [")
    for (x, y, z), rid in zip(xyz, robots):
        cr, cg, cb = _robot_color(int(rid))
        lines.append("    Pose {")
        lines.append(f"      translation {x:.4f} {y:.4f} {z:.4f}")
        lines.append("      children [")
        lines.append("        Shape {")
        lines.append(f"          appearance PBRAppearance {{ baseColor {cr:.3f} {cg:.3f} {cb:.3f} roughness 0.5 metalness 0 }}")
        lines.append(f"          geometry Sphere {{ radius {r:.4f} subdivision 2 }}")
        lines.append("        }")
        lines.append("      ]")
        lines.append("    }")
    lines.append("  ]")
    lines.append(f'  name "{title}"')
    lines.append("}")

    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
    with open(out_path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    print(f"wrote {out_path}  ({len(xyz)} spheres, {len(set(robots.tolist()))} robots)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("out")
    ap.add_argument("--run", type=int, default=None)
    ap.add_argument("--scale", type=float, default=1.0)
    ap.add_argument("--radius", type=float, default=1.0)
    ap.add_argument("--title", default=None)
    ap.add_argument("--edge", type=float, default=None,
                    help="chain/cantilever: draw ground only for x <= edge (void beyond)")
    ap.add_argument("--gap", type=float, default=None,
                    help="bridge: draw two platforms with a void of this width centred at x=0")
    a = ap.parse_args()
    export(a.csv, a.out, a.run, a.scale, a.radius, a.title, a.edge, a.gap)


if __name__ == "__main__":
    main()
