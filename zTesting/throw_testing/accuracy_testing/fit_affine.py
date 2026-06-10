"""Fit a 2D affine aim-correction matrix from an accuracy-calibration session.

Pairs each ball landing with its commanded target via throw order (the
session-metadata JSON written by ball_butler_node), so no manual
cluster-mapping step is needed.

Coordinate frames
-----------------
ball_butler_node applies the affine in **BB-local frame**, between
``global_to_bb_local`` and ``solve_throw_local``. So we fit in BB-local
frame too: take each global-frame QTM landing, transform it to BB-local
using the BB position + yaw offset recorded in the session metadata, and
fit the affine ``measured_local → commanded_local``.

Pipeline
--------
  1. Load session metadata → list of (throw_idx, cell_idx, commanded_xyz_global).
  2. Load QTM trajectory JSON → per-ball landing point (first descending
     crossing of the target plane), in global mm.
  3. Sort QTM balls by first-tracked frame to recover throw order.
  4. Pair (i-th landing ↔ i-th scheduled commanded target).
  5. Transform both landings and commanded targets into BB-local frame
     using bb_mocap_position_mm + bb_yaw_offset_rad from the metadata.
  6. Group by cell_idx, take per-cell mean landing (BB-local).
  7. Least-squares fit a 2D affine in BB-local frame:
        [target_x_local]   [a b tx] [mean_landing_x_local]
        [target_y_local] = [c d ty] [mean_landing_y_local]
        [      1       ]   [0 0  1] [        1           ]
  8. Print before/after error report and write a matrix file in the format
     ball_butler_node already consumes (``matrix`` + ``n_pairs`` +
     ``provenance``).

Usage:
    python fit_affine.py <session_metadata.json> <qtm_export.json> \\
        [--out throw_affine_correction.json] [--plane-z 750] \\
        [--skip-throws b3 b7] [--no-plot]

The output JSON is loadable directly by ball_butler_node (set
``apply_aim_correction`` ROS param to true and point
``aim_correction_file`` at it).
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from collections import defaultdict
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

import numpy as np


GROUND_CONTACT_Z_MM = -10.0


@dataclass
class CommandedThrow:
    throw_idx: int
    cell_idx: int
    target_xyz_global: tuple[float, float, float]


@dataclass
class BallLanding:
    label: str  # e.g. "b3"
    first_frame: int  # for ordering
    landing_xy_global: np.ndarray  # shape (2,)


def load_session_metadata(path: Path) -> tuple[list[CommandedThrow], dict]:
    with open(path) as f:
        data = json.load(f)
    schedule = [
        CommandedThrow(
            throw_idx=entry["throw_idx"],
            cell_idx=entry["cell_idx"],
            target_xyz_global=tuple(entry["target_mm"]),
        )
        for entry in data["schedule"]
    ]
    return schedule, data


def _normalize_qtm_label(label: str) -> str:
    text = label.strip()
    if text.startswith("New "):
        try:
            return f"b{int(text[len('New '):])}"
        except ValueError:
            return text
    return text


def load_qtm_landings(path: Path, plane_z: float) -> list[BallLanding]:
    """For each QTM marker, find the first descending crossing of plane_z.

    Returns landings ordered by their first tracked frame (= throw order).
    """
    with open(path) as f:
        qtm = json.load(f)

    landings: list[BallLanding] = []
    for marker in qtm["Markers"]:
        name = _normalize_qtm_label(marker["Name"])
        first_frame: int | None = None
        crossing: np.ndarray | None = None
        for part in marker["Parts"]:
            values = np.array(part["Values"])
            if values.size == 0:
                continue
            start = part["Range"]["Start"]
            if first_frame is None:
                first_frame = start

            z = values[:, 2]
            below = np.where(z <= GROUND_CONTACT_Z_MM)[0]
            if len(below) > 0:
                values = values[: below[0] + 1]
                z = values[:, 2]

            for i in range(len(z) - 1):
                if z[i] > plane_z and z[i + 1] <= plane_z:
                    dz = z[i] - z[i + 1]
                    frac = (z[i] - plane_z) / dz if dz != 0 else 0.5
                    p0 = values[i, :3]
                    p1 = values[i + 1, :3]
                    crossing = (p0 + frac * (p1 - p0))[:2]
                    break
            if crossing is not None:
                break

        if first_frame is None or crossing is None:
            continue
        landings.append(
            BallLanding(label=name, first_frame=first_frame, landing_xy_global=crossing)
        )

    landings.sort(key=lambda b: b.first_frame)
    return landings


def global_to_bb_local_xy(
    x_g: float, y_g: float,
    bb_pos_mm: tuple[float, float, float],
    yaw_offset_rad: float,
) -> tuple[float, float]:
    """Match jugglebot.can.throw_ballistics.global_to_bb_local (xy only)."""
    dx = x_g - bb_pos_mm[0]
    dy = y_g - bb_pos_mm[1]
    cos_o = math.cos(yaw_offset_rad)
    sin_o = math.sin(yaw_offset_rad)
    # Rotation is the inverse of yaw_offset (global → local)
    x_l =  dx * cos_o + dy * sin_o
    y_l = -dx * sin_o + dy * cos_o
    return x_l, y_l


def pair_and_fit(
    schedule: list[CommandedThrow],
    landings: list[BallLanding],
    bb_pos_mm: tuple[float, float, float],
    yaw_offset_rad: float,
    skip_labels: set[str],
) -> tuple[np.ndarray, dict]:
    """Pair landings to throws by order, transform to BB-local, fit affine."""
    landings = [b for b in landings if b.label not in skip_labels]

    n_pairs = min(len(schedule), len(landings))
    if n_pairs < 3:
        raise ValueError(
            f"Need at least 3 paired throws to fit affine; got {n_pairs} "
            f"({len(schedule)} commanded, {len(landings)} landings detected)"
        )

    # Group by cell_idx: each cell maps mean(landings_local) → target_local
    by_cell: dict[int, list[tuple[np.ndarray, np.ndarray]]] = defaultdict(list)
    pairs_used: list[dict] = []

    for i in range(n_pairs):
        cmd = schedule[i]
        landing = landings[i]
        # Commanded target → BB-local
        tx, ty = global_to_bb_local_xy(
            cmd.target_xyz_global[0], cmd.target_xyz_global[1],
            bb_pos_mm, yaw_offset_rad,
        )
        # Measured landing → BB-local
        mx, my = global_to_bb_local_xy(
            landing.landing_xy_global[0], landing.landing_xy_global[1],
            bb_pos_mm, yaw_offset_rad,
        )
        target_local = np.array([tx, ty])
        landing_local = np.array([mx, my])
        by_cell[cmd.cell_idx].append((landing_local, target_local))
        pairs_used.append({
            "throw_idx": cmd.throw_idx,
            "cell_idx": cmd.cell_idx,
            "qtm_label": landing.label,
            "first_frame": landing.first_frame,
            "landing_xy_global": landing.landing_xy_global.tolist(),
            "landing_xy_bb_local": [float(mx), float(my)],
            "target_xy_global": list(cmd.target_xyz_global[:2]),
            "target_xy_bb_local": [float(tx), float(ty)],
        })

    src_points = []  # measured-landing BB-local (the input to the affine)
    dst_points = []  # commanded-target BB-local (the desired output)
    cell_ids = []
    for cell_idx, samples in sorted(by_cell.items()):
        landings_arr = np.array([s[0] for s in samples])
        target = samples[0][1]
        src_points.append(landings_arr.mean(axis=0))
        dst_points.append(target)
        cell_ids.append(cell_idx)

    src = np.array(src_points)
    dst = np.array(dst_points)
    if len(src) < 3:
        raise ValueError(
            f"Only {len(src)} distinct cells with landings; need ≥3 for affine fit"
        )

    # Two independent least-squares: dst_x = a*sx + b*sy + tx, dst_y = c*sx + d*sy + ty
    ones = np.ones((len(src), 1))
    src_h = np.hstack([src, ones])
    params_x, _, _, _ = np.linalg.lstsq(src_h, dst[:, 0], rcond=None)
    params_y, _, _, _ = np.linalg.lstsq(src_h, dst[:, 1], rcond=None)

    affine = np.eye(3)
    affine[0, :] = params_x
    affine[1, :] = params_y

    corrected = (affine[:2, :2] @ src.T).T + affine[:2, 2]
    errors_before = np.linalg.norm(dst - src, axis=1)
    errors_after = np.linalg.norm(dst - corrected, axis=1)

    stats = {
        "n_throws_paired": n_pairs,
        "n_cells": len(src),
        "cell_ids": cell_ids,
        "src_bb_local": src,
        "dst_bb_local": dst,
        "corrected_bb_local": corrected,
        "errors_before_mm": errors_before,
        "errors_after_mm": errors_after,
        "pairs_used": pairs_used,
    }
    return affine, stats


def print_report(affine: np.ndarray, stats: dict) -> None:
    print("\n── Affine Correction Fit (BB-local frame) ──────────")
    print(
        f"  Paired throws: {stats['n_throws_paired']}, "
        f"cells with landings: {stats['n_cells']}"
    )

    linear = affine[:2, :2]
    U, sigma, Vt = np.linalg.svd(linear)
    if np.linalg.det(U @ Vt) < 0:
        U[:, -1] *= -1
        sigma[-1] *= -1
    R = U @ Vt
    angle_deg = np.degrees(np.arctan2(R[1, 0], R[0, 0]))
    translation = affine[:2, 2]
    det = np.linalg.det(linear)

    print("\n  Affine matrix (maps measured-local → commanded-local):")
    for r in range(3):
        row = "    [ "
        for c in range(3):
            row += f"{affine[r, c]:+10.4f} "
        print(row + "]")

    print("\n  Decomposition:")
    print(f"    Scale factors:  {sigma[0]:.4f}, {sigma[1]:.4f}")
    print(f"    Rotation:       {angle_deg:+.3f}°")
    print(f"    Translation:    ({translation[0]:+.1f}, {translation[1]:+.1f}) mm")
    print(f"    Determinant:    {det:.4f}")

    eb = stats["errors_before_mm"]
    ea = stats["errors_after_mm"]
    print("\n  Per-cell error in BB-local (Euclidean, mm):")
    print(
        f"    Before:  mean {np.mean(eb):6.1f}  max {np.max(eb):6.1f}  "
        f"std {np.std(eb):6.1f}"
    )
    print(
        f"    After:   mean {np.mean(ea):6.1f}  max {np.max(ea):6.1f}  "
        f"std {np.std(ea):6.1f}"
    )
    reduction = (
        (1 - np.mean(ea) / np.mean(eb)) * 100 if np.mean(eb) > 0 else 0.0
    )
    print(f"    Reduction:   {reduction:.1f}%")


def render_plot(stats: dict, output_path: Path) -> None:
    import matplotlib.pyplot as plt

    src = stats["src_bb_local"]
    dst = stats["dst_bb_local"]
    corrected = stats["corrected_bb_local"]

    fig, ax = plt.subplots(figsize=(9, 9))
    ax.scatter(dst[:, 0], dst[:, 1], c="red", marker="x", s=120,
               linewidths=2, label="Commanded target (BB-local)", zorder=5)
    ax.scatter(src[:, 0], src[:, 1], c="orange", s=60, alpha=0.7,
               label="Measured mean landing (before)")
    ax.scatter(corrected[:, 0], corrected[:, 1], c="green", s=60, alpha=0.7,
               label="After affine correction")
    for i in range(len(src)):
        ax.plot([src[i, 0], dst[i, 0]], [src[i, 1], dst[i, 1]],
                color="orange", alpha=0.4, linewidth=0.8)
        ax.plot([corrected[i, 0], dst[i, 0]], [corrected[i, 1], dst[i, 1]],
                color="green", alpha=0.4, linewidth=0.8)
    ax.set_xlabel("X (BB-local, mm)")
    ax.set_ylabel("Y (BB-local, mm)")
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    ax.legend()
    ax.set_title("Affine aim correction: measured-local → commanded-local")
    plt.tight_layout()
    plt.savefig(output_path, dpi=150)
    print(f"  Plot saved to: {output_path}")


def write_matrix_file(
    path: Path,
    affine: np.ndarray,
    stats: dict,
    session_metadata: dict,
) -> None:
    """Write a JSON in the exact format ball_butler_node consumes."""
    output = {
        "matrix": affine.tolist(),  # 3x3, node accepts 2x3 or 3x3
        "n_pairs": stats["n_throws_paired"],
        "provenance": {
            "fit_at": datetime.now().isoformat(timespec="seconds"),
            "fit_from_session": session_metadata.get("timestamp"),
            "frame": "BB-local",
            "n_cells": stats["n_cells"],
            "mean_error_before_mm": float(np.mean(stats["errors_before_mm"])),
            "mean_error_after_mm": float(np.mean(stats["errors_after_mm"])),
            "max_error_before_mm": float(np.max(stats["errors_before_mm"])),
            "max_error_after_mm": float(np.max(stats["errors_after_mm"])),
            "bb_mocap_position_mm_at_fit": session_metadata.get("bb_mocap_position_mm"),
            "bb_yaw_offset_rad_at_fit": session_metadata.get("bb_yaw_offset_rad"),
            "pairs_used": stats["pairs_used"],
        },
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w") as f:
        json.dump(output, f, indent=2)
    print(f"\n  Matrix written to: {path}")
    print(
        "  To apply in production: copy to "
        "Jugglebot/ros_ws/src/jugglebot/resources/throw_affine_correction.json "
        "(or set ball_butler_node's aim_correction_file param to this path) "
        "and launch with apply_aim_correction:=true."
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("session_metadata", type=Path)
    parser.add_argument("qtm_export", type=Path)
    parser.add_argument("--out", type=Path,
                        default=Path("throw_affine_correction.json"))
    parser.add_argument("--plane-z", type=float, default=None,
                        help="Target plane Z (mm). Defaults to grid center Z from session metadata.")
    parser.add_argument("--skip-throws", nargs="+", default=[],
                        metavar="LABEL", help="QTM labels to exclude (e.g. b3 b7)")
    parser.add_argument("--no-plot", action="store_true")
    args = parser.parse_args()

    if not args.session_metadata.exists():
        print(f"Error: session metadata not found: {args.session_metadata}")
        return 1
    if not args.qtm_export.exists():
        print(f"Error: QTM export not found: {args.qtm_export}")
        return 1

    schedule, metadata = load_session_metadata(args.session_metadata)
    plane_z = (args.plane_z if args.plane_z is not None
               else metadata["grid_center_mm"][2])

    bb_pos_mm = tuple(metadata["bb_mocap_position_mm"])
    yaw_offset = float(metadata["bb_yaw_offset_rad"])

    print(f"Loaded session: {metadata.get('timestamp', '?')}")
    print(f"  Scheduled throws: {len(schedule)}")
    print(f"  Target plane Z (global): {plane_z:.1f} mm")
    print(f"  BB pose at session: pos={bb_pos_mm}, yaw_offset={math.degrees(yaw_offset):.2f}°")
    print(f"  Affine applied during session: {metadata.get('aim_correction_loaded')}")

    landings = load_qtm_landings(args.qtm_export, plane_z)
    print(f"  QTM landings detected: {len(landings)}")

    skip = set(args.skip_throws)
    affine, stats = pair_and_fit(
        schedule, landings, bb_pos_mm, yaw_offset, skip,
    )
    print_report(affine, stats)
    write_matrix_file(args.out, affine, stats, metadata)

    if not args.no_plot:
        plot_path = args.out.with_suffix(".png")
        try:
            render_plot(stats, plot_path)
        except Exception as e:
            print(f"  (Plot skipped: {e})")

    return 0


if __name__ == "__main__":
    sys.exit(main())
