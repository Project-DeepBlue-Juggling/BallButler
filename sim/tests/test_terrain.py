"""Phase 3 terrain & obstacle traversal validation tests — headless, scripted.

Tests:
  1. Full terrain course traversal at 1.0 m/s — per-section analysis.
  2. Longitudinal slope climbing (5deg to 25deg) — identify max climbable angle.
  3. Tip-over analysis — CoM vs support polygon on slopes.

Validation criteria (from the simulation plan):
  - Successful traversal of the multi-terrain course at 1 m/s.
  - No tip-over on slopes <= 10deg.
  - Identified maximum safe slope angle.

Usage:
    cd sim && python -m tests.test_terrain
"""

import sys
from pathlib import Path

# Ensure sim/ is on the import path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import math
import mujoco
import numpy as np

from utils.constants import PREFIXES
from utils.transforms import quat_to_euler
from utils.model_helpers import get_ids, get_wheel_body_ids, load_terrain_model
from utils.sim_runner import step_with_control

# ── Constants ─────────────────────────────────────────────────────────────────

# Terrain course section boundaries (x coordinates)
COURSE_SECTIONS = {
    "concrete_1": (-2, 4),
    "curb_cut_up": (4, 5),
    "sidewalk": (5, 11),
    "curb_cut_down": (11, 12),
    "grass": (12, 18),
    "gravel": (18, 24),
    "concrete_2": (24, 30),
}

# Slope parameters
SLOPE_ANGLES = [5, 10, 15, 20, 25]
SLOPE_X_CENTERS = [2, 8, 14, 20, 26]
SLOPE_Y = 5.0
SLOPE_HX = 1.5  # half-length of each slope


# ── Helpers ───────────────────────────────────────────────────────────────────

def reposition_rover(model, data, ids, x, y, z=0.21,
                     pitch_deg=0.0, heading_deg=0.0):
    """Move the rover to a new position with optional orientation.

    Args:
        pitch_deg:   Nose-up angle in degrees (positive = uphill).
        heading_deg: CCW rotation about z in degrees.
    """
    chassis_free = ids[-1]
    adr = model.jnt_qposadr[chassis_free]

    # Zero velocities
    data.qvel[:] = 0

    # Build quaternion: heading (z) then pitch (y), applied in ZYX order
    hp = math.radians(pitch_deg) / 2
    hh = math.radians(heading_deg) / 2
    # q_heading = (cos(hh), 0, 0, sin(hh))
    # q_pitch   = (cos(hp), 0, sin(hp), 0)   — rotation about body y (nose up)
    # Combined  = q_heading * q_pitch  (MuJoCo uses wxyz)
    ch, sh = math.cos(hh), math.sin(hh)
    cp, sp = math.cos(hp), math.sin(hp)
    qw = ch * cp
    qx = -sh * sp
    qy = ch * sp
    qz = sh * cp

    data.qpos[adr:adr+3] = [x, y, z]
    data.qpos[adr+3:adr+7] = [qw, qx, qy, qz]

    # Reset joint positions
    for jid in ids[0] + ids[1] + ids[2]:  # yaw + drive + susp
        data.qpos[model.jnt_qposadr[jid]] = 0

    mujoco.mj_forward(model, data)


def tipover_margin(wheel_xpos, com_pos):
    """Minimum signed distance from projected CoM to support polygon edges.

    Positive -> CoM inside polygon (stable).
    Negative -> CoM outside polygon (tipping).

    Args:
        wheel_xpos: (4, 3) wheel body world positions.
        com_pos:    (3,)   rover centre-of-mass world position.
    """
    w2d = wheel_xpos[:, :2]
    c2d = com_pos[:2]

    # Order vertices CCW by angle from centroid
    centre = w2d.mean(axis=0)
    angles = np.arctan2(w2d[:, 1] - centre[1], w2d[:, 0] - centre[0])
    order = np.argsort(angles)
    poly = w2d[order]

    # Signed distance from CoM to each edge (positive = inside for CCW polygon)
    n = len(poly)
    min_dist = float("inf")
    for i in range(n):
        p1 = poly[i]
        p2 = poly[(i + 1) % n]
        edge = p2 - p1
        to_com = c2d - p1
        cross = edge[0] * to_com[1] - edge[1] * to_com[0]
        dist = cross / np.linalg.norm(edge)
        min_dist = min(min_dist, dist)

    return min_dist


# ── Tests ─────────────────────────────────────────────────────────────────────

def test_full_course():
    """Test 1: Drive the entire terrain course at 1.0 m/s."""
    print("=" * 70)
    print("TEST 1: Full terrain course traversal at 1.0 m/s")
    print("=" * 70)

    model, data = load_terrain_model()
    ids = get_ids(model)

    # Settle
    step_with_control(model, data, ids, 0, 0, 0, 2.0)

    # Drive forward at 1.0 m/s for 30 seconds
    rec = step_with_control(model, data, ids, 1.0, 0, 0, 30.0,
                            record_wheels=True)

    cx = rec["cx"]
    cz = rec["cz"]

    final_x = cx[-1]
    avg_speed = final_x / 30.0
    print(f"  Distance covered: {final_x:.2f} m  (expected ~30 m)")
    print(f"  Average speed: {avg_speed:.3f} m/s  (target 1.0)")
    print()

    # Per-section analysis
    print("  Per-section analysis:")
    for section, (x_start, x_end) in COURSE_SECTIONS.items():
        mask = (cx >= x_start) & (cx <= x_end)
        if not mask.any():
            print(f"    {section:16s}  NOT REACHED")
            continue

        sc_cz = cz[mask]
        sc_pitch = np.degrees(rec["pitch"][mask])
        sc_roll = np.degrees(rec["roll"][mask])
        susp_max = max(
            np.abs(rec[f"{p}_susp"][mask]).max() * 1000 for p in PREFIXES
        )

        print(f"    {section:16s}  z={sc_cz.min():.4f}..{sc_cz.max():.4f} m  "
              f"pitch={sc_pitch.min():+.1f}..{sc_pitch.max():+.1f}deg  "
              f"roll={sc_roll.min():+.2f}..{sc_roll.max():+.2f}deg  "
              f"susp_max={susp_max:.1f} mm")

    passed = final_x > 24  # Must at least reach the gravel section
    print()
    print(f"  Result: {'PASS' if passed else 'FAIL'}"
          f"  (reached x={final_x:.1f} m)")
    print()
    return passed


def test_slope_climbing():
    """Test 2: Drive up slopes of increasing angle, identify max climbable."""
    print("=" * 70)
    print("TEST 2: Longitudinal slope climbing at 0.5 m/s")
    print("=" * 70)

    max_climbable = 0

    prev_landing_end = -10.0  # no previous landing pad
    for angle, x_center in zip(SLOPE_ANGLES, SLOPE_X_CENTERS):
        model, data = load_terrain_model()
        ids = get_ids(model)

        # Position rover on flat ground before the slope, clear of any
        # previous slope's landing pad (which can be 0.5+ m above ground).
        low_edge_x = x_center - SLOPE_HX * math.cos(math.radians(angle))
        approach_x = max(prev_landing_end + 0.5, low_edge_x - 0.5)
        reposition_rover(model, data, ids, approach_x, SLOPE_Y, 0.21)

        # Record where this slope's landing pad ends for next iteration
        high_edge_x = x_center + SLOPE_HX * math.cos(math.radians(angle))
        prev_landing_end = high_edge_x + 2.0  # landing pad is 2m long

        # Settle
        step_with_control(model, data, ids, 0, 0, 0, 1.5)
        chassis_adr = model.jnt_qposadr[ids[-1]]
        x0 = data.qpos[chassis_adr]
        z0 = data.qpos[chassis_adr + 2]

        # Drive forward at 0.5 m/s for 12 seconds
        rec = step_with_control(model, data, ids, 0.5, 0, 0, 12.0)

        cx = rec["cx"]
        cz = rec["cz"]
        pitch = np.degrees(rec["pitch"])

        distance = cx[-1] - x0
        max_height = float(cz.max() - z0)
        expected_top = 3.0 * math.sin(math.radians(angle))
        max_abs_pitch = float(np.abs(pitch).max())

        # Reached the top? (height within 20% of target)
        reached_top = max_height > expected_top * 0.8
        # Clean climb? (no flip — pitch stays below 60deg)
        clean_climb = max_abs_pitch < 60.0
        climbed = reached_top and clean_climb
        if climbed:
            max_climbable = angle

        # Suspension analysis
        susp_info = ""
        for p in PREFIXES:
            s = rec[f"{p}_susp"] * 1000
            if np.abs(s).max() > 20:
                susp_info += f"  {p}={s.min():.0f}..{s.max():.0f}mm"

        if climbed:
            status = "CLIMBED"
        elif reached_top:
            status = "REACHED TOP (flipped at crest)"
        else:
            status = "STUCK/SLID"
        print(f"  {angle:2d}deg slope: dist={distance:.2f} m  max_height={max_height:.3f} m  "
              f"(target {expected_top:.3f} m)  "
              f"pitch={pitch.min():+.1f}..{pitch.max():+.1f}deg  {status}"
              f"{susp_info}")

    print()
    print(f"  Maximum climbable angle: {max_climbable}deg")
    passed = max_climbable >= 10
    print(f"  Result: {'PASS' if passed else 'FAIL'}"
          f"  (requirement: climb <=10deg slopes)")
    print()
    return passed, max_climbable


def test_slope_tipover():
    """Test 3: Tip-over margin analysis on slopes (longitudinal and lateral)."""
    print("=" * 70)
    print("TEST 3: Slope tip-over analysis")
    print("=" * 70)

    chassis_body_name = "chassis"
    max_safe_longitudinal = 0
    max_safe_lateral = 0

    # --- Longitudinal (driving up) ---
    print("  Longitudinal (stationary on slope):")
    for angle, x_center in zip(SLOPE_ANGLES, SLOPE_X_CENTERS):
        model, data = load_terrain_model()
        ids = get_ids(model)
        wheel_ids = get_wheel_body_ids(model)
        chassis_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY,
                                        chassis_body_name)

        # Position rover on the slope (midpoint height, pitched to match slope)
        slope_z = SLOPE_HX * math.sin(math.radians(angle)) + 0.21 + 0.03
        reposition_rover(model, data, ids, x_center, SLOPE_Y, slope_z,
                         pitch_deg=-angle)

        # Settle for 3 seconds
        step_with_control(model, data, ids, 0, 0, 0, 3.0)

        # Read settled state
        wheel_xpos = np.array([data.xpos[wid].copy() for wid in wheel_ids])
        com = data.subtree_com[chassis_id].copy()
        margin = tipover_margin(wheel_xpos, com)

        chassis_adr = model.jnt_qposadr[ids[-1]]
        roll, pitch_val, _ = quat_to_euler(
            data.qpos[chassis_adr+3], data.qpos[chassis_adr+4],
            data.qpos[chassis_adr+5], data.qpos[chassis_adr+6],
        )
        pitch_deg = math.degrees(pitch_val)
        roll_deg = math.degrees(roll)

        # Stable if positive margin AND not flipped (roll/pitch within reason)
        flipped = abs(roll_deg) > 45 or abs(pitch_deg) > 45
        stable = margin > 0.005 and not flipped
        if stable:
            max_safe_longitudinal = angle

        status = "FLIPPED" if flipped else ("STABLE" if stable else "TIPPING")
        print(f"    {angle:2d}deg: margin={margin*1000:+.1f} mm  "
              f"pitch={pitch_deg:+.1f}deg  roll={roll_deg:+.1f}deg  "
              f"{status}")

    print(f"    Max safe longitudinal: {max_safe_longitudinal}deg")
    print()

    # --- Lateral (stationary on slope, oriented along contour) ---
    print("  Lateral (stationary on slope, heading=90deg):")
    for angle, x_center in zip(SLOPE_ANGLES, SLOPE_X_CENTERS):
        model, data = load_terrain_model()
        ids = get_ids(model)
        wheel_ids = get_wheel_body_ids(model)
        chassis_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY,
                                        chassis_body_name)

        # Place rover on slope midpoint, facing along contour (+y)
        slope_z = SLOPE_HX * math.sin(math.radians(angle)) + 0.21 + 0.03
        reposition_rover(model, data, ids, x_center, SLOPE_Y, slope_z,
                         heading_deg=90.0)

        # Settle for 3 seconds
        step_with_control(model, data, ids, 0, 0, 0, 3.0)

        chassis_adr = model.jnt_qposadr[ids[-1]]
        roll_val, pitch_val, _ = quat_to_euler(
            data.qpos[chassis_adr+3], data.qpos[chassis_adr+4],
            data.qpos[chassis_adr+5], data.qpos[chassis_adr+6],
        )
        roll_deg = math.degrees(roll_val)
        pitch_deg = math.degrees(pitch_val)

        # Check wheel positions for margin
        wheel_xpos = np.array([data.xpos[wid].copy() for wid in wheel_ids])
        com = data.subtree_com[chassis_id].copy()
        margin = tipover_margin(wheel_xpos, com)

        flipped = abs(roll_deg) > 45 or abs(pitch_deg) > 45
        stable = margin > 0.005 and not flipped
        if stable:
            max_safe_lateral = angle

        status = "FLIPPED" if flipped else ("STABLE" if stable else "TIPPING")
        print(f"    {angle:2d}deg: margin={margin*1000:+.1f} mm  "
              f"roll={roll_deg:+.1f}deg  pitch={pitch_deg:+.1f}deg  "
              f"{status}")

    print(f"    Max safe lateral: {max_safe_lateral}deg")
    print()

    # Summary
    max_safe = min(max_safe_longitudinal, max_safe_lateral)
    passed = max_safe >= 10
    print(f"  Overall max safe slope: {max_safe}deg")
    print(f"  Result: {'PASS' if passed else 'FAIL'}"
          f"  (requirement: stable on <=10deg slopes)")
    print()
    return passed, max_safe


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    print()
    print("BallButler Rover — Phase 3 Terrain & Obstacle Traversal Validation")
    print("=" * 70)
    print()

    results = {}
    results["full_course"] = test_full_course()
    results["slope_climbing"] = test_slope_climbing()
    results["slope_tipover"] = test_slope_tipover()

    print("=" * 70)
    print("SUMMARY")
    print("=" * 70)
    all_passed = True
    for name, result in results.items():
        passed = bool(result[0]) if isinstance(result, tuple) else bool(result)
        status = "PASS" if passed else "FAIL"
        if not passed:
            all_passed = False
        print(f"  {name}: {status}")
    print()

    if all_passed:
        print("All Phase 3 validation criteria met.")
    else:
        print("Some tests did not meet criteria — see details above.")
        sys.exit(1)
    print()


if __name__ == "__main__":
    main()
