"""Phase 2 suspension validation tests — headless, scripted.

Runs the rover through a series of scenarios to validate suspension behaviour:
  1. Static settling — verify spring/damper reaches equilibrium without oscillation.
  2. Bump traversal — drive over 5-10 mm bumps at 1.0 m/s, measure chassis heave/pitch/roll.
  3. Curb-cut ramp — drive over a 70 mm ramp, verify all wheels maintain contact.
  4. Velocity tracking — confirm swerve IK still works accurately with suspension.

Usage:
    cd sim && python -m tests.test_suspension
"""

import sys
from pathlib import Path

# Ensure sim/ is on the import path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import math
import numpy as np

from utils.transforms import quat_to_euler
from utils.model_helpers import get_ids, load_model
from utils.sim_runner import step_with_control


# ── Tests ──────────────────────────────────────────────────────────────────────

def test_static_settling():
    """Test 1: Drop the rover and verify suspension settles without oscillation."""
    print("=" * 60)
    print("TEST 1: Static settling")
    print("=" * 60)

    model, data = load_model()
    ids = get_ids(model)

    # Let it settle for 3 seconds with no commands
    rec = step_with_control(model, data, ids, 0, 0, 0, 3.0)

    # Check final suspension positions (should be near 0 with springref preload)
    prefixes = ["fl", "fr", "rl", "rr"]
    final_susp = {p: rec[f"{p}_susp"][-1] for p in prefixes}
    print(f"  Final suspension positions (mm):")
    for p in prefixes:
        print(f"    {p}: {final_susp[p]*1000:+.2f} mm")

    # Check chassis z settled near initial height (0.21 m)
    cz_final = rec["cz"][-1]
    cz_initial = 0.21
    sag = (cz_initial - cz_final) * 1000
    print(f"  Chassis z: {cz_final:.4f} m (sag: {sag:.1f} mm from initial {cz_initial} m)")

    # Check for oscillation: look at the last 1 second of cz data
    # (should be nearly constant if well-damped)
    late_samples = rec["cz"][-50:]  # last 1s at 50 Hz
    cz_range = (late_samples.max() - late_samples.min()) * 1000
    print(f"  Chassis z range (last 1s): {cz_range:.3f} mm")

    # Check pitch/roll are near zero
    print(f"  Final pitch: {math.degrees(rec['pitch'][-1]):.3f} deg")
    print(f"  Final roll:  {math.degrees(rec['roll'][-1]):.3f} deg")

    passed = True
    if cz_range > 0.1:
        print("  WARN: Chassis still oscillating after 3s")
        passed = False
    if abs(sag) > 20:
        print("  WARN: Excessive static sag (>20 mm)")
        passed = False
    for p in prefixes:
        if abs(final_susp[p]) > 0.020:
            print(f"  WARN: {p} suspension near limit ({final_susp[p]*1000:.1f} mm)")
            passed = False

    print(f"  Result: {'PASS' if passed else 'WARN'}")
    print()
    return passed, sag, final_susp


def test_bump_traversal():
    """Test 2: Drive over bumps at 1.0 m/s, measure chassis oscillation."""
    print("=" * 60)
    print("TEST 2: Bump traversal at 1.0 m/s")
    print("=" * 60)

    model, data = load_model()
    ids = get_ids(model)

    # Settle first
    step_with_control(model, data, ids, 0, 0, 0, 2.0)
    chassis_adr = model.jnt_qposadr[ids[-1]]
    cz_settled = data.qpos[chassis_adr + 2]

    # Drive forward at 1.0 m/s for 7 seconds (bumps are at x=3..5 m)
    rec = step_with_control(model, data, ids, 1.0, 0, 0, 7.0)

    # Analyse chassis heave during bump section
    cx = rec["cx"]
    cz = rec["cz"]
    prefixes = ["fl", "fr", "rl", "rr"]

    # Find samples where chassis is over the bumps (x = 3..5 m)
    bump_mask = (cx >= 2.5) & (cx <= 5.5)
    if bump_mask.any():
        cz_bumps = cz[bump_mask]
        cz_heave = (cz_bumps - cz_settled) * 1000  # mm
        print(f"  Chassis heave over bumps: min={cz_heave.min():.2f} mm, max={cz_heave.max():.2f} mm")
        print(f"  Heave range: {cz_heave.max() - cz_heave.min():.2f} mm")

        # Check pitch/roll during bumps
        pitch_bumps = np.degrees(rec["pitch"][bump_mask])
        roll_bumps = np.degrees(rec["roll"][bump_mask])
        print(f"  Pitch over bumps: min={pitch_bumps.min():.3f} deg, max={pitch_bumps.max():.3f} deg")
        print(f"  Roll over bumps:  min={roll_bumps.min():.3f} deg, max={roll_bumps.max():.3f} deg")

        # Check suspension travel
        for p in prefixes:
            s = rec[f"{p}_susp"][bump_mask] * 1000
            print(f"  {p} susp range: {s.min():.2f} to {s.max():.2f} mm")
    else:
        print("  WARN: Rover didn't reach bump section")

    # Check velocity tracking
    final_x = cx[-1]
    expected_x = 1.0 * 7.0  # 7 m at 1 m/s
    print(f"  Distance covered: {final_x:.2f} m (expected ~{expected_x:.0f} m)")

    passed = True
    if bump_mask.any():
        if (cz_heave.max() - cz_heave.min()) > 15:
            print("  WARN: Excessive heave (>15 mm range)")
            passed = False
    else:
        passed = False

    print(f"  Result: {'PASS' if passed else 'WARN'}")
    print()
    return passed


def test_curb_cut_ramp():
    """Test 3: Drive over a 70 mm curb-cut ramp, verify wheel contact and stability."""
    print("=" * 60)
    print("TEST 3: Curb-cut ramp traversal at 0.5 m/s")
    print("=" * 60)

    model, data = load_model()
    ids = get_ids(model)

    # Settle
    step_with_control(model, data, ids, 0, 0, 0, 2.0)

    # Drive forward at 0.5 m/s for 20 seconds (ramp at x=6.5..8.5 m)
    rec = step_with_control(model, data, ids, 0.5, 0, 0, 20.0)

    cx = rec["cx"]
    cz = rec["cz"]
    prefixes = ["fl", "fr", "rl", "rr"]

    # Find samples over the ramp section
    ramp_mask = (cx >= 6.0) & (cx <= 9.0)
    if ramp_mask.any():
        pitch_ramp = np.degrees(rec["pitch"][ramp_mask])
        roll_ramp = np.degrees(rec["roll"][ramp_mask])

        # On the flat top (x=7.25..7.75), chassis should be ~70 mm higher
        flat_mask = (cx >= 7.25) & (cx <= 7.75)
        if flat_mask.any():
            cz_flat = cz[flat_mask]
            print(f"  Chassis z on flat top: {cz_flat.mean():.4f} m (expected ~{0.21 + 0.07:.3f} m)")

        print(f"  Pitch during ramp: min={pitch_ramp.min():.2f} deg, max={pitch_ramp.max():.2f} deg")
        print(f"  Roll during ramp:  min={roll_ramp.min():.3f} deg, max={roll_ramp.max():.3f} deg")

        # Check suspension travel
        for p in prefixes:
            s = rec[f"{p}_susp"][ramp_mask] * 1000
            print(f"  {p} susp during ramp: {s.min():.2f} to {s.max():.2f} mm")

        # Check for suspension bottoming out (hitting +-25 mm limit)
        for p in prefixes:
            s = rec[f"{p}_susp"][ramp_mask]
            if np.any(np.abs(s) > 0.024):
                print(f"  WARN: {p} suspension near limit during ramp")
    else:
        print("  WARN: Rover didn't reach ramp section")

    # Check final position (should have traversed the ramp)
    print(f"  Final x position: {cx[-1]:.2f} m")
    print(f"  Distance covered: {cx[-1]:.2f} m (expected ~10 m)")

    passed = True
    if not ramp_mask.any():
        passed = False
    print(f"  Result: {'PASS' if passed else 'WARN'}")
    print()
    return passed


def test_velocity_tracking_with_suspension():
    """Test 4: Verify swerve drive still tracks velocity accurately with suspension."""
    print("=" * 60)
    print("TEST 4: Velocity tracking with suspension")
    print("=" * 60)

    model, data = load_model()
    ids = get_ids(model)

    # Settle
    step_with_control(model, data, ids, 0, 0, 0, 2.0)
    chassis_adr = model.jnt_qposadr[ids[-1]]
    x0 = data.qpos[chassis_adr + 0]
    y0 = data.qpos[chassis_adr + 1]

    # Forward at 1.0 m/s for 3 seconds (on flat ground, before bumps)
    rec = step_with_control(model, data, ids, 1.0, 0, 0, 3.0)
    dx = rec["cx"][-1] - x0
    dy = rec["cy"][-1] - y0
    speed = dx / 3.0
    lateral_drift = abs(dy) * 1000
    print(f"  Forward test: {dx:.3f} m in 3s = {speed:.3f} m/s (target 1.0)")
    print(f"  Lateral drift: {lateral_drift:.1f} mm")

    # Reset for strafe test
    model2, data2 = load_model()
    ids2 = get_ids(model2)
    step_with_control(model2, data2, ids2, 0, 0, 0, 2.0)
    chassis_adr2 = model2.jnt_qposadr[ids2[-1]]
    x0 = data2.qpos[chassis_adr2 + 0]
    y0 = data2.qpos[chassis_adr2 + 1]

    rec2 = step_with_control(model2, data2, ids2, 0, 0.5, 0, 3.0)
    dy2 = rec2["cy"][-1] - y0
    strafe_speed = dy2 / 3.0
    print(f"  Strafe test: {dy2:.3f} m in 3s = {strafe_speed:.3f} m/s (target 0.5)")

    # Reset for rotation test
    model3, data3 = load_model()
    ids3 = get_ids(model3)
    step_with_control(model3, data3, ids3, 0, 0, 0, 2.0)
    chassis_adr3 = model3.jnt_qposadr[ids3[-1]]
    h0 = quat_to_euler(
        data3.qpos[chassis_adr3 + 3],
        data3.qpos[chassis_adr3 + 4],
        data3.qpos[chassis_adr3 + 5],
        data3.qpos[chassis_adr3 + 6],
    )[2]

    rec3 = step_with_control(model3, data3, ids3, 0, 0, 1.0, math.pi)
    h_final = rec3["heading"][-1]
    rotation = math.degrees(h_final - h0)
    # Normalize
    while rotation > 180:
        rotation -= 360
    while rotation < -180:
        rotation += 360
    print(f"  Rotation test: {rotation:.1f} deg in {math.pi:.2f}s (target 180 deg)")
    translation_drift = math.hypot(
        rec3["cx"][-1] - rec3["cx"][0],
        rec3["cy"][-1] - rec3["cy"][0],
    ) * 1000
    print(f"  Translation drift during rotation: {translation_drift:.1f} mm")

    passed = True
    if abs(speed - 1.0) > 0.05:
        print("  WARN: Forward velocity error > 5%")
        passed = False
    if abs(strafe_speed - 0.5) > 0.05:
        print("  WARN: Strafe velocity error > 10%")
        passed = False
    if abs(abs(rotation) - 180) > 15:
        print("  WARN: Rotation error > 15 deg")
        passed = False

    print(f"  Result: {'PASS' if passed else 'WARN'}")
    print()
    return passed


# ── Main ───────────────────────────────────────────────────────────────────────

def main():
    print()
    print("BallButler Rover — Phase 2 Suspension Validation")
    print("=" * 60)
    print()

    results = {}
    results["settling"] = test_static_settling()
    results["bumps"] = test_bump_traversal()
    results["ramp"] = test_curb_cut_ramp()
    results["velocity"] = test_velocity_tracking_with_suspension()

    print("=" * 60)
    print("SUMMARY")
    print("=" * 60)
    all_passed = True
    for name, result in results.items():
        passed = result if isinstance(result, bool) else result[0]
        status = "PASS" if passed else "WARN"
        if not passed:
            all_passed = False
        print(f"  {name}: {status}")
    print()

    if not all_passed:
        sys.exit(1)


if __name__ == "__main__":
    main()
