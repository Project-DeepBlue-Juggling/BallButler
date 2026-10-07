# Mirrored hand experiment — 2026-10-05

The wrong hand side is a strong explanation for most of the original aiming
error, but does not by itself reproduce the remaining corrected feed bias.
This is offline analysis only: no operational code, geometry, affine, or feed
bias settings were changed.

## Method and sign

The production `bb_release_state` uses horizontal release position
`origin + (l*cos(pitch)-d)*(cos(yaw),sin(yaw)) + s*(-sin(yaw),cos(yaw))`.
The configured s is -105.65 mm. At yaw +90 degrees (throwing along +Y), that
puts the hand at +105.65 mm X. The user-reported physical left-hand position
therefore corresponds to **s = +105.65 mm in this code's convention**.

Keep the existing inverse-kinematics command, but change the physical forward
model to positive s. The complete trajectory shifts by
`211.3*(-sin(yaw), cos(yaw))` mm. Velocity, release z, flight time and pitch
are unchanged. This is an exact ballistic simulation under the isolated-error
hypothesis, not an approximation based on a particular flight duration.
Changing the solver sign changes yaw; its pitch/range solve uses s squared.

The script imports the production yaw solver and independently checks the
closed-form displacement against production 3D release states and gravitational
propagation: 12 checks, three horizontal targets, two heights, both signs.
All pass below 0.00001 mm. The historical June solver used the same yaw formula;
later changes to pitch selection do not change this isolated-error result.

## Archived data comparison

Inputs are the actual per-throw landing/target pairs retained in the provenance
of `throw_affine_correction.json` and `throw_affine_correction_v2_residual.json`.
The original raw QTM trajectories were not re-extracted or re-paired. Results
therefore inherit the original pairing and calibrated pose assumptions.
Statistics below weight each target cell equally, averaging repeated throws.

| Case | Throws / cells | Measured miss from target | Simulated miss from target | Measured–simulated distance |
|---|---:|---:|---:|---:|
| June 9, no affine | 41 / 21 | 194.6 mm | 211.3 mm | 42.3 mm |
| June 10, deployed affine | 32 / 16 | 30.9 mm | 48.2 mm | 51.2 mm |

All distance columns are mean Euclidean distances. The uncorrected prediction
matches the measured error direction within 9.18 degrees on average. Its
prediction residual RMS is 46.3 mm, versus 195.0 mm for the ideal unmirrored
model. This is substantial agreement with **no fitted physics parameters**,
but still leaves other geometry, pose, or execution errors unresolved.

For the corrected validation, the old affine is applied to each requested
target FIRST; yaw and physical landing are then recomputed. The residual-only
validation affine is not applied. The 51.2 mm prediction discrepancy demonstrates
that a mirror-only machine is not an adequate model of all remaining errors.

The mirror error rotates with yaw, so a single affine cannot represent its
inverse exactly over the whole workspace. Fitting an affine to the noiseless
mirror-only simulated grid and actually executing those corrected commands
still leaves 25.5 mm mean error (97.2 mm worst). This is an in-sample synthetic
demonstration of model mismatch, not a proposed calibration or a validation score.

## Two-ball feeds

Production `jugglebot_launch.py` defaults `apply_aim_correction` to true, despite
the node's standalone default being false. The deployed resource still contains
the June 9 matrix. Actual runtime overrides/install state are not available here.
`sites.py` sets the catch cup plane to 830 mm; the old grid was at 750 mm.
The isolated mirror shift itself is height-independent, but other errors
absorbed into the old affine need not be.

Using the **June 9 pose only**, and a global request (-40,0) mm (illustrative
of the default P1=-50 plus the 10 mm aim-toward-A displacement), the old affine
plus mirrored hand predicts **(-11.7,+28.2) mm** landing-minus-request.
At (0,0), it predicts (-15.0,+25.2) mm. These examples do not include today's
platform-to-mocap translation or BB calibration and are not predictions of the
current sitting. The recent code/probe record reports approximately
(+25.8,+27.3) mm for 14 October 4 L3 feeds: the X sign differs. Magnitude alone
is not sufficient to claim the current feed miss has been explained.

## Recommended next experiment

1. Correct the geometry sign to positive s and disable the existing affine
   together for an initial diagnostic batch. Keeping the old affine after
   correcting geometry would retain its compensation for the removed error.
2. With a freshly established BB pose, collect repeated isolated feeds at the
   actual columns catch target and height, plus a small surrounding grid. Use
   the same launch conditions and record actual commands, pose, affine state,
   feed bias state and raw ball tracks. Avoid stacking the separate columns
   feed bias during this diagnostic batch.
3. Compare measured landing-minus-request means and scatter. A useful initial
   design is 10–20 throws at the main feed point and at least five at each
   surrounding point, randomized/interleaved to expose drift. Choose spacing
   that covers the actual feed operating region; a 3x3 grid at 50 mm spacing
   is a reasonable starting proposal, subject to reachability.
4. Fit only the residual correction needed after geometry is right, then
   validate on a separate batch at the true catch plane. A local constant
   offset may suffice at one feed point; add affine terms only if spatial
   variation warrants them. Report repeatability separately from mean bias.

The 42.3 mm historical discrepancy is not a guaranteed post-fix accuracy:
changing yaw also changes how any yaw-dependent execution error acts.
The available evidence supports fixing a substantial physical-model error
before recalibrating, while expecting some recalibration to remain necessary.

## Reproduce

`python simulate_mirrored_hand.py --jugglebot E:/PDJ/Github/Jugglebot`

Outputs: `mirrored_hand_results.json` (all cells and metrics) and
`mirrored_hand_comparison.svg` (target/measured/simulated positions).
Requires numpy and a Jugglebot checkout, no ROS installation or hardware.
