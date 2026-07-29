# Rover suspension design tool

Interactive design-space explorer for the BB rover's parallel-bar (parallelogram)
suspension: two bars of length `L` link the chassis wall to the vertical wheel
plate, with a spring-damper pinned between the bars at along-bar distances `x1`
(upper) and `x2` (lower); wall pivots are `h` apart vertically.

**Open `suspension_tool.html` in any browser** (no server, no dependencies):

```
xdg-open zTesting/rover_suspension/suspension_tool.html
```

Sliders for geometry / load / travel drive an animated side view, stat tiles, and
four charts (spring length, motion ratio, spring force at 1g + a dynamic G-factor,
and pivot loads incl. bar axial + bending at the spring pins). Every chart has a
hover tooltip and a table view. Parameters persist in localStorage; "Reset
defaults" clears them.

## Model (sign conventions)

- θ = bar angle above horizontal; active height `z = L·sinθ`; θ > 0 = compression.
- `d = x2 − x1`. Spring length `s(θ) = sqrt(d² + h² − 2·h·d·sinθ)`.
- Motion ratio `ds/dz = −h·d/(L·s)` (cosθ cancels exactly).
- Spring force (virtual work): `F = N·L·s/(h·d)`, `N` = quarter-vehicle weight × G.
  `d > 0` loads the unit in compression, `d < 0` in tension.
- Pivot loads: 9-equation rigid-body statics (two bars + plate, massless links,
  vertical wheel load at horizontal offset `w` from the plate). `w` affects pivot
  loads only, never the spring force.

Note an earlier hand derivation `s = sqrt((x2−x1)² + (h/cosθ)²)` is **wrong**
(2026-07-28, confirmed by two independent blind derivations): it is symmetric in
±θ and predicts spring stroke even when `x1 = x2`, where the true length is
constant `h`.

## Verification chain

1. `verify_statics.py` (`/usr/bin/python3`, 3.8-compatible): 2000 randomized
   configurations — the 9×9 statics solve matches the virtual-work spring force
   to <1e-13 relative, spring force is independent of wheel offset, and global
   force/moment equilibrium holds. Prints fixed-case reference numbers.
2. The JS physics block in the HTML was extraction-tested in node against those
   reference numbers (26 checks).
3. The page re-runs a fixed regression case + a sweep self-check on every load —
   see the "Physics self-check" badge in the footer.
