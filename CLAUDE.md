# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working in the **BallButler** repository.

## Project & cross-repo split

BallButler is the **V0 ball thrower** — a yaw / pitch / linear-hand mechanism driven by a Teensy 4.0 — and a subsystem of the larger **Jugglebot** robot. Work routinely spans two repos:

- **This repo (BallButler):** Teensy firmware (`ball_butler_main/`), sim, test/analysis tooling (`zTesting/`), hardware design, and the engineering `logbook/`.
- **Jugglebot repo (`~/Desktop/Jugglebot`):** the thrower's ROS2 control stack (`ball_butler_node`, `mocap_node`, `teensy_bridge_node`), the deployed `aim_correction`, the ballistics solver, and **the test suite**. It has its own mature `CLAUDE.md` — read it when touching the control stack.

Record cross-repo changes under a logbook entry's `external_changes:` field (see `logbook/README.md`, "Cross-repo note").

## Environment

- **Analysis scripts:** `/usr/bin/python3` (3.8); keep them 3.8-compatible (the 3.9 numpy is broken).
- **ROS2 / the test suite:** the PDJ venv — `~/Desktop/PDJ_venv/venv/bin/python` (or `source ~/Desktop/PDJ_venv/venv/bin/activate`). The system python lacks `hypothesis` / `mcap` / the ROS2 message stubs. Use `~/Desktop/Jugglebot/run_tests.sh`.
- **Flashing:** `ball_butler_main` is a **Teensy 4.0**; `pio upload` is broken on the Jetson (GLIBC) — use `/home/jetson/bin/teensy_loader_cli` + a 134-baud reboot, loader-first. **Two-device hazard** when both Teensys are attached. Full recipe + serial ports are in the recalled memory.

## Workflow

### Checkpoint before sinking effort  ← the most important rule here

Domain and physical context lives with the user, not with you. Before you spend real effort, **stop and do a brief inline sync** — one or two lines stating the approach and the key assumption it rests on — then wait for a quick yes/adjust. Three triggers:

1. **Before committing to a non-trivial approach.** Once you've decided *how* to solve something ("fit a kinematic model" vs. "ship a measured offset"), state it and its assumption and confirm *before* you build or deploy.
2. **When leaning on a domain/physical assumption the user could correct.** Yaw matters; the hand releases at a fixed point along the stroke (`x2`); the temporal offset must make BB throw *earlier*. Surface the assumption explicitly — the user catches a wrong one in seconds.
3. **When deviating from the agreed plan.** Don't silently change course; say so and re-sync.

This is cheap and load-bearing. The 2026-06 temporal-accuracy chapter burned multiple sessions because effort was sunk *before* a sync: a black-box affine was built and deployed before confirming it matched the plan; a "ball-inertia" feedforward was coded and flashed and made the lag **worse** (44→56 ms); a kinematic system-ID was pursued for sessions before its instability was accepted. Each would have been a 30-second conversation up front.

**Root-cause hunts — pre-register the fallback.** Chasing a physical root cause over a pragmatic fix is the right *default*, but before you start, state the **fallback** and the **decision criterion** that flips you to it (e.g. "if the kinematic scales aren't stable across two runs, ship the measured offset"). That turns the pivot into a planned checkpoint instead of a sunk-cost spiral, and keeps you shippable if the hunt fails.

### Suspect the instrument before the physics

When a result is surprising, **distrust the measurement before you conclude anything about the physics.** Reach for the most *direct* ground-truth signal early — an actuator encoder, a contact sensor, a monotonic clock — over an inferred or derived one (a mocap velocity fit, a cross-clock timestamp, an ill-conditioned geometry solve). Before a run, write down what each outcome would *mean*, so you don't re-interpret after the fact.

This project's three longest investigations all reached confident *physical* conclusions that were really **measurement artifacts** — "temporal warm-up drift" (a clock-sync slew), a "+3° steeper / +10 % hot launch" (mocap velocity sampled before release), and absurd kinematic geometry (`l→285`, `s→+120`, an ill-conditioned fit). Each was settled only by a direct sensor: the hand encoder, the monotonic clock, the cone piezo.

### Run tests + commit at the session boundary

End each working session as one unit: **close the logbook entry, run the relevant suite, and commit the logical unit.** Don't let changes or red tests accumulate across sessions — the 2026-06 chapter surfaced 43 uncommitted files and 68 failing tests only at commit time. The Jugglebot suite (`~/Desktop/Jugglebot/run_tests.sh`) is the gate for control-stack changes; firmware/tooling changes get their own verification. Checklist: `logbook/README.md` → "Session close."

### Proactively suggest when to close out a session

Watch for natural session boundaries and **offer** to wrap up and start fresh — don't wait to be asked. Good moments: a unit of work is done, committed, and its logbook entry is closed; the conversation has run long / context is getting large; the topic is about to switch to an unrelated subsystem. Keep it a one-line offer the user can decline, and run the session-close ritual first so the stopping point is genuinely clean.

## Pointers

- `logbook/` + `logbook/README.md` — the engineering record, entry format, and the session-close checklist.
- `DOCUMENTATION_GUIDE.md` — how the doc layers relate (logbook, ADRs, inline) and the cross-repo rule.
- `~/Desktop/Jugglebot/CLAUDE.md` — the control-stack repo's guidance (testing, control-system review, intuition pushback).
- Recalled memory (`MEMORY.md` index) — operational specifics: serial ports, flashing recipe, DDS zombies, the test interpreter.
