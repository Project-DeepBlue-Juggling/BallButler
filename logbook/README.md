# Engineering Logbook

The authoritative record of **why the Ball Butler looks the way it does**:
per-change entries capturing symptoms, diagnosis, alternatives, the change, and
the measured outcome. When a future engineer (human or Claude) asks "why is this
matrix here / why is this threshold set this way / did that actually help?", the
answer lives here.

Modelled on the [Jugglebot logbook](https://github.com/Project-DeepBlue-Juggling/Jugglebot).
See the repo-root [DOCUMENTATION_GUIDE.md](../DOCUMENTATION_GUIDE.md) for how this
layer relates to the README, the ADRs, and inline code comments — and for the
**cross-repo rule** (the V0 thrower's ROS2 control stack lives in the Jugglebot
repo; entries here cross-reference it).

## Quick reference

| Action | How |
|--------|-----|
| Add an entry | Copy [TEMPLATE.md](TEMPLATE.md) → `YYYY-MM-DD-<slug>.md`, fill it in, add a row to [INDEX.md](INDEX.md) |
| Browse | Read [INDEX.md](INDEX.md) (sortable table of all entries) |

There is no automation layer in this repo yet, so entries and the index are
maintained **by hand**. Keep [INDEX.md](INDEX.md) in sync when you add or
re-status an entry.

## Session close

Treat the end of a working session as one unit — don't let changes or red tests
pile up across sessions. Before you stop:

1. **Close the logbook entry** — set `status:` (→ `tuned`/`resolved`), fill the
   Outcome/Verification, note anything still open, and update [INDEX.md](INDEX.md).
2. **Run the relevant tests** — control-stack work: the Jugglebot suite
   (`~/Desktop/Jugglebot/run_tests.sh`); firmware/tooling: the change's own
   verification. Don't commit known-failing code without saying so.
3. **Commit the logical unit** — `Co-Authored-By` trailer; cross-repo work gets a
   commit in each repo.

Claude should proactively *offer* to close out at natural boundaries (unit done,
committed, and logged; context getting long; or a topic switch) rather than
waiting to be asked.

## Entry format

- **Filename:** `YYYY-MM-DD-<slug>.md` — slug is 3–6 hyphen-separated words.
- **Frontmatter** (YAML between `---`) is mandatory. Required: `title`, `type`,
  `date`, `status`. Optional: `phase`, `files_changed`, `external_changes`,
  `commits`, `subsystem`, `tags`, `related_adr`, `related_entries`.

### Frontmatter fields

| Field | Required | Notes |
|-------|----------|-------|
| `title` | ✓ | Human-readable, matches the `# H1`. |
| `type` | ✓ | Controlled — see below. |
| `date` | ✓ | `YYYY-MM-DD`. |
| `status` | ✓ | Controlled ladder — see below. |
| `phase` | | Free text; the bring-up phase this belongs to. |
| `files_changed` | | Every file modified **in this repo** (enables reverse lookups). |
| `external_changes` | | Changes in another repo (almost always Jugglebot) — `"<repo>: <path> (<what>)"`. |
| `commits` | | Short hashes, filled after committing. |
| `subsystem` | | Controlled — see below. |
| `tags` | | Controlled — see below. |
| `related_adr` | | ADR filename in `docs/adr/`, if a decision record governs this work. |
| `related_entries` | | Other logbook slugs this entry builds on. |

### `type` (controlled)

`investigation | bugfix | refactor | feature | optimization`

Each type uses the body sections relevant to it (delete the rest):

| type | sections |
|------|----------|
| investigation | Symptoms, Diagnosis, Discussion, Fix, Outcome |
| bugfix | Problem, Root Cause, Fix, Verification |
| refactor | Motivation, Changes, Migration Notes, Verification |
| feature | Motivation, Design, Implementation, Verification |
| optimization | Motivation, Approach, Benchmarks, Verification |

All types share **Summary** (top) and **Open Questions / Follow-ups** (bottom).

### `status` (ladder)

`open → in-progress → tuned | resolved`

- **open** — nothing done yet.
- **in-progress** — diagnosis done, fix/verification ongoing.
- **tuned** — the symptom in scope is fixed and verified, but a sibling
  investigation is intentionally left open elsewhere.
- **resolved** — every symptom in scope is fixed and verified; no open
  follow-ups inside this entry's scope.

### `subsystem` (controlled)

`throwing | calibration | tracking | firmware | sim | electronics | tooling | docs`

- **throwing** — yaw/pitch/linear thrower mechanism + ballistics aiming.
- **calibration** — thrower localisation and accuracy/aim calibration.
- **tracking** — motion capture (QTM) ingest and rigid-body/ball tracking.
- **firmware** — `ball_butler_main/` (Teensy) and other embedded code.
- **sim** — `sim/` MuJoCo models and controllers.
- **electronics** — PCBs, wiring, circuit design.
- **tooling** — analysis/test scripts under `zTesting/` and helpers.
- **docs** — documentation-only changes.

### `tags` (controlled)

`accuracy | mocap | ballistics | repeatability | safety | testing | docs`

## Withdrawn claims

If an `investigation` entry's earlier conclusion turns out wrong, **do not delete
it**. Add a "Withdrawn claims" section near the bottom with the date, the
retracted claim, the evidence, and a pointer to the correct finding. Silent edits
destroy the investigation history.

## Cross-repo note

The V0 thrower's ROS2 control nodes (`ball_butler_node`, `mocap_node`,
`teensy_bridge_node`) and the deployed correction live in the **Jugglebot** repo.
BallButler holds firmware, sim, test tooling, hardware design, and this
engineering record. When an entry's work touches the control stack, record those
files under `external_changes:` and reference them by their Jugglebot path.
