# Documentation Guide

The single reference for **how documentation is organised in the Ball Butler
repo** — where each kind of information lives and how the layers cross-reference
each other. Consult it before creating or editing any markdown artifact.

Modelled on the [Jugglebot documentation system](https://github.com/Project-DeepBlue-Juggling/Jugglebot),
trimmed to what this (younger) repo actually uses. Layers Jugglebot has but
BallButler does not yet — a published MkDocs site, `plans/`, and a
`.claude/` automation layer — can be adopted from that repo when the need
arises; they are intentionally absent for now.

---

## 1. The cross-repo rule (read this first)

The Ball Butler V0 thrower's **ROS2 control stack lives in the
[Jugglebot](https://github.com/Project-DeepBlue-Juggling/Jugglebot) repo**, not
here. That includes `ball_butler_node`, `mocap_node`, `teensy_bridge_node`, and
the deployed aim-correction matrix under `ros_ws/.../resources/`.

**This repo (BallButler)** holds: firmware (`ball_butler_main/`), simulation
(`sim/`), test & analysis tooling (`zTesting/`), hardware design
(`ball_butler_circuit_diagram/`, `hand_sensor_pcb_rail/`), and the **engineering
record** (this guide, `logbook/`, `docs/adr/`).

Consequence: a single piece of work often spans both repos. The rule:

- **The narrative record lives in BallButler's `logbook/`** (this is where we
  document Ball Butler work), and it **cross-references Jugglebot by path**.
- Files changed *in this repo* go in a logbook entry's `files_changed:`.
- Files changed *in Jugglebot* go in `external_changes:` as
  `"Jugglebot: <path> (<what>)"`.

---

## 2. Documentation layers

| # | Layer | Path | What belongs here |
|---|-------|------|-------------------|
| 1 | **Public README** | [README.md](README.md) | Project identity: what Ball Butler is, V0 state, V1+ plan. Public-facing. |
| 2 | **This guide** | [DOCUMENTATION_GUIDE.md](DOCUMENTATION_GUIDE.md) | How docs are organised (you are here). |
| 3 | **Engineering logbook** | [logbook/](logbook/) | Per-change record: *why* a change looks the way it does, and whether it worked. |
| 4 | **Architecture Decision Records** | [docs/adr/](docs/adr/) | Single significant, hard-to-reverse decisions. |
| 5 | **Inline source docs** | `*.cpp`, `*.py`, `*.h`, … | Docstrings / `why` comments next to the code. |

Duplication between layers is a bug. Keep authoritative content in one place and
cross-reference from the others.

> Claude's private memory (`~/.claude/.../memory/`) is **not** a documentation
> layer. Don't copy project docs there.

### 2.1 [README.md](README.md) — public landing

Project identity and the V0 → V1+ story. Keep it accurate to the hardware; it is
the first thing an outside reader sees. Not the place for investigation
write-ups.

### 2.2 [logbook/](logbook/) — engineering logbook

The authoritative record of every substantive change: symptoms/motivation,
diagnosis/design, the change, and the **measured** outcome. Full spec in
[logbook/README.md](logbook/README.md); copy [logbook/TEMPLATE.md](logbook/TEMPLATE.md)
to start one; [logbook/INDEX.md](logbook/INDEX.md) lists all entries.

Essentials:
- Filename `YYYY-MM-DD-<slug>.md` (3–6-word slug).
- Mandatory YAML frontmatter: `title`, `type`, `date`, `status`.
- `type` ∈ `investigation | bugfix | refactor | feature | optimization`; each
  type has its own body sections (see the logbook README).
- Maintained by hand — update [INDEX.md](logbook/INDEX.md) when you add or
  re-status an entry.

### 2.3 [docs/adr/](docs/adr/) — Architecture Decision Records

One file per significant, costly-to-reverse decision (a frame convention, a
hardware split, an interface contract). The logbook records *what happened*; an
ADR records *why a structural choice was made* and what was rejected. Format and
the running index live in [docs/adr/index.md](docs/adr/index.md). If a decision
is really just "why this code change looks this way", it's a logbook entry, not
an ADR.

### 2.4 Inline source documentation

- **File/module header** — one line when the filename doesn't make the role
  obvious.
- **Comments document *why*, not *what*** — a hidden constraint, a sign/unit
  convention, a workaround for a specific bug. If deleting the comment wouldn't
  confuse a future reader, don't write it.
- **No task-journal comments** ("added for the X run") — that belongs in the
  commit message and the logbook.
- **Control/geometry conventions inline are load-bearing.** Sign conventions,
  frame definitions, and unit conventions in comments are normative — update them
  in the same change as the code.

---

## 3. Decision tree — "I need to document X"

- **The *why* behind a concrete change, and whether it worked?**
  → Logbook entry. Cross-reference Jugglebot paths under `external_changes:` if
  the control stack was touched.
- **A significant, hard-to-reverse structural decision?**
  → ADR under [docs/adr/](docs/adr/).
- **The *why* behind a single line/block of code?**
  → Inline comment, only if non-obvious. Otherwise the logbook is the better home.
- **Public project description?**
  → [README.md](README.md). Keep it minimal and accurate.
- **None of the above fit.** → Ask before inventing a new layer.

---

## 4. Maintenance

Keep this guide current when a layer is added or its conventions change. If this
repo later adopts Jugglebot's `plans/`, MkDocs site, or `.claude/` automation,
add the corresponding layer here and in the table above.
