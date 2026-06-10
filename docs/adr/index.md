# Architecture Decision Records

This directory holds **Architecture Decision Records (ADRs)** for the Ball Butler
— short documents that capture a single significant, hard-to-reverse decision:
the context, the options weighed, the choice, and its consequences.

Modelled on the [Jugglebot ADRs](https://github.com/Project-DeepBlue-Juggling/Jugglebot/tree/main/docs/adr).
See the repo-root [DOCUMENTATION_GUIDE.md](../../DOCUMENTATION_GUIDE.md) for how
ADRs relate to the logbook (the logbook records *what happened*; an ADR records
*why a structural choice was made*).

## When to write an ADR

Write one when a decision is **structural and costly to reverse** — a transport
choice, a frame/convention, a hardware split, an interface contract. If it's just
"why does this code change look this way", that's a [logbook](../../logbook/)
entry, not an ADR. When in doubt, log it; promote to an ADR only if it becomes a
load-bearing constraint others must design around.

## Format

- **Filename:** `NNNN-<kebab-case-title>.md`, zero-padded sequential (`0001-…`).
- **Frontmatter / heading** records `status` (`proposed | accepted | superseded`),
  date, and (if superseded) the superseding ADR.
- **Body:** Context → Decision → Consequences (and Alternatives considered).
- Add a row to the table below.

## Index

| ADR | Status | Title |
|-----|--------|-------|
| _none yet_ | | The first structural decision to arise will be recorded here. |
