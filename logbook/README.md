# Engineering Logbook

Structured, searchable record of investigations, bugfixes, refactors, features, and optimisations in the BallButler firmware and rover code. Every non-trivial change should get a logbook entry that captures *why* the change was made, not just *what* changed — symptoms observed, hypotheses considered and rejected, tradeoffs accepted, side effects noticed.

Git history tells you *what* changed; commit messages tell you *what was intended*. Neither captures the full reasoning arc. This logbook fills that gap so future collaborators (human or AI) can search by subsystem, symptom, or file and reconstruct the context.

This logbook is modelled on the Jugglebot project's logbook conventions ([Jugglebot/logbook/README.md](https://github.com/Project-DeepBlue-Juggling/Jugglebot/blob/main/logbook/README.md)). Hardware-investigation workflows in the Jugglebot repo (`/investigate`, `/log`, `/logbook`) target Jugglebot files; BallButler entries are currently created manually using the same structure.

## Layout

```
logbook/
├── README.md     # this file
├── INDEX.md      # chronological table — newest first
└── YYYY-MM-DD-<kebab-slug>.md   # one file per entry
```

## Entry conventions

- **Filename**: `YYYY-MM-DD-<kebab-slug>.md`, ISO date matching the day the work landed.
- **Frontmatter**: YAML with `title`, `type` (bugfix / feature / refactor / optimisation / investigation), `date`, `status` (open / in-progress / resolved / tuned / superseded), `files_changed`, `commits`, `subsystem`, `tags`.
- **Sections** (use what's relevant — not every entry needs all of them):
  - **Summary** — one paragraph: what changed and why.
  - **Symptom** *(bug-fix only)* — observed behaviour, reproduction.
  - **Diagnosis** *(bug-fix only)* — root cause and how it was identified.
  - **Fix** — what was changed, file by file.
  - **Discussion** — *why this approach over alternatives, what was ruled out, tradeoffs accepted*. The most valuable section for future readers; non-negotiable when a hypothesis was withdrawn mid-investigation or when the chosen approach beat another reasonable one for non-obvious reasons.
  - **Verification** — what was tested, on what hardware, with what result. Cite test commands and dates.
  - **Related** — links to other entries, plans, external issues.

## Index

[INDEX.md](INDEX.md) lists every entry chronologically (newest first), with status and one-line title. Append your new entry to the top.
