---
title: <descriptive title>
type: investigation        # investigation | bugfix | refactor | feature | optimization
date: YYYY-MM-DD
status: open               # open | in-progress | tuned | resolved
# --- Context (optional) ---
phase: ""                  # bring-up phase, if applicable
related_adr: ""            # docs/adr/ filename, if a decision record governs this
related_entries:           # other logbook slugs this builds on
  - YYYY-MM-DD-slug
# --- Traceability ---
files_changed:             # every file modified IN THIS REPO
  - <path/to/file>
external_changes:          # changes in another repo (usually Jugglebot)
  - "Jugglebot: <path> (<what changed>)"
commits:                   # short hashes, filled after committing
  - <abcdef0>
# --- Classification ---
subsystem:                 # throwing | calibration | tracking | firmware | sim | electronics | tooling | docs
  - <subsystem>
tags:                      # accuracy | mocap | ballistics | repeatability | safety | testing | docs
  - <tag>
---

# <title>

## Summary

<2–3 sentences: what changed and why, with the headline result.>

<!---------------------------------------------------------------------
  SECTION GUIDE — use the sections for your entry type; delete the rest.
    investigation:  Symptoms, Diagnosis, Discussion, Fix, Outcome
    bugfix:         Problem, Root Cause, Fix, Verification
    refactor:       Motivation, Changes, Migration Notes, Verification
    feature:        Motivation, Design, Implementation, Verification
    optimization:   Motivation, Approach, Benchmarks, Verification
  All types: Summary (top) + Open Questions / Follow-ups (bottom).
---------------------------------------------------------------------->

## Motivation

<Why this work was needed. Be concrete about the observed problem.>

## Design

<How it works / how it's applied. Math, pipeline order, conventions.>

## Implementation

<What was actually done. Commands run, files touched, enabling fixes.>

## Verification

<Evidence it works. Numbers, before/after, tests. Be honest about residuals.>

## Open Questions / Follow-ups

1. <What's left, what to watch, what to confirm.>
