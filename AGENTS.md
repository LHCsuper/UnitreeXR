# AGENTS.md

This file defines project-level rules for Cursor / AI agents working in this repository.

## Engineering Principles

1. Do not control a real Unitree robot without explicit instruction.
2. The first research target is `PICO 4 Ultra → XRoboToolkit → Python raw pose`.
3. Coordinate definitions must not be guessed from experience.
4. Coordinate-related information must be classified as one of:
   - Confirmed Fact
   - Source Evidence
   - Experimental Evidence
   - Hypothesis
   - Unknown
5. Do not write hypotheses as facts before experimental confirmation.
6. Do not secretly add coordinate conversion in the XR raw input layer.
7. All coordinate conversions must be explicit, named, and testable.
8. Raw experimental data must never be overwritten.
9. Experiment files must use traceable IDs, for example `EXP-001`, `EXP-002`.
10. Update `docs/WORKLOG.md` after each meaningful engineering operation.
11. Record each important architectural choice in `docs/DECISIONS.md`.
12. `docs/STATUS.md` must reflect the current real state, not future plans.
13. Before modifying code, read first:
    - `AGENTS.md`
    - `docs/STATUS.md`
    - `docs/PROJECT_PLAN.md`
    - related experiment records
14. Keep Python simple, explicit, and testable.
15. Do not implement unverified functionality just to make the project look complete.

## Current Scope

Current phase is Phase 1 — XR Data Pipeline Validation.

EXP-001 may validate whether XR data can flow through `PICO 4 Ultra → XRoboToolkit → PC Service → xrobotoolkit_sdk → Python`.

Do not start PICO pose coordinate calibration, Unitree integration, external repository cloning, package installation, or real robot control in this phase.