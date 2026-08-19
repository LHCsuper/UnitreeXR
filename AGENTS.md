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

Current phase is Phase 3 — Unitree Coordinate Mapping.

Allowed in this phase:

- Inspect Unitree `xr_teleoperate` source.
- Inspect `TeleData` semantics.
- Study the XR controller pose → Unitree wrist target mapping.
- Derive explicit coordinate transforms.
- Offline mathematical validation.
- MeshCat / visualization validation.
- Add traceable Phase 3 experiments.

Still prohibited:

- Controlling a real robot without explicit instruction.
- Secretly adding coordinate conversion in the raw XR acquisition layer.
- Writing Hypotheses as Facts.
- Entering real robot integration before offline mapping validation is complete.
- Modifying raw XR pose data using an unverified left/right mirror assumption.