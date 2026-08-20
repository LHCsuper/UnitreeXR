## DEC-001 — Separate raw XR acquisition from coordinate conversion

Status: Accepted

### Context

Raw XR pose data will need to be inspected, archived, and calibrated before any downstream robot-specific interpretation is allowed.

### Decision

Keep raw acquisition and coordinate conversion in separate modules and separate workflow stages.

### Reason

This prevents hidden coordinate transforms from contaminating source data and keeps experiments testable.

### Consequence

All conversion logic must be explicit, named, and verifiable before it can be used downstream.

## DEC-002 — Keep raw experiment data out of Git

Status: Accepted

### Context

Raw experiment outputs can be large, ephemeral, or environment-specific.

### Decision

Store raw experiment data under `data/raw` and exclude it from Git, keeping only `.gitkeep` placeholders.

### Reason

This preserves repository cleanliness while keeping the directory structure stable.

### Consequence

Raw data must be archived outside the tracked history or in a dedicated storage workflow.

## DEC-003 — Delay real Unitree control until offline validation is complete

Status: Accepted

### Context

The XR pose pipeline and coordinate conventions must be validated before any robot actuation is attempted.

### Decision

Do not introduce real Unitree control until offline coordinate validation and adapter checks are complete.

### Reason

This reduces safety risk and avoids building on unverified assumptions.

### Consequence

Robot integration remains out of scope for Phase 0 through Phase 4.

## DEC-004 — Target fixed logical operational EE frames for baseline arm IK

Status: Accepted

### Context

`wheelloong_m2` has wrist-roll frames and articulated gripper links, but no
explicit palm, gripper-base, tool-center, or calibrated fingertip TCP frame.
EXP-004 derived reproducible robot-side frames from the checked-in geometry.

### Decision

Baseline arm IK will target logical operational EE frames `W_L` and `W_R`.
Their origins are the means of the four direct gripper root joint origins.
Their shared axis semantics are:

```text
+Y_W: physical finger-extension direction from wrist/palm toward the fingers
+Z_W: positive direct gripper hinge-axis direction
+X_W: y_W cross z_W
```

These are fixed palm/gripper-root operational frames, not calibrated
fingertip TCPs. This is deliberately consistent at the conceptual level with
Unitree-style IK that constructs fixed operational EE frames from wrist/arm
joints; it does not assert implementation identity with a specific Unitree
solver.

### Reason

The fixed frames provide an explicit, symmetric, testable robot-side target
contract without claiming unavailable fingertip calibration. Their rotations
are derived from transformed finger-extension vectors and direct hinge axes,
not from link-name guesses or independently chosen axis signs.

### Consequence

Later offline arm IK work must target `W_L` / `W_R` and preserve the explicit
fixed transforms recorded in EXP-004 unless a new decision supersedes them.
This decision does not implement IK, define a final physical TCP, connect XR,
or authorize robot control.
