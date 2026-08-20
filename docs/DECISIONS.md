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

## DEC-005 — Expose a named 14-DOF arm interface before any IK work

Status: Accepted

### Context

The loaded Pinocchio model has `nq=42` and `nv=32`, including non-arm and
non-scalar gripper joints. Exposing those backend indices to later consumers
would make the arm target contract ambiguous and brittle.

### Decision

Use one public `q_arm` ordering of the seven named left-arm joints followed by
the seven named right-arm joints. Resolve every name to its Pinocchio
configuration and velocity index at model load time. The kinematics backbone
returns `^torso T_WL` / `^torso T_WR` and torso-expressed `6x14` Jacobians for
the unchanged S0.5b logical operational frames.

### Reason

This makes the robot-side motion interface explicit and testable without
leaking Pinocchio's full configuration layout or silently changing frame
conventions.

### Consequence

No later FK/Jacobian/IK consumer may use handwritten Pinocchio joint indices
for these arms. The module is kinematics-only: this decision adds no IK,
optimizer, XR path, controller, model change, or robot-control authority.

## DEC-006 — Keep symbolic FK independent from numeric FK

Status: Accepted

### Context

S1.0 provides numeric Pinocchio FK, while later offline work may require
symbolic expressions. Reusing numeric pose results as symbolic outputs would
not verify symbolic model construction or the full configuration mapping.

### Decision

Build a separate `WheelloongM2CasadiKinematics` implementation using
`pinocchio.casadi.Model` converted from the numeric model loaded from the
same checked-in URDF. It uses the existing named 14-DOF mapping and unchanged
S0.5b `W_L` / `W_R` fixed transforms, then returns only torso-relative
symbolic FK expressions and a CasADi evaluation function.

### Reason

Independent numeric and symbolic FK paths make their agreement observable
without duplicating, modifying, or introducing a second model file.

### Consequence

S1.1 remains FK-only. It creates no symbolic IK, cost, NLP, IPOPT, Opti, or
solver component, and does not modify the current numeric FK interface.

## DEC-007 — Use torso-frame current-minus-target SE(3) errors before solver work

Status: Accepted

### Context

S1.0 and S1.1 expose only the unchanged logical operational frames `W_L` and
`W_R` as torso-relative poses. A later offline IK stage needs one explicit,
Euler-angle-free error convention before any optimizer is considered.

### Decision

For each arm, define pose error in torso axes as:

```text
e_p = p_current - p_target
e_R = Log(R_current * R_target^T)
```

Use Pinocchio `log3` for numeric evaluation and a matching CasADi
small-angle-safe SO(3)-log expression for symbolic evaluation. The
solver-free dual-arm cost is the explicit sum of weighted translation and
rotation error squares, nominal-configuration regularization, and
previous-configuration smoothness terms.

### Reason

This fixes the sign, orientation direction, coordinate frame, and cost-term
meaning in a testable math layer while preserving the existing S0.5b `W_L` /
`W_R` operational EE contract.

### Consequence

This decision adds no IK iteration, NLP, Opti, IPOPT, solver, controller, XR
path, model edit, or robot-control authority. Any later solver design must
consume this convention unless a new decision explicitly supersedes it.

## DEC-008 — Use a reusable offline Opti/IPOPT baseline with soft dual-arm pose cost

Status: Accepted

### Context

S1.2.0 established named 14-DOF pose-error and cost mathematics but did not
select configurations. The first offline IK implementation must preserve the
existing `W_L` / `W_R` target contract and named URDF joint limits.

### Decision

Create one parameterized CasADi `Opti` problem with the existing
`q_arm(14)` order as its decision variable. Reuse S1.1 symbolic FK and S1.2.0
symbolic SE(3) error to minimize weighted dual-arm pose, neutral
regularization, and `q_prev` smoothness costs. Constrain only named arm
position limits read from the URDF, configure IPOPT with quiet default
printing, and accept an optional `q_init` warm-start seed on every solve.

### Reason

This creates a testable offline solver with explicit target and constraint
semantics while avoiding unscoped dynamic, collision, control, or XR work.

### Consequence

The baseline is a soft-cost IK solver: a solution can retain nonzero pose
residual when regularization and smoothness select a lower-cost configuration.
It remains offline only and adds no trajectory, velocity/acceleration limit,
collision avoidance, torque, XR path, controller, model edit, or robot-control
authority.
