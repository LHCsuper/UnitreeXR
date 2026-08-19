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