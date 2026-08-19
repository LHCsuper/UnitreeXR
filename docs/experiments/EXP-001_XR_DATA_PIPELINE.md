# EXP-001 — XR Data Pipeline Validation

## Objective

Validate whether Python can continuously receive real-time data from XRoboToolkit.

## Question

Can the current runtime path deliver stable XR data through the following pipeline?

```text
PICO
→ XRoboToolkit
→ PC Service
→ xrobotoolkit_sdk
→ Python
```

## Hypothesis

Before execution, the XR data pipeline status was unknown.

## Setup

- VR device: PICO 4 Ultra
- PC OS: Ubuntu 22.04 x86_64
- Python module: `xrobotoolkit_sdk`
- Runtime library: `/usr/local/lib/libPXREARobotSDK.so`
- Diagnostic script: `scripts/diagnostics/xr_stream_probe.py`

## Procedure

1. Start the required XRoboToolkit runtime components.
2. Ensure the PICO XRoboToolkit APK is running and connected as required by the SDK.
3. Run `scripts/diagnostics/xr_stream_probe.py` from the repository root.
4. Observe whether initialization succeeds.
5. Observe whether XR timestamp is non-zero and continues changing.
6. Observe whether headset, left controller, and right controller poses can be read.
7. Observe whether all three pose arrays have length 7.
8. Move the headset or controllers and observe whether printed data changes.
9. Stop the script with Ctrl+C and confirm shutdown is clean.

## Observation Targets

- `xrt.init()` succeeds.
- XR timestamp is non-zero and keeps changing.
- Headset pose is readable.
- Left controller pose is readable.
- Right controller pose is readable.
- Each pose is an array-like value with length 7.
- Data changes when the device moves.

## Explicit Non-Goals

EXP-001 does not interpret any physical meaning of xyz or quaternion values.

EXP-001 must not perform:

- quaternion to Euler conversion
- rotation matrix conversion
- coordinate conversion
- position delta calibration
- A-button zeroing
- CSV recording
- Unitree integration

## Raw Data

Live validation on PICO 4 Ultra through the local PC Service.

## Observations

- `xrt.init()` connects successfully to the local PC Service.
- The Python SDK connects to PC Service at `127.0.0.1:60061`.
- `xrt.get_time_stamp_ns()` returns a non-zero value that keeps changing.
- Headset pose is readable and changes when the headset moves or rotates.
- Right controller pose is readable and changes when the right controller moves or rotates.
- Left controller pose is readable and changes when the left controller moves or rotates.
- Headset, left controller, and right controller poses are all arrays of length 7.
- `xrt.close()` executes normally.

## Analysis

The end-to-end XR data transport path is functional:

```text
PICO
→ XRoboToolkit
→ PC Service
→ xrobotoolkit_sdk
→ Python
```

Coordinate semantics were intentionally not analyzed in this experiment.

## Conclusion

PASS — XR data transport is verified; coordinate semantics remain unverified.

## Confidence

Confirmed for data transport only.

## Open Questions

Coordinate semantics remain unverified and are deferred to Phase 2:

- Position unit
- XYZ directions
- Handedness
- Tracking origin
- Quaternion convention
- Controller local frame
- Recenter behavior

## Related Files

- `scripts/diagnostics/xr_stream_probe.py`
- `docs/STATUS.md`
- `docs/COORDINATE_SYSTEM.md`