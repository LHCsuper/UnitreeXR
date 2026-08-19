import time
import math
import xrobotoolkit_sdk as xrt

RUN_TIME = 10.0      # 自动运行 10 秒
SAMPLE_DT = 0.2      # 每 0.2 秒打印一次


def fmt_pose(pose):
    pose = [float(v) for v in pose]

    p = pose[:3]
    q = pose[3:7]

    q_norm = math.sqrt(sum(v * v for v in q))
    valid = q_norm > 1e-6

    return (
        f"pos=[{p[0]:+.4f}, {p[1]:+.4f}, {p[2]:+.4f}]  "
        f"quat=[{q[0]:+.4f}, {q[1]:+.4f}, {q[2]:+.4f}, {q[3]:+.4f}]  "
        f"q_norm={q_norm:.4f}  "
        f"{'VALID' if valid else 'INVALID'}"
    )


xrt.init()
time.sleep(2.0)

print()
print("Pose format: [x, y, z, qx, qy, qz, qw]")
print("Reference frame: current Device Tracking Origin")
print(f"Automatically running for {RUN_TIME:.1f} seconds")
print()

start_time = time.monotonic()

try:
    while time.monotonic() - start_time < RUN_TIME:
        head = xrt.get_headset_pose()
        left = xrt.get_left_controller_pose()
        right = xrt.get_right_controller_pose()

        try:
            timestamp = xrt.get_time_stamp_ns()
        except Exception:
            timestamp = None

        elapsed = time.monotonic() - start_time

        print("=" * 100)
        print(f"elapsed: {elapsed:.2f} s")

        if timestamp is not None:
            print(f"timestamp_ns: {timestamp}")

        print("HEAD  :", fmt_pose(head))
        print("LEFT  :", fmt_pose(left))
        print("RIGHT :", fmt_pose(right))

        time.sleep(SAMPLE_DT)

finally:
    xrt.close()

print("\n10 seconds completed.")