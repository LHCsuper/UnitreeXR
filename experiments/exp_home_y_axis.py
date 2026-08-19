import time
import math
import numpy as np
import xrobotoolkit_sdk as xrt


SAMPLE_TIME = 1.0
SAMPLE_DT = 0.01


# ============================================================
# Quaternion
# q = [qx, qy, qz, qw]
# ============================================================

def quat_to_R(q):
    q = np.asarray(q, dtype=float)

    n = np.linalg.norm(q)
    if n < 1e-8:
        raise ValueError("Invalid quaternion")

    q = q / n

    x, y, z, w = q

    return np.array([
        [
            1 - 2*(y*y + z*z),
            2*(x*y - z*w),
            2*(x*z + y*w)
        ],
        [
            2*(x*y + z*w),
            1 - 2*(x*x + z*z),
            2*(y*z - x*w)
        ],
        [
            2*(x*z - y*w),
            2*(y*z + x*w),
            1 - 2*(x*x + y*y)
        ]
    ])


def pose_to_T(pose):
    pose = np.asarray(pose, dtype=float)

    T = np.eye(4)

    T[:3, :3] = quat_to_R(
        pose[3:7]
    )

    T[:3, 3] = pose[:3]

    return T


# ============================================================
# Quaternion averaging
# ============================================================

def average_pose(samples):

    positions = np.array([
        p[:3] for p in samples
    ])

    quats = np.array([
        p[3:7] for p in samples
    ])

    # quaternion sign alignment
    q_ref = quats[0].copy()

    for i in range(len(quats)):
        if np.dot(quats[i], q_ref) < 0:
            quats[i] *= -1

    p_mean = positions.mean(axis=0)

    q_mean = quats.mean(axis=0)
    q_mean /= np.linalg.norm(q_mean)

    return np.concatenate([
        p_mean,
        q_mean
    ])


def capture_head_pose():

    samples = []

    start = time.monotonic()

    while time.monotonic() - start < SAMPLE_TIME:

        pose = np.array(
            xrt.get_headset_pose(),
            dtype=float
        )

        q_norm = np.linalg.norm(
            pose[3:7]
        )

        if q_norm > 1e-6:
            samples.append(pose)

        time.sleep(SAMPLE_DT)

    if not samples:
        raise RuntimeError(
            "No valid headset pose"
        )

    return average_pose(samples)


# ============================================================
# Analysis
# ============================================================

def analyze(before, after):

    T_D0_H = pose_to_T(before)
    T_D1_H = pose_to_T(after)

    # --------------------------------------------------------
    # Since physical HMD is assumed unchanged:
    #
    # ^D0 T_H =
    # ^D0 T_D1 @ ^D1 T_H
    #
    # Therefore:
    #
    # ^D0 T_D1 =
    # ^D0 T_H @ inverse(^D1 T_H)
    # --------------------------------------------------------

    T_D0_D1 = (
        T_D0_H
        @ np.linalg.inv(T_D1_H)
    )

    R_D0_D1 = T_D0_D1[:3, :3]
    p_D0_D1 = T_D0_D1[:3, 3]

    # New Tracking Origin +Y axis
    # expressed in old Tracking Origin
    new_y_in_old = (
        R_D0_D1
        @ np.array([0.0, 1.0, 0.0])
    )

    old_y = np.array([
        0.0,
        1.0,
        0.0
    ])

    dot = np.clip(
        np.dot(
            old_y,
            new_y_in_old
        ),
        -1.0,
        1.0
    )

    y_tilt_deg = math.degrees(
        math.acos(dot)
    )

    print()
    print("=" * 70)

    print("BEFORE:")
    print(
        "pos =",
        np.round(before[:3], 6)
    )
    print(
        "quat =",
        np.round(before[3:7], 6)
    )

    print()

    print("AFTER:")
    print(
        "pos =",
        np.round(after[:3], 6)
    )
    print(
        "quat =",
        np.round(after[3:7], 6)
    )

    print()

    print("^D0 T_D1 =")
    print(
        np.round(
            T_D0_D1,
            6
        )
    )

    print()

    print(
        "New +Y axis expressed in old frame:"
    )

    print(
        np.round(
            new_y_in_old,
            6
        )
    )

    print()

    print(
        f"Y-axis tilt angle = "
        f"{y_tilt_deg:.4f} deg"
    )

    print()

    print(
        "^D0 p_D1 =",
        np.round(
            p_D0_D1,
            6
        )
    )

    print("=" * 70)

    return y_tilt_deg


# ============================================================
# One experiment
# ============================================================

def run_trial(name):

    print()
    print("#" * 80)
    print(name)
    print("#" * 80)

    print()
    print(
        "摆好头部姿态，并保持不动。"
    )

    input(
        "准备好后按 Enter，记录 Home 前数据..."
    )

    before = capture_head_pose()

    print()
    print("Home 前采集完成。")
    print()
    print(
        "现在保持头的位置和姿态尽量完全不动，"
    )
    print(
        "长按 PICO Home 键执行 recenter。"
    )
    print()
    print(
        "recenter 完成后仍保持头不动。"
    )

    input(
        "完成后按 Enter，记录 Home 后数据..."
    )

    time.sleep(0.5)

    after = capture_head_pose()

    return analyze(
        before,
        after
    )


# ============================================================
# Main
# ============================================================

def main():

    print()
    print("=" * 80)
    print(
        "EXP: PICO Home recenter - Tracking Origin Y-axis validation"
    )
    print("=" * 80)

    print()
    print(
        "目的：验证 Home 前后的 Tracking Origin +Y 是否保持平行。"
    )

    print()
    print(
        "重要：每次 Home 前后，头显必须尽量保持同一个现实姿态。"
    )

    print()

    xrt.init()

    time.sleep(2.0)

    try:

        results = []

        results.append(
            run_trial(
                "TRIAL 1: 头保持正常水平朝前"
            )
        )

        results.append(
            run_trial(
                "TRIAL 2: 明显低头约 30~45 度"
            )
        )

        results.append(
            run_trial(
                "TRIAL 3: 明显向左肩方向歪头约 30~45 度"
            )
        )

        print()
        print("=" * 80)
        print("SUMMARY")
        print("=" * 80)

        for i, angle in enumerate(
            results,
            start=1
        ):
            print(
                f"Trial {i}: "
                f"Y-axis tilt = "
                f"{angle:.4f} deg"
            )

        print()

        print(
            "如果三次结果都接近 0 deg，"
        )
        print(
            "说明 Home 重建坐标系时不会跟随 HMD 的上下/左右倾斜"
        )
        print(
            "而改变 Tracking Origin 的竖直轴。"
        )

    finally:

        xrt.close()


if __name__ == "__main__":
    main()
