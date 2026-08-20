#!/usr/bin/env python3
"""Inspect a MuJoCo model by driving scalar joints with sliders."""

from __future__ import annotations

import argparse
import math
import os
import tkinter as tk
from dataclasses import dataclass
from pathlib import Path
from tkinter import ttk
from typing import Callable

try:
    from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
except ImportError:  # pragma: no cover - optional when running outside ROS setup
    PackageNotFoundError = Exception
    get_package_share_directory = None

import mujoco
import mujoco.viewer
import numpy as np


DEFAULT_RATE_HZ = 30.0
ARM_NAME_TOKEN = "_arm_joint_"
LEFT_ARM_PREFIX = "left_arm_joint_"
RIGHT_ARM_PREFIX = "right_arm_joint_"
GRIPPER_NAME_TOKEN = "gripper_joint"
TORSO_JOINT_NAMES = {"lift_joint", "torso_pitch_joint", "neck_joint_1", "neck_joint_2"}
CHASSIS_JOINT_NAMES = {"x_slide_joint", "y_slide_joint"}


@dataclass(frozen=True)
class JointControl:
    joint_id: int
    name: str
    joint_type: int
    qpos_index: int
    qvel_index: int
    lower: float
    upper: float
    unit: str


def default_model_path() -> str:
    env_path = os.environ.get("WHEELLOONG_M2_MUJOCO_MODEL")
    if env_path:
        return env_path

    candidates: list[Path] = []
    if get_package_share_directory is not None:
        try:
            candidates.append(
                Path(get_package_share_directory("wheelloong_m2"))
                / "mujoco"
                / "wheelloong_m2_controlled.xml"
            )
        except PackageNotFoundError:
            pass

    candidates.append(
        Path(__file__).resolve().parents[1] / "mujoco" / "wheelloong_m2_controlled.xml"
    )

    for candidate in candidates:
        if candidate.exists():
            return str(candidate)
    return str(candidates[-1])


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Load a MuJoCo XML and inspect joints with sliders.",
    )
    parser.add_argument(
        "--model-path",
        default=default_model_path(),
        help="Path to a MuJoCo XML model. Defaults to wheelloong_m2_controlled.xml.",
    )
    parser.add_argument(
        "--filter",
        choices=("all", "arms", "left-arm", "right-arm", "grippers", "torso", "chassis"),
        default="all",
        help="Joint group shown in the slider panel.",
    )
    parser.add_argument(
        "--rate-hz",
        type=float,
        default=DEFAULT_RATE_HZ,
        help="Viewer sync rate while the slider panel is open.",
    )
    parser.add_argument(
        "--width",
        type=int,
        default=760,
        help="Slider window width in pixels.",
    )
    parser.add_argument(
        "--height",
        type=int,
        default=900,
        help="Slider window height in pixels.",
    )
    return parser.parse_args()


def scalar_joint_type(model: mujoco.MjModel, joint_id: int) -> bool:
    joint_type = int(model.jnt_type[joint_id])
    return joint_type in (
        int(mujoco.mjtJoint.mjJNT_HINGE),
        int(mujoco.mjtJoint.mjJNT_SLIDE),
    )


def joint_name(model: mujoco.MjModel, joint_id: int) -> str:
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
    return name if name is not None else f"joint_{joint_id}"


def default_joint_range(joint_type: int) -> tuple[float, float]:
    if joint_type == int(mujoco.mjtJoint.mjJNT_HINGE):
        return -math.pi, math.pi
    return -1.0, 1.0


def joint_unit(joint_type: int) -> str:
    if joint_type == int(mujoco.mjtJoint.mjJNT_HINGE):
        return "rad"
    return "m"


def discover_scalar_joints(model: mujoco.MjModel) -> list[JointControl]:
    controls: list[JointControl] = []
    for joint_id in range(model.njnt):
        if not scalar_joint_type(model, joint_id):
            continue

        name = joint_name(model, joint_id)
        joint_type = int(model.jnt_type[joint_id])
        qpos_index = int(model.jnt_qposadr[joint_id])
        qvel_index = int(model.jnt_dofadr[joint_id])
        if bool(model.jnt_limited[joint_id]):
            lower, upper = [float(value) for value in model.jnt_range[joint_id]]
        else:
            lower, upper = default_joint_range(joint_type)

        controls.append(
            JointControl(
                joint_id=joint_id,
                name=name,
                joint_type=joint_type,
                qpos_index=qpos_index,
                qvel_index=qvel_index,
                lower=lower,
                upper=upper,
                unit=joint_unit(joint_type),
            )
        )
    return controls


def filter_controls(controls: list[JointControl], filter_name: str) -> list[JointControl]:
    predicates: dict[str, Callable[[JointControl], bool]] = {
        "all": lambda joint: True,
        "arms": lambda joint: ARM_NAME_TOKEN in joint.name,
        "left-arm": lambda joint: joint.name.startswith(LEFT_ARM_PREFIX),
        "right-arm": lambda joint: joint.name.startswith(RIGHT_ARM_PREFIX),
        "grippers": lambda joint: GRIPPER_NAME_TOKEN in joint.name,
        "torso": lambda joint: joint.name in TORSO_JOINT_NAMES,
        "chassis": lambda joint: joint.name in CHASSIS_JOINT_NAMES,
    }
    predicate = predicates[filter_name]
    return [joint for joint in controls if predicate(joint)]


class ScrollableFrame(ttk.Frame):
    def __init__(self, master: tk.Widget) -> None:
        super().__init__(master)
        self.canvas = tk.Canvas(self, highlightthickness=0)
        self.scrollbar = ttk.Scrollbar(self, orient="vertical", command=self.canvas.yview)
        self.content = ttk.Frame(self.canvas)
        self.content.bind(
            "<Configure>",
            lambda _event: self.canvas.configure(scrollregion=self.canvas.bbox("all")),
        )
        self.window_id = self.canvas.create_window((0, 0), window=self.content, anchor="nw")
        self.canvas.configure(yscrollcommand=self.scrollbar.set)
        self.canvas.bind("<Configure>", self._on_canvas_configure)
        self.canvas.pack(side="left", fill="both", expand=True)
        self.scrollbar.pack(side="right", fill="y")

    def _on_canvas_configure(self, event: tk.Event) -> None:
        self.canvas.itemconfigure(self.window_id, width=event.width)


class JointSliderApp:
    def __init__(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        viewer_handle: mujoco.viewer.Handle,
        controls: list[JointControl],
        args: argparse.Namespace,
    ) -> None:
        self.model = model
        self.data = data
        self.viewer = viewer_handle
        self.controls = controls
        self.args = args
        self.initial_qpos = np.array(data.qpos, copy=True)
        self.scales: dict[str, tk.Scale] = {}
        self.value_labels: dict[str, ttk.Label] = {}
        self.root = tk.Tk()
        self.root.title(f"MuJoCo joint sliders - {Path(args.model_path).name}")
        self.root.geometry(f"{args.width}x{args.height}")
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self._build_ui()

    def _build_ui(self) -> None:
        header = ttk.Frame(self.root, padding=8)
        header.pack(fill="x")
        ttk.Label(header, text=f"model: {self.args.model_path}").pack(anchor="w")
        ttk.Label(header, text=f"joints: {len(self.controls)} | filter: {self.args.filter}").pack(anchor="w")

        buttons = ttk.Frame(self.root, padding=(8, 0, 8, 8))
        buttons.pack(fill="x")
        ttk.Button(buttons, text="Reset initial", command=self.reset_initial).pack(side="left")
        ttk.Button(buttons, text="Reset zero", command=self.reset_zero).pack(side="left", padx=6)
        ttk.Button(buttons, text="Print values", command=self.print_values).pack(side="left")
        ttk.Button(buttons, text="Close", command=self.close).pack(side="right")

        scrollable = ScrollableFrame(self.root)
        scrollable.pack(fill="both", expand=True)

        for row, joint in enumerate(self.controls):
            self._add_joint_row(scrollable.content, row, joint)

    def _add_joint_row(self, parent: ttk.Frame, row: int, joint: JointControl) -> None:
        frame = ttk.Frame(parent, padding=(8, 4))
        frame.grid(row=row, column=0, sticky="ew")
        frame.columnconfigure(1, weight=1)

        name_label = ttk.Label(frame, text=f"{joint.name} [{joint.unit}]", width=34)
        name_label.grid(row=0, column=0, sticky="w")

        value_label = ttk.Label(frame, text=self._format_value(float(self.data.qpos[joint.qpos_index])))
        value_label.grid(row=0, column=2, sticky="e", padx=(8, 0))
        self.value_labels[joint.name] = value_label

        resolution = 0.001 if joint.unit == "rad" else 0.0005
        scale = tk.Scale(
            frame,
            from_=joint.lower,
            to=joint.upper,
            resolution=resolution,
            orient="horizontal",
            showvalue=False,
            command=lambda raw_value, current_joint=joint: self.set_joint(current_joint, raw_value),
        )
        scale.set(float(self.data.qpos[joint.qpos_index]))
        scale.grid(row=1, column=0, columnspan=3, sticky="ew")
        self.scales[joint.name] = scale

    @staticmethod
    def _format_value(value: float) -> str:
        return f"{value:+.4f}"

    def set_joint(self, joint: JointControl, raw_value: str) -> None:
        value = float(raw_value)
        self.data.qpos[joint.qpos_index] = value
        self.data.qvel[joint.qvel_index] = 0.0
        mujoco.mj_forward(self.model, self.data)
        self.value_labels[joint.name].configure(text=self._format_value(value))
        self.sync_viewer()

    def sync_viewer(self) -> None:
        if not self.viewer.is_running():
            self.close()
            return
        with self.viewer.lock():
            self.viewer.sync()

    def sync_periodically(self) -> None:
        if not self.viewer.is_running():
            self.close()
            return
        self.sync_viewer()
        delay_ms = max(1, int(1000.0 / max(self.args.rate_hz, 1.0)))
        self.root.after(delay_ms, self.sync_periodically)

    def reset_initial(self) -> None:
        self.data.qpos[:] = self.initial_qpos
        self.data.qvel[:] = 0.0
        mujoco.mj_forward(self.model, self.data)
        self._refresh_scales_from_qpos()
        self.sync_viewer()

    def reset_zero(self) -> None:
        for joint in self.controls:
            value = min(max(0.0, joint.lower), joint.upper)
            self.data.qpos[joint.qpos_index] = value
            self.data.qvel[joint.qvel_index] = 0.0
        mujoco.mj_forward(self.model, self.data)
        self._refresh_scales_from_qpos()
        self.sync_viewer()

    def _refresh_scales_from_qpos(self) -> None:
        for joint in self.controls:
            value = float(self.data.qpos[joint.qpos_index])
            self.scales[joint.name].set(value)
            self.value_labels[joint.name].configure(text=self._format_value(value))

    def print_values(self) -> None:
        print("# current joint qpos")
        for joint in self.controls:
            value = float(self.data.qpos[joint.qpos_index])
            print(f"{joint.name}: {value:.6f}  # {joint.unit}")

    def close(self) -> None:
        try:
            if self.viewer.is_running():
                self.viewer.close()
        finally:
            self.root.quit()
            self.root.destroy()

    def run(self) -> None:
        self.sync_periodically()
        self.root.mainloop()


def require_model_path(path: Path) -> None:
    if not path.exists():
        raise FileNotFoundError(f"missing MuJoCo XML: {path}")


def main() -> None:
    args = parse_args()
    model_path = Path(args.model_path).expanduser().resolve()
    require_model_path(model_path)

    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    all_controls = discover_scalar_joints(model)
    controls = filter_controls(all_controls, args.filter)
    if not controls:
        raise SystemExit(f"no scalar joints matched filter: {args.filter}")

    print(f"Loaded model: {model_path}")
    print(f"Scalar joints: {len(all_controls)}, shown: {len(controls)}")
    print("Use the slider window to inspect joint motion. Close either window to exit.")

    with mujoco.viewer.launch_passive(model, data) as viewer_handle:
        app = JointSliderApp(model, data, viewer_handle, controls, args)
        app.run()


if __name__ == "__main__":
    main()