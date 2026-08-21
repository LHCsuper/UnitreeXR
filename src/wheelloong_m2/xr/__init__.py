"""XR-source and explicit robot-target adapter abstractions."""

from .adapter import (
    UNITREE_ROBOT_FROM_OPENXR_BASIS,
    InitializedRelativeXRAdapter,
    RelativeXRMapping,
    XRAdapter,
)
from .robotoolkit_source import XRoboToolkitSource
from .source import FakeXRSource
from .types import XRControllerPose

__all__ = [
    "FakeXRSource",
    "InitializedRelativeXRAdapter",
    "RelativeXRMapping",
    "UNITREE_ROBOT_FROM_OPENXR_BASIS",
    "XRAdapter",
    "XRControllerPose",
    "XRoboToolkitSource",
]
