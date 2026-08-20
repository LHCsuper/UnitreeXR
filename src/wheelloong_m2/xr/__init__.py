"""XR-source and adapter abstractions; currently fake-source only."""

from .adapter import XRAdapter
from .source import FakeXRSource
from .types import XRControllerPose

__all__ = ["FakeXRSource", "XRAdapter", "XRControllerPose"]
