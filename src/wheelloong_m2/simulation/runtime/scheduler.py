"""Deterministic simulation-time scheduler for offline multi-rate loops."""

from __future__ import annotations

from dataclasses import dataclass

from .config import IK_PERIOD_S, SIMULATION_PERIOD_S, TARGET_PERIOD_S


_TIME_EPSILON_S = 1e-12


@dataclass(frozen=True)
class RuntimeTick:
    """Work due at one physics step, timestamped before that step advances."""

    time_s: float
    target_due: bool
    ik_due: bool
    simulation_due: bool = True


class MultiRateScheduler:
    """Generate target, IK, and physics ticks from a simulation-time accumulator.

    It has no wall-clock sleeps. If a caller advances the scheduler slowly,
    overdue periodic events are coalesced into one current-time tick instead
    of replaying stale work.
    """

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        """Reset the accumulator so all three periodic activities begin at t=0."""
        self.simulation_time_s = 0.0
        self.target_tick_count = 0
        self.ik_tick_count = 0
        self.physics_tick_count = 0
        self._next_target_time_s = 0.0
        self._next_ik_time_s = 0.0

    @staticmethod
    def _consume_due(current_time_s: float, next_time_s: float, period_s: float) -> tuple[bool, float]:
        """Return whether an event is due and advance its schedule past now."""
        if current_time_s + _TIME_EPSILON_S < next_time_s:
            return False, next_time_s
        while next_time_s <= current_time_s + _TIME_EPSILON_S:
            next_time_s += period_s
        return True, next_time_s

    def next_tick(self) -> RuntimeTick:
        """Return current due work and advance the simulation accumulator one step."""
        current_time_s = self.simulation_time_s
        target_due, self._next_target_time_s = self._consume_due(
            current_time_s,
            self._next_target_time_s,
            TARGET_PERIOD_S,
        )
        ik_due, self._next_ik_time_s = self._consume_due(
            current_time_s,
            self._next_ik_time_s,
            IK_PERIOD_S,
        )
        if target_due:
            self.target_tick_count += 1
        if ik_due:
            self.ik_tick_count += 1
        self.physics_tick_count += 1
        self.simulation_time_s += SIMULATION_PERIOD_S
        return RuntimeTick(
            time_s=current_time_s,
            target_due=target_due,
            ik_due=ik_due,
        )
