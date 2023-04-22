from dataclasses import dataclass
from typing import Any, Iterable

import numpy as np

from stretch.utils.trajectories import (
    PolyCoefs,
    evaluate_polynomial_at,
    generate_cubic_polynomial,
    generate_linear_polynomial,
    generate_quintic_polynomial,
    is_segment_feasible,
)

# Limits how close together waypoints can be planned
WAYPOINT_ISCLOSE_ATOL = 0.8


@dataclass
class Waypoint:
    time: float
    position: float
    velocity: float | None = None
    acceleration: float | None = None

    def __post_init__(self) -> None:
        if self.acceleration is not None:
            assert self.velocity is not None, "Cannot specify acceleration without velocity"

    def __eq__(self, other: Any) -> bool:
        if not isinstance(other, Waypoint):
            raise NotImplementedError
        return bool(np.isclose(self.time, other.time, atol=WAYPOINT_ISCLOSE_ATOL))

    def __ne__(self, other: Any) -> bool:
        return not self.__eq__(other)

    def __lt__(self, other: Any) -> bool:
        if not isinstance(other, Waypoint):
            raise NotImplementedError
        return bool(np.less(self.time, other.time))

    def __le__(self, other: Any) -> bool:
        if not isinstance(other, Waypoint):
            raise NotImplementedError
        return bool(np.less_equal(self.time, other.time))

    def __gt__(self, other: Any) -> bool:
        return not self.__le__(other)

    def __ge__(self, other: Any) -> bool:
        return not self.__lt__(other)


@dataclass
class SE2Waypoint:
    time: float
    pose: tuple[float, float, float]
    vel_twist: tuple[float, float] | None = None
    acc_twist: tuple[float, float] | None = None

    def __post_init__(self) -> None:
        if self.acc_twist is not None:
            assert self.vel_twist is not None, "Cannot specify acceleration without velocity"


@dataclass
class Segment:
    segment_id: int
    duration: float
    coefs: PolyCoefs
    tol: float = 1e-2

    def __eq__(self, other: Any) -> bool:
        if not isinstance(other, Segment):
            return NotImplemented

        a0, a1, a2, a3, a4, a5 = self.coefs
        b0, b1, b2, b3, b4, b5 = other.coefs

        return bool(
            np.isclose(self.duration, other.duration, atol=self.tol)
            and np.isclose(a0, b0, atol=self.tol)
            and np.isclose(a1, b1, atol=self.tol)
            and np.isclose(a2, b2, atol=self.tol)
            and np.isclose(a3, b3, atol=self.tol)
            and np.isclose(a4, b4, atol=self.tol)
            and np.isclose(a5, b5, atol=self.tol)
        )

    def __ne__(self, other: Any) -> bool:
        return not self.__eq__(other)

    @classmethod
    def zeros(cls, segment_id: int = 2) -> "Segment":
        return cls(segment_id=segment_id, duration=0, coefs=(0, 0, 0, 0, 0, 0))

    @classmethod
    def from_two_waypoints(cls, w1: Waypoint, w2: Waypoint, segment_id: int) -> "Segment":
        if w1.velocity is not None and w2.velocity is not None:
            if w1.acceleration is not None and w2.acceleration is not None:
                duration, coefs = generate_quintic_polynomial(
                    (w1.time, w1.position, w1.velocity, w1.acceleration),
                    (w2.time, w2.position, w2.velocity, w2.acceleration),
                )
            else:
                duration, coefs = generate_cubic_polynomial(
                    (w1.time, w1.position, w1.velocity),
                    (w2.time, w2.position, w2.velocity),
                )
        else:
            duration, coefs = generate_linear_polynomial((w1.time, w1.position), (w2.time, w2.position))
        return cls(segment_id, duration, coefs)

    def evaluate_at(self, t: float) -> tuple[float, float, float]:
        """Evaluates segment at a given time.

        Args:
            t: Time to evaluate segment at.

        Returns:
            Position, velocity, and acceleration at time t.
        """

        return evaluate_polynomial_at(self.coefs, t)

    def is_valid(self, v_des: float, a_des: float) -> bool:
        """Determines whether segment adheres to dynamic limits.

        Args:
            v_des: Desired velocity
            a_des: Desired acceleration

        Returns:
            True if segment is valid, False otherwise.
        """

        success, _, _ = is_segment_feasible(self.duration, self.coefs, v_des, a_des)
        return success


class Spline:
    waypoints: list[Waypoint]

    def __len__(self) -> int:
        return len(self.waypoints)

    def __getitem__(self, index: int) -> Waypoint:
        return self.waypoints[index]

    def __setitem__(self, index: int, waypoint: Waypoint) -> None:
        self.waypoints[index] = waypoint

    def __delitem__(self, index: int) -> None:
        del self.waypoints[index]

    def __iter__(self) -> Iterable[Waypoint]:
        yield from self.waypoints

    def pop(self, index: int = -1) -> Waypoint:
        return self.waypoints.pop(index)

    def clear(self) -> None:
        self.waypoints = []

    def add(self, time: float, pos: float, vel: float | None = None, accel: float | None = None) -> None:
        """Add a waypoint to the spline.

        This method will sort through the existing waypoints
        in the spline to insert the waypoint such that
        waypoint time increases with index in the array.

        Args:
            time: Time of the waypoint.
            pos: Position of the waypoint.
            vel: Velocity of the waypoint.
            accel: Acceleration of the waypoint.
        """

        new_waypoint = Waypoint(time=time, position=pos, velocity=vel, acceleration=accel)
        self.add_waypoint(new_waypoint)

    def add_waypoint(self, new_waypoint: Waypoint) -> None:
        """Add a waypoint to the spline.

        This method will sort through the existing waypoints
        in the spline to insert the waypoint such that
        waypoint time increases with index in the array.

        Args:
            new_waypoint: Waypoint to add to the spline.
        """

        if len(self.waypoints) == 0:
            self.waypoints.append(new_waypoint)
            return

        # Cannot have two waypoints scheduled for the same time
        if new_waypoint in self.waypoints:
            return

        # Prepend or append if before first or after last waypoint
        if new_waypoint < self.waypoints[0]:
            self.waypoints.insert(0, new_waypoint)
            return
        if new_waypoint > self.waypoints[0]:
            self.waypoints.append(new_waypoint)
            return

        # Insert before first later waypoint
        for i, other_waypoint in enumerate(self.waypoints):
            if new_waypoint < other_waypoint:
                self.waypoints.insert(i, new_waypoint)
                return

    def get_num_segments(self) -> int:
        return max(0, len(self.waypoints) - 1)

    def get_segment(self, index: int) -> Segment | None:
        """Retrieves a segment in the spline by index

        Num of segments is one less than number of waypoints in the trajectory.
        Index bounds are [-1 * num_seg, num_seg).

        Args:
            index: Index of the segment to retrieve.

        Returns:
            Segment at the given index.
        """

        if index < -1 * len(self.waypoints) + 1 or index >= len(self.waypoints) - 1:
            return None
        index = index - 1 if index < 0 else index
        w0, w1 = self.waypoints[index], self.waypoints[index + 1]
        return Segment.from_two_waypoints(w0, w1, segment_id=index + 2)

    def evaluate_at(self, t: float) -> tuple[float, float, float] | None:
        """Evaluate a point along the curve at a given time.

        Args:
            t: Time to evaluate the curve at.

        Returns:
            Position, velocity, and acceleration at time t, or None if t is out
            of bounds.
        """

        if len(self.waypoints) < 2:
            return None

        # Return bounds for early or late t
        if t < self.waypoints[0].time:
            wp = self.waypoints[0]
            assert wp.velocity is not None and wp.acceleration is not None
            return (wp.position, wp.velocity, wp.acceleration)
        if t > self.waypoints[-1].time:
            wp = self.waypoints[-1]
            assert wp.velocity is not None and wp.acceleration is not None
            return (wp.position, wp.velocity, wp.acceleration)

        # Find segment indices
        for i in range(self.get_num_segments()):
            if t >= self.waypoints[i].time and t <= self.waypoints[i + 1].time:
                w0 = self.waypoints[i]
                w1 = self.waypoints[i + 1]
                return evaluate_polynomial_at(Segment.from_two_waypoints(w0, w1, -1).coefs, t - w0.time)

        return None

    def is_valid(self, v_des: float, a_des: float) -> bool:
        """Determines whether spline is well-formed and adheres to dynamic limits.

        Args:
            v_des: Desired velocity
            a_des: Desired acceleration

        Returns:
            True if spline is valid, False otherwise.
        """

        if len(self.waypoints) < 2:
            return True

        if not np.isclose(self.waypoints[0].time, 0.0, atol=WAYPOINT_ISCLOSE_ATOL):
            return False

        t = -1.0
        for waypoint in self.waypoints:
            if waypoint.time < 0.0:
                return False
            if np.isclose(waypoint.time, t, atol=WAYPOINT_ISCLOSE_ATOL):
                return False
            if waypoint.time < t:
                return False
            t = waypoint.time

        for i in range(self.get_num_segments()):
            assert (segment := self.get_segment(i)) is not None
            success, _, _ = is_segment_feasible(segment.duration, segment.coefs, v_des, a_des)
            if not success:
                return False

        return True
