import math

import numpy as np

PolyCoefs = tuple[float, float, float, float, float, float]


def deg_to_rad(x: float) -> float:
    # return math.pi * x / 180.0
    return np.deg2rad(x)


def rad_to_deg(x: float) -> float:
    # return 180.0 * x / math.pi
    return np.rad2deg(x)


def evaluate_polynomial_at(poly: PolyCoefs, t: float) -> tuple[float, float, float]:
    """Evaluate a quintic polynomial at a given time.

    Args:
        poly: Polynomial coefficients.
        t: Time at which to evaluate the polynomial.

    Returns:
        Position, velocity, and acceleration at the given time.
    """

    a0, a1, a2, a3, a4, a5 = poly

    t = float(t)
    pos = a0 + (a1 * t) + (a2 * t**2) + (a3 * t**3) + (a4 * t**4) + (a5 * t**5)
    vel = a1 + (2 * a2 * t) + (3 * a3 * t**2) + (4 * a4 * t**3) + (5 * a5 * t**4)
    accel = (2 * a2) + (6 * a3 * t) + (12 * a4 * t**2) + (20 * a5 * t**3)
    return (pos, vel, accel)


def is_segment_feasible(
    duration_s: float,
    poly: PolyCoefs,
    v_des: float,
    a_des: float,
    t: float = 0.0,
    inc: float = 0.1,
) -> tuple[bool, float, float]:
    """Determine whether a segment adheres to dynamic limits.

    Args:
        duration_s: Duration of the segment.
        poly: Polynomial coefficients.
        v_des: Desired velocity.
        a_des: Desired acceleration.
        t: Time at which to start evaluating the segment.
        inc: Time increment to use when evaluating the segment.

    Returns:
        Whether the segment is feasible and the maximum velocity and
        acceleration that can be achieved.
    """

    max_v, max_a = 0.0, 0.0
    success = True

    while t < duration_s:
        _, vel_t, acc_t = evaluate_polynomial_at(poly, t)
        max_v = max(max_v, abs(vel_t))
        max_a = max(max_a, abs(acc_t))
        if abs(vel_t) > v_des or abs(acc_t) > a_des:
            success = False
        t = min(duration_s, t + inc)

    return success, max_v, max_a


def generate_quintic_polynomial(
    i: tuple[float, float, float, float],
    f: tuple[float, float, float, float],
) -> tuple[float, PolyCoefs]:
    """Generate quintic polynomial from two points

    Args:
        i: Initial state (time, position, velocity, acceleration).
        f: Final state (time, position, velocity, acceleration).

    Returns:
        Duration of the polynomial and the polynomial coefficients.
    """

    (t_i, p_i, v_i, a_i), (t_f, p_f, v_f, a_f) = i, f
    dt = t_f - t_i

    a0 = p_i
    a1 = v_i
    a2 = a_i / 2.0
    a3 = (20 * p_f - 20 * p_i - (8 * v_f + 12 * v_i) * dt - (3 * a_i - a_f) * (dt**2)) / (2 * (dt**3))
    a4 = (30 * p_i - 30 * p_f + (14 * v_f + 16 * v_i) * dt + (3 * a_i - 2 * a_f) * (dt**2)) / (2 * (dt**4))
    a5 = (12 * p_f - 12 * p_i - (6 * v_f + 6 * v_i) * dt - (a_i - a_f) * (dt**2)) / (2 * (dt**5))

    return dt, (a0, a1, a2, a3, a4, a5)


def generate_cubic_polynomial(i: tuple[float, float, float], f: tuple[float, float, float]) -> tuple[float, PolyCoefs]:
    """Generate cubic polynomial from two points

    Args:
        i: Initial state (time, position, velocity).
        f: Final state (time, position, velocity).

    Returns:
        Duration of the polynomial and the polynomial coefficients.
    """

    (t_i, p_i, v_i), (t_f, p_f, v_f) = i, f
    dt = t_f - t_i

    a0 = p_i
    a1 = v_i
    a2 = (3 / dt**2) * (p_f - p_i) - (2 / dt) * v_i - (1 / dt) * v_f
    a3 = (-2 / dt**3) * (p_f - p_i) + (1 / dt**2) * (v_f + v_i)
    a4, a5 = 0.0, 0.0

    return dt, (a0, a1, a2, a3, a4, a5)


def generate_linear_polynomial(i: tuple[float, float], f: tuple[float, float]) -> tuple[float, PolyCoefs]:
    """Generate linear polynomial from two points

    Args:
        i: Initial state (time, position).
        f: Final state (time, position).

    Returns:
        Duration of the polynomial and the polynomial coefficients.
    """

    (t_i, p_i), (t_f, p_f) = i, f
    dt = t_f - t_i

    a0 = p_i
    a1 = (p_f - p_i) / dt
    a2, a3, a4, a5 = 0.0, 0.0, 0.0, 0.0

    return dt, (a0, a1, a2, a3, a4, a5)


def get_pose_diff(
    pose0: tuple[float, float, float],
    pose1: tuple[float, float, float],
    translation_atol: float = 2e-3,
    rotation_atol: float = 2e-2,
) -> tuple[float, float]:
    """Return the motion required to get from pose 0 to pose 1.

    Assumed that between pose 0 and pose 1, there has only been
    either a translation or rotation motion.

    Args:
        pose0: Initial pose (x, y, theta).
        pose1: Final pose (x, y, theta).
        translation_atol: Absolute tolerance for translation.
        rotation_atol: Absolute tolerance for rotation.

    Returns:
        The distance to travel and the angle to rotate.
    """

    x0, y0, theta0 = pose0
    x1, y1, theta1 = pose1
    theta0 = np.arctan2(np.sin(theta0), np.cos(theta0))  # Constrains to [-pi, pi]
    theta1 = np.arctan2(np.sin(theta1), np.cos(theta1))  # Constrains to [-pi, pi]

    # For now, we use a simplified motion model where we assume
    # that every motion is either a translation OR a rotation,
    # and the translation is either straight forward or straight back
    if np.isclose(x0, x1, atol=translation_atol) and np.isclose(y0, y1, atol=translation_atol):
        return 0.0, theta1 - theta0

    if np.isclose(theta0, theta1, atol=rotation_atol):
        dx = x1 - x0
        dy = y1 - y0
        drive_angle = math.atan2(dy, dx)
        distance = math.hypot(dy, dx)
        if np.isclose(drive_angle, theta0, atol=rotation_atol):
            return distance, 0.0
        opposite_theta0 = np.arctan2(np.sin(theta0 + np.pi), np.cos(theta0 + np.pi))  # Constrains to [-pi, pi]
        if np.isclose(drive_angle, opposite_theta0, atol=rotation_atol):
            return -distance, 0.0

    return 0.0, 0.0
