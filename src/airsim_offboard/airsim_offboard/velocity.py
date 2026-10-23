"""Pure velocity and yaw helpers for the offboard control node.

Kept free of rclpy, airsim and cv2 imports so the flight math can be unit tested
without a ROS 2 environment or a running simulator.
"""
import math


def step_axis(value: float, sign: int, step: float, limit: float) -> float:
    """Advance one velocity axis by ``sign * step``, clamped to +/- ``limit``."""
    return max(-limit, min(limit, value + sign * step))


def decay_velocity(value: float, decay_rate: float) -> float:
    """Move ``value`` toward zero by ``decay_rate`` without overshooting or flipping sign."""
    if value > 0:
        return max(0.0, value - decay_rate)
    if value < 0:
        return min(0.0, value + decay_rate)
    return 0.0


def wrap_yaw(yaw: float) -> float:
    """Wrap a yaw angle into [-pi, pi]."""
    while yaw < -math.pi:
        yaw += 2 * math.pi
    while yaw > math.pi:
        yaw -= 2 * math.pi
    return yaw
