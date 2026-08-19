"""Pure-math helpers for vision_bridge_node.py -- no ROS imports, so this is
testable with plain python3 (no `source /opt/ros/humble/setup.bash` needed).
"""
import math


def quat_from_z_axis(normal: list[float]) -> tuple[float, float, float, float]:
    """Shortest-arc quaternion (x, y, z, w) that rotates the reference
    +Z axis to point along ``normal``.

    Used to turn a hand's palm normal vector into a ROS orientation: the
    resulting pose's local +Z axis points along the palm normal. This is
    NOT a full 3-DOF hand orientation -- a single vector only fixes 2 of 3
    rotational degrees of freedom, so rotation *about* the normal itself
    (how the hand is "twisted") is left at whatever this formula's
    minimal-rotation choice happens to produce, not a real measurement.
    Good enough for "which way is the palm facing" (safety fencing,
    interaction), not for anything needing the hand's roll.
    """
    nx, ny, nz = normal
    norm = math.sqrt(nx * nx + ny * ny + nz * nz)
    if norm < 1e-8:
        return (0.0, 0.0, 0.0, 1.0)  # degenerate input, identity rotation
    nx, ny, nz = nx / norm, ny / norm, nz / norm

    # z x n (cross product of reference +Z with the target normal)
    vx, vy, vz = -ny, nx, 0.0
    c = nz  # dot(z, n)

    if c < -0.999999:
        # n points almost exactly opposite +Z -- cross product above is
        # near-zero/undefined, so pick an arbitrary perpendicular axis and
        # rotate 180 degrees around it instead.
        return (1.0, 0.0, 0.0, 0.0)

    s = math.sqrt((1.0 + c) * 2.0)
    inv_s = 1.0 / s
    return (vx * inv_s, vy * inv_s, vz * inv_s, s * 0.5)


def quat_rotate_z_axis(q: tuple[float, float, float, float]) -> tuple[float, float, float]:
    """Apply quaternion q to the reference +Z axis -- used only by tests, to
    check quat_from_z_axis actually produces what it claims to."""
    x, y, z, w = q
    # Standard quaternion-rotates-vector formula for v=(0,0,1).
    return (
        2 * (x * z + w * y),
        2 * (y * z - w * x),
        1 - 2 * (x * x + y * y),
    )
