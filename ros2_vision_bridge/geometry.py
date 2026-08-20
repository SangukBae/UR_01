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


def quat_rotate_vector(
    q: tuple[float, float, float, float], v: tuple[float, float, float]
) -> tuple[float, float, float]:
    """Apply quaternion q=(x,y,z,w) to an arbitrary vector v -- general
    version of quat_rotate_z_axis, used to round-trip-test
    matrix_to_quaternion against arbitrary rotation matrices."""
    qx, qy, qz, qw = q
    vx, vy, vz = v
    uvx, uvy, uvz = qy * vz - qz * vy, qz * vx - qx * vz, qx * vy - qy * vx
    uuvx = qy * uvz - qz * uvy
    uuvy = qz * uvx - qx * uvz
    uuvz = qx * uvy - qy * uvx
    return (
        vx + 2 * (qw * uvx + uuvx),
        vy + 2 * (qw * uvy + uuvy),
        vz + 2 * (qw * uvz + uuvz),
    )


def matrix_to_quaternion(
    R: list[list[float]], tol: float = 1e-4
) -> tuple[float, float, float, float]:
    """(x, y, z, w) quaternion for a proper 3x3 rotation matrix ``R`` (a
    list of 3 rows), for turning a fixed camera<->flange mounting
    rotation into a TF.

    Raises ValueError if ``R`` isn't -- within ``tol`` -- an orthogonal
    matrix with det=+1. Quaternions can only represent SO(3) (proper
    rotations); silently converting a reflection (det=-1, or any other
    non-rotation from e.g. a transcription typo) would produce a
    quaternion that does NOT actually reproduce R, which is worse than a
    loud error for something feeding a robot-relative TF.
    """
    r = R
    for i in range(3):
        for j in range(3):
            dot = sum(r[k][i] * r[k][j] for k in range(3))
            expected = 1.0 if i == j else 0.0
            if abs(dot - expected) > tol:
                raise ValueError(
                    f"matrix_to_quaternion: R is not orthogonal (columns {i} "
                    f"and {j} of R^T R differ from identity by "
                    f"{dot - expected:.4g}) -- check the matrix for a "
                    "transcription error."
                )
    det = (
        r[0][0] * (r[1][1] * r[2][2] - r[1][2] * r[2][1])
        - r[0][1] * (r[1][0] * r[2][2] - r[1][2] * r[2][0])
        + r[0][2] * (r[1][0] * r[2][1] - r[1][1] * r[2][0])
    )
    if abs(det - 1.0) > tol:
        raise ValueError(
            f"matrix_to_quaternion: det(R) = {det:.4g}, not +1 -- R is a "
            "reflection, not a proper rotation, so it has no quaternion "
            "representation. This usually means one entry's sign was "
            "mistyped -- check against the physical mount before using it."
        )

    trace = r[0][0] + r[1][1] + r[2][2]
    if trace > 0:
        s = math.sqrt(trace + 1.0) * 2  # s = 4*qw
        qw = 0.25 * s
        qx = (r[2][1] - r[1][2]) / s
        qy = (r[0][2] - r[2][0]) / s
        qz = (r[1][0] - r[0][1]) / s
    elif r[0][0] > r[1][1] and r[0][0] > r[2][2]:
        s = math.sqrt(1.0 + r[0][0] - r[1][1] - r[2][2]) * 2  # s = 4*qx
        qw = (r[2][1] - r[1][2]) / s
        qx = 0.25 * s
        qy = (r[0][1] + r[1][0]) / s
        qz = (r[0][2] + r[2][0]) / s
    elif r[1][1] > r[2][2]:
        s = math.sqrt(1.0 + r[1][1] - r[0][0] - r[2][2]) * 2  # s = 4*qy
        qw = (r[0][2] - r[2][0]) / s
        qx = (r[0][1] + r[1][0]) / s
        qy = 0.25 * s
        qz = (r[1][2] + r[2][1]) / s
    else:
        s = math.sqrt(1.0 + r[2][2] - r[0][0] - r[1][1]) * 2  # s = 4*qz
        qw = (r[1][0] - r[0][1]) / s
        qx = (r[0][2] + r[2][0]) / s
        qy = (r[1][2] + r[2][1]) / s
        qz = 0.25 * s

    return (qx, qy, qz, qw)
