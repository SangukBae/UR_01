"""Pure-math tests for geometry.py - run with plain python3, no ROS needed."""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from geometry import quat_from_z_axis, quat_rotate_z_axis  # noqa: E402


def _close(a, b, tol=1e-5):
    return all(abs(x - y) < tol for x, y in zip(a, b))


def test_identity_for_plus_z():
    q = quat_from_z_axis([0, 0, 1])
    assert _close(quat_rotate_z_axis(q), (0, 0, 1))


def test_rotates_to_plus_x():
    q = quat_from_z_axis([1, 0, 0])
    assert _close(quat_rotate_z_axis(q), (1, 0, 0))


def test_rotates_to_arbitrary_normal():
    n = [0.3, 0.6, math.sqrt(1 - 0.3**2 - 0.6**2)]
    q = quat_from_z_axis(n)
    assert _close(quat_rotate_z_axis(q), tuple(n))


def test_handles_opposite_direction():
    q = quat_from_z_axis([0, 0, -1])
    assert _close(quat_rotate_z_axis(q), (0, 0, -1))


def test_quaternion_is_unit_length():
    for n in ([0, 0, 1], [1, 0, 0], [0.3, 0.6, 0.74], [0, 0, -1], [-0.5, 0.5, 0.7071]):
        x, y, z, w = quat_from_z_axis(n)
        mag = math.sqrt(x * x + y * y + z * z + w * w)
        assert abs(mag - 1.0) < 1e-5, (n, mag)


def test_degenerate_zero_vector_returns_identity():
    assert quat_from_z_axis([0, 0, 0]) == (0.0, 0.0, 0.0, 1.0)


if __name__ == "__main__":
    failures = 0
    tests = {n: f for n, f in globals().items() if n.startswith("test_")}
    for name, fn in tests.items():
        try:
            fn()
            print(f"PASS {name}")
        except AssertionError as e:
            failures += 1
            print(f"FAIL {name}: {e}")
    print(f"\n{len(tests) - failures}/{len(tests)} passed")
    sys.exit(1 if failures else 0)
