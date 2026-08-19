"""Forward kinematics for the classic UR10, nominal (uncalibrated) DH parameters.

Used only for a coarse pre-move safety check (workspace bounds): "would this
joint target put the TCP somewhere unreasonable?" -- not for precision path
planning. The real robot uses a per-unit calibration (see
../ros2_ur_driver/config/ur10_calibration.yaml) that shifts the true joint
origins a few millimetres from these nominal values, so this is deliberately
checked with margin, not trusted to the millimetre.

Pure standard library (manual 4x4 matrices): the safety gate in server.py
should not need numpy just to reject an unreasonable target before anything
moves.
"""
from __future__ import annotations

import math

Matrix4 = list[list[float]]

# (a, d, alpha) per joint, base..wrist3 -- classic UR10 (CB-series) DH
# parameters, standard convention: T_i = Rz(theta_i) Tz(d_i) Tx(a_i) Rx(alpha_i)
_DH = (
    (0.0,      0.1273,    math.pi / 2),
    (-0.612,   0.0,       0.0),
    (-0.5723,  0.0,       0.0),
    (0.0,      0.163941,  math.pi / 2),
    (0.0,      0.1157,   -math.pi / 2),
    (0.0,      0.0922,    0.0),
)

_IDENTITY: Matrix4 = [[1.0 if i == j else 0.0 for j in range(4)] for i in range(4)]


def _dh_matrix(theta: float, d: float, a: float, alpha: float) -> Matrix4:
    ct, st = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(alpha), math.sin(alpha)
    return [
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0.0, sa,       ca,      d],
        [0.0, 0.0,      0.0,     1.0],
    ]


def _matmul(a: Matrix4, b: Matrix4) -> Matrix4:
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def forward_kinematics(q_rad: list[float]) -> list[float]:
    """Joint angles (radians, base..wrist3) -> flange position [x, y, z]
    (metres) in the base frame. No tool offset (flange, not TCP)."""
    if len(q_rad) != len(_DH):
        raise ValueError(f"Expected {len(_DH)} joint angles, got {len(q_rad)}.")
    t = _IDENTITY
    for theta, (a, d, alpha) in zip(q_rad, _DH):
        t = _matmul(t, _dh_matrix(theta, d, a, alpha))
    return [t[0][3], t[1][3], t[2][3]]
