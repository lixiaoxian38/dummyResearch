"""Joint angle limits for Dummy (economy kit).

Hardware = mechanical / firmware CDC range.
Software (follow) = hardware inset per joint (most 15°, J5 3°).
FW = firmware CDC degrees (`>j…` / `#GETJPOS`).
ROS = MoveIt / joint_states radians (after RAD_VOLUMN / RAD_DIRECT).

J3 FW = ROS_deg + 90  (RAD_VOLUMN[2]=π/2)
J5/J6 FW = -ROS_deg   (RAD_DIRECT = -1)
"""

from __future__ import annotations

SOFTWARE_MARGIN_DEG = 1.5
# Thin bumper off the mechanical stop (goto / safety).
SOFTWARE_MARGIN_FW_DEG = [1.5, 1.5, 1.5, 1.5, 1.5, 1.5]

# Follow-only box. Camera is on the J5 housing: look-down should be J5, not
# J1 orbit / J3 fold. Floor used to be −70°; Servo then wanted ~−77° and the
# leftover Cartesian error became circling about J1. Stay off the ±90 stop.
FOLLOW_LIMITS_FW_DEG = [
    (-178.5, 178.5),  # J1  HW-1.5; a fake ±120 wall froze the arm
    (-118.5, 98.5),  # J2  HW-1.5
    (20.0, 200.0),  # J3  block elbow-flip below ~20 (was −58°)
    (-180.0, 180.0),  # J4  locked at follow-start (translation-only)
    (-85.0, 75.0),  # J5  pitch; −85 leaves ~5° to HW −90
    (-180.0, 180.0),  # J6  locked at follow-start (camera does not move)
]

# Mechanical / firmware hardware range (CDC FW deg).
JOINT_LIMITS_HW_FW_DEG = [
    (-180.0, 180.0),  # J1
    (-120.0, 100.0),  # J2
    (-60.0, 240.0),  # J3  ROS ±150 → FW -60..240
    (-180.0, 180.0),  # J4
    (-90.0, 90.0),  # J5
    (-180.0, 180.0),  # J6
]

JOINT_LIMITS_FW_DEG = [
    (lo + m, hi - m)
    for (lo, hi), m in zip(JOINT_LIMITS_HW_FW_DEG, SOFTWARE_MARGIN_FW_DEG)
]

# Same policy in ROS degrees (docs / URDF mental model).
JOINT_LIMITS_HW_ROS_DEG = [
    (-180.0, 180.0),
    (-120.0, 100.0),
    (-150.0, 150.0),
    (-180.0, 180.0),
    (-90.0, 90.0),
    (-180.0, 180.0),
]
JOINT_LIMITS_ROS_DEG = [
    (lo + m, hi - m)
    for (lo, hi), m in zip(JOINT_LIMITS_HW_ROS_DEG, SOFTWARE_MARGIN_FW_DEG)
]


def couple_j3_j5_fw(deg):
    """Stop wrist-into-arm: high J3 (fold) plus high +J5 (this mount looks down).

    Single-axis follow box still allows J3=200° and J5=+75° together, which
    collided on 2026-10-18. Charger start J3=120° / J5=+70° stays legal.
    Returns a copy and whether any axis was reduced.
    """
    import numpy as np

    out = np.array(deg, dtype=float, copy=True)
    j3 = float(out[2])
    j5 = float(out[4])

    def j3_cap_for(j5_deg: float) -> float:
        # J5 +30° → J3 may still reach 200°; J5 +75° → J3 capped at 130°.
        if j5_deg <= 30.0:
            return 200.0
        t = min(1.0, (j5_deg - 30.0) / 45.0)
        return 200.0 - t * 70.0

    def j5_cap_for(j3_deg: float) -> float:
        # J3 ≤150° keeps J5 +75°; J3 ≥190° drops J5 cap to +10°.
        if j3_deg <= 150.0:
            return 75.0
        t = min(1.0, (j3_deg - 150.0) / 40.0)
        return 75.0 - t * 65.0

    changed = False
    j3_cap = j3_cap_for(j5)
    if j3 > j3_cap:
        out[2] = j3_cap
        j3 = j3_cap
        changed = True
    j5_cap = j5_cap_for(j3)
    if j5 > j5_cap:
        out[4] = j5_cap
        changed = True
    return out, changed
