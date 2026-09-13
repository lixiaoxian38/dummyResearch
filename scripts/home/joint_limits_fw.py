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

# Follow-only box. Servo Jacobian loves to slam J5 to ±90 and flip J3
# to −58°, then the board is lost even though J5≈70° + J2/J3 still sees it.
# Camera is on J5; keep J5 off the stop so J2/J3 can take the look-down.
FOLLOW_LIMITS_FW_DEG = [
    (-178.5, 178.5),  # J1  HW-1.5; a fake ±120 wall froze the arm
    (-118.5, 98.5),  # J2  HW-1.5
    (20.0, 200.0),  # J3  block elbow-flip below ~20 (was −58°)
    (-180.0, 180.0),  # J4  locked at follow-start (translation-only)
    (-70.0, 75.0),  # J5  moves; cap so Servo cannot sit on ±90
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
