"""Shared scan-angle mounting correction for safety and display consumers."""
import math


def robot_scan_angles(angle_min, angle_increment, *, yaw_offset_deg=0.0,
                      reflect_left_right=False):
    """Rotate by mount yaw, then optionally reflect the robot's lateral axis.

    Software robot coordinates remain forward/left. Reflection changes y's
    sign while preserving x, including after a nonzero mounting yaw correction.
    Leave invalid numeric metadata invalid for each consumer's own validator.
    """
    angle_min += math.radians(yaw_offset_deg % 360.0)
    if reflect_left_right:
        return -angle_min, -angle_increment
    return angle_min, angle_increment
