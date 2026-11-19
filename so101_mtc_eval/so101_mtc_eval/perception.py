"""The camera's block pose against the true one (M3, ADR 0001 criterion 3): the errors and their summary."""
import math

import numpy as np


def yaw_of(quaternion):
    """The yaw (rad) of a quaternion (x, y, z, w), about the vertical."""
    x, y, z, w = quaternion
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def errors(true_xy, true_yaw, found_xy, found_yaw):
    """Position error (m) and yaw error (rad, modulo a quarter turn, as a cube looks the same)."""
    distance = math.hypot(found_xy[0] - true_xy[0], found_xy[1] - true_xy[1])
    return distance, abs(math.remainder(found_yaw - true_yaw, math.pi / 2))


def summary(rows):
    """How many blocks were found, and the median, 95th percentile and maximum of their errors."""
    found = [row for row in rows if row["found"]]
    result = {"blocks": len(rows), "found": len(found)}
    if found:
        for key, name in (("error_mm", "error_mm"), ("yaw_error_deg", "yaw_error_deg")):
            values = np.array([row[key] for row in found])
            result[name] = {"median": float(np.median(values)), "p95": float(np.percentile(values, 95)),
                            "max": float(values.max())}
    return result
