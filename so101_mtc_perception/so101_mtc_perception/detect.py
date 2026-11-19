"""The block in the fixed camera's image, by colour (OpenCV), and its pose on the table.

The block is red on a grey table; the arm is yellow and black. Lit from above, the block's top face is bright while
its sides, seen at an angle near the image's edges, are dark: a brightness threshold keeps the top face alone. Its
centre lies over the block's centre, at the height of the top face, so the ray through it meets the plane z = edge
there (geometry.pixel_to_plane). The yaw comes from one edge of the face, its two corners projected the same way.
"""
from dataclasses import dataclass
import math

import cv2
import numpy as np

from so101_mtc_perception.geometry import pixel_to_plane


@dataclass(frozen=True)
class Thresholds:
    """HSV thresholds of OpenCV's 8-bit scale (hue 0-179): red wraps around 0."""

    hue_low_max: int = 10  # red: hue <= this, or >= hue_high_min
    hue_high_min: int = 170
    min_saturation: int = 120
    min_value: int = 150  # the lit top face; the sides are darker
    min_area: float = 20.0  # px², smaller blobs are noise


@dataclass(frozen=True)
class Detection:
    u: float  # px, the top face's centre
    v: float
    corners: np.ndarray  # 4x2 px, the rectangle fitted to the face, in order around it
    area: float  # px²


def detect_block(image_rgb, thresholds=Thresholds()):
    """The block's top face in an RGB image, or None when no red blob is large enough."""
    hsv = cv2.cvtColor(np.asarray(image_rgb, dtype=np.uint8), cv2.COLOR_RGB2HSV)
    hue, saturation, value = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    red = (hue <= thresholds.hue_low_max) | (hue >= thresholds.hue_high_min)
    mask = (red & (saturation >= thresholds.min_saturation) & (value >= thresholds.min_value)).astype(np.uint8)
    mask = cv2.morphologyEx(mask * 255, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    contour = max(contours, key=cv2.contourArea)
    area = float(cv2.contourArea(contour))
    if area < thresholds.min_area:
        return None
    rectangle = cv2.minAreaRect(contour)
    return Detection(float(rectangle[0][0]), float(rectangle[0][1]), cv2.boxPoints(rectangle), area)


def block_pose(detection, camera_matrix, world_from_camera, top_z):
    """(x, y, yaw) of the block in the world: its centre and the yaw of its sides, in (-45, 45] degrees (rad), since a
    cube looks the same every quarter turn. None when a ray misses the plane z = top_z."""
    centre = pixel_to_plane(detection.u, detection.v, camera_matrix, world_from_camera, top_z)
    first = pixel_to_plane(*detection.corners[0], camera_matrix, world_from_camera, top_z)
    second = pixel_to_plane(*detection.corners[1], camera_matrix, world_from_camera, top_z)
    if centre is None or first is None or second is None:
        return None
    edge = second - first
    yaw = math.remainder(math.atan2(edge[1], edge[0]), math.pi / 2.0)
    return float(centre[0]), float(centre[1]), yaw
