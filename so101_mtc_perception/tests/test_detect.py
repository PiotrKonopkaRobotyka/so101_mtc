"""The block's detection and pose on synthetic images: the top face drawn where the camera sees it."""
import math

import cv2
import numpy as np
import pytest

from so101_mtc_perception.detect import Thresholds, block_pose, detect_block

# the simulated camera of so101_mtc_description: 640x480, a 58 deg vertical field of view, straight down from 0.55 m
# over (0.22, 0)
F = 240.0 / math.tan(math.radians(29.0))
K = np.array([[F, 0.0, 320.0], [0.0, F, 240.0], [0.0, 0.0, 1.0]])
EDGE = 0.025
RED, DARK_RED, YELLOW, GREY = (230, 35, 25), (60, 10, 8), (230, 190, 30), (180, 180, 178)


def world_from_camera():
    """camera_link at (0.22, 0, 0.55) with rpy (0, pi/2, 0), then the optical frame's rpy (-pi/2, 0, -pi/2)."""
    def rpy(r, p, y):
        cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
        return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                         [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                         [-sp, cp * sr, cp * cr]])
    pose = np.eye(4)
    pose[:3, :3] = rpy(0, math.pi / 2, 0) @ rpy(-math.pi / 2, 0, -math.pi / 2)
    pose[:3, 3] = [0.22, 0.0, 0.55]
    return pose


def project(points, pose=None):
    """World points → pixels through the pinhole camera."""
    pose = world_from_camera() if pose is None else pose
    camera = (np.asarray(points) - pose[:3, 3]) @ pose[:3, :3]
    pixels = camera @ K.T
    return pixels[:, :2] / pixels[:, 2:]


def face(x, y, yaw, z=EDGE):
    corners = [(+1, +1), (-1, +1), (-1, -1), (+1, -1)]
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[x + EDGE / 2 * (c * a - s * b), y + EDGE / 2 * (s * a + c * b), z] for a, b in corners])


def image_with(*polygons):
    image = np.full((480, 640, 3), GREY, dtype=np.uint8)
    for points, colour in polygons:
        cv2.fillPoly(image, [np.round(project(points)).astype(np.int32)], colour, lineType=cv2.LINE_AA)
    return image


@pytest.mark.parametrize("x, y, yaw_deg", [(0.256, -0.091, -5.0), (0.20, 0.12, 30.0), (0.33, -0.15, 44.0),
                                           (0.15, 0.0, 0.0), (0.30, 0.20, -40.0)])
def test_the_pose_from_the_top_face_matches_the_block(x, y, yaw_deg):
    detection = detect_block(image_with((face(x, y, math.radians(yaw_deg)), RED)))
    assert detection is not None
    found_x, found_y, found_yaw = block_pose(detection, K, world_from_camera(), EDGE)
    assert math.hypot(found_x - x, found_y - y) < 0.0015  # a pixel covers about 1.2 mm
    assert abs(math.degrees(math.remainder(found_yaw - math.radians(yaw_deg), math.pi / 2))) < 2.0
    assert -math.pi / 4 <= found_yaw <= math.pi / 4


def test_no_block_no_detection():
    assert detect_block(image_with()) is None


def test_the_yellow_arm_and_the_dark_sides_are_ignored():
    top = face(0.25, 0.05, 0.3)
    side = np.vstack([top[:2], top[1::-1] - [0.0, 0.0, EDGE]])  # a side face, under two top corners
    arm = face(0.10, -0.10, 0.0, z=0.10)
    detection = detect_block(image_with((arm, YELLOW), (side, DARK_RED), (top, RED)))
    found_x, found_y, _ = block_pose(detection, K, world_from_camera(), EDGE)
    assert math.hypot(found_x - 0.25, found_y - 0.05) < 0.0015


def test_a_speck_is_noise():
    image = image_with()
    cv2.circle(image, (100, 100), 2, RED, -1)
    assert detect_block(image, Thresholds()) is None
