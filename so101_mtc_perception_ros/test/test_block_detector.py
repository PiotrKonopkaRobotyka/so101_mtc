"""The installed node's conversions, without a simulator: a sensor_msgs/Image of a synthetic frame and the camera's
TF transform give the block's pose."""
import math

import cv2
import numpy as np
from geometry_msgs.msg import Transform
from sensor_msgs.msg import Image

from so101_mtc_perception_ros.block_detector import image_array, matrix, pose_from

# the simulated camera: 640x480, 58 deg vertical field of view, straight down from 0.55 m over (0.22, 0)
F = 240.0 / math.tan(math.radians(29.0))
K = np.array([[F, 0.0, 320.0], [0.0, F, 240.0], [0.0, 0.0, 1.0]])
EDGE = 0.025


def camera_transform():
    """world → camera_optical_frame as TF gives it: (0.22, 0, 0.55), x of the image along -y of world."""
    transform = Transform()
    transform.translation.x, transform.translation.z = 0.22, 0.55
    # rotation matrix [[0, -1, 0], [-1, 0, 0], [0, 0, -1]] as a quaternion
    transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w = (
        -math.sqrt(0.5), math.sqrt(0.5), 0.0, 0.0)
    return transform


def frame_with_block(x, y, yaw):
    pose = matrix(camera_transform())
    c, s = math.cos(yaw), math.sin(yaw)
    corners = np.array([[x + EDGE / 2 * (c * a - s * b), y + EDGE / 2 * (s * a + c * b), EDGE]
                        for a, b in ((1, 1), (-1, 1), (-1, -1), (1, -1))])
    camera = (corners - pose[:3, 3]) @ pose[:3, :3]
    pixels = (camera @ K.T)[:, :2] / (camera @ K.T)[:, 2:]
    image = np.full((480, 640, 3), (180, 180, 178), dtype=np.uint8)
    cv2.fillPoly(image, [np.round(pixels).astype(np.int32)], (230, 35, 25), lineType=cv2.LINE_AA)
    return image


def as_message(image):
    message = Image(height=image.shape[0], width=image.shape[1], encoding="rgb8", step=image.shape[1] * 3)
    message.data = image.tobytes()
    return message


def test_camera_transform_is_a_rotation_that_looks_down():
    pose = matrix(camera_transform())
    assert np.allclose(pose[:3, :3] @ pose[:3, :3].T, np.eye(3))
    assert np.allclose(pose[:3, 2], [0, 0, -1])  # the optical axis points down


def test_image_message_round_trip():
    image = frame_with_block(0.25, -0.05, 0.2)
    assert np.array_equal(image_array(as_message(image)), image)


def test_pose_from_a_frame_matches_the_block():
    found = pose_from(image_array(as_message(frame_with_block(0.27, 0.08, 0.35))), K, matrix(camera_transform()), EDGE)
    x, y, z, yaw = found
    assert math.hypot(x - 0.27, y - 0.08) < 0.0015
    assert z == EDGE / 2
    assert abs(math.degrees(math.remainder(yaw - 0.35, math.pi / 2))) < 2.0


def test_no_block_no_pose():
    empty = np.full((480, 640, 3), (180, 180, 178), dtype=np.uint8)
    assert pose_from(empty, K, matrix(camera_transform()), EDGE) is None
