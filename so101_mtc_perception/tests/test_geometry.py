import numpy as np
import pytest

from so101_mtc_perception.geometry import pixel_to_plane, pixel_to_ray

# 640 x 480 image, focal length 500 px, the principal point in the centre
K = np.array([[500.0, 0.0, 320.0], [0.0, 500.0, 240.0], [0.0, 0.0, 1.0]])

# The camera 0.5 m above the world origin, looking straight down:
# optical x = world x, optical y = world -y, optical z (forward) = world -z.
LOOKING_DOWN = np.array([
    [1.0, 0.0, 0.0, 0.0],
    [0.0, -1.0, 0.0, 0.0],
    [0.0, 0.0, -1.0, 0.5],
    [0.0, 0.0, 0.0, 1.0],
])


def test_principal_point_looks_along_the_optical_axis():
    np.testing.assert_allclose(pixel_to_ray(320, 240, K), [0.0, 0.0, 1.0], atol=1e-12)


def test_ray_is_a_unit_vector():
    assert np.linalg.norm(pixel_to_ray(0, 0, K)) == pytest.approx(1.0)


def test_principal_point_meets_the_table_below_the_camera():
    np.testing.assert_allclose(pixel_to_plane(320, 240, K, LOOKING_DOWN, 0.0), [0.0, 0.0, 0.0], atol=1e-12)


def test_pixel_offset_scales_with_the_distance():
    # 100 px right and 100 px down at f = 500 px and 0.5 m: 0.1 m along optical x and y
    np.testing.assert_allclose(pixel_to_plane(420, 340, K, LOOKING_DOWN, 0.0), [0.1, -0.1, 0.0], atol=1e-12)


def test_higher_plane_is_closer():
    # the top face of a 2 cm block is 0.48 m from the camera, where 100 px is 0.096 m
    np.testing.assert_allclose(pixel_to_plane(420, 240, K, LOOKING_DOWN, 0.02), [0.096, 0.0, 0.02], atol=1e-12)


def test_plane_behind_the_camera_gives_none():
    assert pixel_to_plane(320, 240, K, LOOKING_DOWN, 1.0) is None


def test_ray_parallel_to_the_plane_gives_none():
    # the camera looks horizontally along world x: optical x = world -y, optical y = world -z
    looking_ahead = np.array([
        [0.0, 0.0, 1.0, 0.0],
        [-1.0, 0.0, 0.0, 0.0],
        [0.0, -1.0, 0.0, 0.5],
        [0.0, 0.0, 0.0, 1.0],
    ])
    assert pixel_to_plane(320, 240, K, looking_ahead, 0.0) is None
