"""Camera geometry: from the block's pixel in the image to a point on the table.

One camera measures no depth, but the block lies on the table: the ray through its pixel meets the table plane in
one point. The camera frame is the ROS optical frame (REP 103): x right, y down, z forward. Lens distortion is
ignored (the simulated camera has none).
"""

import numpy as np


def pixel_to_ray(u: float, v: float, camera_matrix: np.ndarray) -> np.ndarray:
    """Unit direction of the ray through pixel (u, v), in the optical frame.

    camera_matrix is K of `sensor_msgs/CameraInfo` as 3x3: [[fx, 0, cx], [0, fy, cy], [0, 0, 1]].
    """
    ray = np.linalg.solve(np.asarray(camera_matrix, dtype=float), [u, v, 1.0])
    return ray / np.linalg.norm(ray)


def pixel_to_plane(
    u: float, v: float, camera_matrix: np.ndarray, world_from_camera: np.ndarray, plane_z: float
) -> np.ndarray | None:
    """Point in the world frame where the ray through pixel (u, v) meets the horizontal plane z = plane_z.

    world_from_camera is the 4x4 pose of the optical frame in the world frame (from TF). None when the ray runs
    parallel to the plane or meets it behind the camera.
    """
    world_from_camera = np.asarray(world_from_camera, dtype=float)
    origin = world_from_camera[:3, 3]
    direction = world_from_camera[:3, :3] @ pixel_to_ray(u, v, camera_matrix)
    if abs(direction[2]) < 1e-9:
        return None
    distance = (plane_z - origin[2]) / direction[2]
    if distance <= 0.0:
        return None
    return origin + distance * direction
