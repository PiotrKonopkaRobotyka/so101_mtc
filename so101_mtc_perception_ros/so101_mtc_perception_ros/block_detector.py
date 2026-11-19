"""The block's pose from the fixed camera (M3): a thin ROS 2 layer over so101_mtc_perception.

It takes /camera/image_raw (rgb8) and /camera/camera_info (sensor data QoS), and the camera's pose in world from TF.
so101_mtc_perception finds the block's top face by colour and projects it onto the plane of the top face. The pose
goes out on /block_pose (geometry_msgs/PoseStamped in world, stamped with the image's time; reliable and transient
local, so that a late subscriber still gets the last one), with the block's centre half its edge above the table.

    ros2 run so101_mtc_perception_ros block_detector --ros-args -p use_sim_time:=true
"""
import math
from pathlib import Path

import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory

from so101_mtc_perception.detect import block_pose, detect_block


def image_array(message):
    """sensor_msgs/Image (rgb8) as an HxWx3 array, without cv_bridge."""
    if message.encoding != "rgb8":
        raise ValueError(f"expected rgb8, got {message.encoding}")
    rows = np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.step)
    return rows[:, : message.width * 3].reshape(message.height, message.width, 3)


def matrix(transform):
    """geometry_msgs/Transform as a 4x4 matrix."""
    t, q = transform.translation, transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    pose = np.eye(4)
    pose[:3, :3] = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                    [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                    [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
    pose[:3, 3] = [t.x, t.y, t.z]
    return pose


def pose_from(image_rgb, camera_matrix, world_from_camera, edge):
    """(x, y, z, yaw) of the block's centre in world, or None when the image shows no block."""
    detection = detect_block(image_rgb)
    if detection is None:
        return None
    found = block_pose(detection, camera_matrix, world_from_camera, top_z=edge)
    if found is None:
        return None
    x, y, yaw = found
    return x, y, edge / 2.0, yaw


def main(args=None):
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from rclpy.time import Time
    from sensor_msgs.msg import CameraInfo, Image
    from tf2_ros import Buffer, TransformListener

    class BlockDetector(Node):
        def __init__(self):
            super().__init__("block_detector")
            task = Path(get_package_share_directory("so101_mtc_description")) / "config" / "task.yaml"
            self.edge = yaml.safe_load(task.read_text())["block"]["size"]
            self.camera_matrix = None
            self.tf = Buffer()
            self.tf_listener = TransformListener(self.tf, self)
            self.create_subscription(CameraInfo, "/camera/camera_info", self.on_info, qos_profile_sensor_data)
            self.create_subscription(Image, "/camera/image_raw", self.on_image, qos_profile_sensor_data)
            latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.publisher = self.create_publisher(PoseStamped, "/block_pose", latched)

        def on_info(self, message):
            self.camera_matrix = np.array(message.k, dtype=float).reshape(3, 3)

        def on_image(self, message):
            if self.camera_matrix is None:
                return
            try:
                transform = self.tf.lookup_transform("world", message.header.frame_id, Time())
            except Exception as error:  # TF not there yet
                self.get_logger().warn(f"no camera pose: {error}", throttle_duration_sec=5.0)
                return
            found = pose_from(image_array(message), self.camera_matrix, matrix(transform.transform), self.edge)
            if found is None:
                self.get_logger().warn("no block in the image", throttle_duration_sec=5.0)
                return
            x, y, z, yaw = found
            pose = PoseStamped()
            pose.header.stamp, pose.header.frame_id = message.header.stamp, "world"
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = x, y, z
            pose.pose.orientation.z, pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            self.publisher.publish(pose)

    rclpy.init(args=args)
    node = BlockDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
