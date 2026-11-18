"""The installed so101_mtc_bringup: the launch file loads, and the MuJoCo plugins name what exists.

    colcon test --packages-select so101_mtc_bringup && colcon test-result --verbose
"""
import importlib.util
import xml.etree.ElementTree as ET
from pathlib import Path

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch.actions import RegisterEventHandler

SHARE = Path(get_package_share_directory("so101_mtc_bringup"))
DESCRIPTION = Path(get_package_share_directory("so101_mtc_description"))


def load_launch():
    spec = importlib.util.spec_from_file_location("sim_launch", SHARE / "launch" / "sim.launch.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description()


def test_launch_loads_with_its_arguments_and_events():
    launch = load_launch()
    assert {a.name for a in launch.get_launch_arguments()} == {"headless", "rviz", "sim_speed_factor"}
    # spawners chained by events: arm and gripper after the broadcaster, MoveIt and RViz after them
    assert sum(isinstance(e, RegisterEventHandler) for e in launch.entities) == 2


def test_plugins_name_the_camera_frame_and_the_block():
    plugins = yaml.safe_load((SHARE / "config" / "mujoco_plugins.yaml").read_text())
    plugins = plugins["/**"]["ros__parameters"]["mujoco_plugins"]
    urdf = ET.fromstring(xacro.process_file(str(DESCRIPTION / "urdf" / "so101_mtc.urdf.xacro")).toxml())
    scene = ET.parse(DESCRIPTION / "mujoco" / "scene.xml").getroot()
    links = {link.get("name") for link in urdf.findall("link")}
    cameras = {c.get("name") for c in scene.iter("camera")}
    bodies = {b.get("name") for b in scene.iter("body")}
    camera_plugin = plugins["mujoco_camera_plugin"]
    for name, settings in camera_plugin.items():
        if isinstance(settings, dict):
            assert name in cameras, name
            assert settings["frame_name"] in links, settings["frame_name"]
    assert set(plugins["free_joint_state_publisher"]["body_names"]) <= bodies
