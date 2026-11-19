"""The installed pick and place launch file and the node's parameter files, without a simulator: the launch file loads
with its default arguments, and the parameter files hold every parameter the node reads. None has a default in the
code, so a missing one would stop the node only once it runs.

    colcon test --packages-select so101_mtc_task && colcon test-result --verbose
"""
import importlib.util
import re
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

SHARE = Path(get_package_share_directory("so101_mtc_task"))
SOURCES = Path(__file__).resolve().parents[1]  # the package's sources, next to this test


def load_launch():
    spec = importlib.util.spec_from_file_location("pick_place_launch", SHARE / "launch" / "pick_place.launch.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description()


def installed_parameters(file_name, prefix):
    return yaml.safe_load((SHARE / "config" / file_name).read_text())["/**"]["ros__parameters"][prefix]


def test_launch_loads_with_its_default_arguments_and_starts_the_node():
    launch = load_launch()  # MoveItConfigsBuilder expands the URDF and reads the MoveIt config here
    defaults = {argument.name: argument.default_value[0].text for argument in launch.get_launch_arguments()}
    assert defaults == {"execute": "true", "run_at_start": "true", "camera": "false"}
    nodes = [entity for entity in launch.entities if isinstance(entity, Node)]
    assert sorted(node.node_executable for node in nodes) == ["block_detector", "pick_place"]  # the detector on camera


def test_pick_place_yaml_holds_every_parameter_the_node_reads():
    source = (SOURCES / "src" / "pick_place_node.cpp").read_text()
    read = set(re.findall(r'parameter<.+?>\("(\w+)"\)', source))
    assert len(read) > 10
    assert read <= set(installed_parameters("pick_place.yaml", "pick_place")), read


def test_grasp_poses_yaml_holds_every_parameter_the_generator_reads():
    source = (SOURCES / "src" / "grasp_poses.cpp").read_text()
    read = set(re.findall(r'name\("(\w+)"\)', source))
    assert len(read) == 9
    assert read <= set(installed_parameters("grasp_poses.yaml", "grasp_poses")), read


def test_camera_true_reads_the_camera_and_starts_the_detector():
    from launch import LaunchContext
    launch = load_launch()
    node = next(entity for entity in launch.entities
                if isinstance(entity, Node) and entity.node_executable == "pick_place")
    # the parameters as launch_ros normalised them: names and values as tuples of substitutions
    source = next(value for parameters in node._Node__parameters if isinstance(parameters, dict)
                  for key, value in parameters.items()
                  if "".join(getattr(part, "text", "") for part in key) == "pick_place.block_source")
    detector = next(entity for entity in launch.entities
                    if isinstance(entity, Node) and entity.node_executable == "block_detector")
    for spelling, expected in (("true", "camera"), ("True", "camera"), ("1", "camera"), ("false", "true_pose")):
        context = LaunchContext()
        context.launch_configurations["camera"] = spelling
        assert "".join(part.perform(context) for part in source) == expected, spelling
        assert detector.condition.evaluate(context) == (expected == "camera"), spelling
