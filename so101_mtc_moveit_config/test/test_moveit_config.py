"""The installed so101_mtc_moveit_config, checked without a simulator or move_group.

The robot model and the planning scene come from moveit_py, MoveIt's Python bindings, loaded from the expanded URDF
and our SRDF, so the collision checks are MoveIt's own.

    colcon test --packages-select so101_mtc_moveit_config && colcon test-result --verbose
"""
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from moveit.core import collision_detection
from moveit.core.planning_scene import PlanningScene
from moveit.core.robot_model import RobotModel
from moveit.core.robot_state import RobotState

SHARE = Path(get_package_share_directory("so101_mtc_moveit_config"))
DESCRIPTION = Path(get_package_share_directory("so101_mtc_description"))
ARM = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]


def load_yaml(name):
    return yaml.safe_load((SHARE / "config" / name).read_text())


@pytest.fixture(scope="module")
def urdf_path(tmp_path_factory):
    path = tmp_path_factory.mktemp("urdf") / "so101_mtc.urdf"
    path.write_text(xacro.process_file(str(DESCRIPTION / "urdf" / "so101_mtc.urdf.xacro")).toxml())
    return path


@pytest.fixture(scope="module")
def urdf(urdf_path):
    return ET.parse(urdf_path).getroot()


@pytest.fixture(scope="module")
def srdf():
    return ET.parse(SHARE / "config" / "so101_mtc.srdf").getroot()


@pytest.fixture(scope="module")
def model(urdf_path):
    return RobotModel(urdf_xml_path=str(urdf_path), srdf_xml_path=str(SHARE / "config" / "so101_mtc.srdf"))


def named_states(srdf):
    return {(s.get("group"), s.get("name")): {j.get("name"): float(j.get("value")) for j in s.findall("joint")}
            for s in srdf.findall("group_state")}


def in_collision(model, positions):
    """MoveIt's collision check (self and the table, which is a link of the URDF) at the given joint positions."""
    scene = PlanningScene(model)
    state = RobotState(model)
    state.set_to_default_values()
    state.joint_positions = positions
    state.update()
    result = collision_detection.CollisionResult()
    scene.check_self_collision(collision_detection.CollisionRequest(), result, state)
    return result.collision


def test_setup_assistant_file_names_the_urdf_and_the_srdf():
    config = yaml.safe_load((SHARE / ".setup_assistant").read_text())["moveit_setup_assistant_config"]
    assert (Path(get_package_share_directory(config["urdf"]["package"])) / config["urdf"]["relative_path"]).is_file()
    assert (SHARE / config["srdf"]["relative_path"]).is_file()


def test_groups_and_end_effector(model, srdf):
    assert model.get_joint_model_group("arm").active_joint_model_names == ARM
    assert model.get_joint_model_group("gripper").active_joint_model_names == ["gripper"]
    chain = srdf.find("group[@name='arm']/chain")
    assert (chain.get("base_link"), chain.get("tip_link")) == ("base_link", "tcp")
    end_effector = srdf.find("end_effector")
    assert (end_effector.get("parent_link"), end_effector.get("group"), end_effector.get("parent_group")) == (
        "tcp", "gripper", "arm")


def test_named_states_within_limits_and_home_is_the_start_pose(urdf, srdf):
    limits = {j.get("name"): (float(j.find("limit").get("lower")), float(j.find("limit").get("upper")))
              for j in urdf.findall("joint") if j.get("type") == "revolute"}
    states = named_states(srdf)
    assert set(states) == {("arm", "home"), ("gripper", "open"), ("gripper", "closed")}
    for (group, name), joints in states.items():
        for joint, value in joints.items():
            assert limits[joint][0] <= value <= limits[joint][1], (name, joint)
    # the robot starts at home in the simulator: the initial values of the URDF's ros2_control tag
    initial = {j.get("name"): float(j.find("state_interface[@name='position']/param[@name='initial_value']").text)
               for j in urdf.find("ros2_control").findall("joint")}
    assert states[("arm", "home")] == {name: initial[name] for name in ARM}
    # the gripper closes past the jaws' contact with the 25 mm block (about 0.10 rad, ADR 0003) and opens past it
    assert states[("gripper", "closed")]["gripper"] < 0.10 < states[("gripper", "open")]["gripper"]


def test_named_states_are_collision_free(model, srdf):
    states = named_states(srdf)
    home = states[("arm", "home")]
    for gripper in ("open", "closed"):
        assert not in_collision(model, {**home, **states[("gripper", gripper)]}), gripper


def test_collisions_are_checked(model):
    """Not every check passes: the arm reaching down through the table (the lower arm and the wrist hit it) and
    the arm folded back (the gripper hits the base and the shoulder) collide."""
    assert in_collision(model, {"shoulder_pan": 0.0, "shoulder_lift": 1.6, "elbow_flex": 0.0, "wrist_flex": 0.0,
                                "wrist_roll": 0.0, "gripper": 0.0})
    assert in_collision(model, {"shoulder_pan": 0.0, "shoulder_lift": -1.7, "elbow_flex": 1.69, "wrist_flex": 1.6,
                                "wrist_roll": 0.0, "gripper": 0.0})


def test_controllers_agree_with_moveit_and_the_groups():
    ros2_control = load_yaml("ros2_controllers.yaml")
    types = {name: entry["type"] for name, entry in ros2_control["controller_manager"]["ros__parameters"].items()
             if isinstance(entry, dict)}
    moveit = load_yaml("moveit_controllers.yaml")["moveit_simple_controller_manager"]
    assert set(moveit["controller_names"]) | {"joint_state_broadcaster"} == set(types)
    # the action each ros2_control controller serves → the MoveIt handle that sends to it
    expected = {"joint_trajectory_controller/JointTrajectoryController": ("FollowJointTrajectory",
                                                                         "follow_joint_trajectory"),
                "parallel_gripper_action_controller/GripperActionController": ("ParallelGripperCommand", "gripper_cmd")}
    for name in moveit["controller_names"]:
        assert (moveit[name]["type"], moveit[name]["action_ns"]) == expected[types[name]], name
    assert moveit["arm_controller"]["joints"] == ros2_control["arm_controller"]["ros__parameters"]["joints"] == ARM
    assert moveit["gripper_controller"]["joints"] == [ros2_control["gripper_controller"]["ros__parameters"]["joint"]]
    assert ros2_control["gripper_controller"]["ros__parameters"]["allow_stalling"] is True


def test_kinematics_and_limits():
    assert load_yaml("kinematics.yaml")["arm"]["kinematics_solver"] == "kdl_kinematics_plugin/KDLKinematicsPlugin"
    limits = load_yaml("joint_limits.yaml")["joint_limits"]
    assert set(limits) == set(ARM) | {"gripper"}
    for joint, entry in limits.items():
        assert entry["has_velocity_limits"] and entry["max_velocity"] > 0, joint
        assert entry["has_acceleration_limits"] and entry["max_acceleration"] > 0, joint
