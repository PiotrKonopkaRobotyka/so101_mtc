"""The installed so101_mtc_description: the robot (xacro → URDF) and the MuJoCo scene, and that they agree.

The scene is checked as MuJoCo loads it: MuJoCo's `compile` tool (mujoco_vendor) reads the installed scene.xml
(built from mujoco/scene.xml.xacro and config/task.yaml), attaches the Menagerie robot and saves the whole model as
one MJCF, which the tests read.

    colcon test --packages-select so101_mtc_description && colcon test-result --verbose
"""
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest
import xacro
import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory

PACKAGE = "so101_mtc_description"
SHARE = Path(get_package_share_directory(PACKAGE))
JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
TCP_ADR_0003 = [0.0039, -0.0002, -0.0958]  # m in gripper_link: the centre of a grasped 25 mm block
# URDF link → MJCF body
BODIES = {"base_link": "base", "shoulder_link": "shoulder", "upper_arm_link": "upper_arm",
          "lower_arm_link": "lower_arm", "wrist_link": "wrist", "gripper_link": "gripper",
          "moving_jaw_so101_v1_link": "moving_jaw_so101_v1"}


def floats(text, default=(0.0, 0.0, 0.0)):
    return np.array([float(v) for v in text.split()] if text else default, dtype=float)


def rpy_matrix(roll, pitch, yaw):
    """URDF convention: rotations about the fixed x, y and z axes, in this order."""
    cr, sr, cp, sp, cy, sy = np.cos(roll), np.sin(roll), np.cos(pitch), np.sin(pitch), np.cos(yaw), np.sin(yaw)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def quat_matrix(wxyz):
    """MuJoCo convention: w, x, y, z."""
    w, x, y, z = wxyz / np.linalg.norm(wxyz)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def axis_rotation(axis, angle):
    """Rodrigues' formula."""
    x, y, z = axis / np.linalg.norm(axis)
    K = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * (K @ K)


def origin(element):
    """The <origin> of a URDF joint, visual or collision as a 4x4 transform."""
    o = element.find("origin")
    T = np.eye(4)
    if o is not None:
        T[:3, :3] = rpy_matrix(*floats(o.get("rpy")))
        T[:3, 3] = floats(o.get("xyz"))
    return T


def joint_to(urdf, link):
    return next(j for j in urdf.findall("joint") if j.find("child").get("link") == link)


def link_in_world(urdf, link, q=None):
    """The pose of a URDF link in world at the joint angles q (zero where not given)."""
    T = np.eye(4)
    while link != "world":
        joint = joint_to(urdf, link)
        T_joint = origin(joint)
        if joint.get("type") == "revolute":
            angle = (q or {}).get(joint.get("name"), 0.0)
            T_joint[:3, :3] = T_joint[:3, :3] @ axis_rotation(floats(joint.find("axis").get("xyz")), angle)
        T = T_joint @ T
        link = joint.find("parent").get("link")
    return T


def body_in_world(mjcf, name, q):
    """The pose of an MJCF body in the world at the joint angles q."""
    parents = {child: parent for parent in mjcf.iter() for child in parent}
    body = named(mjcf, "body", name)
    T = np.eye(4)
    while body.tag == "body":
        T_body = np.eye(4)
        T_body[:3, :3] = quat_matrix(floats(body.get("quat"), (1, 0, 0, 0)))
        T_body[:3, 3] = floats(body.get("pos"))
        joint = body.find("joint")
        if joint is not None:
            assert not np.any(floats(joint.get("pos"))), "the joint is expected at the body's origin"
            T_body[:3, :3] = T_body[:3, :3] @ axis_rotation(floats(joint.get("axis"), (0, 0, 1)), q[joint.get("name")])
        T = T_body @ T
        body = parents[body]
    return T


def area_outline(task):
    """Points on the inner and the outer arc of the task area, in world (x, y)."""
    area = task["area"]
    azimuths = np.radians(np.linspace(*area["azimuth"], 61))
    return [area["centre"] + r * np.array([np.cos(a), np.sin(a)]) for r in area["radius"] for a in azimuths]


def urdf_limits(urdf):
    return {j.get("name"): floats(f"{j.find('limit').get('lower')} {j.find('limit').get('upper')}")
            for j in urdf.findall("joint") if j.get("type") == "revolute"}


def named(root, tag, name):
    return next(e for e in root.iter(tag) if e.get("name") == name)


@pytest.fixture(scope="module")
def urdf():
    return ET.fromstring(xacro.process_file(str(SHARE / "urdf" / "so101_mtc.urdf.xacro")).toxml())


@pytest.fixture(scope="module")
def mjcf(tmp_path_factory):
    out = tmp_path_factory.mktemp("mujoco") / "scene_compiled.xml"
    compile_tool = Path(get_package_prefix("mujoco_vendor")) / "opt" / "mujoco_vendor" / "bin" / "compile"
    result = subprocess.run([str(compile_tool), str(SHARE / "mujoco" / "scene.xml"), str(out)],
                            capture_output=True, text=True, cwd=out.parent, timeout=60)
    assert result.returncode == 0 and out.is_file(), result.stdout + result.stderr
    return ET.parse(out).getroot()


@pytest.fixture(scope="module")
def task():
    return yaml.safe_load((SHARE / "config" / "task.yaml").read_text())


def test_xacro_expands_to_one_tree_under_world(urdf):
    links = {link.get("name") for link in urdf.findall("link")}
    children = [j.find("child").get("link") for j in urdf.findall("joint")]
    parents = {j.find("parent").get("link") for j in urdf.findall("joint")}
    assert {"world", "table", "base_link", "gripper_link", "tcp", "camera_link", "camera_optical_frame"} <= links
    assert sorted(children) == sorted(links - {"world"})  # every link but world has exactly one parent
    assert parents <= links


def test_meshes_resolve_in_the_package(urdf):
    prefix = f"package://{PACKAGE}/"
    meshes = [mesh.get("filename") for mesh in urdf.iter("mesh")]
    assert meshes
    for uri in meshes:
        assert uri.startswith(prefix), uri
        assert (SHARE / uri.removeprefix(prefix)).is_file(), uri


def test_mujoco_loads_the_scene(mjcf):
    for tag, name in [("body", "base"), ("body", "gripper"), ("geom", "table"), ("body", "block"),
                      ("camera", "camera"), ("light", "lamp")]:
        assert named(mjcf, tag, name) is not None


def test_joints_and_limits_match_the_scene(urdf, mjcf):
    limits = urdf_limits(urdf)
    hinges = {j.get("name"): floats(j.get("range")) for j in mjcf.find("worldbody").iter("joint")
              if j.get("type", "hinge") == "hinge"}
    actuators = {a.get("joint"): floats(a.get("ctrlrange")) for a in mjcf.find("actuator")}
    assert sorted(limits) == sorted(hinges) == sorted(actuators) == sorted(JOINTS)
    for name in JOINTS:
        np.testing.assert_allclose(hinges[name], limits[name], atol=1e-4, err_msg=name)
        np.testing.assert_allclose(actuators[name], limits[name], atol=1e-4, err_msg=name)


def test_links_move_as_the_bodies_of_the_scene(urdf, mjcf):
    """MoveIt plans with the URDF and MuJoCo moves the MJCF: the same joint angles must give the same poses."""
    limits = urdf_limits(urdf)
    rng = np.random.default_rng(0)
    configurations = [{name: 0.0 for name in JOINTS}]
    configurations += [{name: rng.uniform(*limits[name]) for name in JOINTS} for _ in range(50)]
    for q in configurations:
        for link, body in BODIES.items():
            T_link, T_body = link_in_world(urdf, link, q), body_in_world(mjcf, body, q)
            np.testing.assert_allclose(T_link[:3, 3], T_body[:3, 3], atol=5e-5, err_msg=link)  # 0.05 mm
            np.testing.assert_allclose(T_link[:3, :3], T_body[:3, :3], atol=2e-4, err_msg=link)  # about 0.01 deg


def test_scene_keeps_the_option_of_the_robot_model(mjcf):
    """<attach> drops the attached model's <option> (ADR 0002): the scene copies it."""
    robot = ET.parse(SHARE / "upstream" / "robotstudio_so101" / "so101.xml").getroot().find("option").attrib
    scene = mjcf.find("option").attrib
    for key, value in robot.items():
        assert key in scene, key
        try:
            assert float(scene[key]) == pytest.approx(float(value)), key
        except ValueError:
            assert scene[key] == value, key


def test_tcp_is_where_adr_0003_puts_it(urdf):
    joint = joint_to(urdf, "tcp")
    assert joint.find("parent").get("link") == "gripper_link"
    np.testing.assert_allclose(origin(joint)[:3, 3], TCP_ADR_0003, atol=1e-9)
    # the axes of the upstream gripper_frame_link: z out of the jaws, along the approach
    np.testing.assert_allclose(origin(joint)[:3, :3], origin(joint_to(urdf, "gripper_frame_link"))[:3, :3], atol=1e-5)


def test_camera_matches_the_scene(urdf, mjcf):
    camera = named(mjcf, "camera", "camera")
    optical = link_in_world(urdf, "camera_optical_frame")
    # a MuJoCo camera looks along its -z with y up in the image; the optical frame looks along z with y down
    expected = quat_matrix(floats(camera.get("quat"), (1, 0, 0, 0))) @ np.diag([1.0, -1.0, -1.0])
    np.testing.assert_allclose(optical[:3, :3], expected, atol=1e-6)
    np.testing.assert_allclose(optical[:3, 3], floats(camera.get("pos")), atol=1e-9)
    # REP 103: camera_link looks along x with z up in the image, so optical z = x, x = -y, y = -z of camera_link
    in_link = link_in_world(urdf, "camera_link")[:3, :3].T @ optical[:3, :3]
    np.testing.assert_allclose(in_link, [[0, 0, 1], [-1, 0, 0], [0, -1, 0]], atol=1e-9)


def test_camera_sees_the_area_and_the_target(urdf, mjcf, task):
    camera = named(mjcf, "camera", "camera")
    width, height = floats(camera.get("resolution"))
    half_height = np.tan(np.radians(float(camera.get("fovy"))) / 2)  # of the image at 1 m, as is half_width
    half_width = half_height * width / height
    world_to_optical = np.linalg.inv(link_in_world(urdf, "camera_optical_frame"))
    margin = task["target"]["radius"]
    for point in area_outline(task):
        x, y, z = (world_to_optical @ [*point, 0.0, 1.0])[:3]
        assert z > 0 and (abs(x) + margin) / z <= half_width and (abs(y) + margin) / z <= half_height, point


def test_lights_do_not_overexpose_the_table(mjcf):
    """The camera looks along the headlight and the lamp at a matte table: light above 1 would clip its colours."""
    headlight = mjcf.find("visual/headlight")
    light = floats(headlight.get("ambient")) + floats(headlight.get("diffuse"))
    for lamp in mjcf.find("worldbody").iter("light"):
        light = light + floats(lamp.get("ambient")) + floats(lamp.get("diffuse"), (0.7, 0.7, 0.7))
    assert np.all(light <= 1.0), light


def test_table_matches_the_scene(urdf, mjcf):
    link = named(urdf, "link", "table")
    size = floats(link.find("collision/geometry/box").get("size"))
    centre = (link_in_world(urdf, "table") @ origin(link.find("collision")))[:3, 3]
    table = named(mjcf, "geom", "table")
    np.testing.assert_allclose(size, 2 * floats(table.get("size")), atol=1e-9)
    np.testing.assert_allclose(centre, floats(table.get("pos")), atol=1e-9)
    assert centre[2] + size[2] / 2 == pytest.approx(0.0)  # the top at z = 0


def test_block_is_the_task_block_resting_in_the_area(urdf, mjcf, task):
    body = named(mjcf, "body", "block")
    assert body.find("joint").get("type") == "free"
    geom = body.find("geom")
    assert geom.get("type") == "box"
    np.testing.assert_allclose(floats(geom.get("size")), [task["block"]["size"] / 2] * 3, atol=1e-9)
    assert float(geom.get("mass")) == pytest.approx(task["block"]["mass"])
    position = floats(body.get("pos"))
    assert position[2] == pytest.approx(task["block"]["size"] / 2)  # on the table top
    # the area's centre is the shoulder_pan axis, and the block starts inside the area
    area = task["area"]
    np.testing.assert_allclose(area["centre"], link_in_world(urdf, "shoulder_link")[:2, 3], atol=1e-4)
    offset = position[:2] - area["centre"]
    assert area["radius"][0] <= np.hypot(*offset) <= area["radius"][1]
    assert area["azimuth"][0] <= np.degrees(np.arctan2(offset[1], offset[0])) <= area["azimuth"][1]


def test_table_holds_the_area_and_the_target(mjcf, task):
    table = named(mjcf, "geom", "table")
    centre, half = floats(table.get("pos"))[:2], floats(table.get("size"))[:2]
    for point in area_outline(task):  # a target disc centred anywhere in the area lies on the table
        assert np.all(np.abs(point - centre) + task["target"]["radius"] <= half), point


def test_ros2_control_runs_the_scene_from_the_home_pose(urdf, mjcf):
    control = urdf.find("ros2_control")
    assert control.find("hardware/plugin").text.strip() == "mujoco_ros2_control/MujocoSystemInterface"
    parameters = {p.get("name"): p.text.strip() for p in control.findall("hardware/param")}
    assert Path(parameters["mujoco_model"]) == SHARE / "mujoco" / "scene.xml"
    assert [j.get("name") for j in control.findall("joint")] == [a.get("joint") for a in mjcf.find("actuator")]
    limits = urdf_limits(urdf)
    for joint in control.findall("joint"):
        assert joint.find("command_interface").get("name") == "position"
        initial = float(joint.find("state_interface[@name='position']/param[@name='initial_value']").text)
        assert limits[joint.get("name")][0] <= initial <= limits[joint.get("name")][1], joint.get("name")
