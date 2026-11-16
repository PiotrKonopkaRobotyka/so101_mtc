# so101_mtc

Pick and place of a block with the simulated SO-101 arm by classical planning: a fixed camera finds the block with
OpenCV, MoveIt 2 and MoveIt Task Constructor (MTC) plan the grasp and the place, ros2_control executes them.

It is the second project of a series that solves the same task with different methods and compares them on the same
metrics (success rate over 50 episodes, time per pick and place): imitation learning (webcam teleoperation and ACT,
`so101_lerobot`), classical planning (this repository) and reinforcement learning (planned).

## Status

M1 One-command sim bringup, in progress: the robot description and the MuJoCo scene are built and tested
(`so101_mtc_description`); the MoveIt config and the bringup launch come next. The scope and the exit criteria are fixed in
[ADR 0001](docs/decisions/0001-project_scope_and_exit_criteria.md), the planned nodes, topics and TF frames in
[docs/architecture.md](docs/architecture.md). The stack is fixed in
[ADR 0002](docs/decisions/0002-simulator_mtc_build_and_robot_model.md): MuJoCo through `mujoco_ros2_control`, MTC
from apt on `/opt/ros/jazzy`, the official SO-101 model. The grasp poses and the IK for the arm's 5 joints are fixed
in [ADR 0003](docs/decisions/0003-ik_and_grasp_poses.md): top-down near the base, tilted further out, exactly
reachable poses for MoveIt's default KDL.

## Packages

- `so101_mtc_perception/` — block detection and camera geometry; plain Python without ROS, tested in CI.
- `so101_mtc_description/` — the official SO-101 with `world`, the table, the TCP and the fixed camera (URDF), the
  MuJoCo scene with the block, and the task's parameters; see
  [Robot description and scene](#robot-description-and-scene).
- Planned: `so101_mtc_moveit_config` (end effector, TCP, IK for 5 joints), `so101_mtc_bringup` (one launch for everything),
  `so101_mtc_task` (the MTC pick and place task in C++), `so101_mtc_perception_ros` (a thin node: image → block pose).

## Workspace setup

Ubuntu 24.04 with ROS 2 Jazzy from apt; the workspace builds on `/opt/ros/jazzy` only (no MoveIt from source):

```bash
mkdir -p so101_mtc_ws/src && cd so101_mtc_ws/src
git clone git@github.com:PiotrKonopkaRobotyka/so101_mtc.git
cd .. && python3 src/so101_mtc/tools/fetch_models.py   # pinned SO-101 models into src/ (~55 MB on disk)
sudo apt install ros-jazzy-moveit ros-jazzy-moveit-task-constructor-core \
  ros-jazzy-moveit-task-constructor-capabilities ros-jazzy-moveit-task-constructor-visualization \
  ros-jazzy-mujoco-ros2-control
rosdep install --from-paths src/so101_mtc --ignore-src -y   # what the packages of this repository declare
colcon build --symlink-install
source install/setup.bash
```

The build needs the fetched models: `so101_mtc_description` installs them from `src/SO-ARM100` and
`src/mujoco_menagerie` and stops with a message when they are missing.

Upgrade ROS packages a whole sync at a time (`sudo apt upgrade`): their dependencies carry no versions, so a partial
upgrade can mix incompatible binaries. The apt line stays until the MoveIt config and the bringup declare MoveIt, MTC
and the MuJoCo bridge themselves (M1).

## Robot description and scene

`so101_mtc_description` holds what ROS and MuJoCo need to know about the robot and its surroundings:

- `urdf/so101_mtc.urdf.xacro` — the official URDF `so101_new_calib`, included unmodified, plus our frames: `world`,
  the table, `tcp` (the centre of a grasped 25 mm block, ADR 0003; z out of the jaws, like the upstream
  `gripper_frame_link`), `camera_link` and `camera_optical_frame` (REP 103), and the `ros2_control` tag for
  `mujoco_ros2_control`, with the joints starting at the home pose.
- `mujoco/scene.xml.xacro` — the MuJoCo scene: the Menagerie SO-101 attached unmodified, the table, the block
  (a free body) and the fixed camera. MJCF has no variables, so the build expands this template with the block of
  `config/task.yaml` and installs the result as `mujoco/scene.xml`.
- `config/task.yaml` — the task shared with project 1 (its ADR 0009): the 25 mm, 10 g block, the area the block and
  the target are drawn from (a ring sector around the shoulder_pan axis, 0.12–0.34 m, ±60°) and the target disc
  (5 cm radius).
- `upstream/` (only in the installed package) — the fetched models, so that every file is found by package name:
  the URDF's meshes as `package://so101_mtc_description/upstream/SO101/assets/…` (the build writes a copy of the URDF
  with these paths; the fetched files stay untouched) and the Menagerie model next to the scene.

Show the robot in RViz, with sliders for its joints (no simulator):

```bash
ros2 launch so101_mtc_description display.launch.py
```

Build again after editing `scene.xml.xacro` or `task.yaml`; the other files are linked into `install/` by
`--symlink-install` and take effect at once. MuJoCo loads the installed scene,
`$(ros2 pkg prefix --share so101_mtc_description)/mujoco/scene.xml`.

## Tests

The ROS-free core needs only numpy and pytest (on Ubuntu 24.04: `sudo apt install python3-numpy python3-pytest`):

```bash
python3 -m pytest so101_mtc_perception
```

GitHub Actions runs the same tests, with the same numpy and pytest versions, on every push to `main` and every pull
request ([tests.yml](.github/workflows/tests.yml)).

The description is tested from the workspace, after the build (ROS 2 Jazzy; not in CI before M5):

```bash
colcon test --packages-select so101_mtc_description && colcon test-result --verbose
```

The test expands the xacro, lets MuJoCo's `compile` load the scene, and checks that the two agree: the joints and
their limits, the poses of all links over random joint angles, the camera and the table; that `tcp` is where ADR 0003
puts it; and that the table and the camera's view hold the whole task area.

## License and third-party code

Apache-2.0 ([LICENSE](LICENSE)). Third-party repositories are pinned in [so101_mtc.repos](so101_mtc.repos) and
never modified: the official SO-101 model (TheRobotStudio SO-ARM100, `Simulation/SO101`, Apache-2.0) and its MuJoCo
Menagerie version (`robotstudio_so101`, Apache-2.0). Code adapted from Automatic Addison's myCobot 280 tutorial
(`mycobot_mtc_pick_place_demo`, BSD-3-Clause, Addison Sears-Collins) says "Based on …" in its header and keeps the
BSD-3-Clause notice.
