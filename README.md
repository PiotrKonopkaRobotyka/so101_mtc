# so101_mtc

Pick and place of a block with the simulated SO-101 arm by classical planning: a fixed camera finds the block with
OpenCV, MoveIt 2 and MoveIt Task Constructor (MTC) plan the grasp and the place, ros2_control executes them.

It is the second project of a series that solves the same task with different methods and compares them on the same
metrics (success rate over 50 episodes, time per pick and place): imitation learning (webcam teleoperation and ACT,
`so101_lerobot`), classical planning (this repository) and reinforcement learning (planned).

## Status

M1 One-command sim bringup, in progress: the robot description, the MuJoCo scene and the MoveIt config are built and
tested (`so101_mtc_description`, `so101_mtc_moveit_config`); the bringup launch comes next. The scope and the exit
criteria are fixed in [ADR 0001](docs/decisions/0001-project_scope_and_exit_criteria.md), the planned nodes, topics
and TF frames in [docs/architecture.md](docs/architecture.md). The stack is fixed in
[ADR 0002](docs/decisions/0002-simulator_mtc_build_and_robot_model.md): MuJoCo through `mujoco_ros2_control`, MTC
from apt on `/opt/ros/jazzy`, the official SO-101 model. The grasp poses and the IK for the arm's 5 joints are fixed
in [ADR 0003](docs/decisions/0003-ik_and_grasp_poses.md): top-down near the base, tilted further out, exactly
reachable poses for MoveIt's default KDL.

## Packages

- `so101_mtc_perception/` — block detection and camera geometry; plain Python without ROS, tested in CI.
- `so101_mtc_description/` — the official SO-101 with `world`, the table, the TCP and the fixed camera (URDF), the
  MuJoCo scene with the block, and the task's parameters; see
  [Robot description and scene](#robot-description-and-scene).
- `so101_mtc_moveit_config/` — our MoveIt 2 config: the SRDF with the end effector at the TCP, KDL, joint limits,
  OMPL, the ros2_control and MoveIt controllers, and `move_group` with MTC's capability; see
  [MoveIt config](#moveit-config).
- Planned: `so101_mtc_bringup` (one launch for everything), `so101_mtc_task` (the MTC pick and place task in C++),
  `so101_mtc_perception_ros` (a thin node: image → block pose).

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
upgrade can mix incompatible binaries. The apt line stays until the bringup declares the MuJoCo bridge too (M1); the
MoveIt config already declares the MoveIt and MTC packages it uses.

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

## MoveIt config

`so101_mtc_moveit_config` is our own MoveIt 2 config for the official SO-101, written by hand (not with the MoveIt
Setup Assistant):

- `config/so101_mtc.srdf` — the groups `arm` (the chain `base_link` → `tcp`) and `gripper` (the moving jaw), the end
  effector at `tcp`, the named states `home` (the start pose), `open` (1.2 rad) and `closed` (0.0 rad, past the
  block), and the link pairs MoveIt does not check (see [Collision pairs](#collision-pairs)).
- `config/kinematics.yaml` — KDL on the full pose (ADR 0003). `config/joint_limits.yaml` — velocity and acceleration
  limits (the URDF's are placeholders). The planner is OMPL with MoveIt's default settings.
- `config/ros2_controllers.yaml` — the ros2_control controllers: `joint_state_broadcaster`, `arm_controller` (joint
  trajectories) and `gripper_controller`, a parallel gripper action controller. It closes toward `closed` and reports
  success once the jaw stops on the block (`allow_stalling`, ADR 0002); the servo limits the force (2.94 N·m in
  MuJoCo), and on the 25 mm block the jaw rests at 0.065 rad.
- `config/moveit_controllers.yaml` — MoveIt → those controllers (`FollowJointTrajectory`, `ParallelGripperCommand`),
  with a start tolerance of 0.2 rad for the gripper: a planned opening starts from `closed` while the jaw rests on
  the block.
- `.setup_assistant` — tells `MoveItConfigsBuilder` where the URDF (`so101_mtc_description`) and the SRDF are, so that
  every launch file builds the same config:
  `MoveItConfigsBuilder("so101_mtc", package_name="so101_mtc_moveit_config")`.
- `launch/move_group.launch.py` — `move_group` with MTC's `ExecuteTaskSolutionCapability`;
  `launch/moveit_rviz.launch.py` — RViz with the MotionPlanning display, the MTC panel and the camera image.

The controllers YAML lives here, next to `moveit_controllers.yaml`, because the two must name the same controllers
and joints (the test checks it); the bringup starts the simulator with it. `move_group` executes only with the
controllers running, so it is started by the bringup (M1):

```bash
ros2 launch so101_mtc_moveit_config move_group.launch.py
ros2 launch so101_mtc_moveit_config moveit_rviz.launch.py
```

### Collision pairs

MoveIt's `collisions_updater` (the Setup Assistant's collision sampling, from the command line) finds the link pairs
MoveIt need not check, over 10000 random states. It rewrites the whole SRDF without its comments, so copy only its
`disable_collisions` lines into `config/so101_mtc.srdf`:

```bash
xacro $(ros2 pkg prefix --share so101_mtc_description)/urdf/so101_mtc.urdf.xacro > /tmp/so101_mtc.urdf
ros2 run moveit_setup_assistant collisions_updater --urdf /tmp/so101_mtc.urdf \
  --srdf src/so101_mtc/so101_mtc_moveit_config/config/so101_mtc.srdf --output /tmp/so101_mtc.srdf \
  --default --always --trials 10000
```

It found 7 adjacent pairs and 6 pairs that never collided; none collides always or at the zero pose. A sample proves
nothing, so the 6 "Never" pairs were checked on a dense grid of the joints between the two links (MoveIt's distance,
through `moveit_py`): the closest stays 8.9 mm apart within the joint limits. `--trials 0` does not skip the sampling;
it marks pairs "Never" without testing them, some of which do collide.

## Tests

The ROS-free core needs only numpy and pytest (on Ubuntu 24.04: `sudo apt install python3-numpy python3-pytest`):

```bash
python3 -m pytest so101_mtc_perception
```

GitHub Actions runs the same tests, with the same numpy and pytest versions, on every push to `main` and every pull
request ([tests.yml](.github/workflows/tests.yml)).

The ROS packages are tested from the workspace, after the build (ROS 2 Jazzy; not in CI before M5):

```bash
colcon test --packages-select so101_mtc_description so101_mtc_moveit_config && colcon test-result --verbose
```

The test expands the xacro, lets MuJoCo's `compile` load the scene, and checks that the two agree: the joints and
their limits, the poses of all links over random joint angles, the camera and the table; that `tcp` is where ADR 0003
puts it; and that the table and the camera's view hold the whole task area. The MoveIt config's test loads the robot
into MoveIt through `moveit_py`, without a simulator: the groups and the end effector, the named states (`home` is
the start pose), no collision in them but collisions where the arm reaches through the table, and controllers that
agree with MoveIt's mapping.

## License and third-party code

Apache-2.0 ([LICENSE](LICENSE)). Third-party repositories are pinned in [so101_mtc.repos](so101_mtc.repos) and
never modified: the official SO-101 model (TheRobotStudio SO-ARM100, `Simulation/SO101`, Apache-2.0) and its MuJoCo
Menagerie version (`robotstudio_so101`, Apache-2.0). Code adapted from Automatic Addison's myCobot 280 tutorial
(`mycobot_mtc_pick_place_demo`, BSD-3-Clause, Addison Sears-Collins) says "Based on …" in its header and keeps the
BSD-3-Clause notice.
