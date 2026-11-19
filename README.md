# so101_mtc

Pick and place of a block with the simulated SO-101 arm by classical planning: a fixed camera finds the block with
OpenCV, MoveIt 2 and MoveIt Task Constructor (MTC) plan the grasp and the place, ros2_control executes them.

It is the second project of a series that solves the same task with different methods and compares them on the same
metrics (success rate over 50 episodes, time per pick and place): imitation learning (webcam teleoperation and ACT,
`so101_lerobot`), classical planning (this repository) and reinforcement learning (planned).

## Status

M1 One-command sim bringup done: one command starts MuJoCo with the robot, the table, the block and the camera, the
controllers, MoveIt with MTC's capability and RViz ([Run the simulation](#run-the-simulation)). M2 MTC pick and place
from the block's true pose done: the MTC task picks the block and places it on a target in MuJoCo
([Pick and place with MTC](#pick-and-place-with-mtc)), and 20 of 20 trials pass project 1's success check
([Trials](#trials)). M3 (the block's pose from the camera) comes next. The scope and the exit
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
- `so101_mtc_bringup/` — one launch for everything: MuJoCo, the controllers in order, MoveIt and RViz.
- `so101_mtc_task/` — the pick and place task in C++: the generator of grasp and place poses that the 5-joint arm
  reaches exactly ([Grasp and place poses](#grasp-and-place-poses)) and the MTC node that picks the block and places
  it on a target ([Pick and place with MTC](#pick-and-place-with-mtc)).
- `so101_mtc_interfaces/` — the service `PickPlace`: one pick and place on request.
- `so101_mtc_eval/` — the trials without ROS: the draw of the block and the target, project 1's success check and
  the records with their summary; plain Python, tested in CI. `so101_mtc_eval_ros/` — the trial runner
  ([Trials](#trials)), which also installs `so101_mtc_eval` for the nodes.
- Planned: `so101_mtc_perception_ros` (a thin node: image → block pose).

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
upgrade can mix incompatible binaries. `rosdep` installs what the packages declare (MoveIt, MTC with its capability
and panel, the MuJoCo bridge, the controllers); the apt line installs the largest of them in one go.

## Run the simulation

```bash
ros2 launch so101_mtc_bringup sim.launch.py   # [headless:=true] [rviz:=false] [sim_speed_factor:=5.0]
```

The MuJoCo window shows the scene; RViz shows the robot, the MotionPlanning panel (drag the marker at the TCP, then
Plan & Execute), the MTC panel and the fixed camera's image. The launch starts things in order, by events rather
than sleeps: `robot_state_publisher` and MuJoCo's `ros2_control_node`, then `joint_state_broadcaster`, then the arm's
and the gripper's controllers, then `move_group` and RViz. A controller that fails to start stops the launch, and so
does closing the MuJoCo window.

- `headless:=true` runs MuJoCo without its window; `rviz:=false` leaves RViz out.
- `sim_speed_factor` sets the simulation's speed against real time (5.0: five times faster); below 0, real time.
- The block's true pose is on `/block/free_joint_states`; `/mujoco_ros2_control_node/set_free_joint_state` moves it,
  `/mujoco_ros2_control_node/reset_world` resets the scene (with the arm's controllers deactivated first).
- The camera publishes `/camera/image_raw`, `/camera/depth/image_raw` and `/camera/camera_info` in
  `camera_optical_frame` at 10 Hz of simulation time; Menagerie's wrist camera publishes under
  `/mujoco_camera_plugin/wrist_cam/`.

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
  MuJoCo), and on the 25 mm block the jaw rests at 0.065 rad. The arm's controller accepts a trajectory that ends at
  a tiny velocity, as MTC's time parameterisation leaves it (about 1e-3 rad/s); MoveIt's reference configs do the same.
- `config/moveit_controllers.yaml` — MoveIt → those controllers (`FollowJointTrajectory`, `ParallelGripperCommand`),
  with a start tolerance of 0.2 rad for the gripper: a planned opening starts from `closed` while the jaw rests on
  the block. MoveIt allows 3 s past a trajectory's planned end: the gripper's goal ends only once the jaw stops on the
  block, later when the block turns between the closing jaws.
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

## Grasp and place poses

The SO-101 has 5 arm joints. Its three pitch joints keep the `wrist_roll` axis in one vertical plane, which
`shoulder_pan` turns, so the arm reaches a gripper pose only when the roll axis lies in that plane. Grasp poses
sampled around the block (MTC's `GenerateGraspPose`) mostly miss it and have no IK solution. `so101_mtc_task`'s
library `grasp_poses` builds the poses the arm reaches exactly ([ADR 0003](docs/decisions/0003-ik_and_grasp_poses.md)),
which the MTC task takes through `GeneratePose` and `ComputeIK` with KDL:

- **Plane azimuth in closed form:** the plane must hold the roll axis, which lies 3.9 mm from the `tcp`, and the
  plane itself lies e = 0.18 mm from the `shoulder_pan` axis. For a point p of `gripper_link` that has to land on c,
  the azimuth θ of the plane follows from `r sin(φ − θ) = p_x cos β − p_y sin β + e`, where (r, φ) is c about the pan
  axis and β the jaw angle.
- **Families by the distance from the `shoulder_pan` axis:** top-down up to 0.23 m, with the jaws facing the block's
  sides (four directions); tilted 15° up to 0.29 m and 30° beyond, leaning away from the base, with the jaws
  horizontal (two sides).
- **Waypoints:** a grasp has a pre-grasp 5 cm back along the approach and a lift 5 cm up. A place has a pre-place 5 cm
  up and a retreat 5 cm back along the approach.
- **Jaw sides:** `wrist_roll` = β + 92.8° and stops at −157°/+163°, so a jaw direction in that gap uses the other
  jaw side; the generator leaves such sides out.
- **Approach gap:** the `tcp` is the centre of a block pressed onto the fixed jaw, so the descent stops with the `tcp`
  3.6 mm towards the fixed jaw; otherwise the fixed jaw drags the block. When the block's sides do not face the jaws
  (tilted grasps), its corner reaches further across them, so the gap grows by half the edge × (cos ψ + sin ψ − 1)
  for the misalignment ψ, up to 5.2 mm at 45°. Closing then turns the block and pushes it onto the fixed jaw.
- **Place height:** the held block keeps its pose in the gripper since the grasp, and it tilts when the place uses
  another family than the grasp. The place puts the block's centre over the target and its lowest corner 3 mm above
  the table.

The kinematic facts come from the robot model at zero joint values: the pan axis, the plane's offset, `wrist_roll`'s
zero and limits, the `tcp` and the fixed jaw's side. The generator throws when the model breaks its assumptions, for
example non-parallel pitch axes. Everything else is in `config/grasp_poses.yaml`, its only source: the families, the
distances, the gap and the clearance. The MTC node loads the same file.

The test checks the poses against MoveIt itself, without a simulator. It loads our URDF and SRDF with KDL as
`move_group` does, and asks for a collision-free solution (robot and table, gripper open) of every pose over the task
area: the block every 5 mm and 5°, the target every 1 cm and 10°.

- **Grasps:** 9405 IK solves (grasp, pre-grasp, lift), none failed. MoveIt's FK returns each pose within 0.01 mm and
  0.0004°, and the predicted `wrist_roll` is the solution's.
- **Places:** 5382 solves for blocks held after a top-down, a 15° and a 30° grasp, none failed.
- **The gap:** all 265 jaw sides left out for `wrist_roll` have no solution.
- **Family limits:** run with the limits of ADR 0003's map (0.24 m and 0.30 m), the same test failed only there: the
  pre-grasp or the pre-place, 5 cm above, is out of reach. Hence 0.23 m and 0.29 m (ADR 0003's update).

## Pick and place with MTC

`so101_mtc_task`'s node `pick_place` plans the pick and place with MoveIt Task Constructor and lets `move_group`
execute it. With the simulation running ([Run the simulation](#run-the-simulation)):

```bash
ros2 launch so101_mtc_task pick_place.launch.py   # [execute:=false] [run_at_start:=false]
ros2 service call /pick_place/pick_place so101_mtc_interfaces/srv/PickPlace "{target: {x: 0.20, y: -0.10}}"
```

The launch file loads the robot as `move_group` does, with `config/grasp_poses.yaml` and `config/pick_place.yaml`
(the target of the run at start-up, the distances, the scaling and the timeouts). At start-up the node picks the block
and places it on that target; each call of `~/pick_place` does it again, for the block where it lies and the target
of the request (a point on the table top). The response and the log give the result: whether planning and execution
succeeded, the failing stages, the planning and execution times, and the grasp and the place that ran. RViz's MTC
panel ("Motion Planning Tasks") shows the stages and the solutions.

- **Stages** (after `mycobot_mtc_pick_place_demo`): current state → allow block–table → open → move to pick (OMPL) →
  pick: approach 5 cm along the tcp's z, the grasp poses with IK, allow hand–block, close, attach the block in the
  hand, lift 5 cm, forbid block–table → move to place (OMPL) → place: lower 5 cm, the place poses with IK, release,
  detach, retreat 5 cm, forbid hand–block → home.
- **Poses:** the grasps come from `grasp_poses` for the block's true pose (`/block/free_joint_states`), one candidate
  per family and jaw side; the places are computed while planning, from the block's pose in the hand. MTC runs the
  cheapest solution.
- **The block in the hand:** closing pushes the block onto the fixed jaw and turns its sides to the jaws, so it ends
  centred on the tcp. The planning scene attaches it there rather than where it stood, so the place fits the block
  that is really held.
- **Free motions move the arm only:** `move_group` executes each part of a solution through the SRDF group that moves
  exactly its joints, and the gripper keeps its state between the grasp and the place.

In MuJoCo, six pick and places with blocks of every family ended with the block on the target, 3–9 mm from its centre
and flat on the table. They included two in each family, and two tilted grasps of blocks turned 35–40° against the
jaws. Planning took 0.7–2.0 s, execution about 12.5 s.

## Trials

The trials judge the pick and place in the simulator, by project 1's success check, as fixed in
[ADR 0004](docs/decisions/0004-evaluation_protocol.md) before the first one:

- **Draw:** the block and the target come from the task area as project 1's simulator draws them, with a seed.
- **Success:** for the last 0.2 s of a second after the task, the block lies on the target disc (5 cm), rests on the
  table on a face, is still, and the hand is away from it. Project 1 also checks the contact forces, which ROS does
  not show.
- **Each trial:** the gripper opens and the arm goes home, the block is put down, `/pick_place/pick_place` runs, the
  block's pose decides. No retries; one CSV row per trial, a JSON summary per run, under
  `$DATA_DIR/so101_mtc/m2/`.

With the simulation and the task node running (`pick_place.launch.py run_at_start:=false`):

```bash
ros2 run so101_mtc_eval_ros run_trials --trials 20 --seed 0   # about 18 s per trial; --dry-run lists the draw
```

**M2 result** (20 trials, seed 0): **20 of 20 succeeded**, 95 % Wilson interval 84–100 %. By grasp family: top-down
12 of 12, tilted 15° 6 of 6, tilted 30° 2 of 2. Among the tilted grasps, three blocks were turned 20–38° against the
jaws (ADR 0003's open point); all three ended on the target. The block ended 5.1 mm from the target's centre
(median; at most 12.3 mm). Planning took 1.5 s (median; at most 1.9 s), a whole pick and place 14.2 s (median; from
the call to the arm back home).

## Tests

The ROS-free cores need only numpy and pytest (on Ubuntu 24.04: `sudo apt install python3-numpy python3-pytest`);
each has its own `pyproject.toml`, so each runs on its own:

```bash
python3 -m pytest so101_mtc_perception
python3 -m pytest so101_mtc_eval
```

GitHub Actions runs the same tests, with the same numpy and pytest versions, on every push to `main` and every pull
request ([tests.yml](.github/workflows/tests.yml)).

The ROS packages are tested from the workspace, after the build (ROS 2 Jazzy; not in CI before M5):

```bash
colcon test && colcon test-result --verbose   # about 35 s, 30 of them so101_mtc_task's IK sweep
```

The test expands the xacro, lets MuJoCo's `compile` load the scene, and checks that the two agree: the joints and
their limits, the poses of all links over random joint angles, the camera and the table; that `tcp` is where ADR 0003
puts it; and that the table and the camera's view hold the whole task area. The MoveIt config's test loads the robot
into MoveIt through `moveit_py`, without a simulator: the groups and the end effector, the named states (`home` is
the start pose), no collision in them but collisions where the arm reaches through the table, and controllers that
agree with MoveIt's mapping. The bringup's test loads the launch file and checks that the MuJoCo plugins name a
camera, a frame and a body that exist. `so101_mtc_task`'s gtest checks the pose generator against MoveIt's KDL and FK
over the task area ([Grasp and place poses](#grasp-and-place-poses)); its pytest loads the pick and place launch file
and checks that the parameter files hold every parameter the node reads. `so101_mtc_eval_ros`'s test calls
`run_trials --dry-run` as a person would.

## License and third-party code

Apache-2.0 ([LICENSE](LICENSE)). Third-party repositories are pinned in [so101_mtc.repos](so101_mtc.repos) and
never modified: the official SO-101 model (TheRobotStudio SO-ARM100, `Simulation/SO101`, Apache-2.0) and its MuJoCo
Menagerie version (`robotstudio_so101`, Apache-2.0). Code adapted from Automatic Addison's myCobot 280 tutorial
(`mycobot_mtc_pick_place_demo`, BSD-3-Clause, Addison Sears-Collins) says "Based on …" in its header and keeps the
BSD-3-Clause notice.
