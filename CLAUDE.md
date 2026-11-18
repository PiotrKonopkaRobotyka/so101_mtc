# so101_mtc — project context for Claude Code

Supplements `~/code/CLAUDE.md` (workflow, safety and naming rules). Plan and status live only in the Linear
project [SO-101 MoveIt 2 + MTC pick and place](https://linear.app/robotics-software-engineer/project/so-101-moveit-2-mtc-pick-and-place-3847f63da317);
setup and commands for people are in [README.md](README.md); session log in `~/code/logs/so101_mtc/`.

## Goal

Pick and place of a block with the simulated SO-101 by classical planning: a fixed camera and OpenCV find the block,
MoveIt 2 and MTC plan the grasp and the place. Project 2 of the series "the same task, different methods": the task
and metrics come from `so101_lerobot` (its ADR 0005); exit criteria in ADR 0001, architecture in `docs/`.

## Packages in this repository

- `so101_mtc_perception/` — plain Python, no `rclpy` (`COLCON_IGNORE`); `geometry`: pixel → ray → table plane.
- `so101_mtc_description/` — URDF xacro (upstream + `world`, table, `tcp`, camera, `ros2_control`), MuJoCo scene
  (`scene.xml.xacro` → `scene.xml` at build), `config/task.yaml` (project 1's block, area, target); see README.
- `so101_mtc_moveit_config/` — hand-written: SRDF (`arm` base_link → tcp, `gripper`, eef at tcp), KDL, limits, OMPL,
  `ros2_controllers.yaml` + `moveit_controllers.yaml`, `move_group` with MTC's capability, RViz; see README.
- `so101_mtc_bringup/` — `sim.launch.py` (MuJoCo → spawners by `OnProcessExit` → move_group + RViz), MuJoCo plugins.
- Planned: `so101_mtc_task` (C++ MTC, M2), `so101_mtc_perception_ros` (thin node, M3).

## Third-party and attribution

- Pinned in `so101_mtc.repos`, never modified, fetched into `src/` by `tools/fetch_models.py` (sparse, ~55 MB;
  plain `vcs import` clones ~0.8 GB): SO-ARM100 `Simulation/SO101` (official URDF `so101_new_calib`, Apache-2.0) and
  MuJoCo Menagerie `robotstudio_so101` (MJCF, Apache-2.0).
- Reference only: `mycobot_mtc_pick_place_demo` in `~/code/learning/mycobot_ws` (BSD-3-Clause, Addison
  Sears-Collins). Adapted code gets a "Based on …" header and keeps the BSD notice; never copy it unmarked.

## Build, run, check

- Underlay: `/opt/ros/jazzy` only, never `moveit_ws` (ADR 0002): MTC 0.1.8 and `mujoco_ros2_control` 0.1.2 from apt.
- Build `colcon build --packages-up-to so101_mtc_bringup` (needs the fetched models); run `ros2 launch
  so101_mtc_bringup sim.launch.py [headless:=true rviz:=false sim_speed_factor:=5.0]` (alias `so101_mtc`); test
  `colcon test --packages-up-to so101_mtc_bringup && colcon test-result --verbose` (~3 s, not in CI before M5).
- Core tests: `python3 -m pytest src/so101_mtc/so101_mtc_perception` from the workspace (~1 s). System Python with
  numpy 1.26.4 and pytest 7.4.4 from apt, no venv (`pythonpath` in `pyproject.toml`); CI pins the same versions.
- Data outside git: `DATA_DIR` (default `~/data`) under `so101_mtc/`. Spikes and sim checks (READMEs with commands):
  `~/code/experiments/so101_mtc_{stack_spike,ik_spike,moveit_check}`.

## Decisions (`docs/decisions/`)

- 0001 Project scope and exit criteria: M1 bringup, M2 MTC with the ground-truth pose, M3 camera, M4 50 episodes,
  M5 release; the task and the metrics shared with project 1.
- 0002 Simulator, MTC build and robot model: MuJoCo through `mujoco_ros2_control`, MTC from apt on `/opt/ros/jazzy`,
  the official SO-101 (URDF for MoveIt, Menagerie MJCF for MuJoCo), apt-only Python, sparse fetch of the models.
- 0003 IK and grasp poses: top-down up to ~0.24 m from the pan axis, then tilted 15°/30°; own generator of exactly
  reachable poses → `GeneratePose` + `ComputeIK` with KDL; TCP (0.0039, −0.0002, −0.0958) m in `gripper_link`.

## Pitfalls

- MTC exists twice: apt 0.1.8 in `/opt/ros/jazzy` (ours) and source 0.1.4 with a `storage.cpp` patch in `moveit_ws`
  (other projects). Never build or run with `moveit_ws` or another overlay sourced.
- Upgrade ROS packages a whole sync at a time: apt dependencies carry no versions (ros2_control 4.46 changed ABI).
- 5 arm joints: the arm plane must contain the `wrist_roll` axis, and the TCP is 3.9 mm off it, so poses aimed at the
  block's azimuth are unreachable when tilted (KDL 0/240); use ADR 0003's correction. `position_only_ik` is useless.
- MuJoCo: `<attach>` drops the model's `<option>` (copy it into the scene); services are under
  `/mujoco_ros2_control_node/`; `reset_world` with active controllers leaves the arm at its last command.
- Upstream URDF: relative mesh paths and placeholder limits (the build and `joint_limits.yaml` handle them).
- `scene.xml` is generated: rebuild after editing `scene.xml.xacro` or `task.yaml`; load the installed file only.
  The table and the camera are written twice (URDF and scene); the test compares them.
- Gripper: position targets hold only past contact; the controller closes to `closed` (0.0) and stalls on the 25 mm
  block at 0.065 rad. MTC plans from `closed`, hence `allowed_start_tolerance_joints.gripper` 0.2.
- The tcp is the centre of a block pressed onto the fixed jaw: descend with the tcp 3.6 mm towards the fixed jaw, or
  the jaw drags the block (25–34 mm); closing then pushes the block under the tcp.
- wrist_roll = jaw angle beta + 92.8°, limits −157°/+163°: top-down beta 75°–105° is out, use the other jaw side.

## Current milestone

M0 and M1 done (ROB-41–44, ROB-47–49: one-command bringup, 8/8 grasps through MoveIt). Next M2: the MTC task with
the true pose; pattern: `mycobot_mtc_pick_place_demo`'s stages, with ADR 0003's poses, the approach gap and the jaw
side (Pitfalls) instead of `GenerateGraspPose`.
