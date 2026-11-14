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
- Planned: `so101_mtc_description`, `so101_mtc_moveit_config`, `so101_mtc_bringup` (M1), `so101_mtc_task` (C++ MTC,
  M2), `so101_mtc_perception_ros` (thin node, M3); see `docs/architecture.md`.

## Third-party and attribution

- Pinned in `so101_mtc.repos`, never modified, fetched into `src/` by `tools/fetch_models.py` (sparse, ~55 MB;
  plain `vcs import` clones ~0.8 GB): SO-ARM100 `Simulation/SO101` (official URDF `so101_new_calib`, Apache-2.0) and
  MuJoCo Menagerie `robotstudio_so101` (MJCF, Apache-2.0).
- Reference only: `mycobot_mtc_pick_place_demo` in `~/code/learning/mycobot_ws` (BSD-3-Clause, Addison
  Sears-Collins). Adapted code gets a "Based on …" header and keeps the BSD notice; never copy it unmarked.

## Build, run, check

- Underlay: `/opt/ros/jazzy` only, never `moveit_ws` (ADR 0002): MTC 0.1.8 and `mujoco_ros2_control` 0.1.2 from apt.
  No ROS packages to build yet (M1). The stack spike (MuJoCo scene, minimal MoveIt config, grasp test) is in
  `~/code/experiments/so101_mtc_stack_spike` (its README has the commands).
- Tests: `python3 -m pytest src/so101_mtc/so101_mtc_perception` from the workspace (~1 s). System Python with numpy
  1.26.4 and pytest 7.4.4 from apt, no venv (`pythonpath` in `pyproject.toml`); CI pins the same versions.
- Data outside git: `DATA_DIR` (default `~/data`) under `so101_mtc/`; spike code in `~/code/experiments/so101_mtc_*`.

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
- Gripper position targets hold the block only past contact (25 mm block: 0.1 rad holds, 0.2 rad never touches).
- MuJoCo: `<attach>` drops the model's `<option>` (copy it into the scene); services are under
  `/mujoco_ros2_control_node/`; `reset_world` with active controllers leaves the arm at its last command.
- Upstream URDF: relative mesh paths (`assets/…`), placeholder limits (10); Menagerie's `gripperframe` is 19.9 mm
  from the URDF's `gripper_frame_link` — the TCP is ours.

## Current milestone

M0 Foundations done: ROB-41 skeleton, ROB-42 prior art, ROB-43 stack (ADR 0002), ROB-44 IK (ADR 0003). Next: M1
bringup (issues to be planned in Linear); open points for M2 are in ADR 0003's consequences.
