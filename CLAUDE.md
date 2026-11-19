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
  (`scene.xml.xacro` → `scene.xml` at build), `config/task.yaml` (project 1's block, area, target).
- `so101_mtc_moveit_config/` — hand-written SRDF (`arm` → tcp, `gripper`), KDL, limits, OMPL, controllers, RViz.
- `so101_mtc_bringup/` — `sim.launch.py` (MuJoCo → spawners by `OnProcessExit` → move_group + RViz), MuJoCo plugins.
- `so101_mtc_task/` — C++: library `grasp_poses` (ADR 0003's exactly reachable poses; facts from the model, values
  only in `config/grasp_poses.yaml`), MTC node `pick_place` (service `~/pick_place`, `config/pick_place.yaml`).
- `so101_mtc_interfaces/` — `srv/PickPlace`. `so101_mtc_eval/` — plain Python (`COLCON_IGNORE`): draw, success
  check, records; `so101_mtc_eval_ros/` installs it and runs the trials. Planned: `so101_mtc_perception_ros` (M3).

## Third-party and attribution

- Pinned in `so101_mtc.repos`, never modified, fetched into `src/` by `tools/fetch_models.py` (sparse, ~55 MB):
  SO-ARM100 `Simulation/SO101` (URDF `so101_new_calib`) and MuJoCo Menagerie `robotstudio_so101`, both Apache-2.0.
- Reference only: `mycobot_mtc_pick_place_demo` in `~/code/learning/mycobot_ws` (BSD-3-Clause, Addison
  Sears-Collins). Adapted code gets a "Based on …" header and keeps the BSD notice; never copy it unmarked.

## Build, run, check

- Underlay `/opt/ros/jazzy` only, never `moveit_ws` (ADR 0002): MTC 0.1.8 and `mujoco_ros2_control` 0.1.2 from apt.
- Build `colcon build` (needs the fetched models); run `ros2 launch so101_mtc_bringup sim.launch.py` (alias
  `so101_mtc`; `headless`, `rviz`, `sim_speed_factor`), then `ros2 launch so101_mtc_task pick_place.launch.py`;
  test `colcon test && colcon test-result --verbose` (~35 s).
- Core tests: `python3 -m pytest src/so101_mtc/so101_mtc_<perception|eval>` (~1 s; apt numpy and pytest, pinned in CI).
- Data in `DATA_DIR` (default `~/data`) under `so101_mtc/`; spikes, sim checks and the trials' detached `run.sh`
  (sim, node and runner; > 5 min): `~/code/experiments/so101_mtc_*`.

## Decisions (`docs/decisions/`)

- 0001 Scope and exit criteria: M1 bringup, M2 MTC with the true pose, M3 camera, M4 50 episodes, M5 release.
- 0002 Stack: MuJoCo (`mujoco_ros2_control`), MTC from apt, official SO-101 (URDF + Menagerie MJCF), apt-only Python.
- 0003 IK and grasps: top-down, then tilted 15°/30°; exactly reachable poses for KDL; TCP in the URDF.
- 0004 Evaluation protocol: project 1's check from the block's pose and TF, its draw, seeds, no target before M4.

## Pitfalls

- MTC exists twice: apt 0.1.8 (ours) and source 0.1.4 in `moveit_ws` (other projects): never build or run with another
  overlay sourced. Upgrade ROS packages a whole sync at a time (ros2_control 4.46 changed its ABI).
- 5 arm joints: a pose is reachable only with the `wrist_roll` axis (3.9 mm off the TCP) in the arm plane; take poses
  from `grasp_poses`, never `position_only_ik`. `wrist_roll` = jaw angle + 92.8° within −157°/+163°.
- MuJoCo: `<attach>` drops the model's `<option>` (copied into the scene); services under `/mujoco_ros2_control_node/`;
  `reset_world` with active controllers leaves the arm at its last command; rebuild after editing the scene or task.
- Gripper: it stalls on the block at 0.065 rad short of `closed`, hence the start tolerance 0.2 rad. The tcp is the
  centre of a block pressed onto the fixed jaw: descend `approach_gap` towards the fixed jaw, or the jaw drags it.
- MTC: `Connect` plans the arm only (execution needs one SRDF group); attach by message, never move a world object in
  a stage callback (diff scenes share it); JTC takes MTC's ~1e-3 rad/s end velocity only with `allow_nonzero_…`.

## Current milestone

M2 done (ROB-51–53; README "Trials"). Next M3: the block's pose from the camera (`so101_mtc_perception` detection,
`so101_mtc_perception_ros` node, the task's `camera:=true`), its error over the area; then M4's 50 episodes.
