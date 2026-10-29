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

- Nothing pinned yet: the stack spike (ROB-43) adds `so101_mtc.repos`. Third-party repos are never modified.
- Reference only: `mycobot_mtc_pick_place_demo` in `~/code/learning/mycobot_ws` (BSD-3-Clause, Addison
  Sears-Collins). Adapted code gets a "Based on …" header and keeps the BSD notice; never copy it unmarked.

## Build, run, check

- Build: none until ROB-43 picks the underlay (`/opt/ros/jazzy` with apt MTC, or `~/code/deps/moveit_ws`).
- Tests: `python3 -m pytest src/so101_mtc/so101_mtc_perception` from the workspace (~1 s). System Python with numpy
  1.26.4 and pytest 7.4.4 from apt, no venv (`pythonpath` in `pyproject.toml`); CI pins the same versions.
- Data outside git: `DATA_DIR` (default `~/data`) under `so101_mtc/`; spike code in `~/code/experiments/so101_mtc_*`.

## Decisions (`docs/decisions/`)

- 0001 Project scope and exit criteria: M1 bringup, M2 MTC with the ground-truth pose, M3 camera, M4 50 episodes,
  M5 release; the task and the metrics shared with project 1.

## Pitfalls

- MTC exists twice: apt 0.1.4 in `/opt/ros/jazzy` and source (with a `storage.cpp` patch) in `moveit_ws`. The
  underlay sourced at build time decides which one is used; never build with another overlay sourced.
- 5 arm joints: a grasp pose is reachable only if its approach lies in the vertical plane through the base axis and
  the target; full-pose IK (the upstream KDL config) fails elsewhere (ROB-44).
- The upstream `lerobot_ws` model names its joints "1"–"6" and has no TCP, end effector or camera.

## Current milestone

M0 Foundations: ROB-41 skeleton (done when CI is green), then ROB-42 prior art → ROB-43 stack → ROB-44 IK.
