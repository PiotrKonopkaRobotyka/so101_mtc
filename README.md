# so101_mtc

Pick and place of a block with the simulated SO-101 arm by classical planning: a fixed camera finds the block with
OpenCV, MoveIt 2 and MoveIt Task Constructor (MTC) plan the grasp and the place, ros2_control executes them.

It is the second project of a series that solves the same task with different methods and compares them on the same
metrics (success rate over 50 episodes, time per pick and place): imitation learning (webcam teleoperation and ACT,
`so101_lerobot`), classical planning (this repository) and reinforcement learning (planned).

## Status

M0 Foundations. The scope and the exit criteria are fixed in
[ADR 0001](docs/decisions/0001-project_scope_and_exit_criteria.md), the planned nodes, topics and TF frames in
[docs/architecture.md](docs/architecture.md). Spikes choose the simulator (MuJoCo or Gazebo), where MTC comes from
and the IK for the arm's 5 joints; the ROS packages follow them. So far only the ROS-free core and its tests exist.

## Packages

- `so101_mtc_perception/` — block detection and camera geometry; plain Python without ROS, tested in CI.
- Planned: `so101_mtc_description` (the robot with a TCP frame, the table, the block, the camera),
  `so101_mtc_moveit_config` (end effector, TCP, IK for 5 joints), `so101_mtc_bringup` (one launch for everything),
  `so101_mtc_task` (the MTC pick and place task in C++), `so101_mtc_perception_ros` (a thin node: image → block pose).

## Tests

The ROS-free core needs only numpy and pytest (on Ubuntu 24.04: `sudo apt install python3-numpy python3-pytest`):

```bash
python3 -m pytest so101_mtc_perception
```

GitHub Actions runs the same tests, with the same numpy and pytest versions, on every push to `main` and every pull
request ([tests.yml](.github/workflows/tests.yml)).

## License and third-party code

Apache-2.0 ([LICENSE](LICENSE)). Third-party repositories will be pinned in `so101_mtc.repos` and never modified.
Code adapted from Automatic Addison's myCobot 280 tutorial (`mycobot_mtc_pick_place_demo`, BSD-3-Clause, Addison
Sears-Collins) says "Based on …" in its header and keeps the BSD-3-Clause notice.
