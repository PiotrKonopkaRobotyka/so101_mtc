# 0001. Project scope and exit criteria

- Status: Accepted
- Date: 2026-10-29

## Context

This is the second project of a series that solves one task with different methods: SO-101 picks a block and places
it in a target area, in simulation. The first project, `so101_lerobot`, learns the task by imitation (webcam
teleoperation, about 50 demonstrations, ACT; its ADR 0005); the third will use reinforcement learning. The methods
compare only on the same task and the same metrics.

This project uses classical planning: a fixed camera finds the block with OpenCV, MoveIt 2 and MoveIt Task
Constructor (MTC) plan the grasp and the place, ros2_control executes them. Known on 2026-10-29:

- SO-101 has 5 arm joints and a gripper. A grasp pose is reachable only when its approach lies in the vertical plane
  through the base axis and the block. The upstream MoveIt config (`lerobot_ws`) solves IK with KDL on the full pose
  and defines no end effector or TCP.
- The robot models have no camera; the camera belongs to the scene.
- Project 1 grasps in MuJoCo because grasping in Gazebo is a known risk. Its MuJoCo scene, and so the block area and
  the success check, is confirmed in its ROB-17.
- The simulator, where MTC comes from (apt or the `moveit_ws` underlay), the robot model and the IK are open: spikes
  ROB-43 and ROB-44.

## Decision

The project is finished when all five hold (milestones M1–M5):

1. Bringup (M1): one command starts the simulator with SO-101, a table, the block and a fixed camera, the
   controllers, MoveIt with our own config (end effector, TCP, IK for 5 joints) and RViz.
2. Planning (M2): an MTC task in C++ picks the block from its ground-truth pose and places it in the target area, with
   grasp poses the 5-joint arm can reach. Success is checked in the simulator, not by MoveIt's result.
3. Perception (M3): the block pose comes from the camera image through OpenCV in the ROS-free core instead of the
   ground truth; its position error against the ground truth is measured over the block area.
4. Evaluation (M4): 50 episodes with seeded block positions; the success rate, the time per pick and place, the
   planning time and the failures by stage (perception, IK and planning, grasp, place) are reported.
5. Release (M5): a README with the results (the metrics, a video) and the limitations, Docker, the public repository
   and a write-up comparing the methods.

Shared with project 1, so that the numbers compare:

- The task: one block, one target area, the same block size and start area, copied from project 1's MuJoCo scene
  once its ROB-17 confirms it; never redefined here.
- Success: the success check of project 1's scene (the block in the target area at the end of the episode).
- Episodes: 50, with the block's start positions drawn from the start area with fixed seeds.
- Time per pick and place: from the start of the episode to the release in the target area. Here it includes
  perception and planning; the planning time is also reported on its own.

The success rate is reported as measured; no target is set before the first 50 episodes, as in project 1.

Out of scope, the project's non-goals: the real arm until one is available, learning methods (projects 1 and 3),
several objects, clutter or stacking, more than one task, moving obstacles, our own simulator or robot model, learned
object detectors.

## Consequences

- Every issue in M1–M5 serves one of the five criteria.
- The project depends on project 1's scene (ROB-17): M1 copies the block area and the target area from it. If the
  simulator here is not MuJoCo, the physics difference is a stated limitation.
- Planning comes before perception (M2 before M3): IK and grasping with 5 joints are the larger risk, and the
  ground-truth pose separates planning failures from perception failures.
- The ROS-free core is tested in CI from M0; the ROS packages get `colcon test` in M5.
- Code adapted from the myCobot tutorial (BSD-3-Clause) is marked "Based on …" and keeps the BSD notice; the rest is
  our own, under Apache-2.0.
