# 0004. Evaluation protocol of the pick and place

- Status: Accepted
- Date: 2026-11-19

## Context

ADR 0001 asks M2 to check the success in the simulator, not by MoveIt's result, and M4 to run 50 seeded episodes
whose numbers compare with project 1's: the same task, the same success check, the same draw of the block. The
protocol is fixed here before the first trial, which project 1 did only after its results (its retrospective,
2026-11-18).

Project 1's success check is so101-nexus 0.11.0's `MuJoCoPickAndPlace-v2` (its ADR 0009). Read in its installed code
(`so101_nexus/placement.py:379-411`, `config.py:1588-1594`, `mujoco/pick_and_place.py:446-575`), it counts a success
when, for 0.2 s without a break, all of these hold:

- the block's footprint centre lies on the target disc, of 5 cm radius;
- the table carries at least 90 % of the block's weight;
- the robot and anything else push on the block with at most max(1 mN, 1 % of its weight);
- the block is still, at most 0.01 m/s and 0.5 rad/s;
- the gripper has let go.

It draws the disc first, its distance from the spawn centre and its azimuth uniformly. Then it draws the cube the same
way until the cube lies at least 37.5 mm plus its bounding radius (half its footprint's diagonal) from the disc's
centre, within 100 attempts, with a uniform yaw (`mujoco/pick_and_place.py:376-443`, the attempts at :430;
`config.py:1403,1443-1446`; `mujoco/spawn_utils.py:84,151-172`).

This project sees the simulation through ROS: the block's pose at 50 Hz (`/block/free_joint_states`) and TF, but no
contact forces: `mujoco_ros2_control_msgs` 0.1.2 has messages for free-joint states, external wrenches and the
simulation's state only (`/opt/ros/jazzy/share/mujoco_ros2_control_msgs/{msg,srv}`).

## Decision

1. **Success** (`so101_mtc_eval.success`): one second of the block's poses after the task ends. Over its last 0.2 s,
   without a break, all of these hold:
   - **on the disc:** the block's centre lies within 5 cm of the target (`config/task.yaml`'s target radius);
   - **carried by the table:** its centre is within 2 mm of half its edge above the table. Only a block resting on a
     face reaches that height: on an edge it stands 41 % higher, and lifted or on something it stands higher still;
   - **untouched:** the tcp is at least 5 cm from the block's centre;
   - **still:** at most 0.01 m/s and 0.5 rad/s, from consecutive poses.
2. **Draw** (`so101_mtc_eval.draw`): as so101-nexus 0.11.0 does, in `config/task.yaml`'s area (a ring sector of
   0.12–0.34 m and ±60° around the shoulder_pan axis), with numpy's `default_rng(seed)`. The target comes first, then
   the block at least 55.2 mm from it, with a uniform yaw.
3. **Trials:**
   - M2 runs 20 trials with seed 0, in one run;
   - M4 runs 50 episodes with the same draw, its seeds fixed in M4. Update 2026-11-19: seed 1, one run, with the
     block's pose from the camera (`camera:=true`, ADR 0005); its results go to `$DATA_DIR/so101_mtc/m4/`.
     A trial without a fresh pose from the camera fails at the stage "perception".
4. **One trial:**
   - **reset:** the gripper opens and the arm goes home through `move_group`; then the block is put down and settles
     for 1 s;
   - **the task:** one call of `/pick_place/pick_place`, within 180 s (planning at most 30 s);
   - **the check;**
   - **no retries.**
5. **Reported:**
   - the success rate as measured, with its 95 % Wilson interval (with 20 trials about ±20 points);
   - failures by stage: planning (IK and planning), execution, grasp (the block did not move), place (it moved but
     fails the check);
   - the median planning time and the median task time.

   The task time runs from the call to the task's end with the arm home. So it includes the planning and, after the
   release, the retreat and the move home. Update 2026-11-19 (M4): ADR 0001's time per pick and place ends at the
   release, so the runner also times it: from the call to the gripper's first opening past 0.6 rad after it closed on
   the block (`/joint_states`, simulation time).
6. **Target:** none. ADR 0001 sets none before the first 50 episodes. M2's exit (ADR 0001, criterion 2) is this check,
   run in the simulator.
7. **Records** (`so101_mtc_eval.records`): one CSV row per trial, written as the trial ends, and a JSON summary. Each
   run gets a new pair of files under `$DATA_DIR/so101_mtc/m2/`.

## Consequences

- **What the check misses against project 1:** forces. A block that something presses down or holds sideways while it
  rests on the table could pass here and fail there. The tcp's distance rules out the hand, and nothing else is in the
  scene. The disc is not drawn in MuJoCo; the check uses its centre.
- **Coverage:** grasp, place, the families and ADR 0003's open point all come from the same rows. Each row carries the
  grasp's tilt and, for tilted grasps, how far the block's sides were turned against the jaws.
- **Recovery:** a trial that fails mid-motion leaves the arm anywhere; the reset brings it home before the next trial.
  A reset that fails is noted in the trial's row.
- **The tcp:** read once from TF at the end of the second of poses, for all of them; the arm is home by then.
- **Packaging:** `so101_mtc_eval` is plain Python with its own `pyproject.toml`, tested in CI, and colcon ignores it.
  ADR 0002 rules out pip and a venv, so `so101_mtc_eval_ros` installs it with `ament_python_install_package` from
  `../so101_mtc_eval`. The ROS package therefore builds only inside this repository's layout. M3's perception does
  the same.
- **M4** keeps the draw, the check and the records. It changes the number of episodes and, from M3, the block's pose,
  which comes from the camera.
