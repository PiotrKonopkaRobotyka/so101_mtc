# 0002. Simulator, MTC build and robot model

- Status: Accepted
- Date: 2026-11-03

## Context

ADR 0001 left three choices to a spike (ROB-43): the simulator, where MoveIt Task Constructor (MTC) comes from (and
so the underlay), and the SO-101 model. The prior-art review (ROB-42) recommended MuJoCo through
`mujoco_ros2_control`, MTC from apt and the official model. The spike tested them on the laptop (Ubuntu 24.04, ROS 2
Jazzy, the ROS sync of 2026-10-06); its code is in `~/code/experiments/so101_mtc_stack_spike`, its results in
`~/data/so101_mtc/stack_spike`.

- **MTC from apt (0.1.8)** on `/opt/ros/jazzy`, without the `storage.cpp` patch of `~/code/deps/moveit_ws`: MTC's
  `pick_place_demo` with `execute: true` executes (2 of 2 runs, `move_group` reports `SUCCEEDED`). The symptom the
  patch works around ("the path to execute became invalid", MTC #561, Humble) does not appear: its cause was fixed
  by moveit2#2745, which every Jazzy MoveIt has.
- **Robot model:** the official URDF `so101_new_calib` (TheRobotStudio SO-ARM100) and the MJCF of MuJoCo Menagerie
  (`robotstudio_so101`, derived from it) have the same six joints (`shoulder_pan` … `gripper`) and limits. The
  frames of all seven links agree within 0.003 mm over 201 random configurations. Menagerie's `gripperframe` site
  lies 19.9 mm from the URDF's `gripper_frame_link`.
- **MuJoCo** through `mujoco_ros2_control` 0.1.2 (apt, MuJoCo 3.12), with the Menagerie model attached unmodified,
  a table, a 25 mm block and a fixed camera:
  - an RGB-D image and `camera_info`; the block found in the image and intersected with the table plane
    (`pixel_to_plane`) lands within 1 mm of its true pose;
  - the block's true pose at 50 Hz;
  - resets of the block (`set_free_joint_state`, 2 ms) and of the world (`reset_world`);
  - a real-time factor of 1.0 at about 20 % of one core, and 5× real time at about 53 %;
  - ten picks along MoveIt-planned trajectories: the block held 10 of 10 times with a jaw target of 0.1 rad, 0 of
    10 with 0.2 rad.
- **apt:** ROS packages come in syncs, and their dependencies carry no versions, so a partial upgrade can mix
  binaries built against different libraries (ros2_control 4.46 changed the layout of `JointInfo`).
- **Size of the upstream repositories:** about 0.8 GB to clone (Menagerie alone checks out 1.9 GB), of which the
  project uses about 40 MB.

## Decision

1. **Simulator:** MuJoCo through `mujoco_ros2_control` from apt. Gazebo is not used.
2. **MTC from apt** (0.1.8) on `/opt/ros/jazzy`. The workspace overlays `/opt/ros/jazzy` only, not `moveit_ws`. ROS
   packages are upgraded a whole sync at a time.
3. **Robot model:** the official SO-101, unmodified and pinned in `so101_mtc.repos`:
   - the URDF `so101_new_calib` for `robot_state_publisher` and MoveIt;
   - the Menagerie MJCF for MuJoCo.

   Our description package adds `world`, the table, the TCP, the camera and the `ros2_control` tags. The TCP is
   ours, between the jaws (fixed by ROB-44), not Menagerie's `gripperframe`.
4. **Python:** the nodes use apt packages only (OpenCV 4.6, numpy 1.26, `cv_bridge`). There is no `.venv`.
5. **Fetching the pinned repositories:** `tools/fetch_models.py` fetches only the folders used (`Simulation/SO101`,
   `robotstudio_so101`) at the pinned commits into `src/`, with the layout of `vcs import`.

## Consequences

- **One underlay:** Docker (M5) needs `osrf/ros:jazzy-desktop` and apt packages only. The rule "a project with
  MoveIt builds on `moveit_ws`" in `~/code` does not hold for this project.
- **After a ROS sync upgrade:** check that MTC still executes and the MuJoCo bridge still works.
- **Bridge version:** `mujoco_ros2_control` is young (0.1.4 already changes its interface API). The project stays on
  the apt version until a test says otherwise.
- **Upstream URDF:** its mesh paths are relative (`assets/…`) and its velocity and effort limits are placeholders
  (10). The description package rewrites the paths, and the MoveIt config carries `joint_limits.yaml`.
- **MuJoCo details the packages must respect:**
  - `<attach>` keeps the parent's `<option>`, so our scene copies Menagerie's contact settings (elliptic cone,
    `impratio` 10, a 5 ms step);
  - the camera plugin renders every camera in the MJCF, also Menagerie's `wrist_cam`;
  - camera rates count in simulation time;
  - the services live under `/mujoco_ros2_control_node/`;
  - `reset_world` needs the arm controllers deactivated, or the arm returns to its last command.
- **Gripper:** a position target holds the block only past contact (0.1 rad holds the 25 mm block, 0.2 rad never
  touches it). M2 closes with a force-limited gripper action controller (`allow_stalling`), not a fixed position.
- **Reach for ROB-44:** a vertical gripper reaches only about 6 cm above the block's centre over the block area
  (wrist limit), so ROB-44 plans tilted approaches and lifts.
- **Scene:** the spike's block size and area are provisional. M1 takes project 1's scene once its ROB-17 confirms it.
