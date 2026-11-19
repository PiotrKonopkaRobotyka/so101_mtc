# 0005. Block detection from the fixed camera

- Status: Accepted
- Date: 2026-11-19

## Context

ADR 0001 asks M3 to give the MTC task the block's pose from the camera, through OpenCV in the ROS-free core,
instead of the simulator's true pose, and to measure its position error against the true pose over the block area.

The scene (ROB-47) has one fixed camera, 0.55 m above the table over (0.22, 0) m, looking straight down: 640 × 480
pixels, a 58° vertical field of view (f = 433 px), so a pixel covers about 1.2 mm of the table
(`so101_mtc_description/mujoco/scene.xml.xacro`, the camera `camera`). It publishes `/camera/image_raw` (rgb8, seen
on the topic) and `/camera/camera_info` at 10 Hz of simulation time (`so101_mtc_bringup/config/mujoco_plugins.yaml`,
`camera_publish_rate`); `camera_optical_frame` is in the URDF, so TF gives its pose. The block is a red 25 mm cube
on a grey table; the arm is yellow and black and, at home, out of the camera's view of the block area. Light comes
from above: in a frame of the camera (2026-11-19, the block at (0.256, −0.091) m), the block's top face is bright
(V ≈ 227 of 255) and its side, seen at an angle, dark (V ≈ 34). Python stays with apt packages (ADR 0002):
python3-opencv 4.6.0 and python3-numpy 1.26.4 (`dpkg -s`). `so101_mtc_perception.geometry.pixel_to_plane` already
turns a pixel into a point on a horizontal plane (ROB-41).

## Decision

1. **Detection** (`so101_mtc_perception.detect`): an HSV threshold keeps red pixels that are saturated and bright,
   which leaves the top face and drops the sides; a morphological opening removes specks; the largest contour above
   20 px² is the block, and OpenCV's `minAreaRect` fits its square.
2. **Pose:** the face's centre and two of its corners go through `pixel_to_plane` onto the plane z = the block's
   edge (the top face). The centre is the block's position; the edge between the corners gives its yaw, modulo a
   quarter turn, in (−45°, 45°], since a cube looks the same every 90°. The block's centre is half its edge high.
   Nothing assumes how the camera is turned: TF gives its pose, `camera_info` its K.
3. **ROS layer** (`so101_mtc_perception_ros`'s `block_detector`): it converts the image without `cv_bridge`,
   looks up the camera in TF and publishes `/block_pose` (`PoseStamped` in world, the image's stamp; reliable and
   transient local). It installs the core as `so101_mtc_eval_ros` installs its own (ADR 0004).
4. **The task** reads the block's pose from `/block_pose` when launched with `camera:=true`, only if its stamp is at
   most 1 s old (simulation time); otherwise the true pose, as in M2. The detector publishes at 10 Hz, so a pose from
   before a reset of the block is out of date once the reset has settled for a second (the trials wait 1 s).

## Consequences

- **Measured** (`so101_mtc_eval_ros`'s `measure_perception`, the blocks of ADR 0004's draw): the error over the
  block area is in the README section "Block pose from the camera".
- **Limits:** the block must be red and in view, with the arm away. The pose is read with the arm at home, before
  the task moves it. Another colour, an occlusion or a change of light needs new thresholds or a learned detector,
  which projects 1 and 3 cover. A tilted block (fallen on an edge) would be found where its brightest face points.
- **M4** runs its episodes with `camera:=true`; a trial without a fresh pose fails at the stage "perception".
