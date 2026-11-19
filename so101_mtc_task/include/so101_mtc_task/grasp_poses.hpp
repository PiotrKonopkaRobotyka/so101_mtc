// Grasp and place poses that the 5-joint SO-101 reaches exactly (ADR 0003).
//
// The three pitch joints keep the wrist_roll axis in one vertical plane, the arm plane, which shoulder_pan turns
// about its vertical axis. A pose of the gripper is reachable only when its roll axis lies in that plane, so the
// generator builds every pose from the plane's azimuth theta, which it finds in closed form. A point p of the gripper
// frame lands on the world point c when
//
//     r sin(phi - theta) = p_x cos(beta) - p_y sin(beta) + e,
//
// where (r, phi) is c in polar coordinates about the shoulder_pan axis, beta the jaw angle about the roll axis
// (measured from the plane's normal) and e the plane's offset from the pan axis. MoveIt's KDL then solves these poses
// exactly; poses that miss the plane have no solution at all.
//
// The kinematic facts come from the robot model (URDF and SRDF), the rest from config/grasp_poses.yaml.
#pragma once

#include <optional>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <moveit/robot_model/robot_model.hpp>
#include <rclcpp/node.hpp>

namespace so101_mtc_task
{

/// The generator's parameters (config/grasp_poses.yaml). Lengths in metres, angles in radians.
struct GraspParameters
{
  std::string arm_group;      ///< SRDF group: shoulder_pan, the parallel pitch joints, wrist_roll
  std::string gripper_group;  ///< SRDF group of the moving jaw
  std::string tcp_frame;      ///< the frame the poses are given for
  std::vector<double> tilts;      ///< per family: the approach's tilt from vertical, leaning away from the base
  std::vector<double> max_radii;  ///< per family: the farthest point from the shoulder_pan axis it serves
  double pregrasp_distance = 0.0;  ///< pre-grasp and retreat: back along the approach
  double lift_distance = 0.0;      ///< lift and pre-place: straight up
  double approach_gap = 0.0;       ///< the tcp's offset from the block's centre towards the fixed jaw at the grasp
  double place_clearance = 0.0;    ///< the held block's lowest point above the table at the place
};

/// Reads the parameters under `prefix` (degrees in the file, radians here). They have no defaults in the code: a
/// missing one throws, naming it. Works with nodes that declare their parameters from overrides, as MoveIt's do.
GraspParameters loadParameters(rclcpp::Node& node, const std::string& prefix = "grasp_poses");

/// The arm's kinematic facts, read from the robot model at zero joint values, where the arm reaches forward.
struct ArmGeometry
{
  std::string model_frame;    ///< the frame of all poses (the URDF's root, world)
  std::string gripper_frame;  ///< the link wrist_roll turns (gripper_link): z along the roll axis, x across the jaws
  Eigen::Vector2d pan_axis = Eigen::Vector2d::Zero();  ///< where the vertical shoulder_pan axis meets z = 0
  double plane_offset = 0.0;  ///< e: the arm plane's signed distance from the pan axis, along its normal
  double roll_zero = 0.0;     ///< the jaw angle at wrist_roll = 0, so that wrist_roll = beta - roll_zero
  double roll_min = 0.0;      ///< wrist_roll's limits
  double roll_max = 0.0;
  double fixed_jaw_side = 0.0;  ///< +1 or -1: the fixed jaw's side along x of the gripper frame
  Eigen::Isometry3d gripper_to_tcp = Eigen::Isometry3d::Identity();  ///< the tcp in the gripper frame
};

/// Reads the facts; throws std::runtime_error, naming the reason, when the model breaks the generator's assumptions
/// (a vertical pan axis, parallel horizontal pitch axes, a roll axis across them along z of the gripper frame).
ArmGeometry readArmGeometry(const moveit::core::RobotModelConstPtr& model, const GraspParameters& parameters);

/// A pose of the gripper frame that the arm reaches exactly, with its jaw angle beta.
struct GripperPose
{
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  double jaw_angle = 0.0;
};

/// One way to grasp a block: a family (tilt) and a jaw side, with the tcp poses of its waypoints in the model frame.
struct Grasp
{
  double tilt = 0.0;        ///< the approach's tilt from vertical
  double jaw_angle = 0.0;   ///< beta
  double wrist_roll = 0.0;  ///< the wrist_roll angle the three poses need
  Eigen::Isometry3d pregrasp = Eigen::Isometry3d::Identity();  ///< the grasp, moved back along the approach
  Eigen::Isometry3d grasp = Eigen::Isometry3d::Identity();     ///< where the gripper closes
  Eigen::Isometry3d lift = Eigen::Isometry3d::Identity();      ///< the grasp, moved straight up
};

/// One way to put the held block down: the reverse of a grasp.
struct Place
{
  double tilt = 0.0;
  double jaw_angle = 0.0;
  double wrist_roll = 0.0;
  Eigen::Isometry3d preplace = Eigen::Isometry3d::Identity();  ///< the place, moved straight up
  Eigen::Isometry3d place = Eigen::Isometry3d::Identity();     ///< where the gripper opens
  Eigen::Isometry3d retreat = Eigen::Isometry3d::Identity();   ///< the place, moved back along the approach
};

class PoseGenerator
{
public:
  /// Reads the arm's facts from the model; throws as readArmGeometry does.
  PoseGenerator(const moveit::core::RobotModelConstPtr& model, GraspParameters parameters);

  const ArmGeometry& arm() const { return arm_; }
  const GraspParameters& parameters() const { return parameters_; }

  /// A point's distance from the shoulder_pan axis.
  double radius(const Eigen::Vector3d& point) const;
  /// The tilt of the first family that serves this distance; none beyond the last family.
  std::optional<double> tiltFor(double radius) const;

  /// Grasps of a block standing on the table, given its centre and its yaw about the vertical. Top-down, the jaws
  /// face the block's sides (four directions); tilted, they are horizontal (two sides). Only jaw sides within
  /// wrist_roll's limits; at the grasp, the tcp is approach_gap from the centre towards the fixed jaw.
  std::vector<Grasp> grasps(const Eigen::Vector3d& block_centre, double block_yaw) const;

  /// Places of the held block over a target point on the table top. `block_in_tcp` is the block's pose in the tcp
  /// frame since the grasp and `block_size` its edges: the block's centre goes over the target, its lowest corner
  /// place_clearance above it. The family follows the target's distance; the jaws lie across the arm plane.
  std::vector<Place> places(const Eigen::Vector3d& target, const Eigen::Isometry3d& block_in_tcp,
                            const Eigen::Vector3d& block_size) const;

  /// The gripper pose that puts `point` (in the gripper frame) on `target`, for a tilt and a jaw angle; none when
  /// the target is too close to the pan axis for the plane to pass the point.
  std::optional<GripperPose> gripperPose(const Eigen::Vector3d& point, const Eigen::Vector3d& target, double tilt,
                                         double jaw_angle) const;
  /// The same top-down, with the jaw axis along a world azimuth; the jaw angle follows.
  std::optional<GripperPose> topDownGripperPose(const Eigen::Vector3d& point, const Eigen::Vector3d& target,
                                                double jaw_azimuth) const;

  /// The wrist_roll angle for a jaw angle, in [-pi, pi].
  double wristRoll(double jaw_angle) const;
  bool withinRollLimits(double wrist_roll) const;

private:
  ArmGeometry arm_;
  GraspParameters parameters_;
};

}  // namespace so101_mtc_task
