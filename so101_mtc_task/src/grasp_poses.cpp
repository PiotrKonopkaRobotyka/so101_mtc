#include "so101_mtc_task/grasp_poses.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

#include <moveit/robot_model/revolute_joint_model.hpp>
#include <moveit/robot_state/robot_state.hpp>

namespace so101_mtc_task
{
namespace
{
// The URDF's angles are rounded (1.5708 for pi/2), which tilts its axes by up to about 1e-5 rad
constexpr double kAxisTolerance = 1e-4;

const Eigen::Vector3d kUp = Eigen::Vector3d::UnitZ();

[[noreturn]] void fail(const std::string& reason)
{
  throw std::runtime_error("so101_mtc_task: " + reason);
}

double degrees_to_radians(double degrees)
{
  return degrees * M_PI / 180.0;
}

/// A parameter that the node may already have declared from its overrides; without a value, declaring it throws.
template <typename T>
T required(rclcpp::Node& node, const std::string& name)
{
  if (!node.has_parameter(name))
  {
    node.declare_parameter<T>(name);
  }
  return node.get_parameter(name).get_value<T>();
}

Eigen::Isometry3d moved(const Eigen::Isometry3d& pose, const Eigen::Vector3d& offset)
{
  return Eigen::Translation3d(offset) * pose;
}

/// The gripper frame for the arm plane at azimuth theta: z (fingertips → wrist) tilted back towards the base, x (the
/// jaw axis) at the jaw angle from the plane's normal, and `point` of the frame on `target`.
Eigen::Isometry3d gripperFrame(double theta, double tilt, double jaw_angle, const Eigen::Vector3d& point,
                               const Eigen::Vector3d& target)
{
  const Eigen::Vector3d forward(std::cos(theta), std::sin(theta), 0.0);  // from the pan axis, along the plane
  const Eigen::Vector3d normal(-std::sin(theta), std::cos(theta), 0.0);  // horizontal, across the plane
  const Eigen::Vector3d z = std::cos(tilt) * kUp - std::sin(tilt) * forward;
  const Eigen::Vector3d x = std::cos(jaw_angle) * normal + std::sin(jaw_angle) * z.cross(normal);
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.linear() << x, z.cross(x), z;
  pose.translation() = target - pose.linear() * point;
  return pose;
}

}  // namespace

GraspParameters loadParameters(rclcpp::Node& node, const std::string& prefix)
{
  const auto name = [&prefix](const std::string& key) { return prefix + "." + key; };
  GraspParameters parameters;
  parameters.arm_group = required<std::string>(node, name("arm_group"));
  parameters.gripper_group = required<std::string>(node, name("gripper_group"));
  parameters.tcp_frame = required<std::string>(node, name("tcp_frame"));
  for (double tilt : required<std::vector<double>>(node, name("tilts")))
  {
    parameters.tilts.push_back(degrees_to_radians(tilt));
  }
  parameters.max_radii = required<std::vector<double>>(node, name("max_radii"));
  parameters.pregrasp_distance = required<double>(node, name("pregrasp_distance"));
  parameters.lift_distance = required<double>(node, name("lift_distance"));
  parameters.approach_gap = required<double>(node, name("approach_gap"));
  parameters.place_clearance = required<double>(node, name("place_clearance"));
  if (parameters.tilts.empty() || parameters.tilts.size() != parameters.max_radii.size())
  {
    throw std::invalid_argument(name("tilts") + " and " + name("max_radii") +
                                " must be non-empty lists of the same length, one entry per family");
  }
  return parameters;
}

ArmGeometry readArmGeometry(const moveit::core::RobotModelConstPtr& model, const GraspParameters& parameters)
{
  const moveit::core::JointModelGroup* arm_group = model->getJointModelGroup(parameters.arm_group);
  const moveit::core::JointModelGroup* gripper_group = model->getJointModelGroup(parameters.gripper_group);
  if (!arm_group || !gripper_group || !model->hasLinkModel(parameters.tcp_frame))
  {
    fail("the robot model lacks the group " + parameters.arm_group + " or " + parameters.gripper_group +
         ", or the link " + parameters.tcp_frame);
  }
  const std::vector<const moveit::core::JointModel*>& joints = arm_group->getActiveJointModels();
  if (joints.size() < 3)
  {
    fail(parameters.arm_group + " must have a pan joint, pitch joints and a roll joint");
  }

  // The reference: every arm joint at zero, where the SO-101 reaches straight forward
  moveit::core::RobotState state(model);
  state.setToDefaultValues();
  for (const moveit::core::JointModel* joint : joints)
  {
    const double zero = 0.0;
    if (!joint->satisfiesPositionBounds(&zero))
    {
      fail(joint->getName() + " cannot be at zero, the generator's reference");
    }
    state.setJointPositions(joint, &zero);
  }
  state.update();
  const auto axis = [&](const moveit::core::JointModel* joint) {
    const auto* revolute = dynamic_cast<const moveit::core::RevoluteJointModel*>(joint);
    if (!revolute)
    {
      fail(joint->getName() + " is not a revolute joint");
    }
    // the joint turns its child link about this axis, so the child's orientation carries it to the world
    return Eigen::Vector3d(state.getGlobalLinkTransform(joint->getChildLinkModel()).linear() * revolute->getAxis());
  };

  const moveit::core::JointModel* pan = joints.front();
  const moveit::core::JointModel* roll = joints.back();
  if (std::abs(std::abs(axis(pan).z()) - 1.0) > kAxisTolerance)
  {
    fail(pan->getName() + "'s axis must be vertical");
  }
  const Eigen::Vector3d pitch = axis(joints[1]);
  if (std::abs(pitch.z()) > kAxisTolerance)
  {
    fail(joints[1]->getName() + "'s axis must be horizontal");
  }
  for (std::size_t i = 2; i + 1 < joints.size(); ++i)
  {
    if (axis(joints[i]).cross(pitch).norm() > kAxisTolerance)
    {
      fail(joints[i]->getName() + "'s axis must be parallel to " + joints[1]->getName() + "'s");
    }
  }
  const Eigen::Isometry3d& gripper = state.getGlobalLinkTransform(roll->getChildLinkModel());
  const Eigen::Vector3d roll_axis = axis(roll);
  if ((roll_axis - gripper.linear().col(2)).norm() > kAxisTolerance || std::abs(roll_axis.dot(pitch)) > kAxisTolerance)
  {
    fail(roll->getName() + " must turn about z of " + roll->getChildLinkModel()->getName() +
         ", across the pitch axes");
  }

  ArmGeometry arm;
  arm.model_frame = model->getModelFrame();
  arm.gripper_frame = roll->getChildLinkModel()->getName();
  const Eigen::Vector3d pan_point = state.getGlobalLinkTransform(pan->getChildLinkModel()).translation();
  arm.pan_axis = pan_point.head<2>();

  // The arm plane holds the roll axis and lies across the pitch axes; its normal is (-sin theta, cos theta, 0) for
  // the plane's azimuth theta, the direction from the pan axis towards the wrist
  const Eigen::Vector3d to_wrist = gripper.translation() - pan_point;
  Eigen::Vector3d forward = pitch.cross(kUp).normalized();
  if (forward.dot(to_wrist) < 0.0)
  {
    forward = -forward;
  }
  const Eigen::Vector3d normal = kUp.cross(forward);
  arm.plane_offset = normal.dot(to_wrist);

  // The jaw angle at wrist_roll = 0: the jaw axis (x of the gripper frame) about the roll axis, from the normal
  const Eigen::Vector3d x = gripper.linear().col(0);
  const Eigen::Vector3d z = gripper.linear().col(2);
  arm.roll_zero = std::atan2(x.dot(z.cross(normal)), x.dot(normal));
  const moveit::core::VariableBounds& bounds = roll->getVariableBounds()[0];
  arm.roll_min = bounds.min_position_;
  arm.roll_max = bounds.max_position_;

  arm.gripper_to_tcp = gripper.inverse() * state.getGlobalLinkTransform(parameters.tcp_frame);
  if (arm.gripper_to_tcp.translation().z() >= 0.0)
  {
    fail(parameters.tcp_frame + " must lie towards -z of " + arm.gripper_frame + " (z points from the fingertips "
         "to the wrist)");
  }

  // The moving jaw hinges on the gripper frame; the fixed jaw is on the other side of the jaw axis
  const std::vector<const moveit::core::JointModel*>& jaw_joints = gripper_group->getActiveJointModels();
  if (jaw_joints.size() != 1 || jaw_joints[0]->getParentLinkModel() != roll->getChildLinkModel())
  {
    fail(parameters.gripper_group + " must be one joint hinged on " + arm.gripper_frame);
  }
  const double hinge_x = jaw_joints[0]->getChildLinkModel()->getJointOriginTransform().translation().x();
  arm.fixed_jaw_side = hinge_x > 0.0 ? -1.0 : 1.0;
  return arm;
}

PoseGenerator::PoseGenerator(const moveit::core::RobotModelConstPtr& model, GraspParameters parameters)
  : arm_(readArmGeometry(model, parameters)), parameters_(std::move(parameters))
{
}

double PoseGenerator::radius(const Eigen::Vector3d& point) const
{
  return (point.head<2>() - arm_.pan_axis).norm();
}

std::optional<double> PoseGenerator::tiltFor(double radius) const
{
  for (std::size_t i = 0; i < parameters_.tilts.size(); ++i)
  {
    if (radius <= parameters_.max_radii[i])
    {
      return parameters_.tilts[i];
    }
  }
  return std::nullopt;
}

std::optional<GripperPose> PoseGenerator::gripperPose(const Eigen::Vector3d& point, const Eigen::Vector3d& target,
                                                      double tilt, double jaw_angle) const
{
  // ADR 0003's correction: the plane passes the point's offset across it, not the target
  const Eigen::Vector2d d = target.head<2>() - arm_.pan_axis;
  const double r = d.norm();
  const double across = point.x() * std::cos(jaw_angle) - point.y() * std::sin(jaw_angle) + arm_.plane_offset;
  if (std::abs(across) >= r)
  {
    return std::nullopt;
  }
  const double theta = std::atan2(d.y(), d.x()) - std::asin(across / r);
  return GripperPose{ gripperFrame(theta, tilt, jaw_angle, point, target), jaw_angle };
}

std::optional<GripperPose> PoseGenerator::topDownGripperPose(const Eigen::Vector3d& point,
                                                             const Eigen::Vector3d& target, double jaw_azimuth) const
{
  // With the jaw axis fixed in the world, the vertical roll axis has one place; the plane must pass it
  const Eigen::Vector3d x(std::cos(jaw_azimuth), std::sin(jaw_azimuth), 0.0);
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.linear() << x, kUp.cross(x), kUp;
  pose.translation() = target - pose.linear() * point;
  const Eigen::Vector2d d = pose.translation().head<2>() - arm_.pan_axis;
  const double r = d.norm();
  if (std::abs(arm_.plane_offset) >= r)
  {
    return std::nullopt;
  }
  const double theta = std::atan2(d.y(), d.x()) - std::asin(arm_.plane_offset / r);
  const Eigen::Vector3d normal(-std::sin(theta), std::cos(theta), 0.0);
  return GripperPose{ pose, std::atan2(x.dot(kUp.cross(normal)), x.dot(normal)) };
}

double PoseGenerator::wristRoll(double jaw_angle) const
{
  return std::remainder(jaw_angle - arm_.roll_zero, 2.0 * M_PI);
}

bool PoseGenerator::withinRollLimits(double wrist_roll) const
{
  return wrist_roll >= arm_.roll_min && wrist_roll <= arm_.roll_max;
}

std::vector<Grasp> PoseGenerator::grasps(const Eigen::Vector3d& block_centre, double block_yaw) const
{
  std::vector<Grasp> result;
  const std::optional<double> tilt = tiltFor(radius(block_centre));
  if (!tilt)
  {
    return result;
  }
  // The point of the gripper frame that goes onto the block's centre: approach_gap from the tcp away from the fixed
  // jaw, so that the tcp stops approach_gap from the centre towards it
  const Eigen::Vector3d point =
      arm_.gripper_to_tcp.translation() - arm_.fixed_jaw_side * parameters_.approach_gap * Eigen::Vector3d::UnitX();
  std::vector<GripperPose> candidates;
  if (std::abs(*tilt) < 1e-9)
  {
    for (int side = 0; side < 4; ++side)  // the jaw axis along each normal of the block's sides
    {
      if (auto candidate = topDownGripperPose(point, block_centre, block_yaw + side * M_PI / 2.0))
      {
        candidates.push_back(*candidate);
      }
    }
  }
  else
  {
    for (double jaw_angle : { 0.0, M_PI })  // the jaw axis along the plane's normal: horizontal
    {
      if (auto candidate = gripperPose(point, block_centre, *tilt, jaw_angle))
      {
        candidates.push_back(*candidate);
      }
    }
  }
  for (const GripperPose& candidate : candidates)
  {
    const double wrist_roll = wristRoll(candidate.jaw_angle);
    if (!withinRollLimits(wrist_roll))
    {
      continue;
    }
    Grasp grasp;
    grasp.tilt = *tilt;
    grasp.jaw_angle = candidate.jaw_angle;
    grasp.wrist_roll = wrist_roll;
    grasp.grasp = candidate.pose * arm_.gripper_to_tcp;
    grasp.pregrasp = moved(grasp.grasp, parameters_.pregrasp_distance * candidate.pose.linear().col(2));
    grasp.lift = moved(grasp.grasp, parameters_.lift_distance * kUp);
    result.push_back(grasp);
  }
  return result;
}

std::vector<Place> PoseGenerator::places(const Eigen::Vector3d& target, const Eigen::Isometry3d& block_in_tcp,
                                         const Eigen::Vector3d& block_size) const
{
  std::vector<Place> result;
  const std::optional<double> tilt = tiltFor(radius(target));
  if (!tilt)
  {
    return result;
  }
  const Eigen::Isometry3d block_in_gripper = arm_.gripper_to_tcp * block_in_tcp;
  for (double jaw_angle : { 0.0, M_PI })
  {
    const double wrist_roll = wristRoll(jaw_angle);
    const std::optional<GripperPose> candidate =
        gripperPose(block_in_gripper.translation(), target, *tilt, jaw_angle);
    if (!candidate || !withinRollLimits(wrist_roll))
    {
      continue;
    }
    // The held block turns with the gripper: its lowest corner lies half its vertical extent below its centre
    const Eigen::Matrix3d block_rotation = candidate->pose.linear() * block_in_gripper.linear();
    const double below_centre = 0.5 * block_rotation.row(2).cwiseAbs().transpose().dot(block_size);
    const Eigen::Vector3d centre = target + (below_centre + parameters_.place_clearance) * kUp;
    Eigen::Isometry3d gripper = candidate->pose;
    gripper.translation() = centre - gripper.linear() * block_in_gripper.translation();

    Place place;
    place.tilt = *tilt;
    place.jaw_angle = jaw_angle;
    place.wrist_roll = wrist_roll;
    place.place = gripper * arm_.gripper_to_tcp;
    place.preplace = moved(place.place, parameters_.lift_distance * kUp);
    place.retreat = moved(place.place, parameters_.pregrasp_distance * gripper.linear().col(2));
    result.push_back(place);
  }
  return result;
}

}  // namespace so101_mtc_task
