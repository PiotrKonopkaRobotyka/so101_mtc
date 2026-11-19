#include "so101_mtc_task/mtc_stages.hpp"

#include <cmath>
#include <cstdio>
#include <utility>

#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit/task_constructor/storage.h>
#include <rviz_marker_tools/marker_creation.h>
#include <tf2_eigen/tf2_eigen.hpp>

namespace mtc = moveit::task_constructor;

namespace so101_mtc_task
{
namespace
{
geometry_msgs::msg::PoseStamped stamped(const Eigen::Isometry3d& pose, const std::string& frame)
{
  geometry_msgs::msg::PoseStamped message;
  message.header.frame_id = frame;
  message.pose = tf2::toMsg(pose);
  return message;
}

/// One spawned state's trajectory: no motion, no cost, a comment and a frame marker at the pose for RViz.
mtc::SubTrajectory spawnedPose(const geometry_msgs::msg::PoseStamped& pose, const std::string& comment)
{
  mtc::SubTrajectory trajectory;
  trajectory.setCost(0.0);
  trajectory.setComment(comment);
  rviz_marker_tools::appendFrame(trajectory.markers(), pose, 0.05, "tcp frame");
  return trajectory;
}

}  // namespace

std::string describe(double tilt, double jaw_angle)
{
  char text[64];
  std::snprintf(text, sizeof(text), "tilt %.0f deg, jaw angle %.1f deg", tilt * 180.0 / M_PI, jaw_angle * 180.0 / M_PI);
  return text;
}

GenerateGraspPoses::GenerateGraspPoses(const std::string& name, std::vector<Grasp> grasps)
  : GeneratePose(name), grasps_(std::move(grasps))
{
  setPose(geometry_msgs::msg::PoseStamped());  // GeneratePose's own property, unused here
}

void GenerateGraspPoses::compute()
{
  if (upstream_solutions_.empty())
  {
    return;
  }
  const mtc::SolutionBase& upstream = *upstream_solutions_.pop();
  const planning_scene::PlanningScenePtr scene = upstream.end()->scene()->diff();
  for (const Grasp& grasp : grasps_)
  {
    const geometry_msgs::msg::PoseStamped pose = stamped(grasp.grasp, scene->getPlanningFrame());
    mtc::InterfaceState state(scene);
    forwardProperties(*upstream.end(), state);
    state.properties().set("target_pose", pose);
    spawn(std::move(state), spawnedPose(pose, describe(grasp.tilt, grasp.jaw_angle)));
  }
}

GeneratePlacePoses::GeneratePlacePoses(const std::string& name, std::shared_ptr<const PoseGenerator> generator,
                                       const Eigen::Vector3d& target, const Eigen::Vector3d& block_size)
  : GeneratePlacePose(name), generator_(std::move(generator)), target_(target), block_size_(block_size)
{
  setPose(geometry_msgs::msg::PoseStamped());  // GeneratePlacePose's own target, unused here
}

void GeneratePlacePoses::compute()
{
  if (upstream_solutions_.empty())
  {
    return;
  }
  const mtc::SolutionBase& upstream = *upstream_solutions_.pop();
  const planning_scene::PlanningScenePtr scene = upstream.end()->scene()->diff();
  const moveit::core::RobotState& robot = scene->getCurrentState();
  // GeneratePlacePose::onNewSolution let only solutions with the block attached through
  const moveit::core::AttachedBody* block = robot.getAttachedBody(properties().get<std::string>("object"));
  if (!block)
  {
    return;
  }
  const Eigen::Isometry3d block_in_tcp =
      robot.getGlobalLinkTransform(generator_->parameters().tcp_frame).inverse() * block->getGlobalPose();
  for (const Place& place : generator_->places(target_, block_in_tcp, block_size_))
  {
    const geometry_msgs::msg::PoseStamped pose = stamped(place.place, scene->getPlanningFrame());
    mtc::InterfaceState state(scene);
    forwardProperties(*upstream.end(), state);
    state.properties().set("target_pose", pose);
    spawn(std::move(state), spawnedPose(pose, describe(place.tilt, place.jaw_angle)));
  }
}

}  // namespace so101_mtc_task
