// The generator stages of mtc_stages.hpp.
//
// Based on GeneratePose::compute and GeneratePlacePose::compute of MoveIt Task Constructor
// (https://github.com/moveit/moveit_task_constructor, core/src/stages/generate_pose.cpp and generate_place_pose.cpp,
// Robert Haschke, Michael Goerner): how a monitored solution's scene becomes spawned states with a "target_pose".
// Their license, BSD-3-Clause:
//
//   Copyright (c) 2017, Bielefeld + Hamburg University; Copyright (c) 2019, Bielefeld University.
//   All rights reserved.
//   Redistribution and use in source and binary forms, with or without modification, are permitted provided that
//   the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice, this list of conditions and the
//     following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
//     following disclaimer in the documentation and/or other materials provided with the distribution.
//   * Neither the name of Bielefeld University nor the names of its contributors may be used to endorse or promote
//     products derived from this software without specific prior written permission.
//   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
//   WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
//   PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
//   GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
//   LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
//   OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
