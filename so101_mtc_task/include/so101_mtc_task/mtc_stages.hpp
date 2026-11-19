// MTC generator stages that hand the poses of grasp_poses (ADR 0003) to ComputeIK. They follow the pattern of MTC's
// own GeneratePose and GeneratePlacePose: a monitored stage's solution gives the planning scene, and each pose is
// spawned as a state with the property "target_pose", which a ComputeIK wrapper turns into joint values.
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <moveit/task_constructor/stages/generate_place_pose.h>
#include <moveit/task_constructor/stages/generate_pose.h>

#include "so101_mtc_task/grasp_poses.hpp"

namespace so101_mtc_task
{

/// "tilt 15 deg, jaw angle 180.0 deg": a candidate's comment in RViz's MTC panel and in the logs.
std::string describe(double tilt, double jaw_angle);

/// Spawns the tcp poses of a block's grasps, one per family and jaw side. Monitor the stage that opens the gripper:
/// the spawned states then hold the open hand, which ComputeIK checks for collisions.
class GenerateGraspPoses : public moveit::task_constructor::stages::GeneratePose
{
public:
  GenerateGraspPoses(const std::string& name, std::vector<Grasp> grasps);
  void compute() override;

private:
  std::vector<Grasp> grasps_;
};

/// Spawns the tcp poses that put the held block over a target. Monitor the stage that attaches the block: for each
/// of its solutions, the block's pose in the tcp frame comes from the attached body, so every grasp gets its own
/// places, with the block's centre over the target and its lowest corner just above the table.
class GeneratePlacePoses : public moveit::task_constructor::stages::GeneratePlacePose
{
public:
  GeneratePlacePoses(const std::string& name, std::shared_ptr<const PoseGenerator> generator,
                     const Eigen::Vector3d& target, const Eigen::Vector3d& block_size);
  void compute() override;

private:
  std::shared_ptr<const PoseGenerator> generator_;
  Eigen::Vector3d target_;
  Eigen::Vector3d block_size_;
};

}  // namespace so101_mtc_task
