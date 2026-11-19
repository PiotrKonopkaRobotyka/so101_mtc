// The pick and place task of so101_mtc (ROB-52): MoveIt Task Constructor (MTC) plans the pick of the block from the
// pose the simulator publishes and its place over a target, with ADR 0003's exactly reachable poses (grasp_poses),
// and move_group executes the solution through its ExecuteTaskSolution capability.
//
// The node keeps the robot model and serves ~/pick_place (so101_mtc_interfaces/srv/PickPlace): each call reads the
// block's latest pose, puts the block into the planning scene, plans the task and executes the cheapest solution.
// The pose comes from the simulator (block_source true_pose, M2) or from the camera's block_detector (camera, M3).
// With pick_place.run_at_start, it does one run at start-up, to the target of its parameters.
//
// Based on mtc_node.cpp of mycobot_mtc_pick_place_demo, from Automatic Addison's myCobot 280 tutorial
// (https://github.com/automaticaddison/mycobot_ros2) by Addison Sears-Collins: the order of the stages and their
// set-up. Its license, BSD-3-Clause:
//
//   Redistribution and use in source and binary forms, with or without modification, are permitted provided that
//   the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice, this list of conditions and the
//     following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
//     following disclaimer in the documentation and/or other materials provided with the distribution.
//   * Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote
//     products derived from this software without specific prior written permission.
//   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
//   WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
//   PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
//   INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
//   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
//   IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/task.h>
#include <mujoco_ros2_control_msgs/msg/free_joint_state_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <yaml-cpp/yaml.h>

#include "so101_mtc_interfaces/srv/pick_place.hpp"
#include "so101_mtc_task/grasp_poses.hpp"
#include "so101_mtc_task/mtc_stages.hpp"

namespace mtc = moveit::task_constructor;
using so101_mtc_interfaces::srv::PickPlace;

namespace so101_mtc_task
{
namespace
{
/// The comments of the stages along a solution, by stage name (ComputeIK keeps its generator's comment).
void collectComments(const mtc::SolutionBase& solution, std::map<std::string, std::string>& comments)
{
  if (solution.creator())
  {
    comments.emplace(solution.creator()->name(), solution.comment());
  }
  if (const auto* sequence = dynamic_cast<const mtc::SolutionSequence*>(&solution))
  {
    for (const mtc::SolutionBase* part : sequence->solutions())
    {
      collectComments(*part, comments);
    }
  }
  else if (const auto* wrapped = dynamic_cast<const mtc::WrappedSolution*>(&solution))
  {
    collectComments(*wrapped->wrapped(), comments);
  }
}

double seconds_since(std::chrono::steady_clock::time_point start)
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
}

}  // namespace

class PickPlaceNode : public rclcpp::Node
{
public:
  explicit PickPlaceNode(const rclcpp::NodeOptions& options) : Node("pick_place", options)
  {
  }

  /// Loads the robot model and the generator, then starts listening; needs shared_from_this, so not in the constructor.
  void start()
  {
    model_loader_ = std::make_unique<robot_model_loader::RobotModelLoader>(shared_from_this(), "robot_description");
    generator_ = std::make_shared<PoseGenerator>(model_loader_->getModel(), loadParameters(*this));
    const YAML::Node task = YAML::LoadFile(ament_index_cpp::get_package_share_directory("so101_mtc_description") +
                                           "/config/task.yaml");
    block_size_ = Eigen::Vector3d::Constant(task["block"]["size"].as<double>());

    // The block's pose keeps arriving while a run holds the other callback group. The node keeps only weak pointers
    // to its callback groups: without these members, a group and its callbacks vanish when start() returns.
    blocks_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    runs_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions subscription_options;
    subscription_options.callback_group = blocks_;
    const std::string source = parameter<std::string>("block_source");
    if (source == "true_pose")
    {
      block_subscription_ = create_subscription<mujoco_ros2_control_msgs::msg::FreeJointStateArray>(
          parameter<std::string>("block_topic"), rclcpp::SensorDataQoS(),
          [this](const mujoco_ros2_control_msgs::msg::FreeJointStateArray& message) { onBlock(message); },
          subscription_options);
    }
    else if (source == "camera")
    {
      // block_detector publishes reliable and latched (transient local)
      camera_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
          parameter<std::string>("block_pose_topic"), rclcpp::QoS(1).reliable().transient_local(),
          [this](const geometry_msgs::msg::PoseStamped& message) { onCameraPose(message); }, subscription_options);
    }
    else
    {
      throw std::invalid_argument("pick_place.block_source must be true_pose or camera, not " + source);
    }
    service_ = create_service<PickPlace>(
        "~/pick_place",
        [this](const std::shared_ptr<PickPlace::Request> request, std::shared_ptr<PickPlace::Response> response) {
          RCLCPP_INFO(get_logger(), "Request: target (%.3f, %.3f)", request->target.x, request->target.y);
          *response = run(Eigen::Vector2d(request->target.x, request->target.y));
        },
        rclcpp::ServicesQoS(), runs_);
    if (parameter<bool>("run_at_start"))
    {
      start_timer_ = create_wall_timer(
          std::chrono::milliseconds(0),
          [this] {
            start_timer_->cancel();
            const std::vector<double> target = parameter<std::vector<double>>("target");
            run(Eigen::Vector2d(target.at(0), target.at(1)));
          },
          runs_);
    }
    RCLCPP_INFO(get_logger(), "Ready: %s, %zu grasp families", service_->get_service_name(),
                generator_->parameters().tilts.size());
  }

private:
  template <typename T>
  T parameter(const std::string& name)
  {
    const std::string full = "pick_place." + name;
    if (!has_parameter(full))
    {
      declare_parameter<T>(full);  // no default: config/pick_place.yaml is the only source
    }
    return get_parameter(full).get_value<T>();
  }

  void onBlock(const mujoco_ros2_control_msgs::msg::FreeJointStateArray& message)
  {
    for (const auto& body : message.free_joints)
    {
      if (body.name == block_id())
      {
        Eigen::Isometry3d pose;
        tf2::fromMsg(body.pose.pose, pose);
        RCLCPP_INFO_ONCE(get_logger(), "The block's pose arrives on %s", block_subscription_->get_topic_name());
        std::lock_guard<std::mutex> lock(mutex_);
        block_ = pose;
        block_stamp_ = rclcpp::Time(message.header.stamp, RCL_ROS_TIME);
      }
    }
  }

  void onCameraPose(const geometry_msgs::msg::PoseStamped& message)
  {
    Eigen::Isometry3d pose;
    tf2::fromMsg(message.pose, pose);
    RCLCPP_INFO_ONCE(get_logger(), "The block's pose arrives from the camera on %s",
                     camera_subscription_->get_topic_name());
    std::lock_guard<std::mutex> lock(mutex_);
    block_ = pose;
    block_stamp_ = rclcpp::Time(message.header.stamp, RCL_ROS_TIME);
  }

  std::string block_id()
  {
    return parameter<std::string>("block_id");
  }

  /// The block's latest pose no older than max_pose_age (simulation time), so that a pose from before a reset of the
  /// block is never used; waits up to `timeout` for one.
  std::optional<Eigen::Isometry3d> latestBlock(std::chrono::seconds timeout)
  {
    const rclcpp::Duration max_age = rclcpp::Duration::from_seconds(parameter<double>("max_pose_age"));
    const auto end = std::chrono::steady_clock::now() + timeout;
    while (rclcpp::ok() && std::chrono::steady_clock::now() < end)
    {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (block_ && block_stamp_ && now() - *block_stamp_ <= max_age)
        {
          return block_;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    return std::nullopt;
  }

  /// The block where the simulator has it, as a box in the planning scene; detached first if a run left it attached.
  void putBlockIntoScene(const Eigen::Isometry3d& pose)
  {
    moveit::planning_interface::PlanningSceneInterface scene;
    if (!scene.getAttachedObjects({ block_id() }).empty())
    {
      moveit_msgs::msg::AttachedCollisionObject detach;
      detach.object.id = block_id();
      detach.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
      scene.applyAttachedCollisionObject(detach);
    }
    moveit_msgs::msg::CollisionObject block;
    block.id = block_id();
    block.header.frame_id = generator_->arm().model_frame;
    block.pose = tf2::toMsg(pose);  // the object's frame is the block's; the box sits at its origin
    shape_msgs::msg::SolidPrimitive box;
    box.type = shape_msgs::msg::SolidPrimitive::BOX;
    box.dimensions = { block_size_.x(), block_size_.y(), block_size_.z() };
    block.primitives.push_back(box);
    block.primitive_poses.push_back(tf2::toMsg(Eigen::Isometry3d::Identity()));
    block.operation = moveit_msgs::msg::CollisionObject::ADD;
    scene.applyCollisionObject(block);
  }

  /// The MTC task: current state → open → connect → pick → connect → place → home.
  mtc::Task makeTask(const std::vector<Grasp>& grasps, const Eigen::Vector3d& target)
  {
    const GraspParameters& poses = generator_->parameters();
    const std::string arm = poses.arm_group;
    const std::string hand = poses.gripper_group;
    const std::string tcp = poses.tcp_frame;
    const std::string world = generator_->arm().model_frame;
    const std::string block = block_id();
    const double fraction = parameter<double>("min_fraction");

    mtc::Task task;
    task.stages()->setName("so101 pick and place");
    task.setRobotModel(model_loader_->getModel());
    task.setTimeout(parameter<double>("planning_timeout"));
    task.setProperty("group", arm);
    task.setProperty("eef", parameter<std::string>("end_effector"));
    task.setProperty("ik_frame", tcp);

    // planners: OMPL for free motions, straight lines for the approach, lift, lower and retreat, joint
    // interpolation for the gripper
    auto ompl = std::make_shared<mtc::solvers::PipelinePlanner>(shared_from_this(), "ompl", "RRTConnectkConfigDefault");
    auto cartesian = std::make_shared<mtc::solvers::CartesianPath>();
    for (const auto& planner : std::vector<mtc::solvers::PlannerInterfacePtr>{ ompl, cartesian })
    {
      planner->setMaxVelocityScalingFactor(parameter<double>("velocity_scaling"));
      planner->setMaxAccelerationScalingFactor(parameter<double>("acceleration_scaling"));
    }
    cartesian->setStepSize(parameter<double>("cartesian_step"));
    auto interpolation = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
    // The free motions move the arm only: the gripper is open at both ends of the move to pick and closed at both
    // ends of the move to place. move_group executes a sub-trajectory through the SRDF group that actuates exactly
    // its joints, and no group holds the arm and the gripper together.
    const mtc::stages::Connect::GroupPlannerVector free_motion{ { arm, ompl } };
    const auto direction = [](const std::string& frame, double z) {
      geometry_msgs::msg::Vector3Stamped vector;
      vector.header.frame_id = frame;
      vector.vector.z = z;
      return vector;
    };
    const auto gripper = [&](const std::string& name, const std::string& state) {
      auto stage = std::make_unique<mtc::stages::MoveTo>(name, interpolation);
      stage->setGroup(hand);
      stage->setGoal(state);
      return stage;
    };
    const auto straight = [&](const std::string& name, const geometry_msgs::msg::Vector3Stamped& vector,
                              double distance) {
      auto stage = std::make_unique<mtc::stages::MoveRelative>(name, cartesian);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->properties().set("marker_ns", name);
      stage->setIKFrame(tcp);
      stage->setMinMaxDistance(fraction * distance, distance);
      stage->setDirection(vector);
      return stage;
    };
    const auto allow = [&](const std::string& name, const std::vector<std::string>& links, bool allowed) {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(name);
      stage->allowCollisions(std::vector<std::string>{ block }, links, allowed);
      return stage;
    };
    const auto pose_ik = [&](const std::string& name, std::unique_ptr<mtc::Stage> generator) {
      auto stage = std::make_unique<mtc::stages::ComputeIK>(name, std::move(generator));
      stage->setMaxIKSolutions(2);
      stage->setMinSolutionDistance(1.0);
      stage->setIKFrame(tcp);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      stage->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      return stage;
    };
    const std::vector<std::string> hand_links = parameter<std::vector<std::string>>("hand_links");
    const std::string support = parameter<std::string>("support_link");

    task.add(std::make_unique<mtc::stages::CurrentState>("current state"));
    // the block rests on the table, slightly sunk into it in MuJoCo, until the lift (each scene change is checked
    // for collisions, so this comes first)
    task.add(allow("allow block-table collisions", { support }, true));
    // move_group's scene keeps what the executed stages changed: a run stopped between "allow" and "forbid" would
    // leave the hand free to pass through the block
    task.add(allow("reset hand-block collisions", hand_links, false));
    auto open = gripper("open gripper", parameter<std::string>("gripper_open"));
    mtc::Stage* opened = open.get();
    task.add(std::move(open));
    auto to_pick = std::make_unique<mtc::stages::Connect>("move to pick", free_motion);
    to_pick->setTimeout(parameter<double>("connect_timeout"));
    to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(to_pick));

    mtc::Stage* attached = nullptr;
    {
      auto pick = std::make_unique<mtc::SerialContainer>("pick block");
      task.properties().exposeTo(pick->properties(), { "eef", "group", "ik_frame" });
      pick->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });
      // along the approach (z of the tcp) onto the grasp, from the pre-grasp
      pick->insert(straight("approach", direction(tcp, 1.0), poses.pregrasp_distance));
      auto grasps_stage = std::make_unique<GenerateGraspPoses>("grasp poses", grasps);
      grasps_stage->setMonitoredStage(opened);
      pick->insert(pose_ik("grasp pose IK", std::move(grasps_stage)));
      pick->insert(allow("allow hand-block collisions", hand_links, true));
      pick->insert(gripper("close gripper", parameter<std::string>("gripper_closed")));
      // Closing pushes the block onto the fixed jaw and turns its sides to the jaws: in MuJoCo it ends centred on
      // the tcp and aligned with it (ADR 0003's tcp). The block is attached there, not where it stood, so that the
      // held block's pose is the one the place needs. A message with the box in the tcp frame replaces the world
      // object of the same name, as MTC's own attachObject does without a box (moving the world object instead
      // would move it in the parent scenes too, which share it).
      auto attach = std::make_unique<mtc::stages::ModifyPlanningScene>("attach block in the hand");
      const Eigen::Vector3d size = block_size_;
      attach->setCallback([block, tcp, size](const planning_scene::PlanningScenePtr& scene, const mtc::PropertyMap&) {
        moveit_msgs::msg::AttachedCollisionObject held;
        held.link_name = tcp;
        held.object.id = block;
        held.object.header.frame_id = tcp;
        held.object.pose.orientation.w = 1.0;
        shape_msgs::msg::SolidPrimitive box;
        box.type = shape_msgs::msg::SolidPrimitive::BOX;
        box.dimensions = { size.x(), size.y(), size.z() };
        held.object.primitives.push_back(box);
        held.object.primitive_poses.emplace_back().orientation.w = 1.0;
        held.object.operation = moveit_msgs::msg::CollisionObject::ADD;
        scene->processAttachedCollisionObjectMsg(held);
      });
      attached = attach.get();
      pick->insert(std::move(attach));
      pick->insert(straight("lift", direction(world, 1.0), poses.lift_distance));
      pick->insert(allow("forbid block-table collisions", { support }, false));
      task.add(std::move(pick));
    }

    auto to_place = std::make_unique<mtc::stages::Connect>("move to place", free_motion);
    to_place->setTimeout(parameter<double>("connect_timeout"));
    to_place->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(to_place));

    {
      auto place = std::make_unique<mtc::SerialContainer>("place block");
      task.properties().exposeTo(place->properties(), { "eef", "group", "ik_frame" });
      place->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });
      // straight down onto the place, from the pre-place
      place->insert(straight("lower", direction(world, -1.0), poses.lift_distance));
      auto places_stage = std::make_unique<GeneratePlacePoses>("place poses", generator_, target, block_size_);
      places_stage->setObject(block);
      places_stage->setMonitoredStage(attached);
      place->insert(pose_ik("place pose IK", std::move(places_stage)));
      place->insert(gripper("release", parameter<std::string>("gripper_open")));
      auto detach = std::make_unique<mtc::stages::ModifyPlanningScene>("detach block");
      detach->detachObject(block, tcp);
      place->insert(std::move(detach));
      // back along the approach (-z of the tcp); the fixed jaw slides off the block's side it rests on
      place->insert(straight("retreat", direction(tcp, -1.0), poses.pregrasp_distance));
      place->insert(allow("forbid hand-block collisions", hand_links, false));
      task.add(std::move(place));
    }

    auto home = std::make_unique<mtc::stages::MoveTo>("move home", ompl);
    home->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    home->setGoal(parameter<std::string>("arm_home"));
    task.add(std::move(home));
    return task;
  }

  /// One pick and place to `target` (x, y on the table): plan, then execute the cheapest solution.
  PickPlace::Response run(const Eigen::Vector2d& target_xy)
  {
    PickPlace::Response response;
    const auto fail = [&](const std::string& stage) {
      response.failed_stage = stage;
      RCLCPP_ERROR(get_logger(), "Pick and place failed: %s", stage.c_str());
      return response;
    };
    const std::optional<Eigen::Isometry3d> block = latestBlock(std::chrono::seconds(10));
    if (!block)
    {
      const bool camera = parameter<std::string>("block_source") == "camera";
      char age[32];
      std::snprintf(age, sizeof(age), "%.1f", parameter<double>("max_pose_age"));
      return fail(std::string("perception: no block pose younger than ") + age + " s on " +
                  parameter<std::string>(camera ? "block_pose_topic" : "block_topic"));
    }
    const Eigen::Vector3d centre = block->translation();
    const double yaw = std::atan2(block->linear()(1, 0), block->linear()(0, 0));
    const std::vector<Grasp> grasps = generator_->grasps(centre, yaw, block_size_.x());
    if (grasps.empty())
    {
      return fail("no grasp candidates: the block is " + std::to_string(generator_->radius(centre)) +
                  " m from the shoulder_pan axis");
    }
    const Eigen::Vector3d target(target_xy.x(), target_xy.y(), 0.0);
    RCLCPP_INFO(get_logger(), "Block at (%.3f, %.3f), yaw %.1f deg, %.3f m from the pan axis: %zu grasp candidates; "
                "target (%.3f, %.3f)",
                centre.x(), centre.y(), yaw * 180.0 / M_PI, generator_->radius(centre), grasps.size(), target.x(),
                target.y());
    putBlockIntoScene(*block);

    task_ = std::make_unique<mtc::Task>(makeTask(grasps, target));
    const auto planning_start = std::chrono::steady_clock::now();
    try
    {
      task_->init();
    }
    catch (const mtc::InitStageException& error)
    {
      std::ostringstream text;
      text << error;
      return fail("task initialisation: " + text.str());
    }
    const moveit::core::MoveItErrorCode planned = task_->plan(parameter<int>("max_solutions"));
    response.planning_time = seconds_since(planning_start);
    if (!planned || task_->solutions().empty())
    {
      std::ostringstream text;
      task_->explainFailure(text);
      RCLCPP_ERROR(get_logger(), "Planning failed after %.2f s", response.planning_time);
      return fail(text.str());
    }
    const mtc::SolutionBase& solution = *task_->solutions().front();
    std::map<std::string, std::string> comments;
    collectComments(solution, comments);
    response.grasp = comments["grasp pose IK"];
    response.place = comments["place pose IK"];
    RCLCPP_INFO(get_logger(), "Planned in %.2f s, %zu solutions; the cheapest (cost %.2f) grasps %s, places %s",
                response.planning_time, task_->solutions().size(), solution.cost(), response.grasp.c_str(),
                response.place.c_str());
    task_->introspection().publishSolution(solution);
    if (!parameter<bool>("execute"))
    {
      response.success = true;
      return response;
    }
    const auto execution_start = std::chrono::steady_clock::now();
    const moveit::core::MoveItErrorCode executed = task_->execute(solution);
    response.execution_time = seconds_since(execution_start);
    if (executed.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    {
      return fail("execution: MoveIt error code " + std::to_string(executed.val));
    }
    RCLCPP_INFO(get_logger(), "Executed in %.2f s", response.execution_time);
    response.success = true;
    return response;
  }

  std::unique_ptr<robot_model_loader::RobotModelLoader> model_loader_;
  std::shared_ptr<PoseGenerator> generator_;
  Eigen::Vector3d block_size_ = Eigen::Vector3d::Zero();
  rclcpp::CallbackGroup::SharedPtr blocks_;  // the block's pose
  rclcpp::CallbackGroup::SharedPtr runs_;    // the runs: the service and the run at start-up, one at a time
  std::mutex mutex_;
  std::optional<Eigen::Isometry3d> block_;
  std::optional<rclcpp::Time> block_stamp_;
  rclcpp::Subscription<mujoco_ros2_control_msgs::msg::FreeJointStateArray>::SharedPtr block_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr camera_subscription_;
  rclcpp::Service<PickPlace>::SharedPtr service_;
  rclcpp::TimerBase::SharedPtr start_timer_;
  std::unique_ptr<mtc::Task> task_;  // the last task stays, so that RViz's MTC panel can inspect it
};

}  // namespace so101_mtc_task

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<so101_mtc_task::PickPlaceNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;
  int result = 0;
  try
  {
    node->start();
    executor.add_node(node);
    executor.spin();
  }
  catch (const std::exception& error)
  {
    RCLCPP_FATAL(node->get_logger(), "%s", error.what());
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
