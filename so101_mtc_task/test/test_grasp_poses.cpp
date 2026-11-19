// The grasp and place pose generator against MoveIt's own kinematics, without a simulator or move_group.
//
// The robot model is so101_mtc_description's URDF (expanded at build time) with so101_mtc_moveit_config's SRDF and
// KDL from its kinematics.yaml, loaded the way move_group loads them. Every pose must have a KDL solution that is free
// of collisions with the robot and the table (the gripper open), and MoveIt's FK must return the pose from it.
//
//     colcon test --packages-select so101_mtc_task && colcon test-result --verbose
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <gtest/gtest.h>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

#include "so101_mtc_task/grasp_poses.hpp"

namespace so101_mtc_task
{
namespace
{
constexpr double kDeg = M_PI / 180.0;
constexpr double kMaxPositionError = 1e-4;     // m, ROB-51's acceptance criterion
constexpr double kMaxAngleError = 0.1 * kDeg;  // rad
constexpr double kExact = 1e-9;                // m or rad: the generator's own geometry

std::string readFile(const std::string& path)
{
  std::ifstream file(path);
  if (!file)
  {
    throw std::runtime_error("cannot read " + path);
  }
  std::stringstream text;
  text << file.rdbuf();
  return text.str();
}

/// kinematics.yaml as move_group receives it: robot_description_kinematics.<group>.<key>
std::vector<rclcpp::Parameter> kinematicsParameters(const std::string& path)
{
  std::vector<rclcpp::Parameter> parameters;
  for (const auto& group : YAML::LoadFile(path))
  {
    for (const auto& entry : group.second)
    {
      const std::string name =
          "robot_description_kinematics." + group.first.as<std::string>() + "." + entry.first.as<std::string>();
      const std::string text = entry.second.as<std::string>();
      char* end = nullptr;
      const double number = std::strtod(text.c_str(), &end);
      if (end != text.c_str() && *end == '\0')
      {
        parameters.emplace_back(name, number);
      }
      else
      {
        parameters.emplace_back(name, text);
      }
    }
  }
  return parameters;
}

double angleBetween(const Eigen::Matrix3d& a, const Eigen::Matrix3d& b)
{
  return Eigen::AngleAxisd(a.transpose() * b).angle();
}

/// Counts of an IK sweep, for the summary and the assertions.
struct Sweep
{
  int poses = 0;
  double max_position_error = 0.0;
  double max_angle_error = 0.0;
  double seconds = 0.0;
  std::vector<std::string> failures;
};

}  // namespace

/// The robot model with KDL, its collision scene and the generator, loaded once for all tests.
class GraspPosesTest : public testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    const std::string moveit_config = ament_index_cpp::get_package_share_directory("so101_mtc_moveit_config");
    std::vector<rclcpp::Parameter> overrides = kinematicsParameters(moveit_config + "/config/kinematics.yaml");
    overrides.emplace_back("robot_description", readFile(TEST_URDF));
    overrides.emplace_back("robot_description_semantic", readFile(moveit_config + "/config/so101_mtc.srdf"));
    rclcpp::NodeOptions options;
    options.parameter_overrides(overrides)
        .automatically_declare_parameters_from_overrides(true)
        .arguments({ "--ros-args", "--params-file", GRASP_POSES_YAML });
    node_ = std::make_shared<rclcpp::Node>("test_grasp_poses", options);
    loader_ = std::make_unique<robot_model_loader::RobotModelLoader>(node_, "robot_description", true);
    model_ = loader_->getModel();
    scene_ = std::make_shared<planning_scene::PlanningScene>(model_);
    generator_ = std::make_unique<PoseGenerator>(model_, loadParameters(*node_));

    const YAML::Node task = YAML::LoadFile(ament_index_cpp::get_package_share_directory("so101_mtc_description") +
                                           "/config/task.yaml");
    block_size_ = task["block"]["size"].as<double>();
    area_centre_ = task["area"]["centre"].as<std::vector<double>>();
    area_radius_ = task["area"]["radius"].as<std::vector<double>>();
    area_azimuth_ = task["area"]["azimuth"].as<std::vector<double>>();
  }

  static void TearDownTestSuite()
  {
    generator_.reset();
    scene_.reset();
    model_.reset();
    loader_.reset();  // after the model: the loader owns the KDL plugin's library
    node_.reset();
  }

  /// A point of the task area (task.yaml), by its distance and azimuth (degrees) about the area's centre.
  static Eigen::Vector3d areaPoint(double radius, double azimuth, double z)
  {
    return { area_centre_[0] + radius * std::cos(azimuth * kDeg), area_centre_[1] + radius * std::sin(azimuth * kDeg),
             z };
  }

  /// Distances and azimuths covering the task area, ends included.
  static std::vector<std::pair<double, double>> areaGrid(double radius_step, double azimuth_step)
  {
    std::vector<std::pair<double, double>> grid;
    const int radii = int(std::round((area_radius_[1] - area_radius_[0]) / radius_step));
    const int azimuths = int(std::round((area_azimuth_[1] - area_azimuth_[0]) / azimuth_step));
    for (int i = 0; i <= radii; ++i)
    {
      for (int j = 0; j <= azimuths; ++j)
      {
        grid.emplace_back(area_radius_[0] + i * radius_step, area_azimuth_[0] + j * azimuth_step);
      }
    }
    return grid;
  }

  /// home with the gripper open, where every approach starts
  static moveit::core::RobotState home()
  {
    moveit::core::RobotState state(model_);
    state.setToDefaultValues();
    state.setToDefaultValues(model_->getJointModelGroup(generator_->parameters().arm_group), "home");
    state.setToDefaultValues(model_->getJointModelGroup(generator_->parameters().gripper_group), "open");
    state.update();
    return state;
  }

  /// MoveIt's IK (KDL, its timeout from kinematics.yaml) for a tcp pose, starting from `state` and leaving the solution
  /// in it; the solution must be free of collisions, and FK must return the pose. Returns an error, or "" on success.
  static std::string solve(const Eigen::Isometry3d& pose, moveit::core::RobotState& state, Sweep& sweep)
  {
    const auto collision_free = [](moveit::core::RobotState* candidate, const moveit::core::JointModelGroup* group,
                                   const double* values) {
      candidate->setJointGroupPositions(group, values);
      candidate->update();
      return !scene_->isStateColliding(*candidate);
    };
    const GraspParameters& parameters = generator_->parameters();
    const auto start = std::chrono::steady_clock::now();
    const bool solved = state.setFromIK(model_->getJointModelGroup(parameters.arm_group), pose, parameters.tcp_frame,
                                        0.0, collision_free);
    sweep.seconds += std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    ++sweep.poses;
    if (!solved)
    {
      return "no collision-free IK";
    }
    state.update();
    const Eigen::Isometry3d& reached = state.getGlobalLinkTransform(parameters.tcp_frame);
    const double position_error = (reached.translation() - pose.translation()).norm();
    const double angle_error = angleBetween(reached.linear(), pose.linear());
    sweep.max_position_error = std::max(sweep.max_position_error, position_error);
    sweep.max_angle_error = std::max(sweep.max_angle_error, angle_error);
    if (position_error > kMaxPositionError || angle_error > kMaxAngleError)
    {
      return "FK off by " + std::to_string(position_error * 1e3) + " mm, " + std::to_string(angle_error / kDeg) +
             " deg";
    }
    return "";
  }

  static double wristRoll(const moveit::core::RobotState& state)
  {
    const auto& joints = model_->getJointModelGroup(generator_->parameters().arm_group)->getActiveJointModels();
    return state.getVariablePosition(joints.back()->getName());
  }

  /// The gripper frame of a tcp pose.
  static Eigen::Isometry3d gripperOf(const Eigen::Isometry3d& tcp)
  {
    return tcp * generator_->arm().gripper_to_tcp.inverse();
  }

  static void report(const std::string& name, const Sweep& sweep)
  {
    std::cout << name << ": " << sweep.poses << " IK solves, " << sweep.failures.size() << " failed, max FK error "
              << sweep.max_position_error * 1e3 << " mm / " << sweep.max_angle_error / kDeg << " deg, mean "
              << (sweep.poses ? sweep.seconds / sweep.poses * 1e3 : 0.0) << " ms per solve" << std::endl;
    for (const std::string& failure : sweep.failures)
    {
      std::cout << "  " << failure << std::endl;
    }
  }

  static inline rclcpp::Node::SharedPtr node_;
  static inline std::unique_ptr<robot_model_loader::RobotModelLoader> loader_;
  static inline moveit::core::RobotModelPtr model_;
  static inline planning_scene::PlanningScenePtr scene_;
  static inline std::unique_ptr<PoseGenerator> generator_;
  static inline double block_size_ = 0.0;
  static inline std::vector<double> area_centre_, area_radius_, area_azimuth_;
};

// The facts the generator reads from the URDF agree with ADR 0003's measurements.
TEST_F(GraspPosesTest, ArmFactsFromTheUrdfMatchAdr0003)
{
  const ArmGeometry& arm = generator_->arm();
  EXPECT_EQ(arm.model_frame, "world");
  EXPECT_EQ(arm.gripper_frame, "gripper_link");
  EXPECT_NEAR(arm.pan_axis.x(), 0.0388, 1e-4);  // 38.8 mm in front of base_link
  EXPECT_NEAR(arm.pan_axis.y(), 0.0, 1e-6);
  EXPECT_NEAR(arm.plane_offset, -0.18e-3, 0.01e-3);                    // e = -0.18 mm
  EXPECT_NEAR(generator_->wristRoll(0.0), 92.8 * kDeg, 0.05 * kDeg);  // wrist_roll = beta + 92.8 deg
  EXPECT_NEAR(arm.roll_min, -157.2 * kDeg, 0.05 * kDeg);
  EXPECT_NEAR(arm.roll_max, 162.8 * kDeg, 0.05 * kDeg);
  EXPECT_LT((arm.gripper_to_tcp.translation() - Eigen::Vector3d(0.0039, -0.0002, -0.0958)).norm(), kExact);
  EXPECT_EQ(arm.fixed_jaw_side, -1.0);  // the fixed jaw's face at x = -8.6 mm
}

// The generator's own geometry, without IK: families, jaw directions, the approach gap and the waypoints.
TEST_F(GraspPosesTest, GraspGeometry)
{
  const ArmGeometry& arm = generator_->arm();
  const GraspParameters& parameters = generator_->parameters();
  int index = 0;
  for (const auto& [r, azimuth] : areaGrid(0.005, 5.0))
  {
    const double yaw = std::fmod(index++ * 23.0, 90.0) * kDeg;
    const Eigen::Vector3d block = areaPoint(r, azimuth, block_size_ / 2.0);
    const std::vector<Grasp> grasps = generator_->grasps(block, yaw);
    ASSERT_FALSE(grasps.empty()) << "no grasp at r " << r << ", azimuth " << azimuth;
    for (const Grasp& grasp : grasps)
    {
      const Eigen::Isometry3d gripper = gripperOf(grasp.grasp);
      const Eigen::Vector3d x = gripper.linear().col(0);
      const Eigen::Vector3d z = gripper.linear().col(2);
      EXPECT_EQ(grasp.tilt, *generator_->tiltFor(generator_->radius(block)));
      EXPECT_NEAR(z.z(), std::cos(grasp.tilt), kExact);
      if (grasp.tilt == 0.0)  // the jaw axis along a normal of the block's sides
      {
        EXPECT_NEAR(std::sin(2.0 * (std::atan2(x.y(), x.x()) - yaw)), 0.0, kExact);
      }
      else  // horizontal jaws
      {
        EXPECT_NEAR(x.z(), 0.0, kExact);
      }
      EXPECT_TRUE(generator_->withinRollLimits(grasp.wrist_roll));
      // the tcp approach_gap from the block's centre towards the fixed jaw
      const Eigen::Vector3d expected_tcp = block + arm.fixed_jaw_side * parameters.approach_gap * x;
      EXPECT_LT((grasp.grasp.translation() - expected_tcp).norm(), kExact);
      EXPECT_LT((grasp.pregrasp.translation() - grasp.grasp.translation() - parameters.pregrasp_distance * z).norm(),
                kExact);
      EXPECT_LT((grasp.lift.translation() - grasp.grasp.translation() - Eigen::Vector3d(0, 0, parameters.lift_distance))
                    .norm(),
                kExact);
      EXPECT_LT(angleBetween(grasp.pregrasp.linear(), grasp.grasp.linear()), kExact);
      EXPECT_LT(angleBetween(grasp.lift.linear(), grasp.grasp.linear()), kExact);
    }
  }
}

// Over the whole task area, every grasp, pre-grasp and lift solves with KDL, and FK returns it; the predicted
// wrist_roll is the solution's. Top-down, every jaw direction left out for wrist_roll's limits has no solution.
TEST_F(GraspPosesTest, GraspsSolveWithKdlOverTheTaskArea)
{
  const ArmGeometry& arm = generator_->arm();
  const GraspParameters& parameters = generator_->parameters();
  const Eigen::Vector3d point =
      arm.gripper_to_tcp.translation() - arm.fixed_jaw_side * parameters.approach_gap * Eigen::Vector3d::UnitX();
  Sweep sweep, gap;
  std::map<std::string, int> sides;  // solved grasps per family and jaw side
  int index = 0;
  for (const auto& [r, azimuth] : areaGrid(0.005, 5.0))
  {
    const double yaw = std::fmod(index++ * 23.0, 90.0) * kDeg;
    const Eigen::Vector3d block = areaPoint(r, azimuth, block_size_ / 2.0);
    std::ostringstream where;
    where << "r " << r << " azimuth " << azimuth << " yaw " << yaw / kDeg;
    for (const Grasp& grasp : generator_->grasps(block, yaw))
    {
      const std::string name = where.str() + " tilt " + std::to_string(grasp.tilt / kDeg).substr(0, 4) +
                               " beta " + std::to_string(grasp.jaw_angle / kDeg).substr(0, 6);
      moveit::core::RobotState state = home();
      std::string error = solve(grasp.grasp, state, sweep);
      if (error.empty() && std::abs(wristRoll(state) - grasp.wrist_roll) > kMaxAngleError)
      {
        error = "wrist_roll " + std::to_string(wristRoll(state) / kDeg) + " deg, predicted " +
                std::to_string(grasp.wrist_roll / kDeg);
      }
      const moveit::core::RobotState at_grasp = state;
      for (const auto& [waypoint, pose] : { std::pair{ "pre-grasp", grasp.pregrasp }, std::pair{ "lift", grasp.lift } })
      {
        if (!error.empty())
        {
          break;
        }
        state = at_grasp;
        error = solve(pose, state, sweep);
        if (!error.empty())
        {
          error = std::string(waypoint) + ": " + error;
        }
      }
      if (error.empty())
      {
        ++sides[std::to_string(int(std::round(grasp.tilt / kDeg))) + (std::cos(grasp.jaw_angle) >= 0 ? "+" : "-")];
      }
      else
      {
        sweep.failures.push_back(name + ": " + error);
      }
    }
    if (generator_->tiltFor(generator_->radius(block)) == 0.0)
    {
      for (int side = 0; side < 4; ++side)
      {
        const auto candidate = generator_->topDownGripperPose(point, block, yaw + side * M_PI / 2.0);
        ASSERT_TRUE(candidate.has_value());
        if (generator_->withinRollLimits(generator_->wristRoll(candidate->jaw_angle)))
        {
          continue;
        }
        moveit::core::RobotState state = home();
        if (solve(candidate->pose * arm.gripper_to_tcp, state, gap).empty())
        {
          gap.failures.push_back(where.str() + " beta " + std::to_string(candidate->jaw_angle / kDeg) +
                                 ": solved although wrist_roll is out of its limits");
        }
      }
    }
  }
  report("grasps", sweep);
  report("jaw sides out of wrist_roll's limits", gap);
  for (const auto& [side, count] : sides)
  {
    std::cout << "  solved grasps, tilt " << side << ": " << count << std::endl;
  }
  EXPECT_TRUE(sweep.failures.empty());
  EXPECT_TRUE(gap.failures.empty());
  EXPECT_GT(gap.poses, 0) << "no top-down jaw direction fell into wrist_roll's gap";
  for (const char* side : { "0+", "0-", "15+", "15-", "30+", "30-" })
  {
    EXPECT_GT(sides[side], 0) << "no solved grasp with tilt and jaw side " << side;
  }
}

// The place: over the task area, for blocks held after a top-down, a 15° and a 30° grasp, the pre-place, place and
// retreat solve; the block's centre is over the target and its lowest corner place_clearance above the table.
TEST_F(GraspPosesTest, PlacesSolveWithKdlOverTheTaskArea)
{
  const GraspParameters& parameters = generator_->parameters();
  const Eigen::Vector3d size = Eigen::Vector3d::Constant(block_size_);
  std::vector<std::pair<std::string, Eigen::Isometry3d>> held;  // the block in the tcp frame after a grasp
  for (const auto& [r, azimuth, yaw] : { std::tuple{ 0.18, 10.0, 20.0 }, std::tuple{ 0.27, -20.0, -20.0 },
                                         std::tuple{ 0.32, 30.0, 30.0 } })
  {
    const Eigen::Vector3d centre = areaPoint(r, azimuth, block_size_ / 2.0);
    const std::vector<Grasp> grasps = generator_->grasps(centre, yaw * kDeg);
    ASSERT_FALSE(grasps.empty());
    const Eigen::Isometry3d block =
        Eigen::Translation3d(centre) * Eigen::AngleAxisd(yaw * kDeg, Eigen::Vector3d::UnitZ());
    held.emplace_back("held after a " + std::to_string(int(std::round(grasps[0].tilt / kDeg))) + " deg grasp",
                      grasps[0].grasp.inverse() * block);
  }
  Sweep sweep;
  for (const auto& [name, block_in_tcp] : held)
  {
    for (const auto& [r, azimuth] : areaGrid(0.01, 10.0))
    {
      const Eigen::Vector3d target = areaPoint(r, azimuth, 0.0);
      const std::vector<Place> places = generator_->places(target, block_in_tcp, size);
      ASSERT_EQ(places.size(), 2u) << name << " r " << r << " azimuth " << azimuth;
      for (const Place& place : places)
      {
        std::ostringstream where;
        where << name << ", r " << r << " azimuth " << azimuth << " beta " << place.jaw_angle / kDeg;
        // the block's centre over the target, its lowest corner place_clearance above the table
        const Eigen::Isometry3d block = place.place * block_in_tcp;
        double lowest = 1e9;
        for (int corner = 0; corner < 8; ++corner)
        {
          const Eigen::Vector3d offset((corner & 1 ? 0.5 : -0.5) * size.x(), (corner & 2 ? 0.5 : -0.5) * size.y(),
                                       (corner & 4 ? 0.5 : -0.5) * size.z());
          lowest = std::min(lowest, (block * offset).z());
        }
        EXPECT_NEAR(lowest, parameters.place_clearance, kExact) << where.str();
        EXPECT_LT((block.translation().head<2>() - target.head<2>()).norm(), kExact) << where.str();

        moveit::core::RobotState state = home();
        std::string error = solve(place.place, state, sweep);
        const moveit::core::RobotState at_place = state;
        for (const auto& [waypoint, pose] :
             { std::pair{ "pre-place", place.preplace }, std::pair{ "retreat", place.retreat } })
        {
          if (!error.empty())
          {
            break;
          }
          state = at_place;
          error = solve(pose, state, sweep);
          if (!error.empty())
          {
            error = std::string(waypoint) + ": " + error;
          }
        }
        if (!error.empty())
        {
          sweep.failures.push_back(where.str() + ": " + error);
        }
      }
    }
  }
  report("places", sweep);
  EXPECT_TRUE(sweep.failures.empty());
}

}  // namespace so101_mtc_task

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
