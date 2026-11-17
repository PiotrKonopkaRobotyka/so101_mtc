"""MoveIt's move_group for the SO-101 of so101_mtc, with MTC's capability to execute task solutions.

MoveItConfigsBuilder assembles the config: `.setup_assistant` names the URDF (so101_mtc_description) and the SRDF,
config/ holds kinematics.yaml, joint_limits.yaml and moveit_controllers.yaml, and the planner is OMPL with MoveIt's
default settings. Executing needs the controllers of config/ros2_controllers.yaml running.

    ros2 launch so101_mtc_moveit_config move_group.launch.py [use_sim_time:=true]
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("so101_mtc", package_name="so101_mtc_moveit_config")
        .planning_pipelines(pipelines=["ompl"])
        .planning_scene_monitor(publish_robot_description_semantic=True)
        .to_moveit_configs()
    )
    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true", description="Follow the simulator's /clock"),
        Node(package="moveit_ros_move_group", executable="move_group", output="screen",
             parameters=[moveit_config.to_dict(),
                         {"capabilities": "move_group/ExecuteTaskSolutionCapability", "use_sim_time": use_sim_time}]),
    ])
