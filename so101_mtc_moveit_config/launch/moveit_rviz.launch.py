"""RViz for MoveIt and MTC: the MotionPlanning display (plan and execute from the interactive marker), the Motion
Planning Tasks panel (an MTC task's stages and solutions) and the camera image. Start move_group first.

    ros2 launch so101_mtc_moveit_config moveit_rviz.launch.py [use_sim_time:=true]
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("so101_mtc", package_name="so101_mtc_moveit_config")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    rviz_config = PathJoinSubstitution([FindPackageShare("so101_mtc_moveit_config"), "rviz", "moveit.rviz"])

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true", description="Follow the simulator's /clock"),
        # RViz's MoveIt plugins load the robot model, the IK solver and the planners from these parameters
        Node(package="rviz2", executable="rviz2", output="log", arguments=["-d", rviz_config],
             parameters=[moveit_config.robot_description, moveit_config.robot_description_semantic,
                         moveit_config.robot_description_kinematics, moveit_config.planning_pipelines,
                         moveit_config.joint_limits, {"use_sim_time": use_sim_time}]),
    ])
