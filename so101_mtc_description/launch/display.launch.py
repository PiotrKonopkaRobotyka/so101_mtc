"""Show the SO-101 of so101_mtc in RViz, with sliders for its joints (no simulator).

robot_state_publisher turns the expanded xacro and /joint_states into TF; joint_state_publisher_gui publishes
/joint_states from its sliders (joint_state_publisher, without a window, with gui:=false).

    ros2 launch so101_mtc_description display.launch.py [gui:=false]
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare("so101_mtc_description")
    robot_description = ParameterValue(
        Command(["xacro ", PathJoinSubstitution([share, "urdf", "so101_mtc.urdf.xacro"])]), value_type=str)
    gui = LaunchConfiguration("gui")

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true", description="Sliders for the joints"),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": robot_description}]),
        Node(package="joint_state_publisher_gui", executable="joint_state_publisher_gui", condition=IfCondition(gui)),
        Node(package="joint_state_publisher", executable="joint_state_publisher", condition=UnlessCondition(gui)),
        Node(package="rviz2", executable="rviz2",
             arguments=["-d", PathJoinSubstitution([share, "rviz", "display.rviz"])]),
    ])
