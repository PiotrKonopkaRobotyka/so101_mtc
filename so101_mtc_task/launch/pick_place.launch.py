"""The MTC pick and place of so101_mtc_task (ROB-52), against the running simulation.

Start the simulation first (so101_mtc_bringup's sim.launch.py, alias so101_mtc), then:

    ros2 launch so101_mtc_task pick_place.launch.py [execute:=false] [run_at_start:=false]

The node loads the robot as move_group does (MoveItConfigsBuilder), plans and executes one pick and place at start-up
to the target of config/pick_place.yaml, then serves ~/pick_place (so101_mtc_interfaces/srv/PickPlace) for more runs.
RViz's MTC panel shows the stages and the solutions.
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
    config = FindPackageShare("so101_mtc_task")
    switch = lambda name: ParameterValue(LaunchConfiguration(name), value_type=bool)  # noqa: E731

    return LaunchDescription([
        DeclareLaunchArgument("execute", default_value="true", description="Execute the plan; false: plan only"),
        DeclareLaunchArgument("run_at_start", default_value="true", description="One run at start-up"),
        Node(package="so101_mtc_task", executable="pick_place", output="screen",
             parameters=[moveit_config.to_dict(),
                         PathJoinSubstitution([config, "config", "grasp_poses.yaml"]),
                         PathJoinSubstitution([config, "config", "pick_place.yaml"]),
                         {"use_sim_time": True,
                          "pick_place.execute": switch("execute"),
                          "pick_place.run_at_start": switch("run_at_start")}]),
    ])
