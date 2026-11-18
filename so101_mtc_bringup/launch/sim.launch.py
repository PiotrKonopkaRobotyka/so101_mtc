"""One command for the simulated SO-101 of so101_mtc: MuJoCo with the scene, the controllers, MoveIt and RViz.

The order follows events, not sleeps:
1. robot_state_publisher (the URDF's TF, also the fixed camera's) and MuJoCo's ros2_control_node start; the
   joint_state_broadcaster's spawner starts with them and waits for the controller manager.
2. When that spawner exits, the arm's and the gripper's controllers are spawned.
3. When they are active, move_group (with MTC's capability) and RViz start.
A spawner that fails stops the launch; closing the MuJoCo window stops it too.

    ros2 launch so101_mtc_bringup sim.launch.py [headless:=false] [rviz:=true] [sim_speed_factor:=-1]
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, Shutdown
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def spawner(*controllers):
    return Node(package="controller_manager", executable="spawner", arguments=list(controllers), output="screen")


def after(action, then, what):
    """Start the actions `then` once `action` exits successfully; stop the launch if it fails."""
    def on_exit(event, context):
        return then if event.returncode == 0 else [Shutdown(reason=f"{what} failed (exit code {event.returncode})")]
    return RegisterEventHandler(OnProcessExit(target_action=action, on_exit=on_exit))


def generate_launch_description():
    description = FindPackageShare("so101_mtc_description")
    moveit = FindPackageShare("so101_mtc_moveit_config")
    robot_description = ParameterValue(Command([
        "xacro ", PathJoinSubstitution([description, "urdf", "so101_mtc.urdf.xacro"]),
        " headless:=", LaunchConfiguration("headless"),
        " sim_speed_factor:=", LaunchConfiguration("sim_speed_factor")]), value_type=str)

    joint_state_broadcaster = spawner("joint_state_broadcaster")
    arm_and_gripper = spawner("arm_controller", "gripper_controller")
    moveit_and_rviz = [
        IncludeLaunchDescription(PathJoinSubstitution([moveit, "launch", "move_group.launch.py"])),
        IncludeLaunchDescription(PathJoinSubstitution([moveit, "launch", "moveit_rviz.launch.py"]),
                                 condition=IfCondition(LaunchConfiguration("rviz"))),
    ]

    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="false", description="Run MuJoCo without its window"),
        DeclareLaunchArgument("rviz", default_value="true", description="Start RViz with MoveIt and MTC"),
        DeclareLaunchArgument("sim_speed_factor", default_value="-1",
                              description="Simulation speed vs real time, e.g. 5.0; below 0: real time"),
        Node(package="robot_state_publisher", executable="robot_state_publisher", output="log",
             parameters=[{"robot_description": robot_description, "use_sim_time": True}]),
        # MuJoCo behind the ros2_control hardware interface; the controllers and the plugins (camera, block pose)
        Node(package="mujoco_ros2_control", executable="ros2_control_node", output="both", emulate_tty=True,
             parameters=[{"use_sim_time": True},
                         PathJoinSubstitution([moveit, "config", "ros2_controllers.yaml"]),
                         PathJoinSubstitution([FindPackageShare("so101_mtc_bringup"), "config",
                                               "mujoco_plugins.yaml"])],
             on_exit=Shutdown(reason="MuJoCo stopped")),
        joint_state_broadcaster,
        after(joint_state_broadcaster, [arm_and_gripper], "spawning joint_state_broadcaster"),
        after(arm_and_gripper, moveit_and_rviz, "spawning arm_controller and gripper_controller"),
    ])
